import tkinter
import tkinter.font as tkFont
from tkinter import ttk
import tkintermapview
import socket
import time
import threading
import math
import operator
import functools
import datetime

# --- 1. NMEA 파서 및 유틸리티 ---

def validate_checksum(sentence):
    """NMEA 0183 문장의 체크섬을 검사합니다."""
    try:
        sentence_body, checksum_str = sentence.strip().split('*')
        if sentence_body.startswith(('$', '!')):
            sentence_body = sentence_body[1:]
        nmeadata = bytes(sentence_body, 'utf-8')
        calculated_checksum = functools.reduce(operator.xor, nmeadata, 0)
        return int(checksum_str, 16) == calculated_checksum
    except Exception:
        return False

def safe_float(s, default=0.0):
    try: return float(s)
    except (ValueError, TypeError): return default

def safe_int(s, default=0):
    try: return int(s)
    except (ValueError, TypeError): return default

# --- 좌표 계산 유틸리티 (전역) ---
def deg_to_rad(deg):
    return deg * math.pi / 180.0
def rad_to_deg(rad):
    return rad * 180.0 / math.pi

def calculate_bearing(p1, p2): # p1=(lat, lon), p2=(lat, lon)
    """두 위도/경도 지점 간의 방위(Bearing)를 계산합니다."""
    lat1 = deg_to_rad(p1[0])
    lon1 = deg_to_rad(p1[1])
    lat2 = deg_to_rad(p2[0])
    lon2 = deg_to_rad(p2[1])
    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    bearing = math.atan2(y, x)
    return (rad_to_deg(bearing) + 360.0) % 360.0

def calculate_distance(p1, p2): # p1=(lat, lon), p2=(lat, lon)
    """두 위도/경도 지점 간의 실제 거리(NM)를 계산합니다."""
    R_NM = 3440.065
    lat1 = deg_to_rad(p1[0])
    lon1 = deg_to_rad(p1[1])
    lat2 = deg_to_rad(p2[0])
    lon2 = deg_to_rad(p2[1])
    dLat = lat2 - lat1
    dLon = lon2 - lon1
    a = math.sin(dLat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dLon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R_NM * c
    return distance
# --- 좌표 계산 유틸리티 종료 ---


# --- AIS 디코딩 유틸리티 ---
AIS_ASCII_MAP = {chr(k): v for v, k in enumerate(range(48, 89))}
AIS_ASCII_MAP.update({chr(k): v for v, k in enumerate(range(96, 123), 40)})

AIS_BIN_TO_STR_MAP = {
    0: '@', 1: 'A', 2: 'B', 3: 'C', 4: 'D', 5: 'E', 6: 'F', 7: 'G',
    8: 'H', 9: 'I', 10: 'J', 11: 'K', 12: 'L', 13: 'M', 14: 'N', 15: 'O',
    16: 'P', 17: 'Q', 18: 'R', 19: 'S', 20: 'T', 21: 'U', 22: 'V', 23: 'W',
    24: 'X', 25: 'Y', 26: 'Z', 27: '[', 28: '\\', 29: ']', 30: '^', 31: '_',
    32: ' ', 33: '!', 34: '"', 35: '#', 36: '$', 37: '%', 38: '&', 39: "'",
    40: '(', 41: ')', 42: '*', 43: '+', 44: ',', 45: '-', 46: '.', 47: '/',
    48: '0', 49: '1', 50: '2', 51: '3', 52: '4', 53: '5', 54: '6', 55: '7',
    56: '8', 57: '9', 58: ':', 59: ';', 60: '<', 61: '=', 62: '>', 63: '?'
}
SHIP_TYPE_MAP = {
    70: "Cargo Ship", 80: "Tanker", 60: "Passenger Ship",
    37: "Pleasure Craft", 0: "Not Available"
}
NAV_STATUS_MAP = {
    0: "Under way", 1: "At anchor", 5: "Moored",
    7: "R. in maneuver", 8: "Constr. by draught",
    15: "Not defined"
}

def _payload_to_bin(payload):
    binary_payload = ""
    for char in payload:
        if char not in AIS_ASCII_MAP:
            return None 
        binary_payload += format(AIS_ASCII_MAP[char], '06b')
    return binary_payload

def _signed_int_from_bin(bin_str):
    value = int(bin_str, 2)
    if bin_str.startswith('1'): 
        value -= (1 << len(bin_str))
    return value

def _bin_to_ais_str(bin_str):
    text = ""
    for i in range(0, len(bin_str), 6):
        chunk = bin_str[i:i+6]
        if len(chunk) < 6: break
        val = int(chunk, 2)
        text += AIS_BIN_TO_STR_MAP.get(val, '@')
    return text.strip('@').strip() 
# --- AIS 디코딩 유틸리티 종료 ---


# --- 2. NMEA TCP 서버 스레드 ---

class ClientHandler(threading.Thread):
    """개별 클라이언트 연결을 처리하는 스레드 (다중 AIS 수신)"""
    def __init__(self, client_socket, client_address, server_name, app_state):
        super().__init__(daemon=True)
        self.client_conn = client_socket
        self.client_address = client_address
        self.server_name = server_name # "T1", "T2"
        self.app_state = app_state
        self.running = True
        self.aivdm_cache = {}
        
        self.sentence_parsers = {
            "RMC": self.parse_rmc, "HDT": self.parse_hdt, "ROT": self.parse_rot,
            "DPT": self.parse_dpt, "DBT": self.parse_dbt, "VDM": self.parse_aivdm,
        }
        print(f"[{self.server_name}] 새 클라이언트 연결됨: {self.client_address}")

    def stop(self):
        self.running = False
        if self.client_conn:
            try: self.client_conn.close()
            except: pass
            
    def run(self):
        """클라이언트로부터 NMEA 데이터를 수신하고 파싱합니다."""
        buffer = ""
        while self.running:
            try:
                data = self.client_conn.recv(1024)
                if not data:
                    print(f"[{self.server_name}] 클라이언트 {self.client_address} 연결 끊김.")
                    break
                    
                buffer += data.decode('ascii', errors='ignore')
                
                while '\r\n' in buffer:
                    sentence, buffer = buffer.split('\r\n', 1)
                    if sentence.startswith(('$', '!')):
                        self.parse_nmea_sentence(sentence)
                        
            except (ConnectionResetError, BrokenPipeError):
                print(f"[{self.server_name}] 클라이언트 {self.client_address} 연결 강제 종료됨.")
                break
            except Exception as e:
                if self.running:
                    print(f"[{self.server_name}] 클라이언트 {self.client_address} 소켓 오류: {e}")
                break
        
        # 스레드 종료
        self.stop()
        print(f"[{self.server_name}] 핸들러 {self.client_address} 종료.")
        if self in self.app_state["active_clients"]:
            try:
                self.app_state["active_clients"].remove(self)
            except ValueError:
                pass 

    # --- 파서 헬퍼 함수 ---
    def parse_rmc(self, parts, data_store, lock):
        try:
            if parts[2] != 'A': 
                data_store["GPS_Status"].set("V (Void)")
                return
            data_store["GPS_Status"].set("A (Active)")
            utc_str = parts[1].split(".")[0]
            if len(utc_str) == 6:
                data_store["UTC"].set(f"{utc_str[0:2]}:{utc_str[2:4]}:{utc_str[4:6]} UTC")
            lat_val = safe_float(parts[3])
            lat_deg = int(lat_val / 100)
            lat_min = lat_val - (lat_deg * 100)
            lat = lat_deg + (lat_min / 60)
            if parts[4] == 'S': lat = -lat
            lon_val = safe_float(parts[5])
            lon_deg = int(lon_val / 100)
            lon_min = lon_val - (lon_deg * 100)
            lon = lon_deg + (lon_min / 60)
            if parts[6] == 'W': lon = -lon
            sog = safe_float(parts[7])
            cog = safe_float(parts[8])
            with lock:
                data_store["Lat"].set(f"{lat:.5f}° {parts[4]}")
                data_store["Lon"].set(f"{lon:.5f}° {parts[6]}")
                data_store["SOG"].set(f"{sog:.1f} kn")
                data_store["COG"].set(f"{cog:.1f}°")
                data_store["SPD"].set(f"{sog:.1f} kn")
                data_store["_raw_lat"] = lat
                data_store["_raw_lon"] = lon
                # [신규] CPA/TCPA 계산을 위해 본선 벡터 저장
                data_store["_os_vector"] = (lat, lon, sog, cog)
        except Exception as e:
            print(f"[{self.server_name}] RMC 파싱 오류: {e}")

    def parse_hdt(self, parts, data_store, lock):
        try:
            hdg = safe_float(parts[1])
            with lock:
                data_store["HDG"].set(f"{hdg:.1f}°")
        except Exception as e:
            print(f"[{self.server_name}] HDT 파싱 오류: {e}")

    def parse_rot(self, parts, data_store, lock):
        try:
            rot = safe_float(parts[1]) # deg/min
            with lock:
                data_store["ROT"].set(f"{rot:.1f} °/m")
        except Exception as e:
            print(f"[{self.server_name}] ROT 파싱 오류: {e}")

    def parse_dpt(self, parts, data_store, lock):
        try:
            depth = safe_float(parts[1])
            with lock:
                data_store["DPTH"].set(f"{depth:.1f} m")
        except Exception as e:
            print(f"[{self.server_name}] DPT 파싱 오류: {e}")

    def parse_dbt(self, parts, data_store, lock):
        try:
            depth_m = safe_float(parts[3])
            with lock:
                data_store["DPTH(SNDR)"].set(f"{depth_m:.1f} m")
        except Exception as e:
            print(f"[{self.server_name}] DBT 파싱 오류: {e}")

    def parse_aivdm(self, parts, data_store, lock):
        """!AIVDM 문장을 파싱합니다 (다중 패킷 지원)."""
        try:
            total_parts = int(parts[1])
            part_num = int(parts[2])
            msg_id = parts[3] 
            payload = parts[5]
            if total_parts == 1:
                self.aivdm_cache.pop(msg_id, None) 
                bin_payload = _payload_to_bin(payload)
                if bin_payload:
                    self._parse_aivdm_payload(bin_payload, data_store, lock)
                return
            if part_num == 1:
                self.aivdm_cache[msg_id] = payload
                return
            if part_num == total_parts:
                if msg_id in self.aivdm_cache:
                    full_payload = self.aivdm_cache[msg_id] + payload
                    bin_payload = _payload_to_bin(full_payload)
                    if bin_payload:
                        self._parse_aivdm_payload(bin_payload, data_store, lock)
                    del self.aivdm_cache[msg_id] 
                else:
                    print(f"[{self.server_name}] AIVDM 오류: {msg_id}의 Part 1이 캐시에 없습니다.")
        except Exception as e:
            print(f"[{self.server_name}] AIVDM 파싱 오류: {e}")

    def _parse_aivdm_payload(self, bin_payload, data_store, lock):
        """이진 페이로드를 메시지 타입에 따라 분배"""
        try:
            msg_type = int(bin_payload[0:6], 2)
            mmsi = int(bin_payload[8:38], 2)
            
            with lock: 
                target_data = data_store["AIS_Targets"].setdefault(mmsi, {"mmsi": mmsi})
                
                if msg_type in (1, 2, 3):
                    self._parse_aivdm_msg_1_2_3(bin_payload, target_data)
                elif msg_type == 5:
                    self._parse_aivdm_msg_5(bin_payload, target_data)
                
                target_data["timestamp"] = time.time() 
                
        except Exception as e:
            print(f"[{self.server_name}] AIVDM 페이로드 처리 오류: {e}")

    def _parse_aivdm_msg_1_2_3(self, bin_payload, target_data):
        """동적 데이터 (Msg 1, 2, 3) 파싱"""
        lon_raw = _signed_int_from_bin(bin_payload[61:89])
        target_data["lon"] = lon_raw / 600000.0
        lat_raw = _signed_int_from_bin(bin_payload[89:116])
        target_data["lat"] = lat_raw / 600000.0
        sog_raw = int(bin_payload[50:60], 2)
        target_data["sog"] = sog_raw / 10.0 if sog_raw != 1023 else None
        cog_raw = int(bin_payload[116:128], 2)
        target_data["cog"] = cog_raw / 10.0 if cog_raw != 3600 else None
        hdg_raw = int(bin_payload[128:137], 2)
        target_data["hdg"] = hdg_raw if hdg_raw != 511 else None
        nav_status_code = int(bin_payload[38:42], 2)
        target_data["nav_status_str"] = NAV_STATUS_MAP.get(nav_status_code, "Not defined")
        target_data["is_stopped"] = (target_data["sog"] is not None and target_data["sog"] < 0.1)

    def _parse_aivdm_msg_5(self, bin_payload, target_data):
        """[수정] 정적/항해 데이터 (Msg 5) 파싱 (모든 필드)"""
        target_data["call_sign"] = _bin_to_ais_str(bin_payload[70:112])
        target_data["ship_name"] = _bin_to_ais_str(bin_payload[112:232])
        ship_type_code = int(bin_payload[232:240], 2)
        target_data["ship_type_str"] = SHIP_TYPE_MAP.get(ship_type_code, "Unknown")
        
        # [신규] Dimensions
        dim_a = int(bin_payload[240:249], 2)
        dim_b = int(bin_payload[249:258], 2)
        dim_c = int(bin_payload[258:264], 2)
        dim_d = int(bin_payload[264:270], 2)
        target_data["length"] = dim_a + dim_b
        target_data["beam"] = dim_c + dim_d
        
        # [신규] ETA
        eta_mon = int(bin_payload[270:274], 2)
        eta_day = int(bin_payload[274:279], 2)
        eta_hr = int(bin_payload[279:284], 2)
        eta_min = int(bin_payload[284:290], 2)
        if eta_mon > 0 and eta_day > 0 and eta_hr < 24 and eta_min < 60:
             target_data["eta"] = f"{eta_day:02d}-{eta_mon:02d} {eta_hr:02d}:{eta_min:02d} UTC"
        else:
             target_data["eta"] = "N/A"
             
        # [신규] Draught
        draught_raw = int(bin_payload[290:298], 2)
        target_data["draught"] = draught_raw / 10.0 # 1/10 m
        
        # [신규] Destination
        target_data["destination"] = _bin_to_ais_str(bin_payload[298:418])
        
        print(f"[{self.server_name}] AIS Msg 5 수신: {target_data['ship_name']} (MMSI: {target_data['mmsi']})")

    def parse_nmea_sentence(self, sentence):
        """수신된 NMEA 문장을 '프로필'에 따라 파싱합니다."""
        if not validate_checksum(sentence):
            return
        try:
            sentence_body = sentence.split('*')[0][1:]
            parts = sentence_body.split(',')
            talker_id = parts[0][2:] 
            
            parser_func = self.sentence_parsers.get(talker_id)
            if not parser_func:
                return 
            
            profile = self.app_state["profile_config"]
            data_store = self.app_state["data_store"]
            lock = self.app_state["lock"]
            
            if talker_id in ("RMC", "GGA") and profile["EPFS1"] == self.server_name:
                parser_func(parts, data_store, lock)
            elif talker_id == "HDT" and profile["Heading"] == self.server_name:
                parser_func(parts, data_store, lock)
            elif talker_id == "ROT" and profile["ROT"] == self.server_name:
                parser_func(parts, data_store, lock)
            elif talker_id in ("DPT", "DBT") and profile["Sounder"] == self.server_name:
                parser_func(parts, data_store, lock)
            elif talker_id == "VDM":
                if profile["AIS 1"] == self.server_name:
                    parser_func(parts, data_store, lock)
                if profile["AIS 2"] == self.server_name:
                    parser_func(parts, data_store, lock)
            
        except Exception as e:
            print(f"[{self.server_name}] 파싱 중 오류: {e} (문장: {sentence})")

class NmeaServer(threading.Thread):
    """[수정] 이 스레드는 이제 포트를 열고 클라이언트 핸들러만 생성합니다."""
    def __init__(self, port, config_name, app_state):
        super().__init__(daemon=True)
        self.port = port
        self.config_name = config_name
        self.app_state = app_state     
        self.running = True
        self.sock = None
        
    def stop(self):
        self.running = False
        if self.sock:
            try: 
                socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect(('', self.port))
            except ConnectionRefusedError:
                pass 
            except Exception:
                pass 
        print(f"[{self.config_name}] 리스너 (Port {self.port})가 중지되었습니다.")

    def run(self):
        """TCP 리스너 메인 루프"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('', self.port))
            self.sock.listen(5) 
            print(f"[{self.config_name}] 리스너가 포트 {self.port}에서 시작됩니다.")
        except Exception as e:
            print(f"[{self.config_name}] 리스너 바인딩 실패: {e}")
            return

        while self.running:
            try:
                client_conn, addr = self.sock.accept()
                
                if not self.running: 
                    break
                    
                handler_thread = ClientHandler(client_conn, addr, self.config_name, self.app_state)
                handler_thread.start()
                
                self.app_state["active_clients"].append(handler_thread)
                
            except OSError: 
                if self.running:
                     print(f"[{self.config_name}] 리스너 소켓 오류.")
                break 
            except Exception as e:
                if self.running:
                    print(f"[{self.config_name}] 리스너 오류: {e}")
                break 
        
        print(f"[{self.config_name}] 리스너 루프 종료.")

# --- 3. 설정 팝업창 ---
class PortSettingsWindow(tkinter.Toplevel):
    def __init__(self, master, port_config):
        super().__init__(master)
        self.title("Port Settings (Sensors)")
        self.geometry("250x250") 
        self.transient(master) 
        self.port_config = port_config
        self.temp_vars = {} 
        frame = ttk.Frame(self, padding="10")
        frame.pack(expand=True, fill="both")
        ttk.Label(frame, text="TCP Port Settings", font=("Arial", 12, "bold")).grid(row=0, column=0, columnspan=2, pady=5)
        row_index = 1
        for name in ["T1", "T2", "T3", "T4", "T5"]:
            ttk.Label(frame, text=f"{name}:").grid(row=row_index, column=0, padx=5, pady=5, sticky="w")
            current_port = self.port_config.get(name, {"port": 0})["port"]
            var = tkinter.StringVar(value=str(current_port))
            entry = ttk.Entry(frame, textvariable=var, width=10)
            entry.grid(row=row_index, column=1, padx=5, pady=5)
            self.temp_vars[name] = var
            row_index += 1
        btn_frame = ttk.Frame(self)
        btn_frame.pack(fill="x", pady=5)
        ttk.Button(btn_frame, text="Apply & Restart Servers", command=self.apply).pack(side="right", padx=10)
        ttk.Button(btn_frame, text="Cancel", command=self.destroy).pack(side="right")
    def apply(self):
        try:
            for name, var in self.temp_vars.items():
                new_port = int(var.get())
                if name not in self.port_config:
                    self.port_config[name] = {} 
                self.port_config[name]["port"] = new_port
            print(f"[설정] 포트 설정이 변경되었습니다: {self.port_config}")
            self.master.restart_all_servers()
            self.destroy()
        except ValueError:
            print("[오류] 포트 번호는 숫자여야 합니다.")
        except Exception as e:
            print(f"[오류] 포트 적용 실패: {e}")

class ProfileSettingsWindow(tkinter.Toplevel):
    def __init__(self, master, profile_config, port_config):
        super().__init__(master)
        self.title("Profile 1 Settings")
        self.geometry("350x450")
        self.transient(master)
        self.profile_config = profile_config
        self.port_names = list(port_config.keys()) + ["0 (Off)"] 
        self.temp_vars = {}
        frame = ttk.Frame(self, padding="10")
        frame.pack(expand=True, fill="both")
        ttk.Label(frame, text="Sensor Profile 1", font=("Arial", 12, "bold")).grid(row=0, column=0, columnspan=2, pady=5)
        row_index = 1
        sensor_list = [
            "EPFS1", "EPFS2", "Primary EPFS2", "Heading", "Speed",
            "Time", "ROT", "Sounder", "Wind", "AIS 1", "AIS 2"
        ]
        for name in sensor_list:
            ttk.Label(frame, text=f"{name}:").grid(row=row_index, column=0, padx=5, pady=5, sticky="w")
            if name == "Primary EPFS2":
                var = tkinter.BooleanVar(value=self.profile_config.get(name, False))
                chk = ttk.Checkbutton(frame, variable=var)
                chk.grid(row=row_index, column=1, padx=5, pady=5, sticky="w")
            else:
                var = tkinter.StringVar(value=self.profile_config.get(name, "0 (Off)"))
                dropdown = ttk.OptionMenu(frame, var, var.get(), *self.port_names)
                dropdown.grid(row=row_index, column=1, padx=5, pady=5, sticky="ew")
            self.temp_vars[name] = var
            row_index += 1
        btn_frame = ttk.Frame(self)
        btn_frame.pack(fill="x", pady=5)
        ttk.Button(btn_frame, text="Apply", command=self.apply).pack(side="right", padx=10)
        ttk.Button(btn_frame, text="Cancel", command=self.destroy).pack(side="right")
    def apply(self):
        try:
            for name, var in self.temp_vars.items():
                self.profile_config[name] = var.get()
            print(f"[설정] 프로필 설정이 변경되었습니다: {self.profile_config}")
            self.destroy()
        except Exception as e:
            print(f"[오류] 프로필 적용 실패: {e}")
# --- 3. 설정 팝업창 종료 ---


# --- 4. AIS 타겟 팝업창 (CPA/TCPA 및 Msg 5 데이터 추가) ---
class AisPopup(tkinter.Toplevel):
    """[수정] 사진 2.jpg의 모든 항목을 표시하는 팝업창"""
    def __init__(self, master, mmsi, data_store, lock):
        super().__init__(master)
        self.mmsi = mmsi
        self.data_store = data_store
        self.data_lock = lock
        self.master_app = master # 메인 App 참조
        
        self.title(f"Target Info: {mmsi}")
        self.geometry("400x550") # [수정] 크기 증가
        self.transient(master)
        self.resizable(False, False)
        
        frame = ttk.Frame(self, padding="10")
        frame.pack(expand=True, fill="both")

        self.display_vars = {
            "MMSI": tkinter.StringVar(value=str(mmsi)),
            "ShipName": tkinter.StringVar(value="--"),
            "CallSign": tkinter.StringVar(value="--"),
            "ShipType": tkinter.StringVar(value="--"),
            "Length": tkinter.StringVar(value="-- m"),
            "Beam": tkinter.StringVar(value="-- m"),
            "Draught": tkinter.StringVar(value="-- m"),
            "Destination": tkinter.StringVar(value="--"),
            "ETA": tkinter.StringVar(value="--"),
            "Lat": tkinter.StringVar(value="--"),
            "Lon": tkinter.StringVar(value="--"),
            "SOG": tkinter.StringVar(value="--"),
            "COG": tkinter.StringVar(value="--"),
            "HDG": tkinter.StringVar(value="--"),
            "Status": tkinter.StringVar(value="--"),
            "BRG": tkinter.StringVar(value="--"), 
            "RNG": tkinter.StringVar(value="--"), 
            "CPA": tkinter.StringVar(value="--"), 
            "TCPA": tkinter.StringVar(value="--"), 
        }
        
        # [수정] 레이아웃 (사진 2.jpg와 유사하게)
        
        # --- 정적 데이터 (좌측) ---
        static_frame = ttk.LabelFrame(frame, text="Static Data")
        static_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        
        ttk.Label(static_frame, text="MMSI:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(static_frame, textvariable=self.display_vars["MMSI"], font=("Arial", 10, "bold")).grid(row=0, column=1, sticky="w", padx=5)
        ttk.Label(static_frame, text="Name:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(static_frame, textvariable=self.display_vars["ShipName"], font=("Arial", 10, "bold")).grid(row=1, column=1, sticky="w", padx=5)
        ttk.Label(static_frame, text="Call Sign:").grid(row=2, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(static_frame, textvariable=self.display_vars["CallSign"], font=("Arial", 10, "bold")).grid(row=2, column=1, sticky="w", padx=5)
        ttk.Label(static_frame, text="Type:").grid(row=3, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(static_frame, textvariable=self.display_vars["ShipType"]).grid(row=3, column=1, sticky="w", padx=5)
        ttk.Label(static_frame, text="Length:").grid(row=4, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(static_frame, textvariable=self.display_vars["Length"]).grid(row=4, column=1, sticky="w", padx=5)
        ttk.Label(static_frame, text="Beam:").grid(row=5, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(static_frame, textvariable=self.display_vars["Beam"]).grid(row=5, column=1, sticky="w", padx=5)

        # --- 항해 데이터 (좌측) ---
        voyage_frame = ttk.LabelFrame(frame, text="Voyage Data")
        voyage_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 5), pady=10)
        
        ttk.Label(voyage_frame, text="Draught:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(voyage_frame, textvariable=self.display_vars["Draught"]).grid(row=0, column=1, sticky="w", padx=5)
        ttk.Label(voyage_frame, text="Destination:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(voyage_frame, textvariable=self.display_vars["Destination"]).grid(row=1, column=1, sticky="w", padx=5)
        ttk.Label(voyage_frame, text="ETA:").grid(row=2, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(voyage_frame, textvariable=self.display_vars["ETA"]).grid(row=2, column=1, sticky="w", padx=5)

        # --- 동적 데이터 (우측) ---
        dynamic_frame = ttk.LabelFrame(frame, text="Dynamic Data")
        dynamic_frame.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=(5, 0))

        ttk.Label(dynamic_frame, text="Status:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(dynamic_frame, textvariable=self.display_vars["Status"], font=("Arial", 10, "bold")).grid(row=0, column=1, sticky="w", padx=5)
        
        ttk.Label(dynamic_frame, text="Lat:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(dynamic_frame, textvariable=self.display_vars["Lat"]).grid(row=1, column=1, sticky="w", padx=5)
        ttk.Label(dynamic_frame, text="Lon:").grid(row=2, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(dynamic_frame, textvariable=self.display_vars["Lon"]).grid(row=2, column=1, sticky="w", padx=5)
        
        ttk.Label(dynamic_frame, text="SOG:").grid(row=3, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(dynamic_frame, textvariable=self.display_vars["SOG"]).grid(row=3, column=1, sticky="w", padx=5)
        ttk.Label(dynamic_frame, text="COG:").grid(row=4, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(dynamic_frame, textvariable=self.display_vars["COG"]).grid(row=4, column=1, sticky="w", padx=5)
        ttk.Label(dynamic_frame, text="HDG:").grid(row=5, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(dynamic_frame, textvariable=self.display_vars["HDG"]).grid(row=5, column=1, sticky="w", padx=5)
        
        # --- 계산된 데이터 (우측) ---
        ttk.Separator(dynamic_frame, orient="horizontal").grid(row=6, column=0, columnspan=2, sticky="ew", pady=5)
        
        ttk.Label(dynamic_frame, text="BRG:").grid(row=7, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(dynamic_frame, textvariable=self.display_vars["BRG"]).grid(row=7, column=1, sticky="w", padx=5)
        ttk.Label(dynamic_frame, text="RNG:").grid(row=8, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(dynamic_frame, textvariable=self.display_vars["RNG"]).grid(row=8, column=1, sticky="w", padx=5)
        
        ttk.Separator(dynamic_frame, orient="horizontal").grid(row=9, column=0, columnspan=2, sticky="ew", pady=5)

        ttk.Label(dynamic_frame, text="CPA:").grid(row=10, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(dynamic_frame, textvariable=self.display_vars["CPA"]).grid(row=10, column=1, sticky="w", padx=5)
        ttk.Label(dynamic_frame, text="TCPA:").grid(row=11, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(dynamic_frame, textvariable=self.display_vars["TCPA"]).grid(row=11, column=1, sticky="w", padx=5)

        
        ttk.Button(frame, text="Exit", command=self.destroy).grid(row=2, column=0, columnspan=2, pady=10)
        
        self.update_popup_data() 

    def calculate_cpa_tcpa(self, os_data, tgt_data):
        """[신규] CPA/TCPA 계산 로직"""
        try:
            # 1. 본선(OS) 데이터
            os_lat = os_data["_raw_lat"]
            os_lon = os_data["_raw_lon"]
            os_sog = safe_float(os_data["SOG"].get().split(" ")[0])
            os_cog = safe_float(os_data["COG"].get().split("°")[0])
            
            # 2. 타겟(TGT) 데이터
            tgt_lat = tgt_data.get("lat")
            tgt_lon = tgt_data.get("lon")
            tgt_sog = tgt_data.get("sog")
            tgt_cog = tgt_data.get("cog")
            
            if None in [tgt_lat, tgt_lon, tgt_sog, tgt_cog]:
                return "--", "--" # 데이터 부족

            # 3. 상대 속도 벡터 계산 (단순화된 평면 가정)
            os_vx = os_sog * math.sin(deg_to_rad(os_cog))
            os_vy = os_sog * math.cos(deg_to_rad(os_cog))
            tgt_vx = tgt_sog * math.sin(deg_to_rad(tgt_cog))
            tgt_vy = tgt_sog * math.cos(deg_to_rad(tgt_cog))
            
            v_rel_x = tgt_vx - os_vx
            v_rel_y = tgt_vy - os_vy
            v_rel_speed_kn = math.hypot(v_rel_x, v_rel_y) 

            if v_rel_speed_kn < 0.1: # 거의 평행 또는 정지
                return f"{rng_nm:.2f} NM" if 'rng_nm' in locals() else "--", "Infinite"

            # 4. 상대 위치 벡터 계산 (BRG/RNG)
            rng_nm = calculate_distance((os_lat, os_lon), (tgt_lat, tgt_lon))
            brg_rad = deg_to_rad(calculate_bearing((os_lat, os_lon), (tgt_lat, tgt_lon)))

            p_rel_x = rng_nm * math.sin(brg_rad)
            p_rel_y = rng_nm * math.cos(brg_rad)
            
            # 5. CPA/TCPA 계산
            dot_product = (v_rel_x * p_rel_x) + (v_rel_y * p_rel_y)
            
            t_cpa_hours = -dot_product / (v_rel_speed_kn ** 2)
            
            if t_cpa_hours < 0: # 이미 CPA를 지남
                tcpa_min = 0.0
                cpa_nm = rng_nm # 현재 거리가 CPA
            else:
                tcpa_min = t_cpa_hours * 60.0
                cpa_x = p_rel_x + v_rel_x * t_cpa_hours
                cpa_y = p_rel_y + v_rel_y * t_cpa_hours
                cpa_nm = math.hypot(cpa_x, cpa_y)
            
            return f"{cpa_nm:.2f} NM", f"{tcpa_min:.1f} min"

        except Exception as e:
            print(f"CPA/TCPA 계산 오류: {e}")
            return "--", "--"

    def update_popup_data(self):
        """[수정] 팝업창의 모든 데이터를 1초마다 갱신"""
        try:
            with self.data_lock:
                target_data = self.data_store["AIS_Targets"].get(self.mmsi)
                os_data_copy = {
                    "_raw_lat": self.data_store["_raw_lat"],
                    "_raw_lon": self.data_store["_raw_lon"],
                    "SOG": self.data_store["SOG"], # StringVar
                    "COG": self.data_store["COG"]  # StringVar
                }
            
            if target_data:
                cpa_str, tcpa_str = self.calculate_cpa_tcpa(os_data_copy, target_data)
                
                os_pos = (os_data_copy["_raw_lat"], os_data_copy["_raw_lon"])
                target_pos = (target_data.get('lat', 0), target_data.get('lon', 0))
                if os_pos[0] != 35.10 and target_pos[0] != 0:
                    brg = calculate_bearing(os_pos, target_pos)
                    rng = calculate_distance(os_pos, target_pos)
                    self.display_vars["BRG"].set(f"{brg:.1f}°")
                    self.display_vars["RNG"].set(f"{rng:.2f} NM")
                
                self.display_vars["ShipName"].set(target_data.get("ship_name", "--"))
                self.display_vars["CallSign"].set(target_data.get("call_sign", "--"))
                self.display_vars["ShipType"].set(target_data.get("ship_type_str", "Unknown"))
                self.display_vars["Length"].set(f"{target_data.get('length', 0)} m")
                self.display_vars["Beam"].set(f"{target_data.get('beam', 0)} m")
                self.display_vars["Draught"].set(f"{target_data.get('draught', 0.0):.1f} m")
                self.display_vars["Destination"].set(target_data.get("destination", "--"))
                self.display_vars["ETA"].set(target_data.get("eta", "--"))
                
                self.display_vars["Lat"].set(f"{target_data.get('lat', 0.0):.5f}")
                self.display_vars["Lon"].set(f"{target_data.get('lon', 0.0):.5f}")
                self.display_vars["SOG"].set(f"{target_data.get('sog', 0.0):.1f} kn")
                self.display_vars["COG"].set(f"{target_data.get('cog', 0.0):.1f}°")
                self.display_vars["HDG"].set(f"{target_data.get('hdg', 0.0):.1f}°")
                self.display_vars["Status"].set(target_data.get("nav_status_str", "N/A"))
                
                self.display_vars["CPA"].set(cpa_str)
                self.display_vars["TCPA"].set(tcpa_str)
                
                self.after(1000, self.update_popup_data)
            else:
                print(f"타겟 {self.mmsi} 정보가 사라짐. 팝업을 닫습니다.")
                self.destroy()
        except Exception:
            pass 
# --- 4. AIS 타겟 팝업창 종료 ---


# --- 5. 메인 ECDIS 애플리케이션 (수정됨) ---
class App(tkinter.Tk):
    def __init__(self):
        super().__init__()
        self.title("Mini ECDIS Receiver")
        self.geometry("1200x800")
        
        self.data_store = {
            "UTC": tkinter.StringVar(value="--:--:-- UTC"),
            "Vector": tkinter.StringVar(value="6 min"),
            "GPS_Status": tkinter.StringVar(value="No Fix"),
            "Lat": tkinter.StringVar(value="--° --.----' N"),
            "Lon": tkinter.StringVar(value="---° --.----' E"),
            "COG": tkinter.StringVar(value="---.-°"),
            "SOG": tkinter.StringVar(value="-.- kn"),
            "HDG": tkinter.StringVar(value="---.-°"),
            "SPD": tkinter.StringVar(value="-.- kn"),
            "ROT": tkinter.StringVar(value="-.- °/m"),
            "DPTH": tkinter.StringVar(value="--.- m"),
            "DPTH(SNDR)": tkinter.StringVar(value="--.- m"),
            "_raw_lat": 35.10,
            "_raw_lon": 129.04,
            "_os_vector": (35.10, 129.04, 0.0, 0.0), # (lat, lon, sog, cog)
            "AIS_Targets": {}, 
        }
        
        self.port_config = {
            "T1": {"port": 10110}, "T2": {"port": 10120},
            "T3": {"port": 0}, "T4": {"port": 0}, "T5": {"port": 0},
        }
        
        self.profile_config = {
            "EPFS1": "T1", "EPFS2": "0 (Off)", "Primary EPFS2": False,
            "Heading": "T1", "Speed": "0 (Off)", "Time": "0 (Off)",
            "ROT": "T1", "Sounder": "T1", "Wind": "0 (Off)",
            "AIS 1": "T2", "AIS 2": "0 (Off)",
        }
        
        self.server_listeners = {} 
        self.active_clients = []   
        self.data_lock = threading.Lock()
        
        self.setup_gui_frames()
        self.setup_data_panel()
        self.setup_menu()
        
        self.map_widget.set_position(35.10, 129.04)
        self.map_widget.set_zoom(14)
        self.ship_marker = self.map_widget.set_marker(35.10, 129.04, text="SHIP")
        self.ais_markers = {} 
        
        self.map_mode = tkinter.StringVar(value="VIEW")
        self.active_ais_popup = None 
        
        mode_frame = tkinter.Frame(self.data_frame_container, relief="ridge", borderwidth=1)
        mode_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(mode_frame, text="Map Mode:").pack(side="left", padx=5)
        ttk.Radiobutton(mode_frame, text="View", variable=self.map_mode, value="VIEW").pack(side="left")
        ttk.Radiobutton(mode_frame, text="Target", variable=self.map_mode, value="TARGET").pack(side="left")
        
        self.map_widget.add_left_click_map_command(self.on_map_click)
        
        self.start_all_servers()
        self.update_gui_clock()
        self.update_map_markers() 
        
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def on_map_click(self, pos):
        """[수정] 지도 클릭 핸들러 (클릭 반경 수정)"""
        
        if self.map_mode.get() != "TARGET":
            return
            
        if self.active_ais_popup:
            try:
                self.active_ais_popup.destroy()
            except tkinter.TclError:
                pass 
            self.active_ais_popup = None
            
        click_lat, click_lon = pos
        
        closest_mmsi = None
        min_dist = float('inf')
        
        for mmsi, marker in self.ais_markers.items():
            dist = calculate_distance((click_lat, click_lon), marker.position)
            
            # [수정] 클릭 반경을 0.05NM (약 90m)로 늘림
            if dist < 0.05 and dist < min_dist:
                min_dist = dist
                closest_mmsi = mmsi
                
        if closest_mmsi:
            print(f"[GUI] 타겟 {closest_mmsi} 클릭됨.")
            self.active_ais_popup = AisPopup(self, closest_mmsi, self.data_store, self.data_lock)


    def setup_gui_frames(self):
        self.map_frame = tkinter.Frame(self)
        self.map_frame.pack(side="left", fill="both", expand=True)
        self.data_frame_container = tkinter.Frame(self, width=300)
        self.data_frame_container.pack(side="right", fill="y")
        self.data_frame_container.pack_propagate(False)
        self.map_widget = tkintermapview.TkinterMapView(self.map_frame, width=900, height=800, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)

    def setup_data_panel(self):
        label_font = tkFont.Font(family="Arial", size=10)
        value_font = tkFont.Font(family="Arial", size=10, weight="bold")
        time_frame = tkinter.Frame(self.data_frame_container, relief="ridge", borderwidth=1)
        time_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(time_frame, text="Time").pack(anchor="w", padx=5)
        ttk.Label(time_frame, textvariable=self.data_store["UTC"], font=value_font).pack(anchor="w", padx=5, pady=(0, 5))
        vec_frame = tkinter.Frame(self.data_frame_container, relief="ridge", borderwidth=1)
        vec_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(vec_frame, text="Vector").pack(anchor="w", padx=5)
        ttk.Label(vec_frame, textvariable=self.data_store["Vector"], font=value_font).pack(anchor="w", padx=5, pady=(0, 5))
        nav_frame = tkinter.Frame(self.data_frame_container, relief="ridge", borderwidth=1)
        nav_frame.pack(fill="x", padx=5, pady=5)
        nav_items = [
            ("Prim GPS1", self.data_store["GPS_Status"]),
            ("", self.data_store["Lat"]),
            ("", self.data_store["Lon"]),
            ("COG (EPFS)", self.data_store["COG"]),
            ("SOG (EPFS)", self.data_store["SOG"]),
            ("HDG (Gyro)", self.data_store["HDG"]),
            ("SPD (Log)", self.data_store["SPD"]),
            ("ROT (Gyro)", self.data_store["ROT"]),
            ("DPTH", self.data_store["DPTH"]),
            ("DPTH(SNDR)", self.data_store["DPTH(SNDR)"])
        ]
        for i, (label_text, var) in enumerate(nav_items):
            if label_text: 
                lbl = ttk.Label(nav_frame, text=label_text, font=label_font)
                lbl.grid(row=i, column=0, sticky="w", padx=5, pady=(5,0))
                val_lbl = ttk.Label(nav_frame, textvariable=var, font=value_font)
                val_lbl.grid(row=i, column=1, sticky="e", padx=5, pady=(5,0))
            else: 
                val_lbl = ttk.Label(nav_frame, textvariable=var, font=value_font)
                val_lbl.grid(row=i, column=0, columnspan=2, sticky="e", padx=5, pady=(0,0))
        nav_frame.grid_columnconfigure(1, weight=1) 

    def setup_menu(self):
        self.menubar = tkinter.Menu(self)
        self.config(menu=self.menubar)
        ship_menu = tkinter.Menu(self.menubar, tearoff=0)
        self.menubar.add_cascade(label="Ship", menu=ship_menu)
        sensors_menu = tkinter.Menu(ship_menu, tearoff=0)
        ship_menu.add_cascade(label="Sensors", menu=sensors_menu)
        sensors_menu.add_command(label="Port Settings...", command=self.open_port_settings)
        sensors_menu.add_command(label="Profile...", command=self.open_profile_settings)

    def open_port_settings(self):
        PortSettingsWindow(self, self.port_config)

    def open_profile_settings(self):
        ProfileSettingsWindow(self, self.profile_config, self.port_config)

    def start_all_servers(self):
        """[수정] 리스너 스레드와 클라이언트 핸들러 리스트를 관리합니다."""
        print("[메인] 모든 NMEA 리스너를 시작합니다...")
        self.stop_all_servers() 
        
        self.app_state = {
            "data_store": self.data_store,
            "profile_config": self.profile_config,
            "lock": self.data_lock,
            "active_clients": self.active_clients 
        }
        
        for name, config in self.port_config.items():
            port = config["port"]
            if port > 0: 
                listener_thread = NmeaServer(port, name, self.app_state)
                listener_thread.start()
                self.server_listeners[name] = listener_thread

    def stop_all_servers(self):
        """[수정] 모든 리스너와 활성 클라이언트 핸들러를 중지합니다."""
        
        for name, thread in self.server_listeners.items():
            thread.stop()
            thread.join(timeout=1.0) 
        self.server_listeners.clear()
        
        for client_thread in list(self.active_clients):
            client_thread.stop()
            client_thread.join(timeout=1.0)
        self.active_clients.clear()

    def restart_all_servers(self):
        print("[메인] NMEA 서버를 재시작합니다...")
        self.start_all_servers()

    def update_gui_clock(self):
        if self.data_store["GPS_Status"].get() == "No Fix":
            now = time.gmtime()
            self.data_store["UTC"].set(time.strftime("%H:%M:%S UTC", now))
        self.after(1000, self.update_gui_clock)

    def update_map_markers(self):
        """[수정] 1초마다 본선 및 AIS 타겟 마커를 모두 업데이트 (AIS 실시간 업데이트 버그 수정)"""
        
        gui_update_list = []
        mmsi_in_data = set()
        os_lat, os_lon = None, None
        
        try:
            with self.data_lock:
                os_lat = self.data_store["_raw_lat"]
                os_lon = self.data_store["_raw_lon"]
                
                mmsi_to_delete_from_store = []
                for mmsi, data in self.data_store["AIS_Targets"].items():
                    is_lost = (time.time() - data.get("timestamp", 0)) > 300 
                    is_stopped_and_old = data.get("is_stopped", False) and (time.time() - data.get("timestamp", 0)) > 900
                    
                    if is_lost or is_stopped_and_old:
                        mmsi_to_delete_from_store.append(mmsi)
                    elif "lat" in data and "lon" in data:
                        gui_update_list.append((mmsi, data.copy()))
                        mmsi_in_data.add(mmsi)
                
                for mmsi in mmsi_to_delete_from_store:
                    if mmsi in self.data_store["AIS_Targets"]:
                        del self.data_store["AIS_Targets"][mmsi]
            
            # --- 락(Lock) 없이 GUI 객체 업데이트 수행 ---
            
            if os_lat != 35.10 or os_lon != 129.04:
                self.ship_marker.set_position(os_lat, os_lon)

            for mmsi, data in gui_update_list:
                marker_text = data.get("ship_name", str(mmsi))
                
                if mmsi not in self.ais_markers:
                    print(f"[지도] 새 AIS 타겟 {mmsi} 발견. 지도에 추가.")
                    self.ais_markers[mmsi] = self.map_widget.set_marker(
                        data["lat"], data["lon"], 
                        text=marker_text,
                        marker_color_circle="green",
                        marker_color_outside="green"
                    )
                else:
                    self.ais_markers[mmsi].set_position(data["lat"], data["lon"])
                    if self.ais_markers[mmsi].text != marker_text:
                         self.ais_markers[mmsi].set_text(marker_text)

            mmsi_on_map = set(self.ais_markers.keys())
            mmsi_to_remove_from_map = mmsi_on_map - mmsi_in_data
            
            for mmsi in mmsi_to_remove_from_map:
                 print(f"[지도] AIS 타겟 {mmsi} 정리 (신호 유실).")
                 if mmsi in self.ais_markers: 
                    self.ais_markers[mmsi].delete()
                    del self.ais_markers[mmsi]

        except Exception as e:
            print(f"[지도 오류] 마커 업데이트 실패: {e}")
        
        self.after(1000, self.update_map_markers) 

    def on_closing(self):
        """프로그램 종료 시"""
        print("[메인] 프로그램을 종료합니다...")
        self.stop_all_servers()
        self.destroy()

# --- 5. 메인 프로그램 실행 ---
if __name__ == "__main__":
    app = App()
    app.mainloop()