import socket
import time
import threading
import math
import random
import datetime

# helpers 파일에서 모든 유틸리티 함수 임포트
from ais_helpers import *

# --- AIS 시뮬레이션 엔진 ---
class AisSimulator(threading.Thread):
    """
    AIS 타겟 1척을 시뮬레이션하는 스레드.
    관성, 항로점, NMEA 메시지 생성을 모두 담당.
    """
    def __init__(self, target_data, ip, port): 
        super().__init__(daemon=True)
        
        # [수정] target_data 딕셔너리에서 모든 정보 추출
        self.waypoints = target_data["waypoints"]
        static_data = target_data["static_data"]
        
        self.mmsi = static_data["mmsi"] 
        self.ip = ip           
        self.port = port
        self.static_data = static_data
        
        self.target_idx = 1
        self.running = False      # 스레드 메인 루프 (항해)
        self.is_holding = False   # 홀딩 루프 (정지)
        self.sock = None
        
        self.nav_status_code = self.static_data["nav_status"] 
        self.max_speed_kn = self.static_data["speed"]
        
        self.turn_speed_kn = max(2.0, self.max_speed_kn * 0.4) 
        self.target_speed_kn = self.max_speed_kn   
        self.current_speed_kn = 0.0  
        self.acceleration_knps = 0.1 
        self.deceleration_knps = 0.2
        self.braking_knps = 0.3      
        self.current_heading_deg = 0.0
        self.target_heading_deg = 0.0
        self.TURN_RATE_DEG_PER_SEC = 0.3 
        
        if len(self.waypoints) > 1:
            self.current_heading_deg = calculate_bearing(self.waypoints[0], self.waypoints[1])
            self.target_heading_deg = self.current_heading_deg
        
        self.current_pos = self.waypoints[0]
        self.pos_lock = threading.Lock() # GUI와 위치 공유를 위한 잠금
        
        # [신규] GUI가 이 객체를 직접 참조
        self.map_marker = None
        self.listbox_ref = None

    def calculate_eta(self, waypoints, speed):
        """항로 총 거리를 계산하여 ETA (datetime 객체)를 반환"""
        if speed <= 0:
            return None 
            
        total_distance_nm = 0.0
        for i in range(len(waypoints) - 1):
            total_distance_nm += calculate_distance(waypoints[i], waypoints[i+1])
            
        if total_distance_nm == 0:
            return None
            
        hours_to_arrival = total_distance_nm / speed 
        
        try:
            eta_datetime = datetime.datetime.utcnow() + datetime.timedelta(hours=hours_to_arrival)
        except OverflowError: 
            return None
        
        print(f"[AIS {self.mmsi}] 총 거리: {total_distance_nm:.2f} NM, 예상 시간: {hours_to_arrival:.2f} 시간. ETA: {eta_datetime}")
        return eta_datetime

    def _connect_tcp(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            print(f"[AIS {self.mmsi}] ECDIS 서버 연결 시도... ({self.ip}:{self.port})")
            self.sock.connect((self.ip, self.port))
            print(f"[AIS {self.mmsi}] ECDIS 연결 성공.")
            return True
        except Exception as e:
            print(f"[AIS {self.mmsi}] TCP 연결 실패: {e}")
            return False

    def _send_aivdm_packet(self, payload_str, total_parts=1, part_num=1, msg_id=""):
        """AIVDM 문장을 전송 (다중 패킷 지원)"""
        if not self.sock:
            return False
        try:
            sentence_body = f"AIVDM,{total_parts},{part_num},{msg_id},A,{payload_str},0"
            checksum = calculate_checksum(sentence_body)
            final_sentence = f"!{sentence_body}*{checksum}\r\n"
            self.sock.sendall(final_sentence.encode('utf-8'))
            return True
        except Exception as e:
            if self.running or self.is_holding: 
                print(f"[AIS {self.mmsi}] AIVDM 전송 오류: {e}")
            self.running = False
            self.is_holding = False 
            return False

    def stop(self):
        """GUI에서 호출 시, 모든 루프를 중지하고 소켓을 닫음"""
        print(f"[AIS {self.mmsi}] 시뮬레이션 중지 신호 수신...")
        self.running = False
        self.is_holding = False
        if self.sock:
            try:
                print(f"[AIS {self.mmsi}] 수동 중지. SOG=0.0 / Moored(5) 전송...")
                payload = pack_aivdm_message_1(
                    self.mmsi, self.current_pos[0], self.current_pos[1],
                    0.0, self.current_heading_deg, self.current_heading_deg,
                    nav_status=5 # 5 = Moored
                )
                self._send_aivdm_packet(payload)
            except Exception as e:
                print(f"[AIS {self.mmsi}] SOG=0.0 전송 실패: {e}")
            finally:
                self.sock.close()
                self.sock = None
        print(f"[AIS {self.mmsi}] 연결 종료.")

    def get_current_position(self):
        with self.pos_lock:
            return self.current_pos 
            
    def run(self): # threading.Thread의 run 메서드
        if not self._connect_tcp():
            return
            
        if len(self.waypoints) == 1:
            print(f"[AIS {self.mmsi}] 항로점 1개 감지. 홀딩 모드(SOG=0, Status={self.nav_status_code})로 시작합니다.")
            self.running = False
            self.is_holding = True
            self.holding_nav_status = self.nav_status_code
        else:
            self.running = True
            self.is_holding = False
            self.holding_nav_status = 5 
            
        last_pos_send_time = 0   
        last_static_send_time = 0 
        
        s_data = self.static_data
        
        eta_dt = s_data.get("eta_datetime")
        if not eta_dt and len(self.waypoints) > 1:
            eta_dt = self.calculate_eta(self.waypoints, self.max_speed_kn)

        payload_part_1, payload_part_2 = pack_aivdm_message_5(
            self.mmsi, 
            s_data["call_sign"], s_data["ship_name"], s_data["ship_type"],
            s_data["dim_a"], s_data["dim_b"], s_data["dim_c"], s_data["dim_d"],
            eta_dt, s_data["draught"], s_data["destination"]
        )
        msg_5_group_id = str(random.randint(0, 9)) 
        
        while self.running and self.target_idx < len(self.waypoints):
            delta_time = 1.0 
            target_pos = self.waypoints[self.target_idx]
            distance_to_target_nm = calculate_distance(self.current_pos, target_pos)
            
            if distance_to_target_nm > 0.005:
                self.target_heading_deg = calculate_bearing(self.current_pos, target_pos)
                
            heading_diff = (self.target_heading_deg - self.current_heading_deg + 180) % 360 - 180
            is_turning = abs(heading_diff) > self.TURN_RATE_DEG_PER_SEC
            is_final_wp = (self.target_idx == len(self.waypoints) - 1)
            
            if is_final_wp:
                time_to_stop_sec = self.current_speed_kn / self.braking_knps if self.braking_knps > 0 else 0
                avg_speed_kn = self.current_speed_kn / 2.0
                required_braking_distance_nm = (avg_speed_kn / 3600.0) * time_to_stop_sec
                if distance_to_target_nm <= (required_braking_distance_nm + 0.005):
                    self.target_speed_kn = 0.0
                else:
                    self.target_speed_kn = self.max_speed_kn 
            elif is_turning:
                self.target_speed_kn = self.turn_speed_kn 
            else:
                self.target_speed_kn = self.max_speed_kn 
                
            if self.current_speed_kn < self.target_speed_kn:
                self.current_speed_kn += self.acceleration_knps * delta_time
                self.current_speed_kn = min(self.current_speed_kn, self.target_speed_kn)
            elif self.current_speed_kn > self.target_speed_kn:
                if self.target_speed_kn == 0.0:
                    self.current_speed_kn -= self.braking_knps * delta_time
                else:
                    self.current_speed_kn -= self.deceleration_knps * delta_time
                self.current_speed_kn = max(0.0, self.current_speed_kn)
                
            if is_turning:
                if heading_diff > 0: self.current_heading_deg += self.TURN_RATE_DEG_PER_SEC
                elif heading_diff < 0: self.current_heading_deg -= self.TURN_RATE_DEG_PER_SEC
            else: self.current_heading_deg = self.target_heading_deg
            self.current_heading_deg = self.current_heading_deg % 360.0
            
            dist_per_sec_nm = self.current_speed_kn / 3600.0
            if dist_per_sec_nm > 0:
                new_pos = calculate_destination(self.current_pos, self.current_heading_deg, dist_per_sec_nm)
                with self.pos_lock: self.current_pos = new_pos
            
            arrival_threshold_nm = max(0.005, (self.max_speed_kn / 3600.0) * 2.0)
            if distance_to_target_nm < arrival_threshold_nm:
                if is_final_wp and self.current_speed_kn < 0.1:
                    print(f"[AIS {self.mmsi}] 최종 목적지 도달 및 정지. 홀딩 모드 시작.")
                    self.running = False 
                    self.is_holding = True 
                    break
                elif not is_final_wp:
                    print(f"[AIS {self.mmsi}] 항로점 {self.target_idx} 도달: {target_pos}")
                    self.target_idx += 1
            
            current_time = time.time()
            
            if current_time - last_pos_send_time >= 6.0: 
                payload_1 = pack_aivdm_message_1(
                    self.mmsi, self.current_pos[0], self.current_pos[1],
                    self.current_speed_kn, self.current_heading_deg, self.current_heading_deg,
                    self.nav_status_code
                )
                if not self._send_aivdm_packet(payload_1): 
                    break
                print(f"[AIS {self.mmsi}] 전송 (Msg 1: 속도 {self.current_speed_kn:.1f}Kn)")
                last_pos_send_time = current_time
            
            if current_time - last_static_send_time >= 30.0:
                print(f"[AIS {self.mmsi}] 전송 (Msg 5: 정적 데이터 Part 1/2)")
                if not self._send_aivdm_packet(payload_part_1, 2, 1, msg_5_group_id): break
                time.sleep(0.1) 
                if not self._send_aivdm_packet(payload_part_2, 2, 2, msg_5_group_id): break
                last_static_send_time = current_time
                
            time.sleep(1) 
        
        self.current_speed_kn = 0.0 
        
        while self.is_holding:
            current_time = time.time()
            if current_time - last_pos_send_time >= 6.0:
                payload_1 = pack_aivdm_message_1(
                    self.mmsi, self.current_pos[0], self.current_pos[1],
                    0.0, self.current_heading_deg, self.current_heading_deg,
                    self.holding_nav_status 
                )
                if not self._send_aivdm_packet(payload_1): 
                    break
                print(f"[AIS {self.mmsi}] 홀딩 모드. SOG=0.0 (Msg 1, Status={self.holding_nav_status}) 전송 중...")
                last_pos_send_time = current_time

            if current_time - last_static_send_time >= 30.0:
                print(f"[AIS {self.mmsi}] 홀딩 모드. (Msg 5: 정적 데이터 Part 1/2) 전송")
                if not self._send_aivdm_packet(payload_part_1, 2, 1, msg_5_group_id): break
                time.sleep(0.1) 
                if not self._send_aivdm_packet(payload_part_2, 2, 2, msg_5_group_id): break
                last_static_send_time = current_time

            time.sleep(1) 

        if self.sock:
             self.sock.close()
             self.sock = None
        print(f"[AIS {self.mmsi}] 스레드 종료.")
# --- 5. AIS 시뮬레이터 종료 ---