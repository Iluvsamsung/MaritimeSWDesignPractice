import socket
import time
import threading
import math
import random
import datetime

# helpers 파일에서 모든 유틸리티 함수 임포트
from sim_helpers import *

# --- 4. 본선 시뮬레이션 엔진 (NmeaSimulator) ---
class NmeaSimulator:
    def __init__(self, waypoints, initial_speed, ip, port): 
        self.waypoints = waypoints
        self.ip = ip           
        self.port = port       
        self.target_idx = 1
        self.running = False
        self.is_holding = False 
        self.sock = None
        self.max_speed_kn = initial_speed      
        self.turn_speed_kn = max(2.0, initial_speed * 0.4) # [수정] 최소 2노트
        self.target_speed_kn = self.max_speed_kn   
        self.current_speed_kn = 0.0  
        self.acceleration_knps = 0.1 
        self.deceleration_knps = 0.2
        self.braking_knps = 0.3      # [수정] 제동 감속도 (0.5 -> 0.3)
        self.current_heading_deg = 0.0
        self.target_heading_deg = 0.0
        self.TURN_RATE_DEG_PER_SEC = 0.3 # 분당 18도
        
        if len(self.waypoints) > 1:
            self.current_heading_deg = calculate_bearing(self.waypoints[0], self.waypoints[1])
            self.target_heading_deg = self.current_heading_deg
        self.current_pos = waypoints[0]
        self.pos_lock = threading.Lock()

    def _connect_tcp(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            print(f"[본선] ECDIS 서버 연결 시도... ({self.ip}:{self.port})")
            self.sock.connect((self.ip, self.port))
            print("[본선] ECDIS 연결 성공.")
            return True
        except Exception as e:
            print(f"[본선] TCP 연결 실패: {e}")
            return False

    def _send_nmea(self, sentence_body):
        if not self.sock:
            return False
        checksum = calculate_checksum(sentence_body)
        final_sentence = f"{sentence_body}*{checksum}\r\n"
        try:
            self.sock.sendall(final_sentence.encode('utf-8'))
            return True
        except Exception as e:
            if self.running or self.is_holding:
                print(f"[본선] NMEA 전송 오류: {e}")
            self.running = False
            self.is_holding = False 
            return False

    def stop(self):
        print("[본선] 시뮬레이션 중지 신호 수신...")
        self.running = False    
        self.is_holding = False 
        if self.sock:
            try:
                print("[본선] 수동 중지. SOG=0.0 전송...")
                self._send_holding_packets() 
            except Exception as e:
                print(f"[본선] SOG=0.0 전송 실패: {e}")
            finally:
                self.sock.close()
                self.sock = None
        print("[본선] 연결 종료.")

    def get_current_position(self):
        with self.pos_lock:
            return self.current_pos 
            
    def _send_holding_packets(self):
        tm = time.gmtime()
        time_str = time.strftime("%H%M%S.00", tm)
        date_str = time.strftime("%d%m%y", tm)
        lat_str = format_lat_nmea(self.current_pos[0])
        lon_str = format_lon_nmea(self.current_pos[1])
        lat_dir = 'N' if self.current_pos[0] >= 0 else 'S'
        lon_dir = 'E' if self.current_pos[1] >= 0 else 'W'
        rmc_body = f"$GPRMC,{time_str},A,{lat_str},{lat_dir},{lon_str},{lon_dir},0.0,{self.current_heading_deg:.1f},{date_str},,"
        if not self._send_nmea(rmc_body): return False
        hdt_body = f"$HEHDT,{self.current_heading_deg:.1f},T"
        if not self._send_nmea(hdt_body): return False
        rot_body = f"$GPROT,0.0,A" 
        if not self._send_nmea(rot_body): return False
        dpt_body = "$SDDPT,21.5,,"
        if not self._send_nmea(dpt_body): return False
        dbt_body = "$SDDBT,,f,20.0,M,,F"
        if not self._send_nmea(dbt_body): return False
        mwv_body = "$WIMWV,030.0,R,8.5,N,A"
        if not self._send_nmea(mwv_body): return False
        return True

    def run_simulation(self):
        if not self._connect_tcp():
            return
        
        if len(self.waypoints) == 1:
            print("[본선] 항로점 1개 감지. 홀딩 모드로 시작합니다.")
            self.running = False
            self.is_holding = True
        else:
            self.running = True
            self.is_holding = False
        
        while self.running and self.target_idx < len(self.waypoints):
            delta_time = 1.0 
            target_pos = self.waypoints[self.target_idx]
            distance_to_target_nm = calculate_distance(self.current_pos, target_pos)
            self.target_heading_deg = calculate_bearing(self.current_pos, target_pos)
            heading_diff = (self.target_heading_deg - self.current_heading_deg + 180) % 360 - 180
            current_rot_deg_per_sec = 0.0 
            is_turning = abs(heading_diff) > self.TURN_RATE_DEG_PER_SEC
            is_final_wp = (self.target_idx == len(self.waypoints) - 1)
            
            # --- [핵심 수정] 동적 제동 거리 계산 ---
            if is_final_wp:
                # 1. 제동에 필요한 시간(초) = 현재속도 / 제동감속도
                time_to_stop_sec = self.current_speed_kn / self.braking_knps
                # 2. 제동에 필요한 평균 속도
                avg_speed_kn = self.current_speed_kn / 2.0
                # 3. 제동에 필요한 거리(NM) = 평균속도(NM/s) * 시간(s)
                required_braking_distance_nm = (avg_speed_kn / 3600.0) * time_to_stop_sec
                
                # [수정] 제동 거리에 진입하면 목표 속도를 0으로 설정
                # (도착 임계값 0.005NM를 추가하여 오차 보정)
                if distance_to_target_nm <= (required_braking_distance_nm + 0.005):
                    self.target_speed_kn = 0.0
                else:
                    self.target_speed_kn = self.max_speed_kn # 아직 멀었으면 최대 속도
            
            elif is_turning:
                self.target_speed_kn = self.turn_speed_kn 
            
            else:
                # [확인] 선회가 끝나면, 목표 속도는 다시 max_speed_kn로 설정됨
                self.target_speed_kn = self.max_speed_kn 
            # --- [수정 완료] ---

            # 2. 관성 적용
            if self.current_speed_kn < self.target_speed_kn:
                self.current_speed_kn += self.acceleration_knps * delta_time
                self.current_speed_kn = min(self.current_speed_kn, self.target_speed_kn)
            elif self.current_speed_kn > self.target_speed_kn:
                if self.target_speed_kn == 0.0:
                    self.current_speed_kn -= self.braking_knps * delta_time
                else:
                    self.current_speed_kn -= self.deceleration_knps * delta_time
                self.current_speed_kn = max(0.0, self.current_speed_kn)
            
            # 3. 방위 변경
            if is_turning:
                if heading_diff > 0:
                    self.current_heading_deg += self.TURN_RATE_DEG_PER_SEC
                    current_rot_deg_per_sec = self.TURN_RATE_DEG_PER_SEC
                elif heading_diff < 0:
                    self.current_heading_deg -= self.TURN_RATE_DEG_PER_SEC
                    current_rot_deg_per_sec = -self.TURN_RATE_DEG_PER_SEC
            else:
                self.current_heading_deg = self.target_heading_deg
                current_rot_deg_per_sec = heading_diff
            self.current_heading_deg = self.current_heading_deg % 360.0
            
            # 4. 위치 계산
            dist_per_sec_nm = self.current_speed_kn / 3600.0
            new_pos = calculate_destination(self.current_pos, self.current_heading_deg, dist_per_sec_nm)
            with self.pos_lock: self.current_pos = new_pos
            
            # 5. 도착 판정
            arrival_threshold_nm = max(0.005, (self.max_speed_kn / 3600.0) * 2.0) 
            
            if distance_to_target_nm < arrival_threshold_nm:
                if is_final_wp and self.current_speed_kn < 0.1:
                    print("[본선] 최종 목적지 도달 및 정지. 홀딩 모드 시작.")
                    self.running = False 
                    self.is_holding = True 
                    break 
                elif not is_final_wp:
                    print(f"[본선] 항로점 {self.target_idx} 도달: {target_pos}")
                    self.target_idx += 1
            
            # 6. NMEA 전송
            tm = time.gmtime()
            time_str = time.strftime("%H%M%S.00", tm)
            date_str = time.strftime("%d%m%y", tm)
            lat_str = format_lat_nmea(self.current_pos[0])
            lon_str = format_lon_nmea(self.current_pos[1])
            lat_dir = 'N' if self.current_pos[0] >= 0 else 'S'
            lon_dir = 'E' if self.current_pos[1] >= 0 else 'W'
            rmc_body = f"$GPRMC,{time_str},A,{lat_str},{lat_dir},{lon_str},{lon_dir},{self.current_speed_kn:.1f},{self.current_heading_deg:.1f},{date_str},,"
            if not self._send_nmea(rmc_body): break
            hdt_body = f"$HEHDT,{self.current_heading_deg:.1f},T"
            if not self._send_nmea(hdt_body): break
            rot_deg_per_min = current_rot_deg_per_sec * 60.0 
            rot_body = f"$GPROT,{rot_deg_per_min:.1f},A"
            if not self._send_nmea(rot_body): break
            dpt_body = "$SDDPT,21.5,,"
            if not self._send_nmea(dpt_body): break
            dbt_body = "$SDDBT,,f,20.0,M,,F"
            if not self._send_nmea(dbt_body): break
            mwv_body = "$WIMWV,030.0,R,8.5,N,A"
            if not self._send_nmea(mwv_body): break
            if time.gmtime().tm_sec % 6 == 0:
                print(f"[본선] 전송 (현재 속도: {self.current_speed_kn:.1f}Kn, 목표 속도: {self.target_speed_kn:.1f}Kn)")
            time.sleep(1) 
        
        self.current_speed_kn = 0.0 
        
        while self.is_holding:
            if not self._send_holding_packets():
                break 
            if time.gmtime().tm_sec % 6 == 0:
                print(f"[본선] 홀딩 모드. SOG=0.0 패킷 전송 중...")
            time.sleep(1) 

        if self.sock:
             self.sock.close()
             self.sock = None
        print("[본선] 스레드 종료.")
        
# --- 6. AIS 시뮬레이션 엔진 (AisSimulator) ---
class AisSimulator:
    def __init__(self, waypoints, mmsi, ip, port, 
                 ship_name, ship_type, call_sign, 
                 length, beam, draught, destination, eta, 
                 nav_status, speed): 
        
        self.waypoints = waypoints
        self.mmsi = mmsi 
        self.ip = ip           
        self.port = port
        
        # [신규] 정적 데이터
        self.ship_name = ship_name
        self.ship_type = ship_type
        self.call_sign = call_sign
        self.draught = draught
        self.destination = destination
        self.eta = eta
        self.dim_a = math.ceil(length / 2)
        self.dim_b = length - self.dim_a
        self.dim_c = math.ceil(beam / 2)
        self.dim_d = beam - self.dim_c
        
        self.target_idx = 1
        self.running = False
        self.is_holding = False 
        self.sock = None
        
        self.nav_status_code = nav_status
        self.max_speed_kn = speed
        
        self.turn_speed_kn = max(2.0, self.max_speed_kn * 0.4) # [수정] 5.0 -> 2.0
        self.target_speed_kn = self.max_speed_kn   
        self.current_speed_kn = 0.0  
        self.acceleration_knps = 0.1 
        self.deceleration_knps = 0.2
        self.braking_knps = 0.3      # [수정] 0.5 -> 0.3
        self.current_heading_deg = 0.0
        self.target_heading_deg = 0.0
        self.TURN_RATE_DEG_PER_SEC = 0.3 # [수정] 2.0 -> 0.3
        
        if len(self.waypoints) > 1:
            self.current_heading_deg = calculate_bearing(self.waypoints[0], self.waypoints[1])
            self.target_heading_deg = self.current_heading_deg
        
        self.current_pos = waypoints[0]
        self.pos_lock = threading.Lock()

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
            
    def run_simulation(self):
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
        
        eta_dt = self.eta
        if not eta_dt and len(self.waypoints) > 1:
            eta_dt = self.calculate_eta(self.waypoints, self.max_speed_kn)

        payload_part_1, payload_part_2 = pack_aivdm_message_5(
            self.mmsi, 
            self.call_sign, self.ship_name, self.ship_type,
            self.dim_a, self.dim_b, self.dim_c, self.dim_d,
            eta_dt, self.draught, self.destination
        )
        msg_5_group_id = str(random.randint(0, 9)) 
        
        while self.running and self.target_idx < len(self.waypoints):
            delta_time = 1.0 
            target_pos = self.waypoints[self.target_idx]
            distance_to_target_nm = calculate_distance(self.current_pos, target_pos)
            self.target_heading_deg = calculate_bearing(self.current_pos, target_pos)
            heading_diff = (self.target_heading_deg - self.current_heading_deg + 180) % 360 - 180
            is_turning = abs(heading_diff) > self.TURN_RATE_DEG_PER_SEC
            is_final_wp = (self.target_idx == len(self.waypoints) - 1)
            
            # --- [핵심 수정] 동적 제동 거리 계산 ---
            if is_final_wp:
                time_to_stop_sec = self.current_speed_kn / self.braking_knps
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
            # --- [수정 완료] ---

            # 2. 관성 적용
            if self.current_speed_kn < self.target_speed_kn:
                self.current_speed_kn += self.acceleration_knps * delta_time
                self.current_speed_kn = min(self.current_speed_kn, self.target_speed_kn)
            elif self.current_speed_kn > self.target_speed_kn:
                if self.target_speed_kn == 0.0:
                    self.current_speed_kn -= self.braking_knps * delta_time
                else:
                    self.current_speed_kn -= self.deceleration_knps * delta_time
                self.current_speed_kn = max(0.0, self.current_speed_kn)
            
            # 3. 방위 변경
            if is_turning:
                if heading_diff > 0: self.current_heading_deg += self.TURN_RATE_DEG_PER_SEC
                elif heading_diff < 0: self.current_heading_deg -= self.TURN_RATE_DEG_PER_SEC
            else: self.current_heading_deg = self.target_heading_deg
            self.current_heading_deg = self.current_heading_deg % 360.0
            
            # 4. 위치 계산
            dist_per_sec_nm = self.current_speed_kn / 3600.0
            new_pos = calculate_destination(self.current_pos, self.current_heading_deg, dist_per_sec_nm)
            with self.pos_lock: self.current_pos = new_pos
            
            # 5. 도착 판정
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
            
            # 6. AIVDM 전송 (Msg 1 + Msg 5)
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