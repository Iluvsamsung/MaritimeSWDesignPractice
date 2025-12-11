import tkinter
import tkinter.ttk as ttk
import tkintermapview
import threading
import math
import datetime

# 분리된 파일에서 클래스와 함수 임포트
from sim_helpers import *
from sim_engine import NmeaSimulator
# (AisDetailPopup은 이 파일에서 필요 없음)

class App(tkinter.Tk):
    def __init__(self):
        super().__init__()
        self.title("NMEA 0183 본선(OS) 시뮬레이터")
        self.geometry("1000x800") 
        
        self.os_data = {
            "waypoints": [], "markers": [], "path_obj": None,
            "sim_instance": None, "sim_thread": None, "ship_marker": None,
            # [수정] DoubleVar -> StringVar (Entry 위젯과 호환)
            "speed_var": tkinter.StringVar(value="12.0") 
        }
        self.pending_waypoints = []
        self.pending_markers = []
        self.pending_path_obj = None

        self.map_frame = tkinter.Frame(self)
        self.control_frame = ttk.Frame(self, padding=10) 
        self.map_frame.pack(side="left", fill="both", expand=True)
        self.control_frame.pack(side="right", fill="y")

        self.map_widget = tkintermapview.TkinterMapView(self.map_frame, width=800, height=800, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_position(35.10, 129.04)
        self.map_widget.set_zoom(12)
        self.map_widget.add_left_click_map_command(self.on_map_click)

        # --- 제어판 GUI (본선 전용) ---
        
        # 1. 전송 설정
        lf_dest = ttk.LabelFrame(self.control_frame, text="1. 전송 설정 (Destination)")
        lf_dest.pack(fill="x", pady=(0, 10))
        
        self.os_ip_var = tkinter.StringVar(value="192.168.10.3")
        self.os_port_var = tkinter.StringVar(value="10110")
        
        self.dest_entries = [] 
        
        ttk.Label(lf_dest, text="본선 IP:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        e1 = ttk.Entry(lf_dest, textvariable=self.os_ip_var, width=15)
        e1.grid(row=0, column=1, padx=5, pady=2)
        
        ttk.Label(lf_dest, text="본선 Port:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        e2 = ttk.Entry(lf_dest, textvariable=self.os_port_var, width=8)
        e2.grid(row=1, column=1, padx=5, pady=2, sticky="w")
        
        self.dest_entries = [e1, e2]
        
        # 2. 항로 설정
        lf_mode = ttk.LabelFrame(self.control_frame, text="2. 항로 설정 (Planning)")
        lf_mode.pack(fill="x", pady=(0, 10))
        
        # --- [핵심 수정] 속도 슬라이더 -> 숫자 입력(Entry)으로 변경 ---
        speed_frame = ttk.Frame(lf_mode)
        speed_frame.pack(fill="x", anchor="w")
        
        ttk.Label(speed_frame, text="본선 속도(Knot):").pack(side="left", padx=5)
        self.speed_entry = ttk.Entry(speed_frame, textvariable=self.os_data["speed_var"], width=8)
        self.speed_entry.pack(side="left", padx=5)
        
        self.dest_entries.append(self.speed_entry) 
        # --- [수정 완료] ---
        
        # 3. 실행 제어
        lf_control = ttk.LabelFrame(self.control_frame, text="3. 실행 제어")
        lf_control.pack(fill="x", pady=(0, 10))
        
        self.btn_start = ttk.Button(lf_control, text="시뮬레이션 시작", command=self.start_simulation)
        self.btn_start.pack(fill="x", pady=5)
        
        self.btn_stop = ttk.Button(lf_control, text="시뮬레이션 중지", command=self.stop_simulation)
        self.btn_stop.pack(fill="x", pady=5)

        # 4. 초기화
        lf_clear = ttk.LabelFrame(self.control_frame, text="4. 초기화")
        lf_clear.pack(fill="x")
        
        self.btn_clear = ttk.Button(lf_clear, text="항로 초기화", command=self.clear_route)
        self.btn_clear.pack(fill="x", pady=2)
        self.dest_entries.append(self.btn_clear) 
        
        self.info_label = ttk.Label(self.control_frame, text="지도 클릭: 항로점 추가", wraplength=180, relief="sunken")
        self.info_label.pack(side="bottom", pady=10, fill="x")
        
        self.set_ui_state(running=False) # UI 상태 초기화
        
    # (on_speed_scale_change 함수 제거됨)

    def set_ui_state(self, running):
        state = "disabled" if running else "normal"
        
        for widget in self.dest_entries:
            try:
                if "state" in widget.configure():
                    widget.config(state=state)
            except Exception:
                pass 
        
        self.btn_start.config(state="disabled" if running else "normal")
        self.btn_stop.config(state="normal" if running else "disabled")
        
        if running:
             self.info_label.config(text="시뮬레이션 실행 중...\n'중지'를 눌러야 설정 변경 가능")
        else:
             self.info_label.config(text="지도 클릭: 항로점 추가")
             
    def on_map_click(self, pos):
        if self.os_data.get("sim_instance"): 
             print("오류: 시뮬레이션 실행 중에는 항로점을 추가할 수 없습니다.")
             return
        lat, lon = pos
        
        self.pending_waypoints.append((lat, lon))
        wp_num = len(self.pending_waypoints)
        text = f"OS WP {wp_num}"
        print(f"[본선] 임시 항로점 추가: ({lat:.6f}, {lon:.6f})")
        marker = self.map_widget.set_marker(lat, lon, text=text, text_color="blue")
        self.pending_markers.append(marker)
        
        if len(self.pending_waypoints) > 1:
            if self.pending_path_obj: 
                try: self.pending_path_obj.delete()
                except Exception: pass 
            self.pending_path_obj = self.map_widget.set_path(self.pending_waypoints, color="gray", width=2)

    def clear_pending_route(self):
        self.pending_waypoints.clear()
        for marker in self.pending_markers:
            marker.delete()
        self.pending_markers.clear()
        if self.pending_path_obj:
            self.pending_path_obj.delete()
            self.pending_path_obj = None

    def start_simulation(self):
        
        if not self.os_data["waypoints"] and len(self.pending_waypoints) >= 1:
            print("[본선] 임시 항로를 본선 항로로 자동 저장합니다.")
            if self.pending_path_obj:
                self.pending_path_obj.delete()
            final_os_path_obj = self.map_widget.set_path(self.pending_waypoints, color="blue", width=2)
            
            self.os_data["waypoints"] = list(self.pending_waypoints)
            self.os_data["markers"] = list(self.pending_markers) 
            self.os_data["path_obj"] = final_os_path_obj 
            self.pending_waypoints, self.pending_markers, self.pending_path_obj = [], [], None
        
        if len(self.os_data["waypoints"]) < 1: 
            print("오류: 항로점을 1개 이상 추가하세요.")
            return
            
        if self.os_data.get("sim_instance"):
            print("오류: 시뮬레이션이 이미 실행 중입니다.")
            return

        print("[본선] 시뮬레이션 시작...")
        
        try:
            ip = self.os_ip_var.get()
            port = int(self.os_port_var.get())
            if not ip or not (1024 < port < 65535):
                 raise ValueError("유효하지 않은 IP 또는 Port 범위")
                 
            # --- [핵심 수정] Entry에서 속도 값을 읽고 유효성 검사 ---
            speed = float(self.os_data["speed_var"].get())
            if speed <= 0 and len(self.os_data["waypoints"]) > 1:
                print("오류: 항해 속도는 0보다 커야 합니다. (정지/계류는 WP 1개만 찍으세요)")
                return
            elif speed > 0 and len(self.os_data["waypoints"]) == 1:
                print("경고: 항로점이 1개이므로 속도를 0.0으로 강제 설정합니다 (홀딩 모드).")
                speed = 0.0 # 홀딩 모드 진입을 위해
            # --- [수정 완료] ---
            
        except ValueError as e:
            print(f"[본선 오류] IP/Port/속도 값이 올바르지 않습니다: {e}")
            return 

        start_pos = self.os_data["waypoints"][0]
        if self.os_data["ship_marker"] is None:
            self.os_data["ship_marker"] = self.map_widget.set_marker(start_pos[0], start_pos[1], text="SHIP")
        else:
            self.os_data["ship_marker"].set_position(start_pos[0], start_pos[1])
        
        instance = NmeaSimulator(self.os_data["waypoints"], speed, ip, port)
        
        thread = threading.Thread(target=instance.run_simulation, daemon=True)
        thread.start()
        
        self.os_data["sim_instance"] = instance
        self.os_data["sim_thread"] = thread
        
        self.update_os_marker() 
        self.set_ui_state(running=True)

    def update_os_marker(self):
        sim_instance = self.os_data.get("sim_instance")
        sim_thread = self.os_data.get("sim_thread")

        if sim_instance and sim_thread: 
            pos = sim_instance.get_current_position()
            if pos and self.os_data.get("ship_marker"):
                self.os_data["ship_marker"].set_position(pos[0], pos[1])
            
            if sim_thread.is_alive():
                self.after(500, self.update_os_marker)
            elif self.os_data.get("sim_instance"): 
                 print("[GUI] 본선 시뮬레이션 스레드가 종료되었습니다. GUI를 정리합니다.")
                 self.stop_simulation()
        
    def stop_simulation(self):
        if self.os_data.get("sim_instance"):
            self.os_data["sim_instance"].stop()
            self.os_data["sim_instance"] = None
            self.os_data["sim_thread"] = None
        print("본선 시뮬레이션을 중지했습니다.")
        self.set_ui_state(running=False) 

    def clear_route(self):
        print("[본선] 항로를 초기화합니다.")
        self.stop_simulation()
        self.clear_pending_route()
        
        for marker in self.os_data["markers"]: marker.delete()
        if self.os_data["path_obj"]: self.os_data["path_obj"].delete()
        if self.os_data["ship_marker"]: self.os_data["ship_marker"].delete()
        
        self.os_data = {
            "waypoints": [], "markers": [], "path_obj": None,
            "sim_instance": None, "sim_thread": None, "ship_marker": None,
            "speed_var": self.os_data["speed_var"] 
        }

    def on_closing(self):
        self.stop_simulation()
        self.destroy()

# --- 1. 메인 프로그램 실행 ---
if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_closing) 
    app.mainloop()