# ais_app.py ([Running] 텍스트 파싱 오류 수정)

import tkinter
import tkinter.ttk as ttk
import tkintermapview
import threading
import math
import datetime

# 분리된 파일에서 클래스와 함수 임포트
from ais_helpers import *
from ais_engine import AisSimulator
from ais_popup import AisDetailPopup

# --- 7. GUI 애플리케이션 클래스 (AIS 전용) ---
class App(tkinter.Tk):
    def __init__(self):
        super().__init__()
        self.title("NMEA 0183 다중 AIS 시뮬레이터")
        self.geometry("1000x800") 
        
        self.ais_targets = [] # [ {mmsi, waypoints, markers, path_obj, sim_instance, static_data}, ... ]
        
        self.pending_waypoints = []
        self.pending_markers = []
        self.pending_path_obj = None
        
        self.editing_target = None # 현재 수정 중인 AIS 타겟

        self.map_frame = tkinter.Frame(self)
        self.control_frame = ttk.Frame(self, padding=10) 
        self.map_frame.pack(side="left", fill="both", expand=True)
        self.control_frame.pack(side="right", fill="y")

        self.map_widget = tkintermapview.TkinterMapView(self.map_frame, width=800, height=800, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_position(35.10, 129.04)
        self.map_widget.set_zoom(12)
        self.map_widget.add_left_click_map_command(self.on_map_click)

        # --- 제어판 GUI ---
        
        # 1. 전송 설정 (Destination)
        lf_dest = ttk.LabelFrame(self.control_frame, text="1. 전송 설정 (Destination)")
        lf_dest.pack(fill="x", pady=(0, 10))
        
        self.ais_ip_var = tkinter.StringVar(value="192.168.10.3")
        self.ais_port_var = tkinter.StringVar(value="10120")
        
        self.dest_entries = [] 
        
        ttk.Label(lf_dest, text="AIS IP:").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        e3 = ttk.Entry(lf_dest, textvariable=self.ais_ip_var, width=15)
        e3.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(lf_dest, text="AIS Port:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        e4 = ttk.Entry(lf_dest, textvariable=self.ais_port_var, width=8) 
        e4.grid(row=1, column=1, padx=5, pady=2, sticky="w")
        
        self.dest_entries = [e3, e4]
        
        # 2. AIS 타겟 설정 (Planning)
        lf_mode = ttk.LabelFrame(self.control_frame, text="2. AIS 타겟 설정 (Planning)")
        lf_mode.pack(fill="x", pady=(0, 10))
        
        self.detail_popup = AisDetailPopup(self, self.on_popup_save)
        self.detail_popup.withdraw()

        self.btn_edit_ais = ttk.Button(lf_mode, text="[신규 타겟 생성/편집...]", command=self.open_detail_popup)
        self.dest_entries.append(self.btn_edit_ais)
        self.btn_edit_ais.pack(fill="x", pady=(5,0))

        self.btn_add_target = ttk.Button(lf_mode, text="[현재 항로] -> 타겟 추가", command=self.add_ais_target)
        self.dest_entries.append(self.btn_add_target) 
        self.btn_add_target.pack(fill="x", pady=5)
        
        # 3. AIS 타겟 목록
        self.lf_ais_list = ttk.LabelFrame(self.control_frame, text="3. AIS 타겟 목록") 
        self.lf_ais_list.pack(fill="x", pady=(0, 10))
        
        self.ais_list_label = ttk.Label(self.lf_ais_list, text="추가된 AIS 타겟 (더블클릭/선택 후 수정):") 
        self.ais_list_label.pack(anchor="w", pady=(5, 0)) 
        
        self.ais_listbox = tkinter.Listbox(self.lf_ais_list, height=10)
        self.ais_listbox.bind("<Double-Button-1>", self.edit_selected_ais) 
        self.ais_listbox.pack(fill="x", pady=(5,0))
        
        self.btn_delete_ais = ttk.Button(self.lf_ais_list, text="선택한 AIS 타겟 삭제", command=self.delete_selected_ais)
        self.btn_delete_ais.pack(fill="x", pady=2)
        
        self.btn_edit_selected_ais = ttk.Button(self.lf_ais_list, text="선택한 타겟 수정", command=self.edit_selected_ais)
        self.btn_edit_selected_ais.pack(fill="x", pady=2) 
        
        self.dest_entries.append(self.ais_listbox) 
        self.dest_entries.append(self.btn_delete_ais) 
        self.dest_entries.append(self.btn_edit_selected_ais) 
        self.dest_entries.append(self.ais_list_label) 
        self.dest_entries.append(self.lf_ais_list) 
        
        # 4. 실행 제어
        lf_control = ttk.LabelFrame(self.control_frame, text="4. 실행 제어")
        lf_control.pack(fill="x", pady=(0, 10))
        
        self.btn_start_selected = ttk.Button(lf_control, text="선택한 타겟 시작", command=self.start_selected_simulation)
        self.btn_start_selected.pack(fill="x", pady=5)
        
        self.btn_stop_selected = ttk.Button(lf_control, text="선택한 타겟 중지", command=self.stop_selected_simulation)
        self.btn_stop_selected.pack(fill="x", pady=5)
        
        self.dest_entries.append(self.btn_start_selected)
        self.dest_entries.append(self.btn_stop_selected)

        # 5. 초기화
        lf_clear = ttk.LabelFrame(self.control_frame, text="5. 초기화")
        lf_clear.pack(fill="x")
        
        self.btn_clear_all = ttk.Button(lf_clear, text="!!! 전체 초기화 !!!", command=self.clear_all_routes)
        self.btn_clear_all.pack(fill="x", pady=10)
        self.dest_entries.append(self.btn_clear_all) 
        
        self.info_label = ttk.Label(self.control_frame, text="지도 클릭: 항로점 추가", wraplength=180, relief="sunken")
        self.info_label.pack(side="bottom", pady=10, fill="x")
        
        self.set_ui_state(running=False) 

    def _update_listbox_item_text(self, mmsi, new_text):
        """[신규] MMSI를 기준으로 리스트박스 항목 텍스트를 업데이트합니다."""
        for i, item in enumerate(self.ais_listbox.get(0, tkinter.END)):
            if f"(MMSI: {mmsi})" in item:
                self.ais_listbox.delete(i)
                self.ais_listbox.insert(i, new_text)
                return

    def set_ui_state(self, running):
        """[수정] GUI 컴포넌트 잠금 로직을 제거하고, 정보 라벨만 업데이트합니다."""
        state = "normal" 
        for widget in self.dest_entries:
            try:
                if "state" in widget.configure():
                    widget.config(state=state)
            except Exception:
                pass 
        
        if running:
             self.info_label.config(text="시뮬레이션 실행 중...\n타겟별 중지 가능.\n(신규 타겟 추가/수정 가능)")
        else:
             self.info_label.config(text="[AIS 모드] 지도 클릭: 항로점 추가\n(정보 입력 후 '타겟 추가' 클릭)")


    def open_detail_popup(self):
        """'새 타겟'을 위해 팝업을 엽니다."""
        self.editing_target = None # 새 타겟 모드
        self.detail_popup.open_popup(target_data=None)

    def edit_selected_ais(self, event=None):
        """[수정] 리스트박스에서 선택된 타겟의 정보로 팝업을 엽니다."""
        try:
            selected_indices = self.ais_listbox.curselection()
            if not selected_indices:
                print("[AIS] 수정할 타겟을 목록에서 선택하세요.")
                return
            
            selected_index = selected_indices[0]
            # [--- 수정된 부분 1 ---]
            selected_text = self.ais_listbox.get(selected_index)
            
            # (MMSI: 368962950 [Running]) 에서 MMSI 추출
            mmsi_part = selected_text.split("(MMSI: ")[1] # "368962950 [Running])"
            mmsi_str = mmsi_part.split(" ")[0].replace(")", "")  # "368962950"
            mmsi_to_edit = int(mmsi_str)
            # [--- 수정 끝 ---]

            target_data = None
            for target in self.ais_targets:
                if target["mmsi"] == mmsi_to_edit:
                    target_data = target
                    break
            
            if not target_data:
                print(f"[오류] MMSI {mmsi_to_edit}를 찾을 수 없습니다.")
                return

            if target_data.get("sim_instance") and target_data["sim_instance"].is_alive():
                print(f"[오류] 실행 중인 타겟(MMSI: {mmsi_to_edit})은 수정할 수 없습니다. 먼저 중지하세요.")
                return

            print(f"[AIS] 타겟 {target_data['static_data']['ship_name']} ({mmsi_to_edit}) 수정 시작...")
            
            self.editing_target = target_data
            self.detail_popup.open_popup(target_data)

        except Exception as e:
            print(f"[오류] 타겟 수정 창 열기 실패: {e}")

    def on_popup_save(self, data, save=False):
        """[수정] 팝업창 저장/닫기 콜백 (저장 로직 수정)"""
        
        if data is None: # '취소' 또는 'X' 버튼
             self.editing_target = None
             return

        # '저장' 버튼을 눌렀을 때
        if save:
            if self.editing_target:
                # --- 1. 기존 타겟 수정 저장 ---
                print(f"[AIS] 타겟 {self.editing_target['mmsi']} 정보 업데이트 중...")
                try:
                    s_data = self.editing_target["static_data"]
                    
                    s_data["speed"] = data["speed"]
                    s_data["ship_name"] = data["ship_name"]
                    s_data["nav_status"] = data["nav_status"]
                    s_data["ship_type"] = data["ship_type"]
                    s_data["length"] = data["length"]
                    s_data["beam"] = data["beam"]
                    s_data["draught"] = data["draught"]
                    s_data["destination"] = data["destination"]
                    
                    s_data["call_sign"] = data["call_sign"]
                    if not s_data["call_sign"]: 
                        s_data["call_sign"] = "D7" + str(self.editing_target["mmsi"])[:5]
                    
                    s_data["dim_a"] = math.ceil(s_data["length"] / 2)
                    s_data["dim_b"] = s_data["length"] - s_data["dim_a"]
                    s_data["dim_c"] = math.ceil(s_data["beam"] / 2)
                    s_data["dim_d"] = s_data["beam"] - s_data["dim_c"]
                    s_data["eta_datetime"] = data["eta_datetime"]

                    # 리스트박스 텍스트 업데이트
                    self._update_listbox_item_text(
                        self.editing_target['mmsi'], 
                        f"{s_data['ship_name']} (MMSI: {s_data['mmsi']})"
                    )
                    
                    print(f"[AIS] 타겟 {self.editing_target['mmsi']} 정보 업데이트 완료.")

                except Exception as e:
                    print(f"[오류] AIS 타겟 업데이트 실패: {e}")
            
            else:
                # --- 2. 신규 타겟 저장 (add_ais_target으로 이동) ---
                print("[AIS] 신규 타겟 정보가 팝업에서 입력되었습니다.")
                pass

        self.editing_target = None
        
    def on_map_click(self, pos):
        lat, lon = pos
        
        if self.detail_popup.data_vars["nav_status_str"].get() != "0: Under way":
             if self.pending_waypoints:
                 print("오류: 'At anchor'/'Moored' 상태는 1개의 항로점만 설정할 수 있습니다.")
                 return
        
        self.pending_waypoints.append((lat, lon))
        wp_num = len(self.pending_waypoints)
        text = f"AIS WP {wp_num}"
        print(f"[AIS] 임시 항로점 추가: ({lat:.6f}, {lon:.6f})")
        marker = self.map_widget.set_marker(lat, lon, text=text, text_color="green")
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

    def add_ais_target(self):
        """[수정] 현재 그리던 항로 + 팝업창의 AIS 데이터를 타겟 목록에 추가"""
        
        if self.editing_target:
            print("[오류] 현재 타겟을 수정 중입니다. 팝업을 닫고 '신규 타겟...' 버튼을 다시 누르세요.")
            return

        nav_status_str = self.detail_popup.data_vars["nav_status_str"].get()
        nav_status_code = int(nav_status_str.split(":")[0])
        
        if nav_status_code == 0: 
             if len(self.pending_waypoints) < 2:
                 print("오류: 'Under way' 상태는 2개 이상의 항로점이 필요합니다.")
                 return
        else: 
             if len(self.pending_waypoints) < 1:
                 print("오류: 'At anchor'/'Moored' 상태는 1개의 항로점이 필요합니다.")
                 return
             elif len(self.pending_waypoints) > 1:
                 print("오류: 'At anchor'/'Moored' 상태는 1개의 항로점만 설정할 수 있습니다.")
                 self.clear_pending_route()
                 return
        
        try:
            # 팝업(data_vars)에 저장된 '신규' 데이터를 읽어옴
            mmsi = int(self.detail_popup.data_vars["mmsi"].get())
            speed = safe_float(self.detail_popup.data_vars["speed"].get()) # 숫자로 변환
            ship_name = self.detail_popup.data_vars["ship_name"].get().strip()
            ship_type = int(self.detail_popup.data_vars["ship_type"].get().split(":")[0]) 
            length = safe_int(self.detail_popup.data_vars["length"].get(), 100)
            beam = safe_int(self.detail_popup.data_vars["beam"].get(), 20)
            draught = safe_float(self.detail_popup.data_vars["draught"].get(), 5.0)
            destination = self.detail_popup.data_vars["destination"].get().strip()
            call_sign = self.detail_popup.data_vars["call_sign"].get().strip()
            if not call_sign: 
                call_sign = "D7" + str(mmsi)[:5]
            
            eta_dt = None
            if self.detail_popup.data_vars["eta_month"].get():
                eta_dt = datetime.datetime(
                    year=datetime.datetime.utcnow().year,
                    month=safe_int(self.detail_popup.data_vars["eta_month"].get()),
                    day=safe_int(self.detail_popup.data_vars["eta_day"].get()),
                    hour=safe_int(self.detail_popup.data_vars["eta_hour"].get()),
                    minute=safe_int(self.detail_popup.data_vars["eta_minute"].get())
                )
            
            if not ship_name:
                print("오류: 선박 이름을 입력하세요.")
                return
            if any(t["mmsi"] == mmsi for t in self.ais_targets):
                print(f"오류: MMSI {mmsi}는 이미 사용 중입니다.")
                return
        except ValueError as e:
            print(f"오류: 입력 값 확인 필요 (숫자 필드): {e}")
            return
        except Exception as e:
            print(f"오류: {e}")
            return
            
        final_ais_path_obj = None
        if self.pending_path_obj:
            self.pending_path_obj.delete()
            final_ais_path_obj = self.map_widget.set_path(self.pending_waypoints, color="green", width=2)
        
        target_data = {
            "mmsi": mmsi, 
            "waypoints": list(self.pending_waypoints), 
            "markers": list(self.pending_markers),     
            "path_obj": final_ais_path_obj,         
            "sim_instance": None, 
            "ship_marker": None,
            
            "static_data": {
                "mmsi": mmsi, 
                "speed": speed,
                "ship_name": ship_name,
                "ship_type": ship_type, "call_sign": call_sign, 
                "length": length, "beam": beam, "draught": draught,
                "destination": destination,
                "eta_datetime": eta_dt, 
                "nav_status": nav_status_code,
                "dim_a": math.ceil(length / 2), "dim_b": length - math.ceil(length / 2),
                "dim_c": math.ceil(beam / 2), "dim_d": beam - math.ceil(beam / 2)
            }
        }
        self.ais_targets.append(target_data)
        
        print(f"[AIS] 타겟 {ship_name}({mmsi}) 추가 완료.")
        self.ais_listbox.insert(tkinter.END, f"{ship_name} (MMSI: {mmsi})") 
        
        self.pending_waypoints = []
        self.pending_markers = []
        self.pending_path_obj = None
        
        self.detail_popup.load_vars(target_data=None) # 팝업 변수 초기화

    def start_all_simulations(self):
        sim_started_count = 0
        if not self.ais_targets and len(self.pending_waypoints) >= 1:
            print("[AIS] 임시 항로를 첫 번째 타겟으로 추가하고 시작합니다.")
            self.add_ais_target() 

        if len(self.ais_targets) > 0:
            print(f"[AIS] {len(self.ais_targets)}개의 타겟 시뮬레이션을 시작합니다...")
            for target_data in self.ais_targets:
                if not target_data["sim_instance"]: 
                    if self.start_one_ais_sim(target_data): 
                        sim_started_count += 1
                else:
                    print(f"[AIS {target_data['mmsi']}] 시뮬레이션이 이미 실행 중입니다.")
                    sim_started_count += 1
        else:
             print("[AIS] 추가된 타겟이 없습니다.")
        
        if sim_started_count > 0:
            self.set_ui_state(running=True)
            
    def start_selected_simulation(self):
        """[신규] 선택된 AIS 타겟 1개만 시작"""
        try:
            selected_indices = self.ais_listbox.curselection()
            if not selected_indices:
                print("[AIS] 시작할 타겟을 목록에서 선택하세요.")
                return
            
            selected_text = self.ais_listbox.get(selected_indices[0])
            # [--- 수정 (중복 수정)]
            mmsi_part = selected_text.split("(MMSI: ")[1]
            mmsi_str = mmsi_part.split(" ")[0].replace(")", "")
            mmsi_to_start = int(mmsi_str)
            # [--- 수정 끝 ---]
            
            for target_data in self.ais_targets:
                if target_data["mmsi"] == mmsi_to_start:
                    if target_data["sim_instance"]:
                        print(f"[AIS {mmsi_to_start}] 이미 실행 중입니다.")
                    else:
                        if self.start_one_ais_sim(target_data):
                            self.set_ui_state(running=True) # 최소 1개가 실행 중임을 알림
                    return
        except Exception as e:
            print(f"선택한 타겟 시작 오류: {e}")

    def start_one_ais_sim(self, target_data):
        mmsi = target_data["mmsi"]
        print(f"[AIS {mmsi}] 시뮬레이션 시작...")
        
        try:
            ip = self.ais_ip_var.get()
            port = int(self.ais_port_var.get())
            if not ip or not (1024 < port < 65535):
                 raise ValueError("유효하지 않은 IP 또는 Port 범위")
        except ValueError as e:
            print(f"[AIS {mmsi} 오류] IP/Port 설정 오류: {e}")
            return False 
        
        start_pos = target_data["waypoints"][0]
        if target_data["ship_marker"] is None:
            target_data["ship_marker"] = self.map_widget.set_marker(
                start_pos[0], start_pos[1], 
                text=f"{target_data['static_data']['ship_name']}",
                marker_color_circle="green", 
                marker_color_outside="green"
            )
        else:
            target_data["ship_marker"].set_position(start_pos[0], start_pos[1])

        instance = AisSimulator(
            target_data=target_data,
            ip=ip,
            port=port
        )
            
        instance.start() # 스레드 시작
        target_data["sim_instance"] = instance
        
        s_data = target_data["static_data"]
        self._update_listbox_item_text(mmsi, f"{s_data['ship_name']} (MMSI: {mmsi}) [Running]")
        
        self.update_ais_marker(target_data) 
        return True 

    def update_ais_marker(self, target_data):
        if target_data.get("sim_instance"): 
            pos = target_data["sim_instance"].get_current_position()
            if pos and target_data.get("ship_marker"):
                target_data["ship_marker"].set_position(pos[0], pos[1])
            
            sim_thread = target_data.get("sim_instance") 
            if sim_thread and sim_thread.is_alive():
                 self.after(500, lambda: self.update_ais_marker(target_data))
            elif target_data.get("sim_instance"): 
                 print(f"[GUI] AIS {target_data['mmsi']} 스레드가 종료되었습니다. GUI를 정리합니다.")
                 
                 s_data = target_data["static_data"]
                 self._update_listbox_item_text(target_data["mmsi"], f"{s_data['ship_name']} (MMSI: {target_data['mmsi']})")

                 target_data["sim_instance"] = None 
                 self.check_all_sims_stopped()

    def stop_all_simulations(self):
        for target in self.ais_targets:
            if target["sim_instance"]:
                target["sim_instance"].stop()
                
                s_data = target["static_data"]
                self._update_listbox_item_text(target["mmsi"], f"{s_data['ship_name']} (MMSI: {target['mmsi']})")
                
                target["sim_instance"] = None
        print("모든 AIS 시뮬레이션을 중지했습니다.")
        self.set_ui_state(running=False) 

    def stop_selected_simulation(self):
        """[신규] 선택된 AIS 타겟 1개만 중지"""
        try:
            selected_indices = self.ais_listbox.curselection()
            if not selected_indices:
                print("[AIS] 중지할 타겟을 목록에서 선택하세요.")
                return
            
            selected_text = self.ais_listbox.get(selected_indices[0])
            
            # [--- 수정된 부분 2 ---]
            # (MMSI: 368962950 [Running]) 에서 MMSI 추출
            mmsi_part = selected_text.split("(MMSI: ")[1] # "368962950 [Running])"
            mmsi_str = mmsi_part.split(" ")[0].replace(")", "")  # "368962950"
            mmsi_to_stop = int(mmsi_str)
            # [--- 수정 끝 ---]
            
            for target_data in self.ais_targets:
                if target_data["mmsi"] == mmsi_to_stop:
                    if target_data["sim_instance"]:
                        print(f"[AIS {mmsi_to_stop}] 시뮬레이션을 중지합니다.")
                        target_data["sim_instance"].stop()
                        
                        s_data = target_data["static_data"]
                        self._update_listbox_item_text(mmsi_to_stop, f"{s_data['ship_name']} (MMSI: {mmsi_to_stop})")
                        
                        target_data["sim_instance"] = None
                        self.check_all_sims_stopped()
                    else:
                        print(f"[AIS {mmsi_to_stop}] 이미 중지된 상태입니다.")
                    return
        except Exception as e:
            print(f"선택한 타겟 중지 오류: {e}")

    def check_all_sims_stopped(self):
        """[신규] 모든 시뮬레이션이 멈췄는지 확인하고 GUI를 활성화"""
        if not any(t["sim_instance"] for t in self.ais_targets):
            print("모든 시뮬레이션이 중지되었습니다. GUI를 비활성화 상태에서 해제합니다.")
            self.set_ui_state(running=False)

    def delete_selected_ais(self):
        try:
            selected_indices = self.ais_listbox.curselection()
            if not selected_indices:
                print("[AIS] 삭제할 타겟을 목록에서 선택하세요.")
                return
            
            for selected_index in reversed(selected_indices):
                selected_text = self.ais_listbox.get(selected_index) 
                
                # [--- 수정된 부분 3 ---]
                # (MMSI: 368962950 [Running]) 에서 MMSI 추출
                mmsi_part = selected_text.split("(MMSI: ")[1] # "368962950 [Running])"
                mmsi_str = mmsi_part.split(" ")[0].replace(")", "")  # "368962950"
                mmsi_to_delete = int(mmsi_str)
                # [--- 수정 끝 ---]
                
                target_to_delete = None
                for target in self.ais_targets:
                    if target["mmsi"] == mmsi_to_delete:
                        target_to_delete = target
                        break
                if target_to_delete:
                    print(f"[AIS {mmsi_to_delete}] 타겟을 삭제합니다.")
                    if target_to_delete["sim_instance"]:
                        target_to_delete["sim_instance"].stop()
                    for marker in target_to_delete["markers"]: marker.delete()
                    if target_to_delete["path_obj"]: target_to_delete["path_obj"].delete()
                    if target_to_delete["ship_marker"]: target_to_delete["ship_marker"].delete()
                    self.ais_listbox.delete(selected_index)
                    self.ais_targets.remove(target_to_delete)
                else:
                    print(f"[오류] MMSI {mmsi_to_delete}를 데이터에서 찾을 수 없습니다.")
                    self.ais_listbox.delete(selected_index) 
        except Exception as e:
            print(f"[오류] 타겟 삭제 중 예외 발생: {e}")

    def clear_all_routes(self):
        print("!!! 모든 항로점과 마커를 초기화합니다 !!!")
        self.stop_all_simulations() 
        self.clear_pending_route()
        for target in self.ais_targets:
            for marker in target["markers"]: marker.delete()
            if target["path_obj"]: target["path_obj"].delete()
            if target["ship_marker"]: target_to_delete["ship_marker"].delete() # [버그 수정] target_to_delete -> target
        self.ais_targets.clear()
        self.ais_listbox.delete(0, tkinter.END)
        self.set_ui_state(running=False) 

    def on_closing(self):
        self.stop_all_simulations()
        self.destroy()

# --- 1. 메인 프로그램 실행 ---
if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_closing) 
    app.mainloop()