# ais_popup.py

import tkinter
import tkinter.ttk as ttk
from ais_helpers import *
class AisDetailPopup(tkinter.Toplevel):
    """AIS 세부 편집 팝업창 (MMSI 생성, 속도, 모든 정적/항해 정보)"""
    def __init__(self, master, on_save_callback):
        super().__init__(master)
        self.title("AIS 타겟 세부 편집")
        self.transient(master) 
        
        self.on_save_callback = on_save_callback
        
        # 이 팝업창 전용 StringVar 딕셔너리
        self.data_vars = {
            "mmsi": tkinter.StringVar(),
            "ship_name": tkinter.StringVar(),
            "speed": tkinter.StringVar(), # [수정] Entry 위젯을 위해 String으로
            "nav_status_str": tkinter.StringVar(), 
            "call_sign": tkinter.StringVar(), 
            "length": tkinter.StringVar(),
            "beam": tkinter.StringVar(),
            "draught": tkinter.StringVar(),
            "destination": tkinter.StringVar(),
            "eta_month": tkinter.StringVar(), 
            "eta_day": tkinter.StringVar(),
            "eta_hour": tkinter.StringVar(),
            "eta_minute": tkinter.StringVar(),
            "ship_type": tkinter.StringVar(),
            "mmsi_country": tkinter.StringVar(value="Korea"), 
        }
        
        self.protocol("WM_DELETE_WINDOW", self.close_popup) 
        
        frame = ttk.Frame(self, padding=10)
        frame.pack(fill="both", expand=True)
        
        ttk.Label(frame, text="선박 세부 정보", font=("Arial", 12, "bold")).pack(pady=5)
        
        # --- 위젯 생성 ---
        
        # MMSI
        row_mmsi = ttk.Frame(frame)
        ttk.Label(row_mmsi, text="MMSI:", width=18).pack(side="left")
        self.mmsi_entry = ttk.Entry(row_mmsi, textvariable=self.data_vars["mmsi"])
        self.mmsi_entry.pack(side="left", fill="x", expand=True)
        row_mmsi.pack(fill="x", pady=2)

        # MMSI 생성기
        row_mmsi_gen = ttk.Frame(frame)
        ttk.Label(row_mmsi_gen, text="국가(MID):", width=18).pack(side="left")
        countries = ["Korea", "Japan", "USA", "China", "Random"]
        for country in countries:
            ttk.Radiobutton(row_mmsi_gen, text=country, variable=self.data_vars["mmsi_country"], value=country).pack(side="left")
        ttk.Button(row_mmsi_gen, text="생성", command=self.generate_mmsi).pack(side="left", padx=5)
        row_mmsi_gen.pack(fill="x", pady=2)
        
        # Ship Name
        row_name = ttk.Frame(frame)
        ttk.Label(row_name, text="선박 이름 (Ship Name):", width=18).pack(side="left")
        ttk.Entry(row_name, textvariable=self.data_vars["ship_name"]).pack(side="left", fill="x", expand=True)
        row_name.pack(fill="x", pady=2)
        
        # Ship Type
        row_type = ttk.Frame(frame)
        ttk.Label(row_type, text="선박 종류 (Type):", width=18).pack(side="left")
        self.shiptype_menu = ttk.OptionMenu(row_type, self.data_vars["ship_type"], "70: Cargo", "70: Cargo", "80: Tanker", "60: Passenger", "37: Pleasure")
        self.shiptype_menu.pack(fill="x", expand=True)
        row_type.pack(fill="x", pady=2)
        
        # Nav Status
        row_status = ttk.Frame(frame)
        ttk.Label(row_status, text="항해 상태 (Status):", width=18).pack(side="left")
        self.navstatus_menu = ttk.OptionMenu(row_status, self.data_vars["nav_status_str"], "0: Under way", "0: Under way", "1: At anchor", "5: Moored")
        self.navstatus_menu.pack(fill="x", expand=True)
        row_status.pack(fill="x", pady=2)
        
        # [수정] 속도 (Entry)
        row0 = ttk.Frame(frame)
        # [--- 여기가 수정된 줄입니다 ---]
        ttk.Label(row0, text="속도 (Knot):", width=18).pack(side="left")
        # [--- 수정 끝 ---]
        self.speed_entry = ttk.Entry(row0, textvariable=self.data_vars["speed"], width=6)
        self.speed_entry.pack(side="left", padx=2)
        row0.pack(fill="x", pady=5)
        
        # Call Sign
        row1 = ttk.Frame(frame)
        ttk.Label(row1, text="호출부호 (Call Sign):", width=18).pack(side="left")
        ttk.Entry(row1, textvariable=self.data_vars["call_sign"]).pack(side="left", fill="x", expand=True)
        row1.pack(fill="x", pady=2)
        
        # Dimensions (Length / Beam)
        row2 = ttk.Frame(frame)
        ttk.Label(row2, text="L / B (m):", width=18).pack(side="left")
        ttk.Entry(row2, textvariable=self.data_vars["length"], width=6).pack(side="left", padx=2)
        ttk.Label(row2, text="/").pack(side="left")
        ttk.Entry(row2, textvariable=self.data_vars["beam"], width=6).pack(side="left", padx=2)
        row2.pack(fill="x", pady=2)

        # Draught
        row3 = ttk.Frame(frame)
        ttk.Label(row3, text="흘수 (Draught, m):", width=18).pack(side="left")
        ttk.Entry(row3, textvariable=self.data_vars["draught"], width=6).pack(side="left", padx=2)
        row3.pack(fill="x", pady=2)
        
        # Destination
        row4 = ttk.Frame(frame)
        ttk.Label(row4, text="목적지 (Dest):", width=18).pack(side="left")
        ttk.Entry(row4, textvariable=self.data_vars["destination"]).pack(side="left", fill="x", expand=True)
        row4.pack(fill="x", pady=2)
        
        # ETA
        row5 = ttk.Frame(frame)
        ttk.Label(row5, text="ETA (MM-DD HH:MM):", width=18).pack(side="left")
        ttk.Entry(row5, textvariable=self.data_vars["eta_month"], width=3).pack(side="left")
        ttk.Label(row5, text="-").pack(side="left")
        ttk.Entry(row5, textvariable=self.data_vars["eta_day"], width=3).pack(side="left")
        ttk.Entry(row5, textvariable=self.data_vars["eta_hour"], width=3).pack(side="left")
        ttk.Label(row5, text="Z").pack(side="left")
        ttk.Entry(row5, textvariable=self.data_vars["eta_minute"], width=3).pack(side="left")
        row5.pack(fill="x", pady=2)
        
        ttk.Label(frame, text="* ETA 비워두면 현재 시간 기준 자동 계산").pack(anchor="w", pady=2, padx=5)

        ttk.Button(frame, text="저장 후 닫기", command=self.save_and_close).pack(pady=10)

    def generate_mmsi(self):
        """선택한 국가 코드로 MMSI를 생성하여 Entry에 채웁니다."""
        country = self.data_vars["mmsi_country"].get()
        new_mmsi = generate_random_mmsi(country)
        self.data_vars["mmsi"].set(str(new_mmsi))
        print(f"[MMSI] {country} 코드로 {new_mmsi} 생성.")

    def on_speed_scale_change(self, value_str):
        # (이 함수는 이제 사용되지 않음)
        pass

    def open_popup(self, target_data=None):
        """팝업을 열 때 grab 설정 및 데이터 로드"""
        self.load_vars(target_data) 
        self.deiconify() 
        self.grab_set()  

    def save_and_close(self):
        """저장을 App에 알리고 팝업 닫기"""
        
        try:
            current_data = {
                "mmsi": safe_int(self.data_vars["mmsi"].get()),
                "ship_name": self.data_vars["ship_name"].get().strip(),
                "speed": safe_float(self.data_vars["speed"].get(), 10.0), # [수정] Entry에서 읽기
                "nav_status_str": self.data_vars["nav_status_str"].get(),
                "nav_status": int(self.data_vars["nav_status_str"].get().split(":")[0]),
                "call_sign": self.data_vars["call_sign"].get().strip(),
                "length": safe_int(self.data_vars["length"].get(), 150),
                "beam": safe_int(self.data_vars["beam"].get(), 20),
                "draught": safe_float(self.data_vars["draught"].get(), 8.5),
                "destination": self.data_vars["destination"].get().strip(),
                "ship_type": int(self.data_vars["ship_type"].get().split(":")[0]),
                "eta_datetime": None
            }
            
            if self.data_vars["eta_month"].get():
                current_data["eta_datetime"] = datetime.datetime(
                    year=datetime.datetime.utcnow().year,
                    month=safe_int(self.data_vars["eta_month"].get()),
                    day=safe_int(self.data_vars["eta_day"].get()),
                    hour=safe_int(self.data_vars["eta_hour"].get()),
                    minute=safe_int(self.data_vars["eta_minute"].get())
                )
        except Exception as e:
            print(f"오류: 세부 정보 값 변환 실패: {e}")
            return
            
        self.on_save_callback(current_data, save=True) 
        self.close_popup()

    def close_popup(self):
        """팝업을 닫을 때 grab을 해제하고 숨깁니다."""
        self.on_save_callback(None, save=False) 
        self.grab_release() 
        self.withdraw()     

    def load_vars(self, target_data):
        """팝업창의 StringVar에 데이터를 로드합니다 (신규/수정 공용)."""
        
        if target_data:
            # --- 1. 기존 타겟 수정 ---
            s_data = target_data["static_data"]
            self.data_vars["mmsi"].set(str(target_data["mmsi"]))
            self.data_vars["ship_name"].set(s_data.get("ship_name", "--"))
            self.data_vars["speed"].set(f"{s_data.get('speed', 10.0):.1f}")
            self.data_vars["nav_status_str"].set(
                {0: "0: Under way", 1: "1: At anchor", 5: "5: Moored"}.get(s_data.get("nav_status"), "0: Under way")
            )
            self.data_vars["call_sign"].set(s_data.get("call_sign", ""))
            self.data_vars["length"].set(str(s_data.get("length", 150)))
            self.data_vars["beam"].set(str(s_data.get("beam", 20)))
            self.data_vars["draught"].set(str(s_data.get("draught", 8.5)))
            self.data_vars["destination"].set(s_data.get("destination", ""))
            eta = s_data.get("eta_datetime")
            if eta:
                self.data_vars["eta_month"].set(str(eta.month))
                self.data_vars["eta_day"].set(str(eta.day))
                self.data_vars["eta_hour"].set(str(eta.hour))
                self.data_vars["eta_minute"].set(str(eta.minute))
            else:
                self.data_vars["eta_month"].set("")
                self.data_vars["eta_day"].set("")
                self.data_vars["eta_hour"].set("")
                self.data_vars["eta_minute"].set("")
            
            ship_type_code = s_data.get('ship_type', 70)
            ship_type_str = {70:'70: Cargo', 80:'80: Tanker', 60:'60: Passenger', 37:'37: Pleasure'}.get(ship_type_code, '70: Cargo')
            self.data_vars["ship_type"].set(ship_type_str)
            
        else:
            # --- 2. 신규 타겟 (기본값) ---
            next_mmsi = 987654321
            if self.master.ais_targets: 
                try:
                    next_mmsi = max(t["mmsi"] for t in self.master.ais_targets) + 1
                except ValueError: 
                    next_mmsi = 987654321
            
            self.data_vars["mmsi"].set(str(next_mmsi))
            self.data_vars["ship_name"].set("NEW SHIP")
            self.data_vars["speed"].set("10.0")
            self.data_vars["nav_status_str"].set("0: Under way")
            self.data_vars["call_sign"].set("")
            self.data_vars["length"].set("150")
            self.data_vars["beam"].set("20")
            self.data_vars["draught"].set("8.5")
            self.data_vars["destination"].set("KR PUS")
            self.data_vars["eta_month"].set("")
            self.data_vars["eta_day"].set("")
            self.data_vars["eta_hour"].set("")
            self.data_vars["eta_minute"].set("")
            self.data_vars["ship_type"].set("70: Cargo")
            
        self.on_speed_scale_change(self.data_vars["speed"].get())