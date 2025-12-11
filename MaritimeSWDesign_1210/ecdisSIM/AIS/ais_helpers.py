import math
import time
import functools
import datetime
import operator 
import random

# --- 1. NMEA 유틸리티 (체크섬) ---
def calculate_checksum(sentence):
    """NMEA 0183 문장의 체크섬을 검사합니다."""
    if '$' in sentence or '!' in sentence:
        sentence = sentence.split('$', 1)[-1].split('!', 1)[-1]
    if '*' in sentence:
        sentence = sentence.split('*', 1)[0]
    nmeadata = bytes(sentence, 'utf-8')
    checksum = functools.reduce(operator.xor, nmeadata, 0)
    return f"{checksum:02X}"

# --- 2. 좌표 계산 유틸리티 ---
def deg_to_rad(deg):
    return deg * math.pi / 180.0
def rad_to_deg(rad):
    return rad * 180.0 / math.pi

def calculate_bearing(p1, p2): # p1=(lat, lon), p2=(lat, lon)
    lat1 = deg_to_rad(p1[0])
    lon1 = deg_to_rad(p1[1])
    lat2 = deg_to_rad(p2[0])
    lon2 = deg_to_rad(p2[1])
    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    bearing = math.atan2(y, x)
    return (rad_to_deg(bearing) + 360.0) % 360.0

def calculate_destination(p, bearing, distance_nm): # p=(lat, lon)
    R_NM = 3440.065
    lat1 = deg_to_rad(p[0])
    lon1 = deg_to_rad(p[1])
    brng = deg_to_rad(bearing)
    d_R = distance_nm / R_NM
    lat2 = math.asin(math.sin(lat1) * math.cos(d_R) +
                      math.cos(lat1) * math.sin(d_R) * math.cos(brng))
    lon2 = lon1 + math.atan2(math.sin(brng) * math.sin(d_R) * math.cos(lat1),
                              math.cos(d_R) - math.sin(lat1) * math.sin(lat2))
    return (rad_to_deg(lat2), rad_to_deg(lon2))

def calculate_distance(p1, p2): # p1=(lat, lon), p2=(lat, lon)
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

def format_lat_nmea(lat):
    deg = int(lat)
    min_val = (lat - deg) * 60.0
    return f"{abs(deg):02d}{abs(min_val):07.4f}"
def format_lon_nmea(lon):
    deg = int(lon)
    min_val = (lon - deg) * 60.0
    return f"{abs(deg):03d}{abs(min_val):07.4f}"

def safe_float(s, default=0.0):
    try: return float(s)
    except (ValueError, TypeError): return default

def safe_int(s, default=0):
    try: return int(s)
    except (ValueError, TypeError): return default
# --- 2. 좌표 계산 유틸리티 종료 ---


# --- 3. AIS (!AIVDM) 생성 유틸리티 ---
AIS_CHAR_MAP = {i: chr(i + 48) if i < 40 else chr(i + 56) for i in range(64)}
AIS_STR_MAP = {
    '@': 0, 'A': 1, 'B': 2, 'C': 3, 'D': 4, 'E': 5, 'F': 6, 'G': 7,
    'H': 8, 'I': 9, 'J': 10, 'K': 11, 'L': 12, 'M': 13, 'N': 14, 'O': 15,
    'P': 16, 'Q': 17, 'R': 18, 'S': 19, 'T': 20, 'U': 21, 'V': 22, 'W': 23,
    'X': 24, 'Y': 25, 'Z': 26, '[': 27, '\\': 28, ']': 29, '^': 30, '_': 31,
    ' ': 32, '!': 33, '"': 34, '#': 35, '$': 36, '%': 37, '&': 39, "'": 39,
    '(': 40, ')': 41, '*': 42, '+': 43, ',': 44, '-': 45, '.': 46, '/': 47,
    '0': 48, '1': 49, '2': 50, '3': 51, '4': 52, '5': 53, '6': 54, '7': 55,
    '8': 56, '9': 57, ':': 58, ';': 59, '<': 60, '=': 61, '>': 62, '?': 63
}

# 국가별 MMSI 앞자리 (MID)
COUNTRY_MIDS = {
    "Korea": ["440", "441"],
    "Japan": ["431", "432"],
    "USA": ["338", "366", "367", "368", "369"],
    "China": ["412", "413", "414"],
    "Random": ["999"] # 기타
}

def generate_random_mmsi(country_code="Korea"):
    """선택된 국가의 MID로 9자리 MMSI를 랜덤 생성"""
    mid = random.choice(COUNTRY_MIDS.get(country_code, ["999"])) # 기본값 999
    suffix = str(random.randint(0, 999999)).zfill(6) # 6자리 랜덤 숫자
    return int(mid + suffix)

def _int_to_bin_payload(value, length):
    if value < 0:
        value = (1 << length) + value
    return format(value, f'0{length}b')

def _ais_str_to_payload_bin(text, length_bits):
    binary_payload = ""
    max_chars = length_bits // 6
    text = text.upper().ljust(max_chars, '@') 
    for char in text[:max_chars]:
        val = AIS_STR_MAP.get(char, 0) 
        binary_payload += _int_to_bin_payload(val, 6)
    return binary_payload

def _bin_to_6bit_ascii(payload):
    encoded_chars = []
    for i in range(0, len(payload), 6):
        chunk = payload[i:i+6]
        if len(chunk) < 6:
            chunk = chunk.ljust(6, '0') 
        chunk_val = int(chunk, 2)
        encoded_chars.append(AIS_CHAR_MAP[chunk_val])
    return "".join(encoded_chars)

def pack_aivdm_message_1(mmsi, lat, lon, sog_kn, cog_deg, heading_deg, nav_status): 
    """AIS Class A 위치 보고서 (Message 1) 페이로드를 생성합니다 (168 비트)."""
    payload = ""
    payload += _int_to_bin_payload(1, 6)
    payload += _int_to_bin_payload(0, 2)
    payload += _int_to_bin_payload(mmsi, 30)
    payload += _int_to_bin_payload(nav_status, 4)
    payload += _int_to_bin_payload(0, 8)
    sog_val = int(sog_kn * 10)
    payload += _int_to_bin_payload(sog_val, 10)
    payload += _int_to_bin_payload(1, 1)
    lon_val = int(lon * 60 * 10000)
    payload += _int_to_bin_payload(lon_val, 28)
    lat_val = int(lat * 60 * 10000)
    payload += _int_to_bin_payload(lat_val, 27)
    cog_val = int(cog_deg * 10)
    payload += _int_to_bin_payload(cog_val, 12)
    hdg_val = int(heading_deg)
    payload += _int_to_bin_payload(hdg_val, 9)
    ts_val = time.gmtime().tm_sec
    payload += _int_to_bin_payload(ts_val, 6)
    payload += _int_to_bin_payload(0, 2)
    payload += _int_to_bin_payload(0, 3)
    payload += _int_to_bin_payload(0, 1)
    payload += _int_to_bin_payload(0, 19)
    return _bin_to_6bit_ascii(payload)

def pack_aivdm_message_5(mmsi, call_sign, ship_name, ship_type, dim_a, dim_b, dim_c, dim_d, eta, draught, destination): 
    """AIS 정적/항해 데이터 (Message 5) 페이로드를 생성합니다 (424 비트)."""
    binary_payload = ""
    binary_payload += _int_to_bin_payload(5, 6)
    binary_payload += _int_to_bin_payload(0, 2)
    binary_payload += _int_to_bin_payload(mmsi, 30)
    binary_payload += _int_to_bin_payload(0, 2)
    binary_payload += _int_to_bin_payload(0, 30) # IMO Number
    binary_payload += _ais_str_to_payload_bin(call_sign, 42) 
    binary_payload += _ais_str_to_payload_bin(ship_name, 120) 
    binary_payload += _int_to_bin_payload(ship_type, 8) 
    binary_payload += _int_to_bin_payload(dim_a, 9) 
    binary_payload += _int_to_bin_payload(dim_b, 9) 
    binary_payload += _int_to_bin_payload(dim_c, 6) 
    binary_payload += _int_to_bin_payload(dim_d, 6) 
    binary_payload += _int_to_bin_payload(0, 4) # EPFD Type
    if eta:
        binary_payload += _int_to_bin_payload(eta.month, 4) 
        binary_payload += _int_to_bin_payload(eta.day, 5)   
        binary_payload += _int_to_bin_payload(eta.hour, 5)  
        binary_payload += _int_to_bin_payload(eta.minute, 6)
    else:
        binary_payload += _int_to_bin_payload(0, 4) 
        binary_payload += _int_to_bin_payload(0, 5) 
        binary_payload += _int_to_bin_payload(24, 5)
        binary_payload += _int_to_bin_payload(60, 6)
    draught_val = int(draught * 10)
    binary_payload += _int_to_bin_payload(draught_val, 8)
    binary_payload += _ais_str_to_payload_bin(destination, 120) 
    binary_payload += _int_to_bin_payload(0, 1) # DTE
    binary_payload += _int_to_bin_payload(0, 1) # Spare
    ascii_payload = _bin_to_6bit_ascii(binary_payload)
    return ascii_payload[:56], ascii_payload[56:]
# --- 3. AIS 유틸리티 종료 ---