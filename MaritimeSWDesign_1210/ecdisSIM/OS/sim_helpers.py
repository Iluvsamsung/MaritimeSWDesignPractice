import math
import time
import functools
import datetime
import operator 

# --- 1. NMEA 유틸리티 (체크섬) ---
def calculate_checksum(sentence):
    """NMEA 0183 문장의 체크섬을 검사합니다."""
    if '$' in sentence or '!' in sentence:
        sentence = sentence.split('$', 1)[-1].split('!', 1)[-1]
    if '*' in sentence:
        sentence = sentence.split('*', 1)[0]
    nmeadata = bytes(sentence, 'utf-8')
    checksum = functools.reduce(operator.xor, nmeadata, 0) # 'operator' 사용
    return f"{checksum:02X}"

# --- 2. 좌표 계산 유틸리티 ---
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

def calculate_destination(p, bearing, distance_nm): # p=(lat, lon)
    """한 지점에서 특정 방위로 특정 거리(NM)만큼 이동한 새 지점을 계산합니다."""
    R_NM = 3440.065  # 지구 반경 (Nautical Miles)
    lat1 = deg_to_rad(p[0])
    lon1 = deg_to_rad(p[1])
    brng = deg_to_rad(bearing)
    d_R = distance_nm / R_NM # 각거리

    lat2 = math.asin(math.sin(lat1) * math.cos(d_R) +
                      math.cos(lat1) * math.sin(d_R) * math.cos(brng))
    lon2 = lon1 + math.atan2(math.sin(brng) * math.sin(d_R) * math.cos(lat1),
                              math.cos(d_R) - math.sin(lat1) * math.sin(lat2))
    return (rad_to_deg(lat2), rad_to_deg(lon2))

def calculate_distance(p1, p2): # p1=(lat, lon), p2=(lat, lon)
    """두 위도/경도 지점 간의 실제 거리(NM)를 계산합니다."""
    R_NM = 3440.065  # 지구 반경 (Nautical Miles)
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
    """문자열을 float으로 안전하게 변환"""
    try:
        return float(s)
    except (ValueError, TypeError):
        return default

def safe_int(s, default=0):
    """문자열을 int로 안전하게 변환"""
    try:
        return int(s)
    except (ValueError, TypeError):
        return default
# --- 2. 좌표 계산 유틸리티 종료 ---

# --- 3. AIS 유틸리티 (제거됨) ---