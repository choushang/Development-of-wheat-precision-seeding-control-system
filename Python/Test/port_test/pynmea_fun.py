# 经纬度、速度提取与计算

import pynmea2
def extract_lat_lon_alt(nmea_sentence):
    # 经纬高度显示
    msg = pynmea2.parse(nmea_sentence)
    if isinstance(msg, pynmea2.GGA):
        return msg.latitude, msg.longitude, msg.altitude
    return None, None
def extract_speed_mps(nmea_sentence):
    # 速度计算
    msg = pynmea2.parse(nmea_sentence)
    if isinstance(msg, pynmea2.RMC):
        speed_mps = msg.spd_over_grnd * 0.514444
        return speed_mps
    return None

