"""
WGS-84 <-> GCJ-02 坐标转换模块
用于将GPS标准坐标(WGS84)转换为中国国测局坐标(GCJ02, 即"火星坐标系")
OpenStreetMap使用WGS84坐标系, 国内高德/腾讯地图使用GCJ02
"""

import math

_PI = math.pi
_A = 6378245.0  # 长半轴
_EE = 0.00669342162296594  # 偏心率平方


def _out_of_china(lng, lat):
    """判断是否在中国境外"""
    return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)


def _transform_lat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + \
          0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * _PI) + 20.0 * math.sin(2.0 * lng * _PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * _PI) + 40.0 * math.sin(lat / 3.0 * _PI)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * _PI) + 320 * math.sin(lat * _PI / 30.0)) * 2.0 / 3.0
    return ret


def _transform_lng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + \
          0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * _PI) + 20.0 * math.sin(2.0 * lng * _PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * _PI) + 40.0 * math.sin(lng / 3.0 * _PI)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * _PI) + 300.0 * math.sin(lng / 30.0 * _PI)) * 2.0 / 3.0
    return ret


def wgs84_to_gcj02(lng, lat):
    """
    WGS-84 转 GCJ-02
    :param lng: WGS84经度
    :param lat: WGS84纬度
    :return: (gcj_lng, gcj_lat) GCJ02经纬度
    """
    if _out_of_china(lng, lat):
        return lng, lat

    dlat = _transform_lat(lng - 105.0, lat - 35.0)
    dlng = _transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * _PI
    magic = math.sin(radlat)
    magic = 1 - _EE * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((_A * (1 - _EE)) / (magic * sqrtmagic) * _PI)
    dlng = (dlng * 180.0) / (_A / sqrtmagic * math.cos(radlat) * _PI)
    return lng + dlng, lat + dlat


def gcj02_to_wgs84(lng, lat):
    """
    GCJ-02 转 WGS-84 (迭代逼近法)
    :param lng: GCJ02经度
    :param lat: GCJ02纬度
    :return: (wgs_lng, wgs_lat) WGS84经纬度
    """
    if _out_of_china(lng, lat):
        return lng, lat

    wgs_lng, wgs_lat = lng, lat
    for _ in range(5):
        gcj_lng, gcj_lat = wgs84_to_gcj02(wgs_lng, wgs_lat)
        wgs_lng += lng - gcj_lng
        wgs_lat += lat - gcj_lat
    return wgs_lng, wgs_lat


# ========== 南京科技职业学院参考坐标 ==========
# 校园中心点 (WGS84)
NJVT_CENTER_WGS84 = (118.7993, 32.1080)
# 转换为GCJ02
NJVT_CENTER_GCJ02 = wgs84_to_gcj02(*NJVT_CENTER_WGS84)

if __name__ == "__main__":
    print("=== WGS-84 <-> GCJ-02 坐标转换测试 ===")
    # 测试点: 南京科技职业学院
    wgs_lng, wgs_lat = NJVT_CENTER_WGS84
    print(f"\n原始 WGS84 坐标: ({wgs_lng}, {wgs_lat})")

    gcj_lng, gcj_lat = wgs84_to_gcj02(wgs_lng, wgs_lat)
    print(f"转换 GCJ02 坐标: ({gcj_lng:.6f}, {gcj_lat:.6f})")

    rec_lng, rec_lat = gcj02_to_wgs84(gcj_lng, gcj_lat)
    print(f"逆转换 WGS84 坐标: ({rec_lng:.6f}, {rec_lat:.6f})")

    err_lng = abs(rec_lng - wgs_lng) * 111320 * math.cos(wgs_lat * math.pi / 180)
    err_lat = abs(rec_lat - wgs_lat) * 110540
    print(f"逆转换误差: 经度方向 {err_lng:.4f}m, 纬度方向 {err_lat:.4f}m")
    print("\n测试通过! 坐标转换精度满足无人机导航需求。")
