from __future__ import annotations
import hashlib
import json
import math

WGS84_A_M = 6378137.0
WGS84_F = 1/298.257223563
WGS84_E2 = WGS84_F*(2-WGS84_F)


def decimal_degrees(degrees, minutes, seconds):
    if not all(math.isfinite(value) for value in (degrees,minutes,seconds)) or not 0<=minutes<60 or not 0<=seconds<60:
        raise ValueError('无效度分秒')
    return (-1 if degrees<0 else 1)*(abs(degrees)+minutes/60+seconds/3600)


def geodetic_to_ecef(latitude_deg, longitude_deg, height_ellipsoid_m):
    if not all(math.isfinite(value) for value in (latitude_deg,longitude_deg,height_ellipsoid_m)) or not -90<=latitude_deg<=90 or not -180<=longitude_deg<=180:
        raise ValueError('需要有限WGS84经纬度和椭球高，不能传入缩放整数')
    latitude,longitude = math.radians(latitude_deg),math.radians(longitude_deg)
    radius = WGS84_A_M/math.sqrt(1-WGS84_E2*math.sin(latitude)**2)
    return [(radius+height_ellipsoid_m)*math.cos(latitude)*math.cos(longitude),
        (radius+height_ellipsoid_m)*math.cos(latitude)*math.sin(longitude),
        (radius*(1-WGS84_E2)+height_ellipsoid_m)*math.sin(latitude)]


def ecef_to_geodetic(position):
    if len(position)!=3 or not all(math.isfinite(value) for value in position):
        raise ValueError('需要三个有限ECEF米坐标')
    east_axis,north_axis,polar_axis = position
    radial = math.hypot(east_axis,north_axis)
    if math.hypot(radial,polar_axis)<1:
        raise ValueError('地心附近的椭球坐标不唯一')
    longitude = math.atan2(north_axis,east_axis)
    latitude = math.atan2(polar_axis,radial*(1-WGS84_E2))
    for _ in range(15):
        radius = WGS84_A_M/math.sqrt(1-WGS84_E2*math.sin(latitude)**2)
        updated = math.atan2(polar_axis+WGS84_E2*radius*math.sin(latitude),radial)
        if abs(updated-latitude)<1e-15:
            latitude = updated
            break
        latitude = updated
    height = radial*math.cos(latitude)+polar_axis*math.sin(latitude)-WGS84_A_M*math.sqrt(1-WGS84_E2*math.sin(latitude)**2)
    return [math.degrees(latitude),math.degrees(longitude),height]


class LocalCartesian:
    def __init__(self, latitude_deg, longitude_deg, height_ellipsoid_m):
        self.origin = [latitude_deg,longitude_deg,height_ellipsoid_m]
        self.ecef_origin = geodetic_to_ecef(*self.origin)
        latitude,longitude = math.radians(latitude_deg),math.radians(longitude_deg)
        self.rotation = [
            [-math.sin(longitude),math.cos(longitude),0],
            [-math.sin(latitude)*math.cos(longitude),-math.sin(latitude)*math.sin(longitude),math.cos(latitude)],
            [math.cos(latitude)*math.cos(longitude),math.cos(latitude)*math.sin(longitude),math.sin(latitude)]]
        self.origin_id = hashlib.sha256(json.dumps(self.origin,separators=(',',':')).encode()).hexdigest()[:16]

    def forward(self, latitude_deg, longitude_deg, height_ellipsoid_m):
        delta = [actual-origin for actual,origin in zip(geodetic_to_ecef(latitude_deg,longitude_deg,height_ellipsoid_m),self.ecef_origin)]
        return [sum(coefficient*value for coefficient,value in zip(row,delta)) for row in self.rotation]

    def reverse(self, position_enu_m):
        if len(position_enu_m)!=3 or not all(math.isfinite(value) for value in position_enu_m):
            raise ValueError('需要三个有限ENU米坐标')
        ecef = [self.ecef_origin[axis]+sum(self.rotation[row][axis]*position_enu_m[row] for row in range(3)) for axis in range(3)]
        return ecef_to_geodetic(ecef)

    def descriptor(self):
        return dict(frame_id='mission_map',map_origin_id=self.origin_id,axes='ENU',units='m',
            latitude_deg=self.origin[0],longitude_deg=self.origin[1],height_ellipsoid_m=self.origin[2],ellipsoid='WGS84')


def simulation_frame(datum):
    height = datum['sitl_msl_altitude_m']+datum['sitl_geoid_undulation_m']
    return LocalCartesian(datum['latitude_deg'],datum['longitude_deg'],height)
