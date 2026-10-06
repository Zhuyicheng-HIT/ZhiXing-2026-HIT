from __future__ import annotations
from geodesy import LocalCartesian
import math


class FlightCoordinates:
    def __init__(self, mission_frame, geoid_undulation_m):
        self.mission_frame = mission_frame
        self.geoid_undulation_m = geoid_undulation_m
        self.ekf_frame = None

    def set_ekf_origin(self, latitude_deg, longitude_deg, altitude_msl_m):
        self.ekf_frame = LocalCartesian(latitude_deg,longitude_deg,altitude_msl_m+self.geoid_undulation_m)

    def require_origin(self):
        if self.ekf_frame is None:
            raise ValueError('尚未收到GPS_GLOBAL_ORIGIN，不能拼接各机LOCAL_NED')

    def ned_to_mission(self, position_ned_m):
        self.require_origin()
        north,east,down = position_ned_m
        return self.mission_frame.forward(*self.ekf_frame.reverse([east,north,-down]))

    def mission_to_ned(self, position_enu_m):
        self.require_origin()
        east,north,up = self.ekf_frame.forward(*self.mission_frame.reverse(position_enu_m))
        return [north,east,-up]

    def mission_heading_to_ned(self, heading_rad):
        self.require_origin()
        if isinstance(heading_rad,bool) or not isinstance(heading_rad,(int,float)) or not math.isfinite(heading_rad):
            raise ValueError('任务航向必须为有限弧度值')
        mission_direction = [math.cos(heading_rad),math.sin(heading_rad),0]
        ecef_direction = [sum(self.mission_frame.rotation[row][axis]*mission_direction[row] for row in range(3)) for axis in range(3)]
        east,north,unused_up = [sum(coefficient*value for coefficient,value in zip(row,ecef_direction)) for row in self.ekf_frame.rotation]
        return math.atan2(east,north)
