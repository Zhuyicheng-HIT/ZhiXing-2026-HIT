from __future__ import annotations
import math
from shapely.geometry import Point, shape


def scan_motion(origin, target, speed_m_s):
    if isinstance(speed_m_s,bool) or not isinstance(speed_m_s,(int,float)) or not math.isfinite(speed_m_s) or not 0<speed_m_s<=4:
        raise ValueError('Ground scan speed must be finite and within (0, 4] m/s')
    distance = math.dist(origin[:2],target[:2])
    return dict(origin_enu_m=list(origin),destination_enu_m=list(target),distance_m=distance,
        maximum_ground_speed_m_s=speed_m_s,minimum_slew_s=distance/speed_m_s,
        scope='nominal flat-ground optical-axis motion; not measured ZR10 motion')


def scan_look_target(segment, elapsed_s):
    motion = segment.get('scan_motion')
    if not motion:
        return segment.get('target')
    duration = motion['minimum_slew_s']
    fraction = min(1,max(0,elapsed_s/duration)) if duration else 1
    if fraction>=1:
        return list(motion['destination_enu_m'])
    return [start+(end-start)*fraction for start,end in zip(motion['origin_enu_m'],motion['destination_enu_m'])]


def target_in_sector(vehicle, target_enu_m):
    if not isinstance(target_enu_m,(list,tuple)) or len(target_enu_m)!=3 or any(isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) for value in target_enu_m):
        raise ValueError('Target position requires three finite ENU coordinates')
    inside = shape(vehicle['search_region']).covers(Point(target_enu_m[:2]))
    return dict(action='TRACK_WITHIN_SECTOR' if inside else 'ABANDON_NO_HANDOFF',inside_search_region=inside,
        aircraft_may_leave_work_region=False,handoff=False)
