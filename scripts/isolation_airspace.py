from __future__ import annotations
from shapely.geometry import Point, LineString, shape
from shapely.ops import unary_union


def components(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type=='Polygon':
        return [geometry]
    return [polygon for item in geometry.geoms for polygon in components(item)] if hasattr(geometry,'geoms') else []


def reachable_stations(plan, allowed):
    regions = components(allowed)
    unreachable = []
    for vehicle in plan['vehicles']:
        reachable = [region for region in regions if region.covers(Point(vehicle['home']))]
        missing = [index for index,station in enumerate(vehicle['stations'])
            if not any(region.covers(Point(station)) for region in reachable)]
        if missing:
            unreachable.append(dict(vehicle_id=vehicle['id'],station_indices=missing))
    return dict(connected_components=len(regions),all_stations_reachable=not unreachable,unreachable=unreachable)


def audit_isolation(plan):
    bands = unary_union([shape(partition['buffer']) for partition in plan['partitions']])
    margin = plan['flight_safety']['transit_boundary_margin_m']
    perimeter = shape(plan['perimeter']).buffer(-margin,join_style=2)
    safe = perimeter.difference(bands.buffer(margin,join_style=2))
    crossings = []
    for vehicle in plan['vehicles']:
        for cursor,segment in enumerate(vehicle['segments']):
            start,end = segment['origin'][:2],segment['destination'][:2]
            path = Point(start) if start==end else LineString([start,end])
            if bands.intersects(path):
                crossings.append(dict(vehicle_id=vehicle['id'],cursor=cursor,state=segment['state']))
    alternatives = []
    for permission in (False,True):
        allowed = safe if permission else safe.difference(shape(plan['forest']).buffer(margin,join_style=2))
        alternatives.append(dict(forest_transit_authorized=permission,**reachable_stations(plan,allowed)))
    corridor_candidates = []
    for width in (10,20,30):
        shortened = unary_union([shape(partition['buffer']).intersection(
            shape(partition['geometry']).buffer(-width,join_style=2)) for partition in plan['partitions']])
        allowed = perimeter.difference(shortened.buffer(margin,join_style=2))
        corridor_candidates.append(dict(end_corridor_width_m=width,requires_user_approval=True,
            forest_transit_authorized=True,**reachable_stations(plan,allowed)))
    work_states = {'NAVIGATE','STABILIZE','SCAN','CONFIRM_STATIC','TRACK_MOVING','WAIT_RETURN_SLOT','DESCEND_TO_WORK','REPOSITION_CLIMB','RETURN_CLIMB'}
    work_crossings = [item for item in crossings if item['state'] in work_states]
    isolation_scope = plan.get('airspace_policy',{}).get('isolation_scope','ALL_ALTITUDES')
    return dict(scope='planar isolation diagnostics; work-only transit is authorized by confirmed user policy; not physical flight proof',
        isolation_scope=isolation_scope,work_crossings=work_crossings,
        authorized_policy_respected=not (work_crossings if isolation_scope=='WORK_ONLY' else crossings),
        all_altitude_isolation_respected_by_plan=not crossings,crossings=crossings,
        alternatives=alternatives,corridor_candidates=corridor_candidates,hardware_verified=False)


def require_isolation_permission(plan, data):
    if plan.get('isolation_design',{}).get('kind')=='proposed_end_corridors':
        raise ValueError('候选端部通道尚未获用户确认，不能启动；本文件只用于方案评审')
    permission = data.get('isolation_transit_simulation',False)
    if not isinstance(permission,bool):
        raise ValueError('isolation_transit_simulation必须为布尔值，不能用字符串隐式授权')
    if plan.get('airspace_policy',{}).get('isolation_scope')=='WORK_ONLY':
        return False
    audit = plan.get('isolation_airspace') or audit_isolation(plan)
    if audit['crossings'] and not permission:
        raise ValueError('全高度隔离带阻塞：当前进出场路线穿隔离带。须重新规划通道；仅条件演示可显式授权模拟穿越，不能作为比赛合规方案')
    return permission
