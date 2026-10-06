from __future__ import annotations
import bisect
import heapq
import math
import json
from copy import deepcopy
from pathlib import Path
import yaml
from shapely.geometry import Point, Polygon, LineString, box, mapping, shape
from shapely.ops import unary_union, nearest_points
from geodesy import decimal_degrees, simulation_frame
from scan_protocol import OBSERVATION_FIELDS, validate_scan_profile
from coverage_candidates import sparse_stations
from isolation_airspace import audit_isolation
from scan_motion import scan_motion
from regional_scan import regional_scan
from owned_coverage import balanced_partition, rescue_coverage, route_order

ROOT = Path(__file__).resolve().parents[1]


def site_geometry():
    config = yaml.safe_load((ROOT/'config/site.yaml').read_text(encoding='utf-8'))
    coordinates = json.loads((ROOT/'config/boundary_wgs84.json').read_text(encoding='utf-8'))
    datum = dict(config['site']['datum'])
    longitude,latitude = coordinates['boundary_points_dms']['2']
    datum.update(longitude_deg=decimal_degrees(*longitude),latitude_deg=decimal_degrees(*latitude))
    frame = simulation_frame(datum)
    def project(value):
        longitude,latitude = value
        return frame.forward(decimal_degrees(*latitude),decimal_degrees(*longitude),frame.origin[2])[:2]
    def project_decimal(value):
        longitude,latitude = value
        return frame.forward(latitude,longitude,frame.origin[2])[:2]
    points = {int(key):project(value) for key,value in coordinates['boundary_points_dms'].items()}
    forest = [project(value) for value in coordinates['forest_points_dms']]
    subject3 = [project_decimal(value) for value in coordinates['subject3_exclusion_wgs84']]
    subject3_reinclude = [project_decimal(value) for value in coordinates.get('subject3_reinclude_wgs84', [])]
    return datum,points,forest,subject3,subject3_reinclude


def polygons(geometry):
    if geometry.is_empty:
        return []
    return [geometry] if geometry.geom_type == 'Polygon' else [part for part in geometry.geoms if part.geom_type == 'Polygon']


def route_inside(start, end, region):
    allowed = region.buffer(0.001)
    if not allowed.covers(Point(start)) or not allowed.covers(Point(end)):
        raise ValueError(f'路线端点越界: {start}, {end}')
    if allowed.covers(LineString([start, end])):
        return [tuple(end)]
    vertices = [tuple(start), tuple(end)]
    for polygon in polygons(region):
        vertices.extend(list(polygon.exterior.coords)[:-1])
        for ring in polygon.interiors:
            vertices.extend(list(ring.coords)[:-1])
    edges = [[] for _ in vertices]
    for first in range(len(vertices)):
        for second in range(first + 1, len(vertices)):
            edge = LineString([vertices[first], vertices[second]])
            if allowed.covers(edge):
                length = edge.length
                edges[first].append((second, length))
                edges[second].append((first, length))
    queue, costs, parents = [(0, 0)], {0: 0}, {}
    while queue:
        cost, current = heapq.heappop(queue)
        if current == 1:
            path = [1]
            while path[-1] != 0:
                path.append(parents[path[-1]])
            return [vertices[index] for index in reversed(path[:-1])]
        if cost != costs[current]:
            continue
        for neighbor, length in edges[current]:
            new_cost = cost + length
            if new_cost < costs.get(neighbor, math.inf):
                costs[neighbor], parents[neighbor] = new_cost, current
                heapq.heappush(queue, (new_cost, neighbor))
    raise ValueError(f'不存在合法区域内的通路: {start} -> {end}; 区域{region.geom_type}')


def add_segment(vehicle, destination, state, duration=None, **metadata):
    origin = vehicle['position'][:]
    duration = duration if duration is not None else max(0.01, math.dist(origin, destination) / 6)
    duration = max(duration, 0.01)
    vehicle['segments'].append(dict(start=vehicle['clock'],end=vehicle['clock']+duration,origin=origin,destination=list(destination),state=state,**metadata))
    vehicle['clock'] += duration
    vehicle['position'] = list(destination)


def navigate(vehicle, destination, region, state):
    start = vehicle['position'][:2]
    end = destination[:2]
    if region.buffer(0.001).covers(LineString([start,end])):
        add_segment(vehicle, destination, state)
        return
    for point in route_inside(start, end, region):
        add_segment(vehicle, [*point, destination[2]], state)


def pose_at(vehicle, stamp):
    index = min(bisect.bisect_right(vehicle['ends'], stamp), len(vehicle['segments']) - 1)
    segment = vehicle['segments'][index]
    fraction = max(0, min(1, (stamp-segment['start'])/(segment['end']-segment['start'])))
    return [origin+(end-origin)*fraction for origin,end in zip(segment['origin'],segment['destination'])],segment,index


def validate_separation(vehicles, horizontal=8, vertical=8):
    minimum, closest, violations = math.inf, None, []
    for first_index, first in enumerate(vehicles):
        for second in vehicles[first_index+1:]:
            stamps = sorted(set([0]+first['ends']+second['ends']))
            for start,end in zip(stamps,stamps[1:]):
                offset = [value-other for value,other in zip(pose_at(first,start)[0],pose_at(second,start)[0])]
                delta = [value-other-initial for value,other,initial in zip(pose_at(first,end)[0],pose_at(second,end)[0],offset)]
                norm = sum(value*value for value in delta)
                fraction = max(0,min(1,-sum(value*change for value,change in zip(offset,delta))/norm)) if norm else 0
                nearest = math.sqrt(sum((value+fraction*change)**2 for value,change in zip(offset,delta)))
                if nearest < minimum:
                    minimum,closest = nearest,dict(first=first['id'],second=second['id'],time_s=start+fraction*(end-start))
                low,high = 0.0,1.0
                if abs(delta[2]) < 1e-10:
                    if abs(offset[2]) >= vertical:
                        continue
                else:
                    roots = sorted([(-vertical-offset[2])/delta[2],(vertical-offset[2])/delta[2]])
                    low,high = max(0,roots[0]),min(1,roots[1])
                    if low >= high:
                        continue
                norm = delta[0]**2+delta[1]**2
                fraction = max(low,min(high,-(offset[0]*delta[0]+offset[1]*delta[1])/norm)) if norm else low
                separation = math.hypot(offset[0]+fraction*delta[0],offset[1]+fraction*delta[1])
                if separation < horizontal-1e-6:
                    violations.append(dict(first=first['id'],second=second['id'],time_s=start+fraction*(end-start),horizontal_m=separation))
    return dict(passed=not violations,min_3d_distance_m=minimum,closest=closest,horizontal_limit_m=horizontal,vertical_limit_m=vertical,violations=violations)


def order_stations(stations, safe, home):
    groups = []
    for component in polygons(safe):
        group = [point for point in stations if component.buffer(.001).covers(Point(point))]
        if group:
            groups.append((component,group))
    if sum(len(group) for component,group in groups)!=len(stations):
        raise ValueError('扫描站位未能唯一分配到可飞连通片')
    ordered,current = [],list(home)
    while groups:
        component,group = min(groups,key=lambda item:min(math.dist(current,point) for point in item[1]))
        groups.remove((component,group))
        distances = {}
        def distance(first, second):
            key = tuple(sorted((tuple(first),tuple(second))))
            if key not in distances:
                path = route_inside(first,second,component)
                distances[key] = sum(math.dist(origin,destination) for origin,destination in zip([first]+path[:-1],path))
            return distances[key]
        first = min(group,key=lambda point:math.dist(current,point))
        group.remove(first)
        route = [first]
        while group:
            next_point = min(group,key=lambda point:distance(route[-1],point))
            group.remove(next_point)
            route.append(next_point)
        for _ in range(len(route)):
            improved = False
            for start in range(1,len(route)-1):
                for end in range(start+1,len(route)):
                    before = distance(route[start-1],route[start])
                    after = distance(route[start-1],route[end])
                    if end+1<len(route):
                        before += distance(route[end],route[end+1])
                        after += distance(route[start],route[end+1])
                    if after<before-1e-6:
                        route[start:end+1] = reversed(route[start:end+1])
                        improved = True
            if not improved:
                break
        ordered.extend(route)
        current = route[-1]
    return ordered


def greedy_stations(part, safe, footprint, home):
    radius, remaining, stations = footprint/2,part,[]
    min_x,min_y,max_x,max_y = part.bounds
    spacing = footprint*.8
    candidates = []
    for column in range(max(1,math.ceil((max_x-min_x)/spacing))):
        for row in range(max(1,math.ceil((max_y-min_y)/spacing))):
            point = Point(min_x+(column+.5)*spacing,min_y+(row+.5)*spacing)
            if safe.covers(point):
                candidates.append(point)
    for _ in range(400):
        if remaining.area < .01:
            break
        for polygon in polygons(remaining):
            point = polygon.representative_point()
            candidates.append(point if safe.covers(point) else nearest_points(safe,point)[0])
        best,score = None,0
        for point in candidates:
            area = box(point.x-radius,point.y-radius,point.x+radius,point.y+radius).intersection(remaining).area
            if area > score:
                best,score = point,area
        if score < .01:
            break
        stations.append([best.x,best.y])
        remaining = remaining.difference(box(best.x-radius,best.y-radius,best.x+radius,best.y+radius))
        candidates = [point for point in candidates if point.distance(best)>.1]
    for index in reversed(range(len(stations))):
        others = unary_union([box(point[0]-radius,point[1]-radius,point[0]+radius,point[1]+radius)
            for other,point in enumerate(stations) if other!=index])
        if part.difference(others).area<.01:
            stations.pop(index)
    return order_stations(stations,safe,home),remaining.area


def stations_for(part, safe, footprint, home):
    config = json.loads((ROOT/'config/coverage_planning.json').read_text(encoding='utf-8'))
    return sparse_stations(part,safe,footprint,home,greedy_stations,order_stations,
        preferred_clearance=config['preferred_station_clearance_m'])


def build_plan(homes=None, subject=1, footprint=80,proposed_isolation_end_corridor_m=None):
    if proposed_isolation_end_corridor_m is not None and (
            isinstance(proposed_isolation_end_corridor_m,bool) or proposed_isolation_end_corridor_m not in (10,20,30)):
        raise ValueError('候选端部通道宽度仅支持10/20/30m，尚待用户确认')
    if subject not in (1,2):
        raise ValueError('仅支持科目一、二')
    if not 40 <= footprint <= 120:
        raise ValueError('模拟扫描边长必须在40~120m；不是实机标定值')
    datum,points,forest_points,subject3_points,subject3_reinclude_points = site_geometry()
    scan_profile = json.loads((ROOT/'config/scan_profile.json').read_text(encoding='utf-8'))
    airspace_policy = json.loads((ROOT/'config/airspace_policy.json').read_text(encoding='utf-8'))
    if airspace_policy.get('isolation_scope') not in ('WORK_ONLY','ALL_ALTITUDES') or not isinstance(airspace_policy.get('forest_transit_authorized'),bool):
        raise ValueError('Invalid explicit airspace policy')
    validate_scan_profile(scan_profile)
    observation_conditions = {name:scan_profile[name] for name in OBSERVATION_FIELDS}
    coverage_planning = json.loads((ROOT/'config/coverage_planning.json').read_text(encoding='utf-8'))
    if coverage_planning.get('strategy') not in ('greedy','sparse_grid_candidates','owned_scan_cells'):
        raise ValueError('Unknown coverage planning strategy')
    clearance = coverage_planning.get('preferred_station_clearance_m')
    if isinstance(clearance,bool) or not isinstance(clearance,(int,float)) or not math.isfinite(clearance) or clearance<0:
        raise ValueError('Invalid preferred station clearance')
    station_solver = greedy_stations if coverage_planning['strategy']=='greedy' else stations_for
    coordination = json.loads((ROOT/'config/coordination.json').read_text(encoding='utf-8'))
    interval = coordination.get('entry_release_interval_s')
    if isinstance(interval,bool) or not isinstance(interval,(int,float)) or not math.isfinite(interval) or interval<=0:
        raise ValueError('Invalid entry release interval')
    if coordination.get('entry_policy') not in ('exclusive','layered_reserved'):
        raise ValueError('Unknown entry coordination policy')
    if coordination.get('return_policy','exclusive') not in ('exclusive','layered_reserved'):
        raise ValueError('Unknown return coordination policy')
    if coordination.get('return_policy')=='layered_reserved' and coordination['entry_policy']!='layered_reserved':
        raise ValueError('Reserved returns require the layered command reservation executor')
    if coordination.get('hold_recovery_policy','arrived_and_slow_before_releasing_paths')!='arrived_and_slow_before_releasing_paths':
        raise ValueError('Unknown hold recovery policy')
    distance = coordination.get('minimum_command_path_distance_m')
    if isinstance(distance,bool) or not isinstance(distance,(int,float)) or not math.isfinite(distance) or distance<=10:
        raise ValueError('Reserved command path distance must exceed the 10m measured hold threshold')
    safety = json.loads((ROOT/'config/flight_safety.json').read_text(encoding='utf-8'))
    for name in ('nominal_work_altitude_m','commanded_altitude_margin_m','navigation_boundary_margin_m',
                 'transit_boundary_margin_m','estimated_horizontal_body_radius_m','maximum_altitude_m','minimum_operating_altitude_m'):
        value = safety.get(name)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Invalid flight safety parameter: '+name)
    work_altitude = safety['nominal_work_altitude_m']+safety['commanded_altitude_margin_m']
    if not safety['minimum_operating_altitude_m']<work_altitude<60 or safety['maximum_altitude_m']!=120:
        raise ValueError('Work altitude must leave margin above the operating floor and below entry layers')
    entry_altitudes = coordination.get('entry_altitudes_m')
    if not isinstance(entry_altitudes,dict) or set(entry_altitudes)!={f'uav_{index}' for index in range(1,7)}:
        raise ValueError('Entry altitudes must specify each of the six vehicles')
    if any(isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value)
           or not safety['minimum_operating_altitude_m']<=value<=safety['maximum_altitude_m']
           for value in entry_altitudes.values()) or len(set(entry_altitudes.values()))!=6:
        raise ValueError('Entry altitudes must be distinct and within the operating altitude range')
    entry_order = coordination.get('entry_order')
    if not isinstance(entry_order,list) or len(entry_order)!=6 or set(entry_order)!=set(entry_altitudes):
        raise ValueError('Entry order must contain every vehicle exactly once')
    safety['rule_minimum_altitude_m'] = 40 if subject==1 else 30
    if safety['minimum_operating_altitude_m']<safety['rule_minimum_altitude_m']:
        raise ValueError('Operating altitude floor cannot be below the competition rule floor')
    if safety['estimated_horizontal_body_radius_m']>=min(safety['navigation_boundary_margin_m'],safety['transit_boundary_margin_m']):
        raise ValueError('Estimated body envelope must fit inside navigation and transit margins')
    perimeter = Polygon([points[index] for index in range(1,26)])
    launch = Polygon([points[index] for index in [2,3,4,5]])
    forest = Polygon(forest_points)
    subject3_exclusion = Polygon(subject3_points).buffer(2.0,join_style=2)
    subject3_reinclude = Polygon(subject3_reinclude_points) if subject3_reinclude_points else Polygon()
    legal = perimeter.difference(unary_union([forest,subject3_exclusion])).union(subject3_reinclude)
    legal = legal.difference(forest).intersection(perimeter)
    transit = perimeter.buffer(-safety['transit_boundary_margin_m'],join_style=2)
    departure = Polygon([points[index] for index in [6,8,9,1]]).intersection(legal).difference(launch)
    grass = Polygon([points[index] for index in [20,15,16,19]]).intersection(legal).difference(launch)
    house = legal.difference(unary_union([departure,grass,launch]))
    west_forest_corridor = Polygon([points[1],points[25],points[24],forest_points[1],
        forest_points[2],forest_points[3],forest_points[4],points[13],points[9]])
    transferred = house.intersection(west_forest_corridor)
    departure = departure.union(transferred)
    house = house.difference(transferred)
    regions = dict(takeoff=departure,house=house,grass=grass)
    names = dict(takeoff='出发搜索区',house='房区（补全余区）',grass='草地区')
    homes = [[-38,12],[-21,12],[-38,29],[-21,29],[-38,46],[-21,46]] if homes is None else homes
    if len(homes)!=6 or any(len(point)!=2 or not all(math.isfinite(float(value)) for value in point) for point in homes):
        raise ValueError('需要六个有限ENU坐标[东,北]')
    if any(not launch.buffer(-4).covers(Point(point)) for point in homes):
        raise ValueError('起降位置必须位于2345起降框内，距边界至少4m')
    if min(math.dist(first,second) for index,first in enumerate(homes) for second in homes[index+1:])<12:
        raise ValueError('起降位置间距必须至少12m')
    vehicles,partitions = [],[]
    for sector_index,(zone,region) in enumerate(regions.items()):
        zone_altitude = scan_profile['region_scans'][zone].get('work_altitude_m',work_altitude)
        if not safety['minimum_operating_altitude_m']<zone_altitude<=safety['maximum_altitude_m']:
            raise ValueError('Regional work altitude outside operating limits')
        scan_configuration = regional_scan(scan_profile,zone,footprint,zone_altitude)
        optimized_pair = None
        min_x,min_y,max_x,max_y = region.bounds
        horizontal = max_x-min_x > max_y-min_y
        low,high = (min_x,max_x) if horizontal else (min_y,max_y)
        for _ in range(35):
            middle = (low+high)/2
            clipping = box(-2000,-2000,middle,2000) if horizontal else box(-2000,-2000,2000,middle)
            if region.intersection(clipping).area < region.area/2:
                low = middle
            else:
                high = middle
        split = (low+high)/2
        band = box(split-10,-2000,split+10,2000) if horizontal else box(-2000,split-10,2000,split+10)
        if coverage_planning['strategy']=='owned_scan_cells':
            horizontal,split,band,optimized_pair = balanced_partition(region,scan_configuration,
                homes[sector_index*2:sector_index*2+2],safety['navigation_boundary_margin_m'],clearance)
        for part_index in range(2):
            clipping = (box(-2000,-2000,split,2000) if part_index==0 else box(split,-2000,2000,2000)) if horizontal else (box(-2000,-2000,2000,split) if part_index==0 else box(-2000,split,2000,2000))
            part = region.intersection(clipping)
            flight_region = part.difference(band)
            safe = flight_region.buffer(-safety['navigation_boundary_margin_m'],join_style=2)
            if safe.is_empty:
                raise ValueError('隔离带留下的飞行区域过小')
            vehicle_index = sector_index*2+part_index
            scan_configuration = regional_scan(scan_profile,zone,footprint,zone_altitude)
            if optimized_pair is None:
                stations,uncovered = station_solver(part,safe,scan_configuration['station_footprint_m'],homes[vehicle_index])
            else:
                cells = optimized_pair[part_index]['cells']
                stations,uncovered = [cell['center'] for cell in cells],optimized_pair[part_index]['missing']
            vehicles.append(dict(id=f'uav_{vehicle_index+1}',zone=zone,label=names[zone],home=list(homes[vehicle_index]),position=[*homes[vehicle_index],0],clock=0,segments=[],stations=stations,work_altitude_m=zone_altitude,nominal_work_altitude_m=zone_altitude-safety['commanded_altitude_margin_m'],entry_altitude_m=0,coverage_uncovered_m2=uncovered,flight_region=mapping(flight_region),navigation_region=mapping(safe),search_region=mapping(part),_safe=safe))
            vehicles[-1]['scan_configuration'] = scan_configuration
            if optimized_pair is not None:
                vehicles[-1]['station_tasks'] = [dict(geometry=mapping(cell['target']),legs=cell['legs']) for cell in cells]
                vehicles[-1]['partition_predicted_s'] = optimized_pair[part_index]['predicted_s']
        partitions.append(dict(zone=zone,label=names[zone],geometry=mapping(region),buffer=mapping(region.intersection(band)),split_axis='east' if horizontal else 'north',split_m=split))
    if coverage_planning['strategy']=='owned_scan_cells':
        rescue_coverage(vehicles,legal.difference(launch))
        for vehicle in vehicles:
            cells = [dict(center=station,task=task) for station,task in zip(vehicle['stations'],vehicle['station_tasks'])]
            ordered = route_order(cells,vehicle['home'])
            vehicle['stations'] = [item['center'] for item in ordered]
            vehicle['station_tasks'] = [item['task'] for item in ordered]
    if proposed_isolation_end_corridor_m is not None:
        for partition in partitions:
            original = shape(partition['buffer'])
            partition['work_isolation_buffer'] = partition['buffer']
            partition['buffer'] = mapping(original.intersection(shape(partition['geometry']).buffer(
                -proposed_isolation_end_corridor_m,join_style=2)))
        forbidden = unary_union([shape(partition['buffer']) for partition in partitions])
        transit = transit.difference(forbidden.buffer(safety['transit_boundary_margin_m'],join_style=2))
    for vehicle in vehicles:
        vehicle['entry_altitude_m'] = entry_altitudes[vehicle['id']]
    vehicles_by_id = {vehicle['id']:vehicle for vehicle in vehicles}
    ordered = [vehicles_by_id[vehicle_id] for vehicle_id in entry_order]
    for vehicle in vehicles:
        add_segment(vehicle,[*vehicle['home'],2],'TAKEOFF',duration=2)
        add_segment(vehicle,vehicle['position'],'WAIT_RELEASE',duration=3)
    release = 5
    for entry_rank,vehicle in enumerate(ordered):
        release = 5+entry_rank*coordination['entry_release_interval_s']
        vehicle['entry_release_s'] = release
        if release>vehicle['clock']:
            add_segment(vehicle,vehicle['position'],'WAIT_RELEASE',duration=release-vehicle['clock'])
        altitude = vehicle['entry_altitude_m']
        add_segment(vehicle,[*vehicle['home'],altitude],'CLIMB',duration=(altitude-2)/2)
    gate_line = LineString([points[11],points[12]])
    gate_north = gate_line.parallel_offset(20.0,'left',join_style=2)
    gate_points = [
        list(gate_north.interpolate(0.08,normalized=True).coords[0]),
        list(gate_north.interpolate(0.92,normalized=True).coords[0]),
    ]
    for vehicle in ordered:
        altitude = vehicle['entry_altitude_m']
        work_altitude = vehicle['work_altitude_m']
        ingress_transit = transit.difference(unary_union([
            Point(other['home']).buffer(distance+0.5,resolution=4)
            for other in vehicles if other is not vehicle and abs(other['entry_altitude_m']-altitude)<8]))
        for delay in range(0,301,2):
            candidate = dict(vehicle,position=vehicle['position'][:],segments=vehicle['segments'][:])
            if delay:
                add_segment(candidate,candidate['position'],'WAIT_ENTRY_PATH',duration=delay)
            if candidate['zone']=='grass':
                gate = gate_points[0 if candidate['id']=='uav_5' else 1]
                navigate(candidate,[*gate,altitude],ingress_transit,'INGRESS')
            navigate(candidate,[*candidate['stations'][0],altitude],ingress_transit,'INGRESS')
            if altitude>work_altitude:
                add_segment(candidate,[*candidate['stations'][0],work_altitude],'DESCEND_TO_WORK',duration=(altitude-work_altitude)/2)
            else:
                add_segment(candidate,candidate['position'],'DESCEND_TO_WORK',duration=.01)
            trial = [candidate if other is vehicle else other for other in vehicles]
            for other in trial:
                other['ends'] = [segment['end'] for segment in other['segments']]
            separation = validate_separation(trial)
            if separation['passed']:
                vehicle.update(candidate)
                vehicle['nominal_entry_path_wait_s'] = delay
                break
        else:
            raise ValueError('No collision-checked nominal entry path for {}: {}'.format(vehicle['id'],separation['violations'][:3]))
    for vehicle in vehicles:
        work_altitude = vehicle['work_altitude_m']
        for station_index,station in enumerate(vehicle['stations']):
            try:
                path = route_inside(vehicle['position'][:2],station,vehicle['_safe'])
            except ValueError:
                altitude = vehicle['entry_altitude_m']
                add_segment(vehicle,[*vehicle['position'][:2],altitude],'REPOSITION_CLIMB',duration=(altitude-work_altitude)/2)
                navigate(vehicle,[*station,altitude],transit,'REPOSITION')
                if altitude>work_altitude:
                    add_segment(vehicle,[*station,work_altitude],'DESCEND_TO_WORK',duration=(altitude-work_altitude)/2)
                else:
                    add_segment(vehicle,vehicle['position'],'DESCEND_TO_WORK',duration=.01)
            else:
                for point in path:
                    add_segment(vehicle,[*point,work_altitude],'NAVIGATE')
            add_segment(vehicle,vehicle['position'],'STABILIZE',duration=2,station=station_index)
            first_east,first_north = vehicle['scan_configuration']['initial_look_offset_enu_m']
            previous_look = [station[0]+first_east,station[1]+first_north,0]
            if 'station_tasks' in vehicle:
                scan_legs = vehicle['station_tasks'][station_index]['legs']
            else:
                scan_legs = []
                for east,north in vehicle['scan_configuration']['offsets_enu_m']:
                    target = [station[0]+east,station[1]+north,0]
                    scan_legs.append(dict(origin=previous_look,target=target))
                    previous_look = target
            for observation_index,leg in enumerate(scan_legs):
                command_id = '{}:{}:{}'.format(vehicle['id'],station_index,observation_index)
                target = leg['target']
                motion = scan_motion(leg['origin'],target,vehicle['scan_configuration']['ground_scan_speed_m_s'])
                add_segment(vehicle,vehicle['position'],'SCAN',duration=motion['minimum_slew_s']+scan_profile['minimum_observation_s'],station=station_index,observation=observation_index,command_id=command_id,target=target,scan_motion=motion,
                    rapid_repoint='station_tasks' in vehicle,scan_footprint=leg.get('footprint'),evidence='synthetic_gimbal_result')
                previous_look = target
            if station_index==0:
                add_segment(vehicle,vehicle['position'],'CONFIRM_STATIC',duration=10,station=station_index,evidence='synthetic_detection')
            if station_index==1:
                add_segment(vehicle,vehicle['position'],'TRACK_MOVING',duration=120,station=station_index,evidence='synthetic_tracking')
    release = max(segment['end'] for vehicle in vehicles for segment in vehicle['segments']
        if segment['state'] in ('INGRESS','REPOSITION','DESCEND_TO_WORK'))+2
    for vehicle in sorted(vehicles,key=lambda item:item['clock']):
        work_altitude = vehicle['work_altitude_m']
        delays = range(0,1802,2) if coordination['return_policy']=='layered_reserved' else [max(0,release-vehicle['clock'])]
        for delay in delays:
            candidate = deepcopy(vehicle)
            add_segment(candidate,candidate['position'],'WAIT_RETURN_SLOT',duration=delay)
            altitude = candidate['entry_altitude_m']
            add_segment(candidate,[*candidate['position'][:2],altitude],'RETURN_CLIMB',duration=(altitude-work_altitude)/2)
            if candidate['zone']=='grass':
                gate = gate_points[0 if candidate['id']=='uav_5' else 1]
                navigate(candidate,[*gate,altitude],transit,'EGRESS')
            navigate(candidate,[*candidate['home'],altitude],transit,'EGRESS')
            add_segment(candidate,[*candidate['home'],2],'RETURN_DESCEND',duration=(altitude-2)/2)
            add_segment(candidate,[*candidate['home'],0],'LAND',duration=2)
            trial = [candidate if other['id']==vehicle['id'] else other for other in vehicles]
            for other in trial:
                other['ends'] = [segment['end'] for segment in other['segments']]
            if validate_separation(trial)['passed']:
                vehicle.update(candidate)
                vehicle['nominal_return_path_wait_s'] = delay
                release = vehicle['clock']+2
                break
        else:
            raise ValueError('No collision-checked nominal return slot for '+vehicle['id'])
    duration = max(vehicle['clock'] for vehicle in vehicles)
    for vehicle in vehicles:
        add_segment(vehicle,vehicle['position'],'COMPLETE',duration=duration-vehicle['clock'])
        vehicle['ends'] = [segment['end'] for segment in vehicle['segments']]
        vehicle.pop('_safe')
    validation = validate_separation(vehicles)
    if not validation['passed']:
        raise ValueError('连续轨迹分离校验失败: {}'.format(validation['violations'][:3]))
    deadline = 1500 if subject==1 else 1800
    covered = unary_union([box(east-vehicle['scan_configuration']['station_footprint_m']/2,north-vehicle['scan_configuration']['station_footprint_m']/2,east+vehicle['scan_configuration']['station_footprint_m']/2,north+vehicle['scan_configuration']['station_footprint_m']/2) for vehicle in vehicles for east,north in vehicle['stations']])
    if coverage_planning['strategy']=='owned_scan_cells':
        covered = unary_union([shape(segment['scan_footprint']) for vehicle in vehicles for segment in vehicle['segments'] if segment['state']=='SCAN'])
    uncovered = legal.difference(launch).difference(covered)
    coverage = dict(uncovered_m2=uncovered.area,uncovered=mapping(uncovered),ratio=1-uncovered.area/legal.difference(launch).area)
    forest_crossings = [dict(vehicle=vehicle['id'],state=segment['state'],start_s=segment['start']) for vehicle in vehicles for segment in vehicle['segments'] if segment['state'] in ('INGRESS','EGRESS','REPOSITION') and forest.intersects(LineString([segment['origin'][:2],segment['destination'][:2]]))]
    satellite_file = ROOT/'config/satellite.json'
    satellite = json.loads(satellite_file.read_text(encoding='utf-8')) if satellite_file.exists() else None
    plan = dict(schema='zhixin/fleet/v5',mode='deterministic_kinematic_simulation',subject=subject,datum=datum,flight_safety=safety,map_frame=simulation_frame(datum).descriptor(),perimeter=mapping(perimeter),forest=mapping(forest),subject3_exclusion=mapping(subject3_exclusion),subject3_reinclude=mapping(subject3_reinclude),launch=mapping(launch),legal=mapping(legal),partitions=partitions,vehicles=vehicles,duration_s=duration,deadline_s=deadline,within_time_budget=duration<=deadline,validation=validation,coverage=coverage,footprint_m=footprint,satellite=satellite,forest_crossings=forest_crossings,requires_forest_transit_permission=bool(forest_crossings),limitations=['扫描矩形为待标定模拟配置，不是附件给定或实机覆盖保证','平地AGL；缺少真实地形和遮挡模型','71m高程基准未确认；仿真单独假设MSL71m及大地水准面差0m，不能用于实机','云台和目标事件为合成，科二抛投待确认','不控制飞控；不是六机SITL物理闭环','四点区域按科三不搜索区处理，但三角形区域已重新加入搜索'])
    plan['observation_conditions'] = observation_conditions
    plan['airspace_policy'] = airspace_policy
    plan['requires_forest_transit_permission'] = bool(forest_crossings) and not airspace_policy['forest_transit_authorized']
    plan['limitations'] = [item for item in plan['limitations'] if not item.startswith('严格禁止跨树林')]
    plan['scan_motion_model'] = dict(maximum_ground_speed_m_s=scan_profile['maximum_ground_scan_speed_m_s'],
        dwell_s=scan_profile['minimum_observation_s'],order='regional_serpentine_field_tiling',
        first_point_origin='RAPID_REPOINT_TO_SWEEP_START',calibrated=False,
        stations=sum(len(vehicle['stations']) for vehicle in vehicles),
        maximum_station_scan_s=max(sum(segment['end']-segment['start'] for segment in vehicle['segments'] if segment['state']=='SCAN' and segment['station']==station)
            for vehicle in vehicles for station in range(len(vehicle['stations']))))
    plan['coordination'] = coordination
    plan['grass_entry_gates_enu'] = [list(point) for point in gate_points]
    plan['nominal_return_timing_model'] = coordination['return_policy']+'; nominal continuous-separation check, native arrival-driven reservations'
    plan['coverage_planning'] = coverage_planning
    plan['regional_scan_summary'] = [dict(vehicle_id=vehicle['id'],zone=vehicle['zone'],
        stations=len(vehicle['stations']),observations_per_station=None if 'station_tasks' in vehicle else len(vehicle['scan_configuration']['offsets_enu_m']),
        sweep_segments=sum(segment['state']=='SCAN' for segment in vehicle['segments']),
        instantaneous_frame_m=vehicle['scan_configuration']['instantaneous_frame_m'],
        work_altitude_m=vehicle['work_altitude_m'],
        station_footprint_m=vehicle['scan_configuration']['station_footprint_m'],
        scan_zoom=vehicle['scan_configuration']['scan_zoom'],
        ground_scan_speed_m_s=vehicle['scan_configuration']['ground_scan_speed_m_s'],
        entry_release_s=vehicle['entry_release_s'],nominal_entry_path_wait_s=vehicle['nominal_entry_path_wait_s'],
        entry_complete_s=next(segment['end'] for segment in vehicle['segments'] if segment['state']=='DESCEND_TO_WORK'),
        scan_s=sum(segment['end']-segment['start'] for segment in vehicle['segments'] if segment['state']=='SCAN'),
        search_complete_s=next(segment['start'] for segment in vehicle['segments'] if segment['state']=='WAIT_RETURN_SLOT'))
        for vehicle in vehicles]
    plan['limitations'].append('有效方形视场由手册水平FOV和16:9估算；斜视投影、真实SDK变焦倍率、识别能力及房区遮挡未标定')
    independent_uncovered = {vehicle['id']:vehicle['coverage_uncovered_m2'] for vehicle in vehicles}
    plan['coverage']['own_search_uncovered_m2'] = independent_uncovered
    if coverage_planning['strategy']=='owned_scan_cells':
        independent_uncovered = {vehicle['id']:shape(vehicle['search_region']).difference(unary_union(
            [shape(segment['scan_footprint']) for segment in vehicle['segments'] if segment['state']=='SCAN'])).area for vehicle in vehicles}
        plan['coverage']['own_search_uncovered_m2'] = independent_uncovered
    plan['coverage']['independent_vehicle_coverage_complete'] = all(area<.01 for area in independent_uncovered.values())
    if not plan['coverage']['independent_vehicle_coverage_complete']:
        plan['limitations'].append('全场扫描矩形并集覆盖不代表每机独立覆盖：'+', '.join(
            f'{vehicle_id}自身搜索残片未覆盖{area:.2f}平方米' for vehicle_id,area in independent_uncovered.items() if area>=.01))
    plan['isolation_airspace'] = audit_isolation(plan)
    plan['isolation_design'] = dict(kind='closed_bands' if proposed_isolation_end_corridor_m is None else 'proposed_end_corridors',
        end_corridor_width_m=proposed_isolation_end_corridor_m,user_approved=False)
    if proposed_isolation_end_corridor_m is not None:
        plan['limitations'].append('端部通道候选尚未获用户确认；保留原作业分区和20m工作隔离，仅截短全高度禁飞带端部，不得直接采用')
    return plan
