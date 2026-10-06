from __future__ import annotations
import json
import math
from shapely.geometry import Point, LineString, box, shape
from shapely.ops import unary_union
from fleet_core import ROOT, build_plan
from scan_protocol import ScanProtocol


def metrics(plan):
    search = shape(plan['legal']).difference(shape(plan['launch']))
    station_fields,edge_routes = [],[]
    edge_stations,work_edge_stations = 0,0
    by_vehicle = []
    for vehicle in plan['vehicles']:
        config = vehicle['scan_configuration']
        width,height = config.get('instantaneous_frame_m',[config.get('instantaneous_square_m'),config.get('instantaneous_square_m')])
        radius_x,radius_y = width/2,height/2
        edge_stations += sum(shape(plan['perimeter']).boundary.distance(Point(station))<10 for station in vehicle['stations'])
        work_boundary = vehicle.get('flight_region') or vehicle.get('navigation_region') or vehicle.get('search_region')
        if work_boundary is not None:
            work_geometry = shape(work_boundary)
            if work_geometry.boundary is not None:
                work_edge_stations += sum(work_geometry.boundary.distance(Point(station))<10 for station in vehicle['stations'])
        for station_index in range(len(vehicle['stations'])):
            footprints = []
            for segment in vehicle['segments']:
                if segment['state']=='SCAN' and segment['station']==station_index:
                    motion = segment['scan_motion']
                    origin,target = motion['origin_enu_m'],motion['destination_enu_m']
                    footprints.append(box(min(origin[0],target[0])-radius_x,min(origin[1],target[1])-radius_y,
                        max(origin[0],target[0])+radius_x,max(origin[1],target[1])+radius_y))
            station_fields.append(unary_union(footprints).intersection(search))
        for segment in vehicle['segments']:
            if segment['state']=='NAVIGATE':
                line = LineString([segment['origin'][:2],segment['destination'][:2]])
                if shape(plan['perimeter']).boundary.distance(line)<10:
                    edge_routes.append(line.length)
        scan_s = sum(segment['end']-segment['start'] for segment in vehicle['segments'] if segment['state']=='SCAN')
        by_vehicle.append(dict(id=vehicle['id'],zone=vehicle['zone'],stations=len(vehicle['stations']),scan_s=scan_s))
    covered = unary_union(station_fields)
    return dict(stations=sum(len(vehicle['stations']) for vehicle in plan['vehicles']),
        sweep_segments=sum(segment['state']=='SCAN' for vehicle in plan['vehicles'] for segment in vehicle['segments']),
        duration_s=plan['duration_s'],uncovered_m2=search.difference(covered).area,
        station_scan_area_equivalent_ratio=sum(field.area for field in station_fields)/search.area,
        repeated_station_scan_area_equivalent_m2=sum(field.area for field in station_fields)-covered.area,
        stations_within_10m_perimeter=edge_stations,stations_within_10m_work_boundary=work_edge_stations,
        navigate_segment_length_touching_10m_perimeter_m=sum(edge_routes),vehicles=by_vehicle)


def main():
    baseline = json.loads((ROOT/'validation/pre_optimized_plan.json').read_text(encoding='utf-8'))
    current = build_plan()
    profile = json.loads((ROOT/'config/scan_profile.json').read_text(encoding='utf-8'))
    protocol = ScanProtocol(current,profile)
    assert len(protocol.observations)==sum(segment['state']=='SCAN' for vehicle in current['vehicles'] for segment in vehicle['segments'])
    targets = [shape(task['geometry']) for vehicle in current['vehicles'] for task in vehicle['station_tasks']]
    assigned = unary_union(targets)
    assert sum(target.area for target in targets)-assigned.area<.01
    assert shape(current['legal']).difference(shape(current['launch'])).difference(assigned).area<.01
    for vehicle in current['vehicles']:
        for station,task in zip(vehicle['stations'],vehicle['station_tasks']):
            assert shape(vehicle['navigation_region']).buffer(1e-7).covers(Point(station))
            radius = vehicle['scan_configuration']['station_footprint_m']/2
            assert shape(task['geometry']).difference(box(station[0]-radius-.001,station[1]-radius-.001,
                station[0]+radius+.001,station[1]+radius+.001)).area<.01
    old,new = metrics(baseline),metrics(current)
    assert new['uncovered_m2']<.01
    assert new['station_scan_area_equivalent_ratio']<old['station_scan_area_equivalent_ratio']
    assert new['sweep_segments']<old['sweep_segments']
    for zone in ('takeoff','house','grass'):
        times = [row['search_complete_s'] for row in current['regional_scan_summary'] if row['zone']==zone]
        assert max(times)/min(times)<1.4
    assert current['validation']['passed']
    report = dict(passed=True,scope='flat-ground nominal rectangular coverage and total search timing; not real camera or native flight',
        baseline=old,current=new,owned_target_overlap_m2=sum(target.area for target in targets)-assigned.area,
        duplicate_area_metric='Sum of per-station swept unions clipped to search minus global union; excludes intentional within-station row overlap',
        same_scan_speed_m_s=4,straight_pair_boundaries=True,horizontal_work_gap_m=20,
        pair_search_completion_ratio_limit=1.4,pure_scan_time_is_not_total_workload=True,
        nominal_return_model=current['nominal_return_timing_model'],native_flight_verified=False)
    (ROOT/'validation/owned_coverage_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
