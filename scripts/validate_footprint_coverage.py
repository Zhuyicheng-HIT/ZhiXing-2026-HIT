from __future__ import annotations
import json
import math
from shapely.geometry import LineString, Point, box, shape
from shapely.ops import unary_union
import fleet_core
from coverage_candidates import interior_stations, sparse_stations


def footprints(stations, width):
    radius = width/2
    return unary_union([box(east-radius,north-radius,east+radius,north+radius)
        for east,north in stations])


def metrics(plan):
    boundary = shape(plan['perimeter']).boundary
    edge_routes = []
    edge_stations = 0
    uncovered_by_vehicle = {}
    for vehicle in plan['vehicles']:
        search = shape(vehicle['search_region'])
        navigation = shape(vehicle['navigation_region'])
        missing = search.difference(footprints(vehicle['stations'],vehicle['scan_configuration']['station_footprint_m'])).area
        uncovered_by_vehicle[vehicle['id']] = missing
        assert all(navigation.distance(Point(point))<1e-7 for point in vehicle['stations'])
        edge_stations += sum(boundary.distance(Point(point))<10 for point in vehicle['stations'])
        for segment in vehicle['segments']:
            if segment['state']=='NAVIGATE':
                line = LineString([segment['origin'][:2],segment['destination'][:2]])
                assert navigation.buffer(1e-7).covers(line)
                if boundary.distance(line)<10:
                    edge_routes.append(line.length)
    return dict(stations=sum(len(vehicle['stations']) for vehicle in plan['vehicles']),
        coverage_ratio=plan['coverage']['ratio'],duration_s=plan['duration_s'],
        own_search_uncovered_m2=uncovered_by_vehicle,
        stations_within_10m_of_perimeter=edge_stations,
        total_length_of_navigate_segments_touching_10m_edge_band_m=sum(edge_routes))


def main():
    square = box(0,0,80,80)
    assert square.difference(footprints([[40,40]],80)).area==0
    assert square.boundary.distance(Point(40,40))==40
    target = box(0,0,60,60)
    safe = target.buffer(-4)
    moved = interior_stations(target,safe,80,[[20,20]],28)
    assert target.difference(footprints(moved,80)).area<.01
    assert safe.boundary.distance(Point(moved[0]))>safe.boundary.distance(Point(20,20))
    plan = fleet_core.build_plan()
    if plan['coverage_planning']['strategy']=='owned_scan_cells':
        baseline = metrics(json.loads((fleet_core.ROOT/'validation/pre_optimized_plan.json').read_text(encoding='utf-8')))
    else:
        original = fleet_core.stations_for
        try:
            fleet_core.stations_for = lambda part,safe,footprint,home: sparse_stations(
                part,safe,footprint,home,fleet_core.greedy_stations,fleet_core.order_stations)
            baseline = metrics(fleet_core.build_plan())
        finally:
            fleet_core.stations_for = original
        assert metrics(plan)['stations']<=baseline['stations']
    current = metrics(plan)
    assert math.isclose(current['coverage_ratio'],1,abs_tol=1e-8)
    key = 'total_length_of_navigate_segments_touching_10m_edge_band_m'
    assert current[key]<baseline[key]
    report = dict(scope='nominal_flat_ground_geometry_only',hardware_verified=False,
        independent_vehicle_coverage_complete=all(area<.01 for area in current['own_search_uncovered_m2'].values()),
        limitation='House-region thin remnants depend on neighboring scan footprints; not independent per-vehicle complete coverage',
        edge_metric='Whole NAVIGATE segment length when any point is within 10m of perimeter, not clipped in-band length',
        baseline=baseline,current=current,edge_segment_length_reduction_ratio=1-current[key]/baseline[key])
    output = fleet_core.ROOT/'validation/footprint_coverage_report.json'
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
