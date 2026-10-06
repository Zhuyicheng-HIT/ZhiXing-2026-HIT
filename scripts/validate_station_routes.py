from __future__ import annotations
import json
from shapely.geometry import Point, box, shape
from shapely.ops import unary_union
from fleet_core import ROOT, build_plan, polygons


def main():
    report = dict(scope='规划几何与连通性，非飞控或相机验收',configurations=[])
    for footprint in (80,):
        plan = build_plan(footprint=footprint)
        vehicles = []
        for vehicle in plan['vehicles']:
            components = polygons(shape(vehicle['navigation_region']))
            visited = []
            for station in vehicle['stations']:
                matches = [index for index,component in enumerate(components) if component.buffer(.001).covers(Point(station))]
                assert len(matches)==1
                if not visited or visited[-1]!=matches[0]:
                    visited.append(matches[0])
            assert len(visited)==len(set(visited))
            repositions = sum(segment['state']=='REPOSITION_CLIMB' for segment in vehicle['segments'])
            assert repositions==max(0,len(visited)-1)
            coverage = unary_union([shape(segment['scan_footprint']) for segment in vehicle['segments']
                if segment['state']=='SCAN' and segment.get('scan_footprint')])
            uncovered = shape(vehicle['search_region']).difference(coverage).area
            assert uncovered<=vehicle['coverage_uncovered_m2']+.01
            vehicles.append(dict(id=vehicle['id'],stations=len(vehicle['stations']),visited_components=visited,
                repositions=repositions,uncovered_m2=uncovered))
        assert plan['validation']['passed']
        if footprint==80:
            assert plan['coverage']['ratio']>.99999
        report['configurations'].append(dict(footprint_m=footprint,duration_s=plan['duration_s'],
            global_coverage_ratio=plan['coverage']['ratio'],vehicles=vehicles))
    (ROOT/'validation/station_routes_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
