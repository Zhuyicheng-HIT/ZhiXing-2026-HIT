from __future__ import annotations
import json
from unittest.mock import patch
from pathlib import Path
from shapely.geometry import Point, box, shape
from shapely.ops import unary_union
import fleet_core
from scan_protocol import ScanProtocol


def main():
    original_read = Path.read_text

    def configured_read(path, *args, **kwargs):
        if path==fleet_core.ROOT/'config/coverage_planning.json':
            return json.dumps(dict(strategy='greedy',hardware_authorized=False))
        return original_read(path,*args,**kwargs)

    with patch.object(Path,'read_text',configured_read):
        baseline = fleet_core.build_plan()
    candidate = fleet_core.build_plan()
    comparisons = []
    for old_vehicle,new_vehicle in zip(baseline['vehicles'],candidate['vehicles']):
        for field in ('home','flight_region','search_region','navigation_region','work_altitude_m'):
            assert old_vehicle[field]==new_vehicle[field]
        assert len(new_vehicle['stations'])<=len(old_vehicle['stations'])
        radius = candidate['footprint_m']/2
        covered = unary_union([box(east-radius,north-radius,east+radius,north+radius)
            for east,north in new_vehicle['stations']])
        remaining = shape(new_vehicle['search_region']).difference(covered).area
        assert remaining<=old_vehicle['coverage_uncovered_m2']+.01, (new_vehicle['id'],remaining,old_vehicle['coverage_uncovered_m2'])
        navigation = shape(new_vehicle['navigation_region'])
        outside_navigation = max(navigation.distance(Point(station)) for station in new_vehicle['stations'])
        assert outside_navigation<1e-7, (new_vehicle['id'],outside_navigation)
        assert all(shape(new_vehicle['flight_region']).covers(Point(station)) for station in new_vehicle['stations'])
        comparisons.append(dict(id=new_vehicle['id'],zone=new_vehicle['zone'],baseline_stations=len(old_vehicle['stations']),
            candidate_stations=len(new_vehicle['stations']),uncovered_m2=remaining,
            maximum_navigation_roundoff_distance_m=outside_navigation))
    assert candidate['coverage']['uncovered_m2']<=baseline['coverage']['uncovered_m2']+.01
    profile = json.loads((fleet_core.ROOT/'config/scan_profile.json').read_text(encoding='utf-8'))
    old_protocol,new_protocol = ScanProtocol(baseline,profile),ScanProtocol(candidate,profile)
    assert old_protocol.plan_revision!=new_protocol.plan_revision
    try:
        new_protocol.restore(old_protocol.checkpoint())
    except ValueError:
        pass
    else:
        raise AssertionError('Previous station plan checkpoint was accepted')
    for vehicle in candidate['vehicles']:
        for station_index,station in enumerate(vehicle['stations']):
            scans = [segment for segment in vehicle['segments'] if segment['state']=='SCAN' and segment['station']==station_index]
            assert len(scans)==9
            expected = [[station[0]+east,station[1]+north,0]
                for north in [-candidate['footprint_m']/3,0,candidate['footprint_m']/3]
                for east in [-candidate['footprint_m']/3,0,candidate['footprint_m']/3]]
            assert [segment['target'] for segment in scans]==expected
    report = dict(vehicles=comparisons,baseline_duration_s=baseline['duration_s'],
        candidate_duration_s=candidate['duration_s'],coverage_ratio=candidate['coverage']['ratio'],
        unchanged_footprint_m=candidate['footprint_m'],unchanged_observation_conditions=True,
        unchanged_work_regions_and_isolation=True,nine_point_row_major_order_preserved=True,
        changed_plan_revision=True,old_checkpoint_rejected=True,
        baseline_global_uncovered_m2=baseline['coverage']['uncovered_m2'],
        candidate_global_uncovered_m2=candidate['coverage']['uncovered_m2'],
        ideal_trajectory_validation=candidate['validation'],
        scope='nominal planar rectangle coverage; not actual optics or physical timing',hardware_verified=False)
    target = fleet_core.ROOT/'validation/sparse_coverage_comparison.json'
    target.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
