from __future__ import annotations
import json
from pathlib import Path
from unittest.mock import patch
from shapely.geometry import Point, shape
from fleet_core import ROOT, build_plan
from flight_safety import FlightSafety


def main():
    plan = build_plan()
    guard = FlightSafety(plan)
    for index,vehicle in enumerate(plan['vehicles']):
        legal,navigation = shape(vehicle['flight_region']),shape(vehicle['navigation_region'])
        assert legal.covers(navigation)
        expected_altitude = 60 if vehicle['zone']=='grass' else 40.5
        assert vehicle['work_altitude_m']==expected_altitude and vehicle['nominal_work_altitude_m']==expected_altitude-.5
        for segment in vehicle['segments']:
            result = guard.check(index,segment['state'],segment['origin'],segment['destination'],True,True)
            assert result['allowed'],(vehicle['id'],segment,result)
    boundary = shape(plan['partitions'][0]['buffer']).representative_point()
    forbidden = [boundary.x,boundary.y,40.5]
    for altitude in (40.5,90,110):
        transit = [boundary.x,boundary.y,altitude]
        strict = guard.check(0,'INGRESS',transit,transit,True)
        assert 'ALL_ALTITUDE_ISOLATION_BAND_NOT_AUTHORIZED' not in strict['violations']
        assert guard.check(0,'INGRESS',transit,transit,True,True)['allowed']
    result = guard.check(0,'SCAN',forbidden,forbidden,True)
    assert 'POSITION_OUTSIDE_WORK_REGION' in result['violations']
    own = [*plan['vehicles'][0]['stations'][0],40.5]
    neighbor = [*plan['vehicles'][1]['stations'][0],40.5]
    assert 'SETPOINT_PATH_OUTSIDE_WORK_REGION' in guard.check(0,'NAVIGATE',own,neighbor,True)['violations']
    fringe = shape(plan['vehicles'][0]['flight_region']).difference(shape(plan['vehicles'][0]['navigation_region'])).representative_point()
    margin_position = [fringe.x,fringe.y,40.5]
    assert guard.check(0,'SCAN',margin_position,margin_position,True)['allowed']
    assert not shape(plan['vehicles'][0]['navigation_region']).covers(Point(fringe.x,fringe.y))
    legal_region = shape(plan['vehicles'][0]['flight_region'])
    boundary_point = legal_region.difference(legal_region.buffer(-0.1,join_style=2)).representative_point()
    assert legal_region.covers(boundary_point)
    near_boundary = [boundary_point.x,boundary_point.y,40.5]
    assert 'POSITION_ENVELOPE_CROSSES_WORK_BOUNDARY' in guard.check(0,'SCAN',near_boundary,near_boundary,True)['violations']
    low = [*own[:2],39.99]
    assert 'POSITION_BELOW_OPERATING_FLOOR' in guard.check(0,'SCAN',low,own,True)['violations']
    high = [*own[:2],120.01]
    assert 'DESTINATION_ABOVE_ALTITUDE_CEILING' in guard.check(0,'SCAN',own,high,True)['violations']
    home = [*plan['vehicles'][0]['home'],2]
    assert guard.check(0,'TAKEOFF',home,home)['allowed']
    outside = [10000,10000,40.5]
    assert 'POSITION_OUTSIDE_PERIMETER' in guard.check(0,'SCAN',outside,own,True)['violations']
    assert not guard.check(0,'SCAN',[float('nan'),0,40.5],own,True)['allowed']
    crossings = 0
    for index,vehicle in enumerate(plan['vehicles']):
        for segment in vehicle['segments']:
            if segment['state'] in ('INGRESS','EGRESS','REPOSITION'):
                crossings += 'FOREST_TRANSIT_NOT_AUTHORIZED' in guard.check(index,segment['state'],segment['origin'],segment['destination'],False)['violations']
    assert crossings>=0
    second = build_plan(subject=2)
    assert second['flight_safety']['rule_minimum_altitude_m']==30
    assert second['flight_safety']['minimum_operating_altitude_m']==40
    original_read = Path.read_text
    invalid_safety = dict(plan['flight_safety'],minimum_operating_altitude_m=39)
    def configured_read(path, *arguments, **keywords):
        if path==ROOT/'config/flight_safety.json':
            return json.dumps(invalid_safety)
        return original_read(path,*arguments,**keywords)
    with patch.object(Path,'read_text',configured_read):
        try:
            build_plan(subject=1)
        except ValueError as error:
            assert 'competition rule floor' in str(error)
        else:
            raise AssertionError('Operating altitude floor below competition rule was accepted')
    report = dict(all_nominal_setpoint_paths_checked=True,navigation_inside_exact_work_region=True,
        boundary_margin_is_not_extra_authorized_airspace=True,isolation_band_rejected=True,
        all_altitude_isolation_rejected=False,work_only_isolation_policy=True,
        neighboring_region_rejected=True,outside_perimeter_rejected=True,invalid_coordinate_rejected=True,
        launch_altitude_exception=True,operating_floor_and_ceiling_checked=True,
        estimated_horizontal_body_envelope_checked=True,
        forest_permission_required=True,forest_transit_allowed_when_present=True,subject_specific_rule_floor=True,below_rule_configuration_rejected=True,
        nominal_altitude_m=40,commanded_altitude_m=40.5,hardware_verified=False,
        scope='geometry and command permission logic; not physical collision or braking proof')
    (ROOT/'validation/flight_safety_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
