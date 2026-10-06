from __future__ import annotations
import json
from shapely.geometry import shape
from fleet_core import ROOT, build_plan
from flight_safety import FlightSafety
from isolation_airspace import require_isolation_permission


def main():
    baseline = build_plan()
    candidate = build_plan(proposed_isolation_end_corridor_m=10)
    assert not candidate['isolation_airspace']['crossings']
    assert candidate['isolation_airspace']['all_altitude_isolation_respected_by_plan']
    assert candidate['isolation_airspace']['alternatives'][1]['all_stations_reachable']
    guard = FlightSafety(candidate)
    for index,(original,vehicle) in enumerate(zip(baseline['vehicles'],candidate['vehicles'])):
        assert original['stations']==vehicle['stations']
        assert original['flight_region']==vehicle['flight_region']
        assert original['navigation_region']==vehicle['navigation_region']
        assert [segment['command_id'] for segment in original['segments'] if segment['state']=='SCAN']==[
            segment['command_id'] for segment in vehicle['segments'] if segment['state']=='SCAN']
        for segment in vehicle['segments']:
            safety = guard.check(index,segment['state'],segment['origin'],segment['destination'],True)
            assert safety['allowed'],(vehicle['id'],segment,safety)
    for original,partition in zip(baseline['partitions'],candidate['partitions']):
        assert original['buffer']==partition['work_isolation_buffer']
        assert shape(original['buffer']).covers(shape(partition['buffer']))
    try:
        require_isolation_permission(candidate,{'isolation_transit_simulation':True})
    except ValueError as error:
        assert '用户确认' in str(error)
    else:
        raise AssertionError('Unapproved candidate unexpectedly executable')
    report = dict(passed=True,requires_user_approval=True,active_default_changed=False,
        end_corridor_width_m=10,original_work_regions_preserved=True,original_scan_order_preserved=True,
        command_segments_avoid_all_altitude_isolation=True,estimated_body_envelope_checked=True,
        no_isolation_override_used_in_path_check=True,unapproved_candidate_start_rejected=True,
        forest_permission_still_required=candidate['requires_forest_transit_permission'],
        station_counts={vehicle['id']:len(vehicle['stations']) for vehicle in candidate['vehicles']},
        nominal_duration_s=candidate['duration_s'],native_flight_verified=False,hardware_verified=False,
        scope='candidate geometry and nominal command paths only; not approved, native flown or braking proof')
    (ROOT/'missions/fleet_plan_isolation_corridor_candidate.json').write_text(json.dumps(candidate,ensure_ascii=False,indent=2),encoding='utf-8')
    (ROOT/'validation/isolation_corridor_candidate_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
