from __future__ import annotations
import json
import math
from copy import deepcopy
from shapely.geometry import Point, shape
from fleet_core import ROOT, build_plan
from flight_safety import FlightSafety
from isolation_airspace import require_isolation_permission
from scan_motion import scan_motion, scan_look_target, target_in_sector
from scan_protocol import ScanProtocol
from sim_server import FleetSimulation


def rejected(function):
    try:
        function()
    except ValueError:
        return
    raise AssertionError('Expected rejection')


def main():
    plan = build_plan()
    assert plan['airspace_policy']['isolation_scope']=='WORK_ONLY'
    assert plan['airspace_policy']['forest_transit_authorized']
    assert not plan['requires_forest_transit_permission']
    assert not require_isolation_permission(plan,{})
    assert plan['isolation_airspace']['crossings'] and not plan['isolation_airspace']['work_crossings']
    guard = FlightSafety(plan)
    checks = 0
    maximum_speed = 0
    for index,vehicle in enumerate(plan['vehicles']):
        for segment in vehicle['segments']:
            result = guard.check(index,segment['state'],segment['origin'],segment['destination'])
            assert result['allowed'],result
            checks += 1
            if segment['state']=='SCAN':
                motion = segment['scan_motion']
                assert motion['minimum_slew_s']+plan['observation_conditions']['minimum_observation_s']<=60
                if motion['minimum_slew_s']:
                    assert math.isclose(motion['distance_m']/motion['minimum_slew_s'],4)
                elapsed = motion['minimum_slew_s']/2
                measured = math.dist(scan_look_target(segment,elapsed),scan_look_target(segment,elapsed+.001))/.001
                assert measured<=4+1e-7
                maximum_speed = max(maximum_speed,measured)
                assert scan_look_target(segment,motion['minimum_slew_s']+1)==segment['target']
        for station in range(len(vehicle['stations'])):
            scans = [segment for segment in vehicle['segments'] if segment['state']=='SCAN' and segment['station']==station]
            assert [segment['observation'] for segment in scans]==list(range(len(scans)))
            for previous,current in zip(scans,scans[1:]):
                if current.get('rapid_repoint'):
                    assert 'station_tasks' in vehicle
                else:
                    assert previous['target']==current['scan_motion']['origin_enu_m']
    for first_index,first in enumerate(plan['vehicles']):
        for second in plan['vehicles'][first_index+1:]:
            assert shape(first['flight_region']).intersection(shape(second['flight_region'])).area<1e-6
    point = plan['vehicles'][0]['stations'][0]
    assert not guard.check(0,'INGRESS',[*point,110],[10000,10000,110])['allowed']
    assert not guard.check(0,'SCAN',[*plan['vehicles'][1]['stations'][0],40.5],[*plan['vehicles'][1]['stations'][0],40.5])['allowed']
    for speed in (0,-1,4.01,True,float('nan')):
        rejected(lambda:scan_motion([0,0,0],[1,1,0],speed))
    profile = json.loads((ROOT/'config/scan_profile.json').read_text(encoding='utf-8'))
    protocol = ScanProtocol(plan,profile)
    command_id = next(iter(protocol.observations))
    assert protocol.issue('uav_1',command_id,'test')['scan_motion']['minimum_slew_s']>0
    changed = deepcopy(plan)
    changed['airspace_policy']['target_exit_action']='OTHER_POLICY'
    assert ScanProtocol(changed,profile).plan_revision!=protocol.plan_revision
    fleet = FleetSimulation()
    vehicle = fleet.plan['vehicles'][0]
    tracking = next(segment for segment in vehicle['segments'] if segment['state']=='TRACK_MOVING')
    fleet.stamp = tracking['start']+.1
    inside = [*vehicle['stations'][tracking['station']],0]
    assert target_in_sector(vehicle,inside)['action']=='TRACK_WITHIN_SECTOR'
    cursor_before = fleet.stamp
    acknowledgement = fleet.control(dict(action='target_update',vehicle_id=vehicle['id'],target_enu_m=[10000,10000,0]))['target_ack']
    assert acknowledgement['action']=='ABANDON_NO_HANDOFF' and not acknowledgement['handoff']
    assert fleet.stamp==cursor_before and (vehicle['id'],tracking['station']) in fleet.abandoned_tracking
    fleet.control(dict(action='target_update',vehicle_id=vehicle['id'],target_enu_m=inside))
    assert fleet.target_updates[vehicle['id']]['action']=='ABANDON_NO_HANDOFF'
    subject2 = build_plan(subject=2)
    report = dict(passed=True,scope='planning, nominal ground-look interpolation and target-exit decision logic; not physical camera speed or native flight',
        user_confirmed_airspace_policy=plan['airspace_policy'],all_nominal_flight_segments_checked=checks,
        work_regions_do_not_overlap=True,transit_within_perimeter=True,forest_transit_without_override=True,
        regional_serpentine_scan=True,ground_scan_speed_max_m_s=maximum_speed,
        station_scan_s=plan['scan_motion_model']['maximum_station_scan_s'],
        subject1_nominal_duration_s=plan['duration_s'],subject2_nominal_duration_s=subject2['duration_s'],
        subject1_deadline_met=plan['within_time_budget'],subject2_deadline_met=subject2['within_time_budget'],
        target_exit_abandons_without_handoff=True,target_exit_never_authorizes_aircraft_crossing=True,
        hardware_verified=False)
    (ROOT/'validation/work_only_scan_speed_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
