from __future__ import annotations
import json
import math
import random
import time
from copy import deepcopy
from types import SimpleNamespace
from fleet_core import ROOT, build_plan
from path_reservations import segment_distance
from waypoint_executor import WaypointExecutor
from scan_protocol import ScanProtocol


def main():
    examples = [(([0,0,0],[10,0,0],[5,-5,0],[5,5,0]),0),
        (([0,0,10],[10,0,10],[5,-5,0],[5,5,0]),10),
        (([0,0,0],[10,0,0],[20,0,0],[30,0,0]),10),
        (([0,0,0],[0,0,0],[3,4,0],[3,4,0]),5),
        (([0,0,0],[10,0,0],[5,3,0],[5,3,0]),3),
        (([0,0,0],[10,0,0],[2,4,0],[8,4,0]),4),
        (([0,0,0],[1e8,0,0],[0,-20,0],[1e8,20,0]),0),
        (([0,0,0],[1e8,0,0],[0,-20,13],[1e8,20,13]),13)]
    for points,expected in examples:
        assert math.isclose(segment_distance(*points),expected,abs_tol=1e-9)
        assert math.isclose(segment_distance(*points[2:],*points[:2]),expected,abs_tol=1e-9)
    generator = random.Random(2026)
    for case in range(100):
        points = [[generator.uniform(-200,200) for axis in range(3)] for endpoint in range(4)]
        distance = segment_distance(*points)
        assert math.isclose(distance,segment_distance(points[1],points[0],points[3],points[2]),abs_tol=1e-8)
        for sample in range(40):
            first_fraction,second_fraction = generator.random(),generator.random()
            first_point = [start+first_fraction*(end-start) for start,end in zip(points[0],points[1])]
            second_point = [start+second_fraction*(end-start) for start,end in zip(points[2],points[3])]
            assert distance<=math.dist(first_point,second_point)+1e-8
    plan = build_plan()
    profile = json.loads((ROOT/'config/scan_profile.json').read_text(encoding='utf-8'))
    protocol = ScanProtocol(plan,profile)
    old_policy = deepcopy(plan)
    old_policy['coordination']['entry_policy'] = 'exclusive'
    old_protocol = ScanProtocol(old_policy,profile)
    assert protocol.plan_revision!=old_protocol.plan_revision
    try:
        protocol.restore(old_protocol.checkpoint())
    except ValueError:
        pass
    else:
        raise AssertionError('A checkpoint from the old coordination policy was accepted')
    commands = []
    events = []
    now = time.monotonic()
    fleet = SimpleNamespace(plan=plan,gimbal_geometry=json.loads((ROOT/'config/gimbal_geometry.json').read_text(encoding='utf-8')),
        telemetry=[dict(position=[*vehicle['home'],2],boot_ms=1000,received=True,last_seen=now,armed=True,mode='GUIDED',speed_m_s=0)
            for vehicle in plan['vehicles']],
        emit=lambda *args,**kwargs:events.append(args),setpoint=lambda *args,**kwargs:commands.append(args),
        pending_by_vehicle={},completed=set(),faults={},running=True)
    executor = WaypointExecutor(fleet)
    executor.begin()
    first,second = executor.entry_order[:2]
    assert not executor.allowed(first)
    for item in fleet.telemetry:
        item['boot_ms'] += 4000
    assert executor.allowed(first) and executor.entry_released[first]
    assert executor.reserve_path(first,executor.segment(first)['destination'])
    assert not executor.allowed(second)
    for item in fleet.telemetry:
        item['boot_ms'] += 1000
    assert executor.allowed(second)
    assert executor.segment(first)['state']=='CLIMB'
    fleet.telemetry[first]['position'] = executor.segment(first)['destination'][:]
    executor.advance_cursor(first)
    assert executor.allowed(second)
    assert executor.segment(first)['state']=='INGRESS'
    assert executor.reserve_path(first,executor.segment(first)['destination'])
    assert executor.allowed(second)
    assert not executor.waiting_for_entry(first) and not executor.waiting_for_entry(second)
    assert executor.entry_released[first] and executor.entry_released[second] and not executor.entered[first]
    assert executor.reservation_audit['maximum_parallel_entries']==2
    saved = executor.cursors[:]
    executor.reservation_keys = [None]*6
    executor.reservation_destinations = [None]*6
    fleet.telemetry[first]['position'] = [0,0,110]
    executor.reservation_destinations[first] = [0,0,40.5]
    fleet.telemetry[second]['position'] = [-20,0,100]
    for other_index,item in enumerate(fleet.telemetry):
        if other_index not in (first,second):
            item['position'] = [1000+other_index*50,1000,40.5]
    assert not executor.reserve_path(second,[20,0,100])
    assert executor.cursors==saved and executor.reservation_waits[second]['vehicle_id']==plan['vehicles'][first]['id']
    assert commands[-1][1]==[-20,0,100]
    fleet.telemetry[first]['position'] = [0,0,40.5]
    executor.reservation_destinations[first] = [0,0,40.5]
    assert executor.reserve_path(second,[20,0,100])
    assert executor.cursors==saved and executor.reservation_waits[second] is None
    executor.hold_all()
    assert executor.cursors==saved and all(key is None for key in executor.reservation_keys)
    assert executor.entry_released[first] and executor.entry_released[second]
    executor.holds = None
    executor.hold_yaws = None
    executor.entered = [True]*6
    executor.entered[second] = False
    reposition = next((index,cursor) for index,vehicle in enumerate(plan['vehicles'])
        for cursor,segment in enumerate(vehicle['segments']) if segment['state']=='REPOSITION_CLIMB')
    reposition_owner,reposition_cursor = reposition
    executor.cursors[reposition_owner] = reposition_cursor
    executor.entered[reposition_owner] = True
    executor.entered[(reposition_owner+1)%6] = False
    assert not executor.allowed(reposition_owner)
    report = dict(segment_distance_analytical_cases=True,random_reversal_and_sample_bounds=True,hover_required=True,
        two_second_release_without_waiting_for_higher_ingress=True,parallel_entry_before_work_arrival=True,
        crossing_descent_blocks_lower_route=True,cleared_reservation_resumes_same_cursor=True,
        pause_clears_grants_without_losing_release_state=True,reposition_waits_for_all_entries=True,
        released_ingress_is_not_misclassified_as_queue=True,
        coordination_policy_change_rejects_old_checkpoint=True,
        minimum_reserved_path_distance_m=executor.reservation_distance,
        scope='command scheduler logic; not physical collision validation',hardware_verified=False)
    (ROOT/'validation/entry_reservation_logic_report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
