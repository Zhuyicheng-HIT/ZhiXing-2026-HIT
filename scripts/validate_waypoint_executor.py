from __future__ import annotations
import json
import time
import math
from types import SimpleNamespace
from fleet_core import ROOT, build_plan
from waypoint_executor import WaypointExecutor
from sim_server import FleetSimulation
from scan_protocol import ScanProtocol
from scan_execution import ScanExecution


def main():
    plan = build_plan()
    plan['coordination']['entry_policy'] = 'exclusive'
    plan['coordination']['return_policy'] = 'exclusive'
    commands = []
    yaw_commands = []
    flight_commands = []
    fleet = SimpleNamespace(plan=plan,running=True,faults={},pending=None,completed=set(),synthetic=False,forest_permission=True,isolation_transit_simulation=True,
        epoch='executor-test',stamp=0,emit=lambda *args,**kwargs:None,checkpoint=lambda:None,
        setpoint=lambda index,position,yaw_mission_rad=None:(commands.append((index,list(position))),yaw_commands.append((index,yaw_mission_rad))),command=lambda *args:flight_commands.append(args))
    fleet.telemetry = [dict(position=[*vehicle['home'],2],boot_ms=1000,received=True,last_seen=time.monotonic(),
        armed=True,mode='GUIDED',speed_m_s=0,
        attitude=dict(quaternion_mission_from_body_flu_xyzw=[0,0,0,1],received_monotonic_s=time.monotonic()),
        raw_attitude_ned_frd=dict(angular_speed_rad_s=0)) for vehicle in plan['vehicles']]
    fleet.scan_protocol = ScanProtocol(plan,json.loads((ROOT/'config/scan_profile.json').read_text(encoding='utf-8')))
    fleet.scan_execution = ScanExecution()
    fleet.pending_by_vehicle = {}
    fleet.scan_execution_by_vehicle = {}
    fleet.concurrent_observations = False
    fleet.sync_pending_aliases = lambda: FleetSimulation.sync_pending_aliases(fleet)
    fleet.remove_pending = lambda vehicle_id: FleetSimulation.remove_pending(fleet,vehicle_id)
    fleet.gimbal_geometry = json.loads((ROOT/'config/gimbal_geometry.json').read_text(encoding='utf-8'))
    fleet.pointing_display = lambda target,position,attitude: FleetSimulation.pointing_display(fleet,target,position,attitude)
    fleet.prepare_scan = lambda vehicle_id,command_id,actual_position=None: FleetSimulation.prepare_scan(fleet,vehicle_id,command_id,actual_position)
    executor = WaypointExecutor(fleet)
    executor.begin()
    first = executor.entry_order[0]
    original = executor.cursors[:]
    for item in fleet.telemetry:
        item['boot_ms'] += 4000
    executor.tick()
    assert executor.cursors==original
    assert executor.transit_owner==first
    assert (first,executor.segment(first)['destination']) in commands
    fleet.telemetry[first]['position'] = executor.segment(first)['destination'][:]
    fleet.telemetry[first]['speed_m_s'] = 1.0
    executor.tick()
    assert executor.cursors==original
    fleet.telemetry[first]['speed_m_s'] = 0
    executor.tick()
    assert executor.cursors[first]==original[first]+1
    while not executor.entered[first]:
        fleet.telemetry[first]['position'] = executor.segment(first)['destination'][:]
        executor.tick()
    assert executor.entry_cursor==1
    assert executor.transit_owner in (None,executor.entry_order[1])
    executor.cursors[first] = next(index for index,segment in enumerate(plan['vehicles'][first]['segments']) if segment['state']=='SCAN')
    fleet.telemetry[first]['position'] = executor.segment(first)['destination'][:]
    scan_cursor = executor.cursors[first]
    fleet.telemetry[first]['attitude']['received_monotonic_s'] = time.monotonic()-3
    executor.tick()
    assert executor.scan_wait_state[first]=='SCAN_WAIT_ATTITUDE'
    assert fleet.pending is None and executor.scan_alignment[first] is None
    fleet.telemetry[first]['attitude']['received_monotonic_s'] = time.monotonic()
    executor.tick()
    assert fleet.pending is None and executor.cursors[first]==scan_cursor
    goal = executor.scan_alignment[first]['heading_mission_rad']
    latched_station_heading = json.loads(json.dumps(executor.station_heading[first]))
    original_pointing_display = fleet.pointing_display
    fleet.pointing_display = lambda *arguments:dict(valid=True,executable=dict(feasible=False))
    fleet.telemetry[first]['boot_ms'] += 1000
    executor.tick()
    assert executor.scan_wait_state[first]=='SCAN_WAIT_POINTING'
    assert fleet.pending is None and executor.elapsed[first]==0
    fleet.pointing_display = original_pointing_display
    fleet.telemetry[first]['attitude']['quaternion_mission_from_body_flu_xyzw'] = [0,0,math.sin(goal/2),math.cos(goal/2)]
    fleet.telemetry[first]['raw_attitude_ned_frd']['angular_speed_rad_s'] = math.radians(20)
    fleet.telemetry[first]['boot_ms'] += 1000
    executor.tick()
    assert fleet.pending is None
    fleet.telemetry[first]['raw_attitude_ned_frd']['angular_speed_rad_s'] = 0
    fleet.telemetry[first]['boot_ms'] += 200
    executor.tick()
    assert fleet.pending is None
    fleet.running = False
    executor.tick()
    assert executor.scan_alignment[first]['stable_s']==0
    assert executor.station_heading[first]==latched_station_heading
    fleet.running = True
    fleet.telemetry[first]['boot_ms'] += 1000
    executor.tick()
    assert fleet.pending['command_id']==executor.segment(first)['command_id']
    frozen = executor.cursors[:]
    executor.tick()
    assert executor.cursors==frozen
    held = [position[:] for position in executor.holds]
    fleet.telemetry[first]['position'][0] += .1
    executor.tick()
    assert executor.holds==held
    fleet.completed.add(fleet.pending['command_id'])
    fleet.remove_pending(plan['vehicles'][first]['id'])
    executor.tick()
    assert executor.cursors[first]==frozen[first]+1
    fleet.telemetry[first]['boot_ms'] += 1000
    executor.tick()
    assert executor.scan_alignment[first]['heading_mission_rad']==goal
    assert executor.scan_audit['station_heading_reuses']==1
    assert executor.station_heading[first]['remaining']==latched_station_heading['remaining']-1
    fleet.faults['uav_1'] = 'gimbal_link_lost'
    frozen = executor.cursors[:]
    executor.tick()
    assert executor.cursors==frozen
    fleet.faults.clear()
    fleet.running = False
    executor.tick()
    assert executor.cursors==frozen
    fleet.running = True
    executor.entered = [True]*6
    executor.transit_owner = None
    for index,target in enumerate(executor.holds):
        fleet.telemetry[index].update(position=target[:],speed_m_s=0)
    assert executor.release_settled_hold()
    waiting = next(index for index,segment in enumerate(plan['vehicles'][first]['segments']) if segment['state']=='WAIT_RETURN_SLOT')
    executor.cursors[first] = waiting
    fleet.telemetry[first]['position'] = executor.segment(first)['destination'][:]
    second = (first+1)%6
    executor.entered[second] = False
    executor.tick()
    assert executor.cursors[first]==waiting
    assert executor.return_queue==[first]
    executor.entered[second] = True
    executor.transit_owner = None
    executor.tick()
    assert executor.cursors[first]==waiting+1
    assert not all(executor.search_done)
    assert executor.transit_owner==first
    executor.cursors[second] = next(index for index,segment in enumerate(plan['vehicles'][second]['segments']) if segment['state']=='WAIT_RETURN_SLOT')
    fleet.telemetry[second]['position'] = executor.segment(second)['destination'][:]
    second_cursor = executor.cursors[second]
    executor.tick()
    assert executor.cursors[second]==second_cursor
    assert executor.return_queue==[first,second]
    executor.cursors[first] = next(index for index,segment in enumerate(plan['vehicles'][first]['segments']) if segment['state']=='LAND')
    fleet.telemetry[first].update(position=[*plan['vehicles'][first]['home'],0],armed=False)
    executor.tick()
    assert executor.return_cursor==1
    assert executor.return_queue==[second]
    executor.tick()
    assert executor.transit_owner==second
    assert not any(command[1]==20 for command in flight_commands)
    fleet.concurrent_observations = True
    executor.entered = [True]*6
    executor.entry_cursor = 6
    executor.transit_owner = None
    executor.holds = None
    for index,vehicle in enumerate(plan['vehicles']):
        executor.cursors[index] = next(cursor for cursor,segment in enumerate(vehicle['segments']) if segment['state']=='STABILIZE')
        fleet.telemetry[index].update(position=executor.segment(index)['destination'][:],armed=True,
            last_seen=time.monotonic(),boot_ms=fleet.telemetry[index]['boot_ms']+1000)
        executor.elapsed[index] = 100
    waiting_owner = plan['vehicles'][first]['id']
    waiting_id = next(segment['command_id'] for segment in plan['vehicles'][first]['segments'] if segment['state']=='SCAN')
    fleet.prepare_scan(waiting_owner,waiting_id,fleet.telemetry[first]['position'])
    frozen = executor.cursors[:]
    executor.tick()
    assert executor.cursors[first]==frozen[first]
    assert all(executor.cursors[index]==frozen[index]+1 for index in range(6) if index!=first)
    held = executor.observation_holds[first]
    fleet.telemetry[first]['position'][0] += .1
    fleet.faults['coordination'] = 'test_group_hold'
    frozen = executor.cursors[:]
    executor.tick()
    assert executor.cursors==frozen
    assert executor.observation_holds[first]==held
    assert waiting_owner in fleet.pending_by_vehicle
    fleet.faults.clear()
    destination = executor.segment(first)['destination']
    original_height = destination[2]
    destination[2] = 39.99
    frozen = executor.cursors[:]
    executor.tick()
    assert 'flight_safety' in fleet.faults and executor.cursors==frozen
    assert waiting_owner in fleet.pending_by_vehicle
    assert executor.flight_safety.rejections==1
    destination[2] = original_height
    assert not any(command[1]==20 for command in flight_commands)
    fleet.faults.clear()
    fleet.pending_by_vehicle.clear()
    fleet.sync_pending_aliases()
    fleet.telemetry[first]['attitude'].update(quaternion_mission_from_body_flu_xyzw=[0,0,0,1],received_monotonic_s=time.monotonic())
    fleet.telemetry[first]['raw_attitude_ned_frd']['angular_speed_rad_s'] = 0
    executor.cursors[first] = next(cursor for cursor,segment in enumerate(plan['vehicles'][first]['segments']) if segment['state']=='STABILIZE')
    original_segments = json.dumps(plan['vehicles'][first]['segments'],sort_keys=True)
    executor.approach_heading[first] = None
    prepared_yaw = executor.prepare_approach_heading(first,time.monotonic())
    assert prepared_yaw is not None and executor.scan_alignment[first] is None
    assert not fleet.pending_by_vehicle
    prepared = dict(executor.approach_heading[first])
    assert executor.prepare_approach_heading(first,time.monotonic())==prepared_yaw
    assert executor.approach_heading[first]==prepared
    fleet.telemetry[first]['attitude']['received_monotonic_s'] = time.monotonic()-3
    assert executor.prepare_approach_heading(first,time.monotonic()) is None
    fleet.telemetry[first]['attitude']['received_monotonic_s'] = time.monotonic()
    executor.prepare_heading_during_approach = False
    assert executor.prepare_approach_heading(first,time.monotonic()) is None
    executor.prepare_heading_during_approach = True
    executor.advance_cursor(first)
    executor.station_heading[first] = None
    fleet.telemetry[first]['position'] = executor.segment(first)['destination'][:]
    fleet.telemetry[first]['attitude']['quaternion_mission_from_body_flu_xyzw'] = [0,0,math.sin(prepared_yaw/2),math.cos(prepared_yaw/2)]
    assert not executor.align_scan(first,executor.segment(first),10,time.monotonic())
    assert executor.scan_alignment[first]['stable_s']==0
    assert executor.scan_alignment[first]['heading_mission_rad']==prepared_yaw
    assert executor.scan_audit['approach_heading_uses']>=1
    assert not fleet.pending_by_vehicle
    assert executor.align_scan(first,executor.segment(first),1,time.monotonic())
    assert original_segments==json.dumps(plan['vehicles'][first]['segments'],sort_keys=True)
    report = dict(actual_arrival_required=True,stable_speed_required=True,exclusive_entry_token=True,
        heading_alignment_required=True,angular_speed_gate_verified=True,
        approach_heading_prepared_without_observation=True,prepared_heading_still_requires_fresh_stability=True,
        approach_heading_does_not_change_observation_order=True,approach_heading_disable_and_stale_attitude_checked=True,
        station_heading_preserved_during_pause=True,ordered_station_heading_reused=True,
        stale_attitude_waits_without_completion=True,pause_resets_heading_stability=True,
        infeasible_pointing_blocks_observation=True,
        scan_completion_gate=True,latched_actual_hold=True,pause_fault_preserve_cursors=True,
        coordinated_early_return=True,return_waits_for_all_entries=True,
        return_queue_fifo=True,return_token_until_landed_disarmed=True,
        independent_observation_hold=True,other_five_advance=True,coordination_fault_still_holds_all=True,
        unsafe_setpoint_holds_all_without_skipping_or_rtl=True,
        hardware_verified=False,scope='逻辑测试，非物理六机验收')
    (ROOT/'validation/waypoint_executor_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
