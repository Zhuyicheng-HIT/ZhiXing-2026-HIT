from __future__ import annotations
import argparse
import json
import math
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from urllib.error import HTTPError
from fleet_core import ROOT
from gimbal_mock_client import request, observe


def identity(command):
    return {name:command[name] for name in ('vehicle_id','command_id','mission_epoch','plan_revision','point_revision','attempt_id')}


def complete(base, command):
    session = 'parallel-test-'+command['attempt_id']
    payload = dict(identity(command),executor_session=session)
    request(base,'/api/gimbal/status',dict(payload,status='ACCEPTED'))
    request(base,'/api/gimbal/status',dict(payload,status='RUNNING'))
    return request(base,'/api/gimbal/result',observe(command,session,0))['result_ack']


def verify(base, timeout, report_path):
    before = request(base,'/api/state')
    assert before['mode']=='ardupilot_internal_physics_sitl' and not before['running'] and before['progress']==0
    request(base,'/api/control',dict(action='start',forest_permission=True,isolation_transit_simulation=True,synthetic=False))
    started = time.monotonic()
    while time.monotonic()-started<timeout:
        state = request(base,'/api/state')
        if state['faults']:
            raise AssertionError(state['faults'])
        if len(state.get('pending_commands',[]))==6:
            break
        time.sleep(.2)
    else:
        raise AssertionError('Not all six aircraft reached independent observation wait')
    commands = {pending['vehicle_id']:pending['command'] for pending in state['pending_commands']}
    waiting_owner = state['pending_commands'][0]['vehicle_id']
    waiting_vehicle = next(vehicle for vehicle in state['vehicles'] if vehicle['id']==waiting_owner)
    for owner,command in commands.items():
        assert request(base,'/api/gimbal/command?vehicle_id='+owner)['command']==command
    others = [command for owner,command in commands.items() if owner!=waiting_owner]
    with ThreadPoolExecutor(max_workers=5) as workers:
        acknowledgements = list(workers.map(lambda command:complete(base,command),others))
    assert all(acknowledgement['completed'] for acknowledgement in acknowledgements)
    time.sleep(.5)
    state = request(base,'/api/state')
    held = next(vehicle for vehicle in state['vehicles'] if vehicle['id']==waiting_owner)
    assert held['segment_index']==waiting_vehicle['segment_index'] and held['completed_scans']==0
    assert math.dist(held['position'],waiting_vehicle['position'])<.8
    assert request(base,'/api/gimbal/command?vehicle_id='+waiting_owner)['command']==commands[waiting_owner]
    assert all(vehicle['completed_scans']==1 for vehicle in state['vehicles'] if vehicle['id']!=waiting_owner)
    failed_command = commands[waiting_owner]
    request(base,'/api/gimbal/status',dict(identity(failed_command),executor_session='failed-camera-test',status='FAILED',reason_code='SIMULATED_VIDEO_FAILURE'))
    other_owner = others[0]['vehicle_id']
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        available = request(base,'/api/gimbal/command?vehicle_id='+other_owner)
        if available['active']:
            break
        time.sleep(.2)
    else:
        raise AssertionError('Other aircraft did not reach its next observation while one failed')
    assert complete(base,available['command'])['completed']
    assert request(base,'/api/gimbal/command?vehicle_id='+waiting_owner)['execution']['status']=='FAILED'
    request(base,'/api/gimbal/retry',identity(failed_command))
    retry = request(base,'/api/gimbal/command?vehicle_id='+waiting_owner)['command']
    assert retry['command_id']==failed_command['command_id'] and retry['point_revision']==failed_command['point_revision']
    assert retry['attempt_id']!=failed_command['attempt_id'] and retry['execution_attempt']==2
    try:
        request(base,'/api/gimbal/status',dict(identity(failed_command),executor_session='failed-camera-test',status='ACCEPTED'))
    except HTTPError as error:
        assert error.code==400
    else:
        raise AssertionError('Stale failed attempt was accepted')
    request(base,'/api/control',dict(action='fault',vehicle_id=waiting_owner,reason='SIMULATED_COORDINATION_TEST'))
    frozen = request(base,'/api/state')
    cursors = [vehicle['segment_index'] for vehicle in frozen['vehicles']]
    time.sleep(.8)
    fault = request(base,'/api/state')
    assert [vehicle['segment_index'] for vehicle in fault['vehicles']]==cursors
    assert all(vehicle['telemetry']['mode']=='GUIDED' for vehicle in fault['vehicles'])
    assert all(vehicle['state'] in ('GROUP_HOLD','RECOVERING') for vehicle in fault['vehicles'])
    request(base,'/api/control',dict(action='recover',vehicle_id=waiting_owner))
    assert request(base,'/api/gimbal/command?vehicle_id='+waiting_owner)['command']==retry
    request(base,'/api/control',dict(action='pause'))
    state = request(base,'/api/state')
    report = dict(six_actual_fcu_observation_waits=True,five_parallel_evidence_results=True,
        held_aircraft_cursor_unchanged=True,held_aircraft_command_preserved=True,
        other_five_completed_one_observation=True,coordination_fault_holds_all=True,
        failed_observation_does_not_block_other_aircraft=True,failed_point_retried_with_new_attempt=True,
        stale_failed_attempt_rejected=True,
        no_independent_rtl_observed=True,conditional_forest_transit=True,conditional_isolation_transit=True,competition_airspace_compliant=False,
        synthetic_evidence=True,full_mission_verified=False,hardware_verified=False,
        gazebo_physics_verified=False,wall_duration_s=time.monotonic()-started,
        waiting_vehicle_id=waiting_owner,speedup=state['speed'],
        observed_min_separation_m=state['observed_min_separation_m'],flight_audit=state['flight_audit'],
        flight_safety_audit=state.get('flight_safety_audit'))
    report['entry_policy'] = state.get('entry_policy')
    report['reservation_audit'] = state.get('reservation_audit')
    report_path.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


def main():
    parser = argparse.ArgumentParser(description='Bounded six-FCU parallel scan test; simulated evidence only')
    parser.add_argument('--url',default='http://127.0.0.1:8766')
    parser.add_argument('--timeout',type=float,default=180)
    parser.add_argument('--report',type=Path,default=ROOT/'validation/concurrent_sitl_report.json')
    args = parser.parse_args()
    verify(args.url,args.timeout,args.report)


if __name__=='__main__':
    main()
