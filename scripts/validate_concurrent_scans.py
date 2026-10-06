from __future__ import annotations
import json
import time
from copy import deepcopy
from fleet_core import ROOT
from sim_server import FleetSimulation
from validate_scan_protocol import example_result, rejected


def main():
    fleet = FleetSimulation()
    fleet.concurrent_observations = True
    commands = {}
    for vehicle in fleet.plan['vehicles']:
        command_id = next(command_id for command_id,item in fleet.scan_protocol.observations.items() if item['vehicle_id']==vehicle['id'])
        fleet.prepare_scan(vehicle['id'],command_id)
        commands[vehicle['id']] = deepcopy(fleet.gimbal_command(vehicle['id'])['command'])
    assert len(fleet.pending_by_vehicle)==6
    for owner,command in commands.items():
        assert fleet.gimbal_command(owner)['command']==command
    rejected(lambda:fleet.prepare_scan('uav_1',commands['uav_1']['command_id']))
    result = dict(example_result(commands['uav_3']),attempt_id=commands['uav_3']['attempt_id'],action='observation_result')
    time.sleep(3.1)
    acknowledgement = fleet.control(result)['result_ack']
    assert acknowledgement['completed'] and len(fleet.pending_by_vehicle)==5
    assert not fleet.gimbal_command('uav_3')['active']
    for owner in ('uav_1','uav_2','uav_4','uav_5','uav_6'):
        assert fleet.gimbal_command(owner)['command']==commands[owner]
    assert fleet.control(result)['result_ack']['duplicate']
    wrong = dict(example_result(commands['uav_4']),vehicle_id='uav_5',action='observation_result')
    rejected(lambda:fleet.control(wrong))
    assert len(fleet.pending_by_vehicle)==5
    fleet.running = True
    identity = {name:commands['uav_4'][name] for name in ('vehicle_id','command_id','mission_epoch','plan_revision','point_revision','attempt_id')}
    fleet.control(dict(identity,action='scan_execution_status',executor_session='isolated-test',status='FAILED',reason_code='SIMULATED_VIDEO_FAILURE'))
    assert fleet.gimbal_command('uav_4')['execution']['retry_available']
    assert all(fleet.gimbal_command(owner)['execution']['status']=='ISSUED' for owner in ('uav_1','uav_2','uav_5','uav_6'))
    fleet.control(dict(identity,action='retry_observation'))
    retry = fleet.gimbal_command('uav_4')['command']
    assert retry['command_id']==commands['uav_4']['command_id'] and retry['attempt_id']!=identity['attempt_id']
    rejected(lambda:fleet.control(dict(identity,action='scan_execution_status',executor_session='isolated-test',status='ACCEPTED')))
    for owner in ('uav_1','uav_2','uav_5','uav_6'):
        assert fleet.gimbal_command(owner)['command']==commands[owner]
    execution = fleet.scan_execution_by_vehicle['uav_4']
    execution.started_monotonic -= 400
    fleet.refresh_scan_execution()
    assert fleet.gimbal_command('uav_4')['execution']['status']=='TIMED_OUT'
    assert all(fleet.gimbal_command(owner)['execution']['status']=='ISSUED' for owner in ('uav_1','uav_2','uav_5','uav_6'))
    report = dict(six_independent_commands=True,one_completion_preserves_other_five=True,
        duplicate_completion_isolated=True,cross_vehicle_result_rejected=True,
        same_vehicle_overwrite_rejected=True,failure_retry_timeout_isolated=True,
        stale_attempt_rejected=True,hardware_verified=False,scope='command registry logic only')
    (ROOT/'validation/concurrent_scans_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
