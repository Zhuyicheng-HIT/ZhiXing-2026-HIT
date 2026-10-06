from __future__ import annotations
import json
from copy import deepcopy
from unittest.mock import patch
from fleet_core import ROOT
from sim_server import FleetSimulation
from validate_scan_protocol import example_result, rejected


def identity(command):
    return {name:command[name] for name in ('vehicle_id','command_id','mission_epoch','plan_revision','point_revision','attempt_id')}


def main():
    fleet = FleetSimulation()
    command_id = next(iter(fleet.scan_protocol.observations))
    owner = fleet.scan_protocol.observations[command_id]['vehicle_id']
    with patch('scan_execution.time.monotonic',return_value=100),patch('scan_protocol.time.time',return_value=1000):
        fleet.remove_pending(owner)
        fleet.prepare_scan(owner,command_id)
    fleet.running = True
    command = deepcopy(fleet.pending['command'])
    with patch('scan_execution.time.monotonic',return_value=101):
        rejected(lambda:fleet.control(dict(identity(command),action='scan_execution_status',status='RUNNING',executor_session='session-1')))
        accepted = dict(identity(command),action='scan_execution_status',status='ACCEPTED',executor_session='session-1')
        fleet.control(accepted)
        assert fleet.control(accepted)['execution_ack']['duplicate']
        fleet.control(dict(accepted,status='RUNNING'))
        rejected(lambda:fleet.control(dict(accepted,executor_session='foreign')))
    with patch('scan_execution.time.monotonic',return_value=162):
        assert fleet.gimbal_command()['execution']['status']=='TIMED_OUT'
        assert fleet.pending['command']==command and command_id not in fleet.completed
        rejected(lambda:fleet.control(dict(example_result(command),action='observation_result')))
        fleet.faults[owner] = 'simulated_link_loss'
        rejected(lambda:fleet.control(dict(identity(command),action='retry_observation')))
        fleet.faults.clear()
        with patch('scan_protocol.time.time',return_value=1100):
            fleet.control(dict(identity(command),action='retry_observation'))
    retry = deepcopy(fleet.pending['command'])
    assert retry['execution_attempt']==2 and retry['attempt_id']!=command['attempt_id']
    for name in ('command_id','mission_epoch','plan_revision','point_revision','target_enu_m'):
        assert retry[name]==command[name]
    with patch('scan_execution.time.monotonic',return_value=163):
        rejected(lambda:fleet.control(dict(example_result(command),action='observation_result')))
        rejected(lambda:fleet.control(dict(example_result(retry),action='observation_result')))
        accepted = dict(identity(retry),action='scan_execution_status',status='ACCEPTED',executor_session='session-2')
        fleet.control(accepted)
        fleet.control(dict(accepted,status='RUNNING'))
        result = dict(example_result(retry),attempt_id=retry['attempt_id'],executor_session='session-2',action='observation_result')
        with patch('scan_protocol.time.time',return_value=1104):
            assert fleet.control(result)['result_ack']['completed']
            assert fleet.control(result)['result_ack']['duplicate']
        assert fleet.pending is None and fleet.scan_execution.active['status']=='SUCCEEDED'
    second_id = list(fleet.scan_protocol.observations)[1]
    second_owner = fleet.scan_protocol.observations[second_id]['vehicle_id']
    with patch('scan_execution.time.monotonic',return_value=170),patch('scan_protocol.time.time',return_value=1150):
        fleet.prepare_scan(second_owner,second_id)
        second = deepcopy(fleet.pending['command'])
        assert fleet.control(result)['result_ack']['duplicate']
        assert fleet.pending['command']==second
        fleet.control(dict(identity(second),action='scan_execution_status',status='REJECTED',executor_session='reject-session',reason_code='SIMULATED_CAMERA_UNAVAILABLE'))
        assert fleet.pending['execution']['retry_available'] and second_id not in fleet.completed
        fleet.running = False
        rejected(lambda:fleet.control(dict(identity(second),action='retry_observation')))
        fleet.running = True
        fleet.control(dict(identity(second),action='retry_observation'))
        second_retry = deepcopy(fleet.pending['command'])
        payload = dict(identity(second_retry),action='observation_result',status='PAUSED',reason_code='SIMULATED_OPERATOR_PAUSE')
        fleet.control(payload)
        assert fleet.pending['execution']['retry_available']
        rejected(lambda:fleet.control(dict(example_result(second_retry),attempt_id=second_retry['attempt_id'],action='observation_result')))
    with patch('scan_execution.time.monotonic',return_value=200),patch('scan_protocol.time.time',return_value=1200):
        fleet.remove_pending(owner)
        fleet.prepare_scan(owner,command_id)
    with patch('scan_execution.time.monotonic',return_value=501):
        assert fleet.gimbal_command()['execution']['reason_code']=='ACCEPTANCE_TIMEOUT'
    report = dict(accepted_before_running=True,session_ownership_verified=True,
        monotonic_acceptance_and_execution_timeout=True,timeout_preserves_point=True,
        fault_blocks_retry=True,retry_same_point_new_attempt=True,stale_attempt_rejected=True,
        evidence_required_for_success=True,result_idempotent=True,no_automatic_skip_or_rtl=True,
        previous_point_duplicate_preserves_current_attempt=True,paused_task_blocks_retry=True,
        paused_observation_requires_retry=True,rejection_keeps_original_point=True,
        real_hardware_verified=False)
    (ROOT/'validation/scan_execution_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
