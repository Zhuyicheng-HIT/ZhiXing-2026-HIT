from __future__ import annotations
import json
from copy import deepcopy
from unittest.mock import patch
from fleet_core import ROOT, build_plan
from scan_protocol import ScanProtocol
from sim_server import FleetSimulation


def rejected(callback):
    try:
        callback()
    except ValueError:
        return
    raise AssertionError('非法结果被接收')


def example_result(command):
    issued = command['issued_at']
    count = command['observation_conditions']['minimum_distinct_frames']
    span = command['observation_conditions']['minimum_observation_s']
    return dict(vehicle_id=command['vehicle_id'],command_id=command['command_id'],
        mission_epoch=command['mission_epoch'],plan_revision=command['plan_revision'],
        point_revision=command['point_revision'],status='FINISHED',evidence=dict(
            kind='SIMULATED_OBSERVATION',calibration_id=command['calibration_id'],zoom_actual=command['scan_zoom'],
            decoded=True,inference_completed=True,clarity_passed=True,attitude_valid=True,pointing_valid=True,
            zoom_valid=True,time_uncertainty_s=0,pointing_error_deg=0,stable_duration_s=1,
            source_frames=[dict(source_session='protocol-test',sequence=index,capture_time=issued+span*index/(count-1),
                                processed_at=issued+3) for index in range(count)]))


def main():
    plan = build_plan()
    profile = json.loads((ROOT/'config/scan_profile.json').read_text(encoding='utf-8'))
    protocol = ScanProtocol(plan,profile)
    command_id = next(iter(protocol.observations))
    owner = protocol.observations[command_id]['vehicle_id']
    with patch('scan_protocol.time.time',return_value=1000):
        command = protocol.issue(owner,command_id,'test-epoch')
    result = example_result(command)
    with patch('scan_protocol.time.time',return_value=1004):
        for field,value in (('vehicle_id','uav_999'),('mission_epoch','old'),('plan_revision','old'),('point_revision','old')):
            wrong = dict(result,**{field:value})
            rejected(lambda:protocol.receive(wrong,'test-epoch',command))
        for field,value in (('kind','REAL_OBSERVATION'),('decoded',False),('zoom_actual',2),
                            ('stable_duration_s',-1),('time_uncertainty_s',5),('pointing_error_deg',99)):
            wrong = deepcopy(result)
            wrong['evidence'][field] = value
            rejected(lambda:protocol.receive(wrong,'test-epoch',command))
        for mutation in ('duplicate','stale','too_few','future'):
            wrong = deepcopy(result)
            frames = wrong['evidence']['source_frames']
            if mutation=='duplicate':
                frames[1]['sequence'] = frames[0]['sequence']
            elif mutation=='stale':
                frames[0]['capture_time'] = 999
            elif mutation=='too_few':
                frames.pop()
            else:
                frames[0]['processed_at'] = 1005
            rejected(lambda:protocol.receive(wrong,'test-epoch',command))
        unavailable = {name:result[name] for name in ('vehicle_id','command_id','mission_epoch','plan_revision','point_revision')}
        unavailable.update(status='UNOBSERVABLE',reason_code='TEMPORARY_FRAME_LOSS')
        assert not protocol.receive(unavailable,'test-epoch',command)['completed']
        assert not protocol.receipts
        assert protocol.receive(result,'test-epoch',command)['completed']
        assert protocol.receive(result,'test-epoch',None)['duplicate']
        rejected(lambda:protocol.receive(dict(result,status='SUCCESS_FOUND'),'test-epoch',command))
    saved = protocol.checkpoint()
    restored = ScanProtocol(plan,profile)
    restored.restore(saved)
    assert restored.receipts==protocol.receipts
    changed = deepcopy(plan)
    changed['vehicles'][0]['home'][0] += 1
    rejected(lambda:ScanProtocol(changed,profile).restore(saved))
    changed_profile = dict(profile,scan_zoom=2)
    rejected(lambda:ScanProtocol(plan,changed_profile).restore(saved))
    fleet = FleetSimulation()
    fleet.prepare_scan(owner,command_id)
    command = fleet.gimbal_command()['command']
    with patch('scan_protocol.time.time',return_value=command['issued_at']+4):
        result = dict(example_result(command),action='observation_result')
        assert fleet.control(result)['result_ack']['completed']
        assert fleet.control(result)['result_ack']['duplicate']
    assert command_id in fleet.completed and fleet.pending is None
    print(json.dumps(dict(protocol_version='zhixin/gimbal-command/v1',invalid_results_rejected=True,
        unavailable_keeps_pending=True,idempotent=True,versioned_restore=True,coordinator_integration=True,
        real_hardware_verified=False),ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
