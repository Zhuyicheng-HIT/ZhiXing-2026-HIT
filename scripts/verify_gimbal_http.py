from __future__ import annotations
import argparse
import json
import time
from pathlib import Path
from urllib.error import HTTPError
from fleet_core import ROOT
from gimbal_mock_client import request, observe


def main():
    parser = argparse.ArgumentParser(description='对当前已在等待外部观察结果的仿真任务做接口冒烟验证，结束时暂停')
    parser.add_argument('--url',default='http://127.0.0.1:8766')
    parser.add_argument('--report',type=Path,default=ROOT/'validation/gimbal_http_report.json')
    args = parser.parse_args()
    base = args.url
    before = request(base,'/api/state')
    active = request(base,'/api/gimbal/command')
    if not active['active'] or before['synthetic']:
        raise ValueError('需先启动非自动回传模式，并等待一个实际悬停观察命令')
    command = active['command']
    command_path = '/api/gimbal/command?vehicle_id='+command['vehicle_id']
    time.sleep(1)
    held = request(base,'/api/state')
    assert held['pending']['command_id']==command['command_id']
    assert [vehicle['completed_scans'] for vehicle in held['vehicles']]==[vehicle['completed_scans'] for vehicle in before['vehicles']]
    request(base,'/api/control',dict(action='fault',vehicle_id=command['vehicle_id']))
    fault = request(base,'/api/state')
    assert fault['pending']['command_id']==command['command_id']
    assert command['vehicle_id'] in fault['faults']
    request(base,'/api/control',dict(action='recover',vehicle_id=command['vehicle_id']))
    resumed = request(base,command_path)['command']
    assert resumed==command
    unavailable = {name:command[name] for name in ('vehicle_id','command_id','mission_epoch','plan_revision','point_revision')}
    unavailable.update(status='UNOBSERVABLE',reason_code='SIMULATED_FRAME_LOSS')
    assert not request(base,'/api/gimbal/result',unavailable)['result_ack']['completed']
    previous_attempt = command['attempt_id']
    identity = {name:command[name] for name in ('vehicle_id','command_id','mission_epoch','plan_revision','point_revision','attempt_id')}
    request(base,'/api/gimbal/retry',identity)
    command = request(base,command_path)['command']
    assert command['attempt_id']!=previous_attempt and command['execution_attempt']==2
    assert command['command_id']==resumed['command_id'] and command['point_revision']==resumed['point_revision']
    identity = {name:command[name] for name in ('vehicle_id','command_id','mission_epoch','plan_revision','point_revision','attempt_id')}
    request(base,'/api/gimbal/status',dict(identity,status='ACCEPTED',executor_session='http-smoke'))
    request(base,'/api/gimbal/status',dict(identity,status='RUNNING',executor_session='http-smoke'))
    result = observe(command,'http-smoke',0)
    try:
        request(base,'/api/gimbal/result',dict(result,mission_epoch='stale-epoch'))
    except HTTPError as error:
        assert error.code==400
    else:
        raise AssertionError('过期结果未拒绝')
    acknowledgement = request(base,'/api/gimbal/result',result)['result_ack']
    duplicate = request(base,'/api/gimbal/result',result)['result_ack']
    assert acknowledgement['completed'] and duplicate['duplicate']
    request(base,'/api/control',dict(action='pause'))
    after = request(base,'/api/state')
    report = dict(mode=after['mode'],command=command,external_result_ack=acknowledgement,
        repeat_result_ack=duplicate,actual_position_before=[vehicle['position'] for vehicle in before['vehicles']],
        actual_position_held=[vehicle['position'] for vehicle in held['vehicles']],
        pending_blocks_automatic_completion=True,fault_preserves_command=True,unobservable_keeps_pending=True,
        stale_result_rejected=True,after_paused=not after['running'],
        failed_attempt_retried_original_point=True,accepted_running_lifecycle_verified=True,
        completed_before=sum(vehicle['completed_scans'] for vehicle in before['vehicles']),
        completed_after=sum(vehicle['completed_scans'] for vehicle in after['vehicles']),
        scope='原生飞控悬停与模拟外部观察证据的接口冒烟；非完整任务或实机验收',
        full_mission_run=False,real_vision_verified=False,real_gimbal_verified=False)
    target = args.report
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
