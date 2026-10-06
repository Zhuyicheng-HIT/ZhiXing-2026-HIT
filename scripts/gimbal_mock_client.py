from __future__ import annotations
import argparse
import json
import time
import uuid
from urllib.request import Request, urlopen


def request(base, path, payload=None):
    data = json.dumps(payload).encode('utf-8') if payload is not None else None
    with urlopen(Request(base+path,data=data,headers={'Content-Type':'application/json'}),timeout=10) as response:
        return json.load(response)


def observe(command, session, sequence):
    time.sleep((command.get('scan_motion') or {}).get('minimum_slew_s',0))
    conditions = command['observation_conditions']
    count = max(2,conditions['minimum_distinct_frames'])
    duration = conditions['minimum_observation_s']
    if command['command_type']=='SWEEP_SEGMENT':
        duration += command['scan_motion']['minimum_slew_s']
    interval = duration/(count-1)
    frames = []
    for index in range(count):
        if index:
            time.sleep(interval)
        capture = time.time()
        frames.append(dict(source_session=session,sequence=sequence+index,capture_time=capture,processed_at=time.time()))
    return dict(vehicle_id=command['vehicle_id'],command_id=command['command_id'],
        attempt_id=command['attempt_id'],executor_session=session,
        mission_epoch=command['mission_epoch'],plan_revision=command['plan_revision'],point_revision=command['point_revision'],
        status='FINISHED',evidence=dict(kind='SIMULATED_OBSERVATION',calibration_id=command['calibration_id'],
            zoom_actual=command['scan_zoom'],decoded=True,inference_completed=True,clarity_passed=True,
            attitude_valid=True,pointing_valid=True,zoom_valid=True,time_uncertainty_s=0,
            pointing_error_deg=0,stable_duration_s=max(conditions['minimum_stable_s'],conditions['minimum_observation_s']),
            source_frames=frames))


def main():
    parser = argparse.ArgumentParser(description='仅模拟证据的云台接口客户端；不连接ZR10或YOLO')
    parser.add_argument('--url',default='http://127.0.0.1:8766')
    parser.add_argument('--once',action='store_true')
    parser.add_argument('--vehicle-id',choices=['uav_'+str(index) for index in range(1,7)],help='限定本执行器只领取指定飞机的观察命令')
    args = parser.parse_args()
    session,sequence = 'mock-'+str(uuid.uuid4()),0
    print('SIMULATED_OBSERVATION only; no real camera or inference',flush=True)
    while True:
        path = '/api/gimbal/command'+('?vehicle_id='+args.vehicle_id if args.vehicle_id else '')
        response = request(args.url,path)
        if not response['active']:
            time.sleep(.2)
            continue
        command = response['command']
        identity = {name:command[name] for name in ('vehicle_id','command_id','mission_epoch','plan_revision','point_revision','attempt_id')}
        execution = response.get('execution') or {}
        if execution.get('retry_available'):
            request(args.url,'/api/gimbal/retry',identity)
            continue
        if execution.get('status')=='PAUSED':
            time.sleep(.2)
            continue
        status = dict(identity,executor_session=session,status='ACCEPTED')
        request(args.url,'/api/gimbal/status',status)
        request(args.url,'/api/gimbal/status',dict(status,status='RUNNING'))
        result = observe(command,session,sequence)
        sequence += len(result['evidence']['source_frames'])
        acknowledgement = request(args.url,'/api/gimbal/result',result)['result_ack']
        print(json.dumps(dict(command_id=command['command_id'],ack=acknowledgement)),flush=True)
        if args.once:
            return


if __name__=='__main__':
    main()
