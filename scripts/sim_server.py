from __future__ import annotations
import argparse
import hashlib
import json
import math
import threading
import time
import uuid
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
from fleet_core import ROOT, build_plan, pose_at
from scan_protocol import ScanProtocol, revision
from scan_execution import ScanExecution
from gimbal_geometry import solve_pointing, feasible_pointing
from isolation_airspace import require_isolation_permission
from scan_motion import scan_look_target, target_in_sector
from geodesy import simulation_frame
from manual_drafts import ManualDraftStore
from mission_editor import MissionWaypointStore, mission_waypoints


class FleetSimulation:
    def __init__(self):
        self.lock = threading.RLock()
        self.plan = build_plan()
        self.speed = 10.0
        self.reset()

    def reset(self):
        self.stamp = 0.0
        self.running = False
        self.epoch = str(uuid.uuid4())
        self.completed = set()
        self.faults = {}
        self.pending = None
        self.pending_by_vehicle = {}
        self.scan_execution_by_vehicle = {}
        self.concurrent_observations = getattr(self,'concurrent_observations',False)
        self.scan_execution = ScanExecution()
        self.gimbal_geometry = json.loads((ROOT/'config/gimbal_geometry.json').read_text(encoding='utf-8'))
        profile = json.loads((ROOT/'config/scan_profile.json').read_text(encoding='utf-8'))
        profile['geometry_revision'] = revision(self.gimbal_geometry)
        self.scan_protocol = ScanProtocol(self.plan,profile)
        self.synthetic = True
        self.forest_permission = False
        self.isolation_transit_simulation = False
        self.target_updates = {}
        self.perception = {}
        self.abandoned_tracking = set()
        self.log = []
        self.emit('READY','任务规划完成；当前为运动学仿真，不连接飞控')

    def emit(self, event, message, **extra):
        self.log.append(dict(time_s=round(self.stamp,3),event=event,message=message,**extra))
        self.log = self.log[-200:]

    def checkpoint(self):
        target = ROOT/'runtime/checkpoint.json'
        target.parent.mkdir(exist_ok=True)
        data = dict(schema='zhixin/checkpoint/v1',plan=self.plan,stamp=self.stamp,epoch=self.epoch,
                    completed=sorted(self.completed),pending=self.pending,synthetic=self.synthetic,
                    forest_permission=self.forest_permission,speed=self.speed,scan_protocol=self.scan_protocol.checkpoint())
        data['target_updates'] = self.target_updates
        data['abandoned_tracking'] = [list(item) for item in sorted(self.abandoned_tracking)]
        data['scan_execution'] = self.scan_execution.snapshot()
        data['pending_by_vehicle'] = self.pending_by_vehicle
        temporary = target.with_suffix('.tmp')
        temporary.write_text(json.dumps(data,ensure_ascii=False),encoding='utf-8')
        temporary.replace(target)

    def advance(self, elapsed):
        with self.lock:
            self.refresh_scan_execution()
            if not self.running or self.faults or self.pending:
                return
            end = min(self.plan['duration_s'],self.stamp+elapsed*self.speed)
            events = []
            for vehicle in self.plan['vehicles']:
                for segment in vehicle['segments']:
                    if self.stamp < segment['end'] <= end and segment['state']=='SCAN' and segment['command_id'] not in self.completed:
                        events.append((segment['end'],vehicle['id'],segment['command_id']))
            for stamp,vehicle_id,command_id in sorted(events):
                if self.synthetic:
                    self.completed.add(command_id)
                    self.emit('SCAN_RESULT','模拟云台回传FINISHED；不是YOLO真实检测',vehicle_id=vehicle_id,command_id=command_id)
                else:
                    self.stamp = stamp
                    self.prepare_scan(vehicle_id,command_id)
                    self.emit('WAIT_RESULT','等待带当前epoch和command_id的扫描完成结果',**self.pending)
                    self.checkpoint()
                    return
            previous = self.stamp
            self.stamp = end
            for vehicle in self.plan['vehicles']:
                old = pose_at(vehicle,previous)[1]
                new = pose_at(vehicle,end)[1]
                if old['state'] != new['state']:
                    self.emit(new['state'],vehicle['id']+'进入'+new['state'],vehicle_id=vehicle['id'])
            if end >= self.plan['duration_s']:
                self.running = False
                self.emit('COMPLETE','六机运动学演示结束；科二未执行抛投')
                self.checkpoint()

    def prepare_scan(self, vehicle_id, command_id, actual_position=None, execution_attempt=1):
        if vehicle_id in self.pending_by_vehicle:
            raise ValueError('Aircraft already has an active observation')
        command = self.scan_protocol.issue(vehicle_id,command_id,self.epoch,actual_position)
        index = next(index for index,vehicle in enumerate(self.plan['vehicles']) if vehicle['id']==vehicle_id)
        attitude = None
        if hasattr(self,'telemetry'):
            item = self.telemetry[index]
            if item.get('attitude') and time.monotonic()-item['attitude']['received_monotonic_s']<=2:
                attitude = item['attitude']['quaternion_mission_from_body_flu_xyzw']
        else:
            attitude = [0,0,0,1]
        command['pointing_at_issue'] = self.pointing_display(command['target_enu_m'],actual_position or command['station_enu_m'],attitude)
        execution = ScanExecution()
        execution.begin(command,execution_attempt)
        self.pending_by_vehicle[vehicle_id] = dict(vehicle_id=vehicle_id,command_id=command_id,
            mission_epoch=self.epoch,command=command,execution=execution.snapshot())
        self.scan_execution_by_vehicle[vehicle_id] = execution
        self.sync_pending_aliases()

    def sync_pending_aliases(self):
        self.pending = next(iter(self.pending_by_vehicle.values()),None)
        if self.pending:
            self.scan_execution = self.scan_execution_by_vehicle[self.pending['vehicle_id']]

    def remove_pending(self, vehicle_id):
        self.pending_by_vehicle.pop(vehicle_id,None)
        self.sync_pending_aliases()

    def selected_pending(self, payload):
        return self.pending_by_vehicle.get(payload.get('vehicle_id'))

    def refresh_scan_execution(self):
        for vehicle_id,pending in self.pending_by_vehicle.items():
            execution = self.scan_execution_by_vehicle[vehicle_id]
            execution.refresh(pending['command'])
            pending['execution'] = execution.snapshot()
        self.sync_pending_aliases()

    def pointing_display(self, target, position, attitude):
        if target is None:
            return dict(valid=False,invalid_reason='NO_ACTIVE_GROUND_TARGET',sdk_command_available=False)
        if attitude is None:
            return dict(valid=False,invalid_reason='ATTITUDE_MISSING_OR_STALE',sdk_command_available=False)
        try:
            pointing = solve_pointing(target,position,attitude,self.gimbal_geometry)
            pointing['executable'] = feasible_pointing(pointing['look_direction'],self.gimbal_geometry,
                self.scan_protocol.profile['maximum_pointing_error_deg'])
            return pointing
        except ValueError as error:
            return dict(valid=False,invalid_reason=str(error),sdk_command_available=False)

    def gimbal_command(self, vehicle_id=None):
        with self.lock:
            self.refresh_scan_execution()
            if vehicle_id is not None and vehicle_id not in [vehicle['id'] for vehicle in self.plan['vehicles']]:
                raise ValueError('Unknown vehicle_id')
            pending = self.pending_by_vehicle.get(vehicle_id) if vehicle_id is not None else self.pending
            return dict(active=bool(pending),command=pending.get('command') if pending else None,
                        execution=pending.get('execution') if pending else None,
                        available_vehicle_ids=list(self.pending_by_vehicle),concurrent_observations=self.concurrent_observations,
                        hardware_authorized=False)

    def snapshot(self):
        with self.lock:
            vehicles = []
            for vehicle in self.plan['vehicles']:
                position,segment,index = pose_at(vehicle,self.stamp)
                state = 'RECOVERING' if vehicle['id'] in self.faults else ('GROUP_HOLD' if self.faults or self.pending else segment['state'])
                target = scan_look_target(segment,self.stamp-segment['start'])
                yaw,pitch = 0.0,-90.0
                if target:
                    east,north,height = [value-origin for value,origin in zip(target,position)]
                    yaw = math.degrees(math.atan2(north,east))
                    pitch = math.degrees(math.atan2(height,math.hypot(east,north)))
                vehicles.append(dict(id=vehicle['id'],position=position,state=state,resume_state=segment['state'],
                                     station=segment.get('station'),observation=segment.get('observation'),segment_index=index,
                                     target=target,gimbal=dict(azimuth_enu_deg=yaw,elevation_deg=pitch,
                                         pointing=self.pointing_display(target,position,[0,0,0,1]),attitude_source='SYNTHETIC_EAST_HEADING'),
                                     completed_scans=sum(command.startswith(vehicle['id']+':') for command in self.completed),
                                     fault=self.faults.get(vehicle['id'])))
            data = dict(stamp=self.stamp,running=self.running,paused=not self.running,speed=self.speed,
                        epoch=self.epoch,pending=self.pending,pending_commands=list(self.pending_by_vehicle.values()),faults=self.faults,synthetic=self.synthetic,
                        forest_permission=self.forest_permission,isolation_transit_simulation=self.isolation_transit_simulation,
                        competition_airspace_compliant=self.plan['isolation_airspace']['authorized_policy_respected'] and not self.plan['requires_forest_transit_permission'],target_updates=self.target_updates,vehicles=vehicles,log=self.log[-25:],
                    progress=self.stamp/self.plan['duration_s'],mode=self.plan['mode'],perception=self.perception)
            data['camera'] = self.camera_status()
            return data

    def camera_status(self, freshness_s=2.0):
        now = time.time()
        result = {}
        for index in range(1, 7):
            vehicle_id = f'uav_{index}'
            metadata_path = ROOT/'runtime'/'camera'/f'{vehicle_id}.json'
            image_path = ROOT/'runtime'/'camera'/f'{vehicle_id}.jpg'
            item = dict(vehicle_id=vehicle_id, available=image_path.is_file(), fresh=False)
            try:
                metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
                captured = float(metadata['captured_at_unix_s'])
                item.update(frame_counter=int(metadata.get('frame_counter', 0)),
                            captured_at_unix_s=captured, age_s=max(0.0, now-captured),
                            width=int(metadata.get('width', 0)), height=int(metadata.get('height', 0)))
                item['fresh'] = item['available'] and item['age_s'] <= freshness_s
            except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
                if item['available']:
                    item['age_s'] = max(0.0, now-image_path.stat().st_mtime)
                    item['fresh'] = item['age_s'] <= freshness_s
            result[vehicle_id] = item
        return result

    def control(self, data):
        with self.lock:
            action = data.get('action')
            if action=='start':
                isolation_permission = require_isolation_permission(self.plan,data)
                permission = self.plan.get('airspace_policy',{}).get('forest_transit_authorized',False) or bool(data.get('forest_permission',False))
                if self.plan['requires_forest_transit_permission'] and not permission:
                    raise ValueError('严格禁穿树林时区域不连通。需明确授权模拟跨树林，仅用于条件演示')
                if self.stamp >= self.plan['duration_s']:
                    self.reset()
                self.forest_permission = permission
                self.isolation_transit_simulation = isolation_permission
                self.synthetic = bool(data.get('synthetic',True))
                self.running = True
                self.emit('START','开始/继续动态任务')
            elif action=='pause':
                self.running = False
                self.emit('PAUSE','暂停并保存任务断点')
                self.checkpoint()
            elif action=='reset':
                self.reset()
            elif action=='speed':
                speed = float(data['value'])
                if not math.isfinite(speed) or not .1 <= speed <= 100:
                    raise ValueError('速度倍率范围0.1~100')
                self.speed = speed
            elif action=='replan':
                if self.running:
                    raise ValueError('先暂停；重新规划会清空旧任务进度')
                draft = self.manual_drafts_store.load(self.plan) if data.get('use_manual_draft', False) else None
                self.plan = build_plan(homes=data.get('homes'),subject=int(data.get('subject',1)),footprint=float(data.get('footprint',80)),manual_draft=draft)
                (ROOT/'missions/fleet_plan.json').write_text(json.dumps(self.plan,ensure_ascii=False,indent=2),encoding='utf-8')
                self.reset()
            elif action=='target_update':
                owner = data.get('vehicle_id')
                vehicle = next((item for item in self.plan['vehicles'] if item['id']==owner),None)
                if vehicle is None:
                    raise ValueError('Unknown target owner')
                decision = target_in_sector(vehicle,data.get('target_enu_m'))
                if hasattr(self,'executor'):
                    segment = self.executor.segment(self.plan['vehicles'].index(vehicle))
                else:
                    segment = pose_at(vehicle,self.stamp)[1]
                if segment['state']!='TRACK_MOVING':
                    raise ValueError('Target updates require an active TRACK_MOVING segment')
                if (owner,segment['station']) in self.abandoned_tracking:
                    decision.update(action='ABANDON_NO_HANDOFF',reason='ALREADY_ABANDONED_AT_THIS_STATION')
                self.target_updates[owner] = dict(decision,target_enu_m=data['target_enu_m'],source='external_or_simulated_update_not_verified_vision')
                if decision['action']=='ABANDON_NO_HANDOFF':
                    self.abandoned_tracking.add((owner,segment['station']))
                    self.emit('TARGET_ABANDONED','目标离开本机搜索区域，放弃跟随，不跨区、不交接',vehicle_id=owner)
                self.checkpoint()
                return dict(self.snapshot(),target_ack=decision)
            elif action=='fault':
                vehicle_id = data.get('vehicle_id')
                if vehicle_id not in [vehicle['id'] for vehicle in self.plan['vehicles']]:
                    raise ValueError('未知飞机ID')
                self.faults[vehicle_id] = str(data.get('reason','gimbal_link_lost'))
                self.emit('RECOVERING','保持任务断点；等待重连，不自动独立RTL',vehicle_id=vehicle_id)
                self.checkpoint()
            elif action=='recover':
                vehicle_id = data.get('vehicle_id')
                self.faults.pop(vehicle_id,None)
                self.emit('RESUME_CHECKPOINT','链路模拟恢复；继续原扫描命令/航段',vehicle_id=vehicle_id)
            elif action=='observation_result':
                pending = self.selected_pending(data)
                execution = self.scan_execution_by_vehicle.get(data.get('vehicle_id'))
                receipt_key = self.epoch+':'+str(data.get('command_id'))
                if pending and receipt_key not in self.scan_protocol.receipts:
                    execution.validate_result(data,pending['command'])
                acknowledgement = self.scan_protocol.receive(data,self.epoch,pending.get('command') if pending else None)
                if acknowledgement['completed'] and not acknowledgement['duplicate']:
                    self.completed.add(data['command_id'])
                    execution.active.update(status='SUCCEEDED',retry_available=False)
                    self.remove_pending(data['vehicle_id'])
                elif pending and not acknowledgement['duplicate']:
                    execution.note_result(data)
                    self.refresh_scan_execution()
                self.emit('OBSERVATION_RESULT','逐点观察结果已校验；模拟证据不代表实机检测',command_id=data['command_id'],acknowledgement=acknowledgement)
                self.checkpoint()
                return dict(self.snapshot(),result_ack=acknowledgement)
            elif action=='scan_execution_status':
                pending = self.selected_pending(data)
                if not pending:
                    raise ValueError('No active observation')
                execution = self.scan_execution_by_vehicle[data['vehicle_id']]
                acknowledgement = execution.receive(data,pending['command'])
                self.refresh_scan_execution()
                self.checkpoint()
                return dict(self.snapshot(),execution_ack=acknowledgement,scan_execution=execution.snapshot())
            elif action=='retry_observation':
                pending = self.selected_pending(data)
                if not pending or not self.running or self.faults:
                    raise ValueError('Retry requires active running task with faults recovered')
                command = pending['command']
                execution = self.scan_execution_by_vehicle[data['vehicle_id']]
                execution.validate_attempt(data,command)
                if data.get('attempt_id')!=command['attempt_id']:
                    raise ValueError('Retry requires previous attempt_id')
                self.refresh_scan_execution()
                if not execution.active['retry_available']:
                    raise ValueError('Observation still active; do not create concurrent attempts')
                attempt = command['execution_attempt']+1
                position = command['actual_position_enu_m']
                if hasattr(self,'telemetry'):
                    index = next(index for index,vehicle in enumerate(self.plan['vehicles']) if vehicle['id']==command['vehicle_id'])
                    item = self.telemetry[index]
                    attitude = item.get('attitude')
                    if not attitude or time.monotonic()-attitude['received_monotonic_s']>2:
                        raise ValueError('Retry waiting for fresh aircraft attitude')
                    pointing = self.pointing_display(command['target_enu_m'],item['position'],attitude['quaternion_mission_from_body_flu_xyzw'])
                    if not pointing.get('executable',{}).get('feasible',False) or math.dist(item['position'],command['station_enu_m'])>.8 or item['speed_m_s']>.6:
                        raise ValueError('Retry waiting for stable arrival and feasible nominal pointing')
                    position = item['position']
                self.remove_pending(command['vehicle_id'])
                self.prepare_scan(command['vehicle_id'],command['command_id'],position,attempt)
                self.emit('RETRY_OBSERVATION','重试原观察点；不跳点、不独立RTL',command_id=command['command_id'],execution_attempt=attempt)
                self.checkpoint()
                return dict(self.snapshot(),scan_execution=self.scan_execution_by_vehicle[command['vehicle_id']].snapshot())
            elif action=='result':
                if data.get('mission_epoch') != self.epoch:
                    raise ValueError('过期mission_epoch，结果不接收')
                command_id = data.get('command_id')
                owner = command_id.split(':',1)[0] if isinstance(command_id,str) else None
                if data.get('vehicle_id') != owner or owner not in [vehicle['id'] for vehicle in self.plan['vehicles']]:
                    raise ValueError('扫描结果vehicle_id与命令拥有者不匹配')
                if command_id in self.completed:
                    return self.snapshot()
                pending = self.selected_pending(data)
                if not pending or pending['command_id'] != command_id:
                    raise ValueError('结果必须对应当前等待的command_id')
                if data.get('status') not in ('FINISHED','SUCCESS_FOUND','SUCCESS_NOT_FOUND'):
                    raise ValueError('结果未完成；继续等待/恢复，不跳过观察点')
                self.completed.add(command_id)
                self.remove_pending(owner)
                self.emit('SIMULATED_COMPLETION','人工模拟完成；没有真实观察证据',command_id=command_id)
                self.checkpoint()
            elif action=='perception_result':
                vehicle_id = data.get('vehicle_id')
                if vehicle_id not in {vehicle['id'] for vehicle in self.plan['vehicles']}:
                    raise ValueError('Unknown vehicle_id')
                self.perception[vehicle_id] = dict(data, received_at_unix_s=time.time())
            elif action=='checkpoint':
                self.checkpoint()
            elif action=='restore':
                if self.running:
                    raise ValueError('暂停后再恢复存档')
                saved = json.loads((ROOT/'runtime/checkpoint.json').read_text(encoding='utf-8'))
                protocol = ScanProtocol(saved['plan'],self.scan_protocol.profile)
                protocol.restore(saved['scan_protocol'])
                self.scan_protocol = protocol
                self.plan,self.stamp,self.epoch = saved['plan'],saved['stamp'],saved['epoch']
                self.completed = set(saved['completed'])
                self.pending_by_vehicle = saved.get('pending_by_vehicle') or ({saved['pending']['vehicle_id']:saved['pending']} if saved['pending'] else {})
                self.scan_execution_by_vehicle = {}
                self.scan_execution = ScanExecution()
                for vehicle_id,pending in self.pending_by_vehicle.items():
                    command = pending['command']
                    attempt = command.get('execution_attempt',1)+1
                    now = time.time()
                    command.update(issued_at=now,accept_before=now+self.scan_protocol.profile['command_acceptance_window_s'])
                    execution = ScanExecution()
                    execution.begin(command,attempt)
                    self.scan_execution_by_vehicle[vehicle_id] = execution
                    pending['execution'] = execution.snapshot()
                self.sync_pending_aliases()
                self.synthetic,self.forest_permission,self.speed = saved['synthetic'],saved['forest_permission'],saved['speed']
                self.target_updates = saved.get('target_updates',{})
                self.abandoned_tracking = {tuple(item) for item in saved.get('abandoned_tracking',[])}
                self.faults = {}
                self.emit('RESTORED','加载断点；保持暂停，等待操作员继续')
            else:
                raise ValueError('未知操作')
            return self.snapshot()


def serve(host='127.0.0.1',port=8765,simulation=None):
    simulation = simulation or FleetSimulation()
    manual_drafts = ManualDraftStore(ROOT)
    mission_waypoints_store = MissionWaypointStore(ROOT)
    simulation.mission_waypoints_store = mission_waypoints_store
    simulation.manual_drafts_store = manual_drafts
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,directory=str(ROOT/'web'),**kwargs)

        def send_json(self,value,status=200):
            payload = json.dumps(value,ensure_ascii=False,allow_nan=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type','application/json; charset=utf-8')
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def send_camera_frame(self, vehicle_id):
            if vehicle_id not in {f'uav_{index}' for index in range(1,7)}:
                self.send_json(dict(error='invalid vehicle id'),400)
                return
            target = ROOT/'runtime'/'camera'/f'{vehicle_id}.jpg'
            if not target.is_file():
                self.send_json(dict(error='camera frame unavailable',vehicle_id=vehicle_id),404)
                return
            payload = target.read_bytes()
            self.send_response(200)
            self.send_header('Content-Type','image/jpeg')
            self.send_header('Cache-Control','no-store, no-cache, must-revalidate')
            self.send_header('Content-Length',str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            if urlparse(self.path).path=='/api/mission-waypoints':
                with simulation.lock:
                    reset = parse_qs(urlparse(self.path).query).get('reset', ['0'])[0] == '1'
                    self.send_json(mission_waypoints_store.load(simulation.plan, reset=reset))
                return
            if urlparse(self.path).path=='/api/manual-draft':
                try:
                    with simulation.lock:
                        self.send_json(manual_drafts.load(simulation.plan))
                except (ValueError,OSError) as error:
                    self.send_json(dict(error=str(error)),400)
                return
            if urlparse(self.path).path=='/api/map/coordinates':
                try:
                    query = parse_qs(urlparse(self.path).query)
                    east,north = float(query['east'][0]),float(query['north'][0])
                    if not all(math.isfinite(value) and abs(value)<=10000 for value in (east,north)):
                        raise ValueError('地图取点坐标超出范围')
                    with simulation.lock:
                        frame = simulation_frame(simulation.plan['datum'])
                        latitude,longitude,height = frame.reverse([east,north,0])
                    self.send_json(dict(latitude_deg=latitude,longitude_deg=longitude,
                        east_m=east,north_m=north,map_origin_id=frame.origin_id,datum='WGS84'))
                except (ValueError,KeyError,IndexError) as error:
                    self.send_json(dict(error=str(error)),400)
            elif self.path=='/api/plan':
                with simulation.lock:
                    self.send_json(simulation.plan)
            elif self.path=='/api/plan/isolation-candidate':
                try:
                    candidate = json.loads((ROOT/'missions/fleet_plan_isolation_corridor_candidate.json').read_text(encoding='utf-8'))
                    if candidate['map_frame']['map_origin_id']!=simulation.plan['map_frame']['map_origin_id']:
                        raise ValueError('候选与当前地图原点不一致，需要重新生成候选')
                    self.send_json(candidate)
                except (ValueError,KeyError,OSError) as error:
                    self.send_json(dict(error=str(error)),400)
            elif self.path=='/api/state':
                self.send_json(simulation.snapshot())
            elif urlparse(self.path).path=='/api/camera/frame':
                vehicle_id = parse_qs(urlparse(self.path).query).get('vehicle_id',[None])[0]
                self.send_camera_frame(vehicle_id)
            elif urlparse(self.path).path=='/api/gimbal/command':
                try:
                    vehicle_id = parse_qs(urlparse(self.path).query).get('vehicle_id',[None])[0]
                    self.send_json(simulation.gimbal_command(vehicle_id))
                except ValueError as error:
                    self.send_json(dict(error=str(error)),400)
            else:
                super().do_GET()

        def do_POST(self):
            origin = self.headers.get('Origin')
            if origin and urlparse(origin).netloc != self.headers.get('Host'):
                self.send_json(dict(error='拒绝跨源控制'),403)
                return
            if self.path not in ('/api/control','/api/gimbal/result','/api/gimbal/status','/api/gimbal/retry','/api/perception/result','/api/manual-draft','/api/mission-waypoints'):
                self.send_json(dict(error='未知接口'),404)
                return
            try:
                size = int(self.headers.get('Content-Length','0'))
                if not 0 < size <= (1048576 if self.path=='/api/manual-draft' else 65536):
                    raise ValueError('请求长度非法')
                data = json.loads(self.rfile.read(size))
                if not isinstance(data,dict):
                    raise ValueError('请求必须为JSON对象')
                if self.path=='/api/manual-draft':
                    with simulation.lock:
                        self.send_json(manual_drafts.save(data,simulation.plan))
                    return
                if self.path=='/api/mission-waypoints':
                    with simulation.lock:
                        self.send_json(mission_waypoints_store.save(data,simulation.plan))
                    return
                if self.path=='/api/gimbal/result':
                    data['action'] = 'observation_result'
                elif self.path=='/api/gimbal/status':
                    data['action'] = 'scan_execution_status'
                elif self.path=='/api/gimbal/retry':
                    data['action'] = 'retry_observation'
                elif self.path=='/api/perception/result':
                    data['action'] = 'perception_result'
                self.send_json(simulation.control(data))
            except (ValueError,TypeError,KeyError,OSError) as error:
                self.send_json(dict(error=str(error)),400)

        def log_message(self,*args):
            pass
    server = ThreadingHTTPServer((host,port),Handler)
    def tick():
        previous = time.monotonic()
        while True:
            time.sleep(getattr(simulation,'tick_interval',.05))
            current = time.monotonic()
            simulation.advance(min(1,current-previous))
            previous = current
    threading.Thread(target=tick,daemon=True).start()
    print('仿真网页: http://{}:{} / {}'.format(host,port,simulation.snapshot()['mode']),flush=True)
    try:
        server.serve_forever()
    finally:
        with simulation.lock:
            simulation.running = False
            simulation.checkpoint()
        server.server_close()


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=8765)
    arguments = parser.parse_args()
    serve(arguments.host,arguments.port)
