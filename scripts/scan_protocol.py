from __future__ import annotations
import hashlib
import json
import math
import time
from copy import deepcopy
from scan_motion import scan_motion
from regional_scan import regional_scan


def revision(value):
    encoded = json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def finite_number(value, name):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
        raise ValueError(name+'必须是有限数值')
    return float(value)


OBSERVATION_FIELDS = ('minimum_distinct_frames','minimum_observation_s','maximum_pointing_error_deg',
    'minimum_stable_s','maximum_time_uncertainty_s')


def validate_scan_profile(profile):
    scan_motion([0,0,0],[0,0,0],profile['maximum_ground_scan_speed_m_s'])
    continuous = profile.get('scan_mode')=='CONTINUOUS_SWEEP'
    for name in ('scan_zoom','command_acceptance_window_s','execution_timeout_s',
                 'maximum_pointing_error_deg','maximum_time_uncertainty_s'):
        if finite_number(profile.get(name),name)<=0:
            raise ValueError(name+'必须为正数')
    for name in ('minimum_observation_s','minimum_stable_s'):
        value = finite_number(profile.get(name),name)
        if value<0 or (not continuous and value==0):
            raise ValueError(name+' must be nonnegative for continuous scanning, positive otherwise')
    count = profile.get('minimum_distinct_frames')
    if isinstance(count,bool) or not isinstance(count,int) or count<6:
        raise ValueError('minimum_distinct_frames must be an integer of at least six')
    if not isinstance(profile.get('calibrated'),bool) or not profile.get('calibration_id'):
        raise ValueError('需要明确标定ID及calibrated布尔状态')
    if profile['execution_timeout_s']<=profile['minimum_observation_s']+profile['minimum_stable_s']:
        raise ValueError('Execution timeout must allow stabilization and the full observation window')
    for zone in ('takeoff','house','grass'):
        regional_scan(profile,zone,80,profile['region_scans'][zone].get('work_altitude_m',40.5))


class ScanProtocol:
    def __init__(self, plan, profile):
        self.profile = deepcopy(profile)
        validate_scan_profile(self.profile)
        conditions = {name:self.profile[name] for name in OBSERVATION_FIELDS}
        if plan.get('observation_conditions')!=conditions:
            raise ValueError('Plan observation conditions differ from the execution profile; rebuild the plan')
        self.plan_revision = revision(dict(profile=self.profile,map_frame=plan['map_frame'],subject=plan['subject'],
            airspace_policy=plan.get('airspace_policy'),scan_motion_model=plan.get('scan_motion_model'),
            isolation_airspace=plan.get('isolation_airspace'),flight_safety=plan.get('flight_safety'),coordination=plan.get('coordination'),coverage_planning=plan.get('coverage_planning'),observation_conditions=conditions,footprint_m=plan['footprint_m'],vehicles=[dict(id=vehicle['id'],home=vehicle['home'],
                stations=vehicle['stations'],station_tasks=vehicle.get('station_tasks'),flight_region=vehicle['flight_region']) for vehicle in plan['vehicles']]))
        self.observations = {}
        self.receipts = {}
        self.point_status = {}
        for vehicle in plan['vehicles']:
            expected_scan = regional_scan(self.profile,vehicle['zone'],plan['footprint_m'],vehicle['work_altitude_m'])
            if vehicle.get('scan_configuration')!=expected_scan:
                raise ValueError('Regional scan profile changed; rebuild the plan')
            for segment in vehicle['segments']:
                if segment['state']!='SCAN':
                    continue
                motion = segment.get('scan_motion') or dict(minimum_slew_s=0)
                if not math.isclose(segment['end']-segment['start'],self.profile['minimum_observation_s']+motion['minimum_slew_s'],rel_tol=0,abs_tol=1e-8):
                    raise ValueError('Planned scan duration differs from the observation window')
                content = dict(vehicle_id=vehicle['id'],observation_id=segment['command_id'],
                    station_id='{}:{}'.format(vehicle['id'],segment['station']),
                    command_frame_id=plan['map_frame']['frame_id'],map_origin_id=plan['map_frame']['map_origin_id'],
                    setpoint_type='GROUND_POINT',target_enu_m=segment['target'],
                    station_enu_m=segment['destination'],height_reference='SIMULATION_FLAT_GROUND',
                    calibration_id=self.profile['calibration_id'],scan_zoom=expected_scan['scan_zoom'],
                    instantaneous_frame_m=expected_scan['instantaneous_frame_m'],
                    station_footprint_m=expected_scan['station_footprint_m'],scan_pattern=expected_scan['pattern'],
                    geometry_revision=self.profile.get('geometry_revision'),
                    scan_motion=deepcopy(segment.get('scan_motion')),
                    initial_look_mode=expected_scan['initial_look_mode'] if segment.get('rapid_repoint') or segment['observation']==0 else 'CONTINUE_SWEEP',
                    rapid_repoint_target_enu_m=segment['scan_motion']['origin_enu_m'] if segment.get('rapid_repoint') or segment['observation']==0 else None,
                    observation_conditions={name:self.profile[name] for name in ('minimum_distinct_frames',
                        'minimum_observation_s','maximum_pointing_error_deg','minimum_stable_s',
                        'maximum_time_uncertainty_s')})
                self.observations[segment['command_id']] = dict(content,point_revision=revision(content))

    def issue(self, vehicle_id, command_id, epoch, actual_position=None):
        point = self.observations.get(command_id)
        if point is None or point['vehicle_id']!=vehicle_id:
            raise ValueError('扫描命令不存在或拥有者不匹配')
        now = time.time()
        motion = point.get('scan_motion') or {}
        scan_duration = float(motion.get('minimum_slew_s',0.0)) + self.profile['minimum_observation_s']
        execution_timeout = max(self.profile['execution_timeout_s'], scan_duration + self.profile['minimum_stable_s'] + 10.0)
        command = dict(deepcopy(point),schema='zhixin/gimbal-command/v1',command_id=command_id,
            mission_epoch=epoch,owner='fleet-coordinator',plan_revision=self.plan_revision,
            command_type='SWEEP_SEGMENT' if self.profile.get('scan_mode')=='CONTINUOUS_SWEEP' else 'OBSERVE_POINT',
            issued_at=now,accept_before=now+self.profile['command_acceptance_window_s'],
            execution_timeout_s=execution_timeout,clock_domain='UNIX_UTC_SECONDS',
            actual_position_enu_m=deepcopy(actual_position),hardware_authorized=False,
            calibration_configured=self.profile['calibrated'])
        self.point_status[command_id] = dict(status='PENDING',point_revision=point['point_revision'])
        return command

    def receive(self, payload, epoch, active):
        if not isinstance(payload,dict):
            raise ValueError('扫描结果必须是JSON对象')
        if payload.get('mission_epoch')!=epoch:
            raise ValueError('过期mission_epoch')
        command_id = payload.get('command_id')
        point = self.observations.get(command_id) if isinstance(command_id,str) else None
        if not point or payload.get('vehicle_id')!=point['vehicle_id']:
            raise ValueError('结果拥有者或命令ID不匹配')
        if payload.get('plan_revision')!=self.plan_revision or payload.get('point_revision')!=point['point_revision']:
            raise ValueError('任务或观察点内容版本不匹配，不能沿用旧完成标记')
        key = epoch+':'+command_id
        digest = revision(payload)
        if key in self.receipts:
            if self.receipts[key]['payload_revision']!=digest:
                raise ValueError('重复命令的结果内容冲突；不覆盖已有证据')
            return dict(accepted=True,duplicate=True,completed=self.receipts[key]['completed'])
        if not active or active['command_id']!=command_id or active['mission_epoch']!=epoch:
            raise ValueError('结果不是当前活动扫描命令')
        status = payload.get('status')
        if status in ('UNOBSERVABLE','FAILED','CANCELLED','PAUSED'):
            if not isinstance(payload.get('reason_code'),str) or not payload['reason_code']:
                raise ValueError('未完成观察需要原因码')
            self.point_status[command_id] = dict(status=status,reason_code=payload['reason_code'],
                point_revision=point['point_revision'])
            return dict(accepted=True,duplicate=False,completed=False)
        if status not in ('FINISHED','SUCCESS_FOUND','SUCCESS_NOT_FOUND'):
            raise ValueError('结果状态不能代表观察完成')
        evidence = payload.get('evidence')
        if not isinstance(evidence,dict):
            raise ValueError('观察完成必须带逐点证据')
        kind = evidence.get('kind')
        if kind not in ('SIMULATED_OBSERVATION','REAL_OBSERVATION'):
            raise ValueError('需明确模拟或真实观察证据')
        if kind=='REAL_OBSERVATION' and not self.profile['calibrated']:
            raise ValueError('未配置标定，不能将结果标记为真实有效观察')
        if evidence.get('calibration_id')!=point['calibration_id']:
            raise ValueError('证据标定ID不匹配')
        if abs(finite_number(evidence.get('zoom_actual'),'zoom_actual')-point['scan_zoom'])>1e-6:
            raise ValueError('实际倍率与扫描配置不匹配')
        for name in ('decoded','inference_completed','clarity_passed','attitude_valid','pointing_valid','zoom_valid'):
            if evidence.get(name) is not True:
                raise ValueError(name+'未确认，不能完成观察点')
        frames = evidence.get('source_frames')
        if not isinstance(frames,list) or len(frames)<self.profile['minimum_distinct_frames']:
            raise ValueError('不同源帧数量不足')
        if len(frames)>512:
            raise ValueError('源帧索引过大；只传必要索引，不上传图像')
        identities,timestamps = [],[]
        uncertainty = finite_number(evidence.get('time_uncertainty_s'),'time_uncertainty_s')
        if not 0<=uncertainty<=self.profile['maximum_time_uncertainty_s']:
            raise ValueError('观察时间基准不确定性未满足配置')
        now = time.time()
        for frame in frames:
            if not isinstance(frame,dict) or not isinstance(frame.get('source_session'),str) or not frame['source_session']:
                raise ValueError('源帧需要来源会话')
            sequence = frame.get('sequence')
            if isinstance(sequence,bool) or not isinstance(sequence,int) or sequence<0:
                raise ValueError('源帧序号非法')
            capture = finite_number(frame.get('capture_time'),'capture_time')
            processed = finite_number(frame.get('processed_at'),'processed_at')
            if capture<active['issued_at']-uncertainty or processed<capture or processed>now+uncertainty:
                raise ValueError('源帧时间不属于当前观察或处理时间非法')
            identities.append((frame['source_session'],sequence))
            timestamps.append(capture)
        if len(set(identities))!=len(identities):
            raise ValueError('重复源帧不能算不同观察帧')
        if max(timestamps)-min(timestamps)<self.profile['minimum_observation_s']:
            raise ValueError('实际不同帧覆盖的观察时长不足')
        if finite_number(evidence.get('pointing_error_deg'),'pointing_error_deg')<0 or evidence['pointing_error_deg']>self.profile['maximum_pointing_error_deg']:
            raise ValueError('云台实际指向误差未满足配置')
        if finite_number(evidence.get('stable_duration_s'),'stable_duration_s')<self.profile['minimum_stable_s']:
            raise ValueError('云台稳定时间不足')
        self.receipts[key] = dict(payload_revision=digest,completed=True,evidence=deepcopy(evidence),
            plan_revision=self.plan_revision,point_revision=point['point_revision'],received_at=now)
        self.point_status[command_id] = dict(status='COMPLETED',evidence_kind=kind,point_revision=point['point_revision'])
        return dict(accepted=True,duplicate=False,completed=True,evidence_kind=kind)

    def checkpoint(self):
        return dict(schema='zhixin/scan-checkpoint/v1',plan_revision=self.plan_revision,
            profile_revision=revision(self.profile),receipts=deepcopy(self.receipts),point_status=deepcopy(self.point_status))

    def restore(self, saved):
        if saved['plan_revision']!=self.plan_revision or saved['profile_revision']!=revision(self.profile):
            raise ValueError('扫描断点的任务或观察配置已改变，不能复用旧证据')
        receipts = deepcopy(saved['receipts'])
        states = deepcopy(saved['point_status'])
        for command_id,state in states.items():
            if command_id not in self.observations or state['point_revision']!=self.observations[command_id]['point_revision']:
                raise ValueError('扫描断点的观察点版本不匹配')
        self.receipts,self.point_status = receipts,states
