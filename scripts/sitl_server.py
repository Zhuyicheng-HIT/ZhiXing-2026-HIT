from __future__ import annotations
import argparse
import atexit
import json
import math
from pathlib import Path
import subprocess
import signal
import time
from pymavlink import mavutil
from fleet_core import ROOT, build_plan
from sim_server import FleetSimulation, serve
from scan_protocol import ScanProtocol
from waypoint_executor import WaypointExecutor
from geodesy import simulation_frame
from flight_coordinates import FlightCoordinates
from gimbal_geometry import attitude_ned_frd_to_mission_flu
from fcu_timing import record_arming_transition
from isolation_airspace import require_isolation_permission


class SitlFleet(FleetSimulation):
    def __init__(self, binary, defaults, speedup=5, mission_config=None):
        self.links = []
        self.children = []
        self.telemetry = []
        self.preparing = False
        self.tick_interval = .01
        self.last_commands = 0
        self.takeoff_requested = False
        super().__init__()
        if mission_config is not None:
            options = json.loads(mission_config.read_text(encoding='utf-8'))
            if set(options)-{'homes','subject','footprint_m'}:
                raise ValueError('任务配置只接受homes、subject、footprint_m')
            self.plan = build_plan(homes=options.get('homes'),subject=options.get('subject',1),footprint=options.get('footprint_m',80))
            self.reset()
        self.speed = speedup
        self.plan['mode'] = 'ardupilot_internal_physics_sitl'
        self.concurrent_observations = True
        self.plan['limitations'] = [text for text in self.plan['limitations'] if '不控制飞控' not in text]
        self.plan['limitations'].append('原生ArduPilot内部物理模型；不是Gazebo动力学或真实云台/YOLO')
        self.mission_frame = simulation_frame(self.plan['datum'])
        self.coordinates = [FlightCoordinates(self.mission_frame,self.plan['datum']['sitl_geoid_undulation_m']) for _ in range(6)]
        self.homes_captured = False
        self.runtime = ROOT/'runtime/sitl'
        self.runtime.mkdir(parents=True,exist_ok=True)
        for index, vehicle in enumerate(self.plan['vehicles']):
            directory = self.runtime/vehicle['id']
            directory.mkdir(exist_ok=True)
            east,north = vehicle['home'][:2]
            datum = self.plan['datum']
            latitude,longitude,ellipsoid_height = self.mission_frame.reverse([east,north,0])
            spawn_msl = ellipsoid_height-datum['sitl_geoid_undulation_m']
            logfile = (directory/'flight.log').open('w')
            process = subprocess.Popen([str(binary),'-M','quad','--defaults',str(defaults),
                '-I',str(index),'--sysid',str(index+1),'--speedup',str(speedup),
                '-O',f'{latitude},{longitude},{spawn_msl},0','--wipe'],
                cwd=directory,stdout=logfile,stderr=subprocess.STDOUT)
            logfile.close()
            self.children.append(process)
            self.links.append(None)
            self.telemetry.append(dict(position=[east,north,0.0],last_seen=0,heartbeat=0,armed=False,
                received=False,acks=[],texts=[],version=None,mode='UNKNOWN',streams_requested=False,
                boot_ms=0,armed_since_ms=None,landed_disarmed_ms=None,speed_m_s=0.0,
                raw_local_ned=None,ekf_origin=None,global_position=None,last_origin_request=0.0,
                raw_attitude_ned_frd=None,attitude=None))
        self.executor = WaypointExecutor(self)
        atexit.register(self.close)
        self.emit('SITL_BOOT','启动六个独立原生SITL；网页启动按钮将请求GUIDED/解锁/同时起飞')

    def close(self):
        for process in self.children:
            if process.poll() is None:
                process.terminate()
        for process in self.children:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        for link in self.links:
            if link:
                link.close()

    def receive(self):
        now = time.monotonic()
        for index, link in enumerate(self.links):
            if self.children[index].poll() is not None:
                self.faults[self.plan['vehicles'][index]['id']] = 'SITL进程退出；见runtime/sitl日志'
                continue
            if link is None:
                try:
                    link = mavutil.mavlink_connection(f'tcp:127.0.0.1:{5760+index*10}',
                        source_system=250,autoreconnect=True,retries=0)
                    self.links[index] = link
                except OSError:
                    continue
            telemetry = self.telemetry[index]
            for unused in range(300):
                message = link.recv_match(blocking=False)
                if message is None:
                    break
                kind = message.get_type()
                if kind == 'HEARTBEAT':
                    armed = bool(message.base_mode & 128)
                    record_arming_transition(telemetry,armed)
                    telemetry.update(heartbeat=now,armed=bool(message.base_mode & 128),
                        mode=mavutil.mode_string_v10(message))
                elif kind == 'LOCAL_POSITION_NED':
                    telemetry.update(raw_local_ned=[message.x,message.y,message.z],
                        last_seen=now,boot_ms=message.time_boot_ms,
                        speed_m_s=math.sqrt(message.vx**2+message.vy**2+message.vz**2))
                    if self.coordinates[index].ekf_frame is not None:
                        telemetry.update(position=self.coordinates[index].ned_to_mission(telemetry['raw_local_ned']),received=True)
                elif kind == 'GPS_GLOBAL_ORIGIN':
                    origin = dict(latitude_deg=message.latitude/1e7,longitude_deg=message.longitude/1e7,altitude_msl_m=message.altitude/1000)
                    if telemetry['ekf_origin'] is not None and telemetry['ekf_origin']!=origin and self.executor.begun:
                        self.faults[self.plan['vehicles'][index]['id']] = 'EKF原点改变；保留断点，需核对统一坐标后恢复'
                    self.coordinates[index].set_ekf_origin(**origin)
                    telemetry['ekf_origin'] = origin
                    if telemetry['raw_local_ned'] is not None:
                        telemetry.update(position=self.coordinates[index].ned_to_mission(telemetry['raw_local_ned']),received=True)
                elif kind == 'GLOBAL_POSITION_INT':
                    telemetry['global_position'] = dict(latitude_deg=message.lat/1e7,longitude_deg=message.lon/1e7,
                        altitude_msl_m=message.alt/1000,relative_altitude_m=message.relative_alt/1000)
                elif kind == 'ATTITUDE':
                    telemetry['raw_attitude_ned_frd'] = dict(roll_rad=message.roll,pitch_rad=message.pitch,yaw_rad=message.yaw,
                        angular_speed_rad_s=math.sqrt(message.rollspeed**2+message.pitchspeed**2+message.yawspeed**2),
                        time_boot_ms=message.time_boot_ms,received_monotonic_s=now)
                elif kind == 'COMMAND_ACK':
                    telemetry['acks'] = (telemetry['acks']+[dict(command=message.command,result=message.result)])[-10:]
                elif kind == 'STATUSTEXT':
                    telemetry['texts'] = (telemetry['texts']+[message.text])[-8:]
                elif kind == 'AUTOPILOT_VERSION':
                    telemetry['version'] = message.flight_sw_version
            if telemetry['heartbeat'] and not telemetry['streams_requested']:
                for message_id in (30,32,33,148):
                    link.mav.command_long_send(index+1,1,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,
                        message_id,50000 if message_id==30 else 100000,0,0,0,0,0)
                telemetry['streams_requested'] = True
            if telemetry['heartbeat'] and telemetry['ekf_origin'] is None and now-telemetry['last_origin_request']>1:
                self.command(index,mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,[49])
                telemetry['last_origin_request'] = now
            raw_attitude = telemetry['raw_attitude_ned_frd']
            if raw_attitude and self.coordinates[index].ekf_frame is not None:
                try:
                    quaternion = attitude_ned_frd_to_mission_flu(raw_attitude['roll_rad'],raw_attitude['pitch_rad'],
                        raw_attitude['yaw_rad'],self.mission_frame,self.coordinates[index].ekf_frame)
                    telemetry['attitude'] = dict(quaternion_mission_from_body_flu_xyzw=quaternion,
                        parent_frame_id='mission_map',child_frame_id=self.plan['vehicles'][index]['id']+'/base_link',
                        time_boot_ms=raw_attitude['time_boot_ms'],received_monotonic_s=raw_attitude['received_monotonic_s'])
                except ValueError:
                    telemetry['attitude'] = None

    def command(self, index, command, values):
        link = self.links[index]
        if link and self.telemetry[index]['heartbeat']:
            link.mav.command_long_send(index+1,1,command,0,*(list(values)+[0]*7)[:7])

    def setpoint(self, index, position, yaw_mission_rad=None):
        link = self.links[index]
        north,east,down = self.coordinates[index].mission_to_ned(position)
        mask = 3576 if yaw_mission_rad is None else 2552
        yaw_ned = 0 if yaw_mission_rad is None else self.coordinates[index].mission_heading_to_ned(yaw_mission_rad)
        link.mav.set_position_target_local_ned_send(0,index+1,1,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,north,east,down,0,0,0,0,0,0,yaw_ned,0)

    def advance(self, elapsed):
        with self.lock:
            self.receive()
            self.refresh_scan_execution()
            now = time.monotonic()
            ready = all(item['received'] and now-item['last_seen'] < 2 for item in self.telemetry)
            if self.preparing:
                if not self.running or self.faults:
                    for index,item in enumerate(self.telemetry):
                        if item['received'] and item['armed'] and item['position'][2]>.5:
                            self.setpoint(index,item['position'])
                    return
                if ready and all(item['armed'] and item['mode']=='GUIDED' and abs(item['position'][2]-2)<.3 and item['speed_m_s']<.4 for item in self.telemetry):
                    self.preparing = False
                    self.executor.begin()
                    self.emit('TAKEOFF_CONFIRMED','六机均由实际位置确认达到2m；进入放行调度')
                else:
                    armed_ready = ready and all(item['armed'] and item['mode']=='GUIDED' for item in self.telemetry)
                    if armed_ready:
                        for index,item in enumerate(self.telemetry):
                            if item['position'][2] < 1.5:
                                self.command(index,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,[0,0,0,0,0,0,2])
                        if not self.takeoff_requested:
                            self.takeoff_requested = True
                            self.emit('TAKEOFF_BROADCAST','六机均解锁后在同一调度周期发出起飞请求；非硬实时同步')
                    if now-self.last_commands <= max(.1,1/self.speed):
                        return
                    for index, item in enumerate(self.telemetry):
                        if not item['heartbeat']:
                            continue
                        self.links[index].set_mode('GUIDED')
                        if not item['armed']:
                            self.command(index,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,[1])
                    self.last_commands = now
                return
            if self.executor.begun:
                self.executor.tick()

    def snapshot(self):
        with self.lock:
            return self.executor.snapshot() if hasattr(self,'executor') else super().snapshot()

    def checkpoint(self):
        super().checkpoint()
        if hasattr(self,'executor'):
            target = ROOT/'runtime/checkpoint.json'
            saved = json.loads(target.read_text(encoding='utf-8'))
            saved['waypoint_execution'] = dict(cursors=self.executor.cursors,elapsed_s=self.executor.elapsed,
                scan_alignment=self.executor.scan_alignment,scan_wait_state=self.executor.scan_wait_state,
                station_heading=self.executor.station_heading,
                scan_alignment_audit=self.executor.scan_audit,
                entered=self.executor.entered,search_done=self.executor.search_done,transit_owner=self.executor.transit_owner,
                return_queue=self.executor.return_queue,
                return_released=self.executor.return_released,
                entry_cursor=self.executor.entry_cursor,return_cursor=self.executor.return_cursor)
            temporary = target.with_suffix('.tmp')
            temporary.write_text(json.dumps(saved,ensure_ascii=False),encoding='utf-8')
            temporary.replace(target)

    def control(self, data):
        with self.lock:
            action = data.get('action')
            if action in ('reset','replan','restore'):
                raise ValueError('SITL运行时不允许改变home/恢复未经核验的存档；停止服务后重新启动')
            if action == 'speed':
                raise ValueError('SITL物理倍率由启动参数--speedup确定，不能只加速任务时间')
            if action == 'start' and not self.running:
                require_isolation_permission(self.plan,data)
                if self.plan['requires_forest_transit_permission'] and not bool(data.get('forest_permission',False)):
                    raise ValueError('严格禁穿树林时区域不连通；需要明确授权条件仿真过境')
                if self.stamp >= self.plan['duration_s']:
                    raise ValueError('本次SITL任务已完成；停止服务后重新启动，不能复用已降落状态')
                if not all(item['received'] and time.monotonic()-item['last_seen']<2 for item in self.telemetry):
                    raise ValueError('等待六机定位反馈；检查runtime/sitl/*/flight.log')
                if not self.homes_captured:
                    if any(item['armed'] for item in self.telemetry):
                        raise ValueError('自动读取起点必须在六机未解锁时完成')
                    homes = [item['position'][:2] for item in self.telemetry]
                    plan = build_plan(homes=homes,subject=self.plan['subject'],footprint=self.plan['footprint_m'])
                    plan['mode'] = self.plan['mode']
                    plan['limitations'] = self.plan['limitations'][:]
                    self.plan = plan
                    self.scan_protocol = ScanProtocol(plan,self.scan_protocol.profile)
                    self.executor = WaypointExecutor(self)
                    self.homes_captured = True
                    self.emit('HOMES_CAPTURED','从六机实际EKF原点和LOCAL_NED读取统一ENU起点，起飞前自动生成并冻结航线')
                super().control(data)
                if not self.executor.begun:
                    self.preparing = True
                return self.snapshot()
            return super().control(data)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary',type=Path,default=ROOT/'vendor/ardupilot-4.7.1/build/sitl/bin/arducopter')
    parser.add_argument('--defaults',type=Path,default=ROOT/'vendor/ardupilot-4.7.1/Tools/autotest/default_params/copter.parm')
    parser.add_argument('--speedup',type=float,default=5)
    parser.add_argument('--port',type=int,default=8766)
    parser.add_argument('--mission-config',type=Path)
    arguments = parser.parse_args()
    if not 1 <= arguments.speedup <= 20:
        parser.error('--speedup允许1到20；不能用网页倍率改动物理时间')
    if not arguments.binary.is_file() or not arguments.defaults.is_file():
        parser.error('未找到目标固件/默认参数，请先运行scripts/setup_sitl.sh或显式指定--binary和--defaults')
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,terminate)
    fleet = SitlFleet(arguments.binary.resolve(),arguments.defaults.resolve(),arguments.speedup,arguments.mission_config)
    try:
        serve(port=arguments.port,simulation=fleet)
    finally:
        fleet.close()
