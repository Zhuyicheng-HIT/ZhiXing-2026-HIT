from __future__ import annotations
import argparse
import json
import math
import subprocess
import time
from pathlib import Path
from urllib.request import urlopen
from fleet_core import ROOT
from gimbal_geometry import quaternion_matrix, rotate
from gazebo_replay import pose_message


def state_at(base):
    with urlopen(base+'/api/state',timeout=5) as response:
        return json.load(response)


def position_of(pose):
    return [pose.get('position',{}).get(axis,0) for axis in ('x','y','z')]


def optical_axis(pose):
    quaternion = [pose.get('orientation',{}).get(axis,0) for axis in ('x','y','z','w')]
    return rotate(quaternion_matrix(quaternion),[1,0,0])


def expected_axis(gimbal):
    azimuth = math.radians(gimbal['azimuth_enu_deg'])
    elevation = math.radians(gimbal['elevation_deg'])
    return [math.cos(elevation)*math.cos(azimuth),math.cos(elevation)*math.sin(azimuth),math.sin(elevation)]


def verify_projection():
    cases = [(0,-90),(90,0),(180,-45),(-135,-30),(270,-89),(45,20)]
    for azimuth,elevation in cases:
        message = pose_message('projection',[1,2,3],-math.radians(elevation),math.radians(azimuth))
        quaternion_text = message.split('orientation {',1)[1].split('}',1)[0]
        quaternion = dict(part.split(':',1) for part in quaternion_text.split())
        pose = dict(orientation={axis:float(value) for axis,value in quaternion.items()})
        assert math.dist(optical_axis(pose),expected_axis(dict(azimuth_enu_deg=azimuth,elevation_deg=elevation)))<1e-10
    return len(cases)


def main():
    parser = argparse.ArgumentParser(description='Read-only six-body and six-gimbal Gazebo pose verification; no flight commands.')
    parser.add_argument('--url',default='http://127.0.0.1:8766')
    parser.add_argument('--report',type=Path,default=ROOT/'validation/gazebo_gimbals_report.json')
    arguments = parser.parse_args()
    projection_count = verify_projection()
    before = state_at(arguments.url)
    if before['running'] or any(vehicle.get('telemetry',{}).get('armed',False) for vehicle in before['vehicles']):
        raise ValueError('Verification requires six stationary unarmed vehicles; it never pauses or changes the flight task.')
    assert len(before['vehicles'])==6
    result = subprocess.run(['gz','topic','-e','-n','1','--json-output','-t','/world/zhixin_fleet_kinematic/pose/info'],capture_output=True,text=True,timeout=15,check=True)
    poses = {pose['name']:pose for pose in json.loads(result.stdout)['pose']}
    after = state_at(arguments.url)
    assert not after['running'] and after['epoch']==before['epoch'] and after['stamp']==before['stamp']
    assert [vehicle['segment_index'] for vehicle in after['vehicles']]==[vehicle['segment_index'] for vehicle in before['vehicles']]
    assert all(not vehicle.get('telemetry',{}).get('armed',False) for vehicle in after['vehicles'])
    checks = []
    for vehicle in before['vehicles']:
        subsequent = next(item for item in after['vehicles'] if item['id']==vehicle['id'])
        assert math.dist(vehicle['position'],subsequent['position'])<.01
        assert vehicle['gimbal']['azimuth_enu_deg']==subsequent['gimbal']['azimuth_enu_deg']
        assert vehicle['gimbal']['elevation_deg']==subsequent['gimbal']['elevation_deg']
        body = poses[vehicle['id']]
        camera = poses[vehicle['id']+'_camera']
        expected_body = vehicle['position'][:]
        expected_body[2] += .25
        expected_camera = vehicle['position'][:]
        expected_camera[2] += .14
        body_error = math.dist(position_of(body),expected_body)
        camera_error = math.dist(position_of(camera),expected_camera)
        direction_error = math.dist(optical_axis(camera),expected_axis(vehicle['gimbal']))
        assert body_error<.01 and camera_error<.01 and direction_error<1e-6
        checks.append(dict(vehicle_id=vehicle['id'],body_error_m=body_error,camera_error_m=camera_error,
            optical_axis_error=direction_error,elevation_deg=vehicle['gimbal']['elevation_deg'],
            azimuth_enu_deg=vehicle['gimbal']['azimuth_enu_deg']))
    report = dict(passed=True,checked_at_unix_s=time.time(),mode='gazebo_set_pose_visualization',
        six_bodies_and_six_cameras_verified=True,orientation_projection_case_count=projection_count,
        source_mode=before['mode'],task_epoch_and_cursors_unchanged=True,no_flight_control_requests=True,
        gimbal_joint_dynamics_verified=False,camera_image_generation_verified=False,
        physical_collision_verified=False,browser_visual_verified=False,hardware_verified=False,
        scope='stationary native telemetry position replay and nominal world-frame gimbal display only',vehicles=checks)
    arguments.report.parent.mkdir(parents=True,exist_ok=True)
    arguments.report.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
