from __future__ import annotations
import json
import math
import random
from copy import deepcopy
from fleet_core import ROOT, build_plan
from geodesy import LocalCartesian
from gimbal_geometry import attitude_ned_frd_to_mission_flu, matrix_quaternion, quaternion_matrix, rotate, solve_pointing


def close(first, second):
    assert max(abs(left-right) for left,right in zip(first,second))<1e-8,(first,second)


def reject(callback):
    try:
        callback()
    except ValueError:
        return
    raise AssertionError('应拒绝无效输入')


def main():
    configuration = json.loads((ROOT/'config/gimbal_geometry.json').read_text(encoding='utf-8'))
    frame = LocalCartesian(33.8642,113.7059,71)
    north_heading = attitude_ned_frd_to_mission_flu(0,0,0,frame,frame)
    close(rotate(quaternion_matrix(north_heading),[1,0,0]),[0,1,0])
    close(rotate(quaternion_matrix(north_heading),[0,1,0]),[-1,0,0])
    nearby_frame = LocalCartesian(33.865,113.708,72)
    nearby_quaternion = attitude_ned_frd_to_mission_flu(0,0,0,frame,nearby_frame)
    expected = [sum(coefficient*value for coefficient,value in zip(row,nearby_frame.rotation[1])) for row in frame.rotation]
    close(rotate(quaternion_matrix(nearby_quaternion),[1,0,0]),expected)
    for heading,expected in ((math.pi/2,[1,0,0]),(math.pi,[0,-1,0]),(-math.pi/2,[-1,0,0])):
        quaternion = attitude_ned_frd_to_mission_flu(0,0,heading,frame,frame)
        close(rotate(quaternion_matrix(quaternion),[1,0,0]),expected)
    pitch_up = attitude_ned_frd_to_mission_flu(0,math.pi/2,0,frame,frame)
    close(rotate(quaternion_matrix(pitch_up),[1,0,0]),[0,0,1])
    roll_right_down = attitude_ned_frd_to_mission_flu(math.pi/2,0,0,frame,frame)
    close(rotate(quaternion_matrix(roll_right_down),[0,1,0]),[0,0,1])
    for unused in range(100):
        values = [random.Random(unused).uniform(-1,1),random.Random(unused+101).uniform(-1,1),
                  random.Random(unused+202).uniform(-1,1),random.Random(unused+303).uniform(-1,1)]
        norm = math.sqrt(sum(value*value for value in values))
        quaternion = [value/norm for value in values]
        matrix = quaternion_matrix(quaternion)
        recovered = quaternion_matrix(matrix_quaternion(matrix))
        for row,recovered_row in zip(matrix,recovered):
            close(row,recovered_row)
    forward = solve_pointing([0,40,0],[0,0,40],north_heading,configuration)
    assert abs(forward['azimuth_deg'])<1e-8 and abs(forward['elevation_deg']+45)<1e-8
    left = solve_pointing([-40,0,0],[0,0,40],north_heading,configuration)
    assert abs(left['azimuth_deg']-90)<1e-8
    rear = solve_pointing([0,-40,0],[0,0,40],north_heading,configuration)
    assert not rear['nominal_reachable'] and abs(abs(rear['azimuth_deg'])-180)<1e-8
    nadir = solve_pointing([0,0,0],[0,0,40],north_heading,configuration,.5)
    assert nadir['nadir_azimuth_underdetermined'] and abs(nadir['azimuth_rad']-.5)<1e-8
    assert abs(nadir['elevation_deg']+90)<1e-8
    mounted = deepcopy(configuration)
    mounted['mount_translation_body_flu_m'] = [1,2,3]
    close(solve_pointing([0,40,0],[0,0,40],north_heading,mounted)['pivot_enu_m'],[-2,1,43])
    mounted['mount_translation_body_flu_m'] = [0,0,0]
    mounted['mount_quaternion_body_from_gimbal_xyzw'] = [0,0,math.sin(math.pi/4),math.cos(math.pi/4)]
    assert abs(solve_pointing([0,40,0],[0,0,40],north_heading,mounted)['azimuth_deg']+90)<1e-8
    reject(lambda:solve_pointing([0,0,0],[0,0,0],north_heading,configuration))
    reject(lambda:solve_pointing([float('nan'),0,0],[0,0,40],north_heading,configuration))
    reject(lambda:quaternion_matrix([0,0,0,0]))
    reject(lambda:quaternion_matrix([False,0,0,1]))
    plan = build_plan()
    invalid = []
    total = 0
    for vehicle in plan['vehicles']:
        for segment in vehicle['segments']:
            if segment['state']!='SCAN':
                continue
            total += 1
            pointing = solve_pointing(segment['target'],segment['destination'],north_heading,configuration)
            if not pointing['nominal_reachable']:
                invalid.append(dict(command_id=segment['command_id'],azimuth_deg=pointing['azimuth_deg'],
                    elevation_deg=pointing['elevation_deg']))
    report = dict(ned_frd_to_enu_flu_basis_verified=True,distinct_ekf_origin_rotation_verified=True,quaternion_roundtrip_cases=100,
        body_attitude_and_mount_offset_verified=True,nadir_handled_without_yaw_jump=True,
        unreachable_not_silently_clipped=True,invalid_inputs_rejected=True,
        assumption='固定正北机头、名义零安装外参；不是实际逐帧姿态或实机机械验收',
        total_scan_points=total,nominal_unreachable_count=len(invalid),unreachable_examples=invalid[:12],
        all_planned_observations_nominally_reachable=not invalid,real_mount_calibrated=False,
        sdk_axis_mapping_verified=False,hardware_verified=False)
    (ROOT/'validation/gimbal_geometry_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
