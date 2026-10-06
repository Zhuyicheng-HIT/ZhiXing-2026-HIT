from __future__ import annotations
import math


def vector(values, size, name):
    if not isinstance(values,(list,tuple)) or len(values)!=size:
        raise ValueError(name+'维度非法')
    if any(isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) for value in values):
        raise ValueError(name+'必须为有限数值')
    return [float(value) for value in values]


def transpose(matrix):
    return [list(column) for column in zip(*matrix)]


def multiply(first, second):
    return [[sum(left*right for left,right in zip(row,column)) for column in zip(*second)] for row in first]


def rotate(matrix, direction):
    return [sum(coefficient*value for coefficient,value in zip(row,direction)) for row in matrix]


def quaternion_matrix(quaternion_xyzw):
    quaternion = vector(quaternion_xyzw,4,'quaternion_xyzw')
    norm = math.sqrt(sum(value*value for value in quaternion))
    if norm<1e-12 or abs(norm-1)>.01:
        raise ValueError('四元数未归一化或无效')
    axis_x,axis_y,axis_z,scalar = [value/norm for value in quaternion]
    return [[1-2*(axis_y**2+axis_z**2),2*(axis_x*axis_y-axis_z*scalar),2*(axis_x*axis_z+axis_y*scalar)],
            [2*(axis_x*axis_y+axis_z*scalar),1-2*(axis_x**2+axis_z**2),2*(axis_y*axis_z-axis_x*scalar)],
            [2*(axis_x*axis_z-axis_y*scalar),2*(axis_y*axis_z+axis_x*scalar),1-2*(axis_x**2+axis_y**2)]]


def matrix_quaternion(matrix):
    candidates = [1+matrix[0][0]-matrix[1][1]-matrix[2][2],
                  1-matrix[0][0]+matrix[1][1]-matrix[2][2],
                  1-matrix[0][0]-matrix[1][1]+matrix[2][2],
                  1+matrix[0][0]+matrix[1][1]+matrix[2][2]]
    largest = max(range(4),key=candidates.__getitem__)
    dominant = math.sqrt(max(0,candidates[largest]))/2
    denominator = 4*dominant
    if largest==0:
        result = [dominant,(matrix[0][1]+matrix[1][0])/denominator,
            (matrix[0][2]+matrix[2][0])/denominator,(matrix[2][1]-matrix[1][2])/denominator]
    elif largest==1:
        result = [(matrix[0][1]+matrix[1][0])/denominator,dominant,
            (matrix[1][2]+matrix[2][1])/denominator,(matrix[0][2]-matrix[2][0])/denominator]
    elif largest==2:
        result = [(matrix[0][2]+matrix[2][0])/denominator,(matrix[1][2]+matrix[2][1])/denominator,
            dominant,(matrix[1][0]-matrix[0][1])/denominator]
    else:
        result = [(matrix[2][1]-matrix[1][2])/denominator,(matrix[0][2]-matrix[2][0])/denominator,
            (matrix[1][0]-matrix[0][1])/denominator,dominant]
    return result if result[3]>=0 else [-value for value in result]


def attitude_ned_frd_to_mission_flu(roll_rad, pitch_rad, yaw_rad, mission_frame, ekf_frame):
    roll,pitch,yaw = vector([roll_rad,pitch_rad,yaw_rad],3,'MAVLink ATTITUDE')
    cosine_roll,sine_roll = math.cos(roll),math.sin(roll)
    cosine_pitch,sine_pitch = math.cos(pitch),math.sin(pitch)
    cosine_yaw,sine_yaw = math.cos(yaw),math.sin(yaw)
    ned_from_frd = [
        [cosine_pitch*cosine_yaw,sine_roll*sine_pitch*cosine_yaw-cosine_roll*sine_yaw,
         cosine_roll*sine_pitch*cosine_yaw+sine_roll*sine_yaw],
        [cosine_pitch*sine_yaw,sine_roll*sine_pitch*sine_yaw+cosine_roll*cosine_yaw,
         cosine_roll*sine_pitch*sine_yaw-sine_roll*cosine_yaw],
        [-sine_pitch,sine_roll*cosine_pitch,cosine_roll*cosine_pitch]]
    enu_from_ned = [[0,1,0],[1,0,0],[0,0,-1]]
    frd_from_flu = [[1,0,0],[0,-1,0],[0,0,-1]]
    mission_from_ekf_enu = multiply(mission_frame.rotation,transpose(ekf_frame.rotation))
    rotation = multiply(mission_from_ekf_enu,multiply(enu_from_ned,multiply(ned_from_frd,frd_from_flu)))
    return matrix_quaternion(rotation)


def solve_pointing(target_enu_m, body_position_enu_m, mission_from_body_xyzw, configuration, preferred_azimuth_rad=0):
    target = vector(target_enu_m,3,'target_enu_m')
    position = vector(body_position_enu_m,3,'body_position_enu_m')
    body_rotation = quaternion_matrix(mission_from_body_xyzw)
    mount_rotation = quaternion_matrix(configuration['mount_quaternion_body_from_gimbal_xyzw'])
    offset = rotate(body_rotation,vector(configuration['mount_translation_body_flu_m'],3,'mount_translation_body_flu_m'))
    pivot = [coordinate+translation for coordinate,translation in zip(position,offset)]
    direction_world = [goal-origin for goal,origin in zip(target,pivot)]
    distance = math.sqrt(sum(value*value for value in direction_world))
    if distance<1e-6:
        raise ValueError('目标与云台旋转参考点重合，指向未定义')
    direction_mount = rotate(transpose(mount_rotation),rotate(transpose(body_rotation),direction_world))
    horizontal = math.hypot(direction_mount[0],direction_mount[1])
    nadir = horizontal/distance<1e-8
    azimuth = vector([preferred_azimuth_rad],1,'preferred_azimuth_rad')[0] if nadir else math.atan2(direction_mount[1],direction_mount[0])
    azimuth = (azimuth+math.pi)%(2*math.pi)-math.pi
    elevation = math.atan2(direction_mount[2],horizontal)
    azimuth_deg,elevation_deg = math.degrees(azimuth),math.degrees(elevation)
    azimuth_limits = vector(configuration['nominal_azimuth_limits_deg'],2,'nominal_azimuth_limits_deg')
    elevation_limits = vector(configuration['nominal_elevation_limits_deg'],2,'nominal_elevation_limits_deg')
    if azimuth_limits[0]>azimuth_limits[1] or elevation_limits[0]>elevation_limits[1]:
        raise ValueError('名义限位上下界倒置')
    reachable = azimuth_limits[0]-1e-7<=azimuth_deg<=azimuth_limits[1]+1e-7 and elevation_limits[0]-1e-7<=elevation_deg<=elevation_limits[1]+1e-7
    return dict(valid=True,reference_frame='gimbal_mount_flu',azimuth_rad=azimuth,elevation_rad=elevation,
        azimuth_deg=azimuth_deg,elevation_deg=elevation_deg,look_direction=[value/distance for value in direction_mount],
        pivot_enu_m=pivot,distance_m=distance,nadir_azimuth_underdetermined=nadir,nominal_reachable=reachable,
        invalid_reason=None if reachable else 'OUTSIDE_NOMINAL_GIMBAL_LIMITS',
        calibration_id=configuration['calibration_id'],mount_calibrated=configuration['calibrated'],
        sdk_axis_mapping_verified=configuration['sdk_axis_mapping_verified'],sdk_command_available=False)


def feasible_pointing(direction_mount, configuration, maximum_error_deg):
    direction = vector(direction_mount,3,'direction_mount')
    norm = math.sqrt(sum(component*component for component in direction))
    if norm<1e-12:
        raise ValueError('指向向量不能为零')
    direction = [component/norm for component in direction]
    tolerance = vector([maximum_error_deg],1,'maximum_error_deg')[0]
    if not 0<=tolerance<=180:
        raise ValueError('视线误差限值必须在0至180度之间')
    yaw_lower,yaw_upper = vector(configuration['nominal_azimuth_limits_deg'],2,'nominal_azimuth_limits_deg')
    elevation_lower,elevation_upper = vector(configuration['nominal_elevation_limits_deg'],2,'nominal_elevation_limits_deg')
    if not -180<=yaw_lower<=yaw_upper<=180 or not -90<=elevation_lower<=elevation_upper<=90:
        raise ValueError('规范yaw/elevation限位超出支持范围或倒置')
    desired_yaw = math.atan2(direction[1],direction[0])
    yaw_bounds = [math.radians(yaw_lower),math.radians(yaw_upper)]
    elevation_bounds = [math.radians(elevation_lower),math.radians(elevation_upper)]
    yaw_candidates = list(yaw_bounds)
    for candidate in [desired_yaw,desired_yaw+2*math.pi,desired_yaw-2*math.pi,0.0]:
        if yaw_bounds[0]<=candidate<=yaw_bounds[1]:
            yaw_candidates.append(candidate)
    best = None
    for yaw in yaw_candidates:
        horizontal_projection = direction[0]*math.cos(yaw)+direction[1]*math.sin(yaw)
        elevation_candidates = list(elevation_bounds)
        desired_elevation = math.atan2(direction[2],horizontal_projection)
        if elevation_bounds[0]<=desired_elevation<=elevation_bounds[1]:
            elevation_candidates.append(desired_elevation)
        for elevation in elevation_candidates:
            command_direction = [math.cos(elevation)*math.cos(yaw),math.cos(elevation)*math.sin(yaw),math.sin(elevation)]
            dot = max(-1.0,min(1.0,sum(actual*desired for actual,desired in zip(command_direction,direction))))
            cross = [command_direction[1]*direction[2]-command_direction[2]*direction[1],
                command_direction[2]*direction[0]-command_direction[0]*direction[2],
                command_direction[0]*direction[1]-command_direction[1]*direction[0]]
            error = math.degrees(math.atan2(math.sqrt(sum(component*component for component in cross)),dot))
            candidate = dict(azimuth_rad=yaw,elevation_rad=elevation,azimuth_deg=math.degrees(yaw),
                elevation_deg=math.degrees(elevation),look_direction=command_direction,
                pointing_error_deg=error)
            if best is None or error<best['pointing_error_deg']:
                best = candidate
    return dict(best,within_nominal_limits=True,maximum_error_deg=tolerance,
        feasible=best['pointing_error_deg']<=tolerance+1e-9,
        reference_frame='gimbal_mount_flu',method='MAXIMUM_VIEW_DIRECTION_DOT_PRODUCT',
        model='NOMINAL_TWO_AXIS_GEOMETRY',sdk_command_available=False,hardware_authorized=False)


def wrap_angle(angle_rad):
    return (angle_rad+math.pi)%(2*math.pi)-math.pi


def body_heading(quaternion_xyzw):
    forward = rotate(quaternion_matrix(quaternion_xyzw),[1,0,0])
    if math.hypot(forward[0],forward[1])<1e-8:
        raise ValueError('机体前向接近垂直，水平航向未定义')
    return math.atan2(forward[1],forward[0])


def plan_level_hover_heading(target, position, current_heading_rad, configuration):
    heading = vector([current_heading_rad],1,'current_heading_rad')[0]
    margin = vector([configuration['simulation_heading_alignment']['nominal_yaw_margin_deg']],1,'nominal_yaw_margin_deg')[0]
    if margin<0:
        raise ValueError('航向规划的名义安全余量不能为负')
    lower,upper = configuration['nominal_azimuth_limits_deg']
    if lower+margin>=upper-margin:
        raise ValueError('云台yaw范围不足以保留名义余量')
    candidates = [0.0]+[math.radians(sign*degrees) for degrees in range(1,181) for sign in (1,-1)]
    for change in candidates:
        proposed = wrap_angle(heading+change)
        quaternion = [0,0,math.sin(proposed/2),math.cos(proposed/2)]
        pointing = solve_pointing(target,position,quaternion,configuration)
        if pointing['nominal_reachable'] and lower+margin<=pointing['azimuth_deg']<=upper-margin:
            return dict(heading_mission_rad=proposed,heading_mission_deg=math.degrees(proposed),
                heading_change_rad=wrap_angle(proposed-heading),nominal_pointing=pointing,
                assumption='LEVEL_HOVER_NOMINAL_MOUNT',hardware_authorized=False)
    raise ValueError('水平悬停条件下无法找到满足名义云台范围的航向；不能跳过观察点')


def plan_ordered_hover_heading(targets, position, current_heading_rad, configuration):
    if not targets:
        raise ValueError('Ordered observation targets must not be empty')
    heading = vector([current_heading_rad],1,'current_heading_rad')[0]
    margin = vector([configuration['simulation_heading_alignment']['nominal_yaw_margin_deg']],1,'nominal_yaw_margin_deg')[0]
    lower,upper = vector(configuration['nominal_azimuth_limits_deg'],2,'nominal_azimuth_limits_deg')
    if margin<0 or lower+margin>=upper-margin:
        raise ValueError('Invalid nominal yaw margin')
    best = None
    candidates = [0.0]+[math.radians(sign*degrees) for degrees in range(1,181) for sign in (1,-1)]
    for change in candidates:
        proposed = wrap_angle(heading+change)
        quaternion = [0,0,math.sin(proposed/2),math.cos(proposed/2)]
        prefix = 0
        first_pointing = None
        for target in targets:
            pointing = solve_pointing(target,position,quaternion,configuration)
            if not pointing['nominal_reachable'] or not lower+margin<=pointing['azimuth_deg']<=upper-margin:
                break
            first_pointing = first_pointing or pointing
            prefix += 1
        if prefix and (best is None or prefix>best['ordered_prefix_count']):
            best = dict(heading_mission_rad=proposed,heading_mission_deg=math.degrees(proposed),
                heading_change_rad=wrap_angle(proposed-heading),nominal_pointing=first_pointing,
                ordered_prefix_count=prefix,policy='LONGEST_CONTIGUOUS_PREFIX_MINIMUM_TURN',
                assumption='LEVEL_HOVER_NOMINAL_MOUNT',hardware_authorized=False)
        if prefix==len(targets):
            break
    if best is None:
        raise ValueError('First ordered target is unreachable; observation order must not be changed')
    return best
