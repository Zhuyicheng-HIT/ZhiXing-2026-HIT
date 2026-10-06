from __future__ import annotations
import math
from scan_motion import scan_look_target
import time
from pymavlink import mavutil
from shapely.geometry import Point, shape
from gimbal_geometry import body_heading, plan_level_hover_heading, plan_ordered_hover_heading, wrap_angle
from scan_protocol import finite_number
from flight_safety import FlightSafety
from path_reservations import segment_distance
from fcu_timing import mission_elapsed_seconds


class WaypointExecutor:
    def __init__(self, fleet):
        self.fleet = fleet
        self.begun = False
        self.complete = False
        self.cursors = [0]*6
        self.elapsed = [0.0]*6
        self.previous_boot = [0]*6
        self.last_land_command = [0.0]*6
        self.hover_until = [0]*6
        self.entered = [False]*6
        self.search_done = [False]*6
        self.return_queue = []
        self.holds = None
        self.hold_yaws = None
        self.hold_recovery_audit = dict(wait_checks=0,resumed_after_actual_settlement=0,
            maximum_observed_hold_drift_m=0.0,scope='sampled position and speed; not braking envelope proof')
        self.observation_holds = [None]*6
        self.scan_alignment = [None]*6
        self.station_heading = [None]*6
        self.approach_heading = [None]*6
        self.scan_wait_state = [None]*6
        self.alignment_parameters = fleet.gimbal_geometry['simulation_heading_alignment']
        self.prepare_heading_during_approach = self.alignment_parameters.get('prepare_heading_during_approach',False)
        if not isinstance(self.prepare_heading_during_approach,bool):
            raise ValueError('prepare_heading_during_approach must be boolean')
        for name in ('arrival_tolerance_deg','maximum_angular_speed_deg_s','stable_duration_s'):
            if finite_number(self.alignment_parameters.get(name),name)<=0:
                raise ValueError(name+'必须为正数')
        self.scan_audit = dict(heading_targets_planned=0,station_heading_plans=0,station_heading_reuses=0,alignment_wait_fcu_s=0.0,
            approach_heading_plans=0,approach_heading_uses=0,
            observations_after_heading_alignment=0,actual_pointing_outside_nominal_limits=0,
            outside_nominal_examples=[],observations_with_feasible_pointing=0,
            max_executable_pointing_error_deg=0.0)
        self.transit_owner = None
        self.layered_entry = fleet.plan['coordination']['entry_policy']=='layered_reserved'
        self.layered_return = fleet.plan['coordination'].get('return_policy','exclusive')=='layered_reserved'
        if self.layered_return and not self.layered_entry:
            raise ValueError('Reserved returns require layered command reservations')
        self.return_released = [False]*6
        self.reservation_distance = fleet.plan['coordination']['minimum_command_path_distance_m']
        self.entry_released = [False]*6
        self.entry_handed_off = [False]*6
        self.reservation_destinations = [None]*6
        self.reservation_keys = [None]*6
        self.reservation_holds = [None]*6
        self.reservation_waits = [None]*6
        self.reservation_audit = dict(grants=0,blocked_checks=0,minimum_granted_path_distance_m=None,
            maximum_parallel_entries=0,maximum_parallel_returns=0,
            scope='remaining straight command segments; not physical tracking or braking proof')
        entry_order = fleet.plan['coordination']['entry_order']
        self.entry_order = [next(index for index,vehicle in enumerate(fleet.plan['vehicles'])
            if vehicle['id']==vehicle_id) for vehicle_id in entry_order]
        self.entry_cursor = 0
        self.return_cursor = 0
        self.entry_end = [next(index+1 for index,segment in enumerate(vehicle['segments'])
            if segment['state']=='DESCEND_TO_WORK') for vehicle in fleet.plan['vehicles']]
        self.minimum_separation = math.inf
        self.samples = 0
        self.phase_elapsed_s = [{} for _ in range(6)]
        self.perimeter = shape(fleet.plan['perimeter'])
        self.launch = shape(fleet.plan['launch'])
        self.work_regions = [shape(vehicle['flight_region']) for vehicle in fleet.plan['vehicles']]
        self.flight_safety = FlightSafety(fleet.plan)
        self.audit = dict(position_samples=0,max_outside_perimeter_m=0.0,max_outside_work_region_m=0.0,
            min_height_outside_launch_m=None,max_height_m=0.0,min_perimeter_clearance_m=None,
            min_work_region_clearance_m=None)

    def begin(self):
        self.begun = True
        for index,vehicle in enumerate(self.fleet.plan['vehicles']):
            self.cursors[index] = next(cursor for cursor,segment in enumerate(vehicle['segments']) if segment['state']=='CLIMB')
            self.previous_boot[index] = self.fleet.telemetry[index]['boot_ms']
            rank = self.entry_order.index(index)
            delay = rank*self.fleet.plan['coordination']['entry_release_interval_s'] if self.layered_entry else 0
            self.hover_until[index] = self.previous_boot[index]+3000+round(delay*1000)
        self.fleet.emit('HOVER_CONFIRMED','开始2m悬停等待；按配置使用分层预约或独占进场，实际到达事件推进')

    def segment(self, index):
        segments = self.fleet.plan['vehicles'][index]['segments']
        return segments[min(self.cursors[index],len(segments)-1)]

    def advance_cursor(self, index):
        vehicle = self.fleet.plan['vehicles'][index]
        previous = self.segment(index)['state']
        self.cursors[index] += 1
        self.elapsed[index] = 0.0
        self.scan_alignment[index] = None
        self.scan_wait_state[index] = None
        if previous=='CLIMB' and not self.entered[index] and self.layered_entry:
            self.fleet.emit('ENTRY_LAYER_REACHED','实际到达入场层；预约本机首段进场，其他机按两秒时隙独立申请爬升',vehicle_id=vehicle['id'])
        if not self.entered[index] and self.cursors[index]>=self.entry_end[index]:
            self.entered[index] = True
            if not self.layered_entry:
                self.transit_owner = None
                self.entry_cursor += 1
            self.fleet.emit('ENTRY_SLOT_RELEASED','实际下降到作业区并稳定，完成本机入场',vehicle_id=vehicle['id'])
        if previous=='DESCEND_TO_WORK' and self.entered[index] and self.transit_owner==index:
            self.transit_owner = None
        if previous=='LAND':
            if self.transit_owner==index:
                self.transit_owner = None
            self.return_released[index] = False
            self.reservation_destinations[index] = self.fleet.telemetry[index]['position'][:]
            self.reservation_keys[index] = None
            self.reservation_holds[index] = None
            self.reservation_waits[index] = None
            self.return_cursor += 1
            self.return_queue.remove(index)
            self.fleet.emit('LANDED_DISARMED','实际落地并解除锁定，释放本机返场路径',vehicle_id=vehicle['id'])
        if self.cursors[index]<len(vehicle['segments']):
            self.fleet.emit(self.segment(index)['state'],'航点事件推进',vehicle_id=vehicle['id'])

    def allowed(self, index):
        fleet = self.fleet
        segment = self.segment(index)
        state = segment['state']
        if not self.entered[index]:
            if self.fleet.telemetry[index]['boot_ms'] < self.hover_until[index]:
                return False
            if self.layered_entry:
                if not self.entry_released[index]:
                    self.entry_released[index] = True
                    parallel = sum(released and not entered for released,entered in zip(self.entry_released,self.entered))
                    self.reservation_audit['maximum_parallel_entries'] = max(self.reservation_audit['maximum_parallel_entries'],parallel)
            else:
                if self.entry_cursor>=6 or self.entry_order[self.entry_cursor]!=index:
                    return False
                if self.transit_owner not in (None,index):
                    return False
                self.transit_owner = index
        if state=='WAIT_RETURN_SLOT':
            self.search_done[index] = True
            if index not in self.return_queue:
                self.return_queue.append(index)
            if not all(self.entered):
                return False
            if not self.layered_return:
                if self.return_queue[0]!=index or self.transit_owner not in (None,index):
                    return False
                self.transit_owner = index
            self.return_released[index] = True
            parallel = sum(self.return_released)
            self.reservation_audit['maximum_parallel_returns'] = max(self.reservation_audit['maximum_parallel_returns'],parallel)
            self.advance_cursor(index)
            return False
        if state in ('REPOSITION_CLIMB','RETURN_CLIMB'):
            if self.layered_entry and not all(self.entered):
                return False
            if state=='RETURN_CLIMB' and self.layered_return:
                return True
            if self.transit_owner not in (None,index):
                return False
            self.transit_owner = index
        return True

    def waiting_for_entry(self, index):
        if self.entered[index]:
            return False
        if self.fleet.telemetry[index]['boot_ms']<self.hover_until[index]:
            return True
        if self.layered_entry:
            return not self.entry_released[index]
        return self.entry_cursor>=6 or self.entry_order[self.entry_cursor]!=index

    def waiting_for_transit(self, index):
        if self.segment(index)['state']=='RETURN_CLIMB' and self.layered_return:
            return not all(self.entered)
        return self.segment(index)['state'] in ('REPOSITION_CLIMB','RETURN_CLIMB') and (
            self.transit_owner not in (None,index) or (self.layered_entry and not all(self.entered)))

    def reserve_path(self, index, destination):
        if not self.layered_entry:
            return True
        if self.reservation_keys[index]==self.cursors[index]:
            return True
        start = self.fleet.telemetry[index]['position']
        minimum = math.inf
        blocker = None
        for other_index,other in enumerate(self.fleet.telemetry):
            if other_index==index:
                continue
            end = self.reservation_destinations[other_index]
            distance = segment_distance(start,destination,other['position'],end if end is not None else other['position'])
            if distance<minimum:
                minimum,blocker = distance,other_index
        if minimum<self.reservation_distance:
            self.reservation_audit['blocked_checks'] += 1
            self.reservation_waits[index] = dict(vehicle_id=self.fleet.plan['vehicles'][blocker]['id'],distance_m=minimum)
            if self.reservation_holds[index] is None:
                self.reservation_holds[index] = start[:]
            self.reservation_destinations[index] = self.reservation_holds[index][:]
            self.elapsed[index] = 0.0
            alignment = self.scan_alignment[index]
            if alignment:
                alignment['ready'],alignment['stable_s'] = False,0.0
            self.fleet.setpoint(index,self.reservation_holds[index])
            return False
        self.reservation_audit['grants'] += 1
        previous = self.reservation_audit['minimum_granted_path_distance_m']
        self.reservation_audit['minimum_granted_path_distance_m'] = minimum if previous is None else min(previous,minimum)
        self.reservation_destinations[index] = list(destination)
        self.reservation_keys[index] = self.cursors[index]
        self.reservation_holds[index] = None
        self.reservation_waits[index] = None
        if not self.entered[index] and self.segment(index)['state']=='INGRESS' and not self.entry_handed_off[index]:
            self.entry_handed_off[index] = True
            self.entry_cursor += 1
            self.fleet.emit('ENTRY_PIPELINE_RELEASED','本机首段进场获预约；不再以此阻塞下一架两秒放行',vehicle_id=self.fleet.plan['vehicles'][index]['id'])
        return True

    def hold_all(self):
        if self.holds is None:
            for alignment in self.scan_alignment:
                if alignment:
                    alignment['stable_s'] = 0.0
                    alignment['ready'] = False
            self.holds = [item['position'][:] for item in self.fleet.telemetry]
            for index,target in enumerate(self.reservation_destinations):
                if target is None:
                    self.reservation_destinations[index] = self.holds[index][:]
            self.hold_yaws = []
            for item in self.fleet.telemetry:
                attitude = item.get('attitude')
                try:
                    heading = body_heading(attitude['quaternion_mission_from_body_flu_xyzw']) if attitude and time.monotonic()-attitude['received_monotonic_s']<=2 else None
                except ValueError:
                    heading = None
                self.hold_yaws.append(heading)
        for index,item in enumerate(self.fleet.telemetry):
            if item['received'] and item['armed'] and item['mode']=='GUIDED':
                self.fleet.setpoint(index,self.holds[index],yaw_mission_rad=self.hold_yaws[index])
                self.reservation_keys[index] = None
                self.hold_recovery_audit['maximum_observed_hold_drift_m'] = max(
                    self.hold_recovery_audit['maximum_observed_hold_drift_m'],math.dist(item['position'],self.holds[index]))
                self.reservation_holds[index] = None
                self.reservation_waits[index] = None

    def release_settled_hold(self):
        if self.holds is None:
            return True
        unsettled = any(item['received'] and item['armed'] and item['mode']=='GUIDED' and (
            math.dist(item['position'],self.holds[index])>=.8 or item.get('speed_m_s',math.inf)>=.6)
            for index,item in enumerate(self.fleet.telemetry))
        if unsettled:
            self.hold_recovery_audit['wait_checks'] += 1
            self.hold_all()
            return False
        for index,item in enumerate(self.fleet.telemetry):
            if item['received'] and item['armed'] and item['mode']=='GUIDED':
                self.reservation_destinations[index] = self.holds[index][:]
                self.reservation_keys[index] = None
        self.holds = None
        self.hold_yaws = None
        self.hold_recovery_audit['resumed_after_actual_settlement'] += 1
        self.fleet.emit('HOLD_SETTLED','GUIDED保持点实际到位且低速；重新预约原航段，不跳过任务')
        return True

    def hold_observation(self, index):
        item = self.fleet.telemetry[index]
        if self.observation_holds[index] is None:
            attitude = item.get('attitude')
            heading = body_heading(attitude['quaternion_mission_from_body_flu_xyzw']) if attitude and time.monotonic()-attitude['received_monotonic_s']<=2 else None
            self.observation_holds[index] = (item['position'][:],heading)
        if item['received'] and item['armed'] and item['mode']=='GUIDED':
            position,heading = self.observation_holds[index]
            self.fleet.setpoint(index,position,yaw_mission_rad=heading)

    def prepare_approach_heading(self, index, now):
        if not self.prepare_heading_during_approach:
            return None
        segments = self.fleet.plan['vehicles'][index]['segments']
        upcoming_scan = None
        for cursor in range(self.cursors[index],len(segments)):
            candidate = segments[cursor]
            if candidate['state']=='SCAN':
                upcoming_scan = cursor
                break
            if candidate['state'] not in ('NAVIGATE','DESCEND_TO_WORK','STABILIZE'):
                return None
        if upcoming_scan is None:
            return None
        attitude = self.fleet.telemetry[index].get('attitude')
        if not attitude or now-attitude['received_monotonic_s']>2:
            return None
        scan = segments[upcoming_scan]
        station_key = [scan.get('station'),scan['destination']]
        prepared = self.approach_heading[index]
        if not prepared or prepared['command_id']!=scan['command_id'] or prepared['station_key']!=station_key:
            targets = []
            for candidate in segments[upcoming_scan:]:
                if candidate['state']!='SCAN' or [candidate.get('station'),candidate['destination']]!=station_key:
                    break
                targets.append(candidate['target'])
            try:
                heading = body_heading(attitude['quaternion_mission_from_body_flu_xyzw'])
                goal = plan_ordered_hover_heading(targets,scan['destination'],heading,self.fleet.gimbal_geometry)
            except ValueError:
                return None
            prepared = dict(station_key=station_key,command_id=scan['command_id'],goal=goal)
            self.approach_heading[index] = prepared
            self.scan_audit['approach_heading_plans'] += 1
        return prepared['goal']['heading_mission_rad']

    def align_scan(self, index, segment, elapsed_s, now):
        fleet = self.fleet
        item = fleet.telemetry[index]
        attitude = item.get('attitude')
        raw_attitude = item.get('raw_attitude_ned_frd')
        if not attitude or now-attitude['received_monotonic_s']>2 or not raw_attitude:
            self.scan_wait_state[index] = 'SCAN_WAIT_ATTITUDE'
            self.elapsed[index] = 0.0
            if self.scan_alignment[index]:
                self.scan_alignment[index]['stable_s'] = 0.0
                self.scan_alignment[index]['ready'] = False
            return False
        try:
            heading = body_heading(attitude['quaternion_mission_from_body_flu_xyzw'])
            angular_speed = finite_number(raw_attitude.get('angular_speed_rad_s'),'angular_speed_rad_s')
            if angular_speed<0:
                raise ValueError('角速度模长不能为负')
            if self.scan_alignment[index] is None:
                station_key = [segment.get('station'),segment['destination']]
                latched = self.station_heading[index]
                reusable = latched and latched['station_key']==station_key and latched['remaining']>0
                if reusable:
                    goal = plan_level_hover_heading(segment['target'],segment['destination'],latched['goal']['heading_mission_rad'],fleet.gimbal_geometry)
                    reusable = abs(goal['heading_change_rad'])<1e-9
                if reusable:
                    goal.update(policy='REUSE_ORDERED_STATION_HEADING',ordered_prefix_count=latched['remaining'])
                    latched['remaining'] -= 1
                    self.scan_audit['station_heading_reuses'] += 1
                else:
                    segments = fleet.plan['vehicles'][index]['segments']
                    targets = []
                    for upcoming in segments[self.cursors[index]:]:
                        if upcoming['state']!='SCAN' or [upcoming.get('station'),upcoming['destination']]!=station_key:
                            break
                        targets.append(upcoming['target'])
                    prepared = self.approach_heading[index]
                    if prepared and prepared['station_key']==station_key and prepared['command_id']==segment['command_id']:
                        goal = dict(prepared['goal'],policy='PREPARED_DURING_APPROACH')
                        self.approach_heading[index] = None
                        self.scan_audit['approach_heading_uses'] += 1
                    else:
                        goal = plan_ordered_hover_heading(targets,segment['destination'],heading,fleet.gimbal_geometry)
                    self.station_heading[index] = dict(station_key=station_key,goal=dict(goal),remaining=goal['ordered_prefix_count']-1)
                    self.scan_audit['station_heading_plans'] += 1
                self.scan_alignment[index] = dict(goal,command_id=segment['command_id'],stable_s=0.0,ready=False)
                self.scan_audit['heading_targets_planned'] += 1
                fleet.emit('SCAN_HEADING_TARGET','保持扫描站位，先按名义机械范围调整机头',
                    vehicle_id=fleet.plan['vehicles'][index]['id'],command_id=segment['command_id'],
                    heading_mission_deg=goal['heading_mission_deg'])
                elapsed_s = 0.0
        except ValueError as error:
            self.scan_wait_state[index] = 'SCAN_UNOBSERVABLE'
            fleet.faults[fleet.plan['vehicles'][index]['id']] = str(error)
            return False
        alignment = self.scan_alignment[index]
        fleet.setpoint(index,segment['destination'],yaw_mission_rad=alignment['heading_mission_rad'])
        error_deg = abs(math.degrees(wrap_angle(heading-alignment['heading_mission_rad'])))
        alignment['actual_heading_error_deg'] = error_deg
        alignment['actual_angular_speed_deg_s'] = math.degrees(angular_speed)
        previously_ready = alignment['ready']
        pointing = fleet.pointing_display(segment['target'],item['position'],attitude['quaternion_mission_from_body_flu_xyzw'])
        executable = pointing.get('executable',{})
        pointing_feasible = pointing.get('valid',False) and executable.get('feasible',False)
        alignment['executable_pointing'] = executable
        stable = error_deg<=self.alignment_parameters['arrival_tolerance_deg'] and math.degrees(angular_speed)<=self.alignment_parameters['maximum_angular_speed_deg_s']
        stable = stable and pointing_feasible
        alignment['stable_s'] = alignment['stable_s']+elapsed_s if stable else 0.0
        alignment['ready'] = alignment['stable_s']>=self.alignment_parameters['stable_duration_s']
        alignment['became_ready'] = alignment['ready'] and not previously_ready
        self.scan_wait_state[index] = None if alignment['ready'] else 'SCAN_ALIGN'
        if not pointing_feasible:
            self.scan_wait_state[index] = 'SCAN_WAIT_POINTING'
        if not alignment['ready']:
            self.elapsed[index] = 0.0
            self.scan_audit['alignment_wait_fcu_s'] += elapsed_s
        return alignment['ready']

    def record_aligned_observation(self, index, segment):
        self.scan_audit['observations_after_heading_alignment'] += 1
        item = self.fleet.telemetry[index]
        quaternion = item['attitude']['quaternion_mission_from_body_flu_xyzw']
        pointing = self.fleet.pointing_display(segment['target'],item['position'],quaternion)
        executable = pointing.get('executable',{})
        if executable.get('feasible'):
            self.scan_audit['observations_with_feasible_pointing'] += 1
            self.scan_audit['max_executable_pointing_error_deg'] = max(
                self.scan_audit['max_executable_pointing_error_deg'],executable['pointing_error_deg'])
        if not pointing.get('valid') or not pointing.get('nominal_reachable'):
            self.scan_audit['actual_pointing_outside_nominal_limits'] += 1
            if len(self.scan_audit['outside_nominal_examples'])<20:
                self.scan_audit['outside_nominal_examples'].append(dict(command_id=segment['command_id'],
                    target=segment['target'],actual_position=item['position'][:],
                    actual_pointing=pointing))

    def tick(self):
        fleet = self.fleet
        now = time.monotonic()
        delta = [max(0,(item['boot_ms']-self.previous_boot[index])/1000) for index,item in enumerate(fleet.telemetry)]
        self.previous_boot = [item['boot_ms'] for item in fleet.telemetry]
        if self.begun and not self.complete:
            for index,seconds in enumerate(delta):
                phase = self.segment(index)['state']
                if not fleet.running or fleet.faults or (fleet.pending and not fleet.concurrent_observations):
                    phase = 'PAUSED_OR_GROUP_HOLD'
                elif self.holds is not None:
                    phase = 'WAIT_HOLD_STABLE'
                elif fleet.plan['vehicles'][index]['id'] in fleet.pending_by_vehicle:
                    phase = 'WAIT_RESULT'
                elif self.waiting_for_entry(index):
                    phase = 'WAIT_RELEASE'
                elif self.reservation_waits[index]:
                    phase = 'WAIT_PATH_RESERVATION'
                elif self.waiting_for_transit(index):
                    phase = 'WAIT_TRANSIT_SLOT'
                elif self.scan_wait_state[index]:
                    phase = self.scan_wait_state[index]
                totals = self.phase_elapsed_s[index]
                totals[phase] = totals.get(phase,0)+seconds
        ready = all(item['received'] and now-item['last_seen']<2 for item in fleet.telemetry)
        if ready:
            for index,item in enumerate(fleet.telemetry):
                point = Point(item['position'][:2])
                self.audit['position_samples'] += 1
                self.audit['max_outside_perimeter_m'] = max(self.audit['max_outside_perimeter_m'],point.distance(self.perimeter))
                clearance = point.distance(self.perimeter.boundary)
                minimum = self.audit['min_perimeter_clearance_m']
                self.audit['min_perimeter_clearance_m'] = clearance if minimum is None else min(minimum,clearance)
                self.audit['max_height_m'] = max(self.audit['max_height_m'],item['position'][2])
                if not self.launch.covers(point):
                    height = self.audit['min_height_outside_launch_m']
                    self.audit['min_height_outside_launch_m'] = item['position'][2] if height is None else min(height,item['position'][2])
                if self.segment(index)['state'] in FlightSafety.work_states:
                    self.audit['max_outside_work_region_m'] = max(self.audit['max_outside_work_region_m'],point.distance(self.work_regions[index]))
                    clearance = point.distance(self.work_regions[index].boundary)
                    minimum = self.audit['min_work_region_clearance_m']
                    self.audit['min_work_region_clearance_m'] = clearance if minimum is None else min(minimum,clearance)
            separation = min(math.dist(first['position'],second['position'])
                for index,first in enumerate(fleet.telemetry) for second in fleet.telemetry[index+1:])
            self.minimum_separation = min(self.minimum_separation,separation)
            self.samples += 1
            if separation<10:
                fleet.faults['coordination'] = '实际三维距离小于10m；保持断点，不独立RTL'
        if not ready or not fleet.running or fleet.faults or (fleet.pending and not fleet.concurrent_observations):
            self.hold_all()
            return
        if not self.release_settled_hold():
            return
        for index,item in enumerate(fleet.telemetry):
            segment = self.segment(index)
            if segment['state']=='COMPLETE' and not item['armed']:
                continue
            safety = self.flight_safety.check(index,segment['state'],item['position'],segment['destination'],fleet.forest_permission,
                getattr(fleet,'isolation_transit_simulation',False))
            if not safety['allowed']:
                vehicle_id = fleet.plan['vehicles'][index]['id']
                fleet.faults['flight_safety'] = vehicle_id+': '+', '.join(safety['violations'])
                fleet.emit('FLIGHT_BOUNDARY_HOLD','边界或高度检查未通过；保留断点，不自动改道或独立RTL',vehicle_id=vehicle_id,violations=safety['violations'])
                self.hold_all()
                fleet.checkpoint()
                return
        for index,vehicle in enumerate(fleet.plan['vehicles']):
            item = fleet.telemetry[index]
            segment = self.segment(index)
            state = segment['state']
            if state=='COMPLETE' or self.cursors[index]>=len(vehicle['segments']):
                continue
            if vehicle['id'] in fleet.pending_by_vehicle:
                if item['mode']!='GUIDED' or not item['armed']:
                    fleet.faults[vehicle['id']] = '观察等待期间飞控状态改变；保留断点，等待协调恢复'
                    self.hold_all()
                    return
                self.hold_observation(index)
                continue
            self.observation_holds[index] = None
            if not self.allowed(index):
                if item['armed'] and item['mode']=='GUIDED':
                    fleet.setpoint(index,segment['origin'])
                continue
            if not self.reserve_path(index,segment['destination']):
                continue
            if state=='LAND':
                if not item['armed'] and abs(item['position'][2])<.5:
                    self.advance_cursor(index)
                elif now-self.last_land_command[index]>.5:
                    fleet.command(index,mavutil.mavlink.MAV_CMD_NAV_LAND,[])
                    self.last_land_command[index] = now
                continue
            if item['mode']!='GUIDED' or not item['armed']:
                fleet.faults[vehicle['id']] = '飞控状态改变；保留断点，等待协调恢复，不强制重新解锁'
                continue
            destination = segment['destination']
            alignment = self.scan_alignment[index] if state=='SCAN' else None
            yaw = alignment['heading_mission_rad'] if alignment else self.prepare_approach_heading(index,now) if state!='SCAN' else None
            fleet.setpoint(index,destination,yaw_mission_rad=yaw)
            arrived = math.dist(item['position'],destination)<.8 and item.get('speed_m_s',math.inf)<.6
            moving = math.dist(segment['origin'],destination)>.001
            if moving:
                if arrived:
                    self.advance_cursor(index)
                continue
            if not arrived:
                self.elapsed[index] = 0.0
                continue
            if state=='SCAN':
                command_id = segment['command_id']
                if command_id in fleet.completed:
                    self.advance_cursor(index)
                    continue
                if not self.align_scan(index,segment,delta[index],now):
                    continue
                self.elapsed[index] += 0.0 if self.scan_alignment[index]['became_ready'] else delta[index]
                if not fleet.synthetic:
                    self.record_aligned_observation(index,segment)
                    fleet.prepare_scan(vehicle['id'],command_id,item['position'])
                    fleet.emit('WAIT_RESULT','实际到位后等待本机云台完成事件',**fleet.pending_by_vehicle[vehicle['id']])
                    fleet.checkpoint()
                    if fleet.concurrent_observations:
                        self.hold_observation(index)
                        continue
                    self.hold_all()
                    break
                if self.elapsed[index]>=segment['end']-segment['start']:
                    self.record_aligned_observation(index,segment)
                    fleet.completed.add(command_id)
                    fleet.emit('SCAN_RESULT','合成云台完成；飞机到位由真实FCU确认',vehicle_id=vehicle['id'],command_id=command_id)
                    self.advance_cursor(index)
            else:
                self.elapsed[index] += delta[index]
                if state=='TRACK_MOVING' and (vehicle['id'],segment['station']) in fleet.abandoned_tracking:
                    self.elapsed[index] = segment['end']-segment['start']
                if self.elapsed[index]>=segment['end']-segment['start']:
                    self.advance_cursor(index)
        self.complete = all(self.segment(index)['state']=='COMPLETE' and not item['armed']
            for index,item in enumerate(fleet.telemetry))
        if self.complete:
            fleet.stamp = fleet.plan['duration_s']
            fleet.running = False
            fleet.emit('COMPLETE','六机实际到点任务完成，并全部降落解除锁定；视觉/投放仍未实测')
            fleet.checkpoint()
        else:
            fleet.stamp = fleet.plan['duration_s']*self.progress()

    def progress(self):
        fractions = []
        for index,vehicle in enumerate(self.fleet.plan['vehicles']):
            cursor = self.cursors[index]
            if self.segment(index)['state']=='COMPLETE':
                fractions.append(1.0)
            else:
                fractions.append(vehicle['segments'][max(0,cursor-1)]['end']/self.fleet.plan['duration_s'] if cursor else 0)
        return sum(fractions)/6

    def snapshot(self):
        fleet = self.fleet
        now = time.monotonic()
        vehicles = []
        for index,(vehicle,item) in enumerate(zip(fleet.plan['vehicles'],fleet.telemetry)):
            segment = self.segment(index)
            state = segment['state']
            if not item['received']:
                state = 'WAIT_FCU_POSITION'
            elif fleet.preparing:
                state = 'GUIDED_ARM_TAKEOFF'
            elif not self.begun:
                state = 'READY'
            elif now-item['last_seen']>2:
                state = 'WAIT_FCU_RECONNECT'
            elif vehicle['id'] in fleet.faults:
                state = 'RECOVERING'
            elif fleet.faults or (fleet.pending and not fleet.concurrent_observations):
                state = 'GROUP_HOLD'
            elif vehicle['id'] in fleet.pending_by_vehicle:
                state = 'WAIT_RESULT'
            elif fleet.running and self.holds is not None and item['armed'] and item['mode']=='GUIDED':
                state = 'WAIT_HOLD_STABLE'
            elif self.waiting_for_entry(index):
                state = 'WAIT_RELEASE'
            elif self.reservation_waits[index]:
                state = 'WAIT_PATH_RESERVATION'
            elif self.waiting_for_transit(index):
                state = 'WAIT_TRANSIT_SLOT'
            elif segment['state']=='SCAN' and self.scan_wait_state[index]:
                state = self.scan_wait_state[index]
            position = item['position'][:]
            target = scan_look_target(segment,self.elapsed[index])
            attitude = item.get('attitude')
            quaternion = attitude['quaternion_mission_from_body_flu_xyzw'] if attitude and now-attitude['received_monotonic_s']<=2 else None
            yaw,pitch = 0.0,-90.0
            if target:
                east,north,height = [goal-actual for goal,actual in zip(target,position)]
                yaw = math.degrees(math.atan2(north,east))
                pitch = math.degrees(math.atan2(height,math.hypot(east,north)))
            vehicles.append(dict(id=vehicle['id'],position=position,state=state,resume_state=segment['state'],
                station=segment.get('station'),observation=segment.get('observation'),segment_index=self.cursors[index],
                target=target,planned_position=segment['destination'],scan_alignment=self.scan_alignment[index],gimbal=dict(azimuth_enu_deg=yaw,elevation_deg=pitch,
                    pointing=fleet.pointing_display(target,position,quaternion),attitude_source='MAVLINK_ATTITUDE_NED_FRD'),
                completed_scans=sum(command.startswith(vehicle['id']+':') for command in fleet.completed),
                fault=fleet.faults.get(vehicle['id']),telemetry=dict(item,age_s=round(now-item['last_seen'],2) if item['received'] else None)))
        separation = min(math.dist(first['position'],second['position'])
            for index,first in enumerate(fleet.telemetry) for second in fleet.telemetry[index+1:])
        elapsed = mission_elapsed_seconds(fleet.telemetry)
        return dict(stamp=fleet.stamp,running=fleet.running,paused=not fleet.running,speed=fleet.speed,epoch=fleet.epoch,
            pending=fleet.pending,pending_commands=list(fleet.pending_by_vehicle.values()),concurrent_observations=fleet.concurrent_observations,
            faults=fleet.faults,synthetic=fleet.synthetic,forest_permission=fleet.forest_permission,
            isolation_transit_simulation=getattr(fleet,'isolation_transit_simulation',False),
            competition_airspace_compliant=fleet.plan['isolation_airspace']['authorized_policy_respected'] and not fleet.plan['requires_forest_transit_permission'],target_updates=fleet.target_updates,
            vehicles=vehicles,log=fleet.log[-25:],progress=self.progress() if self.begun else 0,
            mode='ardupilot_internal_physics_sitl',execution='actual_waypoint_state_machine',preparing=fleet.preparing,
            map_frame=fleet.plan.get('map_frame'),homes_captured_from_feedback=getattr(fleet,'homes_captured',False),
            actual_min_separation_m=separation,observed_min_separation_m=self.minimum_separation if self.samples else separation,
            remaining_waypoint_distance_m=max(math.dist(vehicle['position'],vehicle['planned_position']) for vehicle in vehicles),
            fcu_mission_elapsed_s=elapsed,transit_owner=fleet.plan['vehicles'][self.transit_owner]['id'] if self.transit_owner is not None else None,
            flight_audit=dict(self.audit),
            flight_safety_audit=self.flight_safety.snapshot(),
            reservation_audit=dict(self.reservation_audit),
            hold_recovery_audit=dict(self.hold_recovery_audit),
            latched_group_hold_enu_m=self.holds,
            entry_policy=fleet.plan['coordination']['entry_policy'],
            return_policy=fleet.plan['coordination'].get('return_policy','exclusive'),
            active_returns=[vehicle['id'] for index,vehicle in enumerate(fleet.plan['vehicles']) if self.return_released[index]],
            command_path_reservations={vehicle['id']:dict(cursor=self.cursors[index],
                authorized=self.reservation_keys[index]==self.cursors[index],
                destination_enu_m=self.reservation_destinations[index],
                latched_hold_enu_m=self.reservation_holds[index])
                for index,vehicle in enumerate(fleet.plan['vehicles'])},
            active_entries=[vehicle['id'] for index,vehicle in enumerate(fleet.plan['vehicles']) if self.entry_released[index] and not self.entered[index]],
            reservation_waits={vehicle['id']:self.reservation_waits[index] for index,vehicle in enumerate(fleet.plan['vehicles']) if self.reservation_waits[index]},
            scan_alignment_audit=dict(self.scan_audit),
            phase_elapsed_s={vehicle['id']:dict(self.phase_elapsed_s[index]) for index,vehicle in enumerate(fleet.plan['vehicles'])},
            all_landed_disarmed=all(item['received'] and not item['armed'] and abs(item['position'][2])<.5 for item in fleet.telemetry))
