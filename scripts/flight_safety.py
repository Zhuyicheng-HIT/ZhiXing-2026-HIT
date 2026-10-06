from __future__ import annotations
import math
from shapely.geometry import Point, LineString, shape
from shapely.ops import unary_union


class FlightSafety:
    work_states = {'NAVIGATE','STABILIZE','SCAN','CONFIRM_STATIC','TRACK_MOVING',
        'WAIT_RETURN_SLOT','DESCEND_TO_WORK','REPOSITION_CLIMB','RETURN_CLIMB'}
    launch_states = {'CLIMB','TAKEOFF','WAIT_RELEASE','WAIT_ENTRY_PATH','RETURN_DESCEND','LAND','COMPLETE'}

    def __init__(self, plan):
        self.perimeter = shape(plan['perimeter'])
        self.launch = shape(plan['launch'])
        self.forest = shape(plan['forest'])
        self.work_regions = [shape(vehicle['flight_region']) for vehicle in plan['vehicles']]
        self.parameters = plan['flight_safety']
        self.airspace_policy = plan.get('airspace_policy',{})
        radius = self.parameters['estimated_horizontal_body_radius_m']
        self.perimeter_centers = self.perimeter.buffer(-radius,join_style=2)
        self.launch_centers = self.launch.buffer(-radius,join_style=2)
        self.work_centers = [region.buffer(-radius,join_style=2) for region in self.work_regions]
        self.forest_envelope = self.forest.buffer(radius,join_style=2)
        self.isolation_envelope = unary_union([shape(partition['buffer']) for partition in plan['partitions']]).buffer(radius,join_style=2)
        self.checks = 0
        self.rejections = 0
        self.last_violation = None

    def check(self, index, state, position, destination, forest_permission=False,isolation_transit_simulation=False):
        self.checks += 1
        violations = []
        for name,value in (('POSITION',position),('DESTINATION',destination)):
            if len(value)!=3 or any(isinstance(component,bool) or not isinstance(component,(int,float)) or not math.isfinite(component) for component in value):
                violations.append(name+'_NONFINITE')
        if violations:
            return self.result(index,state,violations)
        current,target = Point(position[:2]),Point(destination[:2])
        path = LineString([position[:2],destination[:2]]) if current!=target else current
        if not self.perimeter.covers(current):
            violations.append('POSITION_OUTSIDE_PERIMETER')
        elif not self.perimeter_centers.covers(current):
            violations.append('POSITION_ENVELOPE_CROSSES_PERIMETER')
        if not self.perimeter.covers(path):
            violations.append('SETPOINT_PATH_OUTSIDE_PERIMETER')
        elif not self.perimeter_centers.covers(path):
            violations.append('SETPOINT_ENVELOPE_CROSSES_PERIMETER')
        if state in self.work_states:
            region = self.work_regions[index]
            if not region.covers(current):
                violations.append('POSITION_OUTSIDE_WORK_REGION')
            elif not self.work_centers[index].covers(current):
                violations.append('POSITION_ENVELOPE_CROSSES_WORK_BOUNDARY')
            if not region.covers(path):
                violations.append('SETPOINT_PATH_OUTSIDE_WORK_REGION')
            elif not self.work_centers[index].covers(path):
                violations.append('SETPOINT_ENVELOPE_CROSSES_WORK_BOUNDARY')
        elif state in self.launch_states:
            if not self.launch.covers(path):
                violations.append('VERTICAL_LAUNCH_PATH_OUTSIDE_LAUNCH')
            elif not self.launch_centers.covers(path):
                violations.append('VERTICAL_LAUNCH_ENVELOPE_CROSSES_LAUNCH')
        if not (forest_permission or self.airspace_policy.get('forest_transit_authorized',False)) and self.forest_envelope.intersects(path):
            violations.append('FOREST_TRANSIT_NOT_AUTHORIZED')
        if self.airspace_policy.get('isolation_scope','ALL_ALTITUDES')=='ALL_ALTITUDES' and not isolation_transit_simulation and self.isolation_envelope.intersects(path):
            violations.append('ALL_ALTITUDE_ISOLATION_BAND_NOT_AUTHORIZED')
        for name,value,point in (('POSITION',position,current),('DESTINATION',destination,target)):
            if value[2]>self.parameters['maximum_altitude_m']:
                violations.append(name+'_ABOVE_ALTITUDE_CEILING')
            if not self.launch.covers(point) and value[2]<self.parameters['minimum_operating_altitude_m']:
                violations.append(name+'_BELOW_OPERATING_FLOOR')
        return self.result(index,state,violations)

    def result(self, index, state, violations):
        if violations:
            self.rejections += 1
            self.last_violation = dict(vehicle_index=index,state=state,violations=violations)
        return dict(allowed=not violations,violations=violations)

    def snapshot(self):
        return dict(checks=self.checks,rejections=self.rejections,last_violation=self.last_violation,
            exact_work_boundaries=True,expanded_audit_boundary_m=0.0,
            estimated_horizontal_body_radius_m=self.parameters['estimated_horizontal_body_radius_m'],
            isolation_band_scope='ALL_ALTITUDES_UNLESS_EXPLICIT_SIMULATION_OVERRIDE',
            scope='sampled feedback and requested straight setpoint path; not swept physical braking proof',
            hardware_authorized=False)
