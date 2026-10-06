from __future__ import annotations
import argparse
import json
import math
import time
from pathlib import Path
from urllib.error import URLError
from urllib.request import Request, urlopen
from fleet_core import ROOT


def request(url, payload=None):
    data = json.dumps(payload).encode() if payload is not None else None
    with urlopen(Request(url,data=data,headers={'Content-Type':'application/json'}),timeout=60) as response:
        return json.load(response)


def verify(url, timeout, authorize_forest=False, report_path=None, verify_return_recovery=False,authorize_isolation=False):
    state = request(url+'/api/state')
    duration = request(url+'/api/plan')['duration_s']
    if state['mode'] != 'ardupilot_internal_physics_sitl':
        raise ValueError('验证器只接受原生SITL反馈模式，不能用运动学结果替代')
    if not state['running']:
        request(url+'/api/control',dict(action='start',forest_permission=authorize_forest,isolation_transit_simulation=authorize_isolation,synthetic=True))
    current_plan = request(url+'/api/plan')
    duration = current_plan['duration_s']
    started = time.monotonic()
    report = dict(mode=state['mode'],complete=False,samples=0,min_actual_separation_m=10000,
        states_seen=[],six_armed_observed=False,recovery_verified=False,
        flight_versions=[],synthetic_gimbal=True,hardware_verified=False,gazebo_physics_verified=False)
    report['start_stamp_s'] = state['stamp']
    report['full_run_observed'] = state['stamp']==0
    report['map_frame'] = current_plan.get('map_frame')
    report['coordination'] = current_plan.get('coordination')
    report['isolation_airspace'] = current_plan.get('isolation_airspace')
    report['conditional_isolation_transit'] = authorize_isolation
    report['competition_airspace_compliant'] = current_plan['isolation_airspace']['authorized_policy_respected'] and not current_plan['requires_forest_transit_permission']
    report['airspace_policy'] = current_plan.get('airspace_policy')
    report['scan_motion_model'] = current_plan.get('scan_motion_model')
    report['coverage_planning'] = current_plan.get('coverage_planning')
    report['coverage'] = current_plan.get('coverage')
    report['station_counts'] = {vehicle['id']:len(vehicle['stations']) for vehicle in current_plan['vehicles']}
    report['observation_conditions'] = current_plan.get('observation_conditions')
    report['subject'] = current_plan['subject']
    report['deadline_s'] = current_plan['deadline_s']
    report['planned_observations'] = sum(len(vehicle['stations'])*9 for vehicle in current_plan['vehicles'])
    report['planned_homes_enu_m'] = [vehicle['home'] for vehicle in current_plan['vehicles']]
    seen = set()
    recovery = False
    return_recovery = False
    report['return_recovery_required'] = verify_return_recovery
    report['maximum_sampled_parallel_return_movements'] = 0
    target = report_path or ROOT/'validation/sitl_report.json'
    target.parent.mkdir(parents=True,exist_ok=True)
    while time.monotonic()-started < timeout:
        try:
            state = request(url+'/api/state')
        except (URLError,TimeoutError,OSError) as error:
            report['last_transport_error'] = str(error)
            target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
            time.sleep(1)
            continue
        report['samples'] += 1
        if state.get('fcu_mission_elapsed_s',0)<0:
            raise ValueError('Negative FCU mission time cannot support a timing validation')
        report['min_actual_separation_m'] = min(report['min_actual_separation_m'],state.get('observed_min_separation_m',state['actual_min_separation_m']))
        report['execution'] = state.get('execution','trajectory_feedback')
        if not state['preparing'] and state.get('execution')!='actual_waypoint_state_machine':
            report['max_tracking_error_m'] = max(report.get('max_tracking_error_m',0),state['tracking_error_m'])
        report['six_armed_observed'] |= all(vehicle['telemetry']['armed'] for vehicle in state['vehicles'])
        seen.update(vehicle['state'] for vehicle in state['vehicles'])
        report.update(stamp_s=state['stamp'],flight_versions=[vehicle['telemetry']['version'] for vehicle in state['vehicles']],
            homes_captured_from_feedback=state.get('homes_captured_from_feedback',False),
            flight_audit=state.get('flight_audit'),
            flight_safety_audit=state.get('flight_safety_audit'),
            reservation_audit=state.get('reservation_audit'),
            hold_recovery_audit=state.get('hold_recovery_audit'),
            scan_alignment_audit=state.get('scan_alignment_audit'),
            phase_elapsed_s=state.get('phase_elapsed_s'),
            ekf_origins=[vehicle['telemetry'].get('ekf_origin') for vehicle in state['vehicles']],
            fcu_mission_elapsed_s=state.get('fcu_mission_elapsed_s'),
            completed_scans=sum(vehicle['completed_scans'] for vehicle in state['vehicles']),
            faults=state['faults'],states_seen=sorted(seen))
        active_returns = [vehicle for vehicle in state['vehicles'] if vehicle['id'] in state.get('active_returns',[])]
        moving_returns = [vehicle for vehicle in active_returns if vehicle['telemetry']['armed']
            and vehicle['telemetry'].get('speed_m_s',0)>.6]
        report['maximum_sampled_parallel_return_movements'] = max(report['maximum_sampled_parallel_return_movements'],len(moving_returns))
        if verify_return_recovery and not return_recovery and len(active_returns)>=2 and all(
                vehicle['telemetry']['mode']=='GUIDED' for vehicle in active_returns):
            frozen = request(url+'/api/control',dict(action='pause'))
            time.sleep(.6)
            held = request(url+'/api/state')
            cursors_held = [vehicle['segment_index'] for vehicle in frozen['vehicles']]==[vehicle['segment_index'] for vehicle in held['vehicles']]
            authorizations_cleared = all(not held['command_path_reservations'][vehicle['id']]['authorized']
                for vehicle in held['vehicles'] if vehicle['telemetry']['armed'] and vehicle['telemetry']['mode']=='GUIDED')
            no_rtl = all(vehicle['telemetry']['mode']!='RTL' for vehicle in held['vehicles'])
            report['return_pause_recovery'] = dict(active_returns=[vehicle['id'] for vehicle in active_returns],
                paused_cursors_preserved=cursors_held,guided_authorizations_cleared=authorizations_cleared,
                no_rtl_observed=no_rtl,maximum_sampled_pause_drift_m=max(math.dist(first['position'],second['position'])
                    for first,second in zip(frozen['vehicles'],held['vehicles'])),hardware_verified=False)
            resumed = request(url+'/api/control',dict(action='start',forest_permission=authorize_forest,isolation_transit_simulation=authorize_isolation,synthetic=True))
            settlement_started = time.monotonic()
            settlement_wait_seen = False
            while resumed.get('latched_group_hold_enu_m') is not None:
                settlement_wait_seen = True
                if [vehicle['segment_index'] for vehicle in resumed['vehicles']]!=[vehicle['segment_index'] for vehicle in held['vehicles']]:
                    raise ValueError('A mission cursor advanced before group hold settled')
                if any(resumed['command_path_reservations'][vehicle['id']]['authorized']
                        for vehicle in resumed['vehicles'] if vehicle['telemetry']['armed'] and vehicle['telemetry']['mode']=='GUIDED'):
                    raise ValueError('A new GUIDED movement was authorized while the group was still braking')
                if time.monotonic()-started>=timeout or resumed['faults']:
                    raise ValueError('Hold settlement timed out or encountered a flight fault')
                time.sleep(.1)
                resumed = request(url+'/api/state')
            report['return_pause_recovery'].update(settlement_wait_observed=settlement_wait_seen,
                no_cursor_advance_or_authorization_while_settling=True,
                settlement_wait_wall_s=time.monotonic()-settlement_started)
            settled = resumed.get('hold_recovery_audit',{}).get('resumed_after_actual_settlement',0)>0
            report['native_hold_settlement_verified'] = settled
            return_recovery = cursors_held and authorizations_cleared and no_rtl and settled
            report['parallel_return_pause_recovery_verified'] = return_recovery
            if not return_recovery:
                raise ValueError('Parallel return pause did not preserve cursors and clear GUIDED authorizations')
        if not recovery and any(vehicle['state'] in ('SCAN','NAVIGATE','EGRESS') for vehicle in state['vehicles']):
            frozen = request(url+'/api/control',dict(action='fault',vehicle_id='uav_1'))
            time.sleep(1)
            held = request(url+'/api/state')
            request(url+'/api/control',dict(action='recover',vehicle_id='uav_1'))
            report['recovery_verified'] = frozen['stamp']==held['stamp'] and frozen['epoch']==held['epoch'] and [vehicle['segment_index'] for vehicle in frozen['vehicles']]==[vehicle['segment_index'] for vehicle in held['vehicles']]
            recovery = True
        target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        if 'flight_safety' in state['faults']:
            report['stopped_by_runtime_boundary_guard'] = True
            break
        if state['progress']>=1 and state['all_landed_disarmed']:
            report['complete'] = True
            alignment_audit = state.get('scan_alignment_audit',{})
            report['all_observations_have_feasible_nominal_pointing'] = (
                alignment_audit.get('observations_with_feasible_pointing')==report['planned_observations']
                and report['completed_scans']==report['planned_observations'])
            report['final_actual_positions'] = [vehicle['position'] for vehicle in state['vehicles']]
            report['flight_version_labels'] = ['{}.{}.{} type{}'.format(version>>24,(version>>16)&255,(version>>8)&255,version&255)
                if version else None for version in report['flight_versions']]
            report['within_competition_time_budget_actual'] = state.get('fcu_mission_elapsed_s',math.inf)<=request(url+'/api/plan')['deadline_s']
            audit = state.get('flight_audit',{})
            safety = state.get('flight_safety_audit',{})
            report['sampled_exact_boundary_and_altitude_passed'] = (
                safety.get('rejections')==0 and audit.get('max_outside_perimeter_m')==0
                and audit.get('max_outside_work_region_m')==0
                and audit.get('min_height_outside_launch_m',0)>=current_plan['flight_safety']['minimum_operating_altitude_m']
                and audit.get('max_height_m',float('inf'))<=current_plan['flight_safety']['maximum_altitude_m'])
            report['sampled_estimated_horizontal_envelope_passed'] = (
                report['sampled_exact_boundary_and_altitude_passed']
                and audit.get('min_perimeter_clearance_m',0)>=current_plan['flight_safety']['estimated_horizontal_body_radius_m']
                and audit.get('min_work_region_clearance_m',0)>=current_plan['flight_safety']['estimated_horizontal_body_radius_m'])
            break
        time.sleep(.5)
    report['elapsed_wall_s'] = round(time.monotonic()-started,2)
    report['parallel_return_motion_observed'] = report['maximum_sampled_parallel_return_movements']>=2
    target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    return report['complete'] and (not verify_return_recovery or (return_recovery and report['parallel_return_motion_observed']))


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--url',default='http://127.0.0.1:8766')
    parser.add_argument('--timeout',type=float,default=900)
    parser.add_argument('--authorize-forest-transit',action='store_true')
    parser.add_argument('--authorize-isolation-transit',action='store_true',help='Simulation-only override; never competition airspace acceptance')
    parser.add_argument('--report',type=Path)
    parser.add_argument('--verify-return-recovery',action='store_true')
    arguments = parser.parse_args()
    raise SystemExit(0 if verify(arguments.url.rstrip('/'),arguments.timeout,arguments.authorize_forest_transit,
        arguments.report,arguments.verify_return_recovery,arguments.authorize_isolation_transit) else 1)
