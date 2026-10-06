from __future__ import annotations
import argparse
import json
import math
import time
from copy import deepcopy
from pathlib import Path
from shapely.geometry import shape
from fleet_core import ROOT, navigate
from path_reservations import segment_distance
from sitl_server import SitlFleet


def run(report_path, timeout, scenario='transit'):
    fleet = SitlFleet(ROOT/'vendor/ardupilot-4.7.1/build/sitl/bin/arducopter',
        ROOT/'vendor/ardupilot-4.7.1/Tools/autotest/default_params/copter.parm',20)
    fleet.checkpoint = lambda: None
    started = time.monotonic()
    report = dict(passed=False,scenario=scenario,scope='bounded custom crossing fixture using production native six-FCU executor',
        speedup=20,conditional_forest_transit=True,conditional_isolation_transit=True,competition_airspace_compliant=False,full_mission_verified=False,
        hardware_verified=False,gazebo_physics_verified=False,checkpoint_disk_restore_verified=False)

    def pump_until(predicate, seconds=timeout):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            fleet.advance(.01)
            if fleet.faults:
                raise AssertionError('Unexpected fleet hold: '+str(fleet.faults))
            if predicate():
                return
            time.sleep(.01)
        raise AssertionError('Timed out waiting for actual FCU condition')

    def fixture(destinations, states=None, routed=False):
        executor = fleet.executor
        fleet.pending_by_vehicle.clear()
        for index,vehicle in enumerate(fleet.plan['vehicles']):
            origin = fleet.telemetry[index]['position'][:]
            destination = destinations.get(index,origin)
            phase = (states or {}).get(index,'INGRESS')
            if routed and index in destinations:
                temporary = dict(position=origin,clock=0,segments=[])
                navigate(temporary,destination,shape(fleet.plan['perimeter']).buffer(-2,join_style=2),phase)
                vehicle['segments'] = temporary['segments']
            else:
                vehicle['segments'] = [dict(state=phase,origin=origin,destination=destination[:],start=0,end=1e6)]
            vehicle['segments'].append(dict(state='INGRESS',origin=destination[:],destination=destination[:],start=1e6,end=2e6))
        executor.cursors = [0]*6
        executor.elapsed = [0.0]*6
        executor.entered = [True]*6
        executor.entry_released = [True]*6
        executor.entry_handed_off = [True]*6
        executor.entry_cursor = 6
        executor.return_released = [bool(states and index in states) for index in range(6)]
        executor.reservation_keys = [None]*6
        executor.reservation_destinations = [None]*6
        executor.reservation_holds = [None]*6
        executor.reservation_waits = [None]*6
        executor.observation_holds = [None]*6
        executor.holds = None
        executor.hold_yaws = None

    def arrived(index, destination):
        item = fleet.telemetry[index]
        return math.dist(item['position'],destination)<.5 and item['speed_m_s']<.4

    try:
        pump_until(lambda: all(item['received'] and time.monotonic()-item['last_seen']<2
            for item in fleet.telemetry))
        fleet.control(dict(action='start',forest_permission=True,isolation_transit_simulation=True,synthetic=False))
        pump_until(lambda: len(fleet.pending_by_vehicle)==6)
        report['six_actual_observation_waits_before_fixture'] = True
        report['original_positions'] = [item['position'][:] for item in fleet.telemetry]
        first,second = 4,5
        center = shape(fleet.plan['launch'] if scenario=='return-descent' else fleet.plan['vehicles'][first]['flight_region']).centroid
        vertical_start = [center.x,center.y,110.0]
        if scenario=='return-descent':
            horizontal_start = [center.x,center.y-16,90.0]
            horizontal_end = [center.x,center.y+16,90.0]
            vertical_end = [center.x,center.y,2.0]
            states = {first:'RETURN_DESCEND',second:'EGRESS'}
        else:
            horizontal_start = [center.x-40,center.y,90.0]
            horizontal_end = [center.x+40,center.y,90.0]
            vertical_end = [center.x,center.y,40.5]
            states = None
        for index,start,destination in ((first,vertical_start,vertical_end),(second,horizontal_start,horizontal_end)):
            check = fleet.executor.flight_safety.check(index,(states or {}).get(index,'INGRESS'),start,destination,True)
            assert check['allowed'], check
        fixture({first:vertical_start,second:horizontal_start},routed=scenario=='return-descent')
        pump_until(lambda: arrived(first,vertical_start) and arrived(second,horizontal_start))
        report['staged_positions'] = [item['position'][:] for item in fleet.telemetry]
        fixture({first:vertical_end,second:horizontal_end},states=states)
        baseline_blocks = fleet.executor.reservation_audit['blocked_checks']
        pump_until(lambda: fleet.executor.reservation_waits[second] is not None,10)
        executor = fleet.executor
        saved_cursors = executor.cursors[:]
        waiting = deepcopy(executor.reservation_waits[second])
        waiting_position = fleet.telemetry[second]['position'][:]
        assert waiting['vehicle_id']==fleet.plan['vehicles'][first]['id']
        assert waiting['distance_m']<executor.reservation_distance
        assert executor.reservation_keys[second] is None
        if scenario=='return-descent':
            pump_until(lambda: vertical_start[2]-fleet.telemetry[first]['position'][2]>5 and fleet.telemetry[first]['speed_m_s']>1)
            assert executor.cursors==saved_cursors and executor.reservation_keys[second] is None
        fleet.control(dict(action='pause'))
        pause_start = [item['position'][:] for item in fleet.telemetry]
        pause_boot = [item['boot_ms'] for item in fleet.telemetry]
        pause_deadline = time.monotonic()+(.05 if scenario=='return-descent' else .7)
        maximum_pause_drift = 0.0
        while time.monotonic()<pause_deadline:
            fleet.advance(.01)
            assert executor.cursors==saved_cursors and not fleet.faults
            assert all(item['mode']=='GUIDED' and item['armed'] for item in fleet.telemetry)
            assert all(key is None for key in executor.reservation_keys)
            maximum_pause_drift = max(maximum_pause_drift,max(math.dist(item['position'],position)
                for item,position in zip(fleet.telemetry,pause_start)))
            time.sleep(.01)
        assert maximum_pause_drift<(5.0 if scenario=='return-descent' else 2.0)
        report['pause_maximum_actual_drift_m'] = maximum_pause_drift
        report['pause_actual_fcu_seconds'] = [(item['boot_ms']-boot)/1000
            for item,boot in zip(fleet.telemetry,pause_boot)]
        fleet.control(dict(action='start',forest_permission=True,isolation_transit_simulation=True,synthetic=False))
        hold_wait_start = executor.hold_recovery_audit['wait_checks']
        hold_deadline = time.monotonic()+timeout
        while executor.holds is not None:
            assert time.monotonic()<hold_deadline
            assert executor.cursors==saved_cursors and all(key is None for key in executor.reservation_keys)
            fleet.advance(.01)
            assert not fleet.faults
            time.sleep(.01)
        if scenario=='return-descent':
            assert executor.hold_recovery_audit['wait_checks']>hold_wait_start
        report['native_unsettled_hold_blocks_resume'] = executor.hold_recovery_audit['wait_checks']>hold_wait_start
        pump_until(lambda: executor.reservation_waits[second] is not None,10)
        wait_start_boot = fleet.telemetry[second]['boot_ms']
        maximum_wait_drift = 0.0
        maximum_first_progress = 0.0
        deadline = time.monotonic()+timeout
        while executor.reservation_keys[second] is None:
            assert time.monotonic()<deadline
            fleet.advance(.01)
            assert executor.cursors==saved_cursors and not fleet.faults
            assert all(item['mode']=='GUIDED' and item['armed'] for item in fleet.telemetry)
            maximum_wait_drift = max(maximum_wait_drift,math.dist(fleet.telemetry[second]['position'],waiting_position))
            maximum_first_progress = max(maximum_first_progress,vertical_start[2]-fleet.telemetry[first]['position'][2])
            time.sleep(.01)
        assert maximum_wait_drift<.8
        assert maximum_first_progress>10
        distance_at_grant = segment_distance(fleet.telemetry[second]['position'],horizontal_end,
            fleet.telemetry[first]['position'],vertical_end)
        assert distance_at_grant>=executor.reservation_distance
        report.update(wait_conflict=waiting,waiting_cursor_unchanged=True,
            pause_clears_authorizations=True,resume_rechecks_conflict=True,
            waiter_maximum_actual_drift_m=maximum_wait_drift,
            blocker_actual_vertical_progress_m=maximum_first_progress,
            waiter_actual_fcu_seconds=(fleet.telemetry[second]['boot_ms']-wait_start_boot)/1000,
            remaining_path_distance_at_grant_m=distance_at_grant,
            crossing_segments=dict(vertical_start=vertical_start,vertical_end=vertical_end,
                horizontal_start=horizontal_start,horizontal_end=horizontal_end))
        pump_until(lambda: arrived(first,vertical_end) and arrived(second,horizontal_end))
        assert executor.cursors[first]==saved_cursors[first]+1, 'Actual vertical arrival must advance its original cursor once'
        assert executor.cursors[second]==saved_cursors[second]+1, 'Actual horizontal arrival must advance its original cursor once'
        assert executor.reservation_audit['blocked_checks']>baseline_blocks
        fleet.control(dict(action='pause'))
        report.update(passed=True,original_destinations_reached_by_actual_feedback=True,
            no_independent_rtl_observed=True,ending_armed_paused_hover=True,
            ending_positions=[item['position'][:] for item in fleet.telemetry],
            observed_min_separation_m=executor.minimum_separation,flight_audit=deepcopy(executor.audit),
            hold_recovery_audit=deepcopy(executor.hold_recovery_audit),
            reservation_audit=deepcopy(executor.reservation_audit))
    except Exception as error:
        report['error'] = repr(error)
        raise
    finally:
        report['wall_duration_s'] = time.monotonic()-started
        report_path.parent.mkdir(parents=True,exist_ok=True)
        report_path.write_text(json.dumps(report,indent=2),encoding='utf-8')
        fleet.close()
        print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Owns six FCUs; stop the web SITL service before running')
    parser.add_argument('--report',type=Path,default=ROOT/'validation/crossing_reservations_sitl_report.json')
    parser.add_argument('--timeout',type=float,default=180)
    parser.add_argument('--scenario',choices=('transit','return-descent'),default='transit')
    arguments = parser.parse_args()
    run(arguments.report,arguments.timeout,arguments.scenario)
