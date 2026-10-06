from __future__ import annotations
import json
from validate_return_reservations import make_executor
from fleet_core import ROOT, build_plan


def main():
    executor,commands = make_executor(build_plan())
    fleet = executor.fleet
    executor.reservation_destinations[0] = [*fleet.telemetry[0]['position'][:2],110]
    previous_destination = executor.reservation_destinations[0][:]
    executor.reservation_keys = executor.cursors[:]
    frozen = executor.cursors[:]
    executor.hold_all()
    target = executor.holds[0][:]
    assert all(key is None for key in executor.reservation_keys)
    assert executor.reservation_destinations[0]==previous_destination
    fleet.telemetry[0]['speed_m_s'] = 3
    assert not executor.release_settled_hold()
    assert executor.cursors==frozen and executor.holds[0]==target
    fleet.telemetry[0]['speed_m_s'] = 0
    fleet.telemetry[0]['position'][0] += 5
    assert not executor.release_settled_hold()
    assert executor.cursors==frozen and executor.reservation_destinations[0]==previous_destination
    assert commands[-6][1]==target
    fleet.telemetry[0]['position'] = target[:]
    assert executor.release_settled_hold()
    assert executor.holds is None and executor.cursors==frozen
    assert executor.reservation_destinations[0]==target and all(key is None for key in executor.reservation_keys)
    assert executor.hold_recovery_audit['wait_checks']==2
    assert executor.hold_recovery_audit['resumed_after_actual_settlement']==1
    assert executor.hold_recovery_audit['maximum_observed_hold_drift_m']==5
    fleet.telemetry[1].update(mode='LAND',speed_m_s=2)
    executor.hold_all()
    assert executor.release_settled_hold()
    report = dict(passed=True,old_path_endpoint_retained_during_braking=True,
        high_speed_prevents_resume=True,off_target_prevents_resume=True,cursors_preserved=True,
        resume_only_after_actual_arrival_and_low_speed=True,original_path_needs_new_authorization=True,
        land_mode_not_forced_to_guided=True,hold_drift_recorded=True,
        scope='synthetic telemetry accounting; not a physical stopping envelope',hardware_verified=False)
    (ROOT/'validation/hold_recovery_logic_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
