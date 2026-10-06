from __future__ import annotations
import json
import time
from copy import deepcopy
from types import SimpleNamespace
from fleet_core import ROOT, build_plan
from scan_protocol import ScanProtocol
from waypoint_executor import WaypointExecutor


def make_executor(plan):
    now = time.monotonic()
    commands = []
    fleet = SimpleNamespace(plan=plan,
        gimbal_geometry=json.loads((ROOT/'config/gimbal_geometry.json').read_text(encoding='utf-8')),
        telemetry=[dict(position=[*vehicle['home'],2],boot_ms=1000,received=True,last_seen=now,
            armed=True,mode='GUIDED',speed_m_s=0) for vehicle in plan['vehicles']],
        emit=lambda *args,**kwargs:None,setpoint=lambda *args,**kwargs:commands.append(args))
    executor = WaypointExecutor(fleet)
    executor.begin()
    executor.entered = [True]*6
    executor.entry_cursor = 6
    for index,vehicle in enumerate(plan['vehicles']):
        executor.cursors[index] = next(cursor for cursor,segment in enumerate(vehicle['segments']) if segment['state']=='WAIT_RETURN_SLOT')
        fleet.telemetry[index]['position'] = executor.segment(index)['destination'][:]
    return executor,commands


def main():
    plan = build_plan()
    assert plan['coordination']['return_policy']=='layered_reserved'
    executor,commands = make_executor(plan)
    first,second = 0,1
    executor.entered[5] = False
    saved = executor.cursors[:]
    assert not executor.allowed(first) and executor.cursors==saved
    executor.entered[5] = True
    assert not executor.allowed(first) and executor.cursors[first]==saved[first]+1
    assert not executor.allowed(second) and executor.cursors[second]==saved[second]+1
    assert executor.return_queue==[first,second] and executor.transit_owner is None
    assert executor.return_released[:2]==[True,True]
    assert executor.reservation_audit['maximum_parallel_returns']==2
    assert not all(executor.search_done)
    assert executor.allowed(first) and executor.allowed(second)
    assert not executor.waiting_for_transit(first)
    for index,item in enumerate(executor.fleet.telemetry):
        item['position'] = [1000+index*100,1000,40.5]
    executor.fleet.telemetry[first]['position'] = [0,0,90]
    executor.fleet.telemetry[second]['position'] = [-20,0,80]
    assert executor.reserve_path(first,[0,0,2])
    frozen = executor.cursors[:]
    assert not executor.reserve_path(second,[20,0,80])
    assert executor.cursors==frozen and commands[-1][1]==[-20,0,80]
    executor.hold_all()
    assert executor.cursors==frozen and all(key is None for key in executor.reservation_keys)
    assert executor.return_released[:2]==[True,True]
    executor.holds = None
    executor.hold_yaws = None
    assert executor.reserve_path(first,[0,0,2])
    assert not executor.reserve_path(second,[20,0,80])
    executor.fleet.telemetry[first]['position'] = [0,0,2]
    assert executor.reserve_path(second,[20,0,80]) and executor.cursors==frozen
    executor.cursors[first] = next(cursor for cursor,segment in enumerate(plan['vehicles'][first]['segments']) if segment['state']=='LAND')
    executor.fleet.telemetry[first].update(position=[*plan['vehicles'][first]['home'],0],armed=False)
    executor.transit_owner = 5
    executor.advance_cursor(first)
    assert executor.transit_owner==5 and executor.return_queue==[second]
    assert not executor.return_released[first] and executor.return_released[second]
    assert executor.reservation_keys[first] is None
    profile = json.loads((ROOT/'config/scan_profile.json').read_text(encoding='utf-8'))
    original = deepcopy(plan)
    original['coordination']['return_policy'] = 'exclusive'
    assert ScanProtocol(plan,profile).plan_revision!=ScanProtocol(original,profile).plan_revision
    try:
        ScanProtocol(plan,profile).restore(ScanProtocol(original,profile).checkpoint())
    except ValueError:
        pass
    else:
        raise AssertionError('An exclusive-return checkpoint was accepted under a different policy')
    fallback,_ = make_executor(original)
    initial = fallback.cursors[:]
    assert not fallback.allowed(first) and fallback.cursors[first]==initial[first]+1
    assert not fallback.allowed(second) and fallback.cursors[second]==initial[second]
    assert fallback.transit_owner==first
    invalid = deepcopy(plan)
    invalid['coordination']['entry_policy'] = 'exclusive'
    try:
        make_executor(invalid)
    except ValueError:
        pass
    else:
        raise AssertionError('Reserved returns without reservations must be rejected')
    report = dict(passed=True,return_release_independent_of_fifo=True,all_entries_required=True,
        other_searches_can_continue=True,conflict_keeps_cursor_and_hover=True,
        pause_clears_authorizations=True,resume_rechecks_original_path=True,
        landed_release_does_not_release_other_transit_owner=True,maximum_parallel_returns=2,
        exclusive_fallback_retained=True,policy_changes_plan_revision=True,old_checkpoint_rejected=True,
        scope='executor logic with synthetic positions; not native flight or physical safety',hardware_verified=False)
    (ROOT/'validation/return_reservation_logic_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
