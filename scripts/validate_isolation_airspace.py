from __future__ import annotations
import json
from copy import deepcopy
from fleet_core import ROOT, build_plan
from isolation_airspace import audit_isolation, require_isolation_permission
from sim_server import FleetSimulation


def main():
    plan = build_plan()
    audit = plan['isolation_airspace']
    assert audit==audit_isolation(plan)
    assert audit['crossings'] and not audit['all_altitude_isolation_respected_by_plan']
    assert all(not item['all_stations_reachable'] for item in audit['alternatives'])
    for data in ({'isolation_transit_simulation':'false'},):
        try:
            require_isolation_permission(plan,data)
        except ValueError:
            pass
        else:
            raise AssertionError('Implicit isolation override accepted')
    assert not require_isolation_permission(plan,{})
    simulation = FleetSimulation()
    before = deepcopy((simulation.epoch,simulation.stamp,simulation.completed,simulation.running))
    simulation.control(dict(action='start',forest_permission=True))
    assert not simulation.snapshot()['isolation_transit_simulation']
    assert simulation.snapshot()['competition_airspace_compliant']
    simulation.control(dict(action='pause'))
    assert not simulation.running
    report = dict(passed=True,all_altitude_plan_crossings_exposed=True,
        work_crossings_rejected=True,layered_transit_allowed_inside_perimeter=True,string_override_rejected=True,
        forest_permission_confirmed_by_policy=True,explicit_demo_override_not_required=True,
        audit=audit,hardware_verified=False,scope='geometry and preflight logic; not physical flight acceptance')
    (ROOT/'validation/isolation_airspace_logic_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
