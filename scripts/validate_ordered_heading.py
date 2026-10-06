from __future__ import annotations
import hashlib
import json
import math
from fleet_core import ROOT, build_plan
from gimbal_geometry import plan_level_hover_heading, plan_ordered_hover_heading


def main():
    configuration = json.loads((ROOT/'config/gimbal_geometry.json').read_text(encoding='utf-8'))
    plan = build_plan()
    before = json.dumps(plan,sort_keys=True)
    observations = 0
    greedy_turns = 0
    ordered_turns = 0
    heading_plans = 0
    for vehicle in plan['vehicles']:
        greedy_heading = math.pi/2
        heading = math.pi/2
        segments = vehicle['segments']
        cursor = 0
        while cursor<len(segments):
            segment = segments[cursor]
            if segment['state']!='SCAN':
                cursor += 1
                continue
            targets = []
            for upcoming in segments[cursor:]:
                if upcoming['state']!='SCAN' or upcoming.get('station')!=segment.get('station') or upcoming['destination']!=segment['destination']:
                    break
                targets.append(upcoming['target'])
            goal = plan_ordered_hover_heading(targets,segment['destination'],heading,configuration)
            heading_plans += 1
            ordered_turns += abs(goal['heading_change_rad'])>1e-9
            heading = goal['heading_mission_rad']
            for target in targets[:goal['ordered_prefix_count']]:
                check = plan_level_hover_heading(target,segment['destination'],heading,configuration)
                assert abs(check['heading_change_rad'])<1e-9
                greedy = plan_level_hover_heading(target,segment['destination'],greedy_heading,configuration)
                greedy_turns += abs(greedy['heading_change_rad'])>1e-9
                greedy_heading = greedy['heading_mission_rad']
                observations += 1
            cursor += goal['ordered_prefix_count']
    assert before==json.dumps(plan,sort_keys=True)
    assert observations==sum(segment['state']=='SCAN' for vehicle in plan['vehicles'] for segment in vehicle['segments']) and ordered_turns<greedy_turns
    for targets in ([],[[0,0,100]]):
        try:
            plan_ordered_hover_heading(targets,[0,0,40],0,configuration)
        except ValueError:
            pass
        else:
            raise AssertionError('Invalid or unreachable ordered targets accepted')
    report = dict(observations=observations,greedy_heading_changes=greedy_turns,
        ordered_heading_changes=ordered_turns,ordered_heading_plans=heading_plans,
        full_plan_unchanged=True,plan_sha256=hashlib.sha256(before.encode()).hexdigest(),
        all_ordered_targets_nominally_reachable=True,hardware_verified=False,
        actual_flight_validated=False,policy='LONGEST_CONTIGUOUS_PREFIX_MINIMUM_TURN')
    (ROOT/'validation/ordered_heading_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
