from __future__ import annotations
import json
import math
import random
from fleet_core import ROOT
from gimbal_geometry import feasible_pointing


def direction(yaw_deg, elevation_deg):
    yaw,elevation = math.radians(yaw_deg),math.radians(elevation_deg)
    return [math.cos(elevation)*math.cos(yaw),math.cos(elevation)*math.sin(yaw),math.sin(elevation)]


def main():
    configuration = json.loads((ROOT/'config/gimbal_geometry.json').read_text(encoding='utf-8'))
    exact = feasible_pointing(direction(100,-40),configuration,2)
    assert exact['feasible'] and exact['pointing_error_deg']<1e-10
    near_nadir = feasible_pointing(direction(170,-89.8),configuration,2)
    assert near_nadir['feasible'] and near_nadir['pointing_error_deg']<0.2
    assert -135<=near_nadir['azimuth_deg']<=135 and -90<=near_nadir['elevation_deg']<=25
    behind = feasible_pointing(direction(180,-40),configuration,2)
    assert not behind['feasible'] and behind['pointing_error_deg']>20
    assert not feasible_pointing(direction(0,80),configuration,2)['feasible']
    generator = random.Random(2026)
    for unused in range(100):
        target = direction(generator.uniform(-180,180),generator.uniform(-90,90))
        solution = feasible_pointing(target,configuration,2)
        best_dot = sum(actual*desired for actual,desired in zip(solution['look_direction'],target))
        for yaw in range(-135,136,15):
            for elevation in range(-90,26,5):
                sampled_dot = sum(actual*desired for actual,desired in zip(direction(yaw,elevation),target))
                assert best_dot>=sampled_dot-1e-10
    rejected = 0
    for invalid in [[0,0,0],[True,0,0],[float('nan'),0,1]]:
        try:
            feasible_pointing(invalid,configuration,2)
        except ValueError:
            rejected += 1
    assert rejected==3
    replay = []
    for filename in ['sitl_heading_aligned_report.json','sitl_subject2_heading_aligned_report.json']:
        source = json.loads((ROOT/'validation'/filename).read_text(encoding='utf-8'))
        for item in source['scan_alignment_audit']['outside_nominal_examples']:
            solution = feasible_pointing(item['actual_pointing']['look_direction'],configuration,2)
            assert solution['feasible']
            replay.append(dict(source=filename,command_id=item['command_id'],pointing_error_deg=solution['pointing_error_deg']))
    report = dict(exact_pointing_preserved=True,near_nadir_feasible_without_relaxing_error=True,
        genuinely_unreachable_rejected=True,random_grid_comparison_cases=100,
        invalid_vectors_rejected=True,replayed_samples=len(replay),replay=replay,
        maximum_replay_error_deg=max(item['pointing_error_deg'] for item in replay),
        hardware_verified=False,full_mission_run=False)
    (ROOT/'validation/feasible_pointing_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({name:value for name,value in report.items() if name!='replay'},indent=2))


if __name__=='__main__':
    main()
