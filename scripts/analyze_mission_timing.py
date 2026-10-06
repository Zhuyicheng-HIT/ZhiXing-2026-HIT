from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path


GROUPS = {
    'flight': ['CLIMB','INGRESS','DESCEND_TO_WORK','NAVIGATE','REPOSITION_CLIMB','REPOSITION','RETURN_CLIMB','EGRESS','RETURN_DESCEND','LAND'],
    'observation': ['STABILIZE','SCAN','WAIT_RESULT'],
    'heading_and_pointing': ['SCAN_ALIGN','SCAN_WAIT_POINTING','SCAN_WAIT_ATTITUDE','SCAN_UNOBSERVABLE'],
    'target_events': ['CONFIRM_STATIC','TRACK_MOVING'],
    'queue': ['WAIT_RELEASE','WAIT_RETURN_SLOT','WAIT_TRANSIT_SLOT','WAIT_PATH_RESERVATION'],
    'fault_or_pause': ['PAUSED_OR_GROUP_HOLD','WAIT_HOLD_STABLE'],
    'landed_wait': ['COMPLETE']
}


def summarize(path):
    content = path.read_bytes()
    report = json.loads(content)
    if not report.get('complete') or not report.get('full_run_observed'):
        raise ValueError('Timing comparison requires complete runs observed from zero: '+str(path))
    aircraft = []
    classified = {phase for phases in GROUPS.values() for phase in phases}
    for vehicle_id,phases in report['phase_elapsed_s'].items():
        totals = {name:sum(phases.get(phase,0) for phase in names) for name,names in GROUPS.items()}
        totals['unclassified'] = sum(value for phase,value in phases.items() if phase not in classified)
        active = sum(value for name,value in totals.items() if name!='landed_wait')
        aircraft.append(dict(vehicle_id=vehicle_id,phase_seconds=totals,active_sampled_seconds=active,
            deadline_overrun_s=report['fcu_mission_elapsed_s']-report['deadline_s']))
    critical = max(aircraft,key=lambda vehicle:vehicle['active_sampled_seconds'])
    return dict(source=str(path),source_sha256=hashlib.sha256(content).hexdigest(),subject=report['subject'],
        deadline_s=report['deadline_s'],elapsed_fcu_s=report['fcu_mission_elapsed_s'],
        deadline_passed=report['within_competition_time_budget_actual'],
        observations=report['completed_scans'],critical_vehicle_by_sampled_active_time=critical['vehicle_id'],
        critical_vehicle_phase_seconds=critical['phase_seconds'],vehicles=aircraft,
        alignment_wait_fleet_total_s=report['scan_alignment_audit']['alignment_wait_fcu_s'],
        preparation_and_sampling_residual_s=report['fcu_mission_elapsed_s']-critical['active_sampled_seconds'])


def main():
    parser = argparse.ArgumentParser(description='Analyze completed actual FCU runs; not a timing guarantee')
    parser.add_argument('reports',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    arguments = parser.parse_args()
    runs = [summarize(path) for path in arguments.reports]
    comparisons = []
    for subject in sorted({run['subject'] for run in runs}):
        matching = [run for run in runs if run['subject']==subject]
        if len(matching)<2:
            continue
        first,last = matching[0],matching[-1]
        comparisons.append(dict(subject=subject,baseline=first['source'],candidate=last['source'],
            elapsed_difference_s=last['elapsed_fcu_s']-first['elapsed_fcu_s'],
            summed_heading_wait_difference_s=last['alignment_wait_fleet_total_s']-first['alignment_wait_fleet_total_s'],
            repeated_controlled_experiment=False,
            scope='single run difference; summed fleet waits are not wall-clock savings'))
    result = dict(runs=runs,comparisons=comparisons,
        scope='sampled phase attribution; concurrent phases cannot be added across aircraft to predict total time',
        hardware_verified=False)
    arguments.output.parent.mkdir(parents=True,exist_ok=True)
    arguments.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
