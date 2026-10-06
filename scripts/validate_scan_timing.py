from __future__ import annotations
import json
import math
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch
from fleet_core import ROOT, build_plan
from scan_protocol import ScanProtocol, validate_scan_profile


def rejected(callback):
    try:
        callback()
    except ValueError:
        return
    raise AssertionError('Invalid scan configuration or stale checkpoint was accepted')


def main():
    profile_path = ROOT/'config/scan_profile.json'
    profile = json.loads(profile_path.read_text(encoding='utf-8'))
    baseline = build_plan()
    changed_profile = dict(profile,minimum_observation_s=3.0)
    original_read = Path.read_text

    def configured_read(path, *args, **kwargs):
        if path==profile_path:
            return json.dumps(changed_profile)
        return original_read(path,*args,**kwargs)

    with patch.object(Path,'read_text',configured_read):
        changed = build_plan()
    baseline_protocol = ScanProtocol(baseline,profile)
    changed_protocol = ScanProtocol(changed,changed_profile)
    total = 0
    for old_vehicle,new_vehicle in zip(baseline['vehicles'],changed['vehicles']):
        for field in ('id','home','stations','flight_region'):
            assert old_vehicle[field]==new_vehicle[field]
        old_segments = old_vehicle['segments']
        new_segments = new_vehicle['segments']
        assert len(old_segments)==len(new_segments)
        for old_segment,new_segment in zip(old_segments,new_segments):
            for field in ('state','destination','target','command_id','station'):
                assert old_segment.get(field)==new_segment.get(field)
            old_duration = old_segment['end']-old_segment['start']
            new_duration = new_segment['end']-new_segment['start']
            if new_segment['state']=='SCAN':
                slew = new_segment['scan_motion']['minimum_slew_s']
                assert math.isclose(old_duration,profile['minimum_observation_s']+slew,abs_tol=1e-8)
                assert math.isclose(new_duration,3.0+slew,abs_tol=1e-8)
                total += 1
            elif new_segment['state'] not in ('WAIT_RETURN_SLOT','COMPLETE'):
                assert math.isclose(old_duration,new_duration,abs_tol=1e-8)
    assert total==sum(segment['state']=='SCAN' for vehicle in baseline['vehicles'] for segment in vehicle['segments'])
    assert changed_protocol.plan_revision!=baseline_protocol.plan_revision
    rejected(lambda:ScanProtocol(baseline,changed_profile))
    rejected(lambda:changed_protocol.restore(baseline_protocol.checkpoint()))
    malformed = deepcopy(changed)
    scan = next(segment for segment in malformed['vehicles'][0]['segments'] if segment['state']=='SCAN')
    scan['end'] -= 1
    rejected(lambda:ScanProtocol(malformed,changed_profile))
    for field,value in (('minimum_distinct_frames',5),('minimum_distinct_frames',True),
                        ('minimum_observation_s',float('nan')),('minimum_observation_s',True),
                        ('execution_timeout_s',3.0),('minimum_stable_s',-1)):
        rejected(lambda:validate_scan_profile(dict(changed_profile,**{field:value})))
    assert profile['scan_mode']=='CONTINUOUS_SWEEP'
    assert profile['minimum_observation_s']==profile['minimum_stable_s']==0
    command = baseline_protocol.issue(baseline['vehicles'][0]['id'],next(iter(baseline_protocol.observations)),'continuous-test')
    assert command['command_type']=='SWEEP_SEGMENT'
    assert json.loads(profile_path.read_text(encoding='utf-8'))==profile
    report = dict(scan_count=total,default_observation_s=profile['minimum_observation_s'],
        alternative_observation_s=3.0,plan_and_protocol_aligned=True,
        regional_scan_order_and_geometry_unchanged=True,stale_checkpoint_rejected=True,
        invalid_profiles_rejected=True,configuration_file_unchanged=True,
        baseline_nominal_duration_s=baseline['duration_s'],alternative_nominal_duration_s=changed['duration_s'],
        scope='Configuration and protocol logic; not hardware or physical mission validation')
    (ROOT/'validation/scan_timing_report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
