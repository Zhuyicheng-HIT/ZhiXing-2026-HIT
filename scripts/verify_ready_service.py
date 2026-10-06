from __future__ import annotations
import argparse
import json
import time
from pathlib import Path
from urllib.request import urlopen
from fleet_core import ROOT


def read(base, path):
    with urlopen(base+path,timeout=15) as response:
        assert response.status==200
        return response.read()


def verify(base, report_path):
    state = json.loads(read(base,'/api/state'))
    plan = json.loads(read(base,'/api/plan'))
    assert state['mode']=='ardupilot_internal_physics_sitl' and not state['running']
    assert len(state['vehicles'])==6 and not state['faults']
    for vehicle in state['vehicles']:
        assert vehicle['state']=='READY' and not vehicle['telemetry']['armed']
        assert vehicle['telemetry']['received'] and vehicle['telemetry']['age_s']<2
        assert vehicle['telemetry']['version']>>8==0x040701
    assert b'mapCanvas' in read(base,'/')
    assert b'isolationPermission' in read(base,'/')
    assert b'candidatePreview' in read(base,'/')
    assert b'coveragePlanningStatus' in read(base,'/app.js')
    assert read(base,'/assets/satellite.jpg').startswith(b'\xff\xd8')
    candidate = json.loads(read(base,'/api/plan/isolation-candidate'))
    assert candidate['isolation_design']['kind']=='proposed_end_corridors'
    assert not candidate['isolation_design']['user_approved']
    assert not candidate['isolation_airspace']['crossings']
    after_preview = json.loads(read(base,'/api/state'))
    assert json.loads(read(base,'/api/plan'))==plan
    assert after_preview['epoch']==state['epoch'] and not after_preview['running']
    assert after_preview['stamp']==state['stamp']
    assert [vehicle['segment_index'] for vehicle in after_preview['vehicles']]==[vehicle['segment_index'] for vehicle in state['vehicles']]
    assert all(not vehicle['telemetry']['armed'] for vehicle in after_preview['vehicles'])
    report = dict(checked_at_unix_s=time.time(),url=base,passed=True,
        mode=state['mode'],speedup=state['speed'],six_ready_unarmed=True,fresh_position_feedback=True,
        flight_versions=[vehicle['telemetry']['version'] for vehicle in state['vehicles']],
        station_counts={vehicle['id']:len(vehicle['stations']) for vehicle in plan['vehicles']},
        planned_observations=sum(len(vehicle['stations'])*9 for vehicle in plan['vehicles']),
        coverage_planning=plan.get('coverage_planning'),coordination=plan.get('coordination'),
        isolation_airspace=plan.get('isolation_airspace'),competition_airspace_compliant=state.get('competition_airspace_compliant'),
        nominal_return_timing_model=plan.get('nominal_return_timing_model'),web_html_script_satellite_http=True,
        candidate_preview_http_verified=True,candidate_preview_does_not_start_or_replace_task=True,
        browser_visual_verified=False,browser_click_verified=False,hardware_verified=False,
        scope='HTTP delivery and fresh native FCU ready state; not browser rendering or flight acceptance')
    report_path.parent.mkdir(parents=True,exist_ok=True)
    report_path.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--url',default='http://127.0.0.1:8766')
    parser.add_argument('--report',type=Path,default=ROOT/'validation/ready_service_report.json')
    arguments = parser.parse_args()
    verify(arguments.url.rstrip('/'),arguments.report)
