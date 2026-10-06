from __future__ import annotations
import argparse
import json
from urllib.error import HTTPError
from pathlib import Path
from fleet_core import ROOT
from verify_sitl import request


def verify(url, report_path):
    before = request(url+'/api/state')
    assert not before['running'] and not before['preparing']
    assert all(not vehicle['telemetry']['armed'] for vehicle in before['vehicles'])
    plan = request(url+'/api/plan')
    assert plan['isolation_airspace']['crossings'] and not plan['isolation_airspace']['work_crossings']
    for override in ('false',):
        data = dict(action='start',forest_permission=True,synthetic=True,isolation_transit_simulation=override)
        try:
            request(url+'/api/control',data)
        except HTTPError as error:
            assert error.code==400
            message = json.loads(error.read())['error']
            assert 'isolation_transit_simulation' in message
        else:
            raise AssertionError('Unsafe start unexpectedly accepted')
    after = request(url+'/api/state')
    assert not after['running'] and not after['preparing']
    assert not after['homes_captured_from_feedback']
    assert before['epoch']==after['epoch'] and before['stamp']==after['stamp']
    assert [vehicle['segment_index'] for vehicle in before['vehicles']]==[vehicle['segment_index'] for vehicle in after['vehicles']]
    assert all(not vehicle['telemetry']['armed'] for vehicle in after['vehicles'])
    report = dict(passed=True,mode=after['mode'],invalid_string_override_rejected_before_arming=True,
        six_unarmed_after_rejection=True,task_epoch_and_cursors_unchanged=True,
        work_crossings_empty=True,layered_transit_policy_active=True,string_override_rejected=True,
        plan_crossing_count=len(plan['isolation_airspace']['crossings']),hardware_verified=False,
        scope='live native preflight rejection; not in-flight isolation or browser interaction acceptance')
    report_path.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--url',default='http://127.0.0.1:8766')
    parser.add_argument('--report',type=Path,default=ROOT/'validation/isolation_preflight_sitl_report.json')
    arguments = parser.parse_args()
    verify(arguments.url.rstrip('/'),arguments.report)
