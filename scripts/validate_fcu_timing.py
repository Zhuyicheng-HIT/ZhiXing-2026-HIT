from __future__ import annotations
import json
from fleet_core import ROOT
from fcu_timing import record_arming_transition, mission_elapsed_seconds


def main():
    telemetry = dict(armed=False,armed_since_ms=None,landed_disarmed_ms=None,boot_ms=1000)
    assert mission_elapsed_seconds([telemetry])==0
    record_arming_transition(telemetry,True)
    telemetry['armed'] = True
    telemetry['boot_ms'] = 3000
    assert mission_elapsed_seconds([telemetry])==2
    record_arming_transition(telemetry,False)
    telemetry['armed'] = False
    telemetry['boot_ms'] = 5000
    assert mission_elapsed_seconds([telemetry])==2
    record_arming_transition(telemetry,True)
    telemetry['armed'] = True
    assert telemetry['landed_disarmed_ms'] is None
    assert telemetry['armed_since_ms']==1000
    assert mission_elapsed_seconds([telemetry])==4
    telemetry['boot_ms'] = 10000
    record_arming_transition(telemetry,False)
    telemetry['armed'] = False
    telemetry['boot_ms'] = 20000
    assert mission_elapsed_seconds([telemetry])==9
    other = dict(armed=True,armed_since_ms=0,landed_disarmed_ms=0,boot_ms=11000)
    assert mission_elapsed_seconds([telemetry,other])==11
    other['armed'] = False
    assert mission_elapsed_seconds([other])==0
    record_arming_transition(other,True)
    assert other['armed_since_ms']==0 and other['landed_disarmed_ms'] is None
    assert mission_elapsed_seconds([])==0
    report = dict(first_arming_time_preserved=True,rearm_clears_old_disarm_time=True,
        armed_uses_current_boot_time=True,disarmed_final_time_frozen=True,zero_timestamps_not_missing=True,
        fleet_maximum_duration_checked=True,scope='FCU telemetry accounting; not wall-clock synchronization',hardware_verified=False)
    (ROOT/'validation/fcu_timing_logic_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
