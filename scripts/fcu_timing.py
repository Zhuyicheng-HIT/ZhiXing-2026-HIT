from __future__ import annotations


def record_arming_transition(telemetry, armed):
    if armed and not telemetry['armed']:
        if telemetry['armed_since_ms'] is None:
            telemetry['armed_since_ms'] = telemetry['boot_ms']
        telemetry['landed_disarmed_ms'] = None
    elif telemetry['armed'] and not armed:
        telemetry['landed_disarmed_ms'] = telemetry['boot_ms']


def mission_elapsed_seconds(telemetry):
    durations = []
    for item in telemetry:
        start = item['armed_since_ms']
        if start is None:
            continue
        end = item['boot_ms'] if item['armed'] or item['landed_disarmed_ms'] is None else item['landed_disarmed_ms']
        durations.append((end-start)/1000)
    return max(durations,default=0.0)
