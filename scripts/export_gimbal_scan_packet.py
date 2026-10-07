import argparse
import hashlib
import json
import zipfile
from pathlib import Path

from geodesy import simulation_frame


ROOT = Path(__file__).resolve().parents[1]


def export(plan_path, output):
    content = plan_path.read_bytes()
    plan = json.loads(content)
    frame = simulation_frame(plan['datum'])
    aircraft = []
    for vehicle in plan['vehicles']:
        stations = []
        for index, position in enumerate(vehicle['stations']):
            latitude, longitude, altitude = frame.reverse([*position, vehicle['work_altitude_m']])
            legs = []
            for segment in vehicle['segments']:
                if segment['state'] != 'SCAN' or segment.get('station') != index:
                    continue
                motion = segment['scan_motion']
                legs.append(dict(command_id=segment['command_id'],
                    start_s=segment['start'], duration_s=segment['end']-segment['start'],
                    ground_origin_enu_m=motion['origin_enu_m'],
                    ground_destination_enu_m=motion['destination_enu_m'],
                    ground_speed_m_s=motion['maximum_ground_speed_m_s'],
                    rapid_repoint=segment.get('rapid_repoint', False)))
            stations.append(dict(index=index, aircraft_enu_m=[*position, vehicle['work_altitude_m']],
                longitude_deg=longitude, latitude_deg=latitude,
                task_block_id=vehicle['station_tasks'][index].get('task_block_id'), legs=legs))
        aircraft.append(dict(id=vehicle['id'], zone=vehicle['zone'],
            entry_altitude_m=vehicle['entry_altitude_m'], scan_configuration=vehicle['scan_configuration'],
            stations=stations))
    packet = dict(schema='zhixin/gimbal-scan-packet/v1',
        source_plan_sha256=hashlib.sha256(content).hexdigest(),
        map_frame=plan['map_frame'], duration_s=plan['duration_s'],
        task_blocks=plan.get('task_blocks', []), aircraft=aircraft,
        evidence_scope='nominal kinematic plan; camera and SDK zoom not calibrated; not flight authorization')
    output.parent.mkdir(parents=True, exist_ok=True)
    packet_path = output.with_suffix('.json')
    packet_path.write_text(json.dumps(packet, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    instructions = ('# Gimbal scan packet\n\n'
        'Coordinates use the supplied common WGS84-derived ENU map frame, in metres. '
        'Longitude and latitude are degrees. Aircraft altitude is relative to the map origin.\n\n'
        'Each station holds the aircraft fixed; legs move the optical-axis ground target, '
        'not the aircraft. start_s is relative to simultaneous takeoff. Ground speed and '
        'duration are nominal flat-ground values. Rapid repoints are not timed in simulation.\n\n'
        'task_block_id groups the lower-house remainder under one owner. No-search holes '
        'remain excluded; this is one logical task, not a filled polygon.\n\n'
        'Camera frame, focal ratio and SDK zoom require physical calibration. Recovery '
        'must preserve command_id and the original checkpoint; do not automatically RTL '
        'for a recoverable communication or gimbal interruption.\n')
    with zipfile.ZipFile(output.with_suffix('.zip'), 'w', zipfile.ZIP_DEFLATED) as archive:
        archive.write(packet_path, packet_path.name)
        archive.writestr('README.md', instructions)
    print(packet_path)
    print(output.with_suffix('.zip'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--plan', type=Path, default=ROOT/'missions/fleet_plan.json')
    parser.add_argument('--output', type=Path, default=ROOT/'missions/gimbal_scan_packet')
    arguments = parser.parse_args()
    export(arguments.plan, arguments.output)
