from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
from urllib.error import URLError
from urllib.request import Request, urlopen


def get_json(url: str) -> dict:
    with urlopen(url, timeout=2) as response:
        return json.loads(response.read().decode('utf-8'))


def post_json(url: str, payload: dict) -> dict:
    request = Request(url, data=json.dumps(payload).encode('utf-8'), method='POST',
                      headers={'Content-Type': 'application/json'})
    with urlopen(request, timeout=2) as response:
        return json.loads(response.read().decode('utf-8'))


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


def command_angles(command: dict | None) -> tuple[float, float]:
    pointing = (command or {}).get('pointing_at_issue') or {}
    yaw = pointing.get('azimuth_rad')
    pitch = pointing.get('elevation_rad')
    if yaw is None:
        yaw = math.radians(float(pointing.get('azimuth_deg', 0.0)))
    if pitch is None:
        pitch = math.radians(float(pointing.get('elevation_deg', -90.0)))
    return clamp(float(yaw), -math.pi, math.pi), clamp(float(pitch), -math.pi / 2, math.pi / 2)


def publish(topic: str, value: float, *, dry_run: bool) -> None:
    if dry_run:
        return
    subprocess.run(
        ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double', '-p', f'data: {value:.9f}'],
        check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


def identity(command: dict) -> dict:
    return {name: command[name] for name in (
        'vehicle_id', 'command_id', 'mission_epoch', 'plan_revision', 'point_revision', 'attempt_id')}


def main() -> None:
    parser = argparse.ArgumentParser(description='将仿真任务云台指令驱动到六架 Gazebo 云台关节')
    parser.add_argument('--state-url', default='http://127.0.0.1:8770/api/state')
    parser.add_argument('--command-url', default='http://127.0.0.1:8770/api/gimbal/command')
    parser.add_argument('--status-url', default='http://127.0.0.1:8770/api/gimbal/status')
    parser.add_argument('--period-s', type=float, default=.2)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--executor-session', default='gazebo-gimbal-bridge')
    parser.add_argument('--vehicles', nargs='+', default=[f'uav_{index}' for index in range(1, 7)])
    args = parser.parse_args()
    accepted: set[tuple[str, str, str]] = set()
    running: set[tuple[str, str, str]] = set()
    topics = {
        vehicle_id: (
            f'/uav/{vehicle_id}/gimbal/cmd_yaw',
            f'/uav/{vehicle_id}/gimbal/cmd_pitch',
        ) for vehicle_id in args.vehicles
    }
    while True:
        for vehicle_id in args.vehicles:
            try:
                response = get_json(args.command_url + '?vehicle_id=' + vehicle_id)
            except (OSError, URLError, ValueError):
                continue
            command = response.get('command') if response.get('active') else None
            yaw, pitch = command_angles(command)
            try:
                publish(topics[vehicle_id][0], yaw, dry_run=args.dry_run)
                publish(topics[vehicle_id][1], pitch, dry_run=args.dry_run)
            except (OSError, subprocess.CalledProcessError):
                continue
            if not command or args.dry_run:
                continue
            key = (command['vehicle_id'], command['command_id'], command['attempt_id'])
            base = identity(command)
            if key not in accepted:
                try:
                    post_json(args.status_url, dict(base, status='ACCEPTED',
                                                    executor_session=args.executor_session))
                    accepted.add(key)
                except (OSError, URLError, ValueError):
                    continue
            if key not in running:
                try:
                    post_json(args.status_url, dict(base, status='RUNNING',
                                                    executor_session=args.executor_session))
                    running.add(key)
                except (OSError, URLError, ValueError):
                    continue
        time.sleep(max(.02, args.period_s))


if __name__ == '__main__':
    main()
