from __future__ import annotations

import argparse
import base64
import json
import subprocess
import time
from pathlib import Path

from PIL import Image


def read_frame(topic: str, timeout_s: float = 8.0) -> tuple[bytes, dict]:
    command = ['gz', 'topic', '-e', '-n', '1', '--json-output', '-t', topic]
    completed = subprocess.run(command, check=True, capture_output=True, timeout=timeout_s)
    payload = json.loads(completed.stdout.decode('utf-8'))
    width = int(payload['width'])
    height = int(payload['height'])
    step = int(payload.get('step', width * 3))
    raw = base64.b64decode(payload['data'])
    if step < width * 3 or len(raw) < step * height:
        raise ValueError(f'invalid image payload for {topic}')
    image = Image.frombytes('RGB', (width, height), raw, 'raw', 'RGB', step)
    output = __import__('io').BytesIO()
    image.save(output, format='JPEG', quality=82, optimize=True)
    return output.getvalue(), dict(width=width, height=height, step=step, stamp=payload.get('header', {}).get('stamp'))


def capture_once(vehicle_id: str, output_dir: Path) -> None:
    topic = f'/uav/{vehicle_id}/gimbal/image_raw'
    jpeg, metadata = read_frame(topic)
    output_dir.mkdir(parents=True, exist_ok=True)
    temporary = output_dir / f'{vehicle_id}.jpg.tmp'
    target = output_dir / f'{vehicle_id}.jpg'
    temporary.write_bytes(jpeg)
    temporary.replace(target)
    (output_dir / f'{vehicle_id}.json').write_text(json.dumps(metadata), encoding='utf-8')


def main() -> None:
    parser = argparse.ArgumentParser(description='Bridge Gazebo camera topics to latest JPEG frames')
    parser.add_argument('--output-dir', type=Path, default=Path('runtime/camera'))
    parser.add_argument('--vehicles', nargs='+', default=[f'uav_{index}' for index in range(1, 7)])
    parser.add_argument('--period-s', type=float, default=0.25)
    parser.add_argument('--once', action='store_true')
    args = parser.parse_args()
    while True:
        started = time.monotonic()
        for vehicle_id in args.vehicles:
            try:
                capture_once(vehicle_id, args.output_dir)
            except (OSError, ValueError, subprocess.SubprocessError, json.JSONDecodeError) as error:
                print(f'{vehicle_id}: {error}', flush=True)
        if args.once:
            return
        time.sleep(max(0.0, args.period_s - (time.monotonic() - started)))


if __name__ == '__main__':
    main()
