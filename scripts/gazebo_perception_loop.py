from __future__ import annotations

import argparse
import asyncio
import json
import time
from pathlib import Path
from urllib.request import Request, urlopen

from PIL import Image

from zr10_bridge import ZR10Bridge, backproject_ground_pixel


def get_json(url: str) -> dict:
    with urlopen(url, timeout=3) as response:
        return json.loads(response.read().decode('utf-8'))


def write_atomic(target: Path, value: dict) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    temporary.replace(target)


async def run(args: argparse.Namespace) -> None:
    detector = None
    if args.weights:
        from zr10_bridge import YOLODetector
        detector = YOLODetector(args.weights, confidence=args.confidence)
    bridges = {vehicle_id: ZR10Bridge(simulate=not args.real_zr10, host=args.zr10_host)
               for vehicle_id in args.vehicles}
    if args.real_zr10:
        for bridge in bridges.values():
            await bridge.connect()
    try:
        while True:
            state = get_json(args.state_url)
            by_id = {vehicle['id']: vehicle for vehicle in state.get('vehicles', [])}
            for vehicle_id, bridge in bridges.items():
                frame_path = args.camera_dir / f'{vehicle_id}.jpg'
                record = dict(vehicle_id=vehicle_id, timestamp=time.time(), frame=str(frame_path), detections=[])
                if frame_path.is_file() and vehicle_id in by_id:
                    image = Image.open(frame_path).convert('RGB')
                    width, height = image.size
                    detections = detector.detect(image) if detector else []
                    vehicle = by_id[vehicle_id]
                    position = tuple(float(value) for value in vehicle['position'])
                    gimbal = vehicle.get('gimbal', {})
                    for detection in detections:
                        observation = backproject_ground_pixel(
                            pixel=detection.center,
                            image_size=(width, height),
                            vehicle_enu_m=position,
                            vehicle_yaw_deg=0.0,
                            gimbal_yaw_deg=float(gimbal.get('azimuth_enu_deg', 0.0)),
                            gimbal_pitch_deg=float(gimbal.get('elevation_deg', -90.0)),
                        )
                        command_state = await bridge.track_detection(detection, (width, height))
                        record['detections'].append(dict(
                            class_name=detection.class_name,
                            confidence=detection.confidence,
                            pixel=detection.center,
                            ground_enu_m=[observation.east_m, observation.north_m, 0.0],
                            slant_range_m=observation.slant_range_m,
                            gimbal=dict(yaw_deg=command_state.yaw_deg, pitch_deg=command_state.pitch_deg,
                                        zoom=command_state.zoom),
                        ))
                write_atomic(args.output_dir / f'{vehicle_id}.json', record)
            if args.once:
                return
            await asyncio.sleep(args.period_s)
    finally:
        for bridge in bridges.values():
            await bridge.close()


def main() -> None:
    parser = argparse.ArgumentParser(description='Gazebo image -> YOLO -> ZR-10 tracking loop')
    parser.add_argument('--state-url', default='http://127.0.0.1:8765/api/state')
    parser.add_argument('--camera-dir', type=Path, default=Path('runtime/camera'))
    parser.add_argument('--output-dir', type=Path, default=Path('runtime/perception'))
    parser.add_argument('--weights')
    parser.add_argument('--confidence', type=float, default=.4)
    parser.add_argument('--period-s', type=float, default=.2)
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--real-zr10', action='store_true')
    parser.add_argument('--zr10-host', default='192.168.144.25')
    parser.add_argument('--vehicles', nargs='+', default=[f'uav_{index}' for index in range(1, 7)])
    asyncio.run(run(parser.parse_args()))


if __name__ == '__main__':
    main()
