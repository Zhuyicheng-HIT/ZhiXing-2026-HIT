from __future__ import annotations

import asyncio
import importlib
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, AsyncIterator, Iterable


@dataclass(frozen=True)
class ZR10CameraModel:
    horizontal_fov_deg: float = 61.5
    aspect_ratio: float = 16 / 9
    minimum_zoom: float = 1.0
    maximum_zoom: float = 30.0

    def focal_lengths(self, width: int, height: int, zoom: float = 1.0) -> tuple[float, float]:
        if width <= 0 or height <= 0 or zoom <= 0:
            raise ValueError('invalid image size or zoom')
        horizontal_fov = math.radians(self.horizontal_fov_deg) / zoom
        fx = width / (2 * math.tan(horizontal_fov / 2))
        fy = fx * width / height
        return fx, fy


@dataclass(frozen=True)
class GimbalState:
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    zoom: float
    timestamp: float


@dataclass(frozen=True)
class Detection:
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float
    timestamp: float

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2)


@dataclass(frozen=True)
class GroundObservation:
    east_m: float
    north_m: float
    height_m: float
    bearing_deg: float
    slant_range_m: float
    pixel: tuple[float, float]
    timestamp: float


def _unit(values: Iterable[float]) -> tuple[float, float, float]:
    values = tuple(float(value) for value in values)
    length = math.sqrt(sum(value * value for value in values))
    if length < 1e-9:
        raise ValueError('zero vector')
    return tuple(value / length for value in values)


def backproject_ground_pixel(
    *,
    pixel: tuple[float, float],
    image_size: tuple[int, int],
    vehicle_enu_m: tuple[float, float, float],
    vehicle_yaw_deg: float,
    gimbal_yaw_deg: float,
    gimbal_pitch_deg: float,
    zoom: float = 1.0,
    model: ZR10CameraModel = ZR10CameraModel(),
) -> GroundObservation:
    width, height = image_size
    fx, fy = model.focal_lengths(width, height, zoom)
    px, py = pixel
    cx, cy = width / 2, height / 2
    ray_camera = _unit(((px - cx) / fx, (py - cy) / fy, 1.0))
    yaw = math.radians(vehicle_yaw_deg + gimbal_yaw_deg)
    pitch = math.radians(gimbal_pitch_deg)
    forward = (math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch))
    right = (-math.sin(yaw), math.cos(yaw), 0.0)
    up = (-math.sin(pitch) * math.cos(yaw), -math.sin(pitch) * math.sin(yaw), math.cos(pitch))
    ray_enu = _unit(tuple(ray_camera[0] * right[index] + ray_camera[1] * up[index] + ray_camera[2] * forward[index] for index in range(3)))
    if ray_enu[2] >= -1e-9:
        raise ValueError('camera ray does not intersect ground below vehicle')
    scale = -vehicle_enu_m[2] / ray_enu[2]
    east = vehicle_enu_m[0] + scale * ray_enu[0]
    north = vehicle_enu_m[1] + scale * ray_enu[1]
    return GroundObservation(
        east_m=east,
        north_m=north,
        height_m=vehicle_enu_m[2],
        bearing_deg=math.degrees(math.atan2(east - vehicle_enu_m[0], north - vehicle_enu_m[1])),
        slant_range_m=scale,
        pixel=pixel,
        timestamp=time.time(),
    )


class ZR10Bridge:
    def __init__(self, host: str = '192.168.144.25', port: int = 37260, *, sdk_root: str | Path | None = None,
                 rtsp_url: str | None = None, simulate: bool = False, camera_model: ZR10CameraModel | None = None):
        self.host = host
        self.port = port
        self.sdk_root = Path(sdk_root or os.environ.get('SIYI_SDK_ROOT', '')).expanduser() if (sdk_root or os.environ.get('SIYI_SDK_ROOT')) else None
        self.rtsp_url = rtsp_url or f'rtsp://{host}:8554/main.264'
        self.simulate = simulate
        self.camera_model = camera_model or ZR10CameraModel()
        self.client: Any = None
        self.stream: Any = None
        self.state = GimbalState(0.0, -90.0, 0.0, 1.0, time.time())

    def _load_sdk(self) -> Any:
        if self.sdk_root:
            root = str(self.sdk_root)
            if root not in sys.path:
                sys.path.insert(0, root)
        return importlib.import_module('siyi_sdk.convenience')

    async def connect(self) -> None:
        if self.simulate:
            return
        convenience = self._load_sdk()
        self.client = await convenience.connect_udp(self.host, self.port, auto_reconnect=True)
        await self.client.set_attitude_nowait(self.state.yaw_deg, self.state.pitch_deg)

    async def close(self) -> None:
        if self.stream is not None:
            await self.stream.stop()
            self.stream = None
        if self.client is not None:
            await self.client.close()
            self.client = None

    async def set_attitude(self, yaw_deg: float, pitch_deg: float) -> GimbalState:
        yaw_deg = max(-180.0, min(180.0, float(yaw_deg)))
        pitch_deg = max(-90.0, min(25.0, float(pitch_deg)))
        if self.client is not None:
            await self.client.set_attitude(yaw_deg, pitch_deg)
        self.state = GimbalState(yaw_deg, pitch_deg, self.state.roll_deg, self.state.zoom, time.time())
        return self.state

    async def set_zoom(self, zoom: float) -> GimbalState:
        zoom = max(self.camera_model.minimum_zoom, min(self.camera_model.maximum_zoom, float(zoom)))
        if self.client is not None:
            await self.client.absolute_zoom(zoom)
        self.state = GimbalState(self.state.yaw_deg, self.state.pitch_deg, self.state.roll_deg, zoom, time.time())
        return self.state

    async def gimbal_state(self) -> GimbalState:
        if self.client is not None:
            attitude = await self.client.get_gimbal_attitude()
            zoom = await self.client.get_current_zoom()
            self.state = GimbalState(attitude.yaw_deg, attitude.pitch_deg, attitude.roll_deg, zoom, time.time())
        return self.state

    async def frames(self) -> AsyncIterator[Any]:
        if self.simulate:
            return
        if self.stream is None:
            stream = self.client.create_stream(self.rtsp_url, backend='opencv')
            self.stream = stream
            await stream.start()
        async for frame in self.stream.frames():
            yield frame

    async def track_detection(self, detection: Detection, image_size: tuple[int, int], *, deadband_px: float = 12.0) -> GimbalState:
        width, height = image_size
        cx, cy = detection.center
        dx, dy = cx - width / 2, cy - height / 2
        state = await self.gimbal_state()
        yaw_per_pixel = self.camera_model.horizontal_fov_deg / max(width, 1) / state.zoom
        pitch_per_pixel = self.camera_model.horizontal_fov_deg / max(width, 1) / state.zoom * width / max(height, 1)
        yaw = state.yaw_deg if abs(dx) <= deadband_px else state.yaw_deg + dx * yaw_per_pixel
        pitch = state.pitch_deg if abs(dy) <= deadband_px else state.pitch_deg - dy * pitch_per_pixel
        return await self.set_attitude(yaw, pitch)


class YOLODetector:
    def __init__(self, weights: str | Path, classes: set[str] | None = None, confidence: float = 0.4):
        from ultralytics import YOLO
        self.model = YOLO(str(weights))
        self.classes = classes
        self.confidence = confidence

    def detect(self, frame: Any) -> list[Detection]:
        result = self.model.predict(frame, conf=self.confidence, verbose=False)[0]
        names = result.names
        detections: list[Detection] = []
        for box in result.boxes:
            confidence = float(box.conf[0])
            class_name = str(names[int(box.cls[0])])
            if self.classes and class_name not in self.classes:
                continue
            x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
            detections.append(Detection(class_name, confidence, x1, y1, x2, y2, time.time()))
        return detections


async def main() -> None:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='192.168.144.25')
    parser.add_argument('--port', type=int, default=37260)
    parser.add_argument('--sdk-root', default=None)
    parser.add_argument('--simulate', action='store_true')
    parser.add_argument('--yaw', type=float, default=0)
    parser.add_argument('--pitch', type=float, default=-90)
    parser.add_argument('--zoom', type=float, default=1)
    args = parser.parse_args()
    bridge = ZR10Bridge(args.host, args.port, sdk_root=args.sdk_root, simulate=args.simulate)
    await bridge.connect()
    try:
        await bridge.set_zoom(args.zoom)
        state = await bridge.set_attitude(args.yaw, args.pitch)
        print(state)
    finally:
        await bridge.close()


if __name__ == '__main__':
    asyncio.run(main())
