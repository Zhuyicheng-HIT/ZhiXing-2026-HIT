#!/usr/bin/env python3
"""Replay the mission state machine without requiring a live flight controller."""
from __future__ import annotations
import argparse
import json
from pathlib import Path

def replay(path: Path, scan_seconds: float) -> int:
    mission = json.loads(path.read_text(encoding="utf-8"))
    previous = None
    for waypoint in mission["waypoints"]:
        if previous is not None and waypoint["index"] != previous + 1:
            raise ValueError(f"non-sequential waypoint index at {waypoint['index']}")
        previous = waypoint["index"]
        print(f"{mission['vehicle_id']} WP{waypoint['index']:03d} NAVIGATE ({waypoint['x_m']:.1f}, {waypoint['y_m']:.1f}, {waypoint['z_agl_m']:.1f})")
        print(f"{mission['vehicle_id']} WP{waypoint['index']:03d} HOVER_STABLE {waypoint['hover_stable_s']:.1f}s")
        print(f"{mission['vehicle_id']} WP{waypoint['index']:03d} {waypoint['gimbal_command']} -> WAIT_FOR {waypoint['wait_for']}")
        print(f"{mission['vehicle_id']} WP{waypoint['index']:03d} SCAN_FINISHED after {scan_seconds:.1f}s")
    print(f"{mission['vehicle_id']} COMPLETE {len(mission['waypoints'])} waypoints")
    return 0

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mission", type=Path)
    parser.add_argument("--scan-seconds", type=float, default=5.0)
    args = parser.parse_args()
    return replay(args.mission, args.scan_seconds)

if __name__ == "__main__":
    raise SystemExit(main())
