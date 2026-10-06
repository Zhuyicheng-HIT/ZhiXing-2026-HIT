#!/usr/bin/env python3
"""Generate two-vehicle-per-zone hover-and-scan missions."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import yaml

def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
        previous = current
    return inside

def segment_intersections(x, polygon):
    values = []
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if abs(x2 - x1) < 1e-9:
            if abs(x - x1) < 1e-6:
                values.extend([y1, y2])
            continue
        if min(x1, x2) <= x <= max(x1, x2):
            values.append(y1 + (x - x1) * (y2 - y1) / (x2 - x1))
    return sorted(values)

def sampled_segment(x, y0, y1, spacing):
    if y1 < y0:
        y0, y1 = y1, y0
    length = y1 - y0
    if length < 1e-6:
        return []
    count = max(1, math.ceil(length / spacing))
    return [(x, y0 + length * index / count) for index in range(count + 1)]

def generate_scan_points(polygon, config):
    margin = float(config["edge_margin_m"])
    spacing_x = float(config["track_spacing_m"])
    spacing_y = float(config["point_spacing_m"])
    min_x = min(item[0] for item in polygon) + margin
    max_x = max(item[0] for item in polygon) - margin
    if min_x >= max_x:
        raise ValueError("edge_margin_m leaves no scanable x range")
    count = max(1, math.ceil((max_x - min_x) / spacing_x))
    xs = [min_x + (max_x - min_x) * index / count for index in range(count + 1)]
    result = []
    for line_index, x in enumerate(xs):
        intersections = segment_intersections(x, polygon)
        line_points = []
        for index in range(0, len(intersections) - 1, 2):
            y0, y1 = intersections[index:index + 2]
            if y1 - y0 >= spacing_y * 0.5:
                line_points.extend(sampled_segment(x, y0 + margin, y1 - margin, spacing_y))
        if line_index % 2:
            line_points.reverse()
        result.extend(line_points)
    return [point for point in result if point_in_polygon(point, polygon)]

def split_route(points, count):
    return [points[round(len(points) * index / count):round(len(points) * (index + 1) / count)] for index in range(count)]

def build_mission(vehicle, zone_name, route, site, mission, part):
    altitude = float(mission["altitude_agl_m"])
    home = next(item for item in site["vehicles"] if item["id"] == vehicle["id"])["home"]
    waypoints = [{
        "index": index, "x_m": round(x, 3), "y_m": round(y, 3), "z_agl_m": altitude,
        "action": "HOVER_SCAN", "hover_stable_s": float(mission["hover_stable_s"]),
        "scan_timeout_s": float(mission["scan_timeout_s"]), "gimbal_command": "START_SCAN",
        "wait_for": "FINISHED|SUCCESS_FOUND|SUCCESS_NOT_FOUND"
    } for index, (x, y) in enumerate(route)]
    return {
        "schema": "zhixin-2026/mission/v1", "vehicle_id": vehicle["id"], "zone": zone_name,
        "zone_part": part, "frame": site["frame"], "datum": site["datum"],
        "home": {"x_m": home[0], "y_m": home[1], "z_agl_m": 0.5},
        "constraints": {"min_agl_m": 40.0, "max_agl_m": 120.0, "return_to_takeoff": True},
        "scan": {key: mission[key] for key in ("footprint_width_m", "footprint_height_m", "track_spacing_m", "point_spacing_m", "overlap_ratio")},
        "waypoints": waypoints
    }

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="../config/site.yaml")
    parser.add_argument("--output", default="../missions")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    data = yaml.safe_load((root / args.config).resolve().read_text(encoding="utf-8"))
    site, mission = data["site"], data["mission"]
    points = {int(key): tuple(value) for key, value in site["boundary_points_m"].items()}
    output = (root / args.output).resolve(); output.mkdir(parents=True, exist_ok=True)
    summary = {"schema": "zhixin-2026/mission-summary/v1", "altitude_agl_m": mission["altitude_agl_m"], "missions": []}
    for zone_name, zone in site["zones"].items():
        polygon = [points[index] for index in zone["vertices"]]
        full_route = generate_scan_points(polygon, mission)
        vehicles = [item for item in site["vehicles"] if item["id"] in zone["vehicle_ids"]]
        for part, (vehicle, route) in enumerate(zip(vehicles, split_route(full_route, len(vehicles))), start=1):
            result = build_mission(vehicle, zone_name, route, site, mission, part)
            filename = output / f"{vehicle['id']}_{zone_name}.json"
            filename.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            summary["missions"].append({"vehicle_id": vehicle["id"], "zone": zone_name, "waypoint_count": len(route), "file": filename.name})
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))

if __name__ == "__main__":
    main()
