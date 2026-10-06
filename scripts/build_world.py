#!/usr/bin/env python3
"""Build the Gazebo Harmonic world from surveyed local ENU polygons."""
from __future__ import annotations
import math
from pathlib import Path
import yaml

def box(name, x, y, z, sx, sy, sz, color):
    return f'''    <model name="{name}"><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose><link name="link"><collision name="collision"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision><visual name="visual"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>'''

def edge_box(name, p0, p1, color, width=0.8, height=0.12):
    x0, y0 = p0; x1, y1 = p1
    length = math.hypot(x1 - x0, y1 - y0); angle = math.atan2(y1 - y0, x1 - x0)
    return f'''    <model name="{name}"><static>true</static><pose>{(x0+x1)/2:.3f} {(y0+y1)/2:.3f} {height/2:.3f} 0 0 {angle:.6f}</pose><link name="link"><collision name="collision"><geometry><box><size>{length:.3f} {width:.3f} {height:.3f}</size></box></geometry></collision><visual name="visual"><geometry><box><size>{length:.3f} {width:.3f} {height:.3f}</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>'''

def main():
    root = Path(__file__).resolve().parents[1]
    site = yaml.safe_load((root / "config/site.yaml").read_text(encoding="utf-8"))["site"]
    points = {int(key): tuple(value) for key, value in site["boundary_points_m"].items()}
    models = []
    boundary = [points[index] for index in [6, 8, 9, 15, 16, 19, 20, 22, 23, 25, 1]]
    for index, (p0, p1) in enumerate(zip(boundary, boundary[1:] + boundary[:1])):
        models.append(edge_box(f"competition_boundary_{index}", p0, p1, "0.55 0.08 0.72 1"))
    colors = {"takeoff": "0.10 0.35 0.90 1", "house": "0.90 0.45 0.05 1", "grass": "0.15 0.65 0.18 1"}
    for zone_name, zone in site["zones"].items():
        polygon = [points[index] for index in zone["vertices"]]
        for index, (p0, p1) in enumerate(zip(polygon, polygon[1:] + polygon[:1])):
            models.append(edge_box(f"{zone_name}_boundary_{index}", p0, p1, colors[zone_name], 1.2, 0.18))
    for index, (x, y, sx, sy, sz) in enumerate([(235, 220, 38, 24, 8), (285, 315, 32, 28, 11), (345, 390, 45, 26, 7), (115, 85, 28, 22, 6)]):
        models.append(box(f"house_obstacle_{index}", x, y, sz / 2, sx, sy, sz, "0.35 0.35 0.38 1"))
    for index, (x, y, r) in enumerate([(50, -155, 8), (95, -245, 10), (430, 70, 12), (520, 250, 9), (480, -280, 10)]):
        models.append(box(f"grass_patch_{index}", x, y, 0.06, r * 2, r * 2, 0.12, "0.18 0.50 0.12 1"))
    uavs = []
    for vehicle in site["vehicles"]:
        x, y = vehicle["home"]
        uavs.append(f'''    <include><uri>model://iris_with_gimbal</uri><name>{vehicle["id"]}</name><pose>{x:.3f} {y:.3f} 0.8 0 0 0</pose></include>''')
    world = f'''<?xml version="1.0"?>
<sdf version="1.9"><world name="zhixin_2026_ke_mu_1_2">
  <gravity>0 0 -9.81</gravity><physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <scene><ambient>0.45 0.45 0.45 1</ambient><background>0.70 0.78 0.88 1</background><shadows>true</shadows></scene>
  <light name="sun" type="directional"><cast_shadows>true</cast_shadows><pose>0 0 700 0.35 -0.45 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.25 0.35 -0.90</direction></light>
  <model name="ground"><static>true</static><link name="link"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>1600 1400</size></plane></geometry></collision><visual name="visual"><geometry><plane><normal>0 0 1</normal><size>1600 1400</size></plane></geometry><material><ambient>0.33 0.43 0.25 1</ambient><diffuse>0.33 0.43 0.25 1</diffuse></material></visual></link></model>
{chr(10).join(models)}
{chr(10).join(uavs)}
</world></sdf>
'''
    (root / "worlds/zhixin_2026_ke_mu_1_2.sdf").write_text(world, encoding="utf-8")
    print(root / "worlds/zhixin_2026_ke_mu_1_2.sdf")

if __name__ == "__main__":
    main()
