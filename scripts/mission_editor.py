from __future__ import annotations
import json
import math
import threading
from pathlib import Path


EDITABLE_STATES = {'INGRESS', 'NAVIGATE', 'REPOSITION', 'EGRESS', 'RETURN'}


def mission_waypoints(plan):
    routes = []
    for vehicle in plan.get('vehicles', []):
        points = []
        for index, segment in enumerate(vehicle.get('segments', [])):
            if segment.get('state') not in EDITABLE_STATES:
                continue
            destination = segment.get('destination')
            if not isinstance(destination, list) or len(destination) < 3:
                continue
            points.append(dict(id=f"{vehicle['id']}:seg:{index}", segment_index=index,
                state=segment['state'], point=[float(value) for value in destination[:3]],
                editable=True, generated=True))
        routes.append(dict(vehicle_id=vehicle['id'], label=vehicle.get('label', vehicle['id']),
            home=list(vehicle.get('home', [0, 0])), points=points))
    return dict(schema='zhixin/mission-waypoints/v1', map_origin_id=plan['map_frame']['map_origin_id'], routes=routes,
        note='只读规划航点快照；编辑结果在重新规划时作为进出场/连接航线建议，不替换扫描站位。')


class MissionWaypointStore:
    def __init__(self, root):
        self.path = Path(root) / 'missions' / 'manual_drafts' / 'mission_waypoints.json'
        self.lock = threading.RLock()

    def load(self, plan, reset=False):
        with self.lock:
            if reset or not self.path.exists():
                return mission_waypoints(plan)
            data = json.loads(self.path.read_text(encoding='utf-8'))
            if data.get('map_origin_id') != plan['map_frame']['map_origin_id']:
                return mission_waypoints(plan)
            return data

    def save(self, data, plan):
        if data.get('map_origin_id') != plan['map_frame']['map_origin_id']:
            raise ValueError('航点文件与当前地图原点不一致')
        vehicles = {vehicle['id'] for vehicle in plan.get('vehicles', [])}
        routes = data.get('routes')
        if not isinstance(routes, list) or len(routes) != len(vehicles):
            raise ValueError('必须为每架飞机提供一条航线')
        result = []
        for route in routes:
            vehicle_id = route.get('vehicle_id')
            if vehicle_id not in vehicles:
                raise ValueError('未知飞机编号')
            points = route.get('points')
            if not isinstance(points, list) or len(points) > 300:
                raise ValueError('每架飞机最多300个航点')
            checked = []
            for point in points:
                value = point.get('point') if isinstance(point, dict) else point
                if not isinstance(value, list) or len(value) != 3 or any(isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item) or abs(item) > 10000 for item in value):
                    raise ValueError('航点必须是三个有限ENU坐标')
                checked.append(dict(id=point.get('id'), state=point.get('state', 'MANUAL'), point=[float(item) for item in value], editable=True, generated=False))
            result.append(dict(vehicle_id=vehicle_id, label=route.get('label', vehicle_id), home=route.get('home', []), points=checked))
        saved = dict(schema='zhixin/mission-waypoints/v1', map_origin_id=plan['map_frame']['map_origin_id'], routes=result,
            note='编辑后的航点建议；应用后需要重新生成时间、碰撞和覆盖检查。')
        with self.lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.path.write_text(json.dumps(saved, ensure_ascii=False, indent=2), encoding='utf-8')
        return saved
