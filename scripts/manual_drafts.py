from __future__ import annotations
import json
import math
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
from geodesy import simulation_frame
from shapely.geometry import LineString, Point, Polygon, shape


class ManualDraftStore:
    def __init__(self, root):
        self.directory = Path(root) / 'missions' / 'manual_drafts'
        self.lock = threading.RLock()

    def load(self, plan):
        with self.lock:
            path = self.directory / 'latest.json'
            if not path.exists():
                return dict(schema='zhixin/manual-draft/v1', map_origin_id=plan['map_frame']['map_origin_id'], features=[])
            return json.loads(path.read_text(encoding='utf-8'))

    def save(self, data, plan):
        if data.get('map_origin_id') != plan['map_frame']['map_origin_id']:
            raise ValueError('地图原点不一致，请导出原草稿并重新取点')
        features = data.get('features')
        if not isinstance(features, list) or len(features) > 200:
            raise ValueError('草稿需要 features 数组，最多200条')
        frame = simulation_frame(plan['datum'])
        perimeter, legal = shape(plan['perimeter']), shape(plan['legal'])
        owners = {'unassigned', *[vehicle['id'] for vehicle in plan['vehicles']]}
        result, warnings = [], []
        for index, feature in enumerate(features):
            if not isinstance(feature, dict):
                raise ValueError('绘制对象必须为JSON对象')
            kind, owner = feature.get('kind'), feature.get('vehicle_id', 'unassigned')
            if kind not in ('search_area', 'exclude_area', 'waypoints') or owner not in owners:
                raise ValueError('绘制类型或飞机编号无效')
            label, notes = feature.get('label', ''), feature.get('notes', '')
            if not isinstance(label, str) or not isinstance(notes, str) or len(label) > 120 or len(notes) > 2000:
                raise ValueError('名称或备注过长')
            points = feature.get('points_enu_m')
            minimum = 1 if kind == 'waypoints' else 3
            if not isinstance(points, list) or not minimum <= len(points) <= 1000:
                raise ValueError('区域至少3点，航点至少1点，每条最多1000点')
            coordinates = []
            for point in points:
                if not isinstance(point, list) or len(point) != 2 or any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or abs(value) > 10000 for value in point):
                    raise ValueError('每个绘制点需要两个有限ENU米坐标')
                coordinates.append([float(value) for value in point])
            geometry = Polygon(coordinates) if kind != 'waypoints' else LineString(coordinates) if len(coordinates) > 1 else Point(coordinates[0])
            if not geometry.is_valid or geometry.is_empty or (kind != 'waypoints' and geometry.area < 1):
                raise ValueError('区域自交或面积过小，请调整顶点')
            if not perimeter.buffer(.05).covers(geometry):
                warnings.append(f'第{index + 1}条超出比赛周界，优化时需要裁剪或修改')
            if kind == 'search_area' and not legal.buffer(.05).covers(geometry):
                warnings.append(f'第{index + 1}条搜索区包含禁搜区或非任务区，优化时需要剔除')
            wgs84 = []
            for point in coordinates:
                latitude, longitude, height = frame.reverse([*point, 0])
                wgs84.append([longitude, latitude])
            result.append(dict(id=f'feature_{index + 1}', kind=kind, vehicle_id=owner, label=label,
                notes=notes, points_enu_m=coordinates, points_wgs84_lon_lat=wgs84))
        stamp = datetime.now(timezone.utc)
        filename = stamp.strftime('%Y%m%dT%H%M%S%fZ') + '_' + uuid.uuid4().hex[:8] + '.json'
        saved = dict(schema='zhixin/manual-draft/v1', map_origin_id=frame.origin_id,
            map_frame=frame.descriptor(), subject=plan['subject'], saved_at=stamp.isoformat(),
            filename=filename, coordinate_order='WGS84 longitude,latitude; ENU east,north',
            usage='optimization_input_only_not_executable', features=result, warnings=warnings)
        payload = json.dumps(saved, ensure_ascii=False, indent=2, allow_nan=False)
        with self.lock:
            self.directory.mkdir(parents=True, exist_ok=True)
            (self.directory / filename).write_text(payload, encoding='utf-8')
            temporary = self.directory / 'latest.tmp'
            temporary.write_text(payload, encoding='utf-8')
            temporary.replace(self.directory / 'latest.json')
        return dict(draft=saved, saved_path=str(self.directory / filename))
