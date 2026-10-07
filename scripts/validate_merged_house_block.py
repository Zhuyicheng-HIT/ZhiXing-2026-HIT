import json
from pathlib import Path

from shapely.geometry import shape
from shapely.ops import unary_union

from fleet_core import side_half_plane
from geodesy import simulation_frame


ROOT = Path(__file__).resolve().parents[1]


def validate(plan):
    blocks = plan.get('task_blocks', [])
    assert len(blocks) == 1, 'Expected one combined lower-house block'
    block = blocks[0]
    assert block['assigned_vehicle'] == 'uav_3'
    target = shape(block['geometry'])
    owners = []
    for vehicle in plan['vehicles']:
        scanned = unary_union([shape(task['geometry']) for task in vehicle['station_tasks']])
        if target.intersection(scanned).area > .01:
            owners.append(vehicle['id'])
        if vehicle['id'] == block['assigned_vehicle']:
            assert target.difference(scanned).area < .01, 'Owner leaves part of the block uncovered'
            marks = [task.get('task_block_id') == block['id'] for task in vehicle['station_tasks']]
            assert marks == sorted(marks, reverse=True), 'Combined block must be processed as one station group'
    assert owners == ['uav_3'], owners
    assert target.difference(shape(plan['legal'])).area < .01
    config = json.loads((ROOT/'config/area_adjustments.json').read_text(encoding='utf-8'))
    road = next(item for item in config['search_exclusions'] if item['id'] == config['takeoff_search_side']['road_exclusion_id'])
    frame = simulation_frame(plan['datum'])
    from shapely.geometry import Polygon
    road_shape = Polygon([frame.forward(latitude,longitude,plan['datum']['altitude_m'])[:2]
        for longitude,latitude in road['points_wgs84_lon_lat']])
    west = side_half_plane(road_shape.intersection(shape(plan['perimeter'])), 'west')
    for vehicle in plan['vehicles']:
        if vehicle['zone'] == 'takeoff':
            assert shape(vehicle['search_region']).difference(west).area < .01
            scans = unary_union([shape(task['geometry']) for task in vehicle['station_tasks']])
            assert scans.difference(west).area < .01
    for index, first in enumerate(plan['vehicles']):
        for second in plan['vehicles'][index+1:]:
            if first['zone'] == second['zone']:
                gap = shape(first['navigation_region']).distance(shape(second['navigation_region']))
                assert gap >= 5-1e-6, (first['id'],second['id'],gap)
    assert plan['duration_s'] <= 1200, plan['duration_s']
    assert plan['coverage']['ratio'] >= .999999
    assert plan['validation']['passed'], plan['validation']
    return dict(passed=True, owner=owners[0], area_m2=round(target.area,2), duration_s=plan['duration_s'])


if __name__ == '__main__':
    plan = json.loads((ROOT/'missions/fleet_plan.json').read_text(encoding='utf-8'))
    print(json.dumps(validate(plan), ensure_ascii=False, indent=2))
