import json
import tempfile
from pathlib import Path
from shapely.geometry import Polygon, mapping
from geodesy import simulation_frame
from manual_drafts import ManualDraftStore


def main():
    datum = dict(latitude_deg=33.864, longitude_deg=113.707, sitl_msl_altitude_m=71, sitl_geoid_undulation_m=0)
    frame = simulation_frame(datum)
    plan = dict(datum=datum, map_frame=frame.descriptor(), subject=1,
        vehicles=[dict(id='uav_1')], perimeter=mapping(Polygon([[-100,-100],[100,-100],[100,100],[-100,100]])),
        legal=mapping(Polygon([[-50,-50],[50,-50],[50,50],[-50,50]])))
    feature = dict(kind='search_area', vehicle_id='uav_1', label='test', notes='', points_enu_m=[[0,0],[20,0],[20,20],[0,20]])
    data = dict(map_origin_id=frame.origin_id, features=[feature])
    with tempfile.TemporaryDirectory() as directory:
        store = ManualDraftStore(directory)
        assert store.load(plan)['features'] == []
        result = store.save(data, plan)
        saved = store.load(plan)
        assert saved == result['draft']
        assert Path(result['saved_path']).exists()
        assert saved['warnings'] == []
        assert abs(saved['features'][0]['points_wgs84_lon_lat'][0][0] - datum['longitude_deg']) < 1e-7
        assert abs(saved['features'][0]['points_wgs84_lon_lat'][0][1] - datum['latitude_deg']) < 1e-7
        route = dict(feature, kind='waypoints', points_enu_m=[[0,0],[150,0]])
        assert store.save(dict(data, features=[route]), plan)['draft']['warnings']
        area = dict(feature, points_enu_m=[[40,40],[60,40],[60,60],[40,60]])
        assert store.save(dict(data, features=[area]), plan)['draft']['warnings']
        invalid = [dict(data, map_origin_id='wrong'), dict(data, features=[dict(feature, vehicle_id='uav_9')]),
            dict(data, features=[dict(feature, points_enu_m=[[0,0],[20,20],[0,20],[20,0]])]),
            dict(data, features=[dict(feature, points_enu_m=[[0,0],[float('nan'),0],[20,20]])]),
            dict(data, features=[dict(feature, points_enu_m=[[0,0],[True,0],[20,20]])])]
        for bad in invalid:
            before = store.load(plan)
            try:
                store.save(bad, plan)
                raise AssertionError('invalid draft accepted')
            except ValueError:
                assert store.load(plan) == before
        assert len(list(store.directory.glob('*.json'))) == 4
        store.save(dict(data, features=[]), plan)
        assert store.load(plan)['features'] == []
    print(json.dumps(dict(passed=True, checks=['save_load', 'history', 'wgs84', 'bounds_warnings', 'invalid_rejection', 'empty_draft']), indent=2))


if __name__ == '__main__':
    main()
