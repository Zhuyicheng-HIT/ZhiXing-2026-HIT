from __future__ import annotations
import json
import math
from copy import deepcopy
from shapely.geometry import box
from shapely.ops import unary_union
from fleet_core import ROOT, build_plan
from regional_scan import regional_scan
from scan_protocol import ScanProtocol


def main():
    profile = json.loads((ROOT/'config/scan_profile.json').read_text(encoding='utf-8'))
    plan = build_plan()
    expected_entry_altitudes = {'uav_1':45,'uav_2':40.5,'uav_3':50,'uav_4':55,'uav_5':60,'uav_6':65}
    assert {vehicle['id']:vehicle['entry_altitude_m'] for vehicle in plan['vehicles']}==expected_entry_altitudes
    entry_order = ['uav_6','uav_4','uav_2','uav_5','uav_3','uav_1']
    assert plan['coordination']['entry_order']==entry_order
    assert [vehicle['id'] for vehicle in sorted(plan['vehicles'],key=lambda item:item['entry_release_s'])]==entry_order
    for vehicle in plan['vehicles']:
        altitude = expected_entry_altitudes[vehicle['id']]
        assert next(segment for segment in vehicle['segments'] if segment['state']=='INGRESS')['destination'][2]==altitude
        assert next(segment for segment in vehicle['segments'] if segment['state']=='EGRESS')['destination'][2]==altitude
        assert next(segment for segment in vehicle['segments'] if segment['state']=='RETURN_CLIMB')['destination'][2]==altitude
        assert next(segment for segment in vehicle['segments'] if segment['state']=='DESCEND_TO_WORK')['destination'][2]==vehicle['work_altitude_m']
    protocol = ScanProtocol(plan,profile)
    releases = sorted(vehicle['entry_release_s'] for vehicle in plan['vehicles'])
    assert all(math.isclose(second-first,2) for first,second in zip(releases,releases[1:]))
    assert releases==[5,7,9,11,13,15]
    actual_climb_starts = sorted(next(segment['start'] for segment in vehicle['segments'] if segment['state']=='CLIMB') for vehicle in plan['vehicles'])
    assert actual_climb_starts==releases
    for vehicle in plan['vehicles']:
        config = vehicle['scan_configuration']
        frame_width,frame_height = config['instantaneous_frame_m']
        radius_x,radius_y = frame_width/2,frame_height/2
        width = config['station_footprint_m']
        fields = [config['initial_look_offset_enu_m'],*config['offsets_enu_m']]
        patches = unary_union([box(east-radius_x,north-radius_y,east+radius_x,north+radius_y) for east,north in fields])
        assert box(-width/2,-width/2,width/2,width/2).difference(patches).area<.01
        assert math.isclose(config['ground_scan_speed_m_s'],4)
        assert math.isclose(frame_width/frame_height,16/9)
        assert config['raw_nadir_frame_m'][0]>=frame_width and config['raw_nadir_frame_m'][1]>=frame_height
        assert vehicle['work_altitude_m']==(60 if vehicle['zone']=='grass' else 40.5)
        for segment in vehicle['segments']:
            if segment['state']=='SCAN':
                assert protocol.observations[segment['command_id']]['scan_zoom']==config['scan_zoom']
        expected_station_s = (config['grid_rows']+1)*max(0,width-frame_height)/4
        for station_index in range(len(vehicle['stations'])):
            scans = [segment for segment in vehicle['segments'] if segment['state']=='SCAN' and segment['station']==station_index]
            if 'station_tasks' in vehicle:
                task = vehicle['station_tasks'][station_index]
                from shapely.geometry import shape
                actual = unary_union([shape(segment['scan_footprint']) for segment in scans])
                assert shape(task['geometry']).difference(actual).area<.01
            else:
                assert math.isclose(sum(segment['end']-segment['start'] for segment in scans),expected_station_s,abs_tol=1e-7)
            assert protocol.observations[scans[0]['command_id']]['rapid_repoint_target_enu_m']==scans[0]['scan_motion']['origin_enu_m']
    changed = deepcopy(profile)
    changed['region_scans']['house']['minimum_overlap_ratio'] = .4
    try:
        ScanProtocol(plan,changed)
    except ValueError:
        pass
    else:
        raise AssertionError('Changed regional scan profile must require replan')
    at_40 = regional_scan(profile,'house',80,40.5)
    report = dict(passed=True,scope='nominal field tiling, timing and command identity; not optical calibration or native flight',
        manual_source='ZR10 user manual v1.9 page 15: horizontal FOV 61.5 degrees, 16:9 video, 10x optical / 30x hybrid',
        zoom_at_40_5m_for_20m_frame_height=at_40['scan_zoom'],
        raw_frame_at_40_5m_m=at_40['raw_nadir_frame_m'],
        common_ground_scan_speed_m_s=4,release_times_s=releases,
        six_simultaneous_takeoffs=True,regional_field_tiling_complete=True,
        nominal_collision_check=plan['validation'],station_count=sum(len(vehicle['stations']) for vehicle in plan['vehicles']),
        nominal_duration_s=plan['duration_s'],deadline_met=plan['within_time_budget'],
        regions=plan['regional_scan_summary'],hardware_verified=False)
    (ROOT/'validation/regional_scan_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
