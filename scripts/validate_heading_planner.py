from __future__ import annotations
import json
import math
from types import SimpleNamespace
from fleet_core import ROOT, build_plan
from gimbal_geometry import plan_level_hover_heading, attitude_ned_frd_to_mission_flu, body_heading
from flight_coordinates import FlightCoordinates
from geodesy import LocalCartesian
from sitl_server import SitlFleet


def main():
    configuration = json.loads((ROOT/'config/gimbal_geometry.json').read_text(encoding='utf-8'))
    coordinates = FlightCoordinates(LocalCartesian(33.86,113.70,71),0)
    coordinates.set_ekf_origin(33.86,113.70,71)
    for heading,expected in [(0,math.pi/2),(math.pi/2,0),(math.pi,-math.pi/2),(-math.pi/2,math.pi)]:
        error = (coordinates.mission_heading_to_ned(heading)-expected+math.pi)%(2*math.pi)-math.pi
        assert abs(error)<1e-10
    for invalid in [True,float('nan'),float('inf')]:
        try:
            coordinates.mission_heading_to_ned(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError('Invalid heading accepted')
    coordinates.set_ekf_origin(33.869,113.714,71)
    for heading in [0,0.8,-1.2,math.pi/2]:
        yaw_ned = coordinates.mission_heading_to_ned(heading)
        quaternion = attitude_ned_frd_to_mission_flu(0,0,yaw_ned,coordinates.mission_frame,coordinates.ekf_frame)
        error = (body_heading(quaternion)-heading+math.pi)%(2*math.pi)-math.pi
        assert abs(error)<1e-7
    messages = []
    sender = SimpleNamespace(links=[SimpleNamespace(mav=SimpleNamespace(set_position_target_local_ned_send=lambda *arguments:messages.append(arguments)))],coordinates=[coordinates])
    SitlFleet.setpoint(sender,0,[1,2,40])
    SitlFleet.setpoint(sender,0,[1,2,40],yaw_mission_rad=0.8)
    assert messages[0][4]==3576 and messages[1][4]==2552
    assert messages[0][14]==0 and abs(messages[1][14]-coordinates.mission_heading_to_ned(0.8))<1e-10
    assert not messages[1][4]&1024 and messages[1][4]&2048
    plan = build_plan()
    count = 0
    turns = 0
    maximum_change = 0.0
    for vehicle in plan['vehicles']:
        heading = math.pi/2
        for segment in vehicle['segments']:
            if segment['state']!='SCAN':
                continue
            result = plan_level_hover_heading(segment['target'],segment['destination'],heading,configuration)
            assert result['nominal_pointing']['nominal_reachable']
            assert -130<=result['nominal_pointing']['azimuth_deg']<=130
            assert not result['hardware_authorized']
            change = abs(math.degrees(result['heading_change_rad']))
            turns += change>1e-8
            maximum_change = max(maximum_change,change)
            heading = result['heading_mission_rad']
            count += 1
    report = dict(planned_observations=count,nominally_reachable_after_heading_plan=count,
        mission_enu_to_ned_heading_cardinals_verified=True,invalid_headings_rejected=True,
        distinct_ekf_origin_heading_roundtrip_verified=True,mavlink_yaw_mask_and_field_verified=True,
        heading_changes=turns,maximum_heading_change_deg=maximum_change,
        assumption='LEVEL_HOVER_NOMINAL_MOUNT',hardware_verified=False)
    (ROOT/'validation/heading_planner_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
