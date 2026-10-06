import json
import math
from shapely.geometry import shape, Point, LineString
from fleet_core import ROOT, build_plan, pose_at, validate_separation
from sim_server import FleetSimulation


def main():
    simulation = FleetSimulation()
    plan = simulation.plan
    assert plan['validation']['passed']
    assert plan['coverage']['ratio'] > .99999
    perimeter,launch = shape(plan['perimeter']),shape(plan['launch'])
    all_inside = True
    all_work_inside = True
    altitude_valid = True
    for vehicle in plan['vehicles']:
        safe = shape(vehicle['flight_region']).buffer(.001)
        for segment in vehicle['segments']:
            origin,end = segment['origin'],segment['destination']
            if math.dist(origin[:2],end[:2]) > 1e-9:
                path = LineString([origin[:2],end[:2]])
                all_inside &= perimeter.buffer(.001).covers(path)
                if segment['state']=='NAVIGATE':
                    all_work_inside &= safe.covers(path)
            for point in [origin,end]:
                if not launch.covers(Point(point[:2])):
                    altitude_valid &= 40-.01 <= point[2] <= 120+.01
    assert all_inside and all_work_inside and altitude_valid
    takeoff = [vehicle['segments'][0] for vehicle in plan['vehicles']]
    assert all(segment['start']==0 and segment['end']==2 for segment in takeoff)
    simulation.control(dict(action='start',synthetic=False))
    simulation.speed = 100
    for _ in range(200):
        simulation.advance(1)
        if simulation.pending:
            break
    assert simulation.pending
    stamp = simulation.stamp
    pending = simulation.pending.copy()
    before = simulation.snapshot()['vehicles']
    simulation.control(dict(action='fault',vehicle_id=pending['vehicle_id']))
    simulation.advance(10)
    assert simulation.stamp==stamp
    assert [vehicle['position'] for vehicle in before]==[vehicle['position'] for vehicle in simulation.snapshot()['vehicles']]
    simulation.control(dict(action='recover',vehicle_id=pending['vehicle_id']))
    assert {name:value for name,value in simulation.pending.items() if name!='execution'}=={name:value for name,value in pending.items() if name!='execution'}
    assert simulation.pending['execution']['status']==pending['execution']['status']
    try:
        wrong = dict(pending,vehicle_id='uav_6' if pending['vehicle_id']!='uav_6' else 'uav_1')
        simulation.control(dict(action='result',**wrong,status='FINISHED'))
        raise AssertionError('应拒绝其他飞机回传的观察结果')
    except ValueError:
        pass
    try:
        simulation.control(dict(action='result',command_id=pending['command_id'],mission_epoch='stale',status='FINISHED'))
        raise AssertionError('不应接受过期结果')
    except ValueError:
        pass
    simulation.control(dict(action='result',**pending,status='FINISHED'))
    simulation.control(dict(action='result',**pending,status='FINISHED'))
    assert len(simulation.completed)==1
    simulation.control(dict(action='pause'))
    simulation.control(dict(action='checkpoint'))
    simulation.control(dict(action='reset'))
    simulation.control(dict(action='restore'))
    assert simulation.stamp==stamp and len(simulation.completed)==1
    simulation.control(dict(action='start',synthetic=True))
    while simulation.running:
        simulation.advance(1)
    snapshot = simulation.snapshot()
    assert all(vehicle['state']=='COMPLETE' for vehicle in snapshot['vehicles'])
    assert all(math.dist(vehicle['position'],[*planned['home'],0])<.001 for vehicle,planned in zip(snapshot['vehicles'],plan['vehicles']))
    assert len(simulation.completed)==sum(segment['state']=='SCAN' for vehicle in plan['vehicles'] for segment in vehicle['segments'])
    try:
        build_plan(homes=[[0,0]]*6)
        raise AssertionError('应拒绝无效起降位置')
    except ValueError:
        pass
    report = dict(mode=plan['mode'],six_simultaneous_takeoffs=True,continuous_separation=plan['validation'],
                  perimeter_containment=all_inside,work_region_containment=all_work_inside,altitude_containment=altitude_valid,
                  geometric_rectangle_coverage=plan['coverage']['ratio'],station_count=sum(len(vehicle['stations']) for vehicle in plan['vehicles']),
                  default_duration_s=plan['duration_s'],within_competition_time_budget=plan['within_time_budget'],
                  forest_permission_required=plan['requires_forest_transit_permission'],work_only_isolation_policy=True,
                  competition_airspace_compliant=plan['isolation_airspace']['authorized_policy_respected'] and not plan['requires_forest_transit_permission'],fault_checkpoint_resume=True,
                  stale_result_rejected=True,result_idempotent=True,checkpoint_restore=True,all_returned_home=True,
                  total_completed_observations=len(simulation.completed),hardware_or_sitl_verified=False)
    target = ROOT/'validation/report.json'
    target.parent.mkdir(exist_ok=True)
    target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=True,indent=2))


if __name__=='__main__':
    main()
