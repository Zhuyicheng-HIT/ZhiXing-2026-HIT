import json
import math
import subprocess
import time
from urllib.request import Request,urlopen
from fleet_core import ROOT


def request(data=None):
    endpoint = 'control' if data else 'state'
    payload = json.dumps(data).encode() if data else None
    with urlopen(Request('http://127.0.0.1:8766/api/'+endpoint,data=payload,headers={'Content-Type':'application/json'}),timeout=4) as response:
        return json.load(response)


def main():
    state = request(dict(action='pause'))
    time.sleep(2)
    result = subprocess.run(['gz','topic','-e','-n','1','--json-output','-t','/world/zhixin_fleet_kinematic/pose/info'],capture_output=True,text=True,timeout=10,check=True)
    poses = json.loads(result.stdout)['pose']
    by_name = {pose['name']:pose for pose in poses}
    errors = []
    for vehicle in state['vehicles']:
        actual = by_name[vehicle['id']]['position']
        position = [actual.get(axis,0) for axis in ['x','y','z']]
        expected = vehicle['position'][:]
        expected[2] += .25
        error = math.dist(position,expected)
        errors.append(dict(vehicle_id=vehicle['id'],gazebo_position=position,expected=expected,error_m=error))
    print(json.dumps(errors,indent=2))
    assert all(error['error_m']<.01 for error in errors)
    report = dict(mode='gazebo_set_pose_visualization',stamp_s=state['stamp'],all_six_pose_updates_verified=True,vehicles=errors,
                  sitl_flight_or_dynamics_verified=False)
    (ROOT/'validation/gazebo_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
