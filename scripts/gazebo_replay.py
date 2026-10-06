import argparse
import json
import math
import subprocess
import time
from urllib.request import urlopen


def pose_message(name,position,pitch=0,yaw=0):
    cosine_pitch,sine_pitch = math.cos(pitch/2),math.sin(pitch/2)
    cosine_yaw,sine_yaw = math.cos(yaw/2),math.sin(yaw/2)
    orientation = [-sine_pitch*sine_yaw,sine_pitch*cosine_yaw,cosine_pitch*sine_yaw,cosine_pitch*cosine_yaw]
    return 'pose {{name: "{}" position {{x:{} y:{} z:{}}} orientation {{x:{} y:{} z:{} w:{}}}}}'.format(name,*position,*orientation)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--url',default='http://127.0.0.1:8765')
    parser.add_argument('--hz',type=float,default=2)
    arguments = parser.parse_args()
    print('Gazebo运动学显示桥：set_pose不是飞控控制，也不模拟动力学',flush=True)
    while True:
        try:
            with urlopen(arguments.url+'/api/state',timeout=3) as response:
                state = json.load(response)
            messages = []
            for vehicle in state['vehicles']:
                east,north,height = vehicle['position']
                messages.append(pose_message(vehicle['id'],[east,north,height+.25]))
                pitch = -math.radians(vehicle['gimbal']['elevation_deg'])
                yaw = math.radians(vehicle['gimbal']['azimuth_enu_deg'])
                messages.append(pose_message(vehicle['id']+'_camera',[east,north,height+.14],pitch,yaw))
            result = subprocess.run(['gz','service','-s','/world/zhixin_fleet_kinematic/set_pose_vector','--reqtype','gz.msgs.Pose_V','--reptype','gz.msgs.Boolean','--timeout','1500','--req',' '.join(messages)],capture_output=True,text=True,timeout=4)
            if result.returncode or 'true' not in result.stdout:
                print('Gazebo未确认姿态更新: '+result.stderr+result.stdout,flush=True)
        except (OSError,ValueError,subprocess.TimeoutExpired) as error:
            print('显示桥连接等待: '+str(error),flush=True)
        time.sleep(1/max(.1,arguments.hz))


if __name__=='__main__':
    main()
