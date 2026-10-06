import argparse
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from fleet_core import ROOT, build_plan


def text(element, path, value):
    target = element.find(path)
    if target is None:
        raise ValueError(f'missing SDF element: {path}')
    target.text = str(value)


def build_model(source, destination, vehicle_id, fdm_port, pose):
    tree = ET.parse(source)
    root = tree.getroot()
    model = root.find('model')
    if model is None:
        raise ValueError(f'missing model in {source}')
    model.set('name', vehicle_id)
    text(model, 'plugin/fdm_port_in', fdm_port)
    pose_element = model.find('pose')
    if pose_element is None:
        pose_element = ET.Element('pose')
        model.insert(0, pose_element)
    pose_element.text = f'{pose[0]} {pose[1]} {pose[2]} 0 0 0'
    camera_link = ET.Element('link', {'name': 'sim_camera'})
    camera_pose = ET.SubElement(camera_link, 'pose')
    camera_pose.text = '0 0 -0.16 0 1.5707963 0'
    sensor = ET.SubElement(camera_link, 'sensor', {'name': f'{vehicle_id}_rgb', 'type': 'camera'})
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'visualize').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '10'
    ET.SubElement(sensor, 'topic').text = f'/uav/{vehicle_id}/gimbal/image_raw'
    camera = ET.SubElement(sensor, 'camera')
    ET.SubElement(camera, 'horizontal_fov').text = '1.073377'
    image = ET.SubElement(camera, 'image')
    ET.SubElement(image, 'width').text = '1280'
    ET.SubElement(image, 'height').text = '720'
    ET.SubElement(image, 'format').text = 'R8G8B8'
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '0.1'
    ET.SubElement(clip, 'far').text = '500'
    model.append(camera_link)
    joint = ET.Element('joint', {'name': 'sim_camera_fixed', 'type': 'fixed'})
    ET.SubElement(joint, 'parent').text = 'iris_with_standoffs::base_link'
    ET.SubElement(joint, 'child').text = 'sim_camera'
    model.append(joint)
    tree.write(destination, encoding='utf-8', xml_declaration=True)
    (destination.parent / 'model.config').write_text(
        f'''<?xml version="1.0"?>\n<model><name>{vehicle_id} iris with gimbal</name><version>2.0</version><sdf version="1.9">model.sdf</sdf></model>\n''',
        encoding='utf-8',
    )


def world_xml(vehicle_data):
    includes = []
    for vehicle in vehicle_data:
        includes.append(
            f'<include><uri>model://{vehicle["id"]}_iris_with_gimbal</uri>'
            f'<name>{vehicle["id"]}</name></include>'
        )
    satellite = (ROOT / 'web' / 'assets' / 'satellite.jpg').resolve()
    ground_material = (
        f'<ambient>0.23 0.31 0.22 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse>'
        f'<pbr><metal><albedo_map>file://{satellite}</albedo_map></metal></pbr>'
    )
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <world name="zhixin_subject1_ardupilot_fleet">
    <physics name="1ms" type="ignore"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <scene><ambient>0.8 0.8 0.8 1</ambient><background>0.55 0.68 0.82 1</background></scene>
    <light name="sun" type="directional"><pose>0 0 500 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.3 0.2 -1</direction></light>
    <model name="ground"><static>true</static><link name="link"><collision name="collision"><geometry><box><size>1600 1600 0.2</size></box></geometry></collision><visual name="visual"><geometry><box><size>1600 1600 0.2</size></box></geometry><material>{ground_material}</material></visual></link><pose>200 70 -0.1 0 0 0</pose></model>
    {''.join(includes)}
  </world>
</sdf>'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', default='/home/zyc/ardupilot_gazebo/models/iris_with_gimbal/model.sdf')
    parser.add_argument('--output', default=str(ROOT / 'runtime' / 'ardupilot_fleet'))
    parser.add_argument('--base-port', type=int, default=9002)
    parser.add_argument('--port-step', type=int, default=10)
    parser.add_argument('--plan', default=str(ROOT / 'missions' / 'fleet_plan.json'))
    args = parser.parse_args()

    output = Path(args.output)
    models = output / 'models'
    if output.exists():
        shutil.rmtree(output)
    models.mkdir(parents=True)
    source = Path(args.source)
    plan_path = Path(args.plan)
    plan = json.loads(plan_path.read_text(encoding='utf-8')) if plan_path.exists() else build_plan()
    vehicle_data = []
    for index, vehicle in enumerate(plan['vehicles']):
        vehicle_id = vehicle['id']
        model_directory = models / f'{vehicle_id}_iris_with_gimbal'
        model_directory.mkdir()
        pose = (*vehicle['home'], 0.25)
        fdm_port = args.base_port + index * args.port_step
        build_model(source, model_directory / 'model.sdf', vehicle_id, fdm_port, pose)
        vehicle_data.append(dict(id=vehicle_id, pose=pose, fdm_port=fdm_port))
    world = output / 'zhixin_subject1_ardupilot_fleet.sdf'
    world.write_text(world_xml(vehicle_data), encoding='utf-8')
    print(world)
    for vehicle in vehicle_data:
        print(f'{vehicle["id"]} fdm_port={vehicle["fdm_port"]} pose={vehicle["pose"]}')


if __name__ == '__main__':
    main()
