import argparse
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
    <model name="ground"><static>true</static><link name="link"><collision name="collision"><geometry><box><size>1600 1600 0.2</size></box></geometry></collision><visual name="visual"><geometry><box><size>1600 1600 0.2</size></box></geometry><material><ambient>0.24 0.31 0.22 1</ambient><diffuse>0.24 0.31 0.22 1</diffuse></material></visual></link><pose>200 70 -0.1 0 0 0</pose></model>
    {''.join(includes)}
  </world>
</sdf>'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', default='/home/zyc/ardupilot_gazebo/models/iris_with_gimbal/model.sdf')
    parser.add_argument('--output', default=str(ROOT / 'runtime' / 'ardupilot_fleet'))
    parser.add_argument('--base-port', type=int, default=9002)
    args = parser.parse_args()

    output = Path(args.output)
    models = output / 'models'
    if output.exists():
        shutil.rmtree(output)
    models.mkdir(parents=True)
    source = Path(args.source)
    plan = build_plan()
    vehicle_data = []
    for index, vehicle in enumerate(plan['vehicles']):
        vehicle_id = vehicle['id']
        model_directory = models / f'{vehicle_id}_iris_with_gimbal'
        model_directory.mkdir()
        pose = (*vehicle['home'], 0.25)
        build_model(source, model_directory / 'model.sdf', vehicle_id, args.base_port + index, pose)
        vehicle_data.append(dict(id=vehicle_id, pose=pose, fdm_port=args.base_port + index))
    world = output / 'zhixin_subject1_ardupilot_fleet.sdf'
    world.write_text(world_xml(vehicle_data), encoding='utf-8')
    print(world)
    for vehicle in vehicle_data:
        print(f'{vehicle["id"]} fdm_port={vehicle["fdm_port"]} pose={vehicle["pose"]}')


if __name__ == '__main__':
    main()
