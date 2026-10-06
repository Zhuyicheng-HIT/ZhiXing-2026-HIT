import math
from fleet_core import ROOT, build_plan, polygons
from shapely.geometry import shape


def visual(name,pose,geometry,color):
    return f'<visual name="{name}"><pose>{pose}</pose><geometry>{geometry}</geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>'


def box_geometry(east,north,height):
    return f'<box><size>{east} {north} {height}</size></box>'


def model(name,pose,visuals,sensors=''):
    return f'<model name="{name}"><static>true</static><pose>{pose}</pose><link name="body">'+''.join(visuals)+sensors+'</link></model>'


def rgb_sensor(vehicle_id):
    return (
        f'<sensor name="{vehicle_id}_rgb" type="camera">'
        '<always_on>true</always_on><visualize>true</visualize><update_rate>10</update_rate>'
        f'<topic>/uav/{vehicle_id}/gimbal/image_raw</topic>'
        '<camera><horizontal_fov>1.073377</horizontal_fov><image>'
        '<width>1280</width><height>720</height><format>R8G8B8</format>'
        '</image><clip><near>0.1</near><far>500</far></clip></camera>'
        '</sensor>'
    )


def edge(name,start,end,color,width=1):
    east,north = (start[0]+end[0])/2,(start[1]+end[1])/2
    angle = math.atan2(end[1]-start[1],end[0]-start[0])
    return model(name,f'{east} {north} .1 0 0 {angle}',[visual('line','0 0 0 0 0 0',box_geometry(math.dist(start,end),width,.12),color)])


def main():
    plan = build_plan()
    satellite = (ROOT/'web'/'assets'/'satellite.jpg').resolve()
    ground_material = (
        '<ambient>0.23 0.31 0.22 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse>'
        f'<pbr><metal><albedo_map>file://{satellite}</albedo_map></metal></pbr>'
    )
    ground_visual = f'<visual name="ground"><pose>0 0 0 0 0 0</pose><geometry>{box_geometry(1400,1400,.2)}</geometry><material>{ground_material}</material></visual>'
    models = [model('ground','200 70 -.1 0 0 0',[ground_visual])]
    for label,geometry,color in [('perimeter',plan['perimeter'],'1 .85 .1 1'),('forest',plan['forest'],'.5 .1 .1 1'),('launch',plan['launch'],'.8 .8 1 1')]:
        for polygon_index,polygon in enumerate(polygons(shape(geometry))):
            coords = list(polygon.exterior.coords)
            for index,(start,end) in enumerate(zip(coords,coords[1:])):
                models.append(edge(f'{label}_{polygon_index}_{index}',start,end,color))
    for partition in plan['partitions']:
        for polygon_index,polygon in enumerate(polygons(shape(partition['buffer']))):
            coords = list(polygon.exterior.coords)
            for index,(start,end) in enumerate(zip(coords,coords[1:])):
                name = '{}_{}_{}'.format(partition['zone'],polygon_index,index)
                models.append(edge('buffer_'+name,start,end,'1 .2 .4 1',2))
    colors = ['.1 .5 1 1','.4 .7 1 1','1 .5 .1 1','1 .8 .1 1','.2 .9 .3 1','.6 1 .2 1']
    for vehicle,color in zip(plan['vehicles'],colors):
        parts = [visual('fuselage','0 0 .08 0 0 0',box_geometry(.22,.18,.12),color)]
        for index,angle in enumerate([math.pi/4,3*math.pi/4,5*math.pi/4,7*math.pi/4]):
            east,north = .35*math.cos(angle),.35*math.sin(angle)
            parts.append(visual('arm_'+str(index),f'{east/2} {north/2} .06 0 0 {angle}',box_geometry(.35,.025,.025),'0.12 .12 .12 1'))
            parts.append(visual('propeller_'+str(index),f'{east} {north} .16 0 0 0','<cylinder><radius>.2286</radius><length>.008</length></cylinder>','.35 .35 .35 .65'))
        parts.append(visual('gimbal_mount','0 0 -.03 0 0 0',box_geometry(.09,.08,.08),'.2 .2 .2 1'))
        east,north = vehicle['home']
        models.append(model(vehicle['id'],f'{east} {north} .25 0 0 0',parts))
        camera = [visual('camera','0 0 0 0 0 0',box_geometry(.08,.07,.06),'.1 .1 .1 1'),visual('lens','.045 0 0 0 1.5708 0','<cylinder><radius>.022</radius><length>.02</length></cylinder>','.1 .6 .9 1')]
        models.append(model(vehicle['id']+'_camera',f'{east} {north} .14 0 0 0',camera,rgb_sensor(vehicle['id'])))
    world = '''<?xml version="1.0"?><sdf version="1.9"><world name="zhixin_fleet_kinematic">
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
<scene><ambient>.6 .6 .6 1</ambient><background>.5 .65 .8 1</background></scene>
<light name="sun" type="directional"><pose>0 0 100 0 0 0</pose><diffuse>.8 .8 .8 1</diffuse><direction>-.3 .2 -1</direction></light>
'''+''.join(models)+'</world></sdf>'
    destination = ROOT/'worlds/fleet_kinematic.sdf'
    destination.write_text(world,encoding='utf-8')
    print(destination)


if __name__=='__main__':
    main()
