from __future__ import annotations
import json
import argparse
import math
import random
import shutil
import subprocess
import re
import zipfile
import xml.etree.ElementTree as ElementTree
from flight_coordinates import FlightCoordinates
from fleet_core import ROOT, site_geometry
from geodesy import LocalCartesian, geodetic_to_ecef, ecef_to_geodetic, simulation_frame


def main(boundary_docx=None):
    datum,points,_,_,_ = site_geometry()
    frame = simulation_frame(datum)
    randomizer = random.Random(2026)
    local = [[0,0,0],[653,-436,110],[-38,12,0]]+[[randomizer.uniform(-1000,1000),randomizer.uniform(-1000,1000),randomizer.uniform(-10,120)] for _ in range(100)]
    error = max(math.dist(point,frame.forward(*frame.reverse(point))) for point in local)
    assert error<1e-6
    global_cases = [[0,0,0],[90,45,71],[-90,-90,120],[-34,179.9,1200],[33.86,113.7,71]]
    ecef_error = max(math.dist(geodetic_to_ecef(*point),geodetic_to_ecef(*ecef_to_geodetic(geodetic_to_ecef(*point)))) for point in global_cases)
    assert ecef_error<1e-6
    transforms = []
    for home in [[-38,12,0],[600,-400,0],[180,550,0]]:
        latitude,longitude,ellipsoid_height = frame.reverse(home)
        converter = FlightCoordinates(frame,31.5)
        converter.set_ekf_origin(latitude,longitude,ellipsoid_height-31.5)
        transforms.append(converter)
    target = [200,300,40]
    converted = [converter.ned_to_mission(converter.mission_to_ned(target)) for converter in transforms]
    shared_error = max(math.dist(point,target) for point in converted)
    assert shared_error<1e-6
    assert math.dist(transforms[0].mission_to_ned(target),transforms[1].mission_to_ned(target))>500
    try:
        FlightCoordinates(frame,0).ned_to_mission([0,0,0])
        raise AssertionError('没有原点不能转换')
    except ValueError:
        pass
    report = dict(roundtrip_enu_max_error_m=error,roundtrip_ecef_max_error_m=ecef_error,
        independent_ekf_origins_shared_target_error_m=shared_error,missing_origin_rejected=True,
        point_2_origin_distance_m=math.dist(points[2],[0,0]),map_frame=frame.descriptor(),height_hardware_verified=False)
    if boundary_docx is not None:
        with zipfile.ZipFile(boundary_docx) as document:
            tree = ElementTree.fromstring(document.read('word/document.xml'))
        text = ''.join(node.text or '' for node in tree.iter('{http://schemas.openxmlformats.org/wordprocessingml/2006/main}t'))
        values = [[int(degrees),int(minutes),float(seconds)] for degrees,minutes,seconds in re.findall(r'(113|33)°(\d+)′(\d+(?:\.\d+)?)″',text.replace(' ',''))]
        canonical = json.loads((ROOT/'config/boundary_wgs84.json').read_text(encoding='utf-8'))
        expected = [item for index in range(1,26) for item in canonical['boundary_points_dms'][str(index)]]
        expected += [item for point in canonical['forest_points_dms'] for item in point]
        assert values==expected, '坐标转录与原始DOCX不同'
        report.update(source_docx_coordinates_verified=True,source_coordinate_pairs=len(values)//2)
    else:
        report['source_docx_coordinates_verified'] = False
    if shutil.which('CartConvert'):
        geographic = [frame.reverse(point) for point in local]
        result = subprocess.run(['CartConvert','-l',*(str(value) for value in frame.origin),'-p','9'],
            input='\n'.join(' '.join(str(value) for value in point) for point in geographic)+'\n',text=True,capture_output=True,check=True)
        reference = [[float(value) for value in row.split()] for row in result.stdout.splitlines()]
        reference_error = max(math.dist(expected,actual) for expected,actual in zip(local,reference))
        assert len(reference)==len(local) and reference_error<1e-6
        report.update(independent_reference='GeographicLib CartConvert',reference_max_error_m=reference_error,reference_verified=True)
    else:
        report.update(reference_verified=False,reference_note='CartConvert未安装；仅往返和多原点自洽验证')
    (ROOT/'validation/coordinates_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=True,indent=2))


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--boundary-docx')
    main(parser.parse_args().boundary_docx)
