from __future__ import annotations
import argparse
import json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
COLORS = ['#2498ff','#86cbff','#ff9400','#ffe464','#14db65','#b4f252']


def pixel(point, transform):
    east,north = point[:2]
    first,second = transform
    determinant = first[0]*second[1]-first[1]*second[0]
    return ((second[1]*(east-first[2])-first[1]*(north-second[2]))/determinant,
        (-second[0]*(east-first[2])+first[0]*(north-second[2]))/determinant)


def polygons(geometry):
    if geometry['type']=='Polygon':
        return [geometry['coordinates']]
    if geometry['type']=='MultiPolygon':
        return geometry['coordinates']
    return []


def fill_geometry(image, geometry, transform, color):
    mask = Image.new('L',image.size,0)
    drawing = ImageDraw.Draw(mask)
    for rings in polygons(geometry):
        if not rings:
            continue
        drawing.polygon([pixel(point,transform) for point in rings[0]],fill=color[3])
        for ring in rings[1:]:
            drawing.polygon([pixel(point,transform) for point in ring],fill=0)
    image.paste(Image.new('RGB',image.size,color[:3]),(0,0),mask)


def panel(plan, satellite, font):
    image = satellite.copy().convert('RGB')
    transform = plan['satellite']['pixel_to_enu']
    fill_geometry(image,plan['forest'],transform,(70,80,70,100))
    for partition in plan['partitions']:
        if partition.get('work_isolation_buffer'):
            fill_geometry(image,partition['work_isolation_buffer'],transform,(255,234,120,110))
        fill_geometry(image,partition['buffer'],transform,(255,65,95,170))
    drawing = ImageDraw.Draw(image)
    for index,vehicle in enumerate(plan['vehicles']):
        for segment in vehicle['segments']:
            if segment['state'] in ('INGRESS','EGRESS','REPOSITION'):
                endpoints = [pixel(segment['origin'],transform),pixel(segment['destination'],transform)]
                drawing.line(endpoints,fill='white',width=5)
                drawing.line(endpoints,fill=COLORS[index],width=3)
        for station in vehicle['stations']:
            east,north = pixel(station,transform)
            drawing.ellipse((east-3,north-3,east+3,north+3),fill=COLORS[index],outline='black')
    drawing.text((20,20),'红：全高度禁区  黄：保留的20m工作间隔',font=font,fill='white',stroke_width=2,stroke_fill='black')
    drawing.text((20,58),'彩线：名义进出场  点：112个固定扫描站位',font=font,fill='white',stroke_width=2,stroke_fill='black')
    return image


def main():
    parser = argparse.ArgumentParser(description='Render the unapproved isolation corridor review.')
    parser.add_argument('--font', type=Path, help='Chinese-capable TrueType or OpenType font file')
    arguments = parser.parse_args()
    baseline = json.loads((ROOT/'missions/fleet_plan.json').read_text(encoding='utf-8'))
    candidate = json.loads((ROOT/'missions/fleet_plan_isolation_corridor_candidate.json').read_text(encoding='utf-8'))
    assert baseline['map_frame']==candidate['map_frame']
    font_candidates = [arguments.font] if arguments.font else [Path('C:/Windows/Fonts/msyh.ttc'),Path('/mnt/c/Windows/Fonts/msyh.ttc'),Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')]
    font_path = next((path for path in font_candidates if path.is_file()),None)
    if font_path is None:
        parser.error('No Chinese-capable font found; provide --font /absolute/path/to/font.ttc')
    font = ImageFont.truetype(str(font_path),30)
    title_font = ImageFont.truetype(str(font_path),36)
    satellite = Image.open(ROOT/'web/assets/satellite.jpg')
    left,right = panel(baseline,satellite,font),panel(candidate,satellite,font)
    width,height = satellite.size
    review = Image.new('RGB',(width*2,height+150),'#132136')
    review.paste(left,(0,70))
    review.paste(right,(width,70))
    drawing = ImageDraw.Draw(review)
    drawing.text((20,10),'当前完整带：5段穿越，默认拒绝启动',font=title_font,fill='#ffabb8')
    drawing.text((width+20,10),'候选：边界留10m通道，待确认',font=title_font,fill='#fff2ac')
    drawing.text((20,height+85),'方案评审图，不是实时飞行；候选未采用、未原生复验。树林过境权限及实机制动包络仍待确认。',font=font,fill='white')
    destination = ROOT/'web/assets/isolation_corridor_review.png'
    review.save(destination)
    print(str(destination))


if __name__=='__main__':
    main()
