import json
from fleet_core import ROOT, build_plan, site_geometry


def solve(matrix, values):
    rows = [list(row)+[value] for row,value in zip(matrix,values)]
    for column in range(3):
        pivot = max(range(column,3),key=lambda index: abs(rows[index][column]))
        rows[column],rows[pivot] = rows[pivot],rows[column]
        scale = rows[column][column]
        rows[column] = [value/scale for value in rows[column]]
        for index in range(3):
            if index!=column:
                scale = rows[index][column]
                rows[index] = [value-scale*other for value,other in zip(rows[index],rows[column])]
    return [row[3] for row in rows]


def satellite_transform():
    pixel = [[304,546],[296,488],[241,490],[250,550],[77,565],[48,865],[444,897],[1028,932],[1087,210],[770,27],[523,51],[490,531]]
    _,points,_,_,_ = site_geometry()
    enu = [points[index] for index in [2,3,4,5,6,8,9,16,19,22,23,25]]
    design = [[*point,1] for point in pixel]
    normal = [[sum(row[first]*row[second] for row in design) for second in range(3)] for first in range(3)]
    transform = [solve(normal,[sum(row[column]*point[axis] for row,point in zip(design,enu)) for column in range(3)]) for axis in range(2)]
    error = sum(sum((sum(coefficient*value for coefficient,value in zip(transform[axis],row))-point[axis])**2 for axis in range(2)) for row,point in zip(design,enu)) / len(enu)
    return dict(pixel_to_enu=transform,rmse_m=error**.5,source='用户提供卫星截图；12点人工配准；非测绘正射图',width=1228,height=1040)


if __name__=='__main__':
    transform = satellite_transform()
    (ROOT/'config/satellite.json').write_text(json.dumps(transform,ensure_ascii=False,indent=2),encoding='utf-8')
    plan = build_plan()
    (ROOT/'missions/fleet_plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2),encoding='utf-8')
    print('卫星配准平面RMSE: {:.2f}m'.format(transform['rmse_m']))
