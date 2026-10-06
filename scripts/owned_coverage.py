from __future__ import annotations
import math
from shapely.geometry import Point, box, mapping, shape
from shapely.ops import nearest_points, unary_union


def interior_center(target, safe, width, clearance):
    min_x,min_y,max_x,max_y = target.bounds
    radius = width/2
    feasible = safe.intersection(box(max_x-radius-1e-7,max_y-radius-1e-7,
        min_x+radius+1e-7,min_y+radius+1e-7))
    if feasible.is_empty:
        return None
    preferred = feasible.intersection(safe.buffer(-clearance,join_style=2))
    if preferred.is_empty:
        low,high = 0,clearance
        preferred = feasible
        for _ in range(12):
            middle = (low+high)/2
            candidate = feasible.intersection(safe.buffer(-middle,join_style=2))
            if candidate.is_empty:
                high = middle
            else:
                low,preferred = middle,candidate
    center = nearest_points(preferred,target.centroid)[0]
    return [center.x,center.y]


def sweep_legs(target, frame, overlap):
    if target.is_empty or target.area<.01:
        return []
    min_x,min_y,max_x,max_y = target.bounds
    frame_width,frame_height = frame
    radius_x,radius_y = frame_width/2,frame_height/2
    count = max(1,math.ceil(max(0,max_y-min_y-frame_height)/(frame_height*(1-overlap))-1e-10)+1)
    rows = [(min_y+max_y)/2] if count==1 else [min_y+radius_y+index*(max_y-min_y-frame_height)/(count-1) for index in range(count)]
    legs = []
    for row,north in enumerate(rows):
        strip = target.intersection(box(min_x-1,north-radius_y,max_x+1,north+radius_y))
        pieces = [strip] if strip.geom_type=='Polygon' else [part for part in getattr(strip,'geoms',[]) if part.geom_type=='Polygon']
        intervals = sorted((part.bounds[0],part.bounds[2]) for part in pieces if part.area>.000001)
        merged = []
        for left,right in intervals:
            if merged and left<=merged[-1][1]+frame_width:
                merged[-1][1] = max(merged[-1][1],right)
            else:
                merged.append([left,right])
        for left,right in (merged if row%2==0 else list(reversed(merged))):
            half_span = max(.02,(right-left-frame_width)/2)
            start,end = (left+right)/2-half_span,(left+right)/2+half_span
            if row%2:
                start,end = end,start
            legs.append(dict(origin=[start,north,0],target=[end,north,0],
                footprint=mapping(box(min(start,end)-radius_x,north-radius_y,max(start,end)+radius_x,north+radius_y))))
    coverage = unary_union([box(min(leg['origin'][0],leg['target'][0])-radius_x,leg['origin'][1]-radius_y,
        max(leg['origin'][0],leg['target'][0])+radius_x,leg['origin'][1]+radius_y) for leg in legs])
    if target.difference(coverage).area>.01:
        raise ValueError('Owned sweep leaves an uncovered target')
    return legs


def route_order(cells, home):
    if len(cells)<3:
        return cells
    remaining = list(cells)
    ordered = []
    current = home
    while remaining:
        next_cell = min(remaining,key=lambda item:math.dist(current,item['center']))
        remaining.remove(next_cell)
        ordered.append(next_cell)
        current = next_cell['center']
    improved = True
    while improved:
        improved = False
        for first_index in range(1,len(ordered)-1):
            for second_index in range(first_index+1,len(ordered)):
                previous = home if first_index==0 else ordered[first_index-1]['center']
                first = ordered[first_index]['center']
                second = ordered[second_index]['center']
                following = ordered[second_index+1]['center'] if second_index+1<len(ordered) else None
                old_length = math.dist(previous,first)
                new_length = math.dist(previous,second)
                if following is not None:
                    old_length += math.dist(second,following)
                    new_length += math.dist(first,following)
                if new_length+1e-6<old_length:
                    ordered[first_index:second_index+1] = reversed(ordered[first_index:second_index+1])
                    improved = True
    return ordered


def owned_layout(part, safe, configuration, home, clearance):
    width = configuration['station_footprint_m']
    min_x,min_y,max_x,max_y = part.bounds
    columns,rows = max(1,math.ceil((max_x-min_x)/width)),max(1,math.ceil((max_y-min_y)/width))
    step_x,step_y = (max_x-min_x)/columns,(max_y-min_y)/rows
    cells = []
    missing = []
    for column in range(columns):
        for row in range(rows):
            target = part.intersection(box(min_x+column*step_x,min_y+row*step_y,
                min_x+(column+1)*step_x,min_y+(row+1)*step_y))
            if target.area<.01:
                continue
            center = interior_center(target,safe,width,clearance)
            if center is None:
                center = nearest_points(safe,target.centroid)[0]
                center = [center.x,center.y]
                reach = box(center[0]-width/2,center[1]-width/2,center[0]+width/2,center[1]+width/2)
                missing.append(target.difference(reach))
                target = target.intersection(reach)
            if target.area>=.01:
                cells.append(dict(center=center,target=target))
    improved = True
    while improved:
        improved = False
        for first in range(len(cells)):
            for second in range(first+1,len(cells)):
                target = cells[first]['target'].union(cells[second]['target'])
                if target.bounds[2]-target.bounds[0]>width or target.bounds[3]-target.bounds[1]>width:
                    continue
                center = interior_center(target,safe,width,clearance)
                if center is not None:
                    cells[first] = dict(center=center,target=target)
                    cells.pop(second)
                    improved = True
                    break
            if improved:
                break
    remaining = unary_union(missing)
    for cell in cells:
        east,north = cell['center']
        rescued = remaining.intersection(box(east-width/2,north-width/2,east+width/2,north+width/2))
        cell['target'] = cell['target'].union(rescued)
        remaining = remaining.difference(rescued)
    for index in reversed(range(len(cells))):
        if len(cells)<=1:
            break
        others = [cell for other,cell in enumerate(cells) if other!=index]
        reaches = [box(cell['center'][0]-width/2,cell['center'][1]-width/2,
            cell['center'][0]+width/2,cell['center'][1]+width/2) for cell in others]
        target = cells[index]['target']
        if target.difference(unary_union(reaches)).area<.01:
            for cell,reach in zip(others,reaches):
                extra = target.intersection(reach)
                cell['target'] = cell['target'].union(extra)
                target = target.difference(extra)
            cells.pop(index)
    for cell in cells:
        cell['legs'] = sweep_legs(cell['target'],configuration['instantaneous_frame_m'],configuration['minimum_overlap_ratio'])
    return route_order(cells,home),remaining.area


def balanced_partition(region, configuration, homes, margin, clearance):
    min_x,min_y,max_x,max_y = region.bounds
    candidates = []
    for horizontal in (True,False):
        low,high = (min_x,max_x) if horizontal else (min_y,max_y)
        for fraction_index in range(8,93):
            fraction = fraction_index/100
            split = low+(high-low)*fraction
            band = box(split-10,-2000,split+10,2000) if horizontal else box(-2000,split-10,2000,split+10)
            pair = []
            for index in range(2):
                clipping = (box(-2000,-2000,split,2000) if index==0 else box(split,-2000,2000,2000)) if horizontal else (box(-2000,-2000,2000,split) if index==0 else box(-2000,split,2000,2000))
                part = region.intersection(clipping)
                flight = part.difference(band)
                safe = flight.buffer(-margin,join_style=2)
                if safe.is_empty or part.is_empty:
                    break
                cells,missing = owned_layout(part,safe,configuration,homes[index],clearance)
                if not cells:
                    break
                scan_s = sum(math.dist(leg['origin'],leg['target'])/configuration['ground_scan_speed_m_s'] for cell in cells for leg in cell['legs'])
                route_s = sum(math.dist(first,second)/6 for first,second in zip([homes[index]]+[cell['center'] for cell in cells[:-1]],[cell['center'] for cell in cells]))
                edge = sum(max(0,clearance-safe.boundary.distance(Point(cell['center']))) for cell in cells)
                pair.append(dict(part=part,flight=flight,safe=safe,cells=cells,missing=missing,
                    predicted_s=scan_s+route_s+len(cells)*2,edge_penalty=edge))
            if len(pair)==2:
                predicted_times = [item['predicted_s'] for item in pair]
                balance_penalty = abs(predicted_times[0]-predicted_times[1])
                score = (max(predicted_times)+8.0*balance_penalty+.1*sum(item['edge_penalty'] for item in pair)+.02*sum(item['missing'] for item in pair),
                    sum(len(item['cells']) for item in pair))
                candidates.append((score,horizontal,split,band,pair))
    if not candidates:
        raise ValueError('No feasible straight balanced partition')
    return min(candidates,key=lambda item:item[0])[1:]


def rescue_coverage(vehicles, search):
    assigned = unary_union([shape(task['geometry']) for vehicle in vehicles for task in vehicle['station_tasks']])
    remaining = search.difference(assigned)
    for vehicle in vehicles:
        config = vehicle['scan_configuration']
        radius = config['station_footprint_m']/2
        for station,task in zip(vehicle['stations'],vehicle['station_tasks']):
            east,north = station
            extra = remaining.intersection(box(east-radius,north-radius,east+radius,north+radius))
            if extra.area>.000001:
                target = shape(task['geometry']).union(extra)
                task.update(geometry=mapping(target),legs=sweep_legs(target,config['instantaneous_frame_m'],config['minimum_overlap_ratio']))
                remaining = remaining.difference(extra)
    for _ in range(100):
        if remaining.area<.01:
            break
        candidates = []
        components = [remaining] if remaining.geom_type=='Polygon' else list(remaining.geoms)
        for vehicle in vehicles:
            config = vehicle['scan_configuration']
            radius = config['station_footprint_m']/2
            for component in components:
                point = nearest_points(vehicle['_safe'],component.representative_point())[0]
                target = remaining.intersection(box(point.x-radius,point.y-radius,point.x+radius,point.y+radius))
                candidates.append((target.area,vehicle,[point.x,point.y],target))
        area,vehicle,station,target = max(candidates,key=lambda item:item[0])
        if area<.01:
            break
        config = vehicle['scan_configuration']
        vehicle['stations'].append(station)
        vehicle['station_tasks'].append(dict(geometry=mapping(target),legs=sweep_legs(target,config['instantaneous_frame_m'],config['minimum_overlap_ratio'])))
        remaining = remaining.difference(target)
    if remaining.area>=.01:
        raise ValueError('Legal search area cannot be covered without changing reach or work boundaries: '+str(remaining.area))
