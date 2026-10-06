from __future__ import annotations
import math
from shapely.geometry import Point, box
from shapely.ops import nearest_points, unary_union


def interior_stations(part, safe, footprint, stations, preferred_clearance):
    radius = footprint/2
    result = [list(point) for point in stations]
    for indices in (range(len(result)),reversed(range(len(result)))):
        for index in indices:
            others = unary_union([box(east-radius,north-radius,east+radius,north+radius)
                for other,(east,north) in enumerate(result) if other!=index])
            required = part.difference(others)
            if required.is_empty or required.area<.01:
                continue
            min_x,min_y,max_x,max_y = required.bounds
            lower_x,upper_x = max_x-radius,min_x+radius
            lower_y,upper_y = max_y-radius,min_y+radius
            if lower_x>=upper_x-1e-7 or lower_y>=upper_y-1e-7:
                continue
            feasible = safe.intersection(box(lower_x,lower_y,upper_x,upper_y))
            if feasible.is_empty:
                continue
            low,high = 0.0,preferred_clearance
            interior = feasible
            for _ in range(16):
                middle = (low+high)/2
                candidate = feasible.intersection(safe.buffer(-middle,join_style=2))
                if candidate.is_empty:
                    high = middle
                else:
                    low,interior = middle,candidate
            previous = Point(result[index])
            point = nearest_points(interior,previous)[0]
            footprint_shape = box(point.x-radius,point.y-radius,point.x+radius,point.y+radius)
            if required.difference(footprint_shape).area<.01 and safe.boundary.distance(point)>safe.boundary.distance(previous)+1e-5:
                result[index] = [point.x,point.y]
    return result


def sparse_stations(part, safe, footprint, home, solver, order, preferred_clearance=0):
    baseline,baseline_uncovered = solver(part,safe,footprint,home)
    radius = footprint/2

    def coverage(stations):
        return unary_union([box(east-radius,north-radius,east+radius,north+radius)
            for east,north in stations])

    def score(stations):
        distance = math.dist(home,stations[0]) if stations else math.inf
        distance += sum(math.dist(first,second) for first,second in zip(stations,stations[1:]))
        boundary_penalty = sum(max(0,preferred_clearance-safe.boundary.distance(Point(point))) for point in stations)
        return len(stations),boundary_penalty,distance

    best = order(interior_stations(part,safe,footprint,baseline,preferred_clearance),safe,home) if preferred_clearance else baseline
    best_uncovered = baseline_uncovered
    min_x,min_y,max_x,max_y = part.bounds
    for phase_x,phase_y in ((0,0),(.5,0),(0,.5),(.5,.5)):
        stations = []
        for column in range(math.ceil((max_x-min_x)/footprint+phase_x)):
            for row in range(math.ceil((max_y-min_y)/footprint+phase_y)):
                candidate = Point(min_x+(column+.5-phase_x)*footprint,
                    min_y+(row+.5-phase_y)*footprint)
                nominal = box(candidate.x-radius,candidate.y-radius,candidate.x+radius,candidate.y+radius)
                if nominal.intersection(part).area<.01:
                    continue
                projected = candidate if safe.covers(candidate) else nearest_points(safe.buffer(-1e-6),candidate)[0]
                station = [projected.x,projected.y]
                if station not in stations:
                    stations.append(station)
        remaining = part.difference(coverage(stations))
        if remaining.area>=.01:
            additions,unused = solver(remaining,safe,footprint,home)
            stations.extend(additions)
        for index in reversed(range(len(stations))):
            others = stations[:index]+stations[index+1:]
            if part.difference(coverage(others)).area<.01:
                stations.pop(index)
        remaining = part.difference(coverage(stations))
        if remaining.area>=.01 or any(safe.distance(Point(station))>1e-7 for station in stations):
            continue
        stations = interior_stations(part,safe,footprint,stations,preferred_clearance) if preferred_clearance else stations
        ordered = order(stations,safe,home)
        if score(ordered)<score(best):
            best,best_uncovered = ordered,remaining.area
    return best,best_uncovered
