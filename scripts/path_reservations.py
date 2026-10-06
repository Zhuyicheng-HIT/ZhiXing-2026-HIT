from __future__ import annotations
import math
from scan_protocol import finite_number


def segment_distance(first_start, first_end, second_start, second_end):
    points = [first_start,first_end,second_start,second_end]
    for point in points:
        if len(point)!=3:
            raise ValueError('Reservation positions require three ENU coordinates')
        for coordinate in point:
            finite_number(coordinate,'reservation coordinate')
    first_delta = [end-start for start,end in zip(first_start,first_end)]
    second_delta = [end-start for start,end in zip(second_start,second_end)]
    offset = [first-second for first,second in zip(first_start,second_start)]

    def dot(first, second):
        return sum(left*right for left,right in zip(first,second))

    def clamp(value):
        return max(0.0,min(1.0,value))

    def cross_product(first, second):
        return [first[1]*second[2]-first[2]*second[1],
            first[2]*second[0]-first[0]*second[2],
            first[0]*second[1]-first[1]*second[0]]

    first_length = dot(first_delta,first_delta)
    second_length = dot(second_delta,second_delta)
    cross = dot(first_delta,second_delta)
    first_offset = dot(first_delta,offset)
    second_offset = dot(second_delta,offset)
    candidates = []
    for first_fraction in (0.0,1.0):
        second_fraction = clamp((second_offset+first_fraction*cross)/second_length) if second_length else 0.0
        candidates.append((first_fraction,second_fraction))
    for second_fraction in (0.0,1.0):
        first_fraction = clamp((second_fraction*cross-first_offset)/first_length) if first_length else 0.0
        candidates.append((first_fraction,second_fraction))
    normal = cross_product(first_delta,second_delta)
    determinant = dot(normal,normal)
    if determinant>0:
        first_fraction = dot(cross_product(second_delta,offset),normal)/determinant
        second_fraction = dot(cross_product(first_delta,offset),normal)/determinant
        if 0<=first_fraction<=1 and 0<=second_fraction<=1:
            candidates.append((first_fraction,second_fraction))
    return min(math.sqrt(sum((offset[axis]+first_fraction*first_delta[axis]-second_fraction*second_delta[axis])**2
        for axis in range(3))) for first_fraction,second_fraction in candidates)
