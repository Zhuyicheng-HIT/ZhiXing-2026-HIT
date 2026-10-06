from __future__ import annotations
import math
from shapely.geometry import box
from shapely.ops import unary_union


def regional_scan(profile, zone, footprint, altitude):
    configuration = profile['region_scans'][zone]
    field = configuration['frame_height_m']
    overlap = configuration['minimum_overlap_ratio']
    scale = configuration['station_footprint_scale']
    if any(isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value)
           for value in (field,overlap,scale,altitude,footprint)):
        raise ValueError('Regional scan dimensions must be finite numbers')
    if field<=0 or not 0<=overlap<1 or scale<=0 or altitude<=0:
        raise ValueError('Invalid regional scan dimensions')
    width = footprint*scale
    horizontal_fov = profile['wide_horizontal_fov_deg']
    aspect = profile['image_aspect_ratio']
    if not 0<horizontal_fov<180 or aspect<1:
        raise ValueError('Invalid camera FOV or image aspect ratio')
    wide_width = 2*altitude*math.tan(math.radians(horizontal_fov/2))
    zoom = wide_width/aspect/field
    if not 1<=zoom<=10:
        raise ValueError('Requested field cannot be achieved within nominal optical zoom range')
    frame_width = field*aspect
    def coordinates(span):
        if span<=0:
            return [0]
        count = max(2,math.ceil(span/(field*(1-overlap))-1e-10)+1)
        return [-span/2+index*span/(count-1) for index in range(count)]
    east_offsets,north_offsets = coordinates(max(0,width-frame_width)),coordinates(max(0,width-field))
    offsets = [[east,north] for row,north in enumerate(north_offsets)
        for east in (east_offsets if row%2==0 else list(reversed(east_offsets)))]
    radius_x,radius_y = frame_width/2,field/2
    covered = unary_union([box(east-radius_x,north-radius_y,east+radius_x,north+radius_y) for east,north in offsets])
    missing = box(-width/2,-width/2,width/2,width/2).difference(covered).area
    if missing>.01:
        raise ValueError('Raster observation fields leave a station coverage gap')
    return dict(zone=zone,complexity=configuration['complexity'],station_footprint_m=width,
        instantaneous_frame_m=[frame_width,field],minimum_overlap_ratio=overlap,grid_rows=len(north_offsets),grid_columns=len(east_offsets),
        pattern='SERPENTINE_FIELD_TILING',scan_mode=profile.get('scan_mode','STOP_AND_OBSERVE'),
        initial_look_offset_enu_m=offsets[0],initial_look_mode='RAPID_REPOINT_UNTIMED_IN_SIMULATION',
        offsets_enu_m=offsets[1:],scan_zoom=zoom,
        raw_nadir_frame_m=[wide_width,wide_width/aspect],work_altitude_m=altitude,
        ground_scan_speed_m_s=profile['maximum_ground_scan_speed_m_s'],
        observation_window_s=profile['minimum_observation_s'],calibrated=False,
        zoom_scope='focal-length ratio estimate; SDK zoom scale not calibrated',
        instantaneous_model='flat ground, full nominal 16:9 nadir frame; distortion, oblique projection and usable detection margins not calibrated')
