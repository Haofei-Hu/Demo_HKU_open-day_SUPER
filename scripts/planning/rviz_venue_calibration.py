"""Same-session lateral boundary selection; no ROS, parameter or flight output."""
from copy import deepcopy
import math
import numpy as np

METHOD='rviz_lateral_points_v1'


def axes(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,s,0.],[-s,c,0.],[0.,0.,1.]])


def world_rotation(converter,quaternion):
    q=np.asarray(quaternion);zero=converter(np.zeros(3),q,np.eye(6))[0]
    rotation=np.column_stack([converter(v,q,np.eye(6))[0]-zero for v in np.eye(3)])
    if (not np.all(np.isfinite(rotation)) or
            not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-8,rtol=0) or
            abs(np.linalg.det(rotation)-1)>1e-8):
        raise ValueError('Calibration requires a rigid localization rotation')
    # Environmental points are not vehicle/IMU poses: do not add the body lever arm.
    return rotation


def selected_geometry(left,right,observation,rotation,rear):
    points=np.asarray([left,right],dtype=float)
    if points.shape!=(2,3) or not np.all(np.isfinite(points)):
        raise ValueError('Select two finite points on the actual lateral boundaries')
    if not math.isfinite(rear) or rear<0:raise ValueError('Invalid rear offset')
    ground=np.asarray(observation['ground_fc_xyz']);origin=np.asarray(observation['origin'])
    basis=axes(observation['ground_fc_yaw_rad'])
    relative=(basis@(rotation@points.T-origin[:,None]-ground[:,None])).T
    left_y,right_y=relative[:,1]
    if not right_y<0<left_y:
        raise ValueError('Left/right order is reversed or the aircraft is outside the selected sides; reset and select again')
    # Never enlarge the requested venue or place it beyond a marked boundary.
    width=min(2.8,float(left_y-right_y));middle=float((left_y+right_y)/2)
    left_limit=middle+width/2;right_limit=middle-width/2
    if not right_limit<0<left_limit:
        raise ValueError('Aircraft is outside the capped 2.8 m box; reset the selection or reposition before preparation')
    return dict(method=METHOD,selected_raw_points=points.tolist(),
                selected_lateral_limits_m=[float(right_y),float(left_y)],
                width_m=width,left_offset_m=left_limit,rear_offset_m=float(rear),
                lateral_limits_m=[right_limit,left_limit],
                forward_limits_m=[-float(rear),5.8-float(rear)],
                floor_height_limits_m=[0.,2.8],
                nominal_maximum_width_m=2.8,
                heading_rule='Sides parallel to the confirmed takeoff heading; picks do not estimate wall yaw')


def apply_geometry(config,geometry,observation,alignment_sha256):
    result=deepcopy(config)
    result['venue']['width_left_m']=geometry['width_m']
    result['venue']['calibration_method']=METHOD
    result['takeoff']['distance_from_left_m']=geometry['left_offset_m']
    result['takeoff']['forward_from_rear_m']=geometry['rear_offset_m']
    result['venue_calibration']=dict(geometry=deepcopy(geometry),anchor=deepcopy(observation),
        alignment_sha256=alignment_sha256,scope='Current localization session only',
        confirmed_by_operator=True,native_control_or_flight_requests=False)
    return result


def validate_geometry(config):
    """Reject malformed or silently widened saved calibration values."""
    data=config.get('venue_calibration',{});geometry=data.get('geometry',{})
    if (data.get('confirmed_by_operator') is not True or geometry.get('method')!=METHOD
            or config['venue'].get('calibration_method')!=METHOD):
        raise ValueError('Missing confirmed RViz venue calibration')
    limits=np.asarray(geometry.get('lateral_limits_m'),dtype=float)
    selected=np.asarray(geometry.get('selected_lateral_limits_m'),dtype=float)
    if (limits.shape!=(2,) or selected.shape!=(2,) or not np.all(np.isfinite([*limits,*selected]))
            or not limits[0]<0<limits[1] or limits[0]<selected[0]-1e-10 or limits[1]>selected[1]+1e-10
            or not 0<limits[1]-limits[0]<=2.8+1e-10):
        raise ValueError('Invalid calibrated lateral interval')
    expected=((config['venue']['width_left_m'],limits[1]-limits[0]),
              (config['takeoff']['distance_from_left_m'],limits[1]),
              (config['takeoff']['forward_from_rear_m'],geometry.get('rear_offset_m')))
    if any(not isinstance(x,(int,float)) or isinstance(x,bool) or not math.isfinite(x)
           for pair in expected for x in pair) or any(abs(a-b)>1e-10 for a,b in expected):
        raise ValueError('Calibrated mission geometry changed after confirmation')
    return data


def validate_session(data,observation,alignment_sha256):
    anchor=data['anchor']
    if data['alignment_sha256']!=alignment_sha256 or anchor['source_ids']!=observation['source_ids']:
        raise ValueError('RViz calibration belongs to another localization session; select boundaries again')
    if np.linalg.norm(np.asarray(anchor['origin'])-observation['origin'])>.005:
        raise ValueError('Localization origin changed since RViz calibration')
    delta=axes(anchor['ground_fc_yaw_rad'])@(np.asarray(observation['ground_fc_xyz'])-anchor['ground_fc_xyz'])
    angle=(observation['ground_fc_yaw_rad']-anchor['ground_fc_yaw_rad']+math.pi)%(2*math.pi)-math.pi
    if np.any(np.abs(delta)>.10) or abs(angle)>math.radians(15):
        raise ValueError('Aircraft moved since RViz calibration; select boundaries again')
    # The existing grounded pose limits are only validation. Never move the box.
    return deepcopy(anchor)


def wireframe(geometry,observation,rotation):
    import itertools
    bounds=(geometry['forward_limits_m'],geometry['lateral_limits_m'],geometry['floor_height_limits_m'])
    vertices=np.array(list(itertools.product(*bounds)))
    aligned=(axes(observation['ground_fc_yaw_rad']).T@(vertices-[0,0,.085]).T).T
    raw=(rotation.T@(aligned+observation['ground_fc_xyz']+np.asarray(observation['origin'])).T).T
    edges=[(i,j) for i in range(8) for j in range(i+1,8) if (i^j) in (1,2,4)]
    return raw,edges
