"""Same-session lateral boundary selection; no ROS, parameter or flight output."""
from copy import deepcopy
import math
import numpy as np

METHOD='rviz_lateral_points_v1'
REHEARSAL_SCOPE='Frozen-scene ground rehearsal only'


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
        raise ValueError(f'Points must bracket the aircraft along its left axis: '
                         f'LEFT={left_y:+.3f} m must be positive, RIGHT={right_y:+.3f} m must be negative. '
                         'Sides follow the nose, not the screen; reset to select again')
    # Use the selected sides exactly; no nominal-width cap, recentering or inset.
    left_limit=float(left_y);right_limit=float(right_y);width=left_limit-right_limit
    return dict(method=METHOD,selected_raw_points=points.tolist(),
                selected_lateral_limits_m=[float(right_y),float(left_y)],
                width_m=width,left_offset_m=left_limit,rear_offset_m=float(rear),
                lateral_limits_m=[right_limit,left_limit],
                forward_limits_m=[-float(rear),5.8-float(rear)],
                floor_height_limits_m=[0.,2.8],
                width_policy='exact_selected_lateral_boundaries',
                heading_rule='Sides parallel to the confirmed takeoff heading; picks do not estimate wall yaw')


def apply_geometry(config,geometry,observation,alignment_sha256):
    result=deepcopy(config)
    result['venue']['width_left_m']=geometry['width_m']
    result['venue']['height_m']=geometry['floor_height_limits_m'][1]
    result['venue']['calibration_method']=METHOD
    result['takeoff']['distance_from_left_m']=geometry['left_offset_m']
    result['takeoff']['forward_from_rear_m']=geometry['rear_offset_m']
    result['venue_calibration']=dict(geometry=deepcopy(geometry),anchor=deepcopy(observation),
        alignment_sha256=alignment_sha256,scope='Current localization session only',
        confirmed_by_operator=True,native_control_or_flight_requests=False)
    return result


def cap_rehearsal_width(geometry,width):
    """Intersect the surveyed sides with an explicitly requested centred rehearsal box."""
    if isinstance(width,bool) or not isinstance(width,(int,float)) or not math.isfinite(width) or width<=0:
        raise ValueError('Rehearsal width must be finite and positive')
    result=deepcopy(geometry)
    right,left=result['selected_lateral_limits_m']
    right=max(right,-width/2);left=min(left,width/2)
    result.update(lateral_limits_m=[right,left],width_m=left-right,left_offset_m=left,
                  width_policy='rehearsal_centered_width_cap',requested_rehearsal_width_m=width)
    return result


def cap_rehearsal_ceiling(geometry,ceiling):
    if (isinstance(ceiling,bool) or not isinstance(ceiling,(int,float))
            or not math.isfinite(ceiling) or not 0<ceiling<=2.8):
        raise ValueError('Rehearsal ceiling must be positive and no higher than the 2.8 m venue')
    result=deepcopy(geometry)
    result.update(floor_height_limits_m=[0.,ceiling],requested_rehearsal_ceiling_m=ceiling)
    return result


def cap_selected_box(geometry,width=None,ceiling=None):
    """Apply explicit operator limits inside the selected venue, including physical preparation."""
    result=deepcopy(geometry)
    if width is not None:
        result=cap_rehearsal_width(result,width)
        result.pop('requested_rehearsal_width_m')
        result.update(width_policy='operator_centered_width_cap',requested_venue_width_m=width)
    if ceiling is not None:
        result=cap_rehearsal_ceiling(result,ceiling)
        result.pop('requested_rehearsal_ceiling_m')
        result['requested_venue_ceiling_m']=ceiling
    return result


def full_size_execution_geometry(geometry):
    """Explicit 5.8 x 2.8 x 2.8 measured-position box around the planning box.

    Preserve the selected heading, rear, floor and lateral midpoint. This is
    operator-supplied geometry, not an inference of walls from the point cloud.
    """
    right,left=geometry['lateral_limits_m']
    forward=geometry['forward_limits_m']
    floor=geometry['floor_height_limits_m']
    rear=geometry['rear_offset_m']
    values=[right,left,rear,*forward,*floor]
    if (not all(math.isfinite(v) for v in values) or not right<left
            or left-right>2.8+1e-10 or abs(forward[1]-forward[0]-5.8)>1e-10
            or rear<0 or not np.allclose(forward,[-rear,5.8-rear],atol=1e-10,rtol=0)
            or floor[0]!=0. or not 0<floor[1]<=2.8):
        raise ValueError('Planning box must fit the 5.8 x 2.8 x 2.8 execution venue')
    centre=(right+left)/2
    return dict(policy='full_size_execution_venue_v1',
                lateral_limits_m=[centre-1.4,centre+1.4],
                forward_limits_m=list(forward),floor_height_limits_m=[0.,2.8])


def separate_execution_geometry(geometry):
    result=deepcopy(geometry)
    result['execution_envelope']=full_size_execution_geometry(geometry)
    return result


def validate_geometry(config):
    """Require saved mission sides to match the operator-confirmed selection."""
    data=config.get('venue_calibration',{});geometry=data.get('geometry',{})
    if (data.get('confirmed_by_operator') is not True or geometry.get('method')!=METHOD
            or config['venue'].get('calibration_method')!=METHOD):
        raise ValueError('Missing confirmed RViz venue calibration')
    limits=np.asarray(geometry.get('lateral_limits_m'),dtype=float)
    selected=np.asarray(geometry.get('selected_lateral_limits_m'),dtype=float)
    if (limits.shape!=(2,) or selected.shape!=(2,) or not np.all(np.isfinite([*limits,*selected]))
            or not limits[0]<0<limits[1] or not selected[0]<0<selected[1]):
        raise ValueError('Invalid calibrated lateral interval')
    expected_limits=selected
    if geometry.get('width_policy')=='rehearsal_centered_width_cap':
        if config.get('execution_context')!='ground_rehearsal':
            raise ValueError('Rehearsal width cap is not a physical venue calibration')
        expected_limits=cap_rehearsal_width(geometry,geometry.get('requested_rehearsal_width_m'))['lateral_limits_m']
    elif geometry.get('width_policy')=='operator_centered_width_cap':
        expected_limits=cap_rehearsal_width(geometry,geometry.get('requested_venue_width_m'))['lateral_limits_m']
    if not np.allclose(limits,expected_limits,atol=1e-10,rtol=0):
        raise ValueError('Invalid calibrated lateral interval')
    ceiling=2.8
    if 'requested_rehearsal_ceiling_m' in geometry:
        if config.get('execution_context')!='ground_rehearsal':
            raise ValueError('Rehearsal ceiling is not a physical venue calibration')
        ceiling=cap_rehearsal_ceiling(geometry,geometry['requested_rehearsal_ceiling_m'])['floor_height_limits_m'][1]
    if 'requested_venue_ceiling_m' in geometry:
        if 'requested_rehearsal_ceiling_m' in geometry:
            raise ValueError('Conflicting ceiling policies')
        ceiling=cap_rehearsal_ceiling(geometry,geometry['requested_venue_ceiling_m'])['floor_height_limits_m'][1]
    if geometry.get('floor_height_limits_m')!=[0.,ceiling]:
        raise ValueError('Calibrated floor/ceiling changed after confirmation')
    expected=((config['venue']['width_left_m'],limits[1]-limits[0]),
              (config['venue']['height_m'],ceiling),
              (config['takeoff']['distance_from_left_m'],limits[1]),
              (config['takeoff']['forward_from_rear_m'],geometry.get('rear_offset_m')))
    if any(not isinstance(x,(int,float)) or isinstance(x,bool) or not math.isfinite(x)
           for pair in expected for x in pair) or any(abs(a-b)>1e-10 for a,b in expected):
        raise ValueError('Calibrated mission geometry changed after confirmation')
    if 'execution_envelope' in geometry:
        if geometry['execution_envelope']!=full_size_execution_geometry(geometry):
            raise ValueError('Execution venue changed after confirmation')
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


def freeze_scene(config,scene,observation,converter):
    """Keep the clicked world box fixed when transferring into a private simulation."""
    data=validate_geometry(config)
    if config.get('execution_context')!='ground_rehearsal' or data.get('scope')!=REHEARSAL_SCOPE:
        raise ValueError('Frozen calibration requires a ground-rehearsal mission')
    anchor=validate_session(data,observation,scene['alignment_sha256'])
    result=deepcopy(scene)
    result['captured_vehicle_pose']={k:deepcopy(scene[k]) for k in
        ('raw_position','raw_quaternion','aligned_position','aligned_quaternion','origin')}
    q=np.asarray(anchor['raw_quaternion'])
    rotation=world_rotation(converter,q)
    zero,aligned_q,_=converter(np.zeros(3),q,np.eye(6))
    yaw=anchor['ground_fc_yaw_rad']
    result.update(aligned_position=anchor['ground_fc_xyz'],
        aligned_quaternion=np.asarray(aligned_q).tolist(),
        local_quaternion=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)],
        raw_quaternion=anchor['raw_quaternion'],origin=anchor['origin'],
        raw_position=(rotation.T@(np.asarray(anchor['ground_fc_xyz'])+anchor['origin']-zero)).tolist(),
        venue_calibration_scope=REHEARSAL_SCOPE)
    return result


def validate_frozen_scene(config,scene):
    data=validate_geometry(config);anchor=data['anchor']
    if (config.get('execution_context')!='ground_rehearsal' or data.get('scope')!=REHEARSAL_SCOPE
            or scene.get('venue_calibration_scope')!=REHEARSAL_SCOPE
            or data['alignment_sha256']!=scene['alignment_sha256']):
        raise ValueError('Missing matching frozen venue calibration')
    for actual,expected in ((scene['aligned_position'],anchor['ground_fc_xyz']),
                            (scene['origin'],anchor['origin']),
                            (scene['raw_quaternion'],anchor['raw_quaternion'])):
        if not np.allclose(actual,expected,atol=1e-10,rtol=0):
            raise ValueError('Frozen scene moved relative to the selected venue')
    x,y,z,w=scene['local_quaternion']
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    angle=(yaw-anchor['ground_fc_yaw_rad']+math.pi)%(2*math.pi)-math.pi
    if abs(angle)>1e-10:raise ValueError('Frozen heading changed after venue selection')
