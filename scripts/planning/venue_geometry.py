"""Vehicle-centre venue planes for a fixed, explicitly supplied ground frame."""
import math

import numpy as np

from control_bridge_preview import quaternion_rpy


def yaw(quaternion):
    q=np.asarray(quaternion,dtype=float)
    if q.shape!=(4,) or not np.all(np.isfinite(q)) or abs(np.linalg.norm(q)-1)>.02:
        raise ValueError('Invalid venue quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def planes_for_frame(mission,converter,raw_quaternion,origin,ground_fc_xyz,ground_yaw):
    """Match inverse_goal's attitude/lever arm and VenueFrame's floor convention."""
    origin=np.asarray(origin,dtype=float);ground=np.asarray(ground_fc_xyz,dtype=float)
    if (origin.shape!=(3,) or ground.shape!=(3,)
            or not np.all(np.isfinite([*origin,*ground,ground_yaw]))):
        raise ValueError('Invalid venue ground/origin')
    goal_q=quaternion_rpy(0.,0.,yaw(raw_quaternion))
    zero=converter(np.zeros(3),goal_q,np.eye(6))[0]
    rotation=np.column_stack([converter(axis,goal_q,np.eye(6))[0]-zero for axis in np.eye(3)])
    if (not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(zero))
            or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-8,rtol=0)
            or abs(np.linalg.det(rotation)-1)>1e-8):
        raise ValueError('Venue conversion must be rigid')
    c,s=math.cos(ground_yaw),math.sin(ground_yaw)
    axes=np.array([[c,s,0.],[-s,c,0.],[0.,0.,1.]])
    matrix=axes@rotation
    offset=axes@(zero-origin-ground)+np.array([0.,0.,.085])
    hull=mission.hull
    lower=np.array([hull['rear']-mission.rear,mission.left-mission.width+hull['right'],hull['bottom']])
    upper=np.array([mission.length-mission.rear-hull['front'],mission.left-hull['left'],mission.height-hull['top']])
    if np.any(lower>=upper):raise ValueError('Vehicle-inclusive venue is empty')
    planes=[]
    for index in range(3):
        planes.extend([np.r_[matrix[index],offset[index]-upper[index]],
                       np.r_[-matrix[index],lower[index]-offset[index]]])
    planes=np.asarray(planes)
    record=dict(schema_version=1,frame='world',
        inequality='normal dot raw_planner_position + offset <= 0',
        halfspaces=planes.tolist(),world_to_venue_matrix=matrix.tolist(),world_to_venue_offset=offset.tolist(),
        fc_centre_lower_forward_left_floor_m=lower.tolist(),fc_centre_upper_forward_left_floor_m=upper.tolist(),
        ground_aligned_xyz=ground.tolist(),takeoff_aligned_yaw_rad=float(ground_yaw),
        fixed_raw_goal_quaternion_xyzw=goal_q.tolist(),
        boundary_policy=mission.boundary_policy(),
        additional_boundary_inset_m=0.,vehicle_extents_applied_once=True,
        captured_cloud_changed=False,optimization_targets_changed=False,motion_acceptance_changed=False)
    return planes,record
