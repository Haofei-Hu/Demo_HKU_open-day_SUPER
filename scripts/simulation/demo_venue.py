#!/usr/bin/env python3
"""Express the rehearsal controller's vehicle-inclusive venue in planner world."""
import hashlib
import math
from pathlib import Path

import numpy as np

from control_bridge_preview import load_converter,quaternion_rpy
from demo_mission import DemoMission


def halfspaces(scene,mission,converter=None):
    """Use the exact fixed goal attitude/lever arm and floor convention of control."""
    if not mission.rehearsal_only:raise ValueError('Frozen-scene bounds are for ground rehearsal only')
    converter=converter or load_converter(Path(scene['alignment_path']))
    raw_q=np.array(scene['raw_quaternion'],dtype=float,copy=True)
    aligned_q=np.array(scene['aligned_quaternion'],dtype=float,copy=True)
    ground=np.asarray(scene['aligned_position'],dtype=float);origin=np.asarray(scene['origin'],dtype=float)
    if (raw_q.shape!=(4,) or aligned_q.shape!=(4,) or ground.shape!=(3,) or origin.shape!=(3,)
            or not np.all(np.isfinite([*raw_q,*aligned_q,*ground,*origin]))
            or abs(np.linalg.norm(raw_q)-1)>.02 or abs(np.linalg.norm(aligned_q)-1)>.02):
        raise ValueError('Invalid frozen venue frame')
    raw_q/=np.linalg.norm(raw_q);aligned_q/=np.linalg.norm(aligned_q)
    def yaw(q):
        x,y,z,w=q
        return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    # inverse_goal deliberately retains raw planner yaw with zero raw roll/pitch.
    goal_q=quaternion_rpy(0.,0.,yaw(raw_q))
    zero=converter(np.zeros(3),goal_q,np.eye(6))[0]
    rotation=np.column_stack([converter(axis,goal_q,np.eye(6))[0]-zero for axis in np.eye(3)])
    if (not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(zero))
            or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-8,rtol=0)
            or abs(np.linalg.det(rotation)-1)>1e-8):raise ValueError('Venue conversion must be rigid')
    c,s=math.cos(yaw(aligned_q)),math.sin(yaw(aligned_q))
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
    record=dict(schema_version=1,scope='Frozen-scene ground rehearsal only',
        frame='world',inequality='normal dot raw_planner_position + offset <= 0',
        halfspaces=planes.tolist(),world_to_venue_matrix=matrix.tolist(),world_to_venue_offset=offset.tolist(),
        fc_centre_lower_forward_left_floor_m=lower.tolist(),fc_centre_upper_forward_left_floor_m=upper.tolist(),
        ground_aligned_xyz=ground.tolist(),takeoff_aligned_yaw_rad=yaw(aligned_q),fixed_raw_goal_quaternion_xyzw=goal_q.tolist(),
        additional_boundary_inset_m=0.,vehicle_extents_applied_once=True,
        source_alignment_sha256=scene.get('alignment_sha256'),source_planner_sha256=scene.get('planner_sha256'),
        captured_cloud_changed=False,optimization_targets_changed=False,motion_acceptance_changed=False)
    return planes,record


def configure(config,scene,mission_file):
    mission=DemoMission.load(mission_file)
    planes,record=halfspaces(scene,mission)
    config['super_planner']['venue_halfspaces']=planes.reshape(-1).tolist()
    record['mission_sha256']=hashlib.sha256(Path(mission_file).read_bytes()).hexdigest()
    record['search_policy']=dict(elapsed_time_limit_s=None,
        termination='Goal/horizon reached or finite admissible grid exhausted; operator can interrupt',
        obstacle_checks_retained=True,venue_checks_retained=True)
    print('DEMO VENUE: same vehicle-inclusive box for A* search and all corridors; no extra inset',flush=True)
    print('DEMO SEARCH: finite venue search to completion; no elapsed-time cutoff',flush=True)
    return record
