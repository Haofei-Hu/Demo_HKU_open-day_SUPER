#!/usr/bin/env python3
"""Separate left-facing virtual mission and read-only scene adaptation."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from demo_mission import DemoMission


class LeftMission(DemoMission):
    MOTION_LIMITS=(2.,1.5)
    PROFILES={'left_facing_48m_rehearsal':(4.8,5.8,'SUPER_OUTBOUND_4_8M',False)}

    def __init__(self,config):
        if config.get('execution_context')!='ground_rehearsal':
            raise ValueError('Left-facing mission is virtual rehearsal only')
        super().__init__(config)


def mission_fixture():
    """Explicit virtual hull/placement; never reuse it as a physical survey."""
    name='left_facing_48m_rehearsal.json'
    local=Path(__file__).resolve().with_name(name)
    source=local if local.is_file() else Path(__file__).resolve().parents[2]/'configs/demonstration'/name
    return json.loads(source.read_text())


def successful_scene(base):
    """Select a completed original run; never take a still-running capture."""
    for report in sorted(Path(base).glob('*/report.json'),reverse=True):
        try:
            data=json.loads(report.read_text())
            scene=report.parent/'scene.json'
            if data.get('result')=='SUCCESS' and data.get('actual_SUPER_tested') is True and scene.is_file():
                return scene
        except (OSError,ValueError):
            continue
    raise ValueError('No successful real-SUPER scene found; finish the original rehearsal or pass --scene-file')


def multiply(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return np.array([w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,
                     w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z])


def rotated_scene(source):
    """Rotate the virtual body +90 degrees about aligned up; keep FC and map fixed."""
    from control_bridge_preview import load_converter
    from demo_mission_simulation import viewer_body_orientation
    source=Path(source).resolve();original=json.loads(source.read_text())
    alignment=Path(original['alignment_path'])
    planner=Path(original['planner_config'])
    for path,key in ((alignment,'alignment_sha256'),(planner,'planner_sha256')):
        if hashlib.sha256(path.read_bytes()).hexdigest()!=original[key]:
            raise ValueError('Frozen scene dependency changed: '+str(path))
    if original.get('virtual_heading_rotation_deg'):
        raise ValueError('Choose an original scene, not an already rotated left-facing scene')
    convert=load_converter(alignment)
    raw_q=np.asarray(original['raw_quaternion']);zero=np.zeros(3);cov=np.eye(6)
    raw_zero=convert(zero,raw_q,cov)[0]
    rotation=np.column_stack([convert(axis,raw_q,cov)[0]-raw_zero for axis in np.eye(3)])
    if not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-8,rtol=0):
        raise ValueError('Scene conversion must be rigid')
    q_world=viewer_body_orientation(rotation.T,[0.,0.,0.,1.])
    q_left=np.array([0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)])
    inverse_world=np.asarray(q_world)*[-1,-1,-1,1]
    new_q=multiply(inverse_world,multiply(q_left,multiply(q_world,raw_q)))
    new_q/=np.linalg.norm(new_q)
    new_zero=convert(zero,new_q,cov)[0]
    # The mounting lever arm changes with attitude; compensate raw IMU position
    # so the virtual FC centre and the entire captured obstacle map stay fixed.
    aligned_position=np.asarray(original['aligned_position'])
    raw_position=rotation.T@(aligned_position+np.asarray(original['origin'])-new_zero)
    position,orientation,_=convert(raw_position,new_q,cov)
    if not np.allclose(position-original['origin'],aligned_position,atol=1e-8,rtol=0):
        raise ValueError('Virtual heading change moved the FC centre')
    expected=multiply(q_left,original['aligned_quaternion'])
    if not math.isclose(abs(float(np.dot(orientation,expected))),1.,abs_tol=1e-8):
        raise ValueError('Virtual heading rotation did not preserve the captured geometry')
    scene=deepcopy(original)
    scene.update(raw_quaternion=new_q.tolist(),raw_position=raw_position.tolist(),
        aligned_quaternion=np.asarray(orientation).tolist(),
        virtual_heading_rotation_deg=90.,source_scene_file=str(source),
        source_scene_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        actual_aircraft_moved=False,obstacle_map_rotated=False)
    return scene


def left_planner_profile(scene,report):
    """Apply left-only motion limits in a new YAML, preserving the source file."""
    import yaml
    source=Path(scene['planner_config'])
    data=source.read_bytes()
    if hashlib.sha256(data).hexdigest()!=scene['planner_sha256']:
        raise ValueError('Source planner configuration changed')
    config=yaml.safe_load(data);before=deepcopy(config['traj_opt']['boundary'])
    speed,acceleration=LeftMission.MOTION_LIMITS
    config['traj_opt']['boundary'].update(max_vel=speed,max_acc=acceleration)
    token=hashlib.sha256(str(Path(report).resolve()).encode()).hexdigest()[:12]
    target=source.with_name('mid360_left_48m_'+token+'.yaml')
    text='# Independent virtual left-facing profile; source YAML unchanged\n'+yaml.safe_dump(config,sort_keys=False)
    with target.open('x') as handle:handle.write(text)
    result=deepcopy(scene)
    result.update(planner_config=str(target),planner_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        source_planner_config=str(source),source_planner_sha256=scene['planner_sha256'],
        left_motion_override=dict(before=before,after=config['traj_opt']['boundary'],
            changed_fields=['traj_opt.boundary.'+key for key in ('max_vel','max_acc')
                            if before[key]!=config['traj_opt']['boundary'][key]]))
    print(f'LEFT PLANNER: max_vel={speed:g} m/s; max_acc={acceleration:g} m/s^2; '+str(target),flush=True)
    return result
