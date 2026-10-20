#!/usr/bin/env python3
"""Express the rehearsal controller's vehicle-inclusive venue in planner world."""
import hashlib
from pathlib import Path

from control_bridge_preview import load_converter
from venue_geometry import planes_for_frame,yaw
from demo_mission import DemoMission


def halfspaces(scene,mission,converter=None):
    """Use the exact fixed goal attitude/lever arm and floor convention of control."""
    if not mission.rehearsal_only:raise ValueError('Frozen-scene bounds are for ground rehearsal only')
    if 'venue_calibration' in mission.config:
        from rviz_venue_calibration import validate_frozen_scene
        validate_frozen_scene(mission.config,scene)
    converter=converter or load_converter(Path(scene['alignment_path']))
    planes,record=planes_for_frame(mission,converter,scene['raw_quaternion'],scene['origin'],
                                  scene['aligned_position'],yaw(scene.get('local_quaternion',scene['aligned_quaternion'])))
    record.update(scope='Frozen-scene ground rehearsal only',
        source_alignment_sha256=scene.get('alignment_sha256'),source_planner_sha256=scene.get('planner_sha256'))
    return planes,record


def configure(config,scene,mission_file):
    mission=DemoMission.load(mission_file)
    planes,record=halfspaces(scene,mission)
    config['super_planner']['venue_halfspaces']=planes.reshape(-1).tolist()
    record['mission_sha256']=hashlib.sha256(Path(mission_file).read_bytes()).hexdigest()
    record['search_policy']=dict(elapsed_time_limit_s=None,
        termination='Goal/horizon reached or finite admissible grid exhausted; operator can interrupt',
        obstacle_checks_retained=True,venue_checks_retained=True,
        from_rest_elapsed_rejection=False,from_rest_epoch='Complete primary/backup pair commit time',
        moving_replan_splice_checks_retained=True)
    print('DEMO VENUE: same vehicle-inclusive box for A* search and all corridors; no extra inset',flush=True)
    print('DEMO SEARCH: finite venue search to completion; no elapsed-time cutoff',flush=True)
    return record
