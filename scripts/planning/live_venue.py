"""Immutable measured-venue contract shared by the live planner and controller."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from control_bridge_preview import load_converter
from demo_mission import DemoMission
from venue_geometry import planes_for_frame

SCOPE='Live grounded venue reference; no flight initiation'
SOURCE_TOPICS=('/Odometry','/super_debug/aligned_pose_cov','/mavros/local_position/pose',
               '/mavros/vision_pose/pose_cov','/mavros/state','/mavros/extended_state',
               '/mavros/estimator_status')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_ids(node):
    result={}
    for topic in SOURCE_TOPICS:
        infos=node.get_publishers_info_by_topic(topic)
        if len(infos)!=1:raise ValueError('Need one live venue source for '+topic)
        result[topic]=bytes(infos[0].endpoint_gid).hex()
    return result


class LiveVenue:
    def __init__(self,path,mission_file,alignment_path,planner_config):
        self.path=Path(path);self.contents=self.path.read_bytes();self.data=json.loads(self.contents)
        data=self.data
        if data.get('schema_version')!=1 or data.get('scope')!=SCOPE:
            raise ValueError('Need a live grounded venue reference')
        for key,path in (('mission_sha256',mission_file),('source_alignment_sha256',alignment_path),
                         ('planner_profile_sha256',planner_config)):
            if digest(path)!=data.get(key):raise ValueError('Live venue file mismatch: '+key)
        mission=DemoMission.load(mission_file)
        if mission.rehearsal_only:raise ValueError('Live venue needs the physical mission')
        self.ground=np.asarray(data['ground_fc_xyz'],dtype=float)
        self.yaw=float(data['ground_fc_yaw_rad'])
        self.origin=np.asarray(data['origin'],dtype=float)
        self.goal_q=np.asarray(data['fixed_raw_goal_quaternion_xyzw'],dtype=float)
        planes,geometry=planes_for_frame(mission,load_converter(alignment_path),
            self.goal_q,self.origin,self.ground,self.yaw)
        configured=np.asarray(yaml.safe_load(Path(planner_config).read_bytes())
                              .get('super_planner',{}).get('venue_halfspaces',[]),dtype=float)
        recorded=np.asarray(data['halfspaces'],dtype=float)
        if (configured.shape!=(24,) or recorded.shape!=(6,4)
                or not np.allclose(configured.reshape(6,4),planes,atol=1e-10,rtol=0)
                or not np.allclose(recorded,planes,atol=1e-10,rtol=0)):
            raise ValueError('Live planner/controller venue planes differ')
        ids=data.get('source_ids',{})
        if set(ids)!=set(SOURCE_TOPICS) or any(
                not isinstance(v,str) or len(v)!=48 or any(c not in '0123456789abcdef' for c in v)
                for v in ids.values()):
            raise ValueError('Invalid live venue source identities')
        self.source_ids=ids
        if 'venue_calibration' in mission.config:
            from rviz_venue_calibration import validate_geometry,validate_session
            calibrated=validate_geometry(mission.config)
            anchor=validate_session(calibrated,data,digest(alignment_path))
            if (not np.allclose(self.ground,anchor['ground_fc_xyz'],atol=1e-10,rtol=0)
                    or abs(self.yaw-anchor['ground_fc_yaw_rad'])>1e-10):
                raise ValueError('Live venue moved relative to the confirmed RViz frame')
        self.geometry=geometry

    def validate(self,node,mapping):
        try:unchanged=self.path.read_bytes()==self.contents
        except OSError as exc:raise ValueError('Cannot verify live venue reference: '+str(exc)) from exc
        if not unchanged:raise ValueError('Live venue reference changed')
        if source_ids(node)!=self.source_ids:raise ValueError('Live venue localization/FC source changed; prepare a fresh session')
        if mapping.origin is None or np.linalg.norm(mapping.origin-self.origin)>.005:
            raise ValueError('Live venue localization origin changed')

    def bind(self,reference):
        # These are the existing grounded displacement and heading bounds.
        # Preserve the captured anchor rather than moving the box at READY.
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        axes=np.array([[c,s,0.],[-s,c,0.],[0.,0.,1.]])
        if np.any(np.abs(axes@(np.asarray(reference.ground)-self.ground))>.10):
            raise ValueError('Estimated ground offset exceeds 0.10 m per axis since venue capture')
        difference=(reference.yaw-self.yaw+math.pi)%(2*math.pi)-math.pi
        if abs(difference)>math.radians(15):raise ValueError('Heading departure exceeds 15 degrees since venue capture')
        reference.ground=tuple(self.ground)
        reference.yaw=self.yaw
