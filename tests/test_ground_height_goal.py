"""FC-floor height inversion across rotated worlds, mounting and lever arms."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/planning'))
sys.path.insert(0,str(ROOT/'scripts/localization'))
from control_bridge_preview import Mapping,quaternion_rpy
from aligned_pose_preview import multiply,rotation
from send_ground_height_goal import height_goal


def pose(p,q):
    return NS(position=NS(**dict(zip(('x','y','z'),p))),orientation=NS(**dict(zip(('x','y','z','w'),q))))


class Tests(unittest.TestCase):
    def geometry(self):
        world_q=quaternion_rpy(.3,.6,1.7);world=rotation(world_q)
        mount_q=quaternion_rpy(.02,-.03,-np.pi/4);lever=np.array([.04,-.03,.11])
        def convert(p,q,cov):
            body=multiply(q,mount_q)
            return world@(p-rotation(body)@lever),multiply(world_q,body),cov
        raw_p=np.array([2.,-.7,.8]);raw_q=quaternion_rpy(.14,-.09,.5)
        p,q,_=convert(raw_p,raw_q,np.eye(6));origin=np.array([1.1,-.4,.7])
        raw,aligned=pose(raw_p,raw_q),pose(p-origin,q)
        mapping=Mapping(convert)
        for _ in range(10):mapping.pair(raw,aligned)
        return mapping,raw,aligned

    def test_rotated_vertical_goal_round_trip_keeps_fc_xy_and_includes_terminal_lever(self):
        mapping,raw,aligned=self.geometry()
        p,q,target=height_goal(mapping,raw,aligned,1.)
        current=np.array([aligned.position.x,aligned.position.y,aligned.position.z])
        np.testing.assert_allclose(target,current+[0,0,.915],atol=1e-12)
        out=mapping.converter(p,q,np.eye(6))[0]-mapping.origin
        np.testing.assert_allclose(out,target,atol=1e-12)
        np.testing.assert_allclose(q,quaternion_rpy(0,0,.5),atol=1e-12)
        naive=np.array([raw.position.x,raw.position.y,raw.position.z+.915])
        self.assertGreater(np.linalg.norm(mapping.converter(naive,q,np.eye(6))[0]-mapping.origin-target),.1)

    def test_rejects_invalid_height_and_unready_mapping(self):
        mapping,raw,aligned=self.geometry()
        for height in (.085,0.,-1.,1.001,float('nan'),float('inf')):
            with self.assertRaises(ValueError):height_goal(mapping,raw,aligned,height)
        mapping.pairs=9
        with self.assertRaisesRegex(ValueError,'matched'):height_goal(mapping,raw,aligned,1.)
        mapping.pairs=10;mapping.failed=True
        with self.assertRaisesRegex(ValueError,'matched'):height_goal(mapping,raw,aligned,1.)

    def test_rejects_nonrigid_conversion_and_invalid_pose(self):
        mapping,raw,aligned=self.geometry()
        converter=mapping.converter
        mapping.converter=lambda p,q,c:(2*converter(p,q,c)[0],q,c)
        with self.assertRaisesRegex(ValueError,'rigid'):height_goal(mapping,raw,aligned,1.)
        mapping.converter=converter;raw.orientation.w=2.
        with self.assertRaisesRegex(ValueError,'quaternion'):height_goal(mapping,raw,aligned,1.)


if __name__=='__main__':unittest.main()
