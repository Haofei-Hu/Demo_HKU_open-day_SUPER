#!/usr/bin/env python3
"""Disarmed forward-5-m audit with 2 m/s and 1 m/s^2 reference limits.

No execution flag. This changes planner/reference limits only, not PX4 limits.
"""
import argparse
from pathlib import Path

import numpy as np
from bounded_reference_preview import RollingGoalReference
from super_flight_control import SuperFlightControl


GroundRollingReference=RollingGoalReference  # Compatibility for the standalone ground audit.


class GroundSpeedAudit(SuperFlightControl):
    PLANNER_LIMITS=(2.,1.)
    REFERENCE_LIMITS=(2.,1.)
    MAX_DISTANCE=5.
    FRAME_FORWARD_LIMIT=5.1  # 5 m target plus the existing 10 cm endpoint margin.

    def __init__(self,alignment_path=None,planner_config=None):
        self.committed_plans=0
        self.last_endpoint_kind=None
        super().__init__(execute=False,distance=5.,alignment_path=alignment_path,
                         planner_config=planner_config)

    def check_stage_endpoint(self,plan):
        endpoint,error,detail=self.endpoint_evidence(plan)
        try:self.target_ok(endpoint,'Committed stage endpoint')
        except ValueError as exc:raise ValueError(str(exc)+'; '+detail) from exc
        return bool(np.all(np.abs(error)<=.10))

    def make_follower(self):
        return GroundRollingReference(self.frame,self.check_stage_endpoint,self.final_goal_reached)

    def final_goal_reached(self,plan,position):
        error=self.frame.axes@(np.asarray(position)-self.requested_target)
        return self.check_stage_endpoint(plan) and bool(np.all(np.abs(error)<=.10))

    def accept_plan(self,plan,msg):
        final=self.check_stage_endpoint(plan)
        self.follower.add_plan(msg)
        self.committed_plans+=1
        kind='FINAL_GOAL_CANDIDATE' if final else 'INTERMEDIATE_STOP'
        if kind!=self.last_endpoint_kind:
            self.get_logger().info('Committed endpoint '+kind+'; '+self.endpoint_evidence(plan)[2])
            self.last_endpoint_kind=kind

    def supported_command(self,flag):
        # Both regular and backup commands must match a verified polynomial.
        return flag in (1,2)


def main():
    import rclpy
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--alignment-path',type=Path,required=True)
    parser.add_argument('--planner-config',type=Path,required=True)
    args=parser.parse_args()
    rclpy.init()
    node=None
    try:
        node=GroundSpeedAudit(args.alignment_path,args.planner_config)
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:node.abort('Ground speed audit interrupted')
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
