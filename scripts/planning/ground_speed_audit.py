#!/usr/bin/env python3
"""Disarmed forward-5-m audit with 2 m/s and 1 m/s^2 reference limits.

No execution flag. This changes planner/reference limits only, not PX4 limits.
"""
import argparse
from pathlib import Path

from super_flight_control import SuperFlightControl


class GroundSpeedAudit(SuperFlightControl):
    PLANNER_LIMITS=(2.,1.)
    REFERENCE_LIMITS=(2.,1.)
    MAX_DISTANCE=5.
    FRAME_FORWARD_LIMIT=5.1  # 5 m target plus the existing 10 cm endpoint margin.

    def __init__(self,alignment_path=None,planner_config=None):
        super().__init__(execute=False,distance=5.,alignment_path=alignment_path,
                         planner_config=planner_config)


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
