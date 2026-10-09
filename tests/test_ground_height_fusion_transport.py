#!/usr/bin/env python3
"""Exercise the height-test relay over isolated ROS DDS, without aircraft."""
from ground_height_fusion_transport import GroundHeightFusionTransport
from test_ground_xy_fusion_transport import main


if __name__ == '__main__':
    main(GroundHeightFusionTransport, height_enabled=True)
