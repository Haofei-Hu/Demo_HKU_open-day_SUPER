#!/usr/bin/env python3
"""Left-facing mission adapter; rejects all physical execution contexts."""
from left_rehearsal import LeftMission
from super_demo_control import DemoMissionControl,VenueReference


class LeftVenueReference(VenueReference):
    MAX_ACCELERATION=1.5


class LeftMissionControl(DemoMissionControl):
    PLANNER_LIMITS=REFERENCE_LIMITS=(2.,1.5)

    def __init__(self,*args,**kwargs):
        if kwargs.get('simulation') is not True:
            raise ValueError('Left-facing controller is virtual rehearsal only')
        super().__init__(*args,**kwargs,mission_class=LeftMission)

    def make_follower(self):
        return LeftVenueReference(self.frame,self.check_stage_endpoint,self.final_goal_reached,
            self.motion_policy,max_speed=self.REFERENCE_LIMITS[0],max_acceleration=self.REFERENCE_LIMITS[1])
