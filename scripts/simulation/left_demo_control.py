#!/usr/bin/env python3
"""Left-facing mission adapter; rejects all physical execution contexts."""
from left_rehearsal import LeftMission
from super_demo_control import DemoMissionControl


class LeftMissionControl(DemoMissionControl):
    def __init__(self,*args,**kwargs):
        if kwargs.get('simulation') is not True:
            raise ValueError('Left-facing controller is virtual rehearsal only')
        super().__init__(*args,**kwargs,mission_class=LeftMission)
