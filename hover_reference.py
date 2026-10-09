#!/usr/bin/env python3
"""Smooth fixed-XY reference for a ground preview of the first hover.

ENU metres. The caller must establish the actual takeoff floor and freeze the
ground FC-centre pose. This curve is not a flight controller or landing detector.
"""
import math


class HoverReference:
    def __init__(self, ground_xyz, yaw, ground_fc_height=.085,
                 hover_fc_height=.5, peak_vertical_speed=.2, hold_seconds=2.):
        self.ground = tuple(float(v) for v in ground_xyz)
        numbers = (*self.ground, yaw, ground_fc_height, hover_fc_height,
                   peak_vertical_speed, hold_seconds)
        if len(self.ground) != 3 or not all(math.isfinite(v) for v in numbers):
            raise ValueError('Expected finite XYZ, yaw, heights, speed and hold duration')
        if not (0 < ground_fc_height < hover_fc_height <= 2
                and 0 < peak_vertical_speed <= .2 and 0 <= hold_seconds <= 6):
            raise ValueError('Invalid first-hover preview limits')
        self.yaw = float(yaw)
        self.lift = hover_fc_height - ground_fc_height
        # max derivative of 10u^3 - 15u^4 + 6u^5 on [0,1] is 1.875.
        self.ramp_seconds = 1.875 * self.lift / peak_vertical_speed
        self.hold_seconds = float(hold_seconds)
        self.duration = 2 * self.ramp_seconds + self.hold_seconds

    @staticmethod
    def smooth(u):
        u = max(0., min(1., u))
        return u ** 3 * (10. + u * (-15. + 6. * u))

    def at(self, seconds):
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError('Elapsed time must be finite and nonnegative')
        if seconds < self.ramp_seconds:
            fraction = self.smooth(seconds / self.ramp_seconds)
            phase = 'RISE_PREVIEW'
        elif seconds < self.ramp_seconds + self.hold_seconds:
            fraction = 1.
            phase = 'HOLD_PREVIEW'
        elif seconds < self.duration:
            fraction = 1. - self.smooth(
                (seconds - self.ramp_seconds - self.hold_seconds) / self.ramp_seconds)
            phase = 'DESCENT_PREVIEW'
        else:
            fraction = 0.
            phase = 'COMPLETE_GROUND_HOLD'
        return (self.ground[0], self.ground[1], self.ground[2] + self.lift * fraction), self.yaw, phase
