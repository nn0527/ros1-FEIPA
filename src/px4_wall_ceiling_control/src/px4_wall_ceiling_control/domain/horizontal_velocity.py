"""Bounded horizontal velocity feedback in ENU; no direct motor control."""
import math

from px4_wall_ceiling_control.domain.recovery_attitude_limits import attitude_from_axis_yaw


class HorizontalVelocityController:
    def __init__(self, kp=1.5, max_acceleration=.30, target_acceleration=.10):
        for value in (kp, max_acceleration, target_acceleration):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('horizontal gains and acceleration limits must be finite and positive')
        self.kp = kp
        self.max_acceleration = max_acceleration
        self.target_acceleration = target_acceleration
        self.reset()

    def reset(self):
        self.target = (0., 0.)
        self.acceleration = (0., 0.)

    def attitude(self, desired_velocity, measured_velocity, yaw, dt):
        values = (*desired_velocity, *measured_velocity, yaw, dt)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('nonfinite horizontal control input')
        dt = min(.05, max(0., dt))
        delta = tuple(a-b for a,b in zip(desired_velocity, self.target))
        length = math.hypot(*delta)
        fraction = min(1., self.target_acceleration*dt/length) if length else 1.
        self.target = tuple(old+fraction*change for old,change in zip(self.target,delta))
        acceleration = tuple(self.kp*(target-measured) for target,measured in zip(self.target,measured_velocity))
        norm = math.hypot(*acceleration)
        scale = min(1., self.max_acceleration/norm) if norm else 1.
        self.acceleration = tuple(value*scale for value in acceleration)
        # A level-hover approximation. Final target cone/rate limits apply in the mux.
        axis = (*self.acceleration, 9.80665)
        norm = math.sqrt(sum(value*value for value in axis))
        return attitude_from_axis_yaw(tuple(value/norm for value in axis),yaw)
