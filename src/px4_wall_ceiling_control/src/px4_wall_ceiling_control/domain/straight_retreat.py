"""Independent lateral damping around a trusted hover roll, at fixed heading."""
import math

from px4_wall_ceiling_control.domain.geometry import clamp, euler_from_quaternion
from px4_wall_ceiling_control.domain.recovery_attitude_limits import attitude_from_axis_yaw


class StraightRetreatAttitude:
    def __init__(self, lateral_kp=.5, roll_correction=math.radians(2.),
                 roll_rate=math.radians(1.)):
        if (not all(math.isfinite(v) and v > 0 for v in
                    (lateral_kp, roll_correction, roll_rate))
                or lateral_kp > 1.5 or roll_correction > math.radians(5.)
                or roll_rate > math.radians(10.)):
            raise ValueError('invalid straight retreat lateral settings')
        self.lateral_kp = lateral_kp
        self.roll_correction = roll_correction
        self.roll_rate = roll_rate
        self.roll = None

    def reset(self, axis, yaw):
        # Reexpress the entry thrust axis at the frozen heading, rather than
        # learning a new heading from the aircraft after its detach rotation.
        self.roll = euler_from_quaternion(attitude_from_axis_yaw(axis, yaw))[0]

    def attitude(self, baseline, forward_attitude, lateral_velocity, yaw, dt,
                 max_tilt, lateral_acceleration_limit):
        values = (*baseline, *forward_attitude, lateral_velocity, yaw, dt,
                  max_tilt, lateral_acceleration_limit)
        if (not all(math.isfinite(v) for v in values) or baseline[2] <= 0
                or not 0 < max_tilt < math.pi/2 or lateral_acceleration_limit < 0):
            raise ValueError('invalid straight retreat feedback')
        base_roll = euler_from_quaternion(attitude_from_axis_yaw(baseline, yaw))[0]
        if self.roll is None:
            self.roll = base_roll
        # Positive leftward speed needs rightward acceleration (positive roll
        # in ROS FLU/ENU). No lateral integral is accumulated.
        acceleration = clamp(self.lateral_kp*lateral_velocity,
                             -lateral_acceleration_limit, lateral_acceleration_limit)
        correction = clamp(math.atan2(acceleration, 9.80665),
                           -self.roll_correction, self.roll_correction)
        desired = clamp(base_roll+correction, -max_tilt, max_tilt)
        step = self.roll_rate*min(.05, max(0., dt))
        self.roll += clamp(desired-self.roll, -step, step)
        pitch = euler_from_quaternion(forward_attitude)[1]
        # Allocate the remaining cone to pitch without accelerating a roll
        # return from an entry outside the normal cone. The final mux retains
        # its existing independent safety projection and slew limits.
        cone = max(max_tilt, abs(self.roll))
        pitch_limit = math.acos(clamp(math.cos(cone)/math.cos(self.roll), -1., 1.))
        pitch = clamp(pitch, -pitch_limit, pitch_limit)
        local = (math.sin(pitch)*math.cos(self.roll), -math.sin(self.roll),
                 math.cos(pitch)*math.cos(self.roll))
        axis = (math.cos(yaw)*local[0]-math.sin(yaw)*local[1],
                math.sin(yaw)*local[0]+math.cos(yaw)*local[1], local[2])
        return attitude_from_axis_yaw(axis, yaw)
