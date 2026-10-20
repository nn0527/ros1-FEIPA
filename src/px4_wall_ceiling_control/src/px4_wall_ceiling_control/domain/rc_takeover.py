"""Rate bounded transition from measured attitude to live pilot targets."""
import math
from px4_wall_ceiling_control.domain.geometry import (
    quaternion_angle, quaternion_slerp, quaternion_multiply, quaternion_normalize,
    rotate_vector,
)


class RCTakeover:
    def __init__(self, attitude, thrust, attitude_rate_deg_s=20., thrust_rate=.15):
        values = tuple(attitude) + (thrust, attitude_rate_deg_s, thrust_rate)
        if (not all(math.isfinite(v) for v in values)
                or sum(v*v for v in attitude) < 1e-9
                or not 0 <= thrust <= 1 or attitude_rate_deg_s <= 0 or thrust_rate <= 0):
            raise ValueError('invalid takeover seed or rate')
        self.attitude = quaternion_normalize(attitude)
        self.thrust = thrust
        # At near-vertical pitch, Euler yaw is undefined. Project body Y
        # instead of letting signed zero choose opposite 180-degree headings.
        forward = rotate_vector(self.attitude, (1., 0., 0.))
        left = rotate_vector(self.attitude, (0., 1., 0.))
        self.yaw = (math.atan2(forward[1], forward[0]) if math.hypot(*forward[:2]) > 1e-6
                    else math.atan2(-left[0], left[1]))
        self.attitude_rate = math.radians(attitude_rate_deg_s)
        self.thrust_rate = thrust_rate
        self.elapsed = 0.

    def step(self, roll, pitch, yaw_rate, thrust, dt):
        if not all(math.isfinite(v) for v in (roll, pitch, yaw_rate, thrust, dt)):
            raise ValueError('nonfinite pilot target')
        # A delayed timer must never skip the transition.
        dt = min(.05, max(0., dt))
        self.elapsed += dt
        self.yaw += yaw_rate * dt
        qz = (0., 0., math.sin(self.yaw/2), math.cos(self.yaw/2))
        qy = (0., math.sin(pitch/2), 0., math.cos(pitch/2))
        qx = (math.sin(roll/2), 0., 0., math.cos(roll/2))
        desired = quaternion_multiply(quaternion_multiply(qz, qy), qx)
        angle = quaternion_angle(self.attitude, desired)
        self.attitude = quaternion_slerp(self.attitude, desired,
                                       min(1., self.attitude_rate * dt / max(angle, 1e-9)))
        desired_thrust = min(1., max(0., thrust))
        step = self.thrust_rate * dt
        self.thrust += min(step, max(-step, desired_thrust - self.thrust))
        return self.attitude, self.thrust, desired
