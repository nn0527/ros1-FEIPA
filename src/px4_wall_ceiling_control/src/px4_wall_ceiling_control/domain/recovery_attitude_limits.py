"""Final recovery target cone and separate thrust-axis/yaw slew limits.

Horizontal command is a normalized thrust proxy, not force or acceleration.
"""
import math
from px4_wall_ceiling_control.domain.geometry import (
    quaternion_normalize, quaternion_multiply, euler_from_quaternion, rotate_vector, clamp)


def axis_yaw(q):
    q = quaternion_normalize(q)
    return rotate_vector(q, (0.0, 0.0, 1.0)), euler_from_quaternion(q)[2]


def cone_axis(axis, limit):
    horizontal = math.hypot(axis[0], axis[1])
    tilt = math.atan2(horizontal, axis[2])
    if tilt <= limit:
        return axis
    if horizontal < 1e-9:
        return (math.sin(limit), 0.0, math.cos(limit))
    return (axis[0] / horizontal * math.sin(limit),
            axis[1] / horizontal * math.sin(limit), math.cos(limit))


def attitude_from_axis_yaw(axis, yaw):
    # Reconstruct Euler heading with the exact world-frame thrust axis.
    x = math.cos(yaw)*axis[0] + math.sin(yaw)*axis[1]
    y = -math.sin(yaw)*axis[0] + math.cos(yaw)*axis[1]
    roll = math.asin(clamp(-y, -1.0, 1.0))
    pitch = math.atan2(x, axis[2])
    qr = (math.sin(roll/2), 0.0, 0.0, math.cos(roll/2))
    qp = (0.0, math.sin(pitch/2), 0.0, math.cos(pitch/2))
    qy = (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2))
    return quaternion_normalize(quaternion_multiply(qy, quaternion_multiply(qp, qr)))


def limit_recovery_attitude(desired, previous, dt, max_tilt, tilt_rate, yaw_rate):
    """Return final q, whether entry/history needed an immediate cone projection."""
    axis, yaw = axis_yaw(desired)
    axis = cone_axis(axis, max_tilt)
    if previous is None:
        return attitude_from_axis_yaw(axis, yaw), False
    prev_axis, prev_yaw = axis_yaw(previous)
    projected = math.acos(clamp(prev_axis[2], -1.0, 1.0)) > max_tilt + 1e-9
    prev_axis = cone_axis(prev_axis, max_tilt)
    distance = math.acos(clamp(sum(a*b for a,b in zip(prev_axis, axis)), -1.0, 1.0))
    step = tilt_rate * max(0.0, dt)
    if distance > step and distance > 1e-9:
        amount = step / distance
        sa = math.sin((1-amount)*distance)/math.sin(distance)
        sb = math.sin(amount*distance)/math.sin(distance)
        axis = tuple(sa*a+sb*b for a,b in zip(prev_axis, axis))
    error = math.atan2(math.sin(yaw-prev_yaw), math.cos(yaw-prev_yaw))
    yaw = prev_yaw + clamp(error, -yaw_rate*max(0.0,dt), yaw_rate*max(0.0,dt))
    return attitude_from_axis_yaw(cone_axis(axis, max_tilt), yaw), projected
