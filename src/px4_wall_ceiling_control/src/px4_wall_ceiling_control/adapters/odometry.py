"""Frame-explicit conversion and validation for ``nav_msgs/Odometry``."""

import math

from px4_wall_ceiling_control.domain.geometry import rotate_vector


def quaternion_tuple(odometry):
    orientation = odometry.pose.pose.orientation
    return (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )


def odometry_is_finite(odometry):
    if odometry is None:
        return False
    position = odometry.pose.pose.position
    linear = odometry.twist.twist.linear
    angular = odometry.twist.twist.angular
    quaternion = quaternion_tuple(odometry)
    values = (
        position.x,
        position.y,
        position.z,
        *quaternion,
        linear.x,
        linear.y,
        linear.z,
        angular.x,
        angular.y,
        angular.z,
    )
    return bool(
        all(math.isfinite(value) for value in values)
        and math.sqrt(sum(value * value for value in quaternion)) > 1.0e-6
    )


def linear_velocity_parent_frame(odometry, twist_in_child_frame=True):
    """Return linear velocity in the odometry parent/world frame.

    ``nav_msgs/Odometry`` defines twist in ``child_frame_id``. MAVROS commonly
    publishes a body child frame, so the pose orientation rotates this vector
    into the ENU parent frame. A compatibility flag supports sources that
    already publish world-frame twist despite the message convention.
    """

    linear = odometry.twist.twist.linear
    vector = (linear.x, linear.y, linear.z)
    if not twist_in_child_frame:
        return vector
    return rotate_vector(quaternion_tuple(odometry), vector)


def odometry_attitude_horizontal_is_finite(odometry, twist_in_child_frame=True):
    """Validate only fields used by attitude/rate and horizontal control.

    Body-frame z velocity contributes to world x/y under tilt and is therefore
    still needed for that transform. World-frame vertical velocity and local
    position Z have no role in the range height controller.
    """
    if odometry is None:
        return False
    q = quaternion_tuple(odometry)
    a, v = odometry.twist.twist.angular, odometry.twist.twist.linear
    values = (*q, a.x, a.y, a.z, v.x, v.y)
    if twist_in_child_frame:
        values += (v.z,)
    return all(math.isfinite(x) for x in values) and abs(sum(x*x for x in q)-1.) <= .01


def vertical_speed_enu_mps(odometry, twist_in_child_frame=True):
    return linear_velocity_parent_frame(odometry, twist_in_child_frame)[2]

