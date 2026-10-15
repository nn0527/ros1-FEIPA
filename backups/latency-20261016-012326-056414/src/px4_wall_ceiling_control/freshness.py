"""Small, side-effect-free validation helpers shared by runtime nodes."""

import math

import rospy


def is_fresh(received_at, timeout_s, now=None):
    """Return True only when a callback receipt time is recent."""
    if received_at is None:
        return False
    current = now if now is not None else rospy.Time.now()
    age = (current - received_at).to_sec()
    return 0.0 <= age <= timeout_s


def is_fresh_receipt(received_at, timeout_s, now=None):
    """Local callback receipts may advance while a control tick uses its old now.

    Re-read ROS time only for that negative-age race. Future timestamps relative
    to the live clock remain invalid; ordinary age/timeouts are never extended.
    Do not use this helper as permission to accept an external future timestamp.
    """
    if received_at is None:
        return False
    current = now if now is not None else rospy.Time.now()
    if (current - received_at).to_sec() < 0.0:
        try:
            current = rospy.Time.now()
        except rospy.exceptions.ROSInitException:
            return False
    return is_fresh(received_at, timeout_s, current)


def is_valid_range(message):
    """Validate a sensor_msgs/Range without treating +/-inf as a command."""
    if message is None or not math.isfinite(message.range):
        return False
    if message.max_range > message.min_range:
        return message.min_range <= message.range <= message.max_range
    return message.range > 0.0


def clamp(value, lower, upper):
    return max(lower, min(upper, value))

