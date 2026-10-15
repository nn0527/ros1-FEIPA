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


def is_fresh_source(stamp, timeout_s, now=None, future_tolerance_s=0.1):
    """Validate a *source* timestamp separately from local receipt freshness.

    MAVROS maps FCU sample times into ROS time; a bounded mapping lead is
    allowed, but missing/zero stamps never authorize feedback. Locally
    generated commands must pass future_tolerance_s=0.0. This helper does not
    prove clock synchronization or sample uniqueness; callers also reject
    duplicate/out-of-order samples and retain independent receipt timeouts.
    """
    if stamp is None:
        return False
    try:
        source = stamp.to_sec()
        current = (now if now is not None else rospy.Time.now()).to_sec()
        timeout = float(timeout_s)
        future = float(future_tolerance_s)
        if not all(math.isfinite(v) for v in (source, current, timeout, future)):
            return False
        if source <= 0.0 or timeout < 0.0 or future < 0.0:
            return False
        # A callback can deliver a sample during a timer whose 'now' was read
        # slightly earlier. Compare against the live clock before declaring a
        # future sample; do not extend the configured source-age limit.
        if source > current:
            try:
                current = max(current, rospy.Time.now().to_sec())
            except rospy.exceptions.ROSInitException:
                pass
        age = current - source
        return math.isfinite(age) and -future <= age <= timeout
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False


def is_valid_range(message):
    """Validate a sensor_msgs/Range without treating +/-inf as a command."""
    if message is None or not math.isfinite(message.range):
        return False
    if message.max_range > message.min_range:
        return message.min_range <= message.range <= message.max_range
    return message.range > 0.0


def clamp(value, lower, upper):
    return max(lower, min(upper, value))
