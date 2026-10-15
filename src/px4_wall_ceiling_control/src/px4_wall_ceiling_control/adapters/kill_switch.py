"""PX4 kill-switch decoding from confirmed FCU parameters; no stick mapping guesses."""
import json
import math
from px4_wall_ceiling_control.freshness import is_fresh_receipt


class KillSwitchConfig:
    def __init__(self, channel, threshold=0.75, minimum=1000, maximum=2000, reverse=1):
        self.channel = int(channel)
        self.threshold, self.minimum, self.maximum, self.reverse = threshold, minimum, maximum, reverse
        if not 0 <= self.channel <= 18 or not all(math.isfinite(v) for v in (threshold, minimum, maximum, reverse)):
            raise ValueError("invalid kill-switch calibration")
        if abs(threshold) > 1 or maximum <= minimum or reverse not in (-1, 1):
            raise ValueError("invalid kill-switch calibration")

    def report(self, channels, fresh):
        value = {"valid": self.channel == 0, "active": False, "channel": self.channel}
        if self.channel == 0:
            return value
        if not fresh or len(channels) < self.channel:
            return value
        raw = channels[self.channel - 1]
        if not self.minimum - 100 <= raw <= self.maximum + 100:
            return value
        fraction = max(0.0, min(1.0, (raw - self.minimum) / (self.maximum - self.minimum)))
        if self.reverse < 0:
            fraction = 1 - fraction
        value.update(valid=True, active=(fraction > self.threshold if self.threshold >= 0 else fraction < -self.threshold))
        return value


def install_kill_guard(node, rospy, String):
    node.kill_guard_enabled = bool(rospy.get_param("~kill_guard_enabled", False))
    node.kill_status, node.kill_status_at = None, None
    def receive(message):
        try:
            value = json.loads(message.data)
            node.kill_status = value if isinstance(value, dict) else None
            node.kill_status_at = rospy.Time.now()
        except (ValueError, TypeError):
            node.kill_status = None
    if node.kill_guard_enabled:
        node.kill_subscription = rospy.Subscriber("/attachment/kill_switch", String, receive, queue_size=1)


def kill_clear(node, now):
    if not getattr(node, "kill_guard_enabled", False):
        return True
    at, status = node.kill_status_at, node.kill_status
    return bool(is_fresh_receipt(at, 0.35, now) and
                status and status.get("valid") is True and status.get("active") is False)
