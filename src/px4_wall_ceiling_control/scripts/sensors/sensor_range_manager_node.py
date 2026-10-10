#!/usr/bin/env python3
"""Validate and independently distribute the ceiling and front ranges."""

import math
import json

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import String

from px4_wall_ceiling_control.freshness import is_fresh, is_valid_range
from px4_wall_ceiling_control.msg import SensorHealth


class SensorRangeManagerNode:
    """One input manager with independent, event-driven data outputs.

    The manager deliberately has no knowledge of operator mode or attachment
    output ownership.  Decisions may observe the sensors they need, while the
    downstream output mux remains the only control authority arbiter.
    """

    def __init__(self):
        self.input_timeout_s = float(rospy.get_param("~input_timeout_s", 0.30))
        health_rate_hz = float(rospy.get_param("~health_publish_rate_hz", 10.0))

        self.ceiling_received_at = None
        self.front_received_at = None
        self.ceiling_latest_valid = False
        self.front_latest_valid = False
        self.ceiling_sequence = 0
        self.front_sequence = 0
        self.ceiling_invalid_count = 0
        self.front_invalid_count = 0
        self.input_errors = {}
        self.input_received_at = {}
        self.driver_errors = {}

        self.ceiling_publisher = rospy.Publisher(
            rospy.get_param("~ceiling_output_topic", "/sensor/ceiling/range"),
            Range,
            queue_size=10,
        )
        self.front_publisher = rospy.Publisher(
            rospy.get_param("~front_output_topic", "/sensor/front/range"),
            Range,
            queue_size=10,
        )
        self.health_publisher = rospy.Publisher(
            rospy.get_param("~health_topic", "/sensor/health"),
            SensorHealth,
            queue_size=10,
            latch=True,
        )

        rospy.Subscriber(
            rospy.get_param("~ceiling_input_topic", "/sensor/ceiling/raw"),
            Range,
            self._on_ceiling,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~front_input_topic", "/sensor/front/raw"),
            Range,
            self._on_front,
            queue_size=10,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(health_rate_hz, 1.0)), self._on_timer
        )
        for channel in ("front", "ceiling"):
            rospy.Subscriber(
                rospy.get_param("~{}_diagnostic_topic".format(channel),
                                "/sensor/{}/diagnostic".format(channel)),
                String, lambda msg, key=channel: self._on_diagnostic(key, msg),
                queue_size=10,
            )

    def _record_input_error(self, channel, message, valid, now):
        # getattr preserves compatibility with minimal test fixtures.
        if not hasattr(self, "input_errors"):
            self.input_errors, self.input_received_at, self.driver_errors = {}, {}, {}
        self.input_received_at[channel] = now
        if valid:
            self.input_errors.pop(channel, None)
            self.driver_errors.pop(channel, None)
        elif not math.isfinite(message.range):
            self.input_errors[channel] = (-3, "nonfinite", "distance is not finite")
        else:
            self.input_errors[channel] = (-4, "out_of_range", "distance {} m outside [{}, {}] m".format(message.range, message.min_range, message.max_range))

    def _on_diagnostic(self, channel, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict) or not isinstance(data.get("valid"), bool):
                raise ValueError("diagnostic requires boolean valid")
            if data["valid"]:
                self.driver_errors.pop(channel, None)
                return
            code = int(data.get("error_code", -5))
            kind = str(data.get("error_type", "parse"))
            reason = str(data.get("error", "invalid lidar frame"))
        except (ValueError, TypeError, OverflowError) as exc:
            code, kind, reason = -9, "diagnostic", "invalid diagnostic: {}".format(exc)
        self.driver_errors[channel] = (code, kind, reason, rospy.Time.now())
        setattr(self, channel + "_latest_valid", False)

    def _channel_error(self, channel, valid, now):
        if valid:
            return 0, "", ""
        driver = getattr(self, "driver_errors", {}).get(channel)
        if driver and (driver[1] in ("serial", "configuration", "driver")
                       or is_fresh(driver[3], self.input_timeout_s, now)):
            return driver[:3]
        received = getattr(self, "input_received_at", {}).get(channel)
        if received is None:
            return -2, "no_data", "no range received since startup"
        if not is_fresh(received, self.input_timeout_s, now):
            return -1, "timeout", "no range received for {:.2f} s".format(self._age(received, now))
        return getattr(self, "input_errors", {}).get(channel, (-1, "timeout", "last valid range expired"))

    def _on_ceiling(self, message):
        now = rospy.Time.now()
        self.ceiling_latest_valid = is_valid_range(message)
        self._record_input_error("ceiling", message, self.ceiling_latest_valid, now)
        if not self.ceiling_latest_valid:
            self.ceiling_invalid_count += 1
            rospy.logwarn_throttle(1.0, "invalid ceiling range input")
            return
        self.ceiling_received_at = now
        self.ceiling_sequence += 1
        self.ceiling_publisher.publish(message)

    def _on_front(self, message):
        now = rospy.Time.now()
        self.front_latest_valid = is_valid_range(message)
        self._record_input_error("front", message, self.front_latest_valid, now)
        if not self.front_latest_valid:
            self.front_invalid_count += 1
            rospy.logwarn_throttle(1.0, "invalid front range input")
            return
        self.front_received_at = now
        self.front_sequence += 1
        self.front_publisher.publish(message)

    def _channel_valid(self, received_at, latest_valid, now):
        return bool(
            latest_valid and is_fresh(received_at, self.input_timeout_s, now)
        )

    @staticmethod
    def _age(received_at, now):
        if received_at is None:
            return math.inf
        return max(0.0, (now - received_at).to_sec())

    def _build_health(self, now):
        status = SensorHealth()
        status.header.stamp = now
        status.ceiling_valid = self._channel_valid(
            self.ceiling_received_at, self.ceiling_latest_valid, now
        )
        status.front_valid = self._channel_valid(
            self.front_received_at, self.front_latest_valid, now
        )
        status.ceiling_age_s = self._age(self.ceiling_received_at, now)
        status.front_age_s = self._age(self.front_received_at, now)
        status.ceiling_sequence = self.ceiling_sequence
        status.front_sequence = self.front_sequence
        status.ceiling_invalid_count = self.ceiling_invalid_count
        status.front_invalid_count = self.front_invalid_count
        for channel in ("front", "ceiling"):
            code, kind, reason = self._channel_error(channel, getattr(status, channel + "_valid"), now)
            setattr(status, channel + "_error_code", code)
            setattr(status, channel + "_error_type", kind)
            setattr(status, channel + "_error_reason", reason)
        return status

    def _on_timer(self, _event):
        self.health_publisher.publish(self._build_health(rospy.Time.now()))


if __name__ == "__main__":
    rospy.init_node("sensor_range_manager_node")
    SensorRangeManagerNode()
    rospy.spin()
