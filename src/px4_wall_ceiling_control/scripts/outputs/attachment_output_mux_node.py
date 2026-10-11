#!/usr/bin/env python3
"""Single MAVROS attitude publisher with bounded, direct RC stick trim."""

import math
import time

import rospy
from geometry_msgs.msg import Quaternion, Vector3Stamped
from mavros_msgs.msg import AttitudeTarget, RCIn, State
from mavros_msgs.srv import ParamGet
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, UInt8, String
from px4_wall_ceiling_control.adapters.kill_switch import install_kill_guard, kill_clear

from px4_wall_ceiling_control.adapters.mavros_setpoint import MavrosSetpointBuilder
from px4_wall_ceiling_control.adapters.odometry import (
    odometry_is_finite,
    vertical_speed_enu_mps,
)
from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion, quaternion_multiply, quaternion_normalize
from px4_wall_ceiling_control.domain.owner_arbiter import (
    CandidateIntent,
    OWNER_CEILING,
    OWNER_NONE,
    OWNER_WALL,
    OwnerArbiter,
)
from px4_wall_ceiling_control.domain.rc_attitude_assist import (
    slew,
    stick_fraction,
    trim_attitude,
)
from px4_wall_ceiling_control.domain.recovery_attitude_limits import limit_recovery_attitude, axis_yaw
from px4_wall_ceiling_control.domain.safety import flight_state_ready
from px4_wall_ceiling_control.freshness import is_fresh
from px4_wall_ceiling_control.msg import (
    AttachmentControlCandidate,
    CeilingAttachmentStatus,
    OperatorCommand,
    WallPerchStatus,
)


class AttachmentOutputMuxNode:
    OWNER_NONE = OWNER_NONE
    OWNER_CEILING = OWNER_CEILING
    OWNER_WALL = OWNER_WALL

    OUTPUT_NONE = 0
    OUTPUT_ATTITUDE_THRUST = 2

    ATTITUDE_AND_THRUST_MASK = MavrosSetpointBuilder.ATTITUDE_AND_THRUST_MASK

    def __init__(self):
        install_kill_guard(self, rospy, String)
        self.output_enabled = bool(rospy.get_param("~output_enabled", False))
        self.ceiling_output_enabled = bool(
            rospy.get_param("~ceiling_output_enabled", False)
        )
        self.wall_output_enabled = bool(
            rospy.get_param("~wall_output_enabled", False)
        )
        self.require_connected = bool(rospy.get_param("~require_connected", True))
        self.require_armed = bool(rospy.get_param("~require_armed", True))
        self.require_offboard = bool(rospy.get_param("~require_offboard", True))
        self.ceiling_candidate_timeout_s = float(
            rospy.get_param("~ceiling_candidate_timeout_s", 0.20)
        )
        self.wall_candidate_timeout_s = float(
            rospy.get_param("~wall_candidate_timeout_s", 0.20)
        )
        self.state_timeout_s = float(rospy.get_param("~state_timeout_s", 1.50))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.50))
        self.operator_timeout_s = float(
            rospy.get_param("~operator_timeout_s", 0.50)
        )
        self.handoff_dwell_s = float(
            rospy.get_param("~handoff_dwell_s", 0.50)
        )
        self.max_velocity_xy_mps = float(
            rospy.get_param("~max_velocity_xy_mps", 0.30)
        )
        self.max_velocity_z_mps = float(
            rospy.get_param("~max_velocity_z_mps", 0.30)
        )
        self.min_thrust = float(rospy.get_param("~min_thrust", 0.10))
        self.max_thrust = float(rospy.get_param("~max_thrust", 0.85))
        self.local_frame_id = rospy.get_param("~local_frame_id", "map")
        self.hover_thrust = float(rospy.get_param("~hover_thrust", 0.50))
        self.vertical_speed_kp = float(rospy.get_param("~vertical_speed_kp", 0.15))
        self.odom_twist_in_child_frame = bool(
            rospy.get_param("~odom_twist_in_child_frame", True)
        )
        self.rc_assist_enabled = bool(rospy.get_param("~rc_assist_enabled", False))
        self.rc_roll_channel_index = int(rospy.get_param("~rc_roll_channel_index", -1))
        self.rc_pitch_channel_index = int(rospy.get_param("~rc_pitch_channel_index", -1))
        self.rc_pwm_low = int(rospy.get_param("~rc_pwm_low", 1000))
        self.rc_pwm_center = int(rospy.get_param("~rc_pwm_center", 1500))
        self.rc_pwm_high = int(rospy.get_param("~rc_pwm_high", 2000))
        self.rc_deadzone = float(rospy.get_param("~rc_deadzone", 0.06))
        self.rc_roll_reverse = bool(rospy.get_param("~rc_roll_reverse", False))
        self.rc_pitch_reverse = bool(rospy.get_param("~rc_pitch_reverse", False))
        self.rc_timeout_s = float(rospy.get_param("~rc_timeout_s", 0.20))
        self.rc_slew_deg_s = float(rospy.get_param("~rc_slew_deg_s", 90.0))
        self.rc_free_max_angle_deg = float(
            rospy.get_param("~rc_free_max_angle_deg", 8.0)
        )
        self.rc_contact_max_angle_deg = float(
            rospy.get_param("~rc_contact_max_angle_deg", 2.0)
        )
        self.rc_wall_detach_roll_deg = float(
            rospy.get_param("~rc_wall_detach_roll_deg", 2.0)
        )
        self.rc_wall_detach_pitch_deg = float(
            rospy.get_param("~rc_wall_detach_pitch_deg", 0.0)
        )
        self.rc_roll_yaw_enabled = bool(rospy.get_param("~rc_roll_yaw_enabled", False))
        self.rc_yaw_max_deg = float(rospy.get_param("~rc_yaw_max_deg", 15.0))
        self.rc_yaw_contact_deg = float(rospy.get_param("~rc_yaw_contact_deg", 3.0))
        self.rc_yaw_detach_deg = float(rospy.get_param("~rc_yaw_detach_deg", 5.0))
        self.rc_yaw_slew_deg_s = float(rospy.get_param("~rc_yaw_slew_deg_s", 20.0))
        self.recovery_attitude_limits_enabled = bool(rospy.get_param("~recovery_attitude_limits_enabled", False))
        self.recovery_max_tilt_deg = float(rospy.get_param("~recovery_max_tilt_deg", 5.0))
        self.recovery_tilt_slew_deg_s = float(rospy.get_param("~recovery_tilt_slew_deg_s", 10.0))
        self.recovery_yaw_slew_deg_s = float(rospy.get_param("~recovery_yaw_slew_deg_s", 20.0))
        self.recovery_final_thrust_cap = float(rospy.get_param("~recovery_final_thrust_cap", 0.60))
        if not (0 < self.recovery_max_tilt_deg < 45 and 0 < self.recovery_tilt_slew_deg_s <= 90
                and 0 < self.recovery_yaw_slew_deg_s <= 90 and 0.1 <= self.recovery_final_thrust_cap <= 1.0):
            raise ValueError("invalid final recovery attitude limits")
        self.approach_attitude_limits_enabled = bool(rospy.get_param("~approach_attitude_limits_enabled", False))
        self.approach_max_tilt_deg = float(rospy.get_param("~approach_max_tilt_deg", 5.0))
        self.approach_tilt_slew_deg_s = float(rospy.get_param("~approach_tilt_slew_deg_s", 10.0))
        self.approach_yaw_slew_deg_s = float(rospy.get_param("~approach_yaw_slew_deg_s", 20.0))
        if not (0 < self.approach_max_tilt_deg <= 5 and 0 < self.approach_tilt_slew_deg_s <= 10
                and 0 < self.approach_yaw_slew_deg_s <= 20):
            raise ValueError("invalid final automatic approach attitude limits")
        self.approach_previous_target = self.approach_previous_at = None
        self.approach_target_publisher = rospy.Publisher("/attachment/approach_target_deg", Vector3Stamped, queue_size=10)
        self.recovery_previous_target = None
        self.recovery_previous_at = None
        self.recovery_target_publisher = rospy.Publisher("/attachment/recovery_target_deg", Vector3Stamped, queue_size=10)
        self.recovery_horizontal_publisher = rospy.Publisher("/attachment/recovery_horizontal_command", Float32, queue_size=10)
        self.preflip_right_roll_deg = float(rospy.get_param("~preflip_right_roll_deg", 0.0))
        self.preflip_roll_slew_deg_s = float(rospy.get_param("~preflip_roll_slew_deg_s", 5.0))
        if not (math.isfinite(self.preflip_right_roll_deg) and 0 <= self.preflip_right_roll_deg <= 5
                and math.isfinite(self.preflip_roll_slew_deg_s) and 0 < self.preflip_roll_slew_deg_s <= 10):
            raise ValueError("invalid preflip right-roll compensation")
        self.preflip_bias_rad = 0.0
        self.preflip_bias_at = None
        self.preflip_bias_publisher = rospy.Publisher("/attachment/preflip_roll_bias_deg", Float32, queue_size=10)
        self.rc_yaw_rad = 0.0
        self.rc_calibration = {}
        if self.rc_roll_yaw_enabled:
            if not self.rc_assist_enabled:
                raise ValueError("Roll/yaw correction requires rc_assist_enabled")
            if not (0 <= self.rc_yaw_contact_deg <= self.rc_yaw_detach_deg <= self.rc_yaw_max_deg <= 30
                    and 0 < self.rc_yaw_slew_deg_s <= 45):
                raise ValueError("RC yaw correction limits are invalid")
            self._load_rc_roll_yaw_calibration()
        self.status_timeout_s = float(rospy.get_param("~status_timeout_s", 0.30))
        self.match_fcu_thrust_on_handover = bool(
            rospy.get_param("~match_fcu_thrust_on_handover", False)
        )
        self.fcu_target_timeout_s = float(
            rospy.get_param("~fcu_target_timeout_s", 0.20)
        )
        self.thrust_handover_ramp_s = float(
            rospy.get_param("~thrust_handover_ramp_s", 1.0)
        )
        if self.rc_assist_enabled and (
            self.rc_roll_channel_index < 0
            or self.rc_pitch_channel_index < 0
            or self.rc_roll_channel_index == self.rc_pitch_channel_index
            or not self.rc_pwm_low < self.rc_pwm_center < self.rc_pwm_high
            or not 0.0 <= self.rc_deadzone < 1.0
            or self.rc_slew_deg_s <= 0.0
            or not 0.0 <= self.rc_contact_max_angle_deg <= self.rc_free_max_angle_deg <= 15.0
            or not 0.0 <= self.rc_wall_detach_roll_deg <= self.rc_free_max_angle_deg
            or not 0.0 <= self.rc_wall_detach_pitch_deg <= self.rc_free_max_angle_deg
        ):
            raise ValueError("RC assist channels or limits are invalid")
        if not self.min_thrust <= self.hover_thrust <= self.max_thrust:
            raise ValueError("hover_thrust must lie within output thrust limits")
        if self.vertical_speed_kp < 0.0:
            raise ValueError("vertical_speed_kp must be nonnegative")
        if self.fcu_target_timeout_s <= 0.0:
            raise ValueError("fcu_target_timeout_s must be positive")
        if self.thrust_handover_ramp_s <= 0.0:
            raise ValueError("thrust_handover_ramp_s must be positive")

        self.ceiling_candidate = None
        self.ceiling_candidate_at = None
        self.wall_candidate = None
        self.wall_candidate_at = None
        self.flight_state = None
        self.flight_state_at = None
        self.odom = None
        self.odom_at = None
        self.rc_channels = None
        self.rc_at = None
        self.rc_roll_rad = 0.0
        self.rc_pitch_rad = 0.0
        self.rc_trim_at = None
        self.ceiling_status = None
        self.ceiling_status_at = None
        self.wall_status = None
        self.wall_status_at = None
        self.operator = None
        self.operator_at = None
        self.prestream_requested = False
        self.prestream_at = None
        self.prestream_timeout_s = float(rospy.get_param("~prestream_timeout_s", 0.30))
        self.fcu_attitude_target = None
        self.fcu_attitude_target_at = None
        self.handover_start_thrust = None
        self.handover_started_at = None
        self.owner = self.OWNER_NONE
        self.interlock_active = False
        self.interlock_clear_since = None
        self.owner_arbiter = OwnerArbiter(self.handoff_dwell_s)
        self.setpoint_builder = MavrosSetpointBuilder(
            self.max_velocity_z_mps,
            self.min_thrust,
            self.max_thrust,
            self.local_frame_id,
        )

        self.attitude_publisher = rospy.Publisher(
            rospy.get_param(
                "~attitude_setpoint_topic", "/mavros/setpoint_raw/attitude"
            ),
            AttitudeTarget,
            queue_size=10,
        )
        self.owner_publisher = rospy.Publisher(
            rospy.get_param("~owner_topic", "/attachment/output_owner"),
            UInt8,
            queue_size=10,
            latch=True,
        )
        self.mode_publisher = rospy.Publisher(
            rospy.get_param("~output_mode_topic", "/attachment/output_mode"),
            UInt8,
            queue_size=10,
            latch=True,
        )
        self.rc_age_publisher = rospy.Publisher(
            rospy.get_param("~rc_age_topic", "/attachment/rc_input_age_ms"),
            Float32,
            queue_size=1,
        )
        self.rc_preview_publisher = rospy.Publisher(
            rospy.get_param("~rc_preview_topic", "/attachment/rc_preview_deg"),
            Vector3Stamped,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~ceiling_candidate_topic",
                "/ceiling_attachment/control_candidate",
            ),
            AttachmentControlCandidate,
            self._on_ceiling_candidate,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~operator_topic", "/operator/command"),
            OperatorCommand,
            self._on_operator,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~wall_candidate_topic", "/wall_perch/control_candidate"
            ),
            AttachmentControlCandidate,
            self._on_wall_candidate,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~state_topic", "/mavros/state"),
            State,
            self._on_state,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~odom_topic", "/mavros/local_position/odom"),
            Odometry,
            self._on_odom,
            queue_size=10,
        )
        if self.match_fcu_thrust_on_handover:
            rospy.Subscriber(
                rospy.get_param(
                    "~fcu_attitude_target_topic",
                    "/mavros/setpoint_raw/target_attitude",
                ),
                AttitudeTarget,
                self._on_fcu_attitude_target,
                queue_size=10,
            )
        if self.rc_assist_enabled:
            rospy.Subscriber(
                rospy.get_param("~rc_topic", "/mavros/rc/in"),
                RCIn,
                self._on_rc,
                queue_size=1,
            )
            rospy.Subscriber(
                rospy.get_param("~ceiling_status_topic", "/ceiling_attachment/status"),
                CeilingAttachmentStatus,
                self._on_ceiling_status,
                queue_size=1,
            )
            rospy.Subscriber(
                rospy.get_param("~wall_status_topic", "/wall_perch/status"),
                WallPerchStatus,
                self._on_wall_status,
                queue_size=1,
            )
        rospy.Subscriber(
            rospy.get_param("~prestream_topic", "/attachment/prestream"),
            Bool,
            self._on_prestream,
            queue_size=10,
        )
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 50.0))
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(publish_rate_hz, 2.0)), self._on_timer
        )

        if not self.output_enabled:
            rospy.logwarn("combined attachment MAVROS output is disabled")

    def _on_ceiling_candidate(self, message):
        if message.source != AttachmentControlCandidate.SOURCE_CEILING:
            rospy.logerr_throttle(1.0, "invalid source on ceiling candidate topic")
            return
        self.ceiling_candidate = message
        self.ceiling_candidate_at = rospy.Time.now()

    def _on_wall_candidate(self, message):
        if message.source != AttachmentControlCandidate.SOURCE_WALL:
            rospy.logerr_throttle(1.0, "invalid source on wall candidate topic")
            return
        self.wall_candidate = message
        self.wall_candidate_at = rospy.Time.now()

    def _on_state(self, message):
        now = rospy.Time.now()
        previous_mode = (
            self.flight_state.mode.upper() if self.flight_state is not None else ""
        )
        next_mode = message.mode.upper()
        if getattr(self, "match_fcu_thrust_on_handover", False):
            if previous_mode != "OFFBOARD" and next_mode == "OFFBOARD":
                if self._handover_source_ready(now):
                    self.handover_start_thrust = float(
                        self.fcu_attitude_target.thrust
                    )
                    # Start the ramp only when the behavior candidate first owns
                    # the output. Until then prestream holds this exact thrust.
                    self.handover_started_at = None
                    rospy.loginfo(
                        "captured ALTCTL handover thrust %.3f",
                        self.handover_start_thrust,
                    )
                else:
                    self.handover_start_thrust = None
                    self.handover_started_at = None
                    rospy.logerr(
                        "OFFBOARD entered without a fresh bounded FCU thrust target"
                    )
            elif next_mode != "OFFBOARD":
                self.handover_start_thrust = None
                self.handover_started_at = None
        self.flight_state = message
        self.flight_state_at = now

    def _on_fcu_attitude_target(self, message):
        # Once OFFBOARD is active this topic reflects the external command.
        # Keep the last ALTCTL value instead of feeding our output back as input.
        if self.flight_state is not None and self.flight_state.mode.upper() == "OFFBOARD":
            return
        self.fcu_attitude_target = message
        self.fcu_attitude_target_at = rospy.Time.now()

    def _on_odom(self, message):
        self.odom = message
        self.odom_at = rospy.Time.now()

    def _load_rc_roll_yaw_calibration(self):
        # Fail startup rather than guess stick channels or direction.
        rospy.wait_for_service("/mavros/param/get", timeout=5.0)
        get = rospy.ServiceProxy("/mavros/param/get", ParamGet)
        deadline = time.monotonic() + 20.0
        def read(name, integer=False):
            while not rospy.is_shutdown():
                response = get(param_id=name)
                if response.success:
                    return response.value.integer if integer else response.value.real
                if time.monotonic() >= deadline:
                    raise ValueError("RC calibration unavailable after parameter-cache wait: " + name)
                rospy.logwarn_throttle(2.0, "Waiting for MAVROS RC parameter cache: %s", name)
                rospy.sleep(.25)
            raise ValueError("RC calibration canceled during shutdown")
        for axis in ("roll", "yaw"):
            channel = int(read("RC_MAP_" + axis.upper(), True))
            if channel <= 0:
                raise ValueError("RC " + axis + " channel is not mapped")
            low, center, high, reverse = (read("RC%d_%s" % (channel, suffix))
                                         for suffix in ("MIN", "TRIM", "MAX", "REV"))
            if not (all(math.isfinite(v) for v in (low, center, high, reverse))
                    and low < center < high and reverse in (-1, 1)):
                raise ValueError("Invalid RC " + axis + " calibration")
            self.rc_calibration[axis] = (channel - 1, low, center, high, reverse < 0)
        if self.rc_calibration["roll"][0] == self.rc_calibration["yaw"][0]:
            raise ValueError("Roll and yaw require distinct RC channels")
        self.rc_roll_channel_index = self.rc_calibration["roll"][0]
        # Legacy roll/pitch assist is disabled by this path; satisfy its validation.
        self.rc_pitch_channel_index = self.rc_calibration["yaw"][0]
        rospy.loginfo("RC roll/yaw correction: calibrated channels %d/%d; pitch and thrust unchanged",
                      self.rc_roll_channel_index + 1, self.rc_pitch_channel_index + 1)

    def _roll_yaw_fraction(self, axis):
        index, low, center, high, reverse = self.rc_calibration[axis]
        value = stick_fraction(self.rc_channels, index, low, center, high, self.rc_deadzone, reverse)
        # PX4 yaw is clockwise in NED; ROS heading is counterclockwise in ENU.
        return -value if axis == "yaw" and value is not None else value

    def _roll_yaw_limits(self, now, source):
        roll = math.radians(self.rc_free_max_angle_deg)
        yaw = math.radians(self.rc_yaw_max_deg)
        if source == AttachmentControlCandidate.SOURCE_WALL:
            if not is_fresh(self.wall_status_at, self.status_timeout_s, now):
                return 0.0, 0.0
            state = self.wall_status.state
            if state in (WallPerchStatus.STATE_FLIP_TO_WALL, WallPerchStatus.STATE_WALL_CAPTURE,
                         WallPerchStatus.STATE_WALL_HOLD, WallPerchStatus.STATE_WALL_PIN,
                         WallPerchStatus.STATE_DETACH_ROTATE):
                # The wall maneuver owns these axes; no RC trim during flip-to-detach.
                return 0.0, 0.0
        return roll, yaw

    def _roll_yaw_attitude(self, now, quaternion, source):
        # Do not change ALTCTL prestream targets or retain a trim across handovers.
        flight = getattr(self, "flight_state", None)
        if (flight is None or not is_fresh(self.flight_state_at, self.state_timeout_s, now)
                or flight.mode.upper() != "OFFBOARD"):
            self.rc_roll_rad = self.rc_yaw_rad = 0.0
            self.rc_trim_at = now
            return quaternion
        roll_limit, yaw_limit = self._roll_yaw_limits(now, source)
        ready = self._rc_ready(now)
        desired_roll = self._roll_yaw_fraction("roll") * roll_limit if ready else 0.0
        desired_yaw = self._roll_yaw_fraction("yaw") * yaw_limit if ready else 0.0
        if not ready:
            rospy.logwarn_throttle(1.0, "RC roll/yaw unavailable; correction returning to zero")
        self.rc_age_publisher.publish(Float32(data=(now-self.rc_at).to_sec()*1000 if ready else -1.0))
        elapsed = max(0.0, min((now-self.rc_trim_at).to_sec() if self.rc_trim_at is not None else .02, .1))
        self.rc_roll_rad = slew(max(-roll_limit, min(self.rc_roll_rad, roll_limit)), desired_roll,
                                math.radians(self.rc_slew_deg_s)*elapsed)
        self.rc_yaw_rad = slew(max(-yaw_limit, min(self.rc_yaw_rad, yaw_limit)), desired_yaw,
                               math.radians(self.rc_yaw_slew_deg_s)*elapsed)
        self.rc_pitch_rad = 0.0
        self.rc_trim_at = now
        # World-up yaw before body-axis roll avoids Euler singularities at the wall.
        yaw_q = (0, 0, math.sin(self.rc_yaw_rad/2), math.cos(self.rc_yaw_rad/2))
        values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        values = quaternion_normalize(quaternion_multiply(yaw_q, trim_attitude(values, self.rc_roll_rad, 0)))
        preview = Vector3Stamped();preview.header.stamp = now
        preview.vector.x = math.degrees(self.rc_roll_rad)
        preview.vector.y = 0.0;preview.vector.z = math.degrees(self.rc_yaw_rad)
        self.rc_preview_publisher.publish(preview)
        return Quaternion(x=values[0], y=values[1], z=values[2], w=values[3])

    def _on_rc(self, message):
        self.rc_channels = tuple(message.channels)
        self.rc_at = rospy.Time.now()
        if getattr(self, "rc_roll_yaw_enabled", False):
            return  # Publish applied correction from the output path, not raw sticks.
        roll = stick_fraction(
            self.rc_channels, self.rc_roll_channel_index, self.rc_pwm_low,
            self.rc_pwm_center, self.rc_pwm_high, self.rc_deadzone,
            self.rc_roll_reverse,
        )
        pitch = stick_fraction(
            self.rc_channels, self.rc_pitch_channel_index, self.rc_pwm_low,
            self.rc_pwm_center, self.rc_pwm_high, self.rc_deadzone,
            self.rc_pitch_reverse,
        )
        if roll is not None and pitch is not None:
            preview = Vector3Stamped()
            preview.header.stamp = self.rc_at
            preview.vector.x = roll * self.rc_free_max_angle_deg
            preview.vector.y = pitch * self.rc_free_max_angle_deg
            self.rc_preview_publisher.publish(preview)

    def _on_ceiling_status(self, message):
        self.ceiling_status = message
        self.ceiling_status_at = rospy.Time.now()

    def _on_wall_status(self, message):
        self.wall_status = message
        self.wall_status_at = rospy.Time.now()

    def _on_operator(self, message):
        self.operator = message
        self.operator_at = rospy.Time.now()

    def _on_prestream(self, message):
        self.prestream_requested = bool(message.data)
        self.prestream_at = rospy.Time.now()

    def _ceiling_fresh(self, now):
        return bool(
            getattr(self, "ceiling_output_enabled", False)
            and is_fresh(
                getattr(self, "ceiling_candidate_at", None),
                getattr(self, "ceiling_candidate_timeout_s", 0.20),
                now,
            )
            and getattr(self, "ceiling_candidate", None) is not None
        )

    def _wall_fresh(self, now):
        return bool(
            getattr(self, "wall_output_enabled", False)
            and is_fresh(
                getattr(self, "wall_candidate_at", None),
                getattr(self, "wall_candidate_timeout_s", 0.20),
                now,
            )
            and getattr(self, "wall_candidate", None) is not None
        )

    def _operator_mode(self, now):
        if not is_fresh(
            getattr(self, "operator_at", None),
            getattr(self, "operator_timeout_s", 0.50),
            now,
        ):
            return OperatorCommand.MODE_ABORT
        operator = getattr(self, "operator", None)
        if operator is None or not operator.enable_control:
            return OperatorCommand.MODE_STANDBY
        return operator.mode

    def _operator_neutral(self, now):
        return self._operator_mode(now) in (
            OperatorCommand.MODE_MANUAL,
            OperatorCommand.MODE_STANDBY,
        )

    @staticmethod
    def _time_to_sec(value):
        if value is None:
            return None
        return value.to_sec() if hasattr(value, "to_sec") else float(value)

    def _ensure_owner_arbiter(self):
        arbiter = getattr(self, "owner_arbiter", None)
        handoff_dwell_s = getattr(self, "handoff_dwell_s", 0.50)
        if arbiter is None:
            arbiter = OwnerArbiter(handoff_dwell_s)
            self.owner_arbiter = arbiter
        arbiter.handoff_dwell_s = max(0.0, float(handoff_dwell_s))
        arbiter.restore(
            self.owner,
            self.interlock_active,
            self._time_to_sec(self.interlock_clear_since),
        )
        return arbiter

    def _sync_arbiter_state(self, arbiter):
        self.owner = arbiter.owner
        self.interlock_active = arbiter.interlock_active
        self.interlock_clear_since = (
            None
            if arbiter.interlock_clear_since_s is None
            else rospy.Time.from_sec(arbiter.interlock_clear_since_s)
        )

    def _candidate_intent(self, candidate, fresh, enabled, required_mode):
        return CandidateIntent(
            enabled=enabled,
            fresh=fresh,
            acquire_request=bool(candidate and candidate.acquire_request),
            keep_ownership=bool(candidate and candidate.keep_ownership),
            candidate_valid=bool(candidate and candidate.valid),
            required_operator_mode=required_mode,
        )

    def _update_owner(self, now):
        arbiter = self._ensure_owner_arbiter()
        operator_mode = self._operator_mode(now)
        event = arbiter.update(
            self._time_to_sec(now),
            operator_mode,
            self._operator_neutral(now),
            self._candidate_intent(
                getattr(self, "ceiling_candidate", None),
                self._ceiling_fresh(now),
                getattr(self, "ceiling_output_enabled", False),
                OperatorCommand.MODE_CEILING,
            ),
            self._candidate_intent(
                getattr(self, "wall_candidate", None),
                self._wall_fresh(now),
                getattr(self, "wall_output_enabled", False),
                OperatorCommand.MODE_WALL,
            ),
        )
        self._sync_arbiter_state(arbiter)
        if event.conflict:
            rospy.logerr_throttle(
                1.0, "attachment output conflict: both decisions request ownership"
            )
        if event.interlock_started:
            rospy.logwarn("attachment owner interlock: %s", event.reason)
        if event.interlock_cleared:
            rospy.loginfo("attachment owner interlock cleared")

    def _flight_ready(self, now):
        if not kill_clear(self, now):
            return False
        ready, _reason = flight_state_ready(
            self.flight_state,
            is_fresh(self.flight_state_at, self.state_timeout_s, now),
            self.require_connected,
            self.require_armed,
            self.require_offboard,
        )
        return ready

    def _odom_fresh(self, now):
        if self.odom is None or not is_fresh(
            self.odom_at, self.odom_timeout_s, now
        ):
            return False
        return odometry_is_finite(self.odom)

    def _get_setpoint_builder(self):
        builder = getattr(self, "setpoint_builder", None)
        if builder is None:
            builder = MavrosSetpointBuilder(
                self.max_velocity_z_mps,
                self.min_thrust,
                self.max_thrust,
                self.local_frame_id,
            )
            self.setpoint_builder = builder
        return builder

    def _rc_ready(self, now):
        if not getattr(self, "rc_assist_enabled", False):
            return True
        if not is_fresh(self.rc_at, self.rc_timeout_s, now) or self.rc_channels is None:
            return False
        if getattr(self, "rc_roll_yaw_enabled", False):
            return all(self._roll_yaw_fraction(axis) is not None for axis in ("roll", "yaw"))
        return all(
            stick_fraction(
                self.rc_channels, index, self.rc_pwm_low, self.rc_pwm_center,
                self.rc_pwm_high, self.rc_deadzone, reverse,
            ) is not None
            for index, reverse in (
                (self.rc_roll_channel_index, self.rc_roll_reverse),
                (self.rc_pitch_channel_index, self.rc_pitch_reverse),
            )
        )

    def _rc_limits(self, now, source):
        free = math.radians(self.rc_free_max_angle_deg)
        contact = math.radians(self.rc_contact_max_angle_deg)
        if source == AttachmentControlCandidate.SOURCE_CEILING:
            if not is_fresh(self.ceiling_status_at, self.status_timeout_s, now):
                return 0.0, 0.0
            state = self.ceiling_status.state
            if state in (
                CeilingAttachmentStatus.STATE_ATTACH_CONTROL,
                CeilingAttachmentStatus.STATE_SURFACE_HOLD,
            ):
                return contact, contact
            if state in (
                CeilingAttachmentStatus.STATE_APPROACH,
                CeilingAttachmentStatus.STATE_DETACH,
                CeilingAttachmentStatus.STATE_RECOVERY_HOVER,
            ):
                return free, free
            return 0.0, 0.0
        if source == AttachmentControlCandidate.SOURCE_WALL:
            if not is_fresh(self.wall_status_at, self.status_timeout_s, now):
                return 0.0, 0.0
            state = self.wall_status.state
            if state in (
                WallPerchStatus.STATE_FRONT_WALL_DETECT,
                WallPerchStatus.STATE_STABILIZE_HOVER,
                WallPerchStatus.STATE_RECOVER,
            ):
                return free, free
            if state == WallPerchStatus.STATE_SLOW_APPROACH:
                return free, contact
            if state == WallPerchStatus.STATE_DETACH_ROTATE:
                return math.radians(self.rc_wall_detach_roll_deg), math.radians(
                    self.rc_wall_detach_pitch_deg
                )
            return contact, contact
        return free, free

    def _assisted_attitude(self, now, quaternion, source):
        if not getattr(self, "rc_assist_enabled", False):
            return self._preflip_bias_attitude(now, quaternion, source)
        if getattr(self, "rc_roll_yaw_enabled", False):
            return self._preflip_bias_attitude(now, self._roll_yaw_attitude(now, quaternion, source), source)
        limits = self._rc_limits(now, source)
        if not self._rc_ready(now):
            rospy.logwarn_throttle(1.0, "RC attitude assist unavailable; trim returning to zero")
            self.rc_age_publisher.publish(Float32(data=-1.0))
            desired_roll = desired_pitch = 0.0
        else:
            channels = self.rc_channels
            desired_roll = stick_fraction(
                channels, self.rc_roll_channel_index, self.rc_pwm_low,
                self.rc_pwm_center, self.rc_pwm_high, self.rc_deadzone,
                self.rc_roll_reverse,
            ) * limits[0]
            desired_pitch = stick_fraction(
                channels, self.rc_pitch_channel_index, self.rc_pwm_low,
                self.rc_pwm_center, self.rc_pwm_high, self.rc_deadzone,
                self.rc_pitch_reverse,
            ) * limits[1]
            self.rc_age_publisher.publish(
                Float32(data=(now - self.rc_at).to_sec() * 1000.0)
            )
        elapsed = (
            (now - self.rc_trim_at).to_sec()
            if self.rc_trim_at is not None else 0.02
        )
        step = math.radians(self.rc_slew_deg_s) * max(0.0, min(elapsed, 0.1))
        # A phase change may tighten the allowed trim immediately.
        self.rc_roll_rad = max(-limits[0], min(self.rc_roll_rad, limits[0]))
        self.rc_pitch_rad = max(-limits[1], min(self.rc_pitch_rad, limits[1]))
        self.rc_roll_rad = slew(self.rc_roll_rad, desired_roll, step)
        self.rc_pitch_rad = slew(self.rc_pitch_rad, desired_pitch, step)
        self.rc_trim_at = now
        values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        x, y, z, w = trim_attitude(values, self.rc_roll_rad, self.rc_pitch_rad)
        return self._preflip_bias_attitude(now, Quaternion(x=x, y=y, z=z, w=w), source)

    def _preflip_bias_attitude(self, now, quaternion, source):
        # ROS body FLU: positive roll tilts upward thrust toward body right (-Y).
        flight = getattr(self, "flight_state", None)
        status = getattr(self, "wall_status", None)
        allowed = bool(
            getattr(self, "preflip_right_roll_deg", 0.0) > 0
            and source in (AttachmentControlCandidate.SOURCE_NONE, AttachmentControlCandidate.SOURCE_WALL)
            and flight is not None and flight.connected and flight.armed and flight.mode.upper() == "OFFBOARD"
            and is_fresh(self.flight_state_at, self.state_timeout_s, now)
            and self._operator_mode(now) == OperatorCommand.MODE_WALL
            and status is not None and is_fresh(self.wall_status_at, self.status_timeout_s, now)
            and status.state in (WallPerchStatus.STATE_IDLE, WallPerchStatus.STATE_FRONT_WALL_DETECT,
                                 WallPerchStatus.STATE_STABILIZE_HOVER, WallPerchStatus.STATE_SLOW_APPROACH))
        if getattr(self, "approach_attitude_limits_enabled", False):
            # Automatic bias belongs only to confirmed approach, never to takeover fallback.
            allowed = allowed and status.state == WallPerchStatus.STATE_SLOW_APPROACH and not status.approach_paused
        applied = 0.0
        if allowed:
            previous_at = getattr(self, "preflip_bias_at", None)
            dt = max(0.0, min((now-previous_at).to_sec() if previous_at is not None else .02, .1))
            bias = slew(getattr(self, "preflip_bias_rad", 0.0), math.radians(self.preflip_right_roll_deg),
                        math.radians(self.preflip_roll_slew_deg_s)*dt)
            values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            roll = euler_from_quaternion(values)[0]
            bound = math.radians(min(5.0, getattr(self, "rc_free_max_angle_deg", 5.0)))
            applied = bias if getattr(self, "approach_attitude_limits_enabled", False) else max(-bound, min(bound, roll+bias))-roll
            values = trim_attitude(values, applied, 0.0)
            quaternion = Quaternion(x=values[0], y=values[1], z=values[2], w=values[3])
            self.preflip_bias_rad = bias
        else:
            # Clear immediately on phase loss; never carry this bias into flip or recovery.
            self.preflip_bias_rad = 0.0
        self.preflip_bias_at = now
        publisher = getattr(self, "preflip_bias_publisher", None)
        if publisher is not None:
            publisher.publish(Float32(data=math.degrees(applied)))
        return quaternion

    def _level_attitude(self):
        orientation = self.odom.pose.pose.orientation
        yaw = euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )[2]
        return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    def _handover_source_ready(self, now):
        if not getattr(self, "match_fcu_thrust_on_handover", False):
            return True
        target = getattr(self, "fcu_attitude_target", None)
        received_at = getattr(self, "fcu_attitude_target_at", None)
        if target is None or not is_fresh(
            received_at, getattr(self, "fcu_target_timeout_s", 0.20), now
        ):
            return False
        if target.type_mask & AttitudeTarget.IGNORE_THRUST:
            return False
        thrust = float(target.thrust)
        # The takeover sample is allowed across MAVROS' complete normalized
        # multicopter thrust range. Behavior output returns to max_thrust after
        # the bumpless ramp completes.
        return bool(math.isfinite(thrust) and 0.0 <= thrust <= 1.0)

    def _prestream_attitude_target(self, now):
        if not getattr(self, "match_fcu_thrust_on_handover", False):
            return self._vertical_attitude_target(now)

        flight_mode = (
            self.flight_state.mode.upper()
            if getattr(self, "flight_state", None) is not None
            else ""
        )
        if flight_mode == "OFFBOARD":
            # FCU target feedback is deliberately ignored in OFFBOARD to avoid
            # reading our own command back as the ALTCTL takeover source. Keep
            # publishing the value captured on the ALTCTL -> OFFBOARD edge
            # until an attachment candidate takes ownership and starts the
            # configured bumpless ramp.
            thrust = getattr(self, "handover_start_thrust", None)
            if thrust is None:
                rospy.logerr_throttle(
                    1.0,
                    "OFFBOARD prestream inhibited: captured ALTCTL thrust unavailable",
                )
                return None
        else:
            if not self._handover_source_ready(now):
                rospy.logwarn_throttle(
                    1.0,
                    "OFFBOARD prestream inhibited: FCU thrust target missing, stale or invalid",
                )
                return None
            thrust = float(self.fcu_attitude_target.thrust)

        orientation = self._assisted_attitude(
            now, self._level_attitude(), AttachmentControlCandidate.SOURCE_NONE
        )
        return self._attitude_target(
            now,
            orientation,
            thrust,
            apply_handover=False,
        )

    def _apply_thrust_handover(self, now, desired_thrust):
        if not getattr(self, "match_fcu_thrust_on_handover", False):
            return desired_thrust
        start = getattr(self, "handover_start_thrust", None)
        if start is None:
            return desired_thrust
        if getattr(self, "handover_started_at", None) is None:
            self.handover_started_at = now
        elapsed = max(0.0, (now - self.handover_started_at).to_sec())
        duration = getattr(self, "thrust_handover_ramp_s", 1.0)
        fraction = min(1.0, elapsed / duration)
        if fraction >= 1.0:
            self.handover_start_thrust = None
            self.handover_started_at = None
            return desired_thrust
        # Cubic smoothstep has zero slope at both ends.
        blend = fraction * fraction * (3.0 - 2.0 * fraction)
        return start + blend * (desired_thrust - start)

    def _vertical_attitude_target(self, now, velocity_z=0.0, source=0):
        if not self._odom_fresh(now) or not math.isfinite(velocity_z):
            return None
        measured = vertical_speed_enu_mps(
            self.odom, getattr(self, "odom_twist_in_child_frame", True)
        )
        if not math.isfinite(measured):
            return None
        thrust = self.hover_thrust + self.vertical_speed_kp * (velocity_z - measured)
        orientation = self._assisted_attitude(now, self._level_attitude(), source)
        return self._attitude_target(now, orientation, thrust)

    def _attitude_target(self, now, quaternion, thrust, apply_handover=True):
        full_range_handover = bool(
            getattr(self, "match_fcu_thrust_on_handover", False)
            and not apply_handover
        )
        if apply_handover:
            thrust = self._apply_thrust_handover(now, thrust)
            full_range_handover = getattr(
                self, "handover_start_thrust", None
            ) is not None
        target = self._get_setpoint_builder().attitude_thrust(
            now, quaternion, thrust
        )
        if target is not None and full_range_handover:
            # MavrosSetpointBuilder enforces the normal behavior cap. Only the
            # captured handover sample and its ramp may temporarily bypass that
            # cap so the first OFFBOARD command exactly matches ALTCTL.
            target.thrust = min(1.0, max(0.0, float(thrust)))
        return target

    def _limit_final_approach_target(self, now, target, source):
        if not getattr(self, "approach_attitude_limits_enabled", False):
            return target
        status = getattr(self, "wall_status", None)
        flight = getattr(self, "flight_state", None)
        active = bool(getattr(self, "approach_attitude_limits_enabled", False)
                      and source == AttachmentControlCandidate.SOURCE_WALL
                      and status is not None and status.state == WallPerchStatus.STATE_SLOW_APPROACH
                      and is_fresh(self.wall_status_at, self.status_timeout_s, now)
                      and flight is not None and flight.mode.upper() == "OFFBOARD"
                      and is_fresh(self.flight_state_at, self.state_timeout_s, now))
        q = tuple(getattr(target.orientation, k) for k in "xyzw")
        previous = getattr(self, "approach_previous_target", None)
        previous_at = getattr(self, "approach_previous_at", None)
        if active:
            dt = max(0.0, min((now-previous_at).to_sec(), .1)) if previous_at is not None else .02
            if previous is None or not is_fresh(previous_at, self.status_timeout_s, now):
                measured = self.odom.pose.pose.orientation
                previous = (measured.x, measured.y, measured.z, measured.w)
                dt = .02
            q, projected = limit_recovery_attitude(q, previous, dt,
                math.radians(self.approach_max_tilt_deg), math.radians(self.approach_tilt_slew_deg_s),
                math.radians(self.approach_yaw_slew_deg_s))
            if projected:
                rospy.logwarn_throttle(1.0, "Approach entry target outside cone: immediate projection before slew limiting")
            target.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
            roll,pitch,_ = euler_from_quaternion(q)
            axis,_ = axis_yaw(q)
            telemetry = Vector3Stamped();telemetry.header.stamp = now
            telemetry.vector.x = math.degrees(roll);telemetry.vector.y = math.degrees(pitch)
            telemetry.vector.z = math.degrees(math.acos(max(-1.,min(1.,axis[2]))))
            self.approach_target_publisher.publish(telemetry)
        # Keep final sent history through ALTCTL prestream, including the RC correction.
        self.approach_previous_target, self.approach_previous_at = q, now
        return target

    def _limit_final_recovery_target(self, now, target, source):
        # This is after RC trim and setpoint-builder thrust handling.
        status = getattr(self, "wall_status", None)
        flight = getattr(self, "flight_state", None)
        active = (getattr(self, "recovery_attitude_limits_enabled", False)
                  and source == AttachmentControlCandidate.SOURCE_WALL
                  and status is not None
                  and status.state in (WallPerchStatus.STATE_RECOVER, WallPerchStatus.STATE_ABORT)
                  and is_fresh(self.wall_status_at, self.status_timeout_s, now)
                  and flight is not None and flight.mode.upper() == "OFFBOARD"
                  and is_fresh(self.flight_state_at, self.state_timeout_s, now))
        q = tuple(getattr(target.orientation, k) for k in "xyzw")
        if active:
            dt = max(0.0, min((now-self.recovery_previous_at).to_sec(), .1)) if self.recovery_previous_at is not None else .02
            q, projected = limit_recovery_attitude(
                q, self.recovery_previous_target, dt, math.radians(self.recovery_max_tilt_deg),
                math.radians(self.recovery_tilt_slew_deg_s), math.radians(self.recovery_yaw_slew_deg_s))
            if projected:
                rospy.logwarn_throttle(1.0, "Recovery entry target outside cone: immediate projection before slew limiting")
            target.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            target.thrust = min(target.thrust, self.recovery_final_thrust_cap)
            axis, yaw = axis_yaw(q)
            preview = Vector3Stamped();preview.header.stamp = now
            preview.vector.x = math.degrees(math.acos(max(-1.0, min(1.0, axis[2]))))
            preview.vector.y = math.degrees(yaw);preview.vector.z = target.thrust
            self.recovery_target_publisher.publish(preview)
            self.recovery_horizontal_publisher.publish(Float32(data=target.thrust*math.hypot(axis[0], axis[1])))
        # Record the actual final target also in preceding protected phases.
        self.recovery_previous_target, self.recovery_previous_at = q, now
        return target

    def _publish_candidate(self, now, candidate):
        if candidate.kind == AttachmentControlCandidate.KIND_VERTICAL_VELOCITY:
            target = self._vertical_attitude_target(
                now, candidate.velocity_z_enu_mps, candidate.source
            )
            if target is None:
                return self.OUTPUT_NONE
            target = self._limit_final_approach_target(now, target, candidate.source)
            self.attitude_publisher.publish(target)
            return self.OUTPUT_ATTITUDE_THRUST
        if candidate.kind == AttachmentControlCandidate.KIND_ATTITUDE_THRUST:
            orientation = self._assisted_attitude(
                now, candidate.attitude_setpoint, candidate.source
            )
            target = self._attitude_target(
                now, orientation, candidate.thrust_normalized
            )
            if target is not None:
                target = self._limit_final_recovery_target(now, target, candidate.source)
                target = self._limit_final_approach_target(now, target, candidate.source)
                self.attitude_publisher.publish(target)
                return self.OUTPUT_ATTITUDE_THRUST
        return self.OUTPUT_NONE

    def _on_timer(self, _event):
        now = rospy.Time.now()
        previous_owner = self.owner
        self._update_owner(now)
        self.owner_publisher.publish(UInt8(data=self.owner))
        if self.owner != previous_owner:
            rospy.loginfo("attachment output owner=%d", self.owner)

        output_mode = self.OUTPUT_NONE
        if self.output_enabled and kill_clear(self, now) and self.owner == self.OWNER_NONE:
            # The mode manager requests this stream before asking PX4 for
            # OFFBOARD, and keeps it alive through the return to ALTCTL.
            if (
                self.prestream_requested
                and is_fresh(self.prestream_at, self.prestream_timeout_s, now)
                and self._odom_fresh(now)
                and self.flight_state is not None
                and is_fresh(self.flight_state_at, self.state_timeout_s, now)
                and self.flight_state.connected
                and self.flight_state.armed
            ):
                if self._rc_ready(now):
                    target = self._prestream_attitude_target(now)
                    if target is not None:
                        target = self._limit_final_approach_target(now, target, AttachmentControlCandidate.SOURCE_NONE)
                        self.attitude_publisher.publish(target)
                        output_mode = self.OUTPUT_ATTITUDE_THRUST
        elif self.output_enabled and self.owner != self.OWNER_NONE:
            if not self._flight_ready(now):
                rospy.logwarn_throttle(1.0, "attachment output inhibited: flight not ready")
            elif not self._odom_fresh(now):
                rospy.logwarn_throttle(1.0, "attachment output inhibited: odometry stale")
            elif self.owner == self.OWNER_CEILING:
                if self.ceiling_candidate.valid:
                    output_mode = self._publish_candidate(
                        now, self.ceiling_candidate
                    )
                else:
                    rospy.logerr_throttle(
                        1.0,
                        "ceiling candidate invalid; output stopped and owner retained",
                    )
            elif self.owner == self.OWNER_WALL:
                if self.wall_candidate.valid:
                    output_mode = self._publish_candidate(now, self.wall_candidate)
                else:
                    rospy.logerr_throttle(
                        1.0,
                        "wall candidate invalid; output stopped and owner retained",
                    )
        self.mode_publisher.publish(UInt8(data=output_mode))


if __name__ == "__main__":
    rospy.init_node("attachment_output_mux_node")
    AttachmentOutputMuxNode()
    rospy.spin()
