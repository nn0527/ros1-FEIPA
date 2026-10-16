#!/usr/bin/env python3
"""ALTCTL -> OFFBOARD -> completed detach -> ALTCTL handover.

The output mux remains the sole MAVROS setpoint publisher. This node requests
its attitude prestream and uses MAVROS only to change flight modes.
"""

import threading
import time
import math

import rospy
from mavros_msgs.msg import EstimatorStatus, RCIn, State
from mavros_msgs.srv import SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, UInt8
from px4_wall_ceiling_control.adapters.kill_switch import install_kill_guard, kill_clear

from px4_wall_ceiling_control.adapters.odometry import (
    odometry_is_finite,
    vertical_speed_enu_mps,
)
from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion
from px4_wall_ceiling_control.freshness import is_fresh_receipt as is_fresh, is_fresh_source
from px4_wall_ceiling_control.msg import (
    CeilingAttachmentStatus,
    OperatorCommand,
    SensorHealth,
    WallPerchStatus,
)


class FlightModeManagerNode:
    IDLE = "IDLE"
    WAITING_WALL = "WAITING_WALL"
    PRESTREAM = "PRESTREAM"
    ENTERING = "ENTERING"
    CANCELING_ENTRY = "CANCELING_ENTRY"
    ACTIVE = "ACTIVE"
    RETURNING = "RETURNING"
    LOCKED = "LOCKED"

    def __init__(self):
        install_kill_guard(self, rospy, String)
        self.enabled = bool(rospy.get_param("~enabled", False))
        self.prestream_s = max(1.2, float(rospy.get_param("~prestream_s", 1.5)))
        self.retry_s = max(1.0, float(rospy.get_param("~retry_s", 2.0)))
        self.enter_timeout_s = max(2.0, float(rospy.get_param("~enter_timeout_s", 6.0)))
        self.entry_stream_dropout_grace_s = float(rospy.get_param("~entry_stream_dropout_grace_s", 0.12))
        if not 0.0 <= self.entry_stream_dropout_grace_s <= 0.15:
            raise ValueError("entry dropout grace must be between 0 and 0.15 seconds")
        self.entry_stream_loss_since = None
        self.canceled_entry_mode = None
        self.cancel_entry_started = None
        self.cancel_entry_pilot_samples = 0
        self.cancel_entry_stable_since = None
        self.last_offboard_finished_at = None
        self.offboard_exit_cleanup = False
        self.pilot_intent_request_at = None
        self.acquire_timeout_s = max(2.0, float(rospy.get_param("~acquire_timeout_s", 6.0)))
        self.state_timeout_s = float(rospy.get_param("~state_timeout_s", 1.5))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.5))
        self.estimator_timeout_s = float(rospy.get_param("~estimator_timeout_s", 1.5))
        self.operator_timeout_s = float(rospy.get_param("~operator_timeout_s", 0.5))
        self.sensor_timeout_s = float(rospy.get_param("~sensor_timeout_s", 0.5))
        self.wall_manual_approach = bool(rospy.get_param("~wall_manual_approach", False))
        self.ceiling_baro_handover_enabled = bool(rospy.get_param("~ceiling_baro_handover_enabled", False))
        self.ceiling_distance_gate_enabled = bool(rospy.get_param("~ceiling_distance_gate_enabled", False))
        self.wall_pilot_mode = str(rospy.get_param("~wall_pilot_mode", "ALTCTL")).upper()
        self.wall_level_handover_enabled = bool(rospy.get_param("~wall_level_handover_enabled", True))
        self.wall_level_handover_tilt_deg = float(rospy.get_param("~wall_level_handover_tilt_deg", 5.))
        self.wall_level_handover_hold_s = float(rospy.get_param("~wall_level_handover_hold_s", .30))
        self.wall_level_handover_retry_s = float(rospy.get_param("~wall_level_handover_retry_s", .50))
        if (not all(math.isfinite(v) for v in (self.wall_level_handover_tilt_deg,
                self.wall_level_handover_hold_s, self.wall_level_handover_retry_s))
                or not 0. < self.wall_level_handover_tilt_deg <= 5.
                or not .30 <= self.wall_level_handover_hold_s <= 2.
                or not .20 <= self.wall_level_handover_retry_s <= 2.):
            raise ValueError("invalid wall level pilot handover settings")
        self.wall_level_returning = False
        self.wall_level_window = None
        self.wall_level_handover_reason = "等待回平"
        self.wall_level_lock = threading.RLock()
        self.wall_horizontal_velocity_enabled = bool(rospy.get_param("/wall_perch_decision_node/horizontal_velocity_enabled",False))
        if self.wall_pilot_mode not in ("ALTCTL","POSCTL"):raise ValueError("unsupported wall pilot mode")
        self.wall_auto_approach = bool(rospy.get_param("~wall_auto_approach", False))
        if self.wall_manual_approach and self.wall_auto_approach:
            raise ValueError("manual and automatic wall approach cannot both be enabled")
        self.manual_wall_wait_timeout_s = max(2.0, float(rospy.get_param("~manual_wall_wait_timeout_s", 60.0)))
        self.manual_wall_wait_started = None
        self.manual_wait_canceled = False
        rospy.Subscriber("/wall_perch/cancel_wait", Bool, self._on_wait_cancel, queue_size=1)
        self.cancel_pub = rospy.Publisher("/attachment/terminal_command", String, queue_size=1)
        self.wall_bench_wait_for_ranges = bool(rospy.get_param("~wall_bench_wait_for_ranges", False))
        self.ignore_range_loss_during_attachment = bool(rospy.get_param(
            "~ignore_range_loss_during_attachment",
            rospy.get_param("/wall_perch_decision_node/ignore_range_loss_during_attachment", False)))
        self.output_timeout_s = float(rospy.get_param("~output_timeout_s", 0.3))
        self.status_timeout_s = float(rospy.get_param("~status_timeout_s", 0.5))
        self.require_rc_center = bool(rospy.get_param("~require_rc_center", False))
        self.rc_roll_channel_index = int(rospy.get_param("~rc_roll_channel_index", -1))
        self.rc_pitch_channel_index = int(rospy.get_param("~rc_pitch_channel_index", -1))
        self.rc_throttle_channel_index = int(
            rospy.get_param("~rc_throttle_channel_index", -1)
        )
        self.rc_pwm_center = int(rospy.get_param("~rc_pwm_center", 1500))
        self.rc_center_tolerance_pwm = int(
            rospy.get_param("~rc_center_tolerance_pwm", 70)
        )
        self.rc_timeout_s = float(rospy.get_param("~rc_timeout_s", 0.20))
        self.handover_max_tilt_deg = float(
            rospy.get_param("~handover_max_tilt_deg", 10.0)
        )
        self.handover_max_vertical_speed_mps = float(
            rospy.get_param("~handover_max_vertical_speed_mps", 0.20)
        )
        self.odom_twist_in_child_frame = bool(
            rospy.get_param("~odom_twist_in_child_frame", True)
        )
        if self.require_rc_center and len({
            self.rc_roll_channel_index,
            self.rc_pitch_channel_index,
            self.rc_throttle_channel_index,
        }) != 3:
            raise ValueError("RC handover requires three distinct stick channels")
        if self.require_rc_center and min(
            self.rc_roll_channel_index,
            self.rc_pitch_channel_index,
            self.rc_throttle_channel_index,
        ) < 0:
            raise ValueError("RC handover channels must be calibrated before flight")

        self.phase = self.IDLE
        self.selected_mode = None
        self.phase_since = time.monotonic()
        self.stream_since = None
        self.last_request_at = None
        self.request_in_flight = False
        self.takeover_attempt = 0
        self.offboard_request = None
        self.lock_until_neutral = False
        self.owner_seen = False
        self.detach_seen = False
        self.recovery_seen = False
        self.completed = False
        self.fault_seen = False
        self.wall_fault_return_ready = False
        self.wall_failure_reason = ""
        self.wall_return_episode = False
        self.ceiling_approach_timeout_returning = False
        self.wall_approach_timeout_returning = False
        self.wall_timeout_stable_since = self.wall_timeout_last_odom = None
        self.wall_timeout_safe_samples = 0
        self.flight = None
        self.flight_at = None
        self.odom = None
        self.odom_at = None
        self.rc_channels = None
        self.rc_at = None
        self.estimator = None
        self.estimator_at = None
        self.operator = None
        self.operator_at = None
        self.sensor = None
        self.sensor_at = None
        self.output_mode = None
        self.output_at = None
        self.owner = 0
        self.owner_at = None
        self.ceiling = None
        self.ceiling_at = None
        self.wall = None
        self.wall_at = None

        self.prestream_pub = rospy.Publisher(
            "/attachment/prestream", Bool, queue_size=10
        )
        self.phase_pub = rospy.Publisher(
            "/attachment/flight_phase", String, queue_size=10, latch=True
        )
        self.phase_reason_pub = rospy.Publisher(
            "/attachment/flight_phase_reason", String, queue_size=10, latch=True
        )
        self.phase_reason = "initialized"
        rospy.Subscriber("/mavros/state", State, self._on_flight, queue_size=10)
        rospy.Subscriber(
            "/mavros/local_position/odom", Odometry, self._on_odom, queue_size=1
        )
        # Intent cleanup needs live pilot input even when stick centering is disabled.
        rospy.Subscriber(
            rospy.get_param("~rc_topic", "/mavros/rc/in"),
            RCIn, self._on_rc, queue_size=1,
        )
        rospy.Subscriber(
            "/mavros/estimator_status", EstimatorStatus,
            self._on_estimator, queue_size=10,
        )
        rospy.Subscriber(
            "/operator/command", OperatorCommand, self._on_operator, queue_size=10
        )
        rospy.Subscriber(
            "/sensor/health", SensorHealth, self._on_sensor, queue_size=10
        )
        rospy.Subscriber(
            "/attachment/output_mode", UInt8, self._on_output, queue_size=10
        )
        rospy.Subscriber(
            "/attachment/output_owner", UInt8, self._on_owner, queue_size=10
        )
        rospy.Subscriber(
            "/ceiling_attachment/status",
            CeilingAttachmentStatus,
            self._on_ceiling,
            queue_size=10,
        )
        rospy.Subscriber(
            "/wall_perch/status", WallPerchStatus, self._on_wall, queue_size=10
        )
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._on_timer)
        if rospy.get_param("~motor_output_telemetry_enabled", False):
            threading.Thread(target=self._motor_telemetry_loop, daemon=True).start()
        if not self.enabled:
            rospy.logwarn("automatic flight mode handover is disabled")

    def _request_motor_telemetry(self):
        # Run only from the telemetry worker, never block the control timer.
        def disarmed():
            flight = self.flight
            return (flight is not None and is_fresh(self.flight_at, self.state_timeout_s, rospy.Time.now())
                    and flight.connected and not flight.armed)
        if not disarmed():
            return False
        from mavros_msgs.srv import MessageInterval
        rospy.wait_for_service('/mavros/set_message_interval', timeout=2)
        if not disarmed():
            return False
        result = rospy.ServiceProxy('/mavros/set_message_interval', MessageInterval)(message_id=36, message_rate=10.0)
        if result.success:
            rospy.loginfo('Requested SERVO_OUTPUT_RAW 10 Hz; reception must be checked in /mavros/rc/out')
        return result.success

    def _motor_telemetry_loop(self):
        # Requests are volatile across FCU reboot. Retry while disarmed;
        # acknowledgement alone does not prove the FCU emits output samples.
        next_attempt = 0.0
        while not rospy.is_shutdown():
            if time.monotonic() >= next_attempt:
                try:
                    if self._request_motor_telemetry():
                        next_attempt = time.monotonic() + 30.0
                except (rospy.ROSException, rospy.ServiceException) as exc:
                    rospy.logwarn_throttle(30, 'Motor output telemetry unavailable: %s', exc)
            time.sleep(2)

    def _on_flight(self, message):
        self.flight, self.flight_at = message, rospy.Time.now()
        if getattr(self,"wall_approach_timeout_returning",False) and message.mode.upper() != "OFFBOARD":
            self.wall_timeout_stable_since=self.wall_timeout_last_odom=None
            self.wall_timeout_safe_samples=0
        if self.phase == self.CANCELING_ENTRY:
            if message.connected and message.mode.upper() == self._pilot_mode():
                self.cancel_entry_pilot_samples = getattr(self, "cancel_entry_pilot_samples", 0) + 1
            else:
                self.cancel_entry_pilot_samples = 0
                if getattr(self, "offboard_exit_cleanup", False):
                    self.pilot_intent_request_at = None
                    self.cancel_entry_stable_since = None

    def _on_odom(self, message):
        received = rospy.Time.now()
        if not hasattr(self, 'wall_level_lock'):self.wall_level_lock = threading.RLock()
        with self.wall_level_lock:
            self.odom, self.odom_at = message, received
            self.handover_odom_snapshot = (message, received)
            if getattr(self, 'wall_level_handover_enabled', False):
                # Observe every delivered frame: a brief tilt excursion between
                # manager ticks must invalidate the continuous-level window.
                # This only records evidence; mode requests remain in the timer.
                self._wall_level_handover_ready(received)

    def _on_rc(self, message):
        self.rc_channels, self.rc_at = tuple(message.channels), rospy.Time.now()

    def _on_estimator(self, message):
        self.estimator, self.estimator_at = message, rospy.Time.now()

    def _on_operator(self, message):
        self.operator, self.operator_at = message, rospy.Time.now()

    def _on_sensor(self, message):
        self.sensor, self.sensor_at = message, rospy.Time.now()

    def _on_output(self, message):
        self.output_mode, self.output_at = int(message.data), rospy.Time.now()
        if self.output_mode == 2:
            self.entry_stream_loss_since = None

    def _on_owner(self, message):
        self.owner, self.owner_at = int(message.data), rospy.Time.now()

    def _on_ceiling(self, message):
        self.ceiling, self.ceiling_at = message, rospy.Time.now()

    def _on_wall(self, message):
        self.wall, self.wall_at = message, rospy.Time.now()

    def _set_phase(self, phase, reason=None):
        if self.phase != phase:
            if reason:
                rospy.loginfo(
                    "attachment flight phase: %s -> %s | reason: %s",
                    self.phase,
                    phase,
                    reason,
                )
            else:
                rospy.loginfo("attachment flight phase: %s -> %s", self.phase, phase)
            self.phase = phase
            self.phase_since = time.monotonic()
        if reason:
            self.phase_reason = reason

    def _requested_mode(self, now):
        if not is_fresh(self.operator_at, self.operator_timeout_s, now):
            return None
        if self.operator is None or not self.operator.enable_control:
            return None
        if self.operator.mode in (
            OperatorCommand.MODE_CEILING,
            OperatorCommand.MODE_WALL,
        ):
            return self.operator.mode
        return None

    def _preflight_failure_reason(self, now, selected):
        if not self.enabled:
            return "flight mode handover disabled"
        if self.flight is None:
            return "flight state unavailable"
        if not is_fresh(self.flight_at, self.state_timeout_s, now):
            return "flight state timeout"
        if not self.flight.connected:
            return "MAVROS disconnected"
        if not self.flight.armed:
            return "vehicle disarmed"
        if self.odom is None:
            return "local odometry unavailable"
        if not is_fresh(self.odom_at, self.odom_timeout_s, now):
            return "local odometry timeout"
        if not odometry_is_finite(self.odom):
            return "local odometry is not finite"
        if self.estimator is None:
            return "estimator status unavailable"
        if not is_fresh(self.estimator_at, self.estimator_timeout_s, now):
            return "estimator status timeout"
        if not (
            self.estimator.attitude_status_flag
            and self.estimator.velocity_vert_status_flag
        ):
            return "attitude or vertical velocity estimate invalid"
        if getattr(self, "require_rc_center", False) and not self._rc_fresh(now):
            return "RC input unavailable or invalid"
        if selected == OperatorCommand.MODE_WALL and not self._wall_horizontal_ready(now):
            detail=(self.wall.horizontal_equilibrium_reason if self.wall is not None
                    and getattr(self.wall,"horizontal_equilibrium_enabled",False) and not self.wall.horizontal_equilibrium_valid
                    else getattr(self.wall,"horizontal_velocity_reason","") if self.wall else "等待贴墙状态")
            return "水平速度接管条件未就绪：" + detail
        if selected == OperatorCommand.MODE_WALL and (
            getattr(self, "wall_bench_wait_for_ranges", False)
            or getattr(self, "ignore_range_loss_during_attachment", False)
        ):
            return None  # Range faults are diagnostic; automatic takeover does not require ranges.
        if selected == OperatorCommand.MODE_CEILING and getattr(self, "ceiling_distance_gate_enabled", False):
            return None  # Decision pauses on range loss; takeover still requires its valid gate.
        if self.sensor is None:
            return "sensor health unavailable"
        if not is_fresh(self.sensor_at, self.sensor_timeout_s, now):
            return "sensor health timeout"
        if selected == OperatorCommand.MODE_CEILING:
            return None if self.sensor.ceiling_valid else "ceiling sensor invalid"
        if selected == OperatorCommand.MODE_WALL:
            if not self.sensor.front_valid:
                return "front sensor invalid"
            return None
        return "unsupported attachment mode"

    def _pilot_mode(self, selected=None):
        selected = getattr(self,"selected_mode",None) if selected is None else selected
        return getattr(self,"wall_pilot_mode","ALTCTL") if selected == OperatorCommand.MODE_WALL else "ALTCTL"

    def _wall_horizontal_ready(self, now, stable=False):
        if not getattr(self,"wall_horizontal_velocity_enabled",False):return True
        wall=getattr(self,"wall",None)
        return bool(wall is not None and is_fresh(self.wall_at,self.status_timeout_s,now)
            and getattr(wall,"horizontal_velocity_enabled",False) and wall.horizontal_velocity_ready
            and (not getattr(wall,"horizontal_equilibrium_enabled",False) or wall.horizontal_equilibrium_valid
                 or (not stable and getattr(self,"phase",None) in (self.IDLE,self.WAITING_WALL)))
            and (not stable or wall.horizontal_velocity_stable)
            and self.estimator is not None and is_fresh(self.estimator_at,self.estimator_timeout_s,now)
            and self.estimator.velocity_horiz_status_flag and not self.estimator.const_pos_mode_status_flag
            and (self._pilot_mode() != "POSCTL" or self.estimator.pos_horiz_rel_status_flag or self.estimator.pos_horiz_abs_status_flag))

    def _preflight_ready(self, now, selected):
        return self._preflight_failure_reason(now, selected) is None

    def _on_wait_cancel(self, message):
        if message.data and self.phase == self.WAITING_WALL:
            self.manual_wait_canceled = True

    def _ceiling_takeover_ready(self, now):
        if not getattr(self, "ceiling_distance_gate_enabled", False) or self.selected_mode != OperatorCommand.MODE_CEILING:
            return True
        status = self.ceiling
        if not (status is not None and is_fresh(self.ceiling_at,self.status_timeout_s,now) and status.activation_ready):return False
        if self.odom is None or not is_fresh(self.odom_at,self.odom_timeout_s,now) or not odometry_is_finite(self.odom):return False
        speed=vertical_speed_enu_mps(self.odom,self.odom_twist_in_child_frame)
        return math.isfinite(speed)

    def _ceiling_wait_reason(self,now):
        if self.ceiling is None or not is_fresh(self.ceiling_at,self.status_timeout_s,now):
            return "吸顶请求已接收：等待新鲜吸顶状态；仍由飞手控制ALTCTL"
        if not self.ceiling.activation_ready:
            return "吸顶请求已接收：等待有效顶距≤1m连续0.30秒；仍由飞手控制ALTCTL"
        if self.odom is None or not is_fresh(self.odom_at,self.odom_timeout_s,now) or not odometry_is_finite(self.odom):
            return "吸顶请求已接收：等待新鲜有效里程计；仍由飞手控制ALTCTL"
        return "吸顶请求已接收：等待有效竖直速度估计；仍由飞手控制ALTCTL"

    def _auto_wall_selected(self):
        return getattr(self, "wall_auto_approach", False) and self.selected_mode == OperatorCommand.MODE_WALL

    def _gated_wall_selected(self):
        return self._manual_wall_selected() or self._auto_wall_selected()

    def _wall_takeover_ready(self, now):
        if not self._auto_wall_selected():
            return self._manual_wall_ready(now)
        wall = self.wall
        if not (wall is not None and is_fresh(self.wall_at, self.status_timeout_s, now)
                and wall.state == WallPerchStatus.STATE_IDLE
                and (not getattr(wall, "horizontal_equilibrium_enabled", False)
                     or wall.horizontal_equilibrium_valid)
                and wall.entry_height_ready):
            return False
        # This bypass applies before OFFBOARD as well as during attachment.
        # Do not manufacture front_ready: valid distance still gates the flip.
        if getattr(self, "ignore_range_loss_during_attachment", False):
            return True
        return bool(wall.front_ready and wall.front_range_fresh
                    and wall.sensor_health_fresh
                    and self.sensor is not None and is_fresh(self.sensor_at, self.sensor_timeout_s, now)
                    and self.sensor.front_valid)

    def _manual_wall_selected(self):
        return getattr(self, "wall_manual_approach", False) and self.selected_mode == OperatorCommand.MODE_WALL

    def _manual_wall_ready(self, now):
        wall = self.wall
        return bool(wall is not None and is_fresh(self.wall_at, self.status_timeout_s, now)
                    and wall.state == WallPerchStatus.STATE_IDLE
                    and wall.flip_ready and wall.front_range_fresh
                    and (not getattr(wall,"horizontal_equilibrium_enabled",False) or wall.horizontal_equilibrium_valid)
                    and wall.entry_height_ready
                    and self.sensor is not None and is_fresh(self.sensor_at, self.sensor_timeout_s, now)
                    and self.sensor.front_valid)

    def _manual_wait_elapsed(self, monotonic_now):
        return monotonic_now - self.manual_wall_wait_started

    def _manual_wait_reason(self, now, monotonic_now):
        remaining = max(0.0, self.manual_wall_wait_timeout_s - self._manual_wait_elapsed(monotonic_now))
        if self._auto_wall_selected():
            wall=getattr(self,"wall",None)
            if wall is not None and getattr(wall,"horizontal_equilibrium_enabled",False) and not wall.horizontal_equilibrium_valid:
                return "等待接管平衡基准：" + wall.horizontal_equilibrium_reason
            if getattr(self, "ignore_range_loss_during_attachment", False):
                return "等待自动接近接管：保持 {}；雷达仅告警，等待高度、平衡基准及预发送就绪，剩余 {:.0f} 秒".format(self._pilot_mode(), remaining)
            return "等待自动接近接管：保持 {}；前距≤0.50 m连续0.40 s且两路雷达有效，等待窗口剩余 {:.0f} 秒".format(self._pilot_mode(),remaining)
        return "等待飞手手动接近：保持 {}；需有效距离连续满足翻转阈值，等待窗口剩余 {:.0f} 秒".format(self._pilot_mode(),remaining)

    def _stream_ready(self, now):
        return bool(
            self.output_mode == 2
            and is_fresh(self.output_at, self.output_timeout_s, now)
            and self.owner == 0
            and is_fresh(self.owner_at, self.output_timeout_s, now)
        )

    def _entry_stream_failed(self, now, monotonic_now):
        if self._stream_ready(now):
            self.entry_stream_loss_since = None
            return False
        # No grace for missing feedback, owner changes or a stale publisher.
        if (not is_fresh(self.output_at, self.output_timeout_s, now)
                or not is_fresh(self.owner_at, self.output_timeout_s, now) or self.owner != 0):
            return True
        if getattr(self, "entry_stream_loss_since", None) is None:
            self.entry_stream_loss_since = monotonic_now
        return monotonic_now - self.entry_stream_loss_since >= getattr(self, "entry_stream_dropout_grace_s", .12)

    def _begin_entry_cancel(self, reason, monotonic_now):
        if self.phase != self.CANCELING_ENTRY:
            self.canceled_entry_mode = self.selected_mode
            self.cancel_entry_started = monotonic_now
            self.cancel_entry_pilot_samples = 0
            self.cancel_entry_stable_since = None
            # Invalidate workers which have not submitted yet; retain selected mode
            # and a tombstone for commands already submitted to the FCU.
            self.takeover_attempt = getattr(self, "takeover_attempt", 0) + 1
            self.offboard_request = None
            self._set_phase(self.CANCELING_ENTRY, reason="取消接管处理中：" + reason)
            self.cancel_pub.publish(String(data="cancel"))
        else:
            self.phase_reason = "取消接管处理中：" + reason

    def _begin_offboard_exit(self, reason, monotonic_now):
        self._begin_entry_cancel(reason, monotonic_now)
        self.offboard_exit_cleanup = True
        self.pilot_intent_request_at = None
        self.last_request_at = None
        self.phase_reason = "OFFBOARD异常退出清理：" + reason + "；禁止新动作"

    def _offboard_exit_request_ready(self, now):
        flight = self.flight
        if (flight is None or not flight.connected
                or not is_fresh(self.flight_at, self.state_timeout_s, now)):
            return False
        if flight.mode.upper() not in ("OFFBOARD", self._pilot_mode()):
            return False  # Never override a third mode selected by the pilot/failsafe.
        if not flight.armed:
            return flight.mode.upper() == self._pilot_mode()  # Same-mode only.
        channels = getattr(self, "rc_channels", None)
        if (channels is None or len(channels) < 4
                or not is_fresh(self.rc_at, self.rc_timeout_s, now)
                or not all(800 <= v <= 2200 for v in channels[:4])):
            return False
        if flight.mode.upper() == "OFFBOARD":
            return self._entry_cancel_handover_ready(now)
        e = self.estimator
        if (e is None or not is_fresh(self.estimator_at, self.estimator_timeout_s, now)
                or not e.attitude_status_flag or not e.velocity_vert_status_flag):
            return False
        return self._pilot_mode() != "POSCTL" or bool(e.velocity_horiz_status_flag
            and (e.pos_horiz_rel_status_flag or e.pos_horiz_abs_status_flag)
            and not e.const_pos_mode_status_flag)

    def _tick_offboard_exit(self, now, monotonic_now, flight_fresh, flight_mode):
        pilot = self._pilot_mode()
        if not flight_fresh or self.flight is None or not self.flight.connected:
            self.phase_reason = "OFFBOARD异常退出清理：等待新鲜飞控反馈；禁止新动作"
            return
        if flight_mode not in (pilot, "OFFBOARD"):
            self.phase_reason = "OFFBOARD异常退出清理：尊重当前" + flight_mode + "；等待飞手回到" + pilot
            return
        if flight_mode == "OFFBOARD":
            self.cancel_entry_pilot_samples = 0
            if self._offboard_exit_request_ready(now):
                if self.cancel_entry_stable_since is None:
                    self.cancel_entry_stable_since = monotonic_now
                if monotonic_now - self.cancel_entry_stable_since >= .3:
                    self._request_mode(pilot)
            else:
                self.cancel_entry_stable_since = None
            self.phase_reason = "OFFBOARD异常退出清理：旧模式再次生效，等待安全交回" + pilot + "；禁止新动作"
            return
        self.cancel_entry_stable_since = None
        submitted = getattr(self, "pilot_intent_request_at", None)
        if submitted is None:
            if self._offboard_exit_request_ready(now):
                # A failsafe fallback reports POSCTL but may retain OFFBOARD intention.
                self._request_mode(pilot)
            self.phase_reason = "OFFBOARD异常退出清理：飞控为" + pilot + "，等待明确重申该模式以清除旧OFFBOARD意图"
            return
        anchor = max(submitted, self.cancel_entry_started,
                     getattr(self, "last_offboard_finished_at", None) or self.cancel_entry_started)
        if (not getattr(self, "request_in_flight", False)
                and self.cancel_entry_pilot_samples >= 2
                and monotonic_now - anchor >= self.enter_timeout_s):
            self._reset(lock=True, reason="OFFBOARD退出清理完成：已明确请求并连续确认" + pilot + "；释放旧请求后可重试")
        else:
            self.phase_reason = "OFFBOARD异常退出清理：已请求" + pilot + "，等待后续连续模式反馈及在途请求结束；禁止新动作"

    def _entry_cancel_handover_ready(self, now):
        # If a behavior acquired ownership meanwhile, retain its full recovery gates.
        if self.owner != 0:
            return self._handover_ready(now)
        if (self.output_mode != 2 or not is_fresh(self.output_at, self.output_timeout_s, now)
                or not is_fresh(self.owner_at, self.output_timeout_s, now)
                or self.flight is None or not self.flight.connected or not self.flight.armed
                or not is_fresh(self.flight_at, self.state_timeout_s, now)
                or self.flight.mode.upper() != "OFFBOARD" or not self._rc_fresh(now)):
            return False
        e, odom = self.estimator, self.odom
        if (e is None or not is_fresh(self.estimator_at, self.estimator_timeout_s, now)
                or not e.attitude_status_flag or not e.velocity_vert_status_flag
                or odom is None or not is_fresh(self.odom_at, self.odom_timeout_s, now)
                or not odometry_is_finite(odom)):
            return False
        if self._pilot_mode() == "POSCTL" and not (e.velocity_horiz_status_flag
                and (e.pos_horiz_rel_status_flag or e.pos_horiz_abs_status_flag)
                and not e.const_pos_mode_status_flag):
            return False
        q = odom.pose.pose.orientation
        roll, pitch, _ = euler_from_quaternion((q.x, q.y, q.z, q.w))
        angular = odom.twist.twist.angular
        return bool(math.cos(roll)*math.cos(pitch) >= math.cos(math.radians(5.))
            and abs(vertical_speed_enu_mps(odom, self.odom_twist_in_child_frame)) <= self.handover_max_vertical_speed_mps
            and all(abs(v) <= .3 for v in (angular.x, angular.y, angular.z)))

    def _tick_entry_cancel(self, now, monotonic_now, flight_fresh, flight_mode):
        if getattr(self, "offboard_exit_cleanup", False):
            return self._tick_offboard_exit(now, monotonic_now, flight_fresh, flight_mode)
        if not flight_fresh or self.flight is None or not self.flight.connected:
            self.phase_reason = "取消接管处理中：等待新鲜飞控模式反馈；禁止新动作"
            return
        if flight_mode == "OFFBOARD":
            self.cancel_entry_pilot_samples = 0
            if self._entry_cancel_handover_ready(now):
                if getattr(self, "cancel_entry_stable_since", None) is None:
                    self.cancel_entry_stable_since = monotonic_now
                if monotonic_now - self.cancel_entry_stable_since >= .3:
                    self._request_mode(self._pilot_mode())
            else:
                self.cancel_entry_stable_since = None
            self.phase_reason = "取消接管处理中：已检测到OFFBOARD，进行有界竖直闭环稳定，等待安全交回" + self._pilot_mode()
        elif flight_mode == self._pilot_mode():
            self.cancel_entry_stable_since = None
            # Two new FCU heartbeats, quiescent service worker and a full entry
            # timeout window prevent a stale pilot-mode sample ending cleanup.
            finished = getattr(self, "last_offboard_finished_at", None)
            anchor = max(self.cancel_entry_started, finished or self.cancel_entry_started)
            if (not getattr(self, "request_in_flight", False)
                    and getattr(self, "cancel_entry_pilot_samples", 0) >= 2
                    and monotonic_now - anchor >= getattr(self, "enter_timeout_s", 6.)):
                self._reset(lock=True, reason="取消接管已完成；飞控确认" + self._pilot_mode() + "，释放旧请求后可重试")
            else:
                self.phase_reason = "取消接管处理中：飞控为" + self._pilot_mode() + "，等待在途请求结束及连续模式确认"
        else:
            # Respect a pilot-selected third mode; keep the tombstone to catch a
            # late OFFBOARD, but never force a mode change from another pilot mode.
            self.phase_reason = "取消接管处理中：飞控为" + flight_mode + "；等待飞手回到" + self._pilot_mode()

    def _rc_fresh(self, now):
        # Frame validity is independent of optional calibrated centering channels.
        channels = getattr(self, "rc_channels", None)
        if (channels is None
                or not is_fresh(getattr(self, "rc_at", None), self.rc_timeout_s, now)):
            return False
        if not getattr(self, "require_rc_center", False):
            return bool(len(channels) >= 4
                        and all(800 <= value <= 2200 for value in channels[:4]))
        indices = (self.rc_roll_channel_index, self.rc_pitch_channel_index,
                   self.rc_throttle_channel_index)
        return bool(len(set(indices)) == 3
                    and all(0 <= index < len(channels) for index in indices)
                    and all(800 <= channels[index] <= 2200 for index in indices))

    def _ceiling_baro_handover_ready(self, now):
        status = getattr(self, "ceiling", None)
        return bool(getattr(self, "ceiling_baro_handover_enabled", False)
                    and self.selected_mode == OperatorCommand.MODE_CEILING
                    and status is not None and is_fresh(self.ceiling_at, self.status_timeout_s, now)
                    and status.state == CeilingAttachmentStatus.STATE_DETACH
                    and status.detach_height_ready and status.baro_fresh
                    and status.baro_reference_ready and not status.detach_failed
                    and not status.fault_detected)

    def _ceiling_approach_timeout_requested(self, now):
        status = getattr(self, "ceiling", None)
        return bool(
            self.selected_mode == OperatorCommand.MODE_CEILING
            and status is not None
            and is_fresh(self.ceiling_at, self.status_timeout_s, now)
            and status.state == CeilingAttachmentStatus.STATE_FAULT
            and getattr(status, "approach_timed_out", False)
        )

    def _wall_approach_timeout_requested(self, now):
        wall=getattr(self, "wall", None)
        return bool(self.selected_mode == OperatorCommand.MODE_WALL
            and wall is not None and is_fresh(self.wall_at,self.status_timeout_s,now)
            and wall.state == WallPerchStatus.STATE_ABORT
            and str(wall.reason).startswith("自动接近超时"))

    def _wall_timeout_handover_failure(self, now):
        if not kill_clear(self,now):return "急停生效或反馈无效"
        wall=getattr(self,"wall",None)
        recovered=bool(getattr(self,"wall_approach_timeout_returning",False)
            and self.selected_mode == OperatorCommand.MODE_WALL and wall is not None
            and is_fresh(self.wall_at,self.status_timeout_s,now)
            and wall.state == WallPerchStatus.STATE_EXIT
            and getattr(wall, "fault_recovery_completed", False))
        if not self._wall_approach_timeout_requested(now) and not recovered:
            return "等待新鲜的接近超时/恢复完成状态"
        f=self.flight
        if (f is None or not f.connected or not f.armed
                or not is_fresh(self.flight_at,self.state_timeout_s,now)
                or f.mode.upper() != "OFFBOARD"):
            return "等待新鲜、已解锁的OFFBOARD反馈"
        if (self.owner != 2 or not is_fresh(self.owner_at,self.output_timeout_s,now)
                or self.output_mode != 2 or not is_fresh(self.output_at,self.output_timeout_s,now)):
            return "等待连续受限姿态输出"
        channels=getattr(self,"rc_channels",None)
        if (channels is None or len(channels)<4
                or not is_fresh(self.rc_at,self.rc_timeout_s,now)
                or not all(800<=v<=2200 for v in channels[:4])):
            return "遥控反馈缺失或过期"
        e=self.estimator
        if (e is None or not is_fresh(self.estimator_at,self.estimator_timeout_s,now)
                or not e.attitude_status_flag or not e.velocity_vert_status_flag):
            return "PX4姿态或竖直速度估计未有效"
        if (self._pilot_mode() == "POSCTL" and not (e.velocity_horiz_status_flag
                and (e.pos_horiz_rel_status_flag or e.pos_horiz_abs_status_flag)
                and not e.const_pos_mode_status_flag)):
            return "PX4位置/水平速度估计未有效，不能自动交回POSCTL"
        odom=self.odom
        if (odom is None or not is_fresh(self.odom_at,self.odom_timeout_s,now)
                or not odometry_is_finite(odom)):
            return "里程计缺失或无效"
        q=odom.pose.pose.orientation
        if not .5<=sum(v*v for v in (q.x,q.y,q.z,q.w))<=1.5:return "姿态四元数无效"
        roll,pitch,_=euler_from_quaternion((q.x,q.y,q.z,q.w))
        if math.cos(roll)*math.cos(pitch)<math.cos(math.radians(5.)):
            return "等待总倾角≤5度"
        a=odom.twist.twist.angular
        if math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z)>math.radians(10.):return "等待角速度≤10度/秒"
        speed=vertical_speed_enu_mps(odom,self.odom_twist_in_child_frame)
        if not math.isfinite(speed) or abs(speed)>self.handover_max_vertical_speed_mps:
            return "等待竖直速度进入原有交接范围"
        return None

    def _wall_timeout_handover_ready(self, now):
        if self._wall_timeout_handover_failure(now) is not None:
            self.wall_timeout_stable_since=self.wall_timeout_last_odom=None
            self.wall_timeout_safe_samples=0
            return False
        last=getattr(self,"wall_timeout_last_odom",None)
        if last is not None and not 0 <= (self.odom_at-last).to_sec() <= self.odom_timeout_s:
            self.wall_timeout_stable_since=None;self.wall_timeout_safe_samples=0
        if self.odom_at != last:
            self.wall_timeout_last_odom=self.odom_at
            self.wall_timeout_safe_samples=getattr(self,"wall_timeout_safe_samples",0)+1
            if getattr(self,"wall_timeout_stable_since",None) is None:
                self.wall_timeout_stable_since=self.odom_at
        return bool(self.wall_timeout_safe_samples>=3 and self.wall_timeout_stable_since is not None
            and (self.odom_at-self.wall_timeout_stable_since).to_sec()>=.3)

    def _wall_return_requires_exit(self):
        return bool(getattr(self, "selected_mode", None) == OperatorCommand.MODE_WALL
                    and (getattr(self, "wall_return_episode", False)
                         or getattr(self, "detach_seen", False)
                         or getattr(self, "recovery_seen", False)
                         or getattr(self, "completed", False)
                         or getattr(self, "wall_fault_return_ready", False)))

    def _wall_level_handover_selected(self):
        wall = getattr(self, 'wall', None)
        return bool(getattr(self, 'wall_level_handover_enabled', False)
                    and self.selected_mode == OperatorCommand.MODE_WALL and wall is not None
                    and wall.state in (WallPerchStatus.STATE_DETACH_ROTATE,
                        WallPerchStatus.STATE_RECOVER, WallPerchStatus.STATE_ABORT, WallPerchStatus.STATE_EXIT))

    def _wall_level_handover_failure(self, now, odom, received):
        if not self._wall_level_handover_selected():return "等待脱离、恢复或ABORT回平阶段"
        if not kill_clear(self, now):return "急停生效或反馈无效"
        if (not is_fresh(self.wall_at, self.status_timeout_s, now)
                or not is_fresh_source(self.wall.header.stamp, self.status_timeout_s, now, future_tolerance_s=.1)):
            return "侧吸阶段反馈过期"
        flight = self.flight
        if (flight is None or not flight.connected or not flight.armed or flight.mode.upper() != 'OFFBOARD'
                or not is_fresh(self.flight_at, self.state_timeout_s, now)):
            return "等待新鲜、已解锁的OFFBOARD反馈"
        if (self.owner != 2 or self.output_mode != 2
                or not is_fresh(self.owner_at, self.output_timeout_s, now)
                or not is_fresh(self.output_at, self.output_timeout_s, now)):
            return "等待侧吸连续姿态输出"
        if not self._rc_fresh(now):return "遥控反馈缺失或过期"
        estimate = self.estimator
        if (estimate is None or not is_fresh(self.estimator_at, self.estimator_timeout_s, now)
                or not estimate.attitude_status_flag or not estimate.velocity_vert_status_flag):
            return "PX4姿态或竖直估计未有效"
        if (self._pilot_mode() == 'POSCTL' and not (estimate.velocity_horiz_status_flag
                and (estimate.pos_horiz_rel_status_flag or estimate.pos_horiz_abs_status_flag)
                and not estimate.const_pos_mode_status_flag)):
            return "PX4位置/水平速度估计未有效，不能交回POSCTL"
        sample_timeout = min(.20, self.odom_timeout_s)
        if (odom is None or not is_fresh(received, sample_timeout, now)
                or not is_fresh_source(odom.header.stamp, sample_timeout, now, future_tolerance_s=.1)
                or not odometry_is_finite(odom)):
            return "里程计缺失、过期或无效"
        q = odom.pose.pose.orientation
        norm = sum(v*v for v in (q.x,q.y,q.z,q.w))
        if not .5 <= norm <= 1.5:return "姿态四元数无效"
        # Total thrust-axis tilt, not separate roll/pitch limits. Magnitudes of
        # angular/horizontal/vertical velocity deliberately do not gate return.
        cos_tilt = 1. - 2.*(q.x*q.x+q.y*q.y)/norm
        if cos_tilt < math.cos(math.radians(self.wall_level_handover_tilt_deg))-1.e-12:
            return "等待实际总倾角≤%.1f度" % self.wall_level_handover_tilt_deg
        return None

    def _wall_level_handover_ready(self, now):
        if not hasattr(self, 'wall_level_lock'):self.wall_level_lock = threading.RLock()
        with self.wall_level_lock:
            odom, received = getattr(self, 'handover_odom_snapshot', (self.odom, self.odom_at))
            failure = self._wall_level_handover_failure(now, odom, received)
            if failure:
                self.wall_level_window = None
                self.wall_level_handover_reason = failure
                return False
            # Integer timestamps avoid cancellation at epoch-sized ROS times.
            source = odom.header.stamp.to_nsec()
            arrival = received.to_nsec()
            hold_ns = int(round(self.wall_level_handover_hold_s*1.e9))
            attempt = getattr(self, 'takeover_attempt', 0)
            window = getattr(self, 'wall_level_window', None)
            if window is not None and window[0] == attempt:
                _, first_source, first_arrival, last_source, last_arrival, samples = window
                if source < last_source or arrival < last_arrival:
                    self.wall_level_window = None
                    self.wall_level_handover_reason = "回平样本乱序，重新确认"
                    return False
                if source == last_source:
                    # Neither a repeated callback nor the service worker can
                    # extend the confirmation window using the same source frame.
                    return min(last_source-first_source,last_arrival-first_arrival)+1 >= hold_ns and samples >= 3
                if source-last_source > 200000001 or arrival-last_arrival > 200000001:
                    window = None
            else:window = None
            if window is None:
                first_source, first_arrival, samples = source, arrival, 0
            samples += 1
            self.wall_level_window = (attempt, first_source, first_arrival, source, arrival, samples)
            duration_ns = min(source-first_source, arrival-first_arrival)
            duration = duration_ns/1.e9
            ready = samples >= 3 and duration_ns+1 >= hold_ns
            self.wall_level_handover_reason = ("回平确认完成，申请交回" + self._pilot_mode() if ready
                else "实际总倾角≤%.1f度确认%.2f/%.2f秒" % (self.wall_level_handover_tilt_deg, duration, self.wall_level_handover_hold_s))
            return ready

    def _wall_level_return_reason(self, confirmed=False):
        text = ("已确认交回" if confirmed else "回平快速交接，等待PX4确认") + self._pilot_mode()
        if getattr(self, 'fault_seen', False):
            text += "；本次含ABORT/故障，未标记正常脱离完成"
        else:
            text += "；未等待高度、速度及离墙验收完成"
        return text

    def _wall_return_exit_ready(self, now):
        status = getattr(self, "wall", None)
        if (status is None or not is_fresh(getattr(self, "wall_at", None), self.status_timeout_s, now)
                or status.state != WallPerchStatus.STATE_EXIT):
            return False
        return bool(getattr(status, "fault_recovery_completed", False)
                    or (getattr(status, "detach_completed", False)
                        and getattr(status, "recovery_completed", False)
                        and not status.fault_detected and not getattr(self, "fault_seen", False)))

    def _handover_ready(self, now):
        if getattr(self, 'wall_level_returning', False):
            return self._wall_level_handover_ready(now)
        if getattr(self,"wall_approach_timeout_returning",False):
            return self._wall_timeout_handover_ready(now)
        # A normal EXIT can be revoked while waiting for POSCTL. Read the live
        # terminal status again here, including the service worker's recheck.
        if self._wall_return_requires_exit() and not self._wall_return_exit_ready(now):
            return False
        if (getattr(self,"selected_mode",None) == OperatorCommand.MODE_WALL and not self._wall_horizontal_ready(now,stable=True)):
            return False
        if self.output_mode != 2 or not is_fresh(
            self.output_at, self.output_timeout_s, now
        ):
            return False
        if self.estimator is None or not is_fresh(
            self.estimator_at, self.estimator_timeout_s, now
        ) or not (
            self.estimator.attitude_status_flag
            and self.estimator.velocity_vert_status_flag
        ):
            return False
        if self.odom is None or not is_fresh(
            self.odom_at, self.odom_timeout_s, now
        ) or not odometry_is_finite(self.odom):
            return False
        if getattr(self, "ceiling_approach_timeout_returning", False):
            # This is an aborted approach, so there is no HOLD pressure baseline.
            # Keep the FAULT stream until a live ALTCTL response. Valid RC input
            # is required, but descent speed/centering must not prevent return.
            return bool(
                self._ceiling_approach_timeout_requested(now)
                and self.flight is not None
                and is_fresh(self.flight_at, self.state_timeout_s, now)
                and self.flight.connected and self.flight.armed
                and self.flight.mode.upper() == "OFFBOARD"
                and self.owner == 1
                and is_fresh(self.owner_at, self.output_timeout_s, now)
                and self._rc_fresh(now)
            )
        if getattr(self, "ceiling_baro_handover_enabled", False) and self.selected_mode == OperatorCommand.MODE_CEILING:
            return self._ceiling_baro_handover_ready(now)  # No legacy speed/settle fallback.
        orientation = self.odom.pose.pose.orientation
        roll, pitch, _yaw = euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )
        limit = math.radians(self.handover_max_tilt_deg)
        vertical_speed = vertical_speed_enu_mps(
            self.odom, self.odom_twist_in_child_frame
        )
        angular = self.odom.twist.twist.angular
        if any(not math.isfinite(v) or abs(v) > 0.3 for v in (angular.x, angular.y, angular.z)):
            return False
        if (
            abs(roll) > limit or abs(pitch) > limit
            or not math.isfinite(vertical_speed)
            or (not getattr(self, "wall_bench_wait_for_ranges", False)
                and abs(vertical_speed) > self.handover_max_vertical_speed_mps)
        ):
            return False
        if not self._rc_fresh(now):
            return False
        if not getattr(self, "require_rc_center", False):
            return True
        return all(
            abs(self.rc_channels[index] - self.rc_pwm_center)
            <= self.rc_center_tolerance_pwm
            for index in (
                self.rc_roll_channel_index,
                self.rc_pitch_channel_index,
                self.rc_throttle_channel_index,
            )
        )

    def _observe_completion(self, now):
        selected_owner = (
            1 if self.selected_mode == OperatorCommand.MODE_CEILING else 2
        )
        if self.owner == selected_owner and is_fresh(
            self.owner_at, self.output_timeout_s, now
        ):
            self.owner_seen = True
        if self.selected_mode == OperatorCommand.MODE_CEILING:
            status, received_at = self.ceiling, self.ceiling_at
            if status is None or not is_fresh(received_at, self.status_timeout_s, now):
                return
            if status.state == CeilingAttachmentStatus.STATE_FAULT or status.detach_failed:
                self.fault_seen = True
            if status.state == CeilingAttachmentStatus.STATE_DETACH:
                self.detach_seen = True
                if self._ceiling_baro_handover_ready(now):
                    self.completed = True
            elif self.detach_seen and status.state == CeilingAttachmentStatus.STATE_RECOVERY_HOVER:
                self.recovery_seen = True
            elif (
                self.recovery_seen
                and status.state == CeilingAttachmentStatus.STATE_NORMAL_FLIGHT
                and not status.detach_failed
                and not status.fault_detected
                and not self.fault_seen
            ):
                self.completed = True
        elif self.selected_mode == OperatorCommand.MODE_WALL:
            status, received_at = self.wall, self.wall_at
            self.completed = False
            self.wall_fault_return_ready = False
            if status is None or not is_fresh(received_at, self.status_timeout_s, now):
                return
            if (status.state in (WallPerchStatus.STATE_DETACH_ROTATE, WallPerchStatus.STATE_RECOVER)
                    or getattr(status, "recovery_completed", False)
                    or getattr(status, "fault_recovery_completed", False)):
                self.wall_return_episode = True
            if status.state == WallPerchStatus.STATE_ABORT or status.fault_detected:
                self.fault_seen = True
                if status.state == WallPerchStatus.STATE_ABORT or not getattr(self, "wall_failure_reason", ""):
                    self.wall_failure_reason = str(status.reason) or "侧吸故障"
                self.completed = False
            if status.state == WallPerchStatus.STATE_DETACH_ROTATE:
                self.detach_seen = True
            elif self.detach_seen and status.state == WallPerchStatus.STATE_RECOVER:
                self.recovery_seen = True
            if status.state == WallPerchStatus.STATE_EXIT:
                if getattr(status, "fault_recovery_completed", False):
                    # Safe leveling after ABORT authorizes a fault handback,
                    # never a successful detach/recover result. Explicit flags
                    # also cover a manager that missed intermediate samples.
                    self.fault_seen = True
                    self.completed = False
                    self.wall_fault_return_ready = True
                    if not getattr(self, "wall_failure_reason", ""):
                        self.wall_failure_reason = str(status.reason) or "侧吸故障"
                elif (getattr(status, "detach_completed", False)
                      and getattr(status, "recovery_completed", False)
                      and not status.fault_detected and not self.fault_seen):
                    self.completed = True

    def _wall_return_reason(self, confirmed=False):
        if getattr(self, "wall_fault_return_ready", False):
            result = "故障恢复后已安全交回" if confirmed else "故障恢复完成，准备安全交回"
            return result + self._pilot_mode() + "；本次侧吸未正常完成：" + getattr(self, "wall_failure_reason", "侧吸故障")
        result = "正常脱离及恢复完成，已交回" if confirmed else "正常脱离及恢复完成，准备交回"
        return result + self._pilot_mode()

    def _wall_approach_aborted(self, now):
        return bool(self.selected_mode == OperatorCommand.MODE_WALL
                    and self.wall is not None and is_fresh(self.wall_at, self.status_timeout_s, now)
                    and self.wall.state == WallPerchStatus.STATE_ABORT
                    and str(self.wall.reason).startswith("approach front distance "))

    def _wall_flip_started(self, now):
        return bool(
            self.wall is not None
            and is_fresh(self.wall_at, self.status_timeout_s, now)
            and self.wall.state == WallPerchStatus.STATE_FLIP_TO_WALL
            and self.wall.candidate_valid and not self.wall.fault_detected
        )

    def _current_offboard_request(self, monotonic_now):
        request = getattr(self, "offboard_request", None)
        return bool(
            request is not None
            and request[0] == getattr(self, "takeover_attempt", 0)
            and request[1] == self.selected_mode
            and 0.0 <= monotonic_now - request[2] <= self.enter_timeout_s
        )

    def _acquire_failure_reason(self, now):
        if self.selected_mode == OperatorCommand.MODE_CEILING:
            status, received_at, label = self.ceiling, self.ceiling_at, "ceiling"
        elif self.selected_mode == OperatorCommand.MODE_WALL:
            status, received_at, label = self.wall, self.wall_at, "wall"
        else:
            return "attachment mode is not selected"
        if status is None:
            return "{} status unavailable".format(label)
        if not is_fresh(received_at, self.status_timeout_s, now):
            return "{} status timeout".format(label)
        detail = str(getattr(status, "reason", "")).strip()
        return detail or "{} candidate did not request ownership".format(label)

    def _request_mode(self, mode):
        now = time.monotonic()
        retry = (self.wall_level_handover_retry_s
                 if getattr(self, 'wall_level_returning', False) and mode == self._pilot_mode() else self.retry_s)
        if self.request_in_flight or (
            self.last_request_at is not None
            and now - self.last_request_at < retry
        ):
            return
        self.last_request_at = now
        self.request_in_flight = True
        threading.Thread(
            target=self._send_mode_request, args=(mode, getattr(self, "takeover_attempt", 0)), daemon=True
        ).start()

    def _send_mode_request(self, mode, attempt=None):
        try:
            if mode not in ("OFFBOARD", self._pilot_mode()):
                return
            rospy.wait_for_service("/mavros/set_mode", timeout=0.2)
            if not kill_clear(self, rospy.Time.now()):
                return
            if mode == "OFFBOARD":
                if (
                    self.phase != self.ENTERING
                    or self._requested_mode(rospy.Time.now()) != self.selected_mode
                    or not self._preflight_ready(rospy.Time.now(), self.selected_mode)
                    or not self._stream_ready(rospy.Time.now())
                    or (self._gated_wall_selected() and not self._wall_takeover_ready(rospy.Time.now()))
                    or not self._ceiling_takeover_ready(rospy.Time.now())
                ):
                    return
            elif mode == self._pilot_mode():
                if self.phase == self.CANCELING_ENTRY:
                    ready = (self._offboard_exit_request_ready(rospy.Time.now())
                             if getattr(self, "offboard_exit_cleanup", False)
                             else self._entry_cancel_handover_ready(rospy.Time.now()))
                    if not ready:
                        return
                elif self.phase != self.RETURNING or not self._handover_ready(rospy.Time.now()):
                    return
            if attempt is not None and attempt != getattr(self, "takeover_attempt", 0):
                return  # A canceled/previous attempt must not send a late mode change.
            if mode != "OFFBOARD" and self.phase == self.RETURNING:
                # A flight callback can arrive while this worker waits for the
                # service, before the timer retires RETURNING. Never override a
                # pilot-selected mode (or a disconnected/disarmed FCU) using
                # still-fresh EXIT and output feedback from the old mode.
                flight = self.flight
                if (flight is None or not flight.connected or not flight.armed
                        or flight.mode.upper() != "OFFBOARD"
                        or not is_fresh(self.flight_at, self.state_timeout_s, rospy.Time.now())):
                    return
            if mode == "OFFBOARD":
                self.offboard_request = (getattr(self, "takeover_attempt", 0),
                                         self.selected_mode, time.monotonic())
            request = SetModeRequest()
            request.custom_mode = mode
            response = self.set_mode(request)
            if not response.mode_sent:
                rospy.logwarn("PX4 did not accept %s mode request", mode)
            elif (mode != "OFFBOARD" and self.phase == self.CANCELING_ENTRY
                  and getattr(self, "offboard_exit_cleanup", False)
                  and (attempt is None or attempt == self.takeover_attempt)):
                # mode_sent means submitted, not confirmed. Require NEW heartbeats.
                self.cancel_entry_pilot_samples = 0
                self.pilot_intent_request_at = time.monotonic()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("MAVROS set_mode %s failed: %s", mode, exc)
        finally:
            if mode == "OFFBOARD":
                self.last_offboard_finished_at = time.monotonic()
            self.request_in_flight = False

    def _reset(self, lock=False, reason=None):
        self.wall_level_returning = False
        self.wall_level_window = None
        self.wall_approach_timeout_returning = False
        self.wall_timeout_stable_since = self.wall_timeout_last_odom = None
        self.wall_timeout_safe_samples = 0
        self.offboard_exit_cleanup = False
        self.pilot_intent_request_at = None
        self.takeover_attempt = getattr(self, "takeover_attempt", 0) + 1
        self.offboard_request = None
        self.selected_mode = None
        self.manual_wall_wait_started = None
        self.manual_wait_canceled = False
        self.stream_since = None
        self.owner_seen = False
        self.detach_seen = False
        self.recovery_seen = False
        self.completed = False
        self.fault_seen = False
        self.wall_fault_return_ready = False
        self.wall_failure_reason = ""
        self.wall_return_episode = False
        self.lock_until_neutral = lock
        self.ceiling_approach_timeout_returning = False
        self._set_phase(self.LOCKED if lock else self.IDLE, reason=reason)

    def _on_timer(self, _event):
        now = rospy.Time.now()
        if not kill_clear(self, now):
            if self.phase == self.RETURNING and getattr(self,"wall_approach_timeout_returning",False):
                self._begin_offboard_exit("急停生效或反馈无效；输出立即停止",time.monotonic())
            elif self.phase in (self.ENTERING, self.CANCELING_ENTRY):
                self._begin_entry_cancel("急停生效或反馈无效；输出立即停止", time.monotonic())
            else:
                self._reset(lock=True, reason="kill switch active or unavailable")
            self.prestream_pub.publish(Bool(data=False))
            self.phase_pub.publish(String(data=self.phase))
            self.phase_reason_pub.publish(String(data=self.phase_reason))
            return
        monotonic_now = time.monotonic()
        requested = self._requested_mode(now)
        flight_mode = self.flight.mode.upper() if self.flight else ""
        flight_fresh = is_fresh(self.flight_at, self.state_timeout_s, now)
        if (self.phase == self.RETURNING and flight_mode == "OFFBOARD"
                and getattr(self, "selected_mode", None) == OperatorCommand.MODE_WALL
                and not getattr(self, "wall_approach_timeout_returning", False)):
            self._observe_completion(now)
        if (self.phase == self.RETURNING and not getattr(self, 'wall_level_returning', False)
                and self._wall_level_handover_selected() and self._wall_level_handover_ready(now)):
            self.wall_level_returning = True
            self.phase_reason = self._wall_level_return_reason()

        if (self.phase in (self.IDLE, self.LOCKED) and flight_fresh and flight_mode == "OFFBOARD"
                and getattr(self, "canceled_entry_mode", None) is not None):
            self.selected_mode = self.canceled_entry_mode
            self._begin_offboard_exit("检测到已取消请求的迟到OFFBOARD反馈", monotonic_now)
        if self.phase == self.LOCKED and requested is None:
            self._reset(
                lock=False,
                reason="operator request released; retry enabled",
            )
        if self.phase == self.IDLE:
            if (
                requested is not None
                and flight_mode == self._pilot_mode(requested)
                and self._preflight_ready(now, requested)
            ):
                self.takeover_attempt = getattr(self, "takeover_attempt", 0) + 1
                self.offboard_request = None
                self.selected_mode = requested
                if self._gated_wall_selected():
                    self.manual_wall_wait_started = monotonic_now
                    self._set_phase(self.WAITING_WALL, reason=self._manual_wait_reason(now, monotonic_now))
                else:
                    self._set_phase(self.PRESTREAM)
        elif self.phase == self.WAITING_WALL:
            if getattr(self, "manual_wait_canceled", False):
                self._reset(lock=True, reason="手动接近等待已取消；仍由飞手控制 ALTCTL")
                self.cancel_pub.publish(String(data="cancel"))
            elif requested != self.selected_mode:
                self._reset(lock=requested is not None, reason="manual wall request canceled")
            elif self._wall_approach_aborted(now):
                self._reset(lock=True, reason=self.wall.reason)
                self.cancel_pub.publish(String(data="cancel"))
            elif flight_fresh and flight_mode == "OFFBOARD":
                # Recognize this attempt's handover despite delayed phase feedback.
                # An unrelated/stale OFFBOARD change must never start a flip.
                if (self._current_offboard_request(monotonic_now)
                    and self._preflight_ready(now, self.selected_mode)
                    and self._stream_ready(now)):
                    if self._wall_flip_started(now):
                        self._set_phase(self.ACTIVE, reason="本次接管已确认：PX4 OFFBOARD，贴墙节点已开始翻转")
                    else:
                        self._set_phase(self.ENTERING, reason="本次 OFFBOARD 请求已确认；等待贴墙节点及有效距离")
                else:
                    reason = (self._preflight_failure_reason(now, self.selected_mode)
                              or "OFFBOARD 不属于本次有效接管，或预发送反馈已过期")
                    self._begin_entry_cancel(reason, monotonic_now)
            elif flight_mode != self._pilot_mode() or not self._preflight_ready(now, self.selected_mode):
                self._reset(lock=True, reason=self._preflight_failure_reason(now, self.selected_mode) or "manual approach requires " + self._pilot_mode())
                self.cancel_pub.publish(String(data="cancel"))
            elif self._manual_wait_elapsed(monotonic_now) >= self.manual_wall_wait_timeout_s:
                self._reset(lock=True, reason="手动接近等待超时，贴墙请求已取消；仍由飞手控制 ALTCTL")
                self.cancel_pub.publish(String(data="cancel"))
            else:
                # Warm the OFFBOARD stream while the pilot approaches in ALTCTL.
                # Distance changes do not restart this independent stream timer.
                if not self._stream_ready(now):
                    self.stream_since = None
                elif self.stream_since is None:
                    self.stream_since = monotonic_now
                stream_warm = (self.stream_since is not None
                               and monotonic_now - self.stream_since >= self.prestream_s)
                if stream_warm and self._wall_takeover_ready(now):
                    self._set_phase(self.ENTERING)
                    self._request_mode("OFFBOARD")
                elif self._wall_takeover_ready(now):
                    self.phase_reason = "距离条件已满足；等待并行预发送完成，仍由飞手控制 " + self._pilot_mode()
                else:
                    self.phase_reason = self._manual_wait_reason(now, monotonic_now)
        elif self.phase == self.PRESTREAM:
            failure_reason = None
            if requested != self.selected_mode:
                failure_reason = "operator request changed or was released"
            elif flight_mode != self._pilot_mode():
                failure_reason = "flight mode changed from {} to {}".format(
                    self._pilot_mode(), flight_mode or "unknown"
                )
            elif not self._preflight_ready(now, self.selected_mode):
                failure_reason = self._preflight_failure_reason(
                    now, self.selected_mode
                )
            if failure_reason is not None:
                rospy.logwarn("OFFBOARD prestream stopped: %s", failure_reason)
                self._reset(
                    lock=requested is not None,
                    reason=failure_reason,
                )
            elif self._stream_ready(now):
                if self.stream_since is None:
                    self.stream_since = monotonic_now
                elif monotonic_now - self.stream_since >= self.prestream_s:
                    if self._ceiling_takeover_ready(now):
                        self._set_phase(self.ENTERING)
                        self._request_mode("OFFBOARD")
                    else:
                        self.phase_reason = self._ceiling_wait_reason(now)
            else:
                self.stream_since = None
        elif self.phase == self.ENTERING:
            failure_reason = None
            if requested != self.selected_mode:
                failure_reason = "operator request changed or was released"
            elif self._wall_approach_aborted(now):
                failure_reason = self.wall.reason
            elif not self._preflight_ready(now, self.selected_mode):
                failure_reason = self._preflight_failure_reason(now, self.selected_mode) or "preflight safety lost"
            elif self._entry_stream_failed(now, monotonic_now):
                failure_reason = "OFFBOARD setpoint stream lost beyond entry grace"
            elif monotonic_now - self.phase_since > self.enter_timeout_s:
                failure_reason = "PX4 OFFBOARD entry timed out"
            if failure_reason is not None:
                self._begin_entry_cancel(failure_reason, monotonic_now)
            elif flight_fresh and flight_mode == "OFFBOARD":
                if not self._current_offboard_request(monotonic_now):
                    self._begin_entry_cancel("未确认本次OFFBOARD请求", monotonic_now)
                elif not self._stream_ready(now):
                    self.phase_reason = "本次OFFBOARD已反馈；等待短暂输出间断恢复，禁止开始侧吸"
                elif (self._manual_wall_selected() and not self._manual_wall_ready(now)
                        and not self._wall_flip_started(now)):
                    if getattr(self, "ignore_range_loss_during_attachment", False):
                        self.phase_reason = "接管等待：距离条件失效，保持中性输出，等待有效距离后翻转"
                    else:
                        self._begin_entry_cancel("OFFBOARD确认时距离条件失效", monotonic_now)
                else:
                    self.canceled_entry_mode = None
                    self._set_phase(self.ACTIVE, reason="PX4 OFFBOARD接管已确认；开始受限自动接近" if self._auto_wall_selected() else "PX4 entered OFFBOARD")
            elif self._stream_ready(now):
                self._request_mode("OFFBOARD")
        elif self.phase == self.CANCELING_ENTRY:
            self._tick_entry_cancel(now, monotonic_now, flight_fresh, flight_mode)
        elif self.phase == self.ACTIVE:
            if flight_fresh and flight_mode != "OFFBOARD":
                failure_reason = "PX4 left OFFBOARD for {}".format(
                    flight_mode or "unknown mode"
                )
                rospy.logwarn("%s; automatic reentry inhibited", failure_reason)
                self._begin_offboard_exit(failure_reason, monotonic_now)
            else:
                self._observe_completion(now)
                if self.selected_mode == OperatorCommand.MODE_CEILING and getattr(self, "ceiling", None) is not None and is_fresh(self.ceiling_at, self.status_timeout_s, now):
                    self.phase_reason = self.ceiling.reason
                if (self._auto_wall_selected() and self.wall is not None
                        and is_fresh(self.wall_at, self.status_timeout_s, now)
                        and self.wall.state == WallPerchStatus.STATE_SLOW_APPROACH):
                    self.phase_reason = self.wall.reason
                if self._wall_level_handover_selected() and self._wall_level_handover_ready(now):
                    self.wall_level_returning = True
                    self._set_phase(self.RETURNING, reason=self._wall_level_return_reason())
                    self._request_mode(self._pilot_mode())
                elif self._wall_approach_timeout_requested(now):
                    self.wall_approach_timeout_returning = True
                    self.wall_timeout_stable_since=self.wall_timeout_last_odom=None
                    self.wall_timeout_safe_samples=0
                    self.cancel_pub.publish(String(data="cancel"))
                    self._set_phase(self.RETURNING, reason="接近超时：取消前进，受控交回" + self._pilot_mode())
                    if self._handover_ready(now):self._request_mode(self._pilot_mode())
                elif self._ceiling_approach_timeout_requested(now):
                    self.ceiling_approach_timeout_returning = True
                    self._set_phase(self.RETURNING, reason="吸顶接近超时：FAULT锁存，等待ALTCTL确认后交回遥控器")
                    if self._handover_ready(now):
                        self._request_mode(self._pilot_mode())
                elif self._ceiling_baro_handover_ready(now):
                    self._set_phase(self.RETURNING, reason=self.ceiling.reason)
                    if self._handover_ready(now):
                        self._request_mode(self._pilot_mode())
                elif ((self.completed or (self.selected_mode == OperatorCommand.MODE_WALL
                                         and getattr(self, "wall_fault_return_ready", False)))
                    and self.owner_seen
                    and (self.owner == 0 or (self.selected_mode == OperatorCommand.MODE_WALL and self.owner == 2))
                    and is_fresh(self.owner_at, self.output_timeout_s, now)):
                    self._set_phase(self.RETURNING, reason=(self._wall_return_reason()
                        if self.selected_mode == OperatorCommand.MODE_WALL else None))
                elif (
                    not self.owner_seen
                    and monotonic_now - self.phase_since > self.acquire_timeout_s
                ):
                    detail = self._acquire_failure_reason(now)
                    failure_reason = "attachment owner was not acquired: {}".format(
                        detail
                    )
                    rospy.logerr("%s; returning to ALTCTL", failure_reason)
                    self._set_phase(self.RETURNING, reason=failure_reason)
                elif self._wall_level_handover_selected():
                    self.phase_reason = "回平后交回" + self._pilot_mode() + "；" + self.wall_level_handover_reason
        elif self.phase == self.RETURNING:
            if ((getattr(self, "ceiling_approach_timeout_returning", False)
                    or getattr(self,"wall_approach_timeout_returning",False))
                    and (not flight_fresh or self.flight is None or not self.flight.connected)):
                self.phase_reason = "waiting for fresh connected PX4 mode confirmation"
            elif (getattr(self,"wall_approach_timeout_returning",False)
                    and not getattr(self, 'wall_level_returning', False) and flight_fresh
                    and self.flight is not None and self.flight.connected and flight_mode == self._pilot_mode()):
                self._begin_offboard_exit("接近超时已交回" + self._pilot_mode() + "，清理旧OFFBOARD意图",monotonic_now)
            elif flight_fresh and self.flight.connected and flight_mode == self._pilot_mode():
                self._reset(
                    lock=True,
                    reason=(self._wall_level_return_reason(confirmed=True) + "；释放请求后可重试"
                        if getattr(self, 'wall_level_returning', False)
                        else self._wall_return_reason(confirmed=True) + "；释放请求后可重试"
                        if self.selected_mode == OperatorCommand.MODE_WALL
                        and (self.completed or getattr(self, "wall_fault_return_ready", False))
                        else "returned to {}; release command before retry".format(self._pilot_mode())),
                )
            elif flight_fresh and flight_mode == "OFFBOARD" and self._handover_ready(now):
                self._request_mode(self._pilot_mode())
            elif flight_fresh and flight_mode == "OFFBOARD":
                if getattr(self, 'wall_level_returning', False):
                    self.phase_reason = self._wall_level_return_reason() + "；" + self.wall_level_handover_reason
                elif getattr(self,"wall_approach_timeout_returning",False):
                    detail=self._wall_timeout_handover_failure(now) or "等待姿态及竖直速度连续安全0.30秒"
                    self.phase_reason="接近超时：受控交回" + self._pilot_mode() + "；" + detail
                elif getattr(self, "ceiling_approach_timeout_returning", False):
                    self.phase_reason = "吸顶接近超时：保持FAULT，等待有效遥控输入、飞控估计和连续输出后请求ALTCTL"
                elif self._wall_return_requires_exit() and not self._wall_return_exit_ready(now):
                    self.phase_reason = "离墙条件重新确认中：保持受控输出，等待新的有效EXIT后交回" + self._pilot_mode()
                else:
                    prefix = (self._wall_return_reason() + "；"
                        if getattr(self, "selected_mode", None) == OperatorCommand.MODE_WALL
                        and (self.completed or getattr(self, "wall_fault_return_ready", False)) else "")
                    self.phase_reason = prefix + "等待水平速度、姿态持续稳定和新鲜有效估计后交回" + self._pilot_mode()
            elif flight_fresh:
                failure_reason = "unexpected flight mode during return: {}".format(
                    flight_mode or "unknown"
                )
                rospy.logwarn("%s", failure_reason)
                self._reset(lock=True, reason=failure_reason)

        self.prestream_pub.publish(
            Bool(data=self.phase in (self.WAITING_WALL, self.PRESTREAM, self.ENTERING, self.CANCELING_ENTRY, self.ACTIVE, self.RETURNING)
                 and (not getattr(self, "offboard_exit_cleanup", False) or flight_mode == "OFFBOARD"))
        )
        self.phase_pub.publish(String(data=self.phase))
        phase_reason_pub = getattr(self, "phase_reason_pub", None)
        if phase_reason_pub is not None:
            phase_reason_pub.publish(
                String(data=getattr(self, "phase_reason", "initialized"))
            )


if __name__ == "__main__":
    rospy.init_node("flight_mode_manager_node")
    from px4_wall_ceiling_control.adapters.async_logging import install_ros_async_logging
    install_ros_async_logging(rospy)
    FlightModeManagerNode()
    rospy.spin()
