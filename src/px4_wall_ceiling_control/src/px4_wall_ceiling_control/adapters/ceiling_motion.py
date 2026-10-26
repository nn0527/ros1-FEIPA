"""Ceiling motion feedback, frozen POSCTL entry and sample-based recovery."""
import math
from contextlib import nullcontext
from functools import wraps
import threading

import rospy
from geometry_msgs.msg import Quaternion
from std_msgs.msg import UInt8, String
from mavros_msgs.msg import AttitudeTarget, EstimatorStatus, OpticalFlowRad
from px4_wall_ceiling_control.adapters.odometry import linear_velocity_parent_frame, odometry_is_finite
from px4_wall_ceiling_control.domain.ceiling_motion import CeilingMotionController, CeilingMotionSettings
from px4_wall_ceiling_control.domain.flow_timestamps import FlowTimestampGuard
from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion, rotate_vector
from px4_wall_ceiling_control.domain.horizontal_equilibrium import HorizontalEquilibrium
from px4_wall_ceiling_control.domain.recovery_attitude_limits import attitude_from_axis_yaw
from px4_wall_ceiling_control.domain.sample_hold import held_samples, record_hold_sample
from px4_wall_ceiling_control.freshness import is_fresh_receipt, is_fresh_source, clamp
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus as S, CeilingRcAssist


def locked_ceiling_callback(callback):
    @wraps(callback)
    def locked(self, *args, **kwargs):
        with getattr(self, 'motion_lock', nullcontext()):
            return callback(self, *args, **kwargs)
    return locked


class CeilingMotionMixin:
    def _init_ceiling_motion(self):
        self.motion_lock = threading.RLock()
        self.motion_enabled = bool(rospy.get_param('~motion_control_enabled', False))
        self.motion_settings = CeilingMotionSettings.read(rospy.get_param)
        self.motion_controller = CeilingMotionController(self.motion_settings)
        self.motion_equilibrium = HorizontalEquilibrium(window=self.motion_settings.motion_equilibrium_window_s)
        self.motion_flow_clock = FlowTimestampGuard(self.motion_settings.motion_flow_timeout_s,
            self.motion_settings.motion_flow_future_tolerance_s)
        self.motion_manager_phase = ''
        self.motion_capture_at = None
        self.motion_flow = self.motion_estimator = self.motion_fcu = None
        self.motion_flow_at = self.motion_estimator_at = self.motion_fcu_at = None
        self.motion_flow_error = self.motion_odom_error = self.motion_estimator_error = ''
        self.motion_last_odom = self.motion_last_estimator = None
        self.motion_ready = False
        self.motion_reason = '等待有效光流、位置和水平速度反馈'
        self.motion_epoch = self.motion_epoch_last = None
        self.motion_paused = False
        self.motion_pause_at = self.motion_bad_since = None
        self.motion_resume_since = self.motion_resume_last = None
        self.motion_loss_at = None
        self.motion_drift = self.motion_speed = self.motion_tilt = self.motion_rate = math.nan
        self.motion_fatal = ''
        self.motion_contact_unsafe = False
        self.motion_recovery_ready = False
        self.motion_recovery_attitude = None
        self.motion_recovery_lock_time = rospy.Time(0)
        self.motion_recovery_since = self.motion_recovery_last = None
        self.motion_processed_recovery = self.motion_processed_pause = None
        self.motion_control_sample = None
        self.motion_contact_released = False
        self.motion_release_since = self.motion_release_last = None
        self.motion_release_stamp = None
        self.motion_recovery_height = None
        self.motion_attempt = 0
        self.motion_id = ''
        self.motion_sequence = 0
        self.motion_last_target_vz = 0.
        self.motion_output = self.motion_output_at = None
        self.motion_owner = self.motion_owner_at = None
        self.motion_integrated_output = None
        self.rc_manual_coordination_enabled = bool(rospy.get_param('~rc_manual_coordination_enabled', False))
        self.motion_rc_feedback = self.motion_rc_feedback_at = None
        self.motion_rc_tick = None
        self.motion_rc_weight = 1.
        self.motion_rc_active = False
        self.motion_rc_ready = False
        if self.motion_enabled:
            rospy.Subscriber('/attachment/ceiling_rc_assist', CeilingRcAssist, self._on_ceiling_rc_assist, queue_size=1)
            rospy.Subscriber('/attachment/flight_phase', String, self._on_ceiling_manager_phase, queue_size=1)
            rospy.Subscriber('/attachment/output_owner', UInt8, self._on_ceiling_owner, queue_size=1)
            rospy.Subscriber('/mavros/setpoint_raw/attitude', AttitudeTarget, self._on_ceiling_output, queue_size=1)
            rospy.Subscriber('/mavros/px4flow/raw/optical_flow_rad', OpticalFlowRad,
                             self._on_ceiling_flow, queue_size=10)
            rospy.Subscriber('/mavros/estimator_status', EstimatorStatus,
                             self._on_ceiling_estimator, queue_size=10)
            rospy.Subscriber('/mavros/setpoint_raw/target_attitude', AttitudeTarget,
                             self._on_ceiling_fcu_target, queue_size=10)

    @locked_ceiling_callback
    def _on_ceiling_rc_assist(self, message):
        now = rospy.Time.now()
        if not self._ceiling_rc_feedback_valid(message, now):
            return
        previous = self.motion_rc_feedback
        if (previous is not None and previous.maneuver_id == message.maneuver_id
                and (message.header.stamp <= previous.header.stamp
                     or message.control_sequence < previous.control_sequence)):
            return
        self.motion_rc_feedback, self.motion_rc_feedback_at = message, now

    def _ceiling_rc_feedback_valid(self, message, now):
        timeout = self.motion_settings.rc_manual_feedback_timeout_s
        return bool(message is not None and message.maneuver_id
            and message.maneuver_id == self.motion_id and message.control_phase == self.state
            and 0 < message.control_sequence <= self.motion_sequence
            and is_fresh_source(message.header.stamp, timeout, now, future_tolerance_s=0.)
            and is_fresh_source(message.candidate_stamp, timeout, now, future_tolerance_s=0.)
            and message.candidate_stamp <= message.header.stamp
            and all(math.isfinite(v) for v in (message.roll_trim_rad, message.pitch_trim_rad)))

    def _update_ceiling_manual(self, now):
        previous = self.motion_rc_tick
        self.motion_rc_tick = now
        dt = min(.05, max(0., (now-previous).to_sec())) if previous is not None else 0.
        if not self.rc_manual_coordination_enabled:
            self.motion_rc_weight, self.motion_rc_active, self.motion_rc_ready = 1., False, True
            return
        message = self.motion_rc_feedback
        self.motion_rc_ready = bool(self._ceiling_rc_feedback_valid(message, now)
            and is_fresh_receipt(self.motion_rc_feedback_at, self.motion_settings.rc_manual_feedback_timeout_s, now)
            and message.input_valid)
        self.motion_rc_active = bool(self.motion_rc_ready and (message.stick_active
            or max(abs(message.roll_trim_rad), abs(message.pitch_trim_rad))
               > math.radians(self.motion_settings.rc_manual_zero_deg)))
        # Keep the original position reference. Manual input only reduces its
        # pull; velocity damping and the drift/tilt/rate guards remain active.
        allowed = self.state in (S.STATE_APPROACH, S.STATE_RECOVERY_HOVER) or (
            self.state == S.STATE_DETACH and self.motion_contact_released)
        target = (self.motion_settings.rc_manual_position_weight
                  if allowed and self.motion_rc_active and not self.motion_paused
                  and self.motion_drift < self.motion_settings.horizontal_drift_warn_m else 1.)
        step = (1.-self.motion_settings.rc_manual_position_weight)*dt/self.motion_settings.rc_manual_blend_s
        self.motion_rc_weight += max(-step, min(step, target-self.motion_rc_weight))
        if self.motion_rc_active and allowed:
            self.motion_controller.horizontal.clear_integral()

    def _ceiling_manual_settled(self):
        return (not self.rc_manual_coordination_enabled or (
            self.motion_rc_ready and not self.motion_rc_active and self.motion_rc_weight >= 1.-1e-9))

    @locked_ceiling_callback
    def _on_ceiling_manager_phase(self, message):
        previous = self.motion_manager_phase
        self.motion_manager_phase = message.data
        if (self.motion_enabled and self._mode_requested(rospy.Time.now())
                and message.data == 'ENTERING' and previous != 'ENTERING'):
            now = rospy.Time.now()
            self._prepare_ceiling_motion(now)
            if self.motion_ready and self._motion_target_fresh(now) and self.motion_equilibrium.freeze(now.to_sec()):
                self.motion_capture_at = now
            else:
                self.motion_fatal = 'OFFBOARD请求前平衡基准失效'

    @locked_ceiling_callback
    def _on_ceiling_owner(self, message):
        self.motion_owner, self.motion_owner_at = message.data, rospy.Time.now()

    @locked_ceiling_callback
    def _on_ceiling_output(self, message):
        self.motion_output, self.motion_output_at = message, rospy.Time.now()

    @locked_ceiling_callback
    def _on_ceiling_flow(self, message):
        now = rospy.Time.now()
        self.motion_flow_error = self.motion_flow_clock.observe(message.header.stamp.to_sec(), now.to_sec())
        self.motion_flow, self.motion_flow_at = message, now

    @locked_ceiling_callback
    def _on_ceiling_estimator(self, message):
        stamp = message.header.stamp
        self.motion_estimator_error = ('估计器源时间戳未递增' if self.motion_last_estimator is not None
                                      and stamp <= self.motion_last_estimator else '')
        self.motion_last_estimator = stamp
        self.motion_estimator, self.motion_estimator_at = message, rospy.Time.now()

    @locked_ceiling_callback
    def _on_ceiling_fcu_target(self, message):
        # OFFBOARD target_attitude echoes our command; never learn it as equilibrium.
        if (self.flight_state is not None and self.flight_state.mode.upper() == 'POSCTL'
                and not self.motion_equilibrium.frozen and self.motion_manager_phase not in ('ENTERING', 'ACTIVE', 'RETURNING', 'CANCELING_ENTRY')):
            if self.motion_fcu is not None and message.header.stamp <= self.motion_fcu.header.stamp:
                self.motion_equilibrium.clear()
                self.motion_fcu_at = None
                return
            self.motion_fcu, self.motion_fcu_at = message, rospy.Time.now()

    def _motion_observe_odom(self, message):
        if not getattr(self, 'motion_enabled', False):
            return
        stamp = message.header.stamp
        previous = self.motion_last_odom
        self.motion_odom_error = ''
        if previous is not None and stamp <= previous.header.stamp:
            self.motion_odom_error = '里程计源时间戳未递增'
        elif previous is not None and self.motion_controller.xy is not None and odometry_is_finite(message):
            a, b = previous.pose.pose.position, message.pose.pose.position
            interval = (stamp-previous.header.stamp).to_sec()
            velocity = linear_velocity_parent_frame(message, self.odom_twist_in_child_frame)
            jump = math.hypot(b.x-a.x, b.y-a.y)
            old_yaw = euler_from_quaternion(tuple(getattr(previous.pose.pose.orientation, k) for k in "xyzw"))[2]
            yaw = euler_from_quaternion(tuple(getattr(message.pose.pose.orientation, k) for k in "xyzw"))[2]
            yaw_jump = abs(math.atan2(math.sin(yaw-old_yaw), math.cos(yaw-old_yaw)))
            if (jump > self.motion_settings.odom_position_jump_m + math.hypot(*velocity[:2])*max(0., interval)
                    or yaw_jump > math.radians(self.motion_settings.odom_yaw_jump_deg)):
                self.motion_fatal = '位置或航向估计跳变：本轮锁存故障，禁止追赶旧参考点'
        self.motion_last_odom = message

    def _motion_observe_state(self, message):
        if not getattr(self, 'motion_enabled', False):
            return
        previous = self.flight_state.mode.upper() if self.flight_state is not None else ''
        if (message.mode.upper() == 'OFFBOARD' and previous == 'POSCTL'
                and self._mode_requested(rospy.Time.now())):
            now = rospy.Time.now()
            self._prepare_ceiling_motion(now)
            if (self.motion_ready and self.motion_equilibrium.freeze(now.to_sec())
                    and self._motion_target_fresh(now)):
                q = tuple(getattr(self.odom.pose.pose.orientation, k) for k in "xyzw")
                yaw = euler_from_quaternion(q)[2]
                p = self.odom.pose.pose.position
                self.motion_controller.capture((p.x, p.y), yaw, self.motion_equilibrium.axis, self.motion_fcu.thrust)
                self.q_hover = Quaternion(*attitude_from_axis_yaw(self.motion_equilibrium.axis, yaw))
                self.motion_attempt += 1
                self.motion_id = 'ceiling-%.9f-%d' % (now.to_sec(), self.motion_attempt)
                self.motion_sequence = 0
            else:
                self.motion_fatal = 'POSCTL接管基准失效，禁止归零姿态接管'
        elif not message.armed or (message.mode.upper() == 'POSCTL' and previous == 'OFFBOARD'):
            # Clear only on the confirmed return, never because a timer recovered.
            self.motion_equilibrium.clear()
            self.motion_capture_at = None
            self.motion_controller.reset()
            self.motion_fatal = ''

    def _motion_target_fresh(self, now):
        t = self.motion_fcu
        if t is None:
            return False
        q = tuple(getattr(t.orientation, k) for k in 'xyzw')
        frozen_valid = (self.motion_equilibrium.frozen and self.motion_capture_at is not None
                        and is_fresh_receipt(self.motion_capture_at, self.motion_settings.motion_capture_timeout_s, now))
        return bool((frozen_valid or (is_fresh_receipt(self.motion_fcu_at, self.motion_settings.motion_fcu_target_timeout_s, now)
            and is_fresh_source(t.header.stamp, self.motion_settings.motion_fcu_target_timeout_s, now)))
            and not t.type_mask & (AttitudeTarget.IGNORE_ATTITUDE | AttitudeTarget.IGNORE_THRUST)
            and all(math.isfinite(v) for v in q) and .5 < sum(v*v for v in q) < 1.5
            and math.isfinite(t.thrust) and .10 <= t.thrust <= self.max_thrust)

    def _ceiling_feedback_error(self, now):
        s = self.motion_settings
        if not self._odom_fresh(now) or self.odom is None or not odometry_is_finite(self.odom):
            return '里程计无效或过期'
        if self.motion_odom_error or not is_fresh_source(self.odom.header.stamp, self.odom_timeout_s, now):
            return self.motion_odom_error or '里程计源时间过期'
        e = self.motion_estimator
        if (e is None or not is_fresh_receipt(self.motion_estimator_at, s.motion_estimator_timeout_s, now)
                or not is_fresh_source(e.header.stamp, s.motion_estimator_timeout_s, now)):
            return '估计器反馈无效或过期'
        if self.motion_estimator_error:
            return self.motion_estimator_error
        if not (e.attitude_status_flag and e.velocity_horiz_status_flag and e.velocity_vert_status_flag
                and (e.pos_horiz_rel_status_flag or e.pos_horiz_abs_status_flag)) or e.const_pos_mode_status_flag:
            return '位置或速度估计未有效'
        f = self.motion_flow
        if f is None or not is_fresh_receipt(self.motion_flow_at, s.motion_flow_timeout_s, now):
            return '光流过期'
        if (f.quality < s.motion_flow_quality_min or f.integration_time_us <= 0
                or not all(math.isfinite(v) for v in (f.integrated_x, f.integrated_y,
                    f.integrated_xgyro, f.integrated_ygyro, f.distance)) or f.distance <= 0):
            return '光流质量、积分或测距无效'
        return self.motion_flow_clock.reason(f.header.stamp.to_sec(), now.to_sec(), self.motion_flow_error)

    def _prepare_ceiling_motion(self, now):
        if not getattr(self, 'motion_enabled', False):
            return
        reason = self._ceiling_feedback_error(now)
        if reason:
            self.motion_epoch = self.motion_epoch_last = None
            self.motion_ready = False
            self.motion_controller.horizontal.clear_integral()
            self.motion_reason = reason
        else:
            # Both sample streams must span the reacquisition interval; timers cannot qualify one frame.
            sample = min(self.odom.header.stamp, self.motion_flow.header.stamp)
            if self.motion_epoch is None:
                self.motion_epoch = sample
            self.motion_epoch_last = sample
            self.motion_ready = held_samples(True, self.motion_epoch, sample, self.motion_settings.motion_reacquire_time_s)
            self.motion_reason = '位置/速度/光流反馈有效' if self.motion_ready else '等待连续新反馈重新准入'
        if self._odom_fresh(now) and self.odom is not None and odometry_is_finite(self.odom):
            q = tuple(getattr(self.odom.pose.pose.orientation, k) for k in "xyzw")
            v = linear_velocity_parent_frame(self.odom, self.odom_twist_in_child_frame)
            self.motion_speed = math.hypot(*v[:2])
            self.motion_tilt = math.degrees(math.acos(clamp(rotate_vector(q, (0., 0., 1.))[2], -1., 1.)))
            a = self.odom.twist.twist.angular
            self.motion_rate = math.degrees(math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z))
            p = self.odom.pose.pose.position
            self.motion_drift = self.motion_controller.drift((p.x, p.y))
        if self.flight_state is not None and self.flight_state.mode.upper() == 'POSCTL' and not self.motion_equilibrium.frozen:
            eligible = (self.motion_ready and self.motion_speed <= self.motion_settings.motion_equilibrium_speed_mps
                        and self.motion_rate <= 20. and self._motion_target_fresh(now))
            self.motion_equilibrium.observe(self.motion_fcu_at.to_sec() if self.motion_fcu_at is not None else 0.,
                tuple(getattr(self.motion_fcu.orientation, k) for k in 'xyzw') if self.motion_fcu is not None else (0., 0., 0., 1.), eligible)
        if self.motion_controller.xy is not None:
            self._update_ceiling_pause(now)
        self._update_ceiling_manual(now)

    def _update_ceiling_pause(self, now):
        s = self.motion_settings
        free = self.state in (S.STATE_CEILING_ARMED, S.STATE_APPROACH, S.STATE_DETACH, S.STATE_RECOVERY_HOVER, S.STATE_ABORT) and not (self.state == S.STATE_ABORT and getattr(self, "hold_timed_out", False))
        if not free:
            self.motion_controller.horizontal.clear_integral()
            self.motion_pause_at = self.motion_loss_at = None
            self.motion_paused = False
            if self._odom_fresh(now) and self.odom.header.stamp != self.motion_processed_pause:
                self.motion_processed_pause = self.odom.header.stamp
                bad = self.motion_tilt > s.actual_tilt_pause_deg or self.motion_rate > s.actual_body_rate_pause_deg_s
                self.motion_bad_since, last = record_hold_sample(bad, self.odom.header.stamp, self.motion_bad_since)
                self.motion_contact_unsafe = held_samples(True, self.motion_bad_since, last, s.motion_limit_confirm_s)
            return
        if not self.motion_ready:
            self.motion_loss_at = self.motion_loss_at or now
            self.motion_paused = True
            self.motion_resume_since = self.motion_resume_last = None
        else:
            self.motion_loss_at = None
        # Confirm measured limits only with distinct odometry samples.
        stamp = self.odom.header.stamp if self.odom is not None else None
        if self.motion_ready and stamp != self.motion_processed_pause:
            self.motion_processed_pause = stamp
            bad = (self.motion_drift > s.horizontal_drift_pause_m or self.motion_tilt > s.actual_tilt_pause_deg
                   or self.motion_rate > s.actual_body_rate_pause_deg_s)
            self.motion_bad_since, bad_last = record_hold_sample(bad, stamp, self.motion_bad_since)
            if held_samples(True, self.motion_bad_since, bad_last, s.motion_limit_confirm_s):
                self.motion_paused = True
            good = (self.motion_drift <= s.horizontal_drift_resume_m and self.motion_tilt <= s.actual_tilt_resume_deg
                    and self.motion_rate <= s.actual_body_rate_resume_deg_s)
            self.motion_resume_since, self.motion_resume_last = record_hold_sample(good, stamp, self.motion_resume_since)
            if self.motion_paused and held_samples(True, self.motion_resume_since, self.motion_resume_last, s.motion_resume_hold_s):
                self.motion_paused = False
                self.motion_pause_at = None
        if self.motion_paused:
            self.motion_pause_at = self.motion_pause_at or now
            self.motion_controller.horizontal.clear_integral()
        if self.motion_loss_at is not None and (now-self.motion_loss_at).to_sec() >= s.motion_feedback_timeout_s:
            self.motion_fatal = '水平反馈持续失效：停止动作并锁存ABORT'
        if self.motion_pause_at is not None and (now-self.motion_pause_at).to_sec() >= s.motion_pause_timeout_s:
            self.motion_fatal = '运动限制持续超限：停止动作并锁存ABORT'

    def _motion_enter_state(self, new_state, now):
        if not getattr(self, 'motion_enabled', False):
            return
        self.motion_controller.horizontal.clear_integral()
        self.motion_rc_feedback = self.motion_rc_feedback_at = None
        self.motion_rc_weight, self.motion_rc_active, self.motion_rc_ready = 1., False, False
        self.motion_rc_tick = now
        self.motion_bad_since = None
        self.motion_contact_unsafe = False
        self.motion_control_sample = None
        self.motion_integrated_output = None
        self.motion_recovery_ready = False
        self.motion_recovery_attitude = None
        self.motion_recovery_lock_time = rospy.Time(0)
        self.motion_recovery_since = self.motion_recovery_last = None
        if new_state == S.STATE_DETACH:
            self.motion_contact_released = False
            self.motion_release_since = self.motion_release_last = self.motion_release_stamp = None
            # A failed attachment has no HOLD reference: establish a separate abort-descent reference.
            if self._baro_enabled() and self.hold_pressure_pa is None and self._baro_fresh(now):
                self.hold_pressure_pa = self.baro_pressure_pa
        elif new_state == S.STATE_RECOVERY_HOVER:
            self.motion_recovery_height = (self.detach_start_height_m-self.detach_drop_m
                if not self.approach_timed_out and self.detach_start_height_m is not None
                else self._height_feedback(now))
            self.motion_contact_released = True
        elif new_state == S.STATE_NORMAL_FLIGHT:
            self.motion_controller.reset()
            self.motion_equilibrium.clear()
            self.motion_capture_at = None
            self.motion_fatal = ''
            self.motion_paused = False
            self.motion_loss_at = self.motion_pause_at = None

    def _motion_release_ready(self, now):
        if self.motion_contact_released:
            return True
        if not self._range_fresh(now) or self.range_message is None:
            self.motion_release_since = self.motion_release_last = None
            return False
        if self.motion_release_stamp != self.range_received_at:
            self.motion_release_stamp = self.range_received_at
            self.motion_release_since, self.motion_release_last = record_hold_sample(
                self.range_message.range >= self.motion_settings.contact_release_distance_m,
                self.range_received_at, self.motion_release_since)
        self.motion_contact_released = held_samples(True, self.motion_release_since,
            self.motion_release_last, self.motion_settings.contact_release_hold_s)
        return self.motion_contact_released

    def _ceiling_recovery_ready(self, now):
        if not getattr(self, 'motion_enabled', False):
            self.motion_recovery_ready = self._state_age_s(now) >= self.recovery_hover_time_s
            return self.motion_recovery_ready
        s = self.motion_settings
        z = self._height_feedback(now)
        feedback = (self._ceiling_manual_settled() and self.motion_ready and not self.motion_paused and not self.motion_fatal and z is not None
                    and self.motion_recovery_height is not None and self._flight_ready(now))
        if not feedback:
            self.motion_recovery_since = self.motion_recovery_last = None
            self.motion_recovery_ready = False
            return False
        sample = min(self.odom.header.stamp, self.motion_pressure_source if self._baro_enabled() else self.odom.header.stamp,
                     self.motion_flow.header.stamp)
        if sample != self.motion_processed_recovery:
            self.motion_processed_recovery = sample
            vz = linear_velocity_parent_frame(self.odom, self.odom_twist_in_child_frame)[2]
            yaw = euler_from_quaternion(tuple(getattr(self.odom.pose.pose.orientation, k) for k in 'xyzw'))[2]
            yaw_error = abs(math.atan2(math.sin(yaw-self.motion_controller.yaw), math.cos(yaw-self.motion_controller.yaw)))
            good = (abs(z-self.motion_recovery_height) <= s.recovery_height_tolerance_m + 1e-9
                    and abs(vz) <= s.recovery_vertical_speed_mps
                    and self.motion_speed <= s.recovery_horizontal_speed_mps
                    and self.motion_drift <= s.horizontal_drift_resume_m
                    and yaw_error <= math.radians(s.yaw_max_offset_deg)
                    and self.motion_tilt <= s.recovery_max_tilt_deg and self.motion_rate <= s.recovery_max_rate_deg_s)
            self.motion_recovery_since, self.motion_recovery_last = record_hold_sample(good, sample, self.motion_recovery_since)
        self.motion_recovery_ready = held_samples(True, self.motion_recovery_since,
            self.motion_recovery_last, s.recovery_stable_time_s)
        if self.motion_recovery_ready and self.motion_recovery_attitude is None:
            self.motion_recovery_attitude = tuple(getattr(self.odom.pose.pose.orientation, k) for k in 'xyzw')
            self.motion_recovery_lock_time = now
            self.motion_controller.horizontal.reset()
        return self.motion_recovery_ready

    def _apply_ceiling_motion_status(self, status, now, dt):
        if not getattr(self, 'motion_enabled', False):
            status.recovery_ready = self.state == S.STATE_RECOVERY_HOVER and self._ceiling_recovery_ready(now)
            return
        s, controller = self.motion_settings, self.motion_controller
        frozen = controller.xy is not None
        status.rc_manual_coordination_enabled = self.rc_manual_coordination_enabled
        status.rc_manual_feedback_ready = self.motion_rc_ready
        status.rc_manual_active = self.motion_rc_active
        status.rc_position_weight = self.motion_rc_weight
        status.motion_control_enabled = True
        status.motion_ready = self.motion_ready
        status.baseline_ready = frozen or (self.motion_equilibrium.ready and self._motion_target_fresh(now))
        status.motion_paused = self.motion_paused
        status.horizontal_drift_m, status.horizontal_speed_mps = self.motion_drift, self.motion_speed
        status.actual_tilt_deg, status.actual_body_rate_deg_s = self.motion_tilt, self.motion_rate
        status.motion_reason = self.motion_fatal or self.motion_reason
        status.yaw_reference_rad = controller.yaw if frozen else math.nan
        status.recovery_target_height_m = self.motion_recovery_height if self.motion_recovery_height is not None else math.nan
        status.recovery_ready = self.state == S.STATE_RECOVERY_HOVER and self._ceiling_recovery_ready(now)
        locked = self.state == S.STATE_RECOVERY_HOVER and self.motion_recovery_attitude is not None
        status.recovery_attitude_locked = locked
        status.recovery_lock_time = self.motion_recovery_lock_time if locked else rospy.Time(0)
        if not frozen or self.odom is None or not odometry_is_finite(self.odom):
            return
        free = self.state in (S.STATE_CEILING_ARMED, S.STATE_APPROACH, S.STATE_RECOVERY_HOVER, S.STATE_ABORT) and not (self.state == S.STATE_ABORT and getattr(self, "hold_timed_out", False))
        if self.state == S.STATE_DETACH:
            free = self._motion_release_ready(now)
        q = tuple(getattr(self.q_hover, k) for k in 'xyzw')
        velocity = linear_velocity_parent_frame(self.odom, self.odom_twist_in_child_frame)
        if locked:
            controller.horizontal.reset()
            q = self.motion_recovery_attitude
        elif free and self.motion_ready and not self.motion_fatal:
            p = self.odom.pose.pose.position
            sample = self.odom.header.stamp
            integral_dt = min(.05, max(0., (sample-self.motion_control_sample).to_sec())) if self.motion_control_sample is not None else 0.
            self.motion_control_sample = sample
            output = self.motion_output
            applied_axis = None
            output_ready = (self.motion_owner == 1 and is_fresh_receipt(self.motion_owner_at, .3, now)
                and output is not None and is_fresh_receipt(self.motion_output_at, .2, now)
                and is_fresh_source(output.header.stamp, .2, now, future_tolerance_s=0.))
            if output_ready:
                applied_axis = rotate_vector(tuple(getattr(output.orientation, k) for k in 'xyzw'), (0., 0., 1.))
            if not output_ready or output.header.stamp == self.motion_integrated_output:
                integral_dt = 0.
            elif self.motion_integrated_output is not None:
                integral_dt = min(integral_dt, max(0., (output.header.stamp-self.motion_integrated_output).to_sec()))
            if output_ready:
                self.motion_integrated_output = output.header.stamp
            q = controller.attitude((p.x, p.y), velocity[:2], dt,
                integrate=integral_dt > 0 and not self.motion_paused and self.state != S.STATE_ABORT
                    and self._ceiling_manual_settled(),
                integral_dt=integral_dt, applied_axis=applied_axis, position_weight=self.motion_rc_weight)
        else:
            controller.horizontal.reset()
        desired_vz = 0.
        if self.state == S.STATE_APPROACH:
            desired_vz = status.approach_velocity_z_enu_mps
        elif self.state in (S.STATE_DETACH, S.STATE_RECOVERY_HOVER):
            z = self._height_feedback(now)
            target = (self.motion_recovery_height if self.state == S.STATE_RECOVERY_HOVER
                      else self.detach_start_height_m-self.detach_drop_m if self.detach_start_height_m is not None else None)
            if z is not None and target is not None:
                gain = s.recovery_height_kp if self.state == S.STATE_RECOVERY_HOVER else self.detach_height_kp
                desired_vz = clamp(gain*(target-z), -self.detach_down_speed_mps, .10)
                if self.state == S.STATE_DETACH and not self.baro_handover_reached:
                    # Reach the separation threshold in finite time, then brake in RECOVERY.
                    desired_vz = min(desired_vz, -min(s.detach_min_down_speed_mps, self.detach_down_speed_mps))
            if not self._mechanism_release_confirmed(now):
                desired_vz = 0.
        if self.motion_paused or not self.motion_ready or self.motion_fatal or self.state == S.STATE_ABORT:
            desired_vz = 0.
        target_vz = controller.vertical_target(desired_vz, dt)
        self.motion_last_target_vz = target_vz
        status.target_vertical_speed_mps = target_vz
        if self.state in (S.STATE_CEILING_ARMED, S.STATE_APPROACH, S.STATE_DETACH, S.STATE_RECOVERY_HOVER, S.STATE_ABORT) and not status.abort_land_requested:
            if self.state != S.STATE_DETACH or self._mechanism_release_confirmed(now):
                support = controller.vertical_support
                axis_z = max(.80, rotate_vector(q, (0., 0., 1.))[2])
                thrust = (support+s.vertical_speed_kp*(target_vz-velocity[2]))/axis_z
                status.thrust_body_z_normalized = -clamp(thrust, self.detach_min_thrust, min(self.max_thrust, support+.12))
                status.attitude_setpoint = Quaternion(*q)
                status.attitude_setpoint_valid = True
        if locked:
            status.reason += '; 姿态已锁定，高度闭环继续；等待第三次CH12确认交回POSCTL'
        elif self.rc_manual_coordination_enabled:
            status.reason += ('; 遥控修正中：暂停水平积分，位置回拉权重%.2f' % self.motion_rc_weight
                              if self.motion_rc_active else '; 位置回拉平滑恢复中'
                              if self.motion_rc_weight < 1.-1e-9 else '; 等待本轮遥控修正反馈'
                              if not self.motion_rc_ready else '; 自动水平稳定')
        status.reason += '; ' + status.motion_reason
        if self.motion_paused:
            status.reason += '; 暂停竖直运动，等待恢复阈值持续满足'
        elif self.motion_drift > s.horizontal_drift_warn_m and free:
            status.reason += '; 水平漂移超过告警阈值'
