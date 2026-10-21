"""Continuous height support from accepted departure through handover."""
import math
import json
from contextlib import nullcontext
from dataclasses import dataclass, fields, asdict

import rospy
from mavros_msgs.msg import AttitudeTarget, EstimatorStatus, OpticalFlowRad
from std_msgs.msg import String

from px4_wall_ceiling_control.domain.departure_height import DepartureHeightController
from px4_wall_ceiling_control.domain.height_continuation import HeightContinuation
from px4_wall_ceiling_control.domain.geometry import rotate_vector
from px4_wall_ceiling_control.domain.range_height_feedback import RangeHeightConfig, RangeHeightFeedback
from px4_wall_ceiling_control.freshness import is_fresh_receipt, is_fresh_source
from px4_wall_ceiling_control.msg import AttachmentControlCandidate as C
from px4_wall_ceiling_control.msg import WallPerchStatus as W
from px4_wall_ceiling_control.srv import SetRangeHeight, SetRangeHeightResponse


@dataclass(frozen=True)
class DepartureFeedback:
    height: float
    velocity: float
    quaternion: tuple
    source: float
    received: float

    @property
    def vertical_fraction(self):
        return rotate_vector(self.quaternion, (0., 0., 1.))[2]

    @property
    def tilt_deg(self):
        return math.degrees(math.acos(max(-1., min(1., self.vertical_fraction))))


class WallDetachVerticalMixin:
    def _init_detach_vertical(self):
        self.detach_vertical_controller = DepartureHeightController(
            height_kp=float(rospy.get_param('~detach_height_kp', .50)),
            velocity_kp=float(rospy.get_param('~detach_velocity_kp', .30)),
            velocity_ki=float(rospy.get_param('~detach_velocity_ki', .12)),
            integral_limit=float(rospy.get_param('~detach_vertical_integral_limit', .06)),
            correction_limit=float(rospy.get_param('~detach_vertical_correction_limit', .12)),
            up_speed=float(rospy.get_param('~recovery_max_up_speed_mps', .10)),
            down_speed=float(rospy.get_param('~recovery_max_down_speed_mps', .15)),
            max_thrust=min(self.max_thrust, float(rospy.get_param('~detach_support_max_thrust', .85))),
            thrust_slew=float(rospy.get_param('~return_thrust_slew_per_s', .50)),
            compensation_start_deg=float(rospy.get_param('~detach_compensation_start_deg', 80.)))
        limits = dict(jump_m=.25, entry_tilt_deg=30., episode_s=1.2, total_s=1.5,
                      confirm_s=.30, min_samples=4, max_gap_s=.20,
                      step_residual_m=.05, residual_rate_mps=.20,
                      restore_tilt_deg=10., restore_speed_mps=.20, blend_s=.30)
        self.detach_height_continuation = HeightContinuation(**{
            key: rospy.get_param('~detach_height_continuation_'+key, value)
            for key, value in limits.items()})
        self._reset_detach_vertical()
        self.detach_range_enabled = bool(rospy.get_param('~detach_range_height_enabled', False))
        # Parameter RPCs belong to startup, never to the timed control path.
        # The immutable configuration is shared by every departure until restart.
        self.detach_range_config = self._load_range_config()
        self.detach_range = RangeHeightFeedback(self.detach_range_config)
        self.detach_range_episode = 0
        self._reset_range_task()
        if self.detach_range_enabled:
            self.range_height_service = rospy.Service('/wall_perch/set_range_height',
                                                      SetRangeHeight, self._on_set_range_height)
        # Vertical range capture must never depend on horizontal flow readiness.
        rospy.Subscriber('/mavros/px4flow/raw/optical_flow_rad', OpticalFlowRad,
                         self._on_detach_range, queue_size=100)
        self.manager_handover_snapshot = None
        rospy.Subscriber('/wall_perch/handover_readiness', String,
                         self._on_range_handover_readiness, queue_size=10)
        # Optical-flow mode already subscribes to the same estimator callback.
        if not getattr(self, 'horizontal_velocity_enabled', False):
            rospy.Subscriber('/mavros/estimator_status', EstimatorStatus,
                             self._on_horizontal_estimator, queue_size=10)

    def _reset_detach_vertical(self):
        controller = getattr(self, 'detach_vertical_controller', None)
        if controller is not None:
            controller.active = False
            controller.clear_integral()
        self.detach_vertical_active = False
        self.detach_vertical_reason = '未开始正常脱离保高'
        self.detach_vertical_base_source = 'none'
        self.detach_vertical_fault = ''
        self.detach_vertical_fault_feedback = None
        self.detach_vertical_output_degraded = False
        self.detach_vertical_output_reason = ''
        self.detach_vertical_last_feedback = None
        self.detach_vertical_value = math.nan
        self.recovery_support_valid = False
        self.recovery_thrust_limit = .0
        continuation = getattr(self, 'detach_height_continuation', None)
        if continuation is not None:
            continuation.reset()
        self.detach_feedback_snapshot = self.detach_feedback_prepared_at = None
        self.detach_feedback_output = None
        self.detach_feedback_valid = False
        self.detach_height_reanchor_note = ''

    def _snapshot_departure_feedback(self):
        """Caller holds the decision node's feedback lock for the whole cycle."""
        feedback, q = self._height_feedback(), self._current_quaternion()
        if feedback is None or q is None:
            return None
        stamp = self.odom.header.stamp.to_sec()
        receipt = getattr(self, 'odom_received_at', None)
        return DepartureFeedback(feedback[0], feedback[1], tuple(q), stamp,
                                 receipt.to_sec() if receipt is not None else stamp)

    def _detach_height_loop_ready(self):
        if self._range_task_active():
            s = self.detach_range.snapshot(rospy.Time.now().to_sec())
            return bool(s.valid and s.velocity_valid and not self.detach_vertical_fault)
        continuation = getattr(self, 'detach_height_continuation', None)
        return not getattr(self, 'detach_vertical_active', False) or continuation is None or continuation.ready

    def _fault_height_recovery(self):
        return (getattr(self, 'state', None) == W.STATE_ABORT or
                (getattr(self, 'state', None) == W.STATE_EXIT and
                 getattr(self, 'fault_recovery_completed', False)))

    def _prepare_detach_vertical_feedback(self, now):
        """Detect discontinuities before state transitions, once per tick."""
        if not getattr(self, 'detach_vertical_active', False):
            return
        if self._range_task_active():
            self._prepare_range_feedback(now)
            if self.detach_range.locked:
                return
        # Recheck live authority even on duplicate evaluations. The feedback
        # lock keeps odometry fixed; repeated source samples add no evidence.
        self.detach_feedback_prepared_at = now
        self.detach_feedback_valid = self.recovery_support_valid = False
        self.detach_feedback_snapshot = None
        self.detach_feedback_output = None
        controller, continuation = self.detach_vertical_controller, self.detach_height_continuation
        flight = getattr(self, 'flight_state', None)
        authority = bool(flight is not None and flight.connected and flight.armed and flight.mode.upper() == 'OFFBOARD'
                         and is_fresh_receipt(getattr(self, 'flight_state_received_at', None), self.state_timeout_s, now))
        reason = ''
        if not authority:
            reason = '脱离保高等待有效OFFBOARD权限'
        elif not self._detach_vertical_feedback_ok(now):
            reason = self.detach_vertical_reason
        else:
            self.detach_feedback_snapshot = self._snapshot_departure_feedback()
            self.detach_feedback_output = self._detach_vertical_output(now, allow_recovery=True)
            if self.detach_feedback_output is None:
                reason = self.detach_vertical_reason
            elif self.detach_feedback_snapshot is None:
                reason = '脱离高度、速度或姿态反馈无效'
        if reason:
            self.detach_vertical_reason = reason
            if not continuation.ready:
                controller.clear_integral()
                if not self._fault_height_recovery():
                    if not continuation.failure:
                        continuation.fail(reason)
                    self.detach_vertical_fault = continuation.failure
            return
        self.detach_feedback_valid = True
        sample = self.detach_feedback_snapshot
        if self._fault_height_recovery():
            return  # Preserve existing fault recovery; never call it normal completion.
        continuation.observe(now.to_sec(), sample.source, sample.height, sample.velocity, sample.tilt_deg)
        if continuation.failure:
            controller.clear_integral()
            self.detach_vertical_fault = self.detach_vertical_reason = (
                '%s; residual=%.3f m, episode=%.3f s, total=%.3f s' %
                (continuation.failure, continuation.residual,
                 continuation.episode_age(now.to_sec()), continuation.total_used))
            self.detach_vertical_fault_feedback = (sample.source, sample.height, sample.velocity)
        elif continuation.reanchor_height is not None:
            old = controller.reference
            controller.reanchor(sample.height)
            self.detach_altitude_m = self.recovery_altitude_m = sample.height
            self.vertical_reference_source = 'bounded_continuation_reanchor'
            self.detach_height_reanchor_note = '高度基准已改变 %.3f→%.3f m' % (old, sample.height)
            rospy.loginfo('wall detach: %s; residual=%.3f m, continuation_total=%.3f s',
                          self.detach_height_reanchor_note, continuation.residual, continuation.total_used)
        if not continuation.ready:
            controller.clear_integral()

    def _detach_vertical_feedback_ok(self, now):
        if self._range_locked():
            estimate = getattr(self, 'horizontal_estimator', None)
            valid = bool(self._odom_fresh(now) and self._current_quaternion() is not None
                         and estimate is not None and estimate.attitude_status_flag
                         and is_fresh_receipt(getattr(self, 'horizontal_estimator_at', None), 1., now))
            if not valid:
                self.detach_vertical_reason = '等待有效实际姿态；测距闭环不读取旧Z/融合竖直速度'
            return valid
        if not self._odom_fresh(now) or self._height_feedback() is None or self._current_quaternion() is None:
            self.detach_vertical_reason = '脱离高度、速度或姿态反馈无效/过期'
            return False
        estimate = getattr(self, 'horizontal_estimator', None)
        if (estimate is None or not is_fresh_receipt(getattr(self, 'horizontal_estimator_at', None), 1., now)
                or not estimate.attitude_status_flag or not estimate.velocity_vert_status_flag
                or not estimate.pos_vert_abs_status_flag):
            self.detach_vertical_reason = '等待有效的竖直位置/速度估计'
            return False
        return True

    def _detach_vertical_output(self, now, allow_recovery=False):
        self.detach_vertical_output_degraded = False
        self.detach_vertical_output_reason = ''
        if (getattr(self, 'horizontal_output_owner', None) != C.SOURCE_WALL
                or not is_fresh_receipt(getattr(self, 'horizontal_output_owner_at', None), .30, now)):
            self.detach_vertical_reason = '等待新鲜侧吸输出权限'
            return None
        def valid(output, receipt, timeout):
            if (output is None or output.type_mask & (AttitudeTarget.IGNORE_THRUST | AttitudeTarget.IGNORE_ATTITUDE)
                    or not is_fresh_receipt(receipt, timeout, now)
                    or not is_fresh_source(output.header.stamp, timeout, now, future_tolerance_s=0.)
                    or not math.isfinite(output.thrust)
                    or not .1-1e-6 <= output.thrust <= self.detach_vertical_controller.max_thrust+1e-6):
                return False
            q = tuple(getattr(output.orientation, k) for k in 'xyzw')
            return all(math.isfinite(v) for v in q) and abs(sum(v*v for v in q)-1.) <= .01
        output, receipt = getattr(self, 'horizontal_output', None), getattr(self, 'horizontal_output_at', None)
        if valid(output, receipt, .20):
            return output
        # Entry remains strict. During an already accepted departure, bounded
        # continuation breaks the candidate-invalid -> no-echo feedback cycle.
        # Both fallback sources disable integration; neither is learned as hover.
        if allow_recovery:
            echo, echo_at = getattr(self, 'detach_fcu_target', None), getattr(self, 'detach_fcu_target_at', None)
            if valid(echo, echo_at, .35):
                self.detach_vertical_output_degraded = True
                self.detach_vertical_output_reason = '最终输出短暂中断，使用新鲜飞控目标限速衔接，积分清零'
                return echo
            if valid(output, receipt, .50):
                self.detach_vertical_output_degraded = True
                self.detach_vertical_output_reason = '最终输出短暂中断，在0.5秒有效期内受限续接，积分清零'
                return output
        self.detach_vertical_reason = '等待可信实际输出，禁止猜测推力'
        return None

    def _begin_detach_vertical(self, now, output):
        self._reset_detach_vertical()
        range_config = self._range_config() if getattr(self, 'detach_range_enabled', False) else None
        if range_config is not None and not range_config.installation_ready:
            self.detach_vertical_reason = '待核对下视测距设备量程、安装轴及上游几何修正'
            return False
        flight = getattr(self, 'flight_state', None)
        if not (flight is not None and flight.connected and flight.armed and flight.mode.upper() == 'OFFBOARD'
                and is_fresh_receipt(getattr(self, 'flight_state_received_at', None), self.state_timeout_s, now)):
            self.detach_vertical_reason = '脱离开始前需要新鲜OFFBOARD权限'
            return False
        if not self._detach_vertical_feedback_ok(now):
            return False
        actual = self._detach_vertical_output(now)
        if actual is None or output is not actual:
            return False
        feedback = self._height_feedback()
        base = getattr(self, 'approach_base_vertical_thrust', None)
        source = 'POSCTL接管垂直支撑'
        if base is None or not math.isfinite(base):
            base = getattr(self, 'flip_base_vertical_thrust', None)
            source = '本次翻转继承支撑'
        if base is None or not math.isfinite(base):
            base, source = self.hover_thrust, '配置悬停基准（未采集接管支撑）'
        try:
            self.detach_vertical_controller.begin(feedback[0], base, float(actual.thrust), now.to_sec())
        except ValueError as error:
            self.detach_vertical_reason = str(error)
            return False
        self.detach_altitude_m = self.recovery_altitude_m = feedback[0]
        self.recovery_wait_for_level = False
        self.vertical_reference_source = 'detach_start_height'
        self.detach_vertical_base_source = source
        self.detach_vertical_active = self.recovery_support_valid = True
        self.detach_vertical_value = self.detach_vertical_controller.value
        self.recovery_thrust_limit = self.detach_vertical_controller.max_thrust
        self.detach_vertical_reason = '已锁定脱离开始高度，继承实际输出推力'
        self.detach_vertical_last_feedback = (self.odom.header.stamp.to_sec(), feedback[0], feedback[1])
        sample = self._snapshot_departure_feedback()
        self.detach_height_continuation.observe(now.to_sec(), sample.source, sample.height, sample.velocity, sample.tilt_deg)
        if getattr(self, 'detach_range_enabled', False):
            self._reset_range_task()
            self.detach_range_episode += 1
            self.detach_range.begin(self.detach_range_episode, now.to_sec(), range_config)
            self.detach_task_phase = 'ROTATING_TO_RANGE_ENTRY'
            self.detach_range_faulted = False
            self.detach_range_synced = False
            self.detach_range_stable_since = self.detach_range_stable_source = None
            self.detach_range_stable_samples = 0
            self.detach_range_handover_started = self.detach_range_posctl_request_at = None
            self.detach_range_posctl_confirmed_at = None
            self.detach_range_exit_entered_at = None
            self.detach_range_last_flight_source = None
        return True

    def _detach_vertical_thrust(self, now, dt):
        # dt is also used by the attitude trajectory. The controller additionally
        # enforces elapsed time and source-sample bounds; duplicate evaluation is idempotent.
        if not getattr(self, 'detach_vertical_active', False):
            return math.nan
        self._prepare_detach_vertical_feedback(now)
        if self._range_locked():
            return self._range_vertical_thrust(now)
        controller = self.detach_vertical_controller
        self.recovery_support_valid = False
        if not self.detach_feedback_valid:
            controller.clear_integral()
            return math.nan
        sample, output = self.detach_feedback_snapshot, self.detach_feedback_output
        height, speed, source = sample.height, sample.velocity, sample.source
        if self.detach_vertical_fault:
            self.detach_vertical_reason = self.detach_vertical_fault
            if not self._fault_height_recovery():
                return math.nan
            # A new coordinate frame invalidates the original height target.
            # Only fault recovery may adopt the current height, after two
            # consistent source frames, with the integral discarded.
            fault_sample = self.detach_vertical_fault_feedback
            if fault_sample is None:
                self.detach_vertical_fault_feedback = (source, height, speed)
                return math.nan
            if source <= fault_sample[0]:
                return math.nan
            elapsed = source-fault_sample[0]
            residual = abs(height-fault_sample[1]-.5*(speed+fault_sample[2])*elapsed)
            self.detach_vertical_fault_feedback = (source, height, speed)
            if elapsed > self.odom_timeout_s or residual > .25:
                return math.nan
            try:
                controller.begin(height, controller.base, float(output.thrust), now.to_sec())
            except ValueError:
                return math.nan
            self.detach_vertical_fault = ''
            self.detach_altitude_m = self.recovery_altitude_m = height
            self.vertical_reference_source = 'fault_recovery_height'
            self.detach_vertical_last_feedback = (source, height, speed)
        old = self.detach_vertical_last_feedback
        if old is not None and source > old[0]:
            source_dt = source-old[0]
            residual = abs(height-old[1]-.5*(speed+old[2])*source_dt)
            if (self._fault_height_recovery()
                    and source_dt <= self.odom_timeout_s and residual > .25):
                controller.clear_integral()
                self.detach_vertical_fault = '脱离高度估计发生不连续跳变'
                self.detach_vertical_fault_feedback = (source, height, speed)
                self.detach_vertical_reason = self.detach_vertical_fault
                return math.nan
        if old is None or source > old[0]:
            self.detach_vertical_last_feedback = (source, height, speed)
        continuation = self.detach_height_continuation
        normal_departure = not self._fault_height_recovery()
        weight = continuation.weight if normal_departure else 1.
        if self.detach_vertical_output_degraded:
            controller.clear_integral()
        try:
            thrust = controller.update(height, speed, sample.vertical_fraction, now.to_sec(), source, float(output.thrust),
                                       integrate=not self.detach_vertical_output_degraded and (not normal_departure or continuation.ready),
                                       correction_weight=weight)
        except ValueError as error:
            controller.clear_integral()
            self.detach_vertical_fault = self.detach_vertical_reason = str(error)
            self.detach_vertical_fault_feedback = (source, height, speed)
            return math.nan
        self.detach_vertical_value = thrust
        self.recovery_support_valid = True
        self.recovery_thrust_limit = controller.max_thrust
        self.vertical_target_speed, self.vertical_correction = controller.target_speed, controller.correction
        self.detach_vertical_reason = ('近竖直仍需墙面/机构承重，保持连续压力' if controller.support_weight < 1.-1e-9
            else '脱离保高输出受限' if controller.saturated else '脱离保高闭环有效')
        if self.detach_vertical_output_reason:
            self.detach_vertical_reason = self.detach_vertical_output_reason
        if self.vertical_reference_source == 'fault_recovery_height':
            self.detach_vertical_reason = '故障恢复已重建高度基准；'+self.detach_vertical_reason
        if normal_departure and continuation.state in ('SUPPORT', 'BLENDING'):
            self.detach_vertical_reason = ('高度闭环暂停、受限支撑有效' if continuation.state == 'SUPPORT'
                                          else '高度闭环平滑恢复、禁止积分')
            self.detach_vertical_reason += ('；残差 %.3f m；单次 %.3f/%.2f s；累计 %.3f/%.2f s；纠偏权重 %.2f' % (
                continuation.residual, continuation.episode_age(now.to_sec()), continuation.episode_s,
                continuation.total_used, continuation.total_s, continuation.weight))
        if self.detach_height_reanchor_note:
            self.detach_vertical_reason += '；'+self.detach_height_reanchor_note
        return thrust

    def _fill_detach_vertical_status(self, status):
        active = getattr(self, 'detach_vertical_active', False)
        controller = getattr(self, 'detach_vertical_controller', None)
        status.recovery_support_valid = bool(active and getattr(self, 'recovery_support_valid', False))
        status.recovery_thrust_limit = getattr(self, 'recovery_thrust_limit', 0.) if active else 0.
        status.detach_vertical_active = active
        status.detach_vertical_reason = getattr(self, 'detach_vertical_reason', '未开始正常脱离保高')
        status.detach_vertical_base_source = getattr(self, 'detach_vertical_base_source', 'none')
        status.detach_vertical_integral = controller.integral if active else 0.
        status.detach_vertical_integral_active = bool(active and controller.integral_active)
        status.detach_vertical_saturated = bool(active and controller.saturated)
        status.detach_vertical_slew_limited = bool(active and controller.slew_limited)
        status.detach_vertical_requested_thrust = controller.requested if active else math.nan
        status.detach_vertical_base_thrust = controller.base if active else math.nan
        status.detach_vertical_support_weight = controller.support_weight if active else 0.
        feedback = self._height_feedback() if active else None
        status.detach_height_error_m = controller.reference-feedback[0] if feedback is not None else math.nan
        if active:
            status.vertical_control_active = status.recovery_support_valid and (
                self._fault_height_recovery() or self._detach_height_loop_ready())
            status.vertical_reference_valid = True
            status.vertical_reference_m = controller.reference
            status.vertical_reference_source = self.vertical_reference_source
            status.vertical_target_enu_mps = controller.target_speed
            status.vertical_thrust_correction = controller.correction
        self._fill_range_status(status)

    def _range_config(self):
        return self.detach_range_config

    def _load_range_config(self):
        default = RangeHeightConfig()
        values = {f.name: rospy.get_param('~detach_range_'+f.name, getattr(default, f.name))
                  for f in fields(default)}
        for key in ('sensor_axis_body', 'reference_offset_body_m'):
            values[key] = tuple(values[key])
        return RangeHeightConfig(**values)

    def _reset_range_task(self):
        if hasattr(self, 'detach_range'):
            self.detach_range.reset()
        self.detach_task_phase = 'NONE'
        self.detach_range_synced = False
        self.detach_range_faulted = False
        self.detach_range_stable_since = self.detach_range_stable_source = None
        self.detach_range_stable_samples = 0
        self.detach_range_handover_started = self.detach_range_posctl_request_at = None
        self.detach_range_posctl_confirmed_at = None
        self.detach_range_exit_entered_at = None
        self.detach_range_block_reason = '未开始脱离'
        self.detach_range_manual_request_id = ''
        self.detach_range_manual_response = None
        self.detach_range_manual_recovery = False
        self.detach_range_previous_fault = ''
        self.detach_range_failure_cause = ''

    def _manual_range_height_block_reason(self, now):
        """Admission for an explicit bounded action, never a fault auto-retry."""
        if not self._range_locked() or self.state not in (W.STATE_DETACH_ROTATE, W.STATE_RECOVER, W.STATE_EXIT, W.STATE_ABORT):
            return '仅本轮离墙测距接管后允许一键定高'
        r = self.detach_range
        if r.target_revision:
            return '本轮已提交一键定高，重复点击不会重置预算'
        if self.detach_range_posctl_request_at is not None:
            return 'POSCTL交回请求已发出，不能再改变高度目标'
        fault = self.detach_range_failure_cause or self.detach_vertical_fault or self.last_reason
        if self.detach_range_faulted and fault not in ('测距闭环总时间预算超时', '测距交回等待超时'):
            return '仅测距控制或交回超时可重新准入；'+fault
        if self.cancel_requested or not self._mode_requested(now):
            return '贴墙控制请求已取消或过期'
        flight = self.flight_state
        if not (self._flight_ready(now) and flight and flight.connected and flight.armed
                and flight.mode.upper() == 'OFFBOARD'
                and is_fresh_source(flight.header.stamp, self.state_timeout_s, now)):
            return '等待有效急停反馈及新鲜已解锁OFFBOARD权限'
        s = r.snapshot(now.to_sec())
        if not s.valid or not s.velocity_valid:
            return '等待新鲜测距高度和有效测距速度窗口'
        output = self._valid_wall_output(now)
        if not self._detach_vertical_feedback_ok(now) or output is None:
            return '姿态反馈或实际侧吸输出无效'
        controller = self.detach_vertical_controller
        if (not self.detach_range_synced or not controller.active
                or not math.isfinite(controller.base) or not .1 <= controller.base < controller.max_thrust
                or not .1-1.e-6 <= output.thrust <= controller.max_thrust+1.e-6):
            return '定高支撑基准或实际推力超出控制器限制'
        if self._detach_progress() < 1. or not self._detach_rotation_pose_valid() or not self._published_recovery_target_ready(now):
            return '等待旋转完成、实测与发送姿态回平及角速度稳定'
        if not self._horizontal_recovery_ready(now) or not self._wall_fusion_samples_ready(now):
            return '等待水平速度稳定及光流融合恢复确认'
        if self.detach_range_faulted:
            front = getattr(self, 'front_range', None)
            if (front is None or not self._front_fresh(now) or not self._sensor_health_fresh(now)
                    or not self.sensor_health.front_valid or not math.isfinite(front.range)
                    or front.range < self.departure_clearance_distance
                    or getattr(self, 'horizontal_takeover_yaw', None) is None):
                return '超时后重新准入需新鲜有效的安全离墙间隙和原接管航向'
        return ''

    def _on_set_range_height(self, request):
        with getattr(self, 'feedback_lock', nullcontext()):
            now = rospy.Time.now()
            r = self.detach_range
            # Same successful request is idempotent, including after a timeout.
            if (request.request_id and request.request_id == self.detach_range_manual_request_id
                    and request.maneuver_id == getattr(self, 'maneuver_id', '')
                    and request.episode == r.episode and self.detach_range_manual_response is not None):
                return self.detach_range_manual_response
            reason = ''
            if (not request.request_id or len(request.request_id) > 80
                    or request.maneuver_id != getattr(self, 'maneuver_id', '')
                    or request.episode != r.episode or request.expected_target_revision != r.target_revision
                    or not 0 <= now.to_sec()-request.requested_at.to_sec() <= .5):
                reason = '定高请求过期、本轮身份或目标版本不匹配'
            if not reason:
                self._prepare_range_feedback(now)
                reason = self._manual_range_height_block_reason(now)
            if not reason:
                try:
                    r.set_manual_target(now.to_sec())
                except ValueError as error:
                    reason = str(error)
            if reason:
                return SetRangeHeightResponse(False, reason, r.target_revision, r.target)
            previous_fault = (self.detach_range_failure_cause or self.detach_vertical_fault or self.last_reason
                              if self.detach_range_faulted else '')
            self.detach_range_previous_fault = previous_fault
            self.detach_range_manual_recovery = bool(self.detach_range_faulted)
            self.detach_range_manual_request_id = request.request_id
            self.detach_range_faulted = False  # This new admitted action has its own budget.
            self.detach_range_failure_cause = ''
            self.detach_vertical_fault = ''
            self.detach_vertical_controller.begin(r.target, self.detach_vertical_controller.base,
                                                  float(self.horizontal_output.thrust), now.to_sec())
            self.detach_altitude_m = self.recovery_altitude_m = r.target
            self.vertical_reference_source = r.target_source
            self.detach_range_stable_since = self.detach_range_stable_source = None
            self.detach_range_stable_samples = 0
            self.detach_range_handover_started = self.detach_range_exit_entered_at = None
            self.detach_range_posctl_request_at = self.detach_range_posctl_confirmed_at = None
            self.manager_handover_snapshot = None
            self.detach_completed = self.recovery_completed = self.fault_recovery_completed = False
            self.detach_feedback_prepared_at = None
            if self.detach_range_manual_recovery:
                self._begin_departure_clearance(now)  # Recollect fresh clearance; preserve frozen heading.
            self._enter_state(W.STATE_RECOVER, now, '人工一键定高：目标 %.2f m' % r.target)
            self.detach_range_manual_response = SetRangeHeightResponse(True, self.last_reason,
                                                                       r.target_revision, r.target)
            rospy.loginfo('wall manual range height: request=%s episode=%s revision=%s first=%.6f target=%.6f previous_fault=%s',
                          request.request_id, r.episode, r.target_revision, r.lock_height, r.target, previous_fault)
            return self.detach_range_manual_response

    def _range_task_active(self):
        return bool(getattr(self, 'detach_range_enabled', False)
                    and getattr(self, 'detach_vertical_active', False)
                    and hasattr(self, 'detach_range') and self.detach_range.started is not None)

    def _on_range_handover_readiness(self, message):
        try:
            value = json.loads(message.data)
            with getattr(self, 'feedback_lock', nullcontext()):
                now = rospy.Time.now()
                if (value.get('episode') != self.detach_range.episode
                        or value.get('maneuver_id') != getattr(self, 'maneuver_id', '')
                        or value.get('target_revision', 0) != self.detach_range.target_revision
                        or not 0 <= now.to_sec()-float(value['source_time']) <= .5):
                    return
                self.manager_handover_snapshot = (value, now)
                request = float(value.get('posctl_request_time', 0.))
                if request > 0 and self.detach_range.started is not None and request >= self.detach_range.started:
                    self.detach_range_posctl_request_at = request
        except (ValueError, KeyError, TypeError, AttributeError):
            return

    def _range_locked(self):
        return self._range_task_active() and self.detach_range.locked

    def _on_detach_range(self, message):
        with getattr(self, 'feedback_lock', nullcontext()):
            if not self._range_task_active():
                return
            now = rospy.Time.now()
            self.detach_range.observe_range(message.header.stamp.to_sec(), now.to_sec(), message.distance)
            self._sync_range_lock(now)

    def _observe_range_pose(self, message, now):
        tracker = getattr(self, 'detach_range', None)
        if tracker is None:
            return
        q = message.pose.pose.orientation
        tracker.observe_pose(message.header.stamp.to_sec(), now.to_sec(), (q.x, q.y, q.z, q.w))
        self._sync_range_lock(now)

    def _sync_range_lock(self, now):
        if not self._range_locked() or self.detach_range_synced:
            return
        c, r = self.detach_vertical_controller, self.detach_range
        # Last actual publication seeds continuity, never a proposed candidate.
        output = getattr(self, 'horizontal_output', None)
        actual = float(output.thrust) if output is not None else math.nan
        if not math.isfinite(actual) or not .1-1.e-6 <= actual <= c.max_thrust+1.e-6:
            self.detach_vertical_fault = '测距首帧已锁定但实际输出推力不可用'
            return
        c.begin(r.target, c.base, actual, now.to_sec())
        c.target_speed = math.nan  # First range has no measured velocity yet.
        self.detach_altitude_m = self.recovery_altitude_m = r.target
        self.vertical_reference_source = 'range_first_qualified_message'
        self.detach_range_synced = True
        self.detach_task_phase = 'FAULT' if self.detach_range_faulted else 'RANGE_HEIGHT_CONTROL'
        # Old Z continuity/reanchors have no authority after this atomic switch.
        self.detach_height_continuation.reset()
        self.detach_vertical_fault = ''
        self.detach_feedback_valid = self.recovery_support_valid = False
        rospy.loginfo('wall range: episode=%s target=%.6f raw=%.6f source=%.9f receipt=%.9f tilt=%.3f pose_dt=%.6f',
                      r.episode, r.target, r.raw_distance, r.lock_source, r.lock_received,
                      r.sample_tilt_deg, r.pose_match_dt_s)

    def _prepare_range_feedback(self, now):
        r = self.detach_range
        s = r.snapshot(now.to_sec())
        self._sync_range_lock(now)
        self.detach_range_snapshot = s
        self.detach_range_block_reason = (self.detach_vertical_fault or getattr(self, 'last_reason', '')
                                          if self.detach_range_faulted else s.reason)
        if not r.locked:
            q = self._current_quaternion()
            if q and rotate_vector(q, (0., 0., 1.))[2] >= math.cos(math.radians(r.config.entry_tilt_deg)):
                self.detach_range_block_reason = '倾角已满足，等待首条有效测距；'+s.reason
            return
        self.detach_feedback_valid = bool(s.valid and self._detach_vertical_feedback_ok(now))
        seconds = now.to_sec()
        failure = ''
        if seconds < r.action_started:
            failure = '测距控制时钟倒退'
        elif seconds-r.action_started >= r.config.control_timeout_s:
            failure = '测距闭环总时间预算超时'
        elif r.invalid_since is not None and seconds-r.invalid_since >= r.config.loss_support_s:
            failure = '测距失效超过短时支撑上限'
        elif (self.detach_range_handover_started is not None
              and seconds-self.detach_range_handover_started >= r.config.handover_wait_s):
            failure = '测距交回等待超时'
        if failure and not self.detach_range_faulted:
            self.detach_range_failure_cause = failure
            self.detach_range_faulted = True
            self.detach_task_phase = 'FAULT'
            self.detach_vertical_fault = failure
            self.detach_range_block_reason = failure

    def _range_height_feedback(self):
        s = self.detach_range.snapshot(rospy.Time.now().to_sec())
        return (s.height, s.velocity) if s.valid else None

    def _range_height_settled(self, now):
        s, c = self.detach_range.snapshot(now.to_sec()), self.detach_range.config
        return bool(s.valid and s.velocity_valid and abs(s.target-s.height) <= c.height_tolerance_m
                    and abs(s.velocity) <= c.speed_tolerance_mps)

    def _range_vertical_thrust(self, now):
        self.recovery_support_valid = False
        c, r = self.detach_vertical_controller, self.detach_range
        s = r.snapshot(now.to_sec())
        flight = getattr(self, 'flight_state', None)
        if not (flight and flight.connected and flight.armed and flight.mode.upper() == 'OFFBOARD'
                and is_fresh_receipt(self.flight_state_received_at, self.state_timeout_s, now)
                and self._detach_vertical_feedback_ok(now)):
            c.clear_integral()
            return math.nan
        output = self._detach_vertical_output(now, allow_recovery=True)
        if output is None or not self.detach_range_synced:
            c.clear_integral()
            return math.nan
        if not s.valid:
            c.clear_integral()
            self.detach_vertical_reason = s.reason+'；目标保留，禁止旧Z回退'
            # Bounded hold uses the last actual collective, no stale error drive.
            if r.invalid_since is None or now.to_sec()-r.invalid_since >= r.config.loss_support_s:
                return math.nan
            self.detach_vertical_value = float(output.thrust)
            c.value = c.requested = self.detach_vertical_value
            c.correction, c.target_speed = 0., math.nan
            c.slew_limited = c.saturated = False
            self.recovery_support_valid = True
            self.recovery_thrust_limit = c.max_thrust
            return self.detach_vertical_value
        weight = min(1., max(0., (now.to_sec()-r.action_started)/r.config.control_blend_s))
        velocity_weight = (min(1., max(0., (now.to_sec()-r.velocity_ready_at)/r.config.control_blend_s))
                           if s.velocity_valid and r.velocity_ready_at is not None else 0.)
        try:
            thrust = c.update(s.height, s.velocity if s.velocity_valid else None,
                rotate_vector(self._current_quaternion(), (0., 0., 1.))[2],
                now.to_sec(), s.source, float(output.thrust),
                integrate=s.velocity_valid and velocity_weight >= 1. and not self.detach_vertical_output_degraded,
                correction_weight=weight, height_only_kp=r.config.height_only_kp,
                velocity_weight=velocity_weight)
        except ValueError as error:
            self.detach_vertical_fault = str(error)
            self.detach_range_failure_cause = self.detach_range_failure_cause or str(error)
            self.detach_range_faulted = True
            self.detach_task_phase = 'FAULT'
            return math.nan
        self.detach_vertical_value = thrust
        self.recovery_support_valid = True
        self.recovery_thrust_limit = c.max_thrust
        self.vertical_target_speed, self.vertical_correction = c.target_speed, c.correction
        self.detach_vertical_reason = s.reason+('；受限输出' if c.saturated else '')
        return thrust

    def _range_stability_ready(self, now):
        """All gates run continuously; duplicate timer ticks add no samples."""
        s, c = self.detach_range.snapshot(now.to_sec()), self.detach_range.config
        reason = ''
        if self.detach_range_faulted or self.state == W.STATE_ABORT:
            reason = '本轮异常，不能标记正常交回'
        elif not self._detach_vertical_feedback_ok(now):
            reason = self.detach_vertical_reason
        elif not self._range_height_settled(now):
            reason = '测距高度/速度尚未稳定或新鲜有效'
        elif self._detach_progress() < 1. or not self._detach_rotation_pose_valid():
            reason = '等待姿态轨迹完成、实测倾角≤5°及角速度回稳'
        elif not self._published_recovery_target_ready(now):
            reason = '等待实际发送姿态回稳'
        elif not self._horizontal_recovery_ready(now):
            reason = '等待水平速度和离墙运动稳定'
        elif not self._departure_clearance_ready(now):
            reason = getattr(self, 'departure_clearance_reason', '等待离墙距离和航向确认')
        elif not self._flight_ready(now) or self._detach_vertical_output(now) is None:
            reason = '等待新鲜OFFBOARD权限与实际输出'
        elif not self._wall_fusion_samples_ready(now):
            reason = getattr(self, 'wall_fusion_reason', '') or '等待飞控融合参数恢复确认'
        elif not self._range_px4_ready(now):
            snapshot = getattr(self, 'manager_handover_snapshot', None)
            reason = (snapshot[0].get('reason') if snapshot else '') or '等待PX4接管估计就绪及有效遥控通道'
        if reason:
            self.detach_range_stable_since = self.detach_range_stable_source = None
            self.detach_range_stable_samples = 0
            self.detach_range_block_reason = reason
            return False
        if s.source != self.detach_range_stable_source:
            self.detach_range_stable_source = s.source
            self.detach_range_stable_samples += 1
            if self.detach_range_stable_since is None:
                self.detach_range_stable_since = now.to_sec()
        ready = (self.detach_range_stable_samples >= c.stable_min_samples
                 and now.to_sec()-self.detach_range_stable_since >= c.stable_time_s)
        self.detach_range_block_reason = '具备交回资格，等待POSCTL确认' if ready else '等待全部条件连续稳定及足够新测距'
        return ready

    def _range_px4_ready(self, now):
        e = getattr(self, 'horizontal_estimator', None)
        snapshot = getattr(self, 'manager_handover_snapshot', None)
        # Manager publishes only independent PX4/RC readiness, not task stability.
        return bool(e is not None and is_fresh_receipt(getattr(self, 'horizontal_estimator_at', None), 1., now)
            and e.attitude_status_flag and e.velocity_vert_status_flag and e.pos_vert_abs_status_flag
            and e.velocity_horiz_status_flag and (e.pos_horiz_rel_status_flag or e.pos_horiz_abs_status_flag)
            and snapshot is not None and snapshot[0].get('ready', False)
            and snapshot[0].get('episode') == self.detach_range.episode
            and snapshot[0].get('maneuver_id') == getattr(self, 'maneuver_id', '')
            and snapshot[0].get('target_revision', 0) == self.detach_range.target_revision
            and 0 <= now.to_sec()-snapshot[0]['source_time'] <= .5
            and is_fresh_receipt(snapshot[1], .5, now))

    def _fill_range_status(self, status):
        r = getattr(self, 'detach_range', None)
        if r is None:
            return
        now = status.header.stamp.to_sec()
        s = r.snapshot(now)
        names = ('NONE', 'ROTATING_TO_RANGE_ENTRY', 'RANGE_HEIGHT_CONTROL', 'HANDOVER_READY', 'FAULT', 'COMPLETED')
        phase = getattr(self, 'detach_task_phase', 'NONE')
        status.detach_task_phase = names.index(phase)
        status.detach_task_phase_name = phase
        status.detach_range_height_enabled = getattr(self, 'detach_range_enabled', False)
        status.detach_range_episode = r.episode
        status.detach_range_maneuver_id = getattr(self, 'maneuver_id', '')
        status.detach_range_manual_target_m = r.config.manual_target_m
        status.detach_range_target_revision = r.target_revision
        status.detach_range_target_source = r.target_source
        status.detach_range_action_started_time = rospy.Time.from_sec(r.action_started or 0.)
        status.detach_range_manual_request_id = self.detach_range_manual_request_id
        status.detach_range_manual_recovery = self.detach_range_manual_recovery
        status.detach_range_previous_fault = self.detach_range_previous_fault
        status.detach_range_manual_block_reason = self._manual_range_height_block_reason(status.header.stamp)
        status.detach_range_manual_available = not bool(status.detach_range_manual_block_reason)
        status.detach_request_time = getattr(self, 'detach_requested_at', None) or rospy.Time()
        status.detach_rotation_start_time = rospy.Time.from_sec(r.started or 0.)
        status.detach_range_target_locked = r.locked
        status.detach_range_source = 'optical_flow_rad.distance/message_stamp'
        status.detach_range_target_m, status.detach_range_height_m = s.target, s.height
        status.detach_range_velocity_mps = s.velocity
        status.detach_range_velocity_valid, status.detach_range_valid = s.velocity_valid, s.valid
        status.detach_range_raw_m, status.detach_range_projected_m = s.raw_distance, s.projected_height
        status.detach_range_sample_tilt_deg, status.detach_range_pose_match_dt_s = s.sample_tilt_deg, s.pose_match_dt_s
        status.detach_range_entry_tilt_deg = r.config.entry_tilt_deg
        q = self._current_quaternion()
        status.detach_range_current_tilt_deg = math.degrees(math.acos(max(-1., min(1., rotate_vector(q, (0., 0., 1.))[2])))) if q else math.nan
        status.detach_range_source_time = rospy.Time.from_sec(s.source)
        status.detach_range_received_time = rospy.Time.from_sec(s.received)
        status.detach_range_lock_source_time = rospy.Time.from_sec(r.lock_source or 0.)
        status.detach_range_lock_received_time = rospy.Time.from_sec(r.lock_received or 0.)
        status.detach_range_lock_raw_m = r.lock_raw
        status.detach_range_lock_height_m = r.lock_height
        status.detach_range_lock_tilt_deg = r.lock_tilt
        status.detach_range_velocity_samples, status.detach_range_age_s = s.sample_count, s.age
        since = getattr(self, 'detach_range_stable_since', None)
        status.detach_range_stable_elapsed_s = max(0., now-since) if since is not None else 0.
        status.detach_range_stable_samples = getattr(self, 'detach_range_stable_samples', 0)
        status.detach_range_handover_ready = phase == 'HANDOVER_READY'
        status.detach_range_block_reason = getattr(self, 'detach_range_block_reason', s.reason)
        status.detach_range_config_json = json.dumps(asdict(r.config), sort_keys=True)
        status.detach_range_rejections_json = json.dumps(r.rejected, sort_keys=True, ensure_ascii=False)
        status.detach_posctl_request_time = rospy.Time.from_sec(getattr(self, 'detach_range_posctl_request_at', None) or 0.)
        status.detach_posctl_confirmed_time = rospy.Time.from_sec(getattr(self, 'detach_range_posctl_confirmed_at', None) or 0.)
        if self._range_locked():
            status.detach_height_error_m = s.target-s.height if s.valid else math.nan
            status.vertical_reference_source = r.target_source
            status.vertical_reference_valid = r.locked
            status.vertical_reference_m = r.target
            status.vertical_control_active = bool(s.valid and self.recovery_support_valid)
