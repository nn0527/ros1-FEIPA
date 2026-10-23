"""Reanchor height, blend to trusted hover, retreat until one clearance frame."""
import math
from px4_wall_ceiling_control.adapters.odometry import linear_velocity_parent_frame
from px4_wall_ceiling_control.domain.geometry import rotate_vector, euler_from_quaternion
from px4_wall_ceiling_control.domain.recovery_attitude_limits import attitude_from_axis_yaw
from px4_wall_ceiling_control.domain.simple_recovery import (
    MAX_TILT_DEG, STABLE_TIME_S, within_margins,
)
from px4_wall_ceiling_control.freshness import is_fresh_receipt, is_fresh_source
from px4_wall_ceiling_control.msg import WallPerchStatus as W


class WallSimpleRecoveryMixin:
    def _begin_simple_recovery(self, now):
        self.recover_baseline_axis = rotate_vector(self.recover_entry_attitude, (0., 0., 1.))
        self.recover_baseline_at = now
        self.recover_height_captured = False
        self._capture_simple_recovery_height(now)

    def _capture_simple_recovery_height(self, now):
        if getattr(self, 'recover_height_captured', False):
            return True
        r, c = self.detach_range, self.detach_vertical_controller
        if not self.detach_range_synced or not r.read_snapshot(now.to_sec()).valid:
            return False
        height = r.capture_recovery_target(now.to_sec())
        c.reanchor(height)  # Keep thrust value/time/slew, clear only old height error/integral.
        self.detach_altitude_m = self.recovery_altitude_m = height
        self.vertical_reference_source = r.target_source
        self.detach_range_stable_since = self.detach_range_stable_source = None
        self.detach_range_stable_samples = 0
        self.manager_handover_snapshot = None  # Old target-revision ACK cannot qualify.
        self.recover_height_captured = True
        return True

    def _observe_recovery_clearance(self, message, now):
        """Latch in the input callback, so one frame cannot be lost between ticks."""
        started = getattr(self, 'departure_started_at', None)
        if (not self._locked_recovery() or not getattr(self, 'departure_active', False)
                or self.departure_clear_confirmed or started is None
                or message.header.stamp <= started
                or not is_fresh_source(message.header.stamp, self.sensor_timeout_s, now, future_tolerance_s=0.)):
            return
        distance = float(message.range)
        if (not math.isfinite(distance) or not message.min_range <= distance <= message.max_range
                or distance <= self.departure_clearance_distance):
            return
        self.departure_distance = distance
        self.departure_clear_confirmed = True
        self.departure_clear_samples = 1
        self.departure_clear_last_stamp = self.departure_clear_since = message.header.stamp
        self.departure_clearance_reason = '单帧墙距>%.2fm已锁存，停止退墙并申请接管' % self.departure_clearance_distance
        # Stop the velocity TARGET now; measured speed is then actively braked.
        self.recovery_horizontal_controller.target = (0., 0.)
        self.recovery_horizontal_controller.clear_integral()

    def _update_simple_departure_clearance(self, now):
        # A front-lidar dropout is not a velocity/attitude fault, nor does it
        # erase a previously latched clearance. No wall-health dwell is needed.
        front = getattr(self, 'front_range', None)
        if front is not None:
            self._observe_recovery_clearance(front, now)
        q = self._current_quaternion() if self._odom_fresh(now) else None
        yaw = getattr(self, 'departure_yaw', None)
        rate = getattr(getattr(getattr(self, 'odom', None), 'twist', None), 'twist', None)
        a = rate.angular if rate is not None else None
        valid = bool(self.horizontal_velocity_ready and q is not None and yaw is not None
            and rotate_vector(q, (0., 0., 1.))[2] >= math.cos(math.radians(MAX_TILT_DEG))
            and a is not None and all(math.isfinite(v) for v in (a.x, a.y, a.z))
            and math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z) <= math.radians(20.))
        if valid:
            measured_yaw = euler_from_quaternion(q)[2]
            valid = abs(math.atan2(math.sin(measured_yaw-yaw), math.cos(measured_yaw-yaw))) <= self.departure_heading_tolerance
        self.departure_feedback_valid = valid
        if self.departure_clear_confirmed:
            self.departure_clearance_reason = '单帧离墙距离已锁存，等待制动稳定及安全接管'
        else:
            self.departure_clearance_reason = ('向后退墙；等待一帧有效墙距>%.2fm（测距丢失不暂停）'
                % self.departure_clearance_distance if valid else '退墙暂停：等待有效速度、姿态及冻结航向')

    def _simple_recovery_baseline(self, now):
        """Level toward verified, frozen POSCTL feedforward even while EKF restores."""
        previous = getattr(self, 'recover_baseline_axis', rotate_vector(self.recover_entry_attitude, (0., 0., 1.)))
        at = getattr(self, 'recover_baseline_at', now)
        dt = min(.05, max(0., (now-at).to_sec()))
        self.recover_baseline_at = now
        equilibrium = getattr(self, 'horizontal_equilibrium', None)
        trusted = bool(equilibrium is not None and equilibrium.ready and equilibrium.frozen)
        if trusted:
            desired = equilibrium.axis
            angle = math.acos(max(-1., min(1., sum(a*b for a, b in zip(previous, desired)))))
            fraction = min(1., math.radians(10.)*dt/angle) if angle > 1.e-9 else 1.
            mixed = tuple(a+(b-a)*fraction for a, b in zip(previous, desired))
            length = math.sqrt(sum(v*v for v in mixed))
            previous = tuple(v/length for v in mixed)
        self.recover_baseline_axis = previous
        return previous, trusted

    def _locked_recovery(self):
        return bool(getattr(self, 'recover_attitude_locked', False)
                    and self.state in (W.STATE_RECOVER, W.STATE_EXIT))

    def _prepare_simple_recovery_velocity(self, now):
        """Accept current valid feedback without pose dwell, PI or reacquire windows."""
        q = self._current_quaternion()
        estimate = getattr(self, 'horizontal_estimator', None)
        snapshot = getattr(self, 'horizontal_flow_snapshot', None)
        flow, flow_at, frame_error = snapshot if snapshot is not None else (None, None, '')
        reason = ''
        if q is None or not self._odom_fresh(now):
            reason = '恢复等待新鲜姿态及水平速度'
        elif not self._wall_fusion_samples_ready(now):
            reason = self.wall_fusion_reason
        elif estimate is None or not is_fresh_receipt(self.horizontal_estimator_at, 1., now):
            reason = '水平估计器反馈过期'
        elif not (estimate.attitude_status_flag and estimate.velocity_horiz_status_flag) or estimate.const_pos_mode_status_flag:
            reason = 'PX4水平速度估计无效'
        elif not is_fresh_source(estimate.header.stamp, 1., now):
            reason = '水平估计器源时间过期'
        elif flow is None or not is_fresh_receipt(flow_at, self.horizontal_flow_timeout, now):
            reason = '光流反馈过期'
        elif (flow.quality < self.horizontal_flow_quality_min or flow.integration_time_us <= 0
              or not all(math.isfinite(v) for v in (flow.integrated_x, flow.integrated_y,
                         flow.integrated_xgyro, flow.integrated_ygyro, flow.distance)) or flow.distance <= 0):
            reason = '光流反馈无效'
        else:
            reason = self.horizontal_flow_clock.reason(flow.header.stamp.to_sec(), now.to_sec(), frame_error)
        if not reason:
            velocity = linear_velocity_parent_frame(self.odom, self.odom_twist_in_child_frame)[:2]
            if not all(math.isfinite(v) for v in velocity):
                reason = '水平速度数值无效'
            else:
                self.horizontal_measured = velocity
        self.horizontal_velocity_ready = not bool(reason)
        self.horizontal_velocity_reason = reason or '恢复水平速度反馈有效'
        self.horizontal_velocity_stable = bool(not reason and getattr(self, 'recover_stable', False))
        self._update_departure_clearance(now)

    def _simple_recovery_attitude(self, now):
        baseline, trusted = self._simple_recovery_baseline(now)
        yaw = self.departure_yaw
        quaternion = attitude_from_axis_yaw(baseline, yaw)
        height_captured = self._capture_simple_recovery_height(now)
        self._prepare_simple_recovery_velocity(now)
        self.horizontal_velocity_active = False
        if not self.horizontal_velocity_ready or not trusted or not height_captured:
            self.recovery_horizontal_controller.reset()
            self.recovery_control_at = now
            if not trusted:
                self.horizontal_velocity_reason = '可信悬停基准缺失，禁止猜测退墙姿态'
            elif not height_captured:
                self.horizontal_velocity_reason = '退墙等待实际测距高度基准'
            return quaternion
        forward = (-self.departure_retreat_speed
                   if self.state == W.STATE_RECOVER and self.departure_feedback_valid
                   and not self.departure_clear_confirmed else 0.)
        previous = getattr(self, 'recovery_control_at', None)
        dt = min(.05, max(0., (now-previous).to_sec())) if previous is not None else 0.
        self.recovery_control_at = now
        if self.departure_clear_confirmed:
            self.recovery_horizontal_controller.target = (0., 0.)
        # Five degrees after leveling; preserve a larger entry angle only
        # while its baseline is smoothly moving inward, never jump its target.
        tilt_limit = max(math.radians(5.), math.acos(max(-1., min(1., baseline[2]))))
        result = self.recovery_horizontal_controller.attitude(
            (forward*math.cos(yaw), forward*math.sin(yaw)), self.horizontal_measured,
            yaw, dt, baseline, integrate=False, max_tilt=tilt_limit)
        self.horizontal_velocity_active = True
        self.horizontal_velocity_reason = '可信悬停基准退墙：目标前向%.2f m/s；%s' % (forward, self.departure_clearance_reason)
        return result

    def _simple_recovery_stability(self, now):
        s = self.detach_range.read_snapshot(now.to_sec())
        self._prepare_simple_recovery_velocity(now)
        velocity, valid = self._range_control_velocity(s, now.to_sec())
        q = self._current_quaternion()
        reason = ''
        if (not getattr(self, 'recover_height_captured', False)
                or self.detach_range_faulted or not self._detach_vertical_feedback_ok(now)):
            reason = '等待有效姿态和测距闭环'
        elif not s.valid or not valid:
            reason = '等待新鲜测距高度及测距速度'
        elif not self.horizontal_velocity_ready or not self._departure_clearance_ready(now):
            reason = self.departure_clearance_reason
        elif not self._flight_ready(now) or self._valid_wall_output(now) is None:
            reason = '等待新鲜OFFBOARD权限和最终输出'
        else:
            angular = self.odom.twist.twist.angular
            tilt = math.acos(max(-1., min(1., rotate_vector(q, (0., 0., 1.))[2])))
            rate = math.sqrt(angular.x**2+angular.y**2+angular.z**2)
            if not within_margins(tilt, rate, s.target-s.height, velocity, math.hypot(*self.horizontal_measured)):
                reason = '等待姿态≤10°、角速≤20°/s、高差≤0.20m、垂直/水平速度≤0.25m/s'
            elif math.hypot(*self.recovery_horizontal_controller.target) > .01:
                reason = '等待退墙速度目标归零'
        if reason:
            self.detach_range_stable_since = self.detach_range_stable_source = None
            self.detach_range_stable_samples = 0
            self.recover_stable = self.horizontal_velocity_stable = False
            self.detach_range_block_reason = reason
            return False
        if s.source != self.detach_range_stable_source:
            self.detach_range_stable_source = s.source
            self.detach_range_stable_samples += 1
            if self.detach_range_stable_since is None:
                self.detach_range_stable_since = now.to_sec()
        self.recover_stable = bool(self.detach_range_stable_samples >= 2
            and now.to_sec()-self.detach_range_stable_since >= STABLE_TIME_S)
        self.horizontal_velocity_stable = self.recover_stable
        self.detach_range_block_reason = '稳定确认，等待POSCTL交接' if self.recover_stable else '余量范围内连续稳定确认中'
        return self.recover_stable
