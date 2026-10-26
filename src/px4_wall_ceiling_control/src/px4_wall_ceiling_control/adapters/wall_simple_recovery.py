"""Keep existing height/hover support; hand over after attitude stabilizes."""
import math
from px4_wall_ceiling_control.adapters.odometry import linear_velocity_parent_frame
from px4_wall_ceiling_control.domain.geometry import rotate_vector
from px4_wall_ceiling_control.domain.recovery_attitude_limits import attitude_from_axis_yaw
from px4_wall_ceiling_control.domain.simple_recovery import (
    STABLE_TIME_S, attitude_within_margins,
)
from px4_wall_ceiling_control.freshness import is_fresh_receipt, is_fresh_source
from px4_wall_ceiling_control.msg import WallPerchStatus as W


class WallSimpleRecoveryMixin:
    def _begin_simple_recovery(self, now):
        self.recover_baseline_axis = rotate_vector(self.recover_entry_attitude, (0., 0., 1.))
        self.recover_baseline_at = now
        self.departure_attitude_controller.reset(self.recover_baseline_axis, self.departure_yaw)
        self.recovery_attitude_at = now
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

    def _update_simple_departure_clearance(self, now):
        # No automatic retreat or wall-distance admission in captured RECOVER.
        self.departure_feedback_valid = False
        self.departure_clearance_reason = '保持翻转结束姿态，无自动后退；等待CH12确认交回'

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
        """Keep the measured rotation-end target until FCU confirms handover."""
        self._capture_simple_recovery_height(now)
        # Horizontal damping and baseline leveling both alter attitude. Disable
        # them in this hold; vertical thrust remains closed loop independently.
        self.recovery_horizontal_controller.reset()
        self.recovery_horizontal_controller.target = (0., 0.)
        self.horizontal_velocity_active = False
        self.horizontal_velocity_reason = '保持翻转结束姿态；高度闭环继续，等待CH12确认交回'
        return self.recover_entry_attitude

    def _simple_recovery_stability(self, now):
        q = self._current_quaternion()
        reason = ''
        if self.detach_range_faulted or not self._odom_fresh(now) or q is None:
            reason = '等待新鲜有效姿态且本轮无故障'
        elif not self._flight_ready(now) or self._valid_wall_output(now) is None:
            reason = '等待新鲜OFFBOARD权限和最终输出'
        else:
            angular = self.odom.twist.twist.angular
            tilt = math.acos(max(-1., min(1., rotate_vector(q, (0., 0., 1.))[2])))
            rate = math.sqrt(angular.x**2+angular.y**2+angular.z**2)
            if not attitude_within_margins(tilt, rate):
                reason = '等待总倾角≤10°、总角速度≤20°/s连续稳定0.5秒'
        if reason:
            self.detach_range_stable_since = self.detach_range_stable_source = None
            self.detach_range_stable_samples = 0
            self.recover_stable = self.horizontal_velocity_stable = False
            self.detach_range_block_reason = reason
            return False
        source = self.odom.header.stamp.to_sec()
        previous = self.detach_range_stable_source
        if previous is not None and (source < previous or source-previous > self.odom_timeout_s):
            self.detach_range_stable_since = None
            self.detach_range_stable_samples = 0
        if source != previous:
            self.detach_range_stable_source = source
            self.detach_range_stable_samples += 1
            if self.detach_range_stable_since is None:
                self.detach_range_stable_since = now.to_sec()
        self.recover_stable = bool(self.detach_range_stable_samples >= 2
            and source-self.detach_range_stable_since >= STABLE_TIME_S-1.e-9)
        self.horizontal_velocity_stable = self.recover_stable
        self.detach_range_block_reason = '姿态稳定，保持结束姿态；等待CH12确认及POSCTL交接' if self.recover_stable else '姿态连续稳定确认中'
        return self.recover_stable
