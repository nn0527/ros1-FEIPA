"""Phase and sensor gates for the optional wall horizontal velocity loop."""
import math

import rospy
from mavros_msgs.msg import OpticalFlowRad, EstimatorStatus

from px4_wall_ceiling_control.adapters.odometry import linear_velocity_parent_frame
from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion, rotate_vector
from px4_wall_ceiling_control.domain.horizontal_velocity import HorizontalVelocityController
from px4_wall_ceiling_control.freshness import is_fresh
from px4_wall_ceiling_control.msg import WallPerchStatus as W


class WallHorizontalVelocityMixin:
    def _init_horizontal_velocity(self):
        self.horizontal_velocity_enabled = bool(rospy.get_param('~horizontal_velocity_enabled', False))
        self.horizontal_approach_speed = float(rospy.get_param('~horizontal_approach_speed_mps', .05))
        self.horizontal_resume_tilt = math.radians(float(rospy.get_param('~horizontal_resume_tilt_deg', 10.)))
        self.horizontal_stable_speed = float(rospy.get_param('~horizontal_stable_speed_mps', .10))
        self.horizontal_stable_time = float(rospy.get_param('~horizontal_stable_time_s', 1.))
        self.horizontal_flow_timeout = float(rospy.get_param('~horizontal_flow_timeout_s', .60))
        self.horizontal_flow_quality_min = int(rospy.get_param('~horizontal_flow_quality_min', 1))
        self.horizontal_reacquire_time = float(rospy.get_param('~horizontal_reacquire_time_s', .30))
        values = (self.horizontal_approach_speed,self.horizontal_resume_tilt,self.horizontal_stable_speed,
                  self.horizontal_stable_time,self.horizontal_flow_timeout,self.horizontal_reacquire_time)
        if any(not math.isfinite(v) or v <= 0 for v in values) or self.horizontal_approach_speed > .3 or not 1 <= self.horizontal_flow_quality_min <= 255:
            raise ValueError('invalid wall horizontal velocity settings')
        self.horizontal_controller = HorizontalVelocityController(
            float(rospy.get_param('~horizontal_velocity_kp', 1.5)),
            float(rospy.get_param('~horizontal_max_acceleration_mps2', .30)),
            float(rospy.get_param('~horizontal_target_acceleration_mps2', .10)))
        self.horizontal_flow = self.horizontal_flow_at = None
        self.horizontal_estimator = self.horizontal_estimator_at = None
        self.horizontal_measured = (math.nan,math.nan)
        self.horizontal_velocity_active = False
        self._reset_horizontal_gate('水平速度闭环未启用' if not self.horizontal_velocity_enabled else '等待光流与水平速度反馈')
        if self.horizontal_velocity_enabled:
            rospy.Subscriber('/mavros/px4flow/raw/optical_flow_rad', OpticalFlowRad, self._on_horizontal_flow, queue_size=10)
            rospy.Subscriber('/mavros/estimator_status', EstimatorStatus, self._on_horizontal_estimator, queue_size=10)

    def _on_horizontal_flow(self, message):
        self.horizontal_flow,self.horizontal_flow_at = message,rospy.Time.now()

    def _on_horizontal_estimator(self, message):
        self.horizontal_estimator,self.horizontal_estimator_at = message,rospy.Time.now()

    def _reset_horizontal_gate(self, reason):
        self.horizontal_measured = (math.nan,math.nan)
        self.horizontal_gate_at = None
        self.horizontal_confirm_at = self.horizontal_last_flow_sample = None
        self.horizontal_flow_samples = 0
        self.horizontal_stable_since = self.horizontal_stable_last_odom = None
        self.horizontal_stable_samples = 0
        self.horizontal_velocity_ready = self.horizontal_velocity_stable = self.horizontal_velocity_active = False
        self.horizontal_velocity_reason = reason
        self.horizontal_controller.reset()

    def _horizontal_phase_allowed(self, now):
        if self.state in (W.STATE_IDLE,W.STATE_FRONT_WALL_DETECT,W.STATE_STABILIZE_HOVER,
                          W.STATE_SLOW_APPROACH,W.STATE_RECOVER,W.STATE_EXIT,W.STATE_ABORT):
            return True
        if self.state == W.STATE_DETACH_ROTATE:
            progress = min(1.,max(0.,self._state_age(now)/self.detach_time_s))
            target = self._quat_slerp(self.q_detach_start,self.q_hover,self._smoothstep5(progress))
            # Resume braking only in the final near-level part of the original rotation.
            return rotate_vector(target,(0.,0.,1.))[2] >= math.cos(math.radians(5.))
        return False

    def _prepare_horizontal_velocity(self, now):
        if not getattr(self,'horizontal_velocity_enabled',False):return
        if not self._horizontal_phase_allowed(now):
            self._reset_horizontal_gate('翻转或贴墙阶段：水平速度闭环暂停')
            return
        q = self._current_quaternion()
        if q is None or not self._odom_fresh(now):
            self._reset_horizontal_gate('等待新鲜有效里程计');return
        axis = rotate_vector(q,(0.,0.,1.))
        tilt = math.acos(max(-1.,min(1.,axis[2])))
        angular = self.odom.twist.twist.angular
        rate = math.sqrt(angular.x**2+angular.y**2+angular.z**2)
        if not math.isfinite(rate) or tilt > self.horizontal_resume_tilt or rate > math.radians(45.):
            self._reset_horizontal_gate('等待机体接近水平（≤10°）且角速度≤45°/s');return
        if self.horizontal_gate_at is None:self.horizontal_gate_at = now
        velocity = linear_velocity_parent_frame(self.odom,self.odom_twist_in_child_frame)
        self.horizontal_measured = velocity[:2]
        estimate = self.horizontal_estimator
        flow = self.horizontal_flow
        reason = ''
        if not all(math.isfinite(v) for v in velocity):reason = '水平速度数值无效'
        elif not is_fresh(self.horizontal_estimator_at,1.,now) or estimate is None:reason = '估计器反馈过期'
        elif not (estimate.attitude_status_flag and estimate.velocity_horiz_status_flag) or estimate.const_pos_mode_status_flag:reason = 'PX4水平速度估计未有效（或处于恒位置推算）'
        elif not is_fresh(self.horizontal_flow_at,self.horizontal_flow_timeout,now) or flow is None:reason = '光流反馈过期'
        elif (self.horizontal_flow_at < self.horizontal_gate_at or self.horizontal_estimator_at < self.horizontal_gate_at):reason = '等待恢复水平后的新光流及估计器样本'
        elif flow.quality < self.horizontal_flow_quality_min or flow.integration_time_us <= 0 or not all(math.isfinite(v) for v in (flow.integrated_x,flow.integrated_y,flow.integrated_xgyro,flow.integrated_ygyro,flow.distance)) or flow.distance <= 0:reason = '光流质量、积分时间或测距无效'
        elif flow.header.stamp.to_sec() and not is_fresh(flow.header.stamp,self.horizontal_flow_timeout,now):reason = '光流源时间戳过期'
        if reason:
            self.horizontal_confirm_at = self.horizontal_last_flow_sample = None
            self.horizontal_flow_samples = 0
            self.horizontal_velocity_ready = False
            self.horizontal_velocity_reason = reason
        else:
            if self.horizontal_last_flow_sample != self.horizontal_flow_at:
                self.horizontal_last_flow_sample = self.horizontal_flow_at
                self.horizontal_flow_samples += 1
                if self.horizontal_confirm_at is None:self.horizontal_confirm_at = self.horizontal_flow_at
            self.horizontal_velocity_ready = bool(self.horizontal_flow_samples >= 2 and self.horizontal_confirm_at is not None
                and (now-self.horizontal_confirm_at).to_sec() >= self.horizontal_reacquire_time)
            self.horizontal_velocity_reason = '水平速度反馈有效' if self.horizontal_velocity_ready else '等待光流连续有效确认'
        recovery = self.state in (W.STATE_DETACH_ROTATE,W.STATE_RECOVER,W.STATE_EXIT,W.STATE_ABORT)
        stable = bool(recovery and self.horizontal_velocity_ready and tilt <= math.radians(5.)
                      and rate <= math.radians(10.) and math.hypot(*self.horizontal_measured) <= self.horizontal_stable_speed)
        if not stable:
            self.horizontal_stable_since = self.horizontal_stable_last_odom = None
            self.horizontal_stable_samples = 0
        elif self.horizontal_stable_last_odom != self.odom_received_at:
            self.horizontal_stable_last_odom = self.odom_received_at
            self.horizontal_stable_samples += 1
            if self.horizontal_stable_since is None:self.horizontal_stable_since = self.odom_received_at
        self.horizontal_velocity_stable = bool(stable and self.horizontal_stable_samples >= 3
            and self.horizontal_stable_since is not None and (self.odom_received_at-self.horizontal_stable_since).to_sec() >= self.horizontal_stable_time)

    def _horizontal_recovery_ready(self, now):
        if not getattr(self,'horizontal_velocity_enabled',False):return True
        self._prepare_horizontal_velocity(now)
        return self.horizontal_velocity_stable

    def _horizontal_attitude(self, now, quaternion):
        if not getattr(self,'horizontal_velocity_enabled',False):return quaternion
        self._prepare_horizontal_velocity(now)
        self.horizontal_velocity_active = False
        stages = (W.STATE_SLOW_APPROACH,W.STATE_DETACH_ROTATE,W.STATE_RECOVER,W.STATE_EXIT,W.STATE_ABORT)
        if quaternion is None or self.state not in stages or not self._horizontal_phase_allowed(now) or not self.horizontal_velocity_ready:
            self.horizontal_controller.reset();return quaternion
        yaw = euler_from_quaternion(self.q_hover)[2]
        forward = self.horizontal_approach_speed if self.state == W.STATE_SLOW_APPROACH and self._auto_approach_ranges_ready(now) else 0.
        desired = (forward*math.cos(yaw),forward*math.sin(yaw))
        quaternion = self.horizontal_controller.attitude(desired,self.horizontal_measured,yaw,self.control_dt)
        self.horizontal_velocity_active = True
        self.horizontal_velocity_reason = ('水平速度接近：目标前向%.2f m/s、横向0' % forward
            if self.state == W.STATE_SLOW_APPROACH else '恢复水平制动：等待水平速度和姿态持续稳定')
        return quaternion

    def _fill_horizontal_status(self, status):
        status.horizontal_velocity_enabled = getattr(self,'horizontal_velocity_enabled',False)
        if not status.horizontal_velocity_enabled:return
        status.horizontal_velocity_ready = self.horizontal_velocity_ready
        status.horizontal_velocity_active = self.horizontal_velocity_active
        status.horizontal_velocity_stable = self.horizontal_velocity_stable
        status.horizontal_velocity_reason = self.horizontal_velocity_reason
        status.horizontal_velocity_x_enu_mps,status.horizontal_velocity_y_enu_mps = self.horizontal_measured
        status.horizontal_target_x_enu_mps,status.horizontal_target_y_enu_mps = self.horizontal_controller.target
        status.horizontal_accel_x_enu_mps2,status.horizontal_accel_y_enu_mps2 = self.horizontal_controller.acceleration
        status.optical_flow_quality = self.horizontal_flow.quality if self.horizontal_flow is not None else 0
