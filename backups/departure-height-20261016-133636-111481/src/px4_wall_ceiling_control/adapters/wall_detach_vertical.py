"""Continuous height support from accepted departure through handover."""
import math

import rospy
from mavros_msgs.msg import AttitudeTarget, EstimatorStatus

from px4_wall_ceiling_control.domain.departure_height import DepartureHeightController
from px4_wall_ceiling_control.domain.geometry import rotate_vector
from px4_wall_ceiling_control.freshness import is_fresh_receipt, is_fresh_source
from px4_wall_ceiling_control.msg import AttachmentControlCandidate as C


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
        self._reset_detach_vertical()
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
        self.detach_vertical_last_feedback = None
        self.detach_vertical_value = math.nan
        self.recovery_support_valid = False
        self.recovery_thrust_limit = .0

    def _detach_vertical_feedback_ok(self, now):
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

    def _detach_vertical_output(self, now):
        output = getattr(self, 'horizontal_output', None)
        if (output is None or output.type_mask & (AttitudeTarget.IGNORE_THRUST | AttitudeTarget.IGNORE_ATTITUDE)
                or not is_fresh_receipt(getattr(self, 'horizontal_output_at', None), .20, now)
                or not is_fresh_source(output.header.stamp, .20, now, future_tolerance_s=0.)
                or getattr(self, 'horizontal_output_owner', None) != C.SOURCE_WALL
                or not is_fresh_receipt(getattr(self, 'horizontal_output_owner_at', None), .30, now)
                or not math.isfinite(output.thrust)
                or not .1 <= output.thrust <= self.detach_vertical_controller.max_thrust):
            self.detach_vertical_reason = '等待新鲜侧吸最终输出以连续衔接推力'
            return None
        q = tuple(getattr(output.orientation, k) for k in 'xyzw')
        if not all(math.isfinite(v) for v in q) or abs(sum(v*v for v in q)-1.) > .01:
            self.detach_vertical_reason = '最终输出姿态无效'
            return None
        return output

    def _begin_detach_vertical(self, now, output):
        self._reset_detach_vertical()
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
        self.detach_vertical_value = actual.thrust
        self.recovery_thrust_limit = self.detach_vertical_controller.max_thrust
        self.detach_vertical_reason = '已锁定脱离开始高度，继承实际输出推力'
        self.detach_vertical_last_feedback = (self.odom.header.stamp.to_sec(), feedback[0], feedback[1])
        return True

    def _detach_vertical_thrust(self, now, dt):
        # dt is also used by the attitude trajectory. The controller additionally
        # enforces elapsed time and source-sample bounds; duplicate evaluation is idempotent.
        if not getattr(self, 'detach_vertical_active', False):
            return math.nan
        controller = self.detach_vertical_controller
        self.recovery_support_valid = False
        if self.detach_vertical_fault:
            self.detach_vertical_reason = self.detach_vertical_fault
            return math.nan
        flight = getattr(self, 'flight_state', None)
        authority = bool(flight is not None and flight.connected and flight.armed and flight.mode.upper() == 'OFFBOARD'
                         and is_fresh_receipt(getattr(self, 'flight_state_received_at', None), self.state_timeout_s, now))
        if not authority:
            controller.clear_integral()
            self.detach_vertical_reason = '脱离保高等待有效OFFBOARD权限'
            return math.nan
        if not self._detach_vertical_feedback_ok(now):
            controller.clear_integral()
            return math.nan
        output = self._detach_vertical_output(now)
        if output is None:
            controller.clear_integral()
            return math.nan
        height, speed = self._height_feedback()
        source = self.odom.header.stamp.to_sec()
        old = self.detach_vertical_last_feedback
        if old is not None and source > old[0]:
            source_dt = source-old[0]
            residual = abs(height-old[1]-.5*(speed+old[2])*source_dt)
            if source_dt <= self.odom_timeout_s and residual > .25:
                controller.clear_integral()
                self.detach_vertical_fault = '脱离高度估计发生不连续跳变'
                self.detach_vertical_reason = self.detach_vertical_fault
                return math.nan
        if old is None or source > old[0]:
            self.detach_vertical_last_feedback = (source, height, speed)
        axis = rotate_vector(self._current_quaternion(), (0., 0., 1.))
        try:
            thrust = controller.update(height, speed, axis[2], now.to_sec(), source, float(output.thrust), integrate=True)
        except ValueError as error:
            controller.clear_integral()
            self.detach_vertical_fault = self.detach_vertical_reason = str(error)
            return math.nan
        self.detach_vertical_value = thrust
        self.recovery_support_valid = True
        self.recovery_thrust_limit = controller.max_thrust
        self.vertical_target_speed, self.vertical_correction = controller.target_speed, controller.correction
        self.detach_vertical_reason = ('近竖直仍需墙面/机构承重，保持连续压力' if controller.support_weight < 1.-1e-9
            else '脱离保高输出受限' if controller.saturated else '脱离保高闭环有效')
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
        status.detach_vertical_base_thrust = controller.base if active else math.nan
        status.detach_vertical_support_weight = controller.support_weight if active else 0.
        status.detach_height_error_m = (controller.reference-self._height_feedback()[0]
                                        if active and self._height_feedback() is not None else math.nan)
        if active:
            status.vertical_control_active = True
            status.vertical_reference_valid = True
            status.vertical_reference_m = controller.reference
            status.vertical_reference_source = 'detach_start_height'
            status.vertical_target_enu_mps = controller.target_speed
            status.vertical_thrust_correction = controller.correction
