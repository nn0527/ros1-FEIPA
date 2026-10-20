"""Explicit pilot takeover inside the sole setpoint publisher.

The manager acknowledges ownership before the first takeover setpoint. A failed
transition stays latched, so old attachment candidates cannot regain control.
"""
import math
import threading
import time
import rospy
from std_msgs.msg import Bool, String, UInt8
from std_srvs.srv import Trigger, TriggerResponse
from mavros_msgs.srv import ParamGet
from geometry_msgs.msg import Quaternion
from px4_wall_ceiling_control.adapters.kill_switch import kill_clear
from px4_wall_ceiling_control.adapters.odometry import vertical_speed_enu_mps
from px4_wall_ceiling_control.domain.geometry import quaternion_angle
from px4_wall_ceiling_control.domain.rc_attitude_assist import stick_fraction
from px4_wall_ceiling_control.domain.rc_takeover import RCTakeover
from px4_wall_ceiling_control.freshness import is_fresh_receipt as is_fresh, is_fresh_source


class RCTakeoverOutput:
    def _init_rc_takeover(self):
        self.rc_takeover_pending = False
        self.rc_takeover_controller = None
        self.rc_takeover_failed = False
        self.rc_takeover_lock = threading.RLock()
        self.rc_takeover_active_pub = rospy.Publisher('/attachment/rc_takeover/active', Bool, queue_size=1)
        self.rc_takeover_ready_pub = rospy.Publisher('/attachment/rc_takeover/ready', Bool, queue_size=1)
        self.rc_takeover_status_pub = rospy.Publisher('/attachment/rc_takeover/status', String, queue_size=1)
        self.rc_takeover_service = rospy.Service('/attachment/rc_takeover', Trigger, self._request_rc_takeover)

    def _takeover_inputs_ready(self, now):
        return bool(self.output_enabled and kill_clear(self, now)
                    and self.flight_state is not None and self.flight_state.connected
                    and self.flight_state.armed and self.flight_state.mode.upper() == 'OFFBOARD'
                    and is_fresh(self.flight_state_at, self.state_timeout_s, now)
                    and self._odom_fresh(now)
                    and is_fresh_source(self.odom.header.stamp, self.odom_timeout_s, now)
                    and is_fresh(self.rc_at, self.rc_timeout_s, now)
                    and is_fresh_source(getattr(self, 'rc_source_stamp', None), self.rc_timeout_s, now))

    def _takeover_axes(self):
        axes = {}
        for axis, (index, low, center, high, reverse) in self.rc_takeover_calibration.items():
            value = stick_fraction(self.rc_channels, index, low, center, high, self.rc_deadzone, reverse)
            if value is None:
                return None
            axes[axis] = -value if axis == 'yaw' else value
        return axes

    def _request_rc_takeover(self, _request):
        # Parameter RPCs run in the service thread, never the output timer.
        with self.rc_takeover_lock:
            if self.rc_takeover_pending:
                return TriggerResponse(True, '接管已锁存；查看 /attachment/rc_takeover/status')
            if (not rospy.get_param('/flight_mode_manager_node/enabled', False)
                    or not is_fresh(getattr(self, 'manager_phase_at', None), .3, rospy.Time.now())):
                return TriggerResponse(False, '模式管理器未启用或反馈过期')
            if not self._takeover_inputs_ready(rospy.Time.now()):
                return TriggerResponse(False, '需要已解锁的 OFFBOARD、新鲜姿态/RC 和有效急停反馈')
            try:
                rospy.wait_for_service('/mavros/param/get', timeout=1.)
                get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
                def read(name, integer=False):
                    response = get(param_id=name)
                    if not response.success:
                        raise ValueError('无法读取 ' + name)
                    return response.value.integer if integer else response.value.real
                calibration = {}
                for axis in ('roll', 'pitch', 'yaw', 'throttle'):
                    channel = int(read('RC_MAP_' + axis.upper(), True))
                    if not 1 <= channel <= 18:
                        raise ValueError('无效遥控通道 ' + axis)
                    low, center, high, reverse = [read('RC%d_%s' % (channel, suffix))
                                                  for suffix in ('MIN', 'TRIM', 'MAX', 'REV')]
                    if axis == 'throttle':
                        center = (low + high) / 2.  # Throttle TRIM may equal MIN in PX4.
                    if not (all(math.isfinite(v) for v in (low, center, high, reverse))
                            and low < center < high and reverse in (-1, 1)):
                        raise ValueError('无效遥控校准 ' + axis)
                    calibration[axis] = channel-1, low, center, high, reverse < 0
                if len({v[0] for v in calibration.values()}) != 4:
                    raise ValueError('遥控通道重复')
                self.rc_takeover_calibration = calibration
                if not self._takeover_inputs_ready(rospy.Time.now()) or self._takeover_axes() is None:
                    raise ValueError('读取参数期间输入失效')
                # Start only on the timer, after the manager's acknowledgement.
                self.rc_takeover_controller = None
                self.rc_takeover_failed = False
                self.rc_takeover_stable_since = None
                self.rc_takeover_pending = True
                return TriggerResponse(True, '已接受平滑接管；等待模式管理器确认')
            except (ValueError, rospy.ROSException, rospy.ServiceException) as error:
                return TriggerResponse(False, str(error))

    def _tick_rc_takeover(self, now):
        if not getattr(self, 'rc_takeover_pending', False):
            return False
        flight = self.flight_state
        if (flight is not None and is_fresh(self.flight_state_at, self.state_timeout_s, now)
                and (not flight.armed or flight.mode.upper() != 'OFFBOARD')):
            self.rc_takeover_pending = False
            self.rc_takeover_active_pub.publish(Bool(data=False))
            self.rc_takeover_ready_pub.publish(Bool(data=False))
            self.rc_takeover_status_pub.publish(String(data='已退出 OFFBOARD，接管输出停止'))
            return True
        self.rc_takeover_active_pub.publish(Bool(data=True))
        manager_ready = (getattr(self, 'manager_phase', '') == 'RC_TAKEOVER'
                         and is_fresh(getattr(self, 'manager_phase_at', None), .3, now))
        if not manager_ready and self.rc_takeover_controller is None:
            self.rc_takeover_ready_pub.publish(Bool(data=False))
            self.rc_takeover_status_pub.publish(String(data='等待模式管理器交出控制权'))
            return False  # Existing stream continues during the handshake.
        if not manager_ready or not self._takeover_inputs_ready(now):
            self.rc_takeover_failed = True
        axes = self._takeover_axes()
        if axes is None:
            self.rc_takeover_failed = True
        if self.rc_takeover_failed:
            self.rc_takeover_ready_pub.publish(Bool(data=False))
            self.mode_publisher.publish(UInt8(data=0))
            self.rc_takeover_status_pub.publish(String(data='接管中止：输入/急停/管理器失效，禁止旧任务恢复；请用遥控器切换模式'))
            return True
        stamp = time.monotonic()
        if self.rc_takeover_controller is None:
            seed = getattr(self, 'last_published_control', None)
            if seed is None or not is_fresh(seed[2], .3, now):
                self.rc_takeover_failed = True
                self.rc_takeover_ready_pub.publish(Bool(data=False))
                self.mode_publisher.publish(UInt8(data=0))
                self.rc_takeover_status_pub.publish(String(data='接管中止：缺少新鲜推力起点；请用遥控器切换模式'))
                return True
            q = tuple(getattr(self.odom.pose.pose.orientation, k) for k in 'xyzw')
            self.rc_takeover_controller = RCTakeover(q, seed[1])
            self.rc_takeover_last_tick = stamp
        controller = self.rc_takeover_controller
        vz = vertical_speed_enu_mps(self.odom, self.odom_twist_in_child_frame)
        desired_vz = axes['throttle'] * .5
        thrust = min(self.max_thrust, max(self.min_thrust,
                     self.hover_thrust + self.vertical_speed_kp * (desired_vz - vz)))
        desired_thrust = thrust
        q, thrust, desired = controller.step(math.radians(10.)*axes['roll'],
            math.radians(10.)*axes['pitch'], math.radians(20.)*axes['yaw'],
            thrust, stamp-self.rc_takeover_last_tick)
        self.rc_takeover_last_tick = stamp
        target = self._attitude_target(now, Quaternion(*q), thrust, apply_handover=False)
        # Preserve the measured thrust seed even if outside normal behavior bounds.
        target.thrust = thrust
        self.attitude_publisher.publish(target)
        self._remember_publication(target, 0, now)
        measured = tuple(getattr(self.odom.pose.pose.orientation, k) for k in 'xyzw')
        converged = (controller.elapsed >= 2. and quaternion_angle(q, desired) < math.radians(2.)
                     and quaternion_angle(measured, desired) < math.radians(5.)
                     and abs(vz-desired_vz) < .2 and abs(thrust-desired_thrust) < .02)
        if not converged:
            self.rc_takeover_stable_since = None
        elif self.rc_takeover_stable_since is None:
            self.rc_takeover_stable_since = stamp
        ready = self.rc_takeover_stable_since is not None and stamp-self.rc_takeover_stable_since >= .5
        self.rc_takeover_ready_pub.publish(Bool(data=ready))
        self.owner_publisher.publish(UInt8(data=0))
        self.mode_publisher.publish(UInt8(data=2))
        self.rc_takeover_status_pub.publish(String(data='姿态已跟随遥控器，等待原遥控模式确认' if ready
                                                   else '从实测姿态平滑跟随当前遥控器'))
        return True


class RCTakeoverManager:
    def _init_rc_takeover_manager(self):
        self.rc_takeover_requested = False
        self.rc_takeover_active_at = self.rc_takeover_ready_at = None
        self.rc_takeover_ready = False
        rospy.Subscriber('/attachment/rc_takeover/active', Bool, self._on_rc_takeover_active, queue_size=1)
        rospy.Subscriber('/attachment/rc_takeover/ready', Bool, self._on_rc_takeover_ready, queue_size=1)

    def _on_rc_takeover_active(self, message):
        self.rc_takeover_active_at = rospy.Time.now()
        if message.data:
            self.rc_takeover_requested = True

    def _on_rc_takeover_ready(self, message):
        self.rc_takeover_ready, self.rc_takeover_ready_at = message.data, rospy.Time.now()

    def _rc_takeover_handover_ready(self, now):
        estimator = self.estimator
        return bool(self.enabled and kill_clear(self, now)
            and self.flight is not None and self.flight.connected and self.flight.armed
            and self.flight.mode.upper() == 'OFFBOARD'
            and is_fresh(self.flight_at, self.state_timeout_s, now)
            and self.rc_takeover_ready and is_fresh(self.rc_takeover_ready_at, .15, now)
            and is_fresh(self.rc_takeover_active_at, .15, now)
            and self.output_mode == 2 and is_fresh(self.output_at, self.output_timeout_s, now)
            and estimator is not None and is_fresh(self.estimator_at, self.estimator_timeout_s, now)
            and estimator.attitude_status_flag and estimator.velocity_vert_status_flag
            and (self._pilot_mode() != 'POSCTL' or
                 (estimator.velocity_horiz_status_flag and estimator.pos_horiz_rel_status_flag))
            and self._wall_fusion_handover_ready(now))

    def _tick_rc_takeover_manager(self, now):
        if not getattr(self, 'rc_takeover_requested', False):
            return False
        # A mode RPC already in flight must finish before acknowledging ownership.
        if self.phase != 'RC_TAKEOVER':
            if self.request_in_flight:
                self.prestream_pub.publish(Bool(data=True))
                return True
            self.takeover_attempt += 1
            self.offboard_request = None
            self.rc_takeover_return_mode = getattr(self, 'last_pilot_mode', self._pilot_mode())
            self._set_phase('RC_TAKEOVER', reason='一键遥控器接管：等待平滑过渡完成')
            self.rc_takeover_ready = False
        self.cancel_pub.publish(String(data='cancel'))
        self.prestream_pub.publish(Bool(data=False))
        flight = self.flight
        if (flight is not None and is_fresh(self.flight_at, self.state_timeout_s, now)
                and (not flight.armed or flight.mode.upper() != 'OFFBOARD')):
            # Keep cancellation latched until the terminal intent is actually neutral.
            self.phase_reason = '已交回飞手：' + flight.mode + '；等待旧动作请求清除'
            if self._requested_mode(now) is None:
                self.rc_takeover_requested = False
                self._reset(lock=True, reason=self.phase_reason)
        elif self._rc_takeover_handover_ready(now):
            self.phase_reason = '平滑过渡完成，等待 PX4 确认 ' + self._pilot_mode()
            self._request_mode(self._pilot_mode())
        else:
            self.phase_reason = '遥控器平滑接管中；等待姿态、速度、估计器及参数恢复条件'
        self.phase_pub.publish(String(data=self.phase))
        self.phase_reason_pub.publish(String(data=self.phase_reason))
        return True
