#!/usr/bin/env python3
"""Ground setup + continuous link guard. No arm or mode-changing services."""
import threading
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import VehicleInfoGet
from std_msgs.msg import Bool, String
from px4_wall_ceiling_control.adapters.ceiling_failsafe import CeilingFailsafeTransport, PARAMETERS
from px4_wall_ceiling_control.freshness import is_fresh_receipt, is_fresh_source


class CeilingFailsafeGuard:
    def __init__(self):
        self.lock = threading.RLock()
        self.state = self.received = None
        self.epoch = 0
        self.verified = False
        self.reason = '等待飞控连接，失联降落参数尚未核验'
        self.transport = CeilingFailsafeTransport('/mavros')
        self.ready = rospy.Publisher('/attachment/ceiling_failsafe_ready', Bool, queue_size=1)
        self.detail = rospy.Publisher('/attachment/ceiling_failsafe_reason', String, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.on_state, queue_size=1)
        rospy.Timer(rospy.Duration(.2), self.publish)
        threading.Thread(target=self.audit_loop, daemon=True).start()

    def connected(self):
        now = rospy.Time.now()
        return bool(self.state is not None and self.state.connected
            and is_fresh_receipt(self.received, 1., now)
            and is_fresh_source(self.state.header.stamp, 1., now, future_tolerance_s=0.))

    def on_state(self, message):
        with self.lock:
            if not self.connected() or not message.connected:
                self.epoch += 1
                self.verified = False
            self.state, self.received = message, rospy.Time.now()

    def publish(self, _event):
        with self.lock:
            if not self.connected():
                self.verified = False
                self.epoch += 1
                self.reason = '飞控断连，失联降落参数未核验；禁止新的顶吸接管'
            self.ready.publish(Bool(data=self.verified and self.connected()))
            self.detail.publish(String(data=self.reason))

    def modern_px4(self):
        rospy.wait_for_service('/mavros/vehicle_info_get', timeout=1.)
        response = rospy.ServiceProxy('/mavros/vehicle_info_get', VehicleInfoGet)(
            self.transport.target_system, self.transport.target_component, False)
        for vehicle in response.vehicles if response.success else ():
            if (vehicle.sysid == self.transport.target_system and vehicle.compid == self.transport.target_component
                    and vehicle.autopilot == 12 and vehicle.available_info & 2):
                major, minor = (vehicle.flight_sw_version >> 24) & 255, (vehicle.flight_sw_version >> 16) & 255
                return (major, minor) >= (1, 14)
        return False  # Unknown firmware may still require COM_OBL_ACT.

    def audit_loop(self):
        rate = rospy.Rate(.2)
        while not rospy.is_shutdown():
            try:
                with self.lock:
                    eligible = self.connected() and not self.state.armed and not self.verified
                    epoch = self.epoch
                if eligible:
                    # Every write requires fresh connected/disarmed state again.
                    for name, (_, required) in PARAMETERS.items():
                        try:
                            actual = self.transport._exchange(name)
                        except RuntimeError:
                            if name == 'COM_OBL_ACT' and self.modern_px4():
                                continue  # Removed in modern PX4; required in older firmware when present.
                            raise
                        if abs(actual-required) > 1e-6:
                            with self.lock:
                                if not self.connected() or self.state.armed or self.epoch != epoch:
                                    raise RuntimeError('飞控状态改变，取消地面参数设置')
                                self.transport._publish(name, required)
                            # PARAM_SET ACK alone is insufficient; perform a separate fresh read.
                            actual = self.transport._exchange(name)
                            if abs(actual-required) > 1e-6:
                                raise RuntimeError(name + ' readback mismatch')
                    with self.lock:
                        if self.connected() and not self.state.armed and self.epoch == epoch:
                            self.verified = True
                            self.reason = '已核验：Offboard失联自动降落；COM_OBL_RC_ACT=4，COM_OF_LOSS_T=0.2s'
            except Exception as error:
                with self.lock:
                    self.verified = False
                    self.reason = '失联降落参数核验失败：' + str(error)
                rospy.logwarn_throttle(5., self.reason)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('ceiling_failsafe_guard_node')
    CeilingFailsafeGuard()
    rospy.spin()
