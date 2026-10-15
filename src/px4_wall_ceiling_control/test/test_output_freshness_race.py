"""No ROS master or real publishers. Force a telemetry callback during one timer tick."""
import unittest
from unittest import mock
import rospy
from mavros_msgs.msg import State, AttitudeTarget
from nav_msgs.msg import Odometry
from test_flight_handover import OutputMux

class OutputFreshnessRaceTests(unittest.TestCase):
    def fixture(self):
        n=OutputMux.__new__(OutputMux);clock=[10.]
        n.owner=0;n._update_owner=mock.Mock();n.owner_publisher=mock.Mock();n.mode_publisher=mock.Mock()
        n.output_enabled=True;n.kill_guard_enabled=False
        n.prestream_requested=True;n.prestream_at=rospy.Time.from_sec(10);n.prestream_timeout_s=.3
        n.flight_state=State(connected=True,armed=True,mode='POSCTL');n.flight_state_at=rospy.Time.from_sec(10);n.state_timeout_s=1.5
        n.odom=Odometry();n.odom.pose.pose.orientation.w=1;n.odom_at=rospy.Time.from_sec(10);n.odom_timeout_s=.5
        n._rc_ready=mock.Mock(return_value=True);target=AttitudeTarget(thrust=.48)
        n._prestream_attitude_target=mock.Mock(return_value=target)
        n._limit_final_approach_target=mock.Mock(return_value=target);n.attitude_publisher=mock.Mock()
        once=[False]
        def callback(_):
            if not once[0]:
                once[0]=True;clock[0]=10.0005
                n._on_odom(n.odom)
        n.owner_publisher.publish.side_effect=callback
        return n,clock
    def test_new_callback_during_tick_is_not_false_expiration(self):
        n,clock=self.fixture()
        with mock.patch.object(rospy.Time,'now',side_effect=lambda:rospy.Time.from_sec(clock[0])):
            n._on_timer(None)
        self.assertEqual(n.mode_publisher.publish.call_args[0][0].data,2)
        n.attitude_publisher.publish.assert_called_once()
    def test_real_future_timestamp_and_actual_staleness_still_rejected(self):
        from px4_wall_ceiling_control.freshness import is_fresh_receipt
        with mock.patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(10.001)):
            self.assertFalse(is_fresh_receipt(rospy.Time.from_sec(11),.3,rospy.Time.from_sec(10)))
            self.assertFalse(is_fresh_receipt(rospy.Time.from_sec(9),.3,rospy.Time.from_sec(10)))
            self.assertTrue(is_fresh_receipt(rospy.Time.from_sec(10.0005),.3,rospy.Time.from_sec(10)))
    def test_kill_guard_receipt_race_keeps_real_active_switch_effective(self):
        from px4_wall_ceiling_control.adapters.kill_switch import kill_clear
        n,clock=self.fixture();n.kill_guard_enabled=True
        n.kill_status_at=rospy.Time.from_sec(10.0005);n.kill_status={'valid':True,'active':False}
        with mock.patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(10.001)):
            self.assertTrue(kill_clear(n,rospy.Time.from_sec(10)))
            n.kill_status['active']=True
            self.assertFalse(kill_clear(n,rospy.Time.from_sec(10)))

if __name__=='__main__':unittest.main()
