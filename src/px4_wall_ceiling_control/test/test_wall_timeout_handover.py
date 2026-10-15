"""Auto-approach timeout returns via PX4 estimates, independent of raw flow gates."""
import unittest
from unittest import mock
import rospy
from mavros_msgs.msg import State,EstimatorStatus
from nav_msgs.msg import Odometry
from px4_wall_ceiling_control.msg import WallPerchStatus as W,OperatorCommand
import test_manual_wall_approach as fixtures
from test_flight_handover import Manager

class WallTimeoutHandoverTests(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.ManualApproachTests();self.f.setUp();n=self.n=self.f.node
        n.wall_auto_approach=True;n.wall_manual_approach=False;n.wall_pilot_mode='POSCTL'
        n.phase=Manager.ACTIVE;n.selected_mode=OperatorCommand.MODE_WALL
        n.flight.mode='OFFBOARD';n.owner=2;n.owner_seen=True;n.request_in_flight=False
        n.completed=n.detach_seen=n.recovery_seen=n.fault_seen=False
        n.estimator=EstimatorStatus(attitude_status_flag=True,velocity_vert_status_flag=True,
            velocity_horiz_status_flag=True,pos_horiz_rel_status_flag=True)
        n.estimator_timeout_s=n.odom_timeout_s=.5;n.rc_timeout_s=.2
        n.rc_channels=[1500]*8;n.odom=Odometry();n.odom.pose.pose.orientation.w=1
        n.odom_twist_in_child_frame=False;n.handover_max_vertical_speed_mps=.2
        n.wall=W(state=W.STATE_ABORT,reason='自动接近超时：总时限20秒，含测距暂停',
            horizontal_velocity_enabled=True,horizontal_velocity_ready=False,horizontal_velocity_stable=False)
        n.wall_horizontal_velocity_enabled=True
        n._handover_ready=Manager._handover_ready.__get__(n)
        n.wall_approach_timeout_returning=False
        n.wall_timeout_stable_since=n.wall_timeout_last_odom=None;n.wall_timeout_safe_samples=0
        self.clock=0
        self.refresh(0)
    def refresh(self,t,odom=True):
        self.clock=t;self.f.clock=100+t;self.f.now=rospy.Time.from_sec(10+t)
        for k in ('flight_at','estimator_at','rc_at','owner_at','output_at','wall_at'):
            setattr(self.n,k,self.f.now)
        if odom:self.n.odom_at=self.f.now
    def tick(self,t,odom=True):self.refresh(t,odom);self.f.tick()
    def start(self):
        self.tick(0);self.assertEqual(self.n.phase,Manager.RETURNING)
        self.n._request_mode.assert_not_called()
    def test_timeout_bypasses_raw_flow_and_height_settle_but_not_short_safe_hold(self):
        self.start();self.n.cancel_pub.publish.assert_called_once()
        self.tick(.15);self.n._request_mode.assert_not_called()
        self.tick(.31);self.n._request_mode.assert_called_once_with('POSCTL')
        self.assertTrue(self.n.prestream_pub.publish.call_args[0][0].data)
    def test_duplicate_odometry_cannot_satisfy_safe_hold(self):
        self.start();self.tick(.15,odom=False);self.tick(.35,odom=False)
        self.n._request_mode.assert_not_called()
    def test_estimator_rc_and_tilt_remain_required(self):
        for fault in ('rc','estimator','tilt','rate','vertical','output'):
            self.setUp();self.start()
            if fault=='rc':self.n.rc_channels=[]
            elif fault=='estimator':self.n.estimator.pos_horiz_rel_status_flag=False
            elif fault=='tilt':self.n.odom.pose.pose.orientation.y=.1
            elif fault=='rate':self.n.odom.twist.twist.angular.x=.3
            elif fault=='vertical':self.n.odom.twist.twist.linear.z=.3
            else:self.n.output_mode=0
            self.tick(.15);self.tick(.4)
            self.n._request_mode.assert_not_called()
    def test_validity_failure_restarts_short_safe_hold(self):
        self.start();self.tick(.15);self.n.rc_channels=[];self.tick(.2)
        self.n.rc_channels=[1500]*8;self.tick(.3);self.tick(.5)
        self.n._request_mode.assert_not_called()
        self.tick(.61);self.n._request_mode.assert_called_once_with('POSCTL')
    def test_posctl_feedback_enters_intent_cleanup_before_new_actions(self):
        self.start();self.tick(.15);self.tick(.31)
        self.n.flight.mode='POSCTL';self.tick(.4)
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
        self.assertTrue(self.n.offboard_exit_cleanup)
        self.assertFalse(self.n.prestream_pub.publish.call_args[0][0].data)
        self.assertEqual(self.n.selected_mode,OperatorCommand.MODE_WALL)
    def test_service_worker_rechecks_posctl_requirements(self):
        self.start();self.tick(.15);self.tick(.31)
        self.n.set_mode=mock.Mock();self.n.estimator.const_pos_mode_status_flag=True
        with mock.patch.object(rospy.Time,'now',return_value=self.f.now),mock.patch.object(rospy,'wait_for_service'):
            self.n._send_mode_request('POSCTL')
        self.n.set_mode.assert_not_called()
    def test_no_rpc_success_is_treated_as_fcu_mode_confirmation(self):
        self.start();self.tick(.15);self.tick(.31)
        self.n.set_mode=mock.Mock(return_value=type('R',(),{'mode_sent':True})())
        with mock.patch.object(rospy.Time,'now',return_value=self.f.now),mock.patch.object(rospy,'wait_for_service'):
            self.n._send_mode_request('POSCTL')
        self.assertEqual(self.n.phase,Manager.RETURNING)
        self.assertEqual(self.n.flight.mode,'OFFBOARD')
    def test_kill_immediately_stops_output_without_losing_cleanup(self):
        self.start();self.n.kill_guard_enabled=True;self.n.kill_status={'valid':True,'active':True}
        self.n.kill_status_at=self.f.now;self.tick(.1)
        self.assertEqual(self.n.phase,Manager.CANCELING_ENTRY)
        self.n._request_mode.assert_not_called()
        self.assertFalse(self.n.prestream_pub.publish.call_args[0][0].data)
    def test_normal_recovery_still_requires_horizontal_stability(self):
        self.n.phase=Manager.RETURNING;self.n.wall.state=W.STATE_RECOVER
        self.assertFalse(self.n._handover_ready(self.f.now))
    def test_other_abort_does_not_trigger_timeout_bypass(self):
        self.n.wall.reason='flip sensor or flight failure'
        self.tick(0);self.assertEqual(self.n.phase,Manager.ACTIVE)
        self.assertFalse(self.n.wall_approach_timeout_returning)
    def test_stale_timeout_status_does_not_enable_bypass(self):
        self.refresh(0);self.n.wall_at=rospy.Time.from_sec(8);self.f.tick()
        self.assertEqual(self.n.phase,Manager.ACTIVE)
    def test_stale_fcu_and_third_pilot_mode_do_not_send_mode_change(self):
        self.start();self.refresh(.4);self.n.flight_at=rospy.Time.from_sec(8);self.f.tick()
        self.n._request_mode.assert_not_called()
        self.n.flight.mode='AUTO.LAND';self.tick(.5)
        self.n._request_mode.assert_not_called()

    def test_recovery_exit_does_not_lose_latched_timeout_return(self):
        self.start();self.tick(.15)
        self.n.wall.state=W.STATE_EXIT;self.n.wall.reason='abort recovery complete'
        self.n.wall.recovery_completed=True
        self.tick(.31);self.n._request_mode.assert_called_once_with('POSCTL')
    def test_odometry_gap_requires_new_short_safe_history(self):
        self.start();self.tick(.15);self.tick(.8)
        self.n._request_mode.assert_not_called()
        self.tick(.95);self.tick(1.11)
        self.n._request_mode.assert_called_once_with('POSCTL')

if __name__=='__main__':unittest.main()
