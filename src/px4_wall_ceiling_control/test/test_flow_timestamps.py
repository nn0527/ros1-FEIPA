"""Clock mapping skew must not erase valid feedback; replay/stale frames must fail."""
import unittest
import math
from px4_wall_ceiling_control.domain.flow_timestamps import FlowTimestampGuard
import test_wall_horizontal_velocity as flow_fixtures
from mavros_msgs.msg import OpticalFlowRad
from px4_wall_ceiling_control.msg import WallPerchStatus as W
import rospy

class FlowTimestampMathTests(unittest.TestCase):
    def test_small_future_mapping_skew_is_valid(self):
        g=FlowTimestampGuard()
        for i in range(20):
            t=10+i*.23
            self.assertEqual(g.observe(t+.05,t),'')
            self.assertEqual(g.reason(t+.05,t+.001),'')
    def test_large_future_and_real_stale_are_rejected(self):
        self.assertIn('超前',FlowTimestampGuard().observe(10.101,10))
        self.assertIn('过期',FlowTimestampGuard().observe(9.39,10))
        g=FlowTimestampGuard();g.observe(10,10)
        self.assertIn('过期',g.reason(10,10.61))
    def test_source_replay_and_backward_time_are_not_fresh_frames(self):
        g=FlowTimestampGuard();g.observe(10,10)
        self.assertIn('重复',g.observe(10,10.1))
        self.assertIn('倒退',g.observe(9.99,10.2))
        self.assertEqual(g.observe(10.3,10.3),'')
    def test_repeated_older_source_cannot_rebase_replay_protection(self):
        g=FlowTimestampGuard();g.observe(10,10)
        self.assertIn('倒退',g.observe(9.9,10.1))
        self.assertIn('倒退',g.observe(9.95,10.2))
        self.assertIn('重复',g.observe(10,10.3))
        self.assertEqual(g.observe(10.4,10.4),'')

    def test_receipt_clock_reset_and_missing_stamp_fail_closed(self):
        g=FlowTimestampGuard();g.observe(10,10)
        self.assertIn('倒退',g.observe(10.01,9.99))
        for x in (0.,-1.,math.nan):self.assertTrue(FlowTimestampGuard().observe(x,10))
    def test_large_mapping_step_requires_reconfirmation(self):
        g=FlowTimestampGuard();self.assertEqual(g.observe(9.5,10),'')
        self.assertIn('跳变',g.observe(10.3,10.3))
        self.assertEqual(g.observe(10.53,10.53),'')
    def test_limits_cannot_silently_disable_timestamp_protection(self):
        for future in (-1,.101,math.nan):
            with self.assertRaises(ValueError):FlowTimestampGuard(future_tolerance=future)
    def test_frame_verdict_is_immutable_for_control_snapshot(self):
        g=FlowTimestampGuard();error=g.observe(10.05,10)
        g.observe(10.05,10.1)  # A later duplicate must not invalidate the old snapshot.
        self.assertEqual(g.reason(10.05,10.1,error),'')
        self.assertIn('重复',g.reason(10.05,10.1))

class FutureFlowFlightTests(unittest.TestCase):
    def setUp(self):
        self.fixture=flow_fixtures.HorizontalFlightTests();self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups);self.n=self.fixture.n
    def feed(self,t,offset=.05):
        self.fixture.feed(t,flow=False)
        m=OpticalFlowRad(quality=100,integration_time_us=20000,distance=1.)
        m.header.stamp=rospy.Time.from_sec(t+offset)
        self.n._on_horizontal_flow(m)
        self.n._prepare_horizontal_velocity(rospy.Time.now())
    def test_4hz_small_future_frames_preserve_approach_and_recovery_stability(self):
        for i in range(12):self.feed(10+i*.23)
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
        self.assertTrue(self.n.horizontal_velocity_active)
        self.n.state=W.STATE_ABORT
        for i in range(12,21):self.feed(10+i*.23)
        self.assertTrue(self.n.horizontal_velocity_stable)
    def test_large_future_cancels_forward_and_needs_new_confirmation(self):
        for i in range(4):self.feed(10+i*.23)
        self.feed(11,offset=.15)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
        self.assertEqual(self.n.horizontal_controller.target,(0.,0.))
        self.feed(11.3);self.assertFalse(self.n.horizontal_velocity_ready)
        self.feed(11.53);self.feed(11.76)
        self.assertTrue(self.n.horizontal_velocity_ready)
    def test_duplicate_stream_does_not_build_confirmation(self):
        for i in range(4):self.feed(10+i*.23)
        previous=self.n.horizontal_flow.header.stamp
        self.fixture.feed(10.92,flow=False)
        message=OpticalFlowRad(quality=100,integration_time_us=20000,distance=1.)
        message.header.stamp=previous;self.n._on_horizontal_flow(message)
        self.n._prepare_horizontal_velocity(rospy.Time.now())
        self.assertFalse(self.n.horizontal_velocity_ready)

if __name__=='__main__':unittest.main()
