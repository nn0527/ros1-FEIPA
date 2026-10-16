"""Tilt-only early pilot handback; no flight services are sent by these tests."""
import math
import copy
import threading
import unittest
from unittest import mock
import rospy
from mavros_msgs.msg import State
from px4_wall_ceiling_control.msg import WallPerchStatus as W, OperatorCommand
from test_flight_handover import Manager
import test_wall_timeout_handover as fixture


class WallLevelHandoverTest(unittest.TestCase):
    def setUp(self):
        self.fx = fixture.WallTimeoutHandoverTests()
        self.fx.setUp()
        self.n = n = self.fx.n
        n.wall_level_handover_enabled = True
        n.wall_level_handover_tilt_deg = 5.
        n.wall_level_handover_hold_s = .30
        n.wall_level_handover_retry_s = .50
        n.wall_level_returning = False
        n.wall_level_window = None
        n.wall_level_lock = threading.RLock()
        n.wall.state = W.STATE_DETACH_ROTATE
        n.wall.reason = 'detach requested'
        n.takeover_attempt = 7
        n.acquire_timeout_s = 6.
        n.retry_s = 2.
        n.last_request_at = None
        self.refresh(0)

    def refresh(self, t, source=None, update_odom=True):
        self.fx.refresh(t, odom=update_odom)
        self.n.wall.header.stamp = self.fx.f.now
        if update_odom:
            self.n.odom.header.stamp = rospy.Time.from_sec(10+t if source is None else source)
        self.n.handover_odom_snapshot = (self.n.odom, self.n.odom_at)

    def tick(self, t, **kwargs):
        self.refresh(t, **kwargs)
        self.fx.f.tick()

    def confirm(self):
        for t in (0., .1, .2, .30):self.tick(t)

    def test_detach_recover_abort_all_hand_back_after_tilt_hold_only(self):
        for phase in (W.STATE_DETACH_ROTATE, W.STATE_RECOVER, W.STATE_ABORT):
            with self.subTest(phase=phase):
                self.setUp()
                self.n.wall.state = phase
                self.n.wall.reason = 'detach timeout' if phase == W.STATE_ABORT else 'recovering'
                self.n.odom.twist.twist.angular.z = 2.0
                self.n.odom.twist.twist.linear.x = 1.2
                self.n.odom.twist.twist.linear.z = -.8
                self.n.wall.horizontal_velocity_ready = False
                self.n.wall.horizontal_velocity_stable = False
                self.n.wall.departure_clearance_ready = False
                self.n.wall.detach_height_error_m = 2.
                for t in (0., .1, .2):self.tick(t)
                self.n._request_mode.assert_not_called()
                self.tick(.30)
                self.n._request_mode.assert_called_once_with('POSCTL')
                self.assertEqual(self.n.phase, Manager.RETURNING)
                self.assertFalse(self.n.completed)
                self.assertTrue(self.n.prestream_pub.publish.call_args.args[0].data)

    def test_roll_pitch_combine_into_total_tilt_and_reset_confirmation(self):
        self.tick(0);self.tick(.1)
        # roll=4 deg, pitch=4 deg => total tilt >5 deg.
        a = math.radians(4.)/2.
        q = self.n.odom.pose.pose.orientation
        q.x=math.sin(a)*math.cos(a);q.y=math.cos(a)*math.sin(a)
        q.z=-math.sin(a)*math.sin(a);q.w=math.cos(a)**2
        self.tick(.2)
        self.assertIsNone(self.n.wall_level_window)
        q.x=q.y=q.z=0.;q.w=1.
        self.tick(.3);self.tick(.4);self.tick(.5)
        self.n._request_mode.assert_not_called()
        self.tick(.6)
        self.n._request_mode.assert_called_once_with('POSCTL')

    def test_duplicate_source_receipts_do_not_extend_window(self):
        self.tick(0)
        for t in (.1,.2,.3):self.tick(t,source=10.)
        self.n._request_mode.assert_not_called()

    def test_out_of_order_source_resets_window(self):
        self.tick(0);self.tick(.1)
        self.tick(.15,source=10.05)
        self.assertIsNone(self.n.wall_level_window)
        self.tick(.2);self.tick(.3);self.tick(.4)
        self.n._request_mode.assert_not_called()
        self.tick(.5)
        self.n._request_mode.assert_called_once_with('POSCTL')

    def test_long_feedback_gap_requires_new_window(self):
        self.tick(0);self.tick(.1);self.tick(.4);self.tick(.5);self.tick(.6)
        self.n._request_mode.assert_not_called()
        self.tick(.7)
        self.n._request_mode.assert_called_once_with('POSCTL')

    def test_brief_tilt_excursion_between_timer_ticks_restarts_window(self):
        self.tick(0);self.tick(.1)
        self.refresh(.15)
        tilted = copy.deepcopy(self.n.odom)
        tilted.pose.pose.orientation.y = math.sin(math.radians(6.)/2.)
        tilted.pose.pose.orientation.w = math.cos(math.radians(6.)/2.)
        with mock.patch.object(rospy.Time,'now',return_value=self.fx.f.now):
            self.n._on_odom(tilted)
        self.assertIsNone(self.n.wall_level_window)
        # A level sample arrives before the next timer; the excursion still counts.
        self.refresh(.18)
        level = copy.deepcopy(self.n.odom)
        level.pose.pose.orientation.y=0.;level.pose.pose.orientation.w=1.
        with mock.patch.object(rospy.Time,'now',return_value=self.fx.f.now):
            self.n._on_odom(level)
        for t in (.2,.3,.4):self.tick(t)
        self.n._request_mode.assert_not_called()
        self.tick(.48)
        self.n._request_mode.assert_called_once_with('POSCTL')

    def test_callback_window_does_not_survive_new_action(self):
        self.tick(0);self.tick(.1);self.tick(.2)
        self.n.takeover_attempt += 1
        self.tick(.3);self.tick(.4);self.tick(.5)
        self.n._request_mode.assert_not_called()
        self.tick(.6)
        self.n._request_mode.assert_called_once_with('POSCTL')

    def test_estimator_rc_kill_output_and_source_validity_remain_required(self):
        for fault in ('rc','estimator','kill','output','source','wall','nonfinite'):
            with self.subTest(fault=fault):
                self.setUp();self.tick(0);self.tick(.1)
                if fault=='rc':self.n.rc_channels=[]
                elif fault=='estimator':self.n.estimator.pos_horiz_rel_status_flag=False
                elif fault=='kill':
                    self.n.kill_guard_enabled=True;self.n.kill_status={'valid':True,'active':True}
                elif fault=='output':self.n.output_mode=0
                elif fault=='nonfinite':self.n.odom.pose.pose.orientation.x=math.nan
                for t in (.2,.3,.4):
                    self.refresh(t)
                    self.n.kill_status_at=self.fx.f.now
                    if fault=='source':self.n.odom.header.stamp=rospy.Time.from_sec(5.)
                    if fault=='wall':self.n.wall.header.stamp=rospy.Time.from_sec(5.)
                    self.fx.f.tick()
                self.n._request_mode.assert_not_called()

    def test_other_phases_never_use_tilt_only_handback(self):
        for phase in (W.STATE_SLOW_APPROACH,W.STATE_FLIP_TO_WALL,W.STATE_WALL_HOLD):
            with self.subTest(phase=phase):
                self.setUp();self.n.wall.state=phase
                self.confirm()
                self.n._request_mode.assert_not_called()
                self.assertEqual(self.n.phase,Manager.ACTIVE)

    def test_exit_or_abort_transition_does_not_restart_level_window(self):
        self.tick(0);self.tick(.1)
        self.n.wall.state=W.STATE_ABORT
        self.tick(.2);self.tick(.3)
        self.n._request_mode.assert_called_once_with('POSCTL')
        self.assertTrue(self.n.fault_seen)
        self.assertFalse(self.n.completed)

    def test_service_ack_is_not_mode_confirmation_and_retry_remains_possible(self):
        self.confirm()
        self.n.set_mode=mock.Mock(return_value=type('Reply',(),{'mode_sent':True})())
        with mock.patch.object(rospy,'wait_for_service'),mock.patch.object(rospy.Time,'now',return_value=self.fx.f.now):
            self.n._send_mode_request('POSCTL',7)
        self.assertEqual(self.n.phase,Manager.RETURNING)
        self.assertTrue(self.n.wall_level_returning)
        self.tick(.4)
        self.assertEqual(self.n._request_mode.call_count,2)

    def test_worker_rechecks_live_tilt_and_pilot_mode_and_attempt(self):
        for revoked in ('tilt','mode','attempt','rc','stale'):
            with self.subTest(revoked=revoked):
                self.setUp();self.confirm();self.n.set_mode=mock.Mock()
                if revoked=='tilt':self.n.odom.pose.pose.orientation.y=.1
                elif revoked=='mode':self.n.flight.mode='AUTO.LAND'
                elif revoked=='attempt':self.n.takeover_attempt=8
                elif revoked=='rc':self.n.rc_channels=[]
                elif revoked=='stale':self.n.odom.header.stamp=rospy.Time.from_sec(5.)
                with mock.patch.object(rospy,'wait_for_service'),mock.patch.object(rospy.Time,'now',return_value=self.fx.f.now):
                    self.n._send_mode_request('POSCTL',7)
                self.n.set_mode.assert_not_called()

    def test_posctl_confirmation_releases_stream_and_does_not_claim_complete(self):
        self.confirm();self.n.flight.mode='POSCTL';self.tick(.4)
        self.assertEqual(self.n.phase,Manager.LOCKED)
        self.assertFalse(self.n.prestream_pub.publish.call_args.args[0].data)
        self.assertIn('已确认交回POSCTL',self.n.phase_reason)
        self.assertNotIn('正常脱离及恢复完成',self.n.phase_reason)
        self.assertFalse(self.n.completed)
        self.assertIsNone(self.n.wall_level_window)

    def test_already_returning_can_use_new_path_without_exit_or_speed(self):
        self.n.phase=Manager.RETURNING
        self.confirm()
        self.n._request_mode.assert_called_once_with('POSCTL')
        self.assertTrue(self.n.wall_level_returning)

    def test_retry_interval_is_half_second_only_for_early_pilot_return(self):
        self.confirm();self.n.request_in_flight=False;self.n.last_request_at=100.
        with mock.patch('threading.Thread') as thread,mock.patch('time.monotonic',return_value=100.49):
            Manager._request_mode(self.n,'POSCTL');thread.assert_not_called()
        with mock.patch('threading.Thread') as thread,mock.patch('time.monotonic',return_value=100.50):
            Manager._request_mode(self.n,'POSCTL');thread.assert_called_once()


if __name__=='__main__':unittest.main()
