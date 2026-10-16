"""Converged early pilot handback; no flight services are sent by these tests."""
import math
import copy
import threading
import unittest
import json
from unittest import mock
import rospy
from std_msgs.msg import String
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
        n.wall_level_handover_height_error_m = .10
        n.wall_level_handover_vertical_speed_mps = .20
        n.wall_level_handover_horizontal_speed_mps = .30
        n.wall_level_returning = False
        n.wall_level_window = None
        n.wall_level_lock = threading.RLock()
        n.wall.state = W.STATE_RECOVER
        n.wall.reason = 'recovering'
        n.wall.vertical_reference_valid = True
        n.wall.vertical_control_active = True
        n.wall.vertical_reference_m = 1.
        n.odom.pose.pose.position.z = 1.
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

    def test_allowed_phases_hand_back_after_joint_convergence_without_legacy_gates(self):
        for phase in (W.STATE_RECOVER, W.STATE_ABORT, W.STATE_EXIT):
            with self.subTest(phase=phase):
                self.setUp()
                self.n.wall.state = phase
                self.n.wall.reason = 'detach timeout' if phase == W.STATE_ABORT else 'recovering'
                self.n.odom.twist.twist.angular.z = 2.0
                self.n.odom.twist.twist.linear.x = .25
                self.n.odom.twist.twist.linear.z = -.15
                # The old 0.10 m/s / 1 s stability and clearance are not gates.
                self.n.wall.horizontal_velocity_ready = False
                self.n.wall.horizontal_velocity_stable = False
                self.n.wall.departure_clearance_ready = False
                self.n.wall.detach_height_error_m = 2.  # Use live reference minus this odom.z.
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

    def test_disallowed_phases_never_start_confirmation_or_handback(self):
        for phase in (W.STATE_DETACH_ROTATE,W.STATE_SLOW_APPROACH,W.STATE_FLIP_TO_WALL,W.STATE_WALL_HOLD):
            with self.subTest(phase=phase):
                self.setUp();self.n.wall.state=phase
                self.confirm()
                self.n._request_mode.assert_not_called()
                self.assertEqual(self.n.phase,Manager.ACTIVE)
                self.assertIsNone(self.n.wall_level_window)
                self.assertFalse(self.n._handover_ready(self.fx.f.now))

    def test_exit_or_abort_transition_does_not_restart_level_window(self):
        self.tick(0);self.tick(.1)
        self.n.wall.state=W.STATE_ABORT
        self.tick(.2)
        self.n.wall.state=W.STATE_EXIT
        self.tick(.3)
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

    def test_worker_rechecks_live_convergence_stage_and_authority(self):
        for revoked in ('tilt','mode','attempt','rc','stale','phase','height','vertical','horizontal','reference','wall_stale'):
            with self.subTest(revoked=revoked):
                self.setUp();self.confirm();self.n.set_mode=mock.Mock()
                if revoked=='tilt':self.n.odom.pose.pose.orientation.y=.1
                elif revoked=='mode':self.n.flight.mode='AUTO.LAND'
                elif revoked=='attempt':self.n.takeover_attempt=8
                elif revoked=='rc':self.n.rc_channels=[]
                elif revoked=='stale':self.n.odom.header.stamp=rospy.Time.from_sec(5.)
                elif revoked=='phase':self.n.wall.state=W.STATE_DETACH_ROTATE
                elif revoked=='height':self.n.odom.pose.pose.position.z+=.2
                elif revoked=='vertical':self.n.odom.twist.twist.linear.z=-.3
                elif revoked=='horizontal':self.n.odom.twist.twist.linear.x=.4
                elif revoked=='reference':self.n.wall.vertical_reference_valid=False
                elif revoked=='wall_stale':self.n.wall.header.stamp=rospy.Time.from_sec(5.)
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

    def test_already_returning_can_use_joint_window_without_exit_completion(self):
        self.n.phase=Manager.RETURNING
        self.confirm()
        self.n._request_mode.assert_called_once_with('POSCTL')
        self.assertTrue(self.n.wall_level_returning)

    def test_height_and_velocity_boundaries_are_strict(self):
        for sign in (-1., 1.):
            for field in ('height','horizontal','vertical'):
                with self.subTest(sign=sign,field=field):
                    self.setUp()
                    self.n.odom.pose.pose.position.z=0.
                    self.n.wall.vertical_reference_m=0.
                    if field=='height':self.n.wall.vertical_reference_m=sign*.10
                    elif field=='horizontal':self.n.odom.twist.twist.linear.x=sign*.30
                    else:self.n.odom.twist.twist.linear.z=sign*.20
                    self.confirm()
                    self.n._request_mode.assert_not_called()

    def test_requested_limits_allow_return_at_two_tenths_second(self):
        for phase in (W.STATE_RECOVER,W.STATE_ABORT,W.STATE_EXIT):
            with self.subTest(phase=phase):
                self.setUp();n=self.n;n.wall.state=phase
                n.wall_level_handover_hold_s=.20
                n.wall_level_handover_height_error_m=.20
                n.wall_level_handover_vertical_speed_mps=.30
                n.wall_level_handover_horizontal_speed_mps=.40
                n.wall.vertical_reference_m=.199;n.odom.pose.pose.position.z=0.
                n.odom.twist.twist.linear.x=.399
                n.odom.twist.twist.linear.z=-.299
                for t in (0.,.10,.199):self.tick(t)
                n._request_mode.assert_not_called()
                self.tick(.20)
                n._request_mode.assert_called_once_with('POSCTL')

    def test_requested_limits_reject_exact_boundary_and_reset_short_window(self):
        for field in ('height','horizontal','up','down'):
            with self.subTest(field=field):
                self.setUp();n=self.n
                n.wall_level_handover_hold_s=.20
                n.wall_level_handover_height_error_m=.20
                n.wall_level_handover_vertical_speed_mps=.30
                n.wall_level_handover_horizontal_speed_mps=.40
                n.wall.vertical_reference_m=0.;n.odom.pose.pose.position.z=0.
                self.tick(0.);self.tick(.10)
                if field=='height':n.wall.vertical_reference_m=.20
                elif field=='horizontal':n.odom.twist.twist.linear.x=.40
                else:n.odom.twist.twist.linear.z=.30 if field=='up' else -.30
                self.tick(.15);self.assertIsNone(n.wall_level_window)
                n.wall.vertical_reference_m=0.
                n.odom.twist.twist.linear.x=n.odom.twist.twist.linear.z=0.
                self.tick(.20);self.tick(.30);n._request_mode.assert_not_called()
                self.tick(.40);n._request_mode.assert_called_once_with('POSCTL')

    def test_height_speed_and_reference_failures_restart_joint_window(self):
        for fault in ('height_high','height_low','up','down','horizontal','diagonal',
                      'invalid_reference','inactive_vertical','reference_nan'):
            with self.subTest(fault=fault):
                self.setUp();self.tick(0);self.tick(.1)
                if fault=='height_high':self.n.odom.pose.pose.position.z=1.101
                elif fault=='height_low':self.n.odom.pose.pose.position.z=.899
                elif fault=='up':self.n.odom.twist.twist.linear.z=.201
                elif fault=='down':self.n.odom.twist.twist.linear.z=-.201
                elif fault=='horizontal':self.n.odom.twist.twist.linear.x=.301
                elif fault=='diagonal':
                    self.n.odom.twist.twist.linear.x=.22
                    self.n.odom.twist.twist.linear.y=.22
                elif fault=='invalid_reference':self.n.wall.vertical_reference_valid=False
                elif fault=='inactive_vertical':self.n.wall.vertical_control_active=False
                elif fault=='reference_nan':self.n.wall.vertical_reference_m=math.nan
                self.tick(.2)
                self.assertIsNone(self.n.wall_level_window)
                self.n._request_mode.assert_not_called()
                self.n.wall.vertical_reference_valid=True
                self.n.wall.vertical_control_active=True
                self.n.wall.vertical_reference_m=1.
                self.n.odom.pose.pose.position.z=1.
                self.n.odom.twist.twist.linear.x=0.
                self.n.odom.twist.twist.linear.y=0.
                self.n.odom.twist.twist.linear.z=0.
                for t in (.3,.4,.5):self.tick(t)
                self.n._request_mode.assert_not_called()
                self.tick(.6)
                self.n._request_mode.assert_called_once_with('POSCTL')

    def test_velocity_limits_use_enu_not_body_components(self):
        # Pitch=4 degrees mixes body X and Z; both limits must use ENU.
        for body_x,body_z,expected in ((.31,-.15,True),(.29,.19,False),(.29,-.19,False)):
            with self.subTest(body_x=body_x,body_z=body_z):
                self.setUp();self.n.odom_twist_in_child_frame=True
                q=self.n.odom.pose.pose.orientation
                q.y=math.sin(math.radians(4.)/2.)
                q.w=math.cos(math.radians(4.)/2.)
                self.n.odom.twist.twist.linear.x=body_x
                self.n.odom.twist.twist.linear.z=body_z
                self.confirm()
                if expected:self.n._request_mode.assert_called_once_with('POSCTL')
                else:self.n._request_mode.assert_not_called()

    def test_at_least_three_distinct_samples_even_with_shorter_hold(self):
        self.n.wall_level_handover_hold_s=.15
        self.tick(0);self.tick(.15)
        self.n._request_mode.assert_not_called()
        self.tick(.16)
        self.n._request_mode.assert_called_once_with('POSCTL')

    def test_brief_height_or_speed_excursion_between_timer_ticks_restarts_window(self):
        for fault in ('height','vertical','horizontal'):
            with self.subTest(fault=fault):
                self.setUp();self.tick(0);self.tick(.1)
                original=copy.deepcopy(self.n.odom)
                bad=copy.deepcopy(original)
                if fault=='height':bad.pose.pose.position.z+=.2
                elif fault=='vertical':bad.twist.twist.linear.z=-.3
                else:bad.twist.twist.linear.x=.4
                self.refresh(.15);bad.header.stamp=self.fx.f.now
                with mock.patch.object(rospy.Time,'now',return_value=self.fx.f.now):
                    self.n._on_odom(bad)
                self.assertIsNone(self.n.wall_level_window)
                self.refresh(.18);original.header.stamp=self.fx.f.now
                with mock.patch.object(rospy.Time,'now',return_value=self.fx.f.now):
                    self.n._on_odom(original)
                for t in (.2,.3,.4):self.tick(t)
                self.n._request_mode.assert_not_called()
                self.tick(.48)
                self.n._request_mode.assert_called_once_with('POSCTL')

    def test_brief_wall_reference_failure_or_phase_regression_restarts_window(self):
        for fault in ('valid','active','reference_jump','phase'):
            with self.subTest(fault=fault):
                self.setUp();self.tick(0);self.tick(.1)
                original=copy.deepcopy(self.n.wall)
                bad=copy.deepcopy(original)
                if fault=='valid':bad.vertical_reference_valid=False
                elif fault=='active':bad.vertical_control_active=False
                elif fault=='reference_jump':bad.vertical_reference_m+=.2
                else:bad.state=W.STATE_DETACH_ROTATE
                self.refresh(.15);bad.header.stamp=self.fx.f.now
                with mock.patch.object(rospy.Time,'now',return_value=self.fx.f.now):
                    self.n._on_wall(bad)
                self.assertIsNone(self.n.wall_level_window)
                self.refresh(.18);original.header.stamp=self.fx.f.now
                with mock.patch.object(rospy.Time,'now',return_value=self.fx.f.now):
                    self.n._on_wall(original)
                for t in (.2,.3,.4):self.tick(t)
                self.n._request_mode.assert_not_called()
                self.tick(.48)
                self.n._request_mode.assert_called_once_with('POSCTL')

    def test_detach_rotate_cannot_prequalify_recover_window(self):
        self.n.wall.state=W.STATE_DETACH_ROTATE
        self.confirm()
        self.n.wall.state=W.STATE_RECOVER
        for t in (.4,.5,.6):self.tick(t)
        self.n._request_mode.assert_not_called()
        self.tick(.7)
        self.n._request_mode.assert_called_once_with('POSCTL')

    def test_approach_timeout_cannot_bypass_height_and_horizontal_requirements(self):
        for fault in ('height','horizontal'):
            with self.subTest(fault=fault):
                self.setUp();self.n.phase=Manager.RETURNING
                self.n.wall.state=W.STATE_ABORT
                self.n.wall.reason='自动接近超时：总时限20秒，含测距暂停'
                self.n.wall_approach_timeout_returning=True
                if fault=='height':self.n.odom.pose.pose.position.z+=.2
                else:self.n.odom.twist.twist.linear.x=.4
                self.confirm()
                self.n._request_mode.assert_not_called()
                self.assertFalse(self.n._handover_ready(self.fx.f.now))

    def test_phase_whitelist_blocks_dispatch_including_canceling_entry(self):
        for manager_phase in (Manager.ACTIVE,Manager.RETURNING,Manager.CANCELING_ENTRY):
            for fault in ('detach_rotate','wall_stale'):
                with self.subTest(manager_phase=manager_phase,fault=fault):
                    self.setUp();self.n.phase=manager_phase
                    self.n.wall_level_returning=True
                    if fault=='detach_rotate':self.n.wall.state=W.STATE_DETACH_ROTATE
                    else:self.n.wall.header.stamp=rospy.Time.from_sec(5.)
                    with mock.patch('threading.Thread') as thread,mock.patch.object(
                            rospy.Time,'now',return_value=self.fx.f.now):
                        Manager._request_mode(self.n,'POSCTL')
                    thread.assert_not_called()

    def test_worker_rechecks_phase_after_service_wait_even_in_canceling_entry(self):
        for manager_phase in (Manager.RETURNING,Manager.CANCELING_ENTRY):
            for cleanup in (False,True):
                for fault in ('detach_rotate','wall_stale'):
                    with self.subTest(manager_phase=manager_phase,cleanup=cleanup,fault=fault):
                        self.setUp();self.confirm();self.n.phase=manager_phase
                        self.n.offboard_exit_cleanup=cleanup
                        self.n._entry_cancel_handover_ready=mock.Mock(return_value=True)
                        self.n._offboard_exit_request_ready=mock.Mock(return_value=True)
                        self.n.set_mode=mock.Mock()
                        def revoke(*_args,**_kwargs):
                            if fault=='detach_rotate':self.n.wall.state=W.STATE_DETACH_ROTATE
                            else:self.n.wall.header.stamp=rospy.Time.from_sec(5.)
                        with mock.patch.object(rospy,'wait_for_service',side_effect=revoke),mock.patch.object(
                                rospy.Time,'now',return_value=self.fx.f.now):
                            self.n._send_mode_request('POSCTL',7)
                        self.n.set_mode.assert_not_called()

    def cancellation_feedback(self, t, reference=1.):
        self.refresh(t)
        self.n.output_health_at=self.fx.f.now
        self.n.output_health={
            'sampled_at_s':self.fx.f.now.to_sec(), 'owner':0, 'output_mode':2,
            'cancel_vertical_control':{'active':True,'height_reference_m':reference},
        }

    def prepare_owner_zero_cancel(self):
        self.n.phase=Manager.CANCELING_ENTRY
        self.n.owner=0
        self.n.offboard_exit_cleanup=False
        # This owner uses mux feedback, not the unavailable wall vertical loop.
        self.n.wall.vertical_reference_valid=False
        self.n.wall.vertical_control_active=False
        self.n.wall.vertical_reference_m=99.

    def test_canceling_owner_zero_uses_mux_height_with_same_joint_window(self):
        for phase in (W.STATE_RECOVER,W.STATE_ABORT,W.STATE_EXIT):
            with self.subTest(phase=phase):
                self.setUp();self.prepare_owner_zero_cancel()
                self.n.wall.state=phase
                self.n.odom.twist.twist.linear.x=.25
                self.n.odom.twist.twist.linear.z=-.15
                self.n.odom.twist.twist.angular.z=2.
                for t in (0.,.1,.2):
                    self.cancellation_feedback(t)
                    self.assertFalse(self.n._entry_cancel_handover_ready(self.fx.f.now))
                self.cancellation_feedback(.3)
                self.assertTrue(self.n._entry_cancel_handover_ready(self.fx.f.now))
                self.n.set_mode=mock.Mock(return_value=type('Reply',(),{'mode_sent':True})())
                with mock.patch.object(rospy,'wait_for_service'),mock.patch.object(
                        rospy.Time,'now',return_value=self.fx.f.now):
                    self.n._send_mode_request('POSCTL',7)
                self.n.set_mode.assert_called_once()
                self.assertEqual(self.n.set_mode.call_args.args[0].custom_mode,'POSCTL')

    def test_canceling_owner_zero_rejects_bad_mux_health_and_motion(self):
        faults=('missing','not_object','receipt_stale','receipt_future','source_stale',
                'source_future','source_nan','source_string','wrong_owner','wrong_mode',
                'inactive','bad_control','no_reference','nan_reference','height_mismatch',
                'horizontal','vertical','tilt')
        for fault in faults:
            with self.subTest(fault=fault):
                self.setUp();self.prepare_owner_zero_cancel()
                for t in (0.,.1):
                    self.cancellation_feedback(t)
                    self.assertFalse(self.n._entry_cancel_handover_ready(self.fx.f.now))
                for t in (.2,.3,.4):
                    self.cancellation_feedback(t)
                    health=self.n.output_health
                    if fault=='missing':self.n.output_health=None
                    elif fault=='not_object':self.n.output_health=[]
                    elif fault=='receipt_stale':self.n.output_health_at=rospy.Time.from_sec(5.)
                    elif fault=='receipt_future':self.n.output_health_at=rospy.Time.from_sec(20.)
                    elif fault=='source_stale':health['sampled_at_s']=5.
                    elif fault=='source_future':health['sampled_at_s']=20.
                    elif fault=='source_nan':health['sampled_at_s']=math.nan
                    elif fault=='source_string':health['sampled_at_s']='10.2'
                    elif fault=='wrong_owner':health['owner']=2
                    elif fault=='wrong_mode':health['output_mode']=0
                    elif fault=='inactive':health['cancel_vertical_control']['active']=False
                    elif fault=='bad_control':health['cancel_vertical_control']=None
                    elif fault=='no_reference':health['cancel_vertical_control'].pop('height_reference_m')
                    elif fault=='nan_reference':health['cancel_vertical_control']['height_reference_m']=math.nan
                    elif fault=='height_mismatch':health['cancel_vertical_control']['height_reference_m']=1.2
                    elif fault=='horizontal':self.n.odom.twist.twist.linear.x=.4
                    elif fault=='vertical':self.n.odom.twist.twist.linear.z=-.3
                    else:
                        self.n.odom.pose.pose.orientation.y=math.sin(math.radians(6.)/2.)
                        self.n.odom.pose.pose.orientation.w=math.cos(math.radians(6.)/2.)
                    self.assertFalse(self.n._entry_cancel_handover_ready(self.fx.f.now))
                    self.assertIsNone(self.n.wall_level_window)

    def test_canceling_worker_rechecks_mux_health_after_service_wait(self):
        for fault in ('inactive','stale','height_mismatch'):
            with self.subTest(fault=fault):
                self.setUp();self.prepare_owner_zero_cancel()
                for t in (0.,.1,.2,.3):
                    self.cancellation_feedback(t)
                    ready=self.n._entry_cancel_handover_ready(self.fx.f.now)
                self.assertTrue(ready)
                self.n.set_mode=mock.Mock()
                def revoke(*_args,**_kwargs):
                    if fault=='inactive':self.n.output_health['cancel_vertical_control']['active']=False
                    elif fault=='stale':self.n.output_health['sampled_at_s']=5.
                    else:self.n.output_health['cancel_vertical_control']['height_reference_m']=1.2
                with mock.patch.object(rospy,'wait_for_service',side_effect=revoke),mock.patch.object(
                        rospy.Time,'now',return_value=self.fx.f.now):
                    self.n._send_mode_request('POSCTL',7)
                self.n.set_mode.assert_not_called()

    def test_owner_zero_cancellation_compatibility_does_not_expand_stage_or_owner_scope(self):
        for manager_phase,wall_phase in ((Manager.ACTIVE,W.STATE_RECOVER),
                (Manager.RETURNING,W.STATE_ABORT),(Manager.CANCELING_ENTRY,W.STATE_DETACH_ROTATE)):
            with self.subTest(manager_phase=manager_phase,wall_phase=wall_phase):
                self.setUp();self.prepare_owner_zero_cancel()
                self.n.phase=manager_phase;self.n.wall.state=wall_phase
                for t in (0.,.1,.2,.3):
                    self.cancellation_feedback(t)
                    self.assertFalse(self.n._handover_ready(self.fx.f.now))
                    self.assertIsNone(self.n.wall_level_window)

    def test_height_reference_change_inside_tolerance_restarts_confirmation(self):
        for cancellation in (False,True):
            with self.subTest(cancellation=cancellation):
                self.setUp()
                if cancellation:self.prepare_owner_zero_cancel()
                for t in (0.,.1,.2):
                    if cancellation:self.cancellation_feedback(t)
                    else:self.refresh(t)
                    self.assertFalse(self.n._handover_ready(self.fx.f.now))
                for t in (.3,.4,.5):
                    if cancellation:self.cancellation_feedback(t,reference=1.05)
                    else:
                        self.refresh(t);self.n.wall.vertical_reference_m=1.05
                    self.assertFalse(self.n._handover_ready(self.fx.f.now))
                if cancellation:self.cancellation_feedback(.6,reference=1.05)
                else:self.refresh(.6)
                self.assertTrue(self.n._handover_ready(self.fx.f.now))

    def test_mux_health_callback_invalidity_cannot_hide_between_timer_ticks(self):
        self.prepare_owner_zero_cancel()
        for t in (0.,.1):
            self.cancellation_feedback(t)
            self.assertFalse(self.n._entry_cancel_handover_ready(self.fx.f.now))
        self.refresh(.15)
        with mock.patch.object(rospy.Time,'now',return_value=self.fx.f.now):
            self.n._on_output_health(String(data='{invalid json'))
        self.assertIsNone(self.n.wall_level_window)
        self.cancellation_feedback(.18)
        restored=String(data=json.dumps(self.n.output_health))
        with mock.patch.object(rospy.Time,'now',return_value=self.fx.f.now):
            self.n._on_output_health(restored)
        for t in (.2,.3,.4):
            self.cancellation_feedback(t)
            self.assertFalse(self.n._entry_cancel_handover_ready(self.fx.f.now))
        self.cancellation_feedback(.48)
        self.assertTrue(self.n._entry_cancel_handover_ready(self.fx.f.now))

    def test_fresh_already_posctl_reassertion_does_not_require_wall_handover_stage(self):
        for armed in (False,True):
            with self.subTest(armed=armed):
                self.setUp();self.n.phase=Manager.CANCELING_ENTRY
                self.n.offboard_exit_cleanup=True
                self.n.flight.mode='POSCTL';self.n.flight.armed=armed
                self.n.wall.state=W.STATE_DETACH_ROTATE
                self.n.wall.header.stamp=rospy.Time.from_sec(5.)
                self.n.wall_at=rospy.Time.from_sec(5.)
                with mock.patch('threading.Thread') as thread,mock.patch.object(
                        rospy.Time,'now',return_value=self.fx.f.now):
                    Manager._request_mode(self.n,'POSCTL')
                thread.assert_called_once()
                self.n.set_mode=mock.Mock(return_value=type('Reply',(),{'mode_sent':True})())
                with mock.patch.object(rospy,'wait_for_service'),mock.patch.object(
                        rospy.Time,'now',return_value=self.fx.f.now):
                    self.n._send_mode_request('POSCTL',7)
                self.n.set_mode.assert_called_once()

    def test_already_posctl_exception_requires_fresh_connected_matching_mode(self):
        for fault in ('stale','disconnected','different_mode'):
            with self.subTest(fault=fault):
                self.setUp();self.n.phase=Manager.CANCELING_ENTRY
                self.n.offboard_exit_cleanup=True
                self.n.flight.mode='POSCTL'
                self.n.wall.state=W.STATE_DETACH_ROTATE
                if fault=='stale':self.n.flight_at=rospy.Time.from_sec(5.)
                elif fault=='disconnected':self.n.flight.connected=False
                else:self.n.flight.mode='AUTO.LAND'
                with mock.patch('threading.Thread') as thread,mock.patch.object(
                        rospy.Time,'now',return_value=self.fx.f.now):
                    Manager._request_mode(self.n,'POSCTL')
                thread.assert_not_called()

    def test_wall_callback_cannot_backdate_confirmation_to_older_odometry(self):
        for transition in ('enter_recover','reference_recovers','reference_changes'):
            with self.subTest(transition=transition):
                self.setUp()
                if transition=='enter_recover':self.n.wall.state=W.STATE_DETACH_ROTATE
                elif transition=='reference_recovers':self.n.wall.vertical_reference_valid=False
                # Last odometry is from .1; the new permissible state/reference
                # first arrives at .2. Confirmation starts no earlier than .2.
                self.tick(0)
                self.tick(.1)
                self.refresh(.2,update_odom=False)
                status=copy.deepcopy(self.n.wall)
                status.state=W.STATE_RECOVER
                status.vertical_reference_valid=True
                if transition=='reference_changes':status.vertical_reference_m=1.05
                with mock.patch.object(rospy.Time,'now',return_value=self.fx.f.now):
                    self.n._on_wall(status)
                for t in (.3,.4):self.tick(t)
                self.n._request_mode.assert_not_called()
                self.tick(.5)
                self.n._request_mode.assert_called_once_with('POSCTL')

    def test_mux_reference_recovery_cannot_backdate_confirmation_to_older_odometry(self):
        self.prepare_owner_zero_cancel()
        self.cancellation_feedback(.1)
        self.n.output_health['cancel_vertical_control']['active']=False
        self.assertFalse(self.n._entry_cancel_handover_ready(self.fx.f.now))
        self.refresh(.2,update_odom=False)
        health=copy.deepcopy(self.n.output_health)
        health['sampled_at_s']=self.fx.f.now.to_sec()
        health['cancel_vertical_control']['active']=True
        with mock.patch.object(rospy.Time,'now',return_value=self.fx.f.now):
            self.n._on_output_health(String(data=json.dumps(health)))
        for t in (.3,.4):
            self.cancellation_feedback(t)
            self.assertFalse(self.n._entry_cancel_handover_ready(self.fx.f.now))
        self.cancellation_feedback(.5)
        self.assertTrue(self.n._entry_cancel_handover_ready(self.fx.f.now))

    def test_canceling_worker_final_recheck_respects_new_pilot_mode_attempt_and_height(self):
        for cleanup in (False,True):
            for change in ('pilot_mode','attempt','height'):
                with self.subTest(cleanup=cleanup,change=change):
                    self.setUp();self.confirm()
                    self.n.phase=Manager.CANCELING_ENTRY
                    self.n.offboard_exit_cleanup=cleanup
                    self.n.set_mode=mock.Mock()
                    original_guard=self.n._wall_pilot_request_phase_ready
                    def race_after_stage_guard(mode,now):
                        ready=original_guard(mode,now)
                        if change=='pilot_mode':self.n.flight.mode='AUTO.LAND'
                        elif change=='attempt':self.n.takeover_attempt+=1
                        else:self.n.odom.pose.pose.position.z+=.2
                        return ready
                    with mock.patch.object(rospy,'wait_for_service'),mock.patch.object(
                            rospy.Time,'now',return_value=self.fx.f.now),mock.patch.object(
                            self.n,'_wall_pilot_request_phase_ready',side_effect=race_after_stage_guard) as guard:
                        self.n._send_mode_request('POSCTL',7)
                    guard.assert_called_once()
                    self.n.set_mode.assert_not_called()

    def test_disabled_early_handback_still_hard_blocks_detach_rotate_posctl_rpc(self):
        for phase in (Manager.RETURNING,Manager.CANCELING_ENTRY):
            with self.subTest(phase=phase):
                self.setUp();self.n.phase=phase
                self.n.wall_level_handover_enabled=False
                self.n.wall.state=W.STATE_DETACH_ROTATE
                self.n.offboard_exit_cleanup=False
                # Even a ready legacy route cannot waive the phase restriction.
                self.n._handover_ready=mock.Mock(return_value=True)
                self.n._entry_cancel_handover_ready=mock.Mock(return_value=True)
                with mock.patch('threading.Thread') as thread,mock.patch.object(
                        rospy.Time,'now',return_value=self.fx.f.now):
                    Manager._request_mode(self.n,'POSCTL')
                thread.assert_not_called()
                self.n.set_mode=mock.Mock()
                with mock.patch.object(rospy,'wait_for_service'),mock.patch.object(
                        rospy.Time,'now',return_value=self.fx.f.now):
                    self.n._send_mode_request('POSCTL',7)
                self.n.set_mode.assert_not_called()

    def test_retry_interval_is_half_second_only_for_early_pilot_return(self):
        self.confirm();self.n.request_in_flight=False;self.n.last_request_at=100.
        with mock.patch('threading.Thread') as thread,mock.patch('time.monotonic',return_value=100.49),mock.patch.object(rospy.Time,'now',return_value=self.fx.f.now):
            Manager._request_mode(self.n,'POSCTL');thread.assert_not_called()
        with mock.patch('threading.Thread') as thread,mock.patch('time.monotonic',return_value=100.50),mock.patch.object(rospy.Time,'now',return_value=self.fx.f.now):
            Manager._request_mode(self.n,'POSCTL');thread.assert_called_once()


if __name__=='__main__':unittest.main()
