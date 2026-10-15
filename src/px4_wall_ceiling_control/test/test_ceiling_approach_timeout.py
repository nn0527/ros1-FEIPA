#!/usr/bin/env python3
"""Mocked flight chain: contact entry, latched timeout, and confirmed return."""
import unittest
from unittest import mock

import rospy
from mavros_msgs.msg import State, EstimatorStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus as S, OperatorCommand
from px4_wall_ceiling_control.domain.owner_arbiter import OwnerArbiter, CandidateIntent
from test_attachment_safety import CeilingDecision
from test_flight_handover import Manager, Terminal


class CeilingApproachTimeoutTests(unittest.TestCase):
    def setUp(self):
        self.clock = 10.0
        self.clock_patch = mock.patch.object(rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(self.clock))
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        params = {'~console_contact_profile': True, '~activation_distance_m': 1.0,
                  '~approach_contact_distance_m': .07, '~approach_settle_time_s': 0.,
                  '~approach_timeout_s': 12.,
                  '~require_mechanism_ack': False, '~detach_baro_height_enabled': True}
        with mock.patch.object(rospy, 'get_param', side_effect=lambda k,d=None:params.get(k,d)), \
             mock.patch.object(rospy, 'Publisher'), mock.patch.object(rospy, 'Subscriber'), mock.patch.object(rospy, 'Timer'):
            self.decision = CeilingDecision()
        self.refresh()
        self.decision._enter_state(S.STATE_APPROACH, rospy.Time.now(), 'test')

    def refresh(self, distance=.2, mode='OFFBOARD', requested=True):
        self.decision._on_state(State(connected=True, armed=True, mode=mode))
        odom = Odometry()
        odom.pose.pose.orientation.w = 1.
        self.decision._on_odom(odom)
        self.decision._on_range(Range(range=distance, min_range=.03, max_range=40.))
        self.decision._on_operator(OperatorCommand(
            mode=OperatorCommand.MODE_CEILING if requested else OperatorCommand.MODE_STANDBY,
            enable_control=requested))

    def expire(self):
        self.clock = 22.01
        self.refresh()
        self.decision._update_state_machine(rospy.Time.now())
        self.assertEqual(self.decision.state, S.STATE_FAULT)
        return self.decision._build_status(rospy.Time.now(), .02)

    def manager(self, status):
        with mock.patch.object(rospy, 'get_param', side_effect=lambda k,d=None:d), \
             mock.patch.object(rospy, 'Publisher'), mock.patch.object(rospy, 'Subscriber'), \
             mock.patch.object(rospy, 'ServiceProxy'), mock.patch.object(rospy, 'Timer'):
            manager = Manager()
        manager.prestream_pub = mock.Mock()
        manager.phase = Manager.ACTIVE
        manager.selected_mode = OperatorCommand.MODE_CEILING
        manager.ceiling_baro_handover_enabled = True
        manager.ceiling = status
        manager.flight = State(connected=True, armed=True, mode='OFFBOARD')
        manager.ceiling_at = manager.flight_at = manager.output_at = manager.owner_at = rospy.Time.now()
        manager.estimator_at = manager.odom_at = manager.rc_at = rospy.Time.now()
        manager.output_mode = 2
        manager.owner = 1
        manager.estimator = EstimatorStatus(attitude_status_flag=True, velocity_vert_status_flag=True)
        manager.odom = self.decision.odom
        manager.odom.twist.twist.linear.z = -.5  # No speed-settle deadlock.
        manager.rc_channels = (1500, 1500, 1625)
        manager.rc_roll_channel_index = 0
        manager.rc_pitch_channel_index = 1
        manager.rc_throttle_channel_index = 2
        manager._requested_mode = lambda now: OperatorCommand.MODE_CEILING
        manager._request_mode = mock.Mock()
        return manager

    def tick_manager(self, manager):
        with mock.patch.dict(Manager._on_timer.__globals__, kill_clear=lambda *a:True):
            manager._on_timer(None)

    def test_median_rejects_isolated_low_sample_and_accepts_seven_cm(self):
        for distance in (.09, .06, .08):
            self.clock += .05
            self.refresh(distance)
            self.decision._update_state_machine(rospy.Time.now())
            self.assertEqual(self.decision.state, S.STATE_APPROACH)
        self.clock += .05
        self.refresh(.07)  # Median(.06, .08, .07) is exactly the entry boundary.
        self.decision._update_state_machine(rospy.Time.now())
        self.assertEqual(self.decision.state, S.STATE_ATTACH_CONTROL)
        self.assertFalse(self.decision._attach_confirmed(rospy.Time.now()))
        self.assertAlmostEqual(self.decision.wall_contact_distance_m, .067)

    def test_contact_at_67_mm_enters_hold_after_one_second(self):
        self.decision._enter_state(S.STATE_ATTACH_CONTROL, rospy.Time.now(), 'test')
        for index in range(10):
            self.clock = 10. + index * .1
            self.refresh(.067)
            self.decision._update_state_machine(rospy.Time.now())
            self.assertEqual(self.decision.state, S.STATE_ATTACH_CONTROL)
        self.clock = 11.
        self.refresh(.067)
        self.assertTrue(self.decision._attach_confirmed(rospy.Time.now()))
        self.decision._update_state_machine(rospy.Time.now())
        self.assertEqual(self.decision.state, S.STATE_SURFACE_HOLD)

    def test_raw_distance_above_67_mm_restarts_contact_confirmation(self):
        self.decision._enter_state(S.STATE_ATTACH_CONTROL, rospy.Time.now(), 'test')
        for index in range(10):
            self.clock = 10. + index * .1
            self.refresh(.066)
        self.clock = 11.
        self.refresh(.0671)
        self.assertLess(self.decision.distance_filtered_m, .067)
        self.assertFalse(self.decision._attach_confirmed(rospy.Time.now()))
        self.decision._update_state_machine(rospy.Time.now())
        self.assertEqual(self.decision.state, S.STATE_ATTACH_CONTROL)
        self.clock = 11.1
        self.refresh(.067)
        self.assertFalse(self.decision._attach_confirmed(rospy.Time.now()))

    def test_entry_waits_for_three_new_samples(self):
        for index in range(3):
            self.clock += .05
            self.refresh(.069)
            self.decision._update_state_machine(rospy.Time.now())
            self.assertEqual(self.decision.state,
                             S.STATE_APPROACH if index < 2 else S.STATE_ATTACH_CONTROL)

    def test_twelve_second_boundary_fault_latches_and_keeps_output(self):
        self.clock = 22.
        self.refresh()
        self.decision._update_state_machine(rospy.Time.now())
        self.assertEqual(self.decision.state, S.STATE_APPROACH)
        status = self.expire()
        self.assertTrue(status.approach_timed_out)
        self.assertTrue(self.decision.rearm_required)
        candidate = self.decision._build_control_candidate(status)
        self.assertTrue(candidate.valid)
        self.assertTrue(candidate.keep_ownership)
        self.assertFalse(candidate.acquire_request)
        arbiter = OwnerArbiter(.5, owner=1)
        event = arbiter.update(self.clock, OperatorCommand.MODE_CEILING, False,
            CandidateIntent(enabled=True, fresh=True, keep_ownership=candidate.keep_ownership,
                            candidate_valid=candidate.valid), CandidateIntent())
        self.assertEqual(event.owner, 1)
        for t in (22.1, 23., 30.):
            self.clock = t
            self.refresh(.05)
            self.decision._update_state_machine(rospy.Time.now())
            self.assertEqual(self.decision.state, S.STATE_FAULT)

    def test_cancel_does_not_release_output_before_altctl_confirmation(self):
        self.expire()
        self.refresh(requested=False)
        self.decision._update_state_machine(rospy.Time.now())
        self.assertEqual(self.decision.state, S.STATE_FAULT)
        self.refresh(mode='ALTCTL', requested=False)
        self.decision._update_state_machine(rospy.Time.now())
        self.assertEqual(self.decision.state, S.STATE_NORMAL_FLIGHT)
        self.assertFalse(self.decision._build_status(rospy.Time.now(), .02).keep_ownership)

    def test_timeout_requests_altctl_without_baro_baseline_and_waits_for_feedback(self):
        manager = self.manager(self.expire())
        self.assertFalse(manager.ceiling.baro_reference_ready)
        self.tick_manager(manager)
        self.assertEqual(manager.phase, Manager.RETURNING)
        manager._request_mode.assert_called_once_with('ALTCTL')
        self.assertEqual(manager.owner, 1)
        manager.set_mode = mock.Mock(return_value=type('Reply', (), {'mode_sent':True})())
        with mock.patch.object(rospy, 'wait_for_service'), mock.patch.dict(Manager._send_mode_request.__globals__, kill_clear=lambda *a:True):
            manager._send_mode_request('ALTCTL')
        self.assertEqual(manager.set_mode.call_args.args[0].custom_mode, 'ALTCTL')
        self.assertEqual(manager.phase, Manager.RETURNING)  # RPC acceptance is not mode feedback.
        manager.flight = State(connected=False, armed=True, mode='ALTCTL')
        self.tick_manager(manager)
        self.assertEqual(manager.phase, Manager.RETURNING)
        manager.flight = State(connected=True, armed=True, mode='ALTCTL', manual_input=True)
        self.tick_manager(manager)
        self.assertEqual(manager.phase, Manager.LOCKED)
        self.assertFalse(manager.prestream_pub.publish.call_args.args[0].data)
        manager._request_mode.reset_mock()
        self.tick_manager(manager)  # Holding the same request cannot reenter OFFBOARD.
        self.assertEqual(manager.phase, Manager.LOCKED)
        manager._request_mode.assert_not_called()

    def test_rejected_altctl_request_retries_without_releasing_fault(self):
        manager = self.manager(self.expire())
        self.tick_manager(manager)
        manager.set_mode = mock.Mock(return_value=type('Reply', (), {'mode_sent':False})())
        with mock.patch.object(rospy, 'wait_for_service'), mock.patch.dict(Manager._send_mode_request.__globals__, kill_clear=lambda *a:True):
            manager._send_mode_request('ALTCTL')
        self.assertEqual(manager.phase, Manager.RETURNING)
        self.assertFalse(manager.request_in_flight)
        manager._request_mode.reset_mock()
        self.tick_manager(manager)
        manager._request_mode.assert_called_once_with('ALTCTL')
        self.assertEqual(self.decision.state, S.STATE_FAULT)
        self.assertTrue(manager.prestream_pub.publish.call_args.args[0].data)

    def test_stale_rc_or_state_blocks_altctl_rpc_then_fresh_rc_allows_return(self):
        manager = self.manager(self.expire())
        manager.rc_at = rospy.Time.from_sec(self.clock - 1.)
        self.tick_manager(manager)
        self.assertEqual(manager.phase, Manager.RETURNING)
        manager._request_mode.assert_not_called()
        manager.set_mode = mock.Mock()
        with mock.patch.object(rospy, 'wait_for_service'), mock.patch.dict(Manager._send_mode_request.__globals__, kill_clear=lambda *a:True):
            manager._send_mode_request('ALTCTL')
        manager.set_mode.assert_not_called()
        manager.rc_at = rospy.Time.now()
        manager.flight_at = rospy.Time.from_sec(self.clock - 2.)
        self.assertFalse(manager._handover_ready(rospy.Time.now()))
        manager.flight_at = rospy.Time.now()
        self.tick_manager(manager)
        manager._request_mode.assert_called_once_with('ALTCTL')

    def test_unrelated_fault_does_not_use_approach_timeout_return(self):
        manager = self.manager(self.expire())
        manager.ceiling.approach_timed_out = False
        self.tick_manager(manager)
        self.assertEqual(manager.phase, Manager.ACTIVE)
        manager._request_mode.assert_not_called()

    def test_confirmed_altctl_clears_terminal_intent(self):
        with mock.patch.object(rospy, 'get_param', side_effect=lambda k,d=None:d), \
             mock.patch.object(rospy, 'Publisher'), mock.patch.object(rospy, 'Subscriber'), \
             mock.patch.object(rospy, 'Timer'), mock.patch.object(rospy, 'set_param'):
            terminal = Terminal()
            terminal.mode = OperatorCommand.MODE_CEILING
            terminal._on_state(State(connected=True, armed=True, mode='OFFBOARD'))
            terminal._on_state(State(connected=True, armed=True, mode='ALTCTL', manual_input=True))
        self.assertEqual(terminal.mode, OperatorCommand.MODE_STANDBY)
        self.assertFalse(terminal.detaching)


if __name__ == '__main__':
    unittest.main()
