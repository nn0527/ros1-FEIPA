"""CH12 consent, fixed recovery target and late RPC checks; no live transports."""
import math
import unittest
from types import SimpleNamespace
from unittest import mock
import rospy
from std_msgs.msg import String
from mavros_msgs.msg import State
from px4_wall_ceiling_control.msg import OperatorCommand as Op, WallPerchStatus as W
import test_rc_wall_toggle as terminal_fixtures
import test_range_height_integration as fixtures
import test_positive_recovery as positive
import test_flight_handover as manager_module
from test_detach_state_machine import at, quaternion


class OperatorConsentTests(unittest.TestCase):
    def setUp(self):
        self.fx = terminal_fixtures.MixedOperatorTest()
        self.fx.setUp(); self.addCleanup(self.fx.doCleanups)
        self.fx.posctl()
        self.n = n = self.fx.node
        self.fx.press()  # first edge starts wall
        n._on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        self.fx.status(W.STATE_WALL_HOLD)
        n.flight_at = rospy.Time.now()
        self.fx.press()  # second edge requests detach
        n._on_timer(None)
        self.assertEqual(n.wall_detach_pub.publish.call_count, 1)

    def recovery(self, state=W.STATE_RECOVER):
        self.fx.status(state)
        self.n.wall_status.recover_attitude_locked = True
        self.n.wall_status.keep_ownership = True
        self.n.flight_at = rospy.Time.now()

    def intent(self):
        self.n._publish_intent()
        return self.n.command_pub.publish.call_args.args[0]

    def test_second_edge_is_detach_only_third_edge_grants_consent(self):
        n = self.n
        self.assertEqual(self.intent().pilot_handover_request_time, at(0.))
        self.fx.status(W.STATE_DETACH_ROTATE)
        n.flight_at = rospy.Time.now()
        self.fx.press()  # ignored while rotating, never queued for recovery
        self.recovery()
        self.assertEqual(self.intent().pilot_handover_request_time, at(0.))
        self.fx.press()
        event = self.intent().pilot_handover_request_time
        self.assertGreater(event.to_sec(), 0.)
        self.assertEqual(n.mode, Op.MODE_WALL)
        self.assertTrue(n.detaching)
        n._on_timer(None)
        self.assertEqual(n.wall_detach_pub.publish.call_count, 1)
        self.recovery(W.STATE_EXIT)
        self.fx.press()
        self.assertEqual(self.intent().pilot_handover_request_time, event)
        n._on_state(State(connected=True, armed=True, mode='POSCTL'))
        self.assertEqual(self.intent().mode, Op.MODE_STANDBY)
        self.assertEqual(self.intent().pilot_handover_request_time, at(0.))
        self.fx.status(W.STATE_IDLE)
        n.flight_at = rospy.Time.now()
        self.fx.press()
        self.assertEqual(self.intent().mode, Op.MODE_WALL)
        self.assertEqual(self.intent().pilot_handover_request_time, at(0.))

    def test_no_edge_means_no_consent_and_cancel_clears_consent(self):
        self.recovery()
        for _ in range(10):
            self.n._on_timer(None)
            self.assertEqual(self.intent().pilot_handover_request_time, at(0.))
        self.fx.press()
        self.assertGreater(self.intent().pilot_handover_request_time.to_sec(), 0.)
        self.n._on_command(String(data='cancel'))
        self.assertEqual(self.intent().pilot_handover_request_time, at(0.))

    def test_invalid_conditions_cannot_grant_consent(self):
        for case in ('kill','stale','fault','disarmed','disconnected','mode','unlocked_target','no_owner'):
            with self.subTest(case=case):
                self.recovery()
                n = self.n
                n.flight_connected = n.flight_armed = True
                n.flight_mode = 'OFFBOARD'
                n.kill_guard_enabled = True
                n._kill_report = lambda: {'valid': True, 'active': False}
                if case == 'kill': n._kill_report = lambda: {'valid': False, 'active': False}
                if case == 'stale': n.wall_status.header.stamp = at(1.)
                if case == 'fault': n.wall_status.fault_detected = True
                if case == 'disarmed': n.flight_armed = False
                if case == 'disconnected': n.flight_connected = False
                if case == 'mode': n.flight_mode = 'POSCTL'
                if case == 'unlocked_target': n.wall_status.recover_attitude_locked = False
                if case == 'no_owner': n.wall_status.keep_ownership = False
                n._on_wall_toggle()
                self.assertEqual(self.intent().pilot_handover_request_time, at(0.))


class ManagerConsentTests(unittest.TestCase):
    def setUp(self):
        self.fx = fixtures.RangeModeManagerTests()
        self.fx.setUp(); self.addCleanup(self.fx.doCleanups)
        self.n = n = self.fx.n
        n.wall.recover_attitude_locked = n.wall.recover_stable = True
        n.wall.detach_rotation_start_time = at(8.)
        n.odom.header.stamp = at(10.)
        n.operator_timeout_s = .5
        self.consent(0.)

    def consent(self, seconds):
        n = self.n
        n.operator = Op(mode=Op.MODE_WALL, enable_control=True, source='terminal',
                        pilot_handover_request_time=at(seconds))
        n.operator.header.stamp = n.operator_at = at(10.)

    def test_stable_recovery_does_not_automatically_hand_back(self):
        self.assertFalse(self.n._handover_ready(at(10.)))
        self.consent(9.5)
        self.assertTrue(self.n._handover_ready(at(10.)))
        self.n.rc_at = at(9.)
        self.assertFalse(self.n._handover_ready(at(10.)))

    def test_stale_future_wrong_mode_or_previous_cycle_consent_is_rejected(self):
        n = self.n
        for case in ('old_event','future_event','old_receipt','old_source','future_source','disabled','wrong_mode','source','new_cycle'):
            with self.subTest(case=case):
                self.consent(9.5)
                n.wall.detach_rotation_start_time = at(8.)
                if case == 'old_event': n.operator.pilot_handover_request_time = at(7.9)
                if case == 'future_event': n.operator.pilot_handover_request_time = at(11.)
                if case == 'old_receipt': n.operator_at = at(9.)
                if case == 'old_source': n.operator.header.stamp = at(9.)
                if case == 'future_source': n.operator.header.stamp = at(11.)
                if case == 'disabled': n.operator.enable_control = False
                if case == 'wrong_mode': n.operator.mode = Op.MODE_CEILING
                if case == 'source': n.operator.source = 'joystick'
                if case == 'new_cycle': n.wall.detach_rotation_start_time = at(9.8)
                self.assertFalse(n._handover_ready(at(10.)))

    def test_rpc_worker_rechecks_consent_after_service_wait(self):
        n = self.n
        self.consent(9.5)
        n.phase = n.RETURNING
        n.wall_pilot_mode = 'POSCTL'
        n.takeover_attempt = 3
        n.flight = State(connected=True, armed=True, mode='OFFBOARD')
        n.flight.header.stamp = n.flight_at = at(10.)
        n.state_timeout_s = 1.
        n.set_mode = mock.Mock(return_value=SimpleNamespace(mode_sent=True))
        n.request_in_flight = True
        with mock.patch.object(manager_module.rospy, 'wait_for_service',
                               side_effect=lambda *a, **kw: self.consent(0.)):
            n._send_mode_request('POSCTL', attempt=3, range_identity=n._wall_range_identity())
        n.set_mode.assert_not_called()
        self.consent(9.5)
        with mock.patch.object(manager_module.rospy, 'wait_for_service'):
            n._send_mode_request('POSCTL', attempt=3, range_identity=n._wall_range_identity())
        self.assertEqual(n.set_mode.call_args.args[0].custom_mode, 'POSCTL')
        self.assertEqual(n.phase, n.RETURNING)  # RPC success is not mode confirmation.


class OutputHoldTests(unittest.TestCase):
    def test_stick_trim_cannot_change_captured_recovery_or_exit(self):
        import test_candidate_latency as mux_fixtures
        fx = mux_fixtures.CandidateLatencyTest()
        fx.setUp(); self.addCleanup(fx.doCleanups)
        n = fx.node
        n.rc_assist_enabled = n.rc_roll_yaw_enabled = True
        n.rc_roll_rad = n.rc_yaw_rad = .1
        n._calibrated_rc_attitude = mock.Mock(side_effect=AssertionError('stick trim reached hold'))
        for phase in (W.STATE_RECOVER, W.STATE_EXIT):
            c = fx.candidate(phase=phase, angle=8.)
            c.recover_attitude_locked = True
            q = n._assisted_attitude(fx.now, c.attitude_setpoint, c.SOURCE_WALL, c)
            self.assertEqual(q, c.attitude_setpoint)
            self.assertEqual((n.rc_roll_rad, n.rc_pitch_rad, n.rc_yaw_rad), (0., 0., 0.))


class LongHoldTests(unittest.TestCase):
    def test_fixed_attitude_and_height_support_survive_long_wait(self):
        fx = positive.PositiveRecoveryTests()
        fx.setUp(); self.addCleanup(fx.doCleanups)
        n = fx.n
        fx.fx.frame(12.9, distance=1., angle=5.)
        fx.finish(5.)
        delattr(n, '_horizontal_attitude')  # Exercise the production attitude path.
        captured = n.recover_entry_attitude
        for i in range(1, 1101):  # 110 seconds; longer than old recovery budget.
            t = 13.+i*.1
            fx.fx.frame(t, distance=1., angle=5., height=math.nan, vz=math.nan)
            n._update_state_machine(at(t))
            _, q, thrust, _ = n._candidate(at(t))
            self.assertIn(n.state, (W.STATE_RECOVER, W.STATE_EXIT))
            self.assertEqual(q, captured)
            self.assertTrue(math.isfinite(thrust))
            self.assertFalse(n.detach_completed)
        # Loss of valid feedback must still take the existing fault path.
        n.detach_vertical_fault = 'test feedback loss'
        n._update_state_machine(at(t+.1))
        self.assertIn(n.state, (W.STATE_ABORT, W.STATE_EXIT))
        if n.state == W.STATE_EXIT:
            self.assertTrue(n.detach_range_faulted)
