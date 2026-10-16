"""Post-detachment routing, height capture and recovery continuity."""
import math
import unittest
from unittest import mock
import rospy
from mavros_msgs.msg import State
from px4_wall_ceiling_control.msg import WallPerchStatus as W
from px4_wall_ceiling_control.domain.geometry import quaternion_angle
import test_detach_state_machine as fixture


class DetachAbortRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.fx = fixture.DetachStateMachineTests()
        self.n = self.fx.node(**{'~detach_abort_route_enabled': True})
        self.fx.enter(self.n, W.STATE_DETACH_ROTATE, 10.)

    def sample(self, t=10.1, pitch=40., height=2.3, source=None):
        self.fx.feedback(self.n, t, tilt=pitch, height=height, source=source)
        self.fx.output(self.n, t, tilt=pitch)
        self.n._prepare_detach_vertical_feedback(fixture.at(t))

    def fail(self, t=10.1):
        self.fx.enter(self.n, W.STATE_ABORT, t)

    def test_pitch_gate_uses_both_signs_and_exact_boundary(self):
        for pitch, expected in ((-20., W.STATE_ABORT), (20., W.STATE_ABORT),
                                (-20.01, W.STATE_RECOVER), (20.01, W.STATE_RECOVER)):
            with self.subTest(pitch=pitch):
                self.setUp();self.sample(pitch=pitch);self.fail()
                self.assertEqual(self.n.state, expected)
                self.assertEqual(self.n.detach_abort_recovery, expected == W.STATE_RECOVER)

    def test_successful_rotation_routes_to_abort_without_full_completion(self):
        self.sample(pitch=3.)
        self.fx.enter(self.n, W.STATE_RECOVER, 10.1)
        self.assertEqual(self.n.state, W.STATE_ABORT)
        self.assertTrue(self.n.detach_rotation_succeeded)
        self.assertFalse(self.n.detach_completed)
        self.assertIn('rotation completed', self.n.last_reason)

    def test_recover_seeds_current_attitude_and_current_height_once(self):
        self.sample();self.fail()
        n = self.n
        self.assertEqual(n.state, W.STATE_RECOVER)
        self.assertAlmostEqual(n.detach_vertical_controller.reference, 2.3)
        self.assertEqual(n.detach_altitude_m, n.recovery_altitude_m)
        self.assertAlmostEqual(n.recovery_altitude_m, 2.3)
        self.assertLess(quaternion_angle(n.detach_abort_recovery_q, fixture.quaternion(40.)), 1e-7)
        entered = n.state_entered_at
        self.sample(10.2, 38., 2.25);self.fail(10.2)
        self.assertEqual(n.state_entered_at, entered)
        self.assertAlmostEqual(n.recovery_altitude_m, 2.3)

    def test_large_angle_recover_candidate_is_not_replaced_by_horizontal_control(self):
        self.sample();self.fail()
        self.n._horizontal_attitude = mock.Mock(side_effect=AssertionError('horizontal override'))
        self.sample(10.12, 40., 2.3)
        _, q, thrust, _ = self.n._candidate(fixture.at(10.12))
        self.assertTrue(math.isfinite(thrust))
        self.assertGreater(math.degrees(self.n._euler_from_quaternion(q)[1]), 39.)
        self.assertLessEqual(quaternion_angle(q, fixture.quaternion(40.)), math.radians(.2)+1e-7)
        self.n._horizontal_attitude.assert_not_called()

    def test_bridge_transitions_only_on_fresh_measured_pitch(self):
        self.sample();self.fail()
        self.sample(10.2, 21., 2.3);self.fx.tick(self.n, 10.2)
        self.assertEqual(self.n.state, W.STATE_RECOVER)
        self.sample(10.3, 20., 2.3);self.fx.tick(self.n, 10.3)
        self.assertEqual(self.n.state, W.STATE_ABORT)
        self.assertFalse(self.n.detach_abort_recovery)
        self.assertAlmostEqual(self.n.recovery_altitude_m, 2.3)

    def test_stale_or_nonfinite_pose_cannot_admit_abort(self):
        self.sample(pitch=2., source=9.)
        self.fail()
        self.assertEqual(self.n.state, W.STATE_RECOVER)
        self.n.odom.pose.pose.orientation.w = math.nan
        self.assertFalse(self.n._detach_abort_gate(fixture.at(10.1)))
        self.assertIsNone(self.n._detach_abort_recovery_attitude(fixture.at(10.1)))

    def test_persistent_cancel_and_original_timeout_do_not_reset_bridge(self):
        self.sample();self.fail()
        entered = self.n.state_entered_at
        self.n.cancel_requested = True
        self.n._mode_requested = lambda now: False
        self.sample(26., 40., 2.3);self.fx.tick(self.n, 26.)
        self.assertEqual(self.n.state, W.STATE_RECOVER)
        self.assertEqual(self.n.state_entered_at, entered)
        self.assertAlmostEqual(self.n.recovery_altitude_m, 2.3)
        self.assertIn('超过原恢复时限', self.n.last_reason)

    def test_top_level_cancel_is_routed_from_detach(self):
        self.sample();self.n.cancel_requested = True
        self.fx.tick(self.n, 10.1)
        self.assertEqual(self.n.state, W.STATE_RECOVER)
        self.assertTrue(self.n.detach_abort_recovery)

    def test_height_jump_is_not_blindly_reanchored(self):
        self.sample(height=3.4)
        self.assertTrue(self.n.detach_vertical_fault)
        self.fail()
        self.assertTrue(self.n.detach_abort_height_pending)
        self.assertAlmostEqual(self.n.detach_vertical_controller.reference, 2.4)
        self.assertTrue(self.n._fault_height_recovery())
        self.sample(10.2, 40., 3.41)
        _, _, thrust, _ = self.n._candidate(fixture.at(10.2))
        self.assertTrue(math.isfinite(thrust))
        self.assertFalse(self.n.detach_vertical_fault)
        self.assertFalse(self.n.detach_abort_height_pending)
        self.assertAlmostEqual(self.n.recovery_altitude_m, 3.41)

    def test_marker_is_atomic_in_candidate(self):
        self.sample();self.fail()
        status = W(state=W.STATE_RECOVER, detach_abort_recovery=True)
        c = self.n._build_control_candidate(status, self.n.maneuver_id, 1)
        self.assertTrue(c.detach_abort_recovery)
        self.assertEqual(c.control_phase, W.STATE_RECOVER)

    def test_other_abort_origins_are_unchanged(self):
        for origin in (W.STATE_SLOW_APPROACH, W.STATE_FLIP_TO_WALL, W.STATE_WALL_HOLD):
            self.n.state = origin;self.n._reset_detach_abort()
            self.sample(pitch=70.);self.fail()
            self.assertEqual(self.n.state, W.STATE_ABORT)
            self.assertFalse(self.n.detach_abort_recovery)

    def test_new_action_and_idle_clear_bridge(self):
        self.sample();self.fail()
        self.fx.enter(self.n, W.STATE_IDLE, 10.2)
        self.assertFalse(self.n.detach_abort_recovery)
        self.assertIsNone(self.n.detach_abort_recovery_q)

    def test_disabled_feature_preserves_legacy_detach_abort(self):
        self.n.detach_abort_route_enabled = False
        self.sample();self.fail()
        self.assertEqual(self.n.state, W.STATE_ABORT)

    def test_confirmed_pilot_exit_or_disarm_retires_bridge_without_abort(self):
        for flight in (State(connected=True, armed=False, mode='OFFBOARD'),
                       State(connected=True, armed=True, mode='POSCTL')):
            with self.subTest(mode=flight.mode, armed=flight.armed):
                self.setUp();self.sample();self.fail()
                with mock.patch.object(rospy.Time, 'now', return_value=fixture.at(10.2)), mock.patch.object(rospy, 'loginfo'):
                    self.n._on_state(flight)
                self.assertEqual(self.n.state, W.STATE_IDLE)
                self.assertFalse(self.n.detach_abort_recovery)
                self.assertFalse(self.n.detach_rotation_succeeded)

    def test_connection_loss_does_not_claim_pilot_has_taken_over(self):
        self.sample();self.fail()
        with mock.patch.object(rospy.Time, 'now', return_value=fixture.at(10.2)):
            self.n._on_state(State(connected=False, armed=True, mode='OFFBOARD'))
        self.assertEqual(self.n.state, W.STATE_RECOVER)
        self.assertTrue(self.n.detach_abort_recovery)


if __name__ == '__main__':
    unittest.main()
