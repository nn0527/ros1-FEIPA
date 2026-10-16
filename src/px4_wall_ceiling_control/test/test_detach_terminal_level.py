#!/usr/bin/env python3
"""Final-output terminal deceleration; deterministic tests send no vehicle commands.

Compose the established publication fixture instead of inheriting its test
methods: this suite must not silently rerun unrelated near-level tests.
"""
import math
import unittest
from unittest import mock

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget, State
from px4_wall_ceiling_control.msg import AttachmentControlCandidate as Candidate, WallPerchStatus as Wall
import test_candidate_latency as fixture
import test_detach_near_level_latch as near


def signed_pitch_axis(q):
    axis, _ = near.axis_heading(q)
    return math.degrees(math.atan2(axis[0], axis[2]))


class DetachTerminalLevelTest(unittest.TestCase):
    def setUp(self):
        self.h = near.DetachNearLevelLatchTest()
        self.h.setUp()
        self.addCleanup(self.h.doCleanups)
        self.fx, self.n = self.h.fx, self.h.n
        self.n.detach_terminal_enabled = True
        self.n.detach_terminal_start_deg = 30.
        self.n.detach_terminal_axis_rate_deg_s = 10.
        self.n.detach_terminal_yaw_rate_deg_s = 20.
        self.n.detach_terminal_response_per_s = 2.
        self.n.wall_terminal_maneuver = None

    def seed(self, *args, **kwargs):
        return self.h.seed(*args, **kwargs)

    def measured(self, *args, **kwargs):
        return self.h.measured(*args, **kwargs)

    def command(self, *args, **kwargs):
        kwargs.setdefault('horizontal', False)
        return self.h.command(*args, **kwargs)

    def publish(self, candidate):
        return self.h.publish(candidate)

    def assert_bounded(self, before, target, dt=.02):
        axis, heading = near.steps_degrees(before, near.q_tuple(target))
        self.assertLessEqual(axis, 10.*dt+1e-6)
        self.assertLessEqual(abs(heading), 20.*dt+1e-6)
        return axis, heading

    def latch(self, previous_pitch=25., requested_pitch=0.):
        self.seed(near.q_rpy(pitch=previous_pitch))
        self.measured(near.q_rpy(pitch=60.))
        target = self.publish(self.command(near.q_rpy(pitch=requested_pitch)))
        self.assertEqual(self.n.wall_terminal_maneuver, 'run-a')
        self.assertEqual(self.n.last_wall_command['publication_slew']['rate_mode'], 'detach_terminal')
        return target

    def test_first_terminal_candidate_slows_before_hloop_or_measured_level(self):
        before = near.q_rpy(pitch=40., yaw=10.)
        self.seed(before)
        self.measured(near.q_rpy(pitch=70.))
        target = self.publish(self.command(near.q_rpy(pitch=29., yaw=100.), thrust=.70))
        axis, heading = self.assert_bounded(before, target)
        self.assertAlmostEqual(axis, .2, places=6)
        self.assertAlmostEqual(heading, .4, places=6)
        self.assertGreater(fixture.tilt(target), 39., 'previous output must not be projected into a cone')
        self.assertAlmostEqual(target.thrust, .53)
        self.assertEqual(self.n.wall_terminal_maneuver, 'run-a')
        self.assertIsNone(getattr(self.n, 'wall_near_level_maneuver', None))
        self.assertEqual(self.n.last_wall_command['publication_slew']['rate_mode'], 'detach_terminal')

    def test_previous_published_axis_can_establish_entry_even_if_new_target_outside(self):
        before = near.q_rpy(pitch=29.)
        self.seed(before)
        self.measured(near.q_rpy(pitch=65.))
        target = self.publish(self.command(near.q_rpy(pitch=45.)))
        self.assert_bounded(before, target)
        self.assertGreater(signed_pitch_axis(near.q_tuple(target)), 29.)
        self.assertEqual(self.n.wall_terminal_maneuver, 'run-a')

    def test_no_entry_while_both_raw_target_and_history_outside_terminal_cone(self):
        before = near.q_rpy(pitch=45.)
        self.seed(before)
        self.measured()
        target = self.publish(self.command(near.q_rpy(pitch=40.)))
        axis, _ = near.steps_degrees(before, near.q_tuple(target))
        self.assertAlmostEqual(axis, 56.25*.02, places=6)
        self.assertIsNone(self.n.wall_terminal_maneuver)

    def test_legacy_fixture_without_explicit_enable_preserves_existing_rate(self):
        del self.n.detach_terminal_enabled
        before = near.q_rpy(pitch=25.)
        self.seed(before)
        self.measured()
        target = self.publish(self.command(near.q_rpy()))
        axis, _ = near.steps_degrees(before, near.q_tuple(target))
        self.assertAlmostEqual(axis, 56.25*.02, places=6)
        self.assertIsNone(self.n.wall_terminal_maneuver)

    def test_terminal_entry_threshold_boundary(self):
        for requested, expected in ((30.001, False), (30., True)):
            with self.subTest(requested=requested):
                self.n._reset_wall_publication_history()
                self.fx.advance()
                self.seed(near.q_rpy(pitch=40.))
                self.measured(near.q_rpy(pitch=60.))
                self.publish(self.command(near.q_rpy(pitch=requested)))
                self.assertEqual(self.n.wall_terminal_maneuver == 'run-a', expected)

    def test_near_level_tail_slows_monotonically_without_crossing_level(self):
        previous = near.q_rpy(pitch=2.)
        self.seed(previous)
        self.measured(near.q_rpy(pitch=20.))
        previous_pitch, previous_step = 2., float('inf')
        for index in range(150):
            if index:
                self.fx.advance()
                self.measured(near.q_rpy(pitch=20.))
            target = self.publish(self.command(near.q_rpy()))
            step, _ = self.assert_bounded(previous, target)
            pitch = signed_pitch_axis(near.q_tuple(target))
            expected_step = previous_pitch*(1.-math.exp(-2.*.02))
            self.assertAlmostEqual(step, expected_step, delta=2e-6)
            self.assertGreater(pitch, 0., 'level target must not be crossed by the output limiter')
            self.assertLess(pitch, previous_pitch)
            self.assertLessEqual(step, previous_step+2e-6)
            previous_pitch, previous_step, previous = pitch, step, near.q_tuple(target)
        self.assertLess(previous_pitch, .01)

    def test_terminal_taper_also_applies_when_near_level_latch_takes_priority(self):
        target = self.latch(previous_pitch=4.)
        self.fx.advance()
        self.measured()
        before = near.q_tuple(target)
        target = self.publish(self.command(near.q_rpy(), horizontal=True))
        step, _ = self.assert_bounded(before, target)
        expected = signed_pitch_axis(before)*(1.-math.exp(-2.*.02))
        self.assertAlmostEqual(step, expected, places=6)
        self.assertLess(step, .2)
        self.assertEqual(self.n.wall_terminal_maneuver, 'run-a')
        self.assertEqual(self.n.wall_near_level_maneuver, 'run-a')
        self.assertEqual(self.n.last_wall_command['publication_slew']['rate_mode'], 'detach_near_level')

    def test_hloop_toggle_and_measured_excursion_keep_terminal_deceleration(self):
        target = self.latch()
        for horizontal, measured_pitch in ((True, 70.), (False, 5.), (True, 45.), (False, 2.)):
            self.fx.advance()
            self.measured(near.q_rpy(pitch=measured_pitch))
            before = near.q_tuple(target)
            target = self.publish(self.command(near.q_rpy(pitch=40., yaw=100.), horizontal=horizontal))
            self.assert_bounded(before, target)
            self.assertEqual(self.n.wall_terminal_maneuver, 'run-a')

    def test_stale_feedback_cannot_establish_terminal_latch(self):
        for source_age, receipt_age in ((.6, 0.), (0., .6)):
            with self.subTest(source_age=source_age, receipt_age=receipt_age):
                self.n._reset_wall_publication_history()
                self.fx.advance()
                self.seed(near.q_rpy(pitch=25.))
                self.measured(source_age=source_age, receipt_age=receipt_age)
                self.n._publish_candidate(self.fx.now, self.command())
                self.assertIsNone(self.n.wall_terminal_maneuver)

    def test_old_action_or_prestream_history_cannot_establish_terminal_latch(self):
        for maneuver, source in (('old-run', Candidate.SOURCE_WALL), ('run-a', Candidate.SOURCE_NONE)):
            with self.subTest(maneuver=maneuver, source=source):
                self.n._reset_wall_publication_history()
                self.fx.advance()
                self.seed(near.q_rpy(pitch=25.), maneuver=maneuver, source=source)
                self.measured()
                self.n._publish_candidate(self.fx.now, self.command())
                self.assertIsNone(self.n.wall_terminal_maneuver)

    def test_measurement_fallback_cannot_establish_terminal_latch(self):
        self.measured(near.q_rpy(pitch=25.))
        self.n.last_published_control = None
        self.n.last_published_maneuver_id = None
        echo = AttitudeTarget(orientation=Quaternion(*near.q_rpy()), thrust=.52)
        echo.header.stamp = self.fx.now
        self.n._on_fcu_attitude_target(echo)
        self.publish(self.command())
        self.assertIsNone(self.n.wall_terminal_maneuver)
        self.fx.advance()
        self.measured()
        self.publish(self.command())
        self.assertEqual(self.n.wall_terminal_maneuver, 'run-a')

    def test_expired_publish_history_cannot_establish_terminal_latch(self):
        self.seed(near.q_rpy(pitch=25.), age=.6)
        self.measured()
        self.n._publish_candidate(self.fx.now, self.command())
        self.assertIsNone(self.n.wall_terminal_maneuver)

    def test_candidate_expiry_before_send_does_not_commit_latch_or_history(self):
        self.seed(near.q_rpy(pitch=25.))
        self.measured()
        before = self.n.last_published_control
        candidate = self.command()
        original = self.n._limit_wall_publication
        def expire(*args, **kwargs):
            result = original(*args, **kwargs)
            self.fx.advance(.21)
            return result
        with mock.patch.object(self.n, '_limit_wall_publication', side_effect=expire):
            self.assertEqual(self.n._publish_candidate(self.fx.now, candidate), fixture.Mux.OUTPUT_NONE)
        self.assertIsNone(self.n.wall_terminal_maneuver)
        self.assertIs(self.n.last_published_control, before)
        self.n.attitude_publisher.publish.assert_not_called()

    def test_feedback_expiry_before_send_does_not_commit_latch_or_history(self):
        self.seed(near.q_rpy(pitch=25.))
        self.measured(source_age=.49)
        before = self.n.last_published_control
        candidate = self.command()
        original = self.n._limit_wall_publication
        def expire(*args, **kwargs):
            result = original(*args, **kwargs)
            self.fx.advance(.02)
            return result
        with mock.patch.object(self.n, '_limit_wall_publication', side_effect=expire):
            self.assertEqual(self.n._publish_candidate(self.fx.now, candidate), fixture.Mux.OUTPUT_NONE)
        self.assertIsNone(self.n.wall_terminal_maneuver)
        self.assertIs(self.n.last_published_control, before)
        self.n.attitude_publisher.publish.assert_not_called()

    def test_publisher_failure_cannot_commit_terminal_latch(self):
        self.seed(near.q_rpy(pitch=25.))
        self.measured()
        before = self.n.last_published_control
        self.n.attitude_publisher.publish.side_effect = RuntimeError('simulated transport failure')
        with self.assertRaises(RuntimeError):
            self.n._publish_candidate(self.fx.now, self.command())
        self.assertIsNone(self.n.wall_terminal_maneuver)
        self.assertIs(self.n.last_published_control, before)

    def test_revoked_permission_cannot_commit_first_terminal_latch(self):
        for permission in ('kill_active', 'kill_invalid', 'owner', 'output_disabled', 'wall_output_disabled', 'rc'):
            with self.subTest(permission=permission):
                self.n._reset_wall_publication_history()
                self.fx.advance()
                self.n.owner = fixture.Mux.OWNER_WALL
                self.n.output_enabled = self.n.wall_output_enabled = True
                self.n.kill_guard_enabled = False
                self.seed(near.q_rpy(pitch=25.))
                self.measured()
                before = self.n.last_published_control
                count = self.n.attitude_publisher.publish.call_count
                candidate = self.command()
                original = self.n._limit_wall_publication
                with mock.patch.object(self.n, '_rc_ready', return_value=True) as rc_ready:
                    def revoke(*args, **kwargs):
                        result = original(*args, **kwargs)
                        if permission.startswith('kill_'):
                            self.n.kill_guard_enabled = True
                            self.n.kill_status_at = self.fx.now
                            self.n.kill_status = {'valid': permission != 'kill_invalid',
                                                  'active': permission == 'kill_active'}
                        elif permission == 'owner':
                            self.n.owner = fixture.Mux.OWNER_NONE
                        elif permission == 'output_disabled':
                            self.n.output_enabled = False
                        elif permission == 'wall_output_disabled':
                            self.n.wall_output_enabled = False
                        elif permission == 'rc':
                            rc_ready.return_value = False
                        return result
                    with mock.patch.object(self.n, '_limit_wall_publication', side_effect=revoke):
                        self.assertEqual(self.n._publish_candidate(self.fx.now, candidate), fixture.Mux.OUTPUT_NONE)
                self.assertEqual(self.n.attitude_publisher.publish.call_count, count)
                self.assertIs(self.n.last_published_control, before)
                self.assertIsNone(self.n.wall_terminal_maneuver)

    def test_stale_feedback_then_restoration_preserves_terminal_latch(self):
        target = self.latch()
        self.fx.advance()
        self.measured(source_age=.6)
        count = self.n.attitude_publisher.publish.call_count
        self.assertEqual(self.n._publish_candidate(self.fx.now, self.command()), fixture.Mux.OUTPUT_NONE)
        self.assertEqual(self.n.attitude_publisher.publish.call_count, count)
        self.assertEqual(self.n.wall_terminal_maneuver, 'run-a')

        self.fx.advance()
        self.measured()
        next_target = self.publish(self.command())
        self.assert_bounded(near.q_tuple(target), next_target, .04)
        self.assertEqual(self.n.wall_terminal_maneuver, 'run-a')

    def test_latched_mode_rejects_expired_history_instead_of_measurement_reset(self):
        self.latch()
        self.fx.advance(.60)
        self.measured(near.q_rpy(pitch=25.))
        echo = AttitudeTarget(orientation=Quaternion(*near.q_rpy()), thrust=.52)
        echo.header.stamp = self.fx.now
        self.n._on_fcu_attitude_target(echo)
        count = self.n.attitude_publisher.publish.call_count
        self.assertEqual(self.n._publish_candidate(self.fx.now, self.command()), fixture.Mux.OUTPUT_NONE)
        self.assertEqual(self.n.attitude_publisher.publish.call_count, count)
        self.assertEqual(self.n.wall_terminal_maneuver, 'run-a')
        self.assertEqual(self.n.wall_candidate_rejection, 'terminal_level_publication_history_stale')

    def test_heading_wrap_uses_short_signed_step_in_terminal_mode(self):
        before = near.q_rpy(pitch=25., yaw=179.8)
        self.seed(before)
        self.measured()
        target = self.publish(self.command(near.q_rpy(pitch=0., yaw=-179.)))
        _, heading = self.assert_bounded(before, target)
        self.assertAlmostEqual(heading, .4, places=6)
        self.assertEqual(self.n.wall_terminal_maneuver, 'run-a')

    def test_recover_continues_real_terminal_publication_without_history_reset(self):
        target = self.latch(previous_pitch=4.)
        for phase in (Wall.STATE_DETACH_ROTATE, Wall.STATE_RECOVER, Wall.STATE_RECOVER):
            self.fx.advance()
            self.measured()
            self.n.recovery_previous_target = near.q_rpy(roll=-5., yaw=-120.)
            self.n.recovery_previous_at = self.fx.now
            before, thrust = near.q_tuple(target), target.thrust
            target = self.publish(self.command(near.q_rpy(roll=-4., yaw=-90.), phase=phase, thrust=.59))
            self.assert_bounded(before, target)
            self.assertLessEqual(abs(target.thrust-thrust), .010001)
            self.assertEqual(self.n.wall_terminal_maneuver, 'run-a')

    def test_abort_keeps_immediate_level_output_and_clears_terminal_latch_on_send(self):
        self.latch()
        self.seed(near.q_rpy(pitch=70.), thrust=.8)
        self.fx.advance()
        self.measured(near.q_rpy(pitch=70.))
        target = self.publish(self.command(near.q_rpy(pitch=60.), phase=Wall.STATE_ABORT, thrust=.49))
        self.assertLessEqual(fixture.tilt(target), 5.000001)
        self.assertAlmostEqual(target.thrust, .78)
        self.assertTrue(self.n.last_wall_command['publication_slew']['abort_level_immediate'])
        self.assertIsNone(self.n.wall_terminal_maneuver)

    def test_failed_abort_publish_does_not_clear_terminal_latch(self):
        self.latch()
        self.fx.advance()
        self.measured()
        before = self.n.last_published_control
        self.n.attitude_publisher.publish.side_effect = RuntimeError('simulated transport failure')
        with self.assertRaises(RuntimeError):
            self.n._publish_candidate(self.fx.now, self.command(phase=Wall.STATE_ABORT))
        self.assertEqual(self.n.wall_terminal_maneuver, 'run-a')
        self.assertIs(self.n.last_published_control, before)

    def test_reset_and_offboard_exit_clear_terminal_latch(self):
        self.latch()
        self.n._reset_wall_publication_history()
        self.assertIsNone(self.n.wall_terminal_maneuver)
        self.assertIsNone(self.n.last_published_control)
        self.fx.advance()
        self.latch()
        self.n._on_state(State(connected=True, armed=True, mode='POSCTL'))
        self.assertIsNone(self.n.wall_terminal_maneuver)
        self.assertIsNone(self.n.last_published_control)


if __name__ == '__main__':
    unittest.main()
