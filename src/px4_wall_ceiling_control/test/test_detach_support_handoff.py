"""Regression contracts for log61's wall-support and range-hold handoffs.

The numerical tests run without ROS. The adapter cases use message fixtures
without launching flight nodes, and are skipped when rospy is unavailable.
Recorded-input replay is a command replay, not a simulated flight outcome.
"""
import json
import math
import os
from pathlib import Path
import sys
import unittest
from dataclasses import replace
from unittest import mock

from px4_wall_ceiling_control.domain.departure_height import DepartureHeightController
from px4_wall_ceiling_control.domain.range_height_feedback import RangeHeightConfig

try:
    import rospy
except ModuleNotFoundError as error:
    if error.name != 'rospy':
        raise
    ROS_AVAILABLE = False
else:
    ROS_AVAILABLE = True
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    # Keep TestCase classes under the imported module so unittest does not
    # discover every old fixture test again through this module's globals.
    import test_range_height_integration as range_fixtures
    from px4_wall_ceiling_control.msg import WallPerchStatus as W


def recorded_fixture():
    return json.loads((Path(__file__).parent/'fixtures'/'detach_support_log61.json').read_text())


def replay_odom_prefix(fixture):
    """Fixed recorded motion, with each new command echoed as next actual.

    This ideal immediate output model compares command policies. It does not
    recalculate vehicle motion or prove a changed physical flight outcome.
    """
    initial = fixture['initial']
    epoch = fixture['metadata']['relative_epoch_unix_s']
    c = DepartureHeightController(height_kp=.3, velocity_kp=.3)
    actual = initial['actual_thrust']
    c.begin(initial['height'], initial['base'], actual, initial['t']-epoch)
    outputs = []
    for row in fixture['ticks']:
        if row['t'] <= initial['t']:
            continue
        if row['range']['target_locked']:
            break
        actual = c.update(row['height'], row['velocity'], row['vertical_fraction'],
                          row['relative_s'], row['odom_source']-epoch, actual)
        outputs.append((row['relative_s'], actual))
    return c, actual, outputs


class DepartureSupportHandoffTests(unittest.TestCase):
    def controller(self):
        c = DepartureHeightController(velocity_kp=.30)
        c.begin(-5.78, .515, .720, 28.5)
        return c

    def test_wall_driven_climb_tracks_height_without_storing_negative_correction(self):
        c = self.controller()
        c.integral = -.04
        actual = .720
        for index in range(1, 111):
            now = 28.5 + index*.02
            height = -5.78 + .45*(now-28.5)
            actual = c.update(height, .45, 0., now, now, actual)
            self.assertAlmostEqual(actual, .720)
            self.assertEqual(c.reference, height)
            self.assertEqual(c.target_speed, 0.)
            self.assertEqual(c.correction, 0.)
            self.assertEqual(c.integral, 0.)
            self.assertFalse(c.integral_active)
            self.assertTrue(c.reference_tracking)
            self.assertEqual(c.reason, 'wall support required at current tilt')
        actual = c.update(height+.009, .45, 1., 30.72, 30.72, actual)
        self.assertTrue(c.reference_tracking)
        self.assertAlmostEqual(c.correction, 0.)
        self.assertAlmostEqual(c.requested, .515)
        self.assertGreaterEqual(actual, .710-1.e-9)

    def test_partial_support_uses_neutral_authority_before_error_correction(self):
        c = self.controller()
        # At vf=.55 the old negative correction lowered demand enough to
        # incorrectly finish the support blend. Neutral base/.85 is > .55.
        c.update(-4.38, .45, .55, 28.52, 28.52, .720)
        self.assertGreater(c.support_weight, 0.)
        self.assertLess(c.support_weight, 1.)
        self.assertEqual(c.reference, -4.38)
        self.assertEqual(c.correction, 0.)
        self.assertEqual(c.target_speed, 0.)
        self.assertTrue(c.reference_tracking)
        self.assertGreater(c.requested, .720)

    def test_full_support_handoff_is_neutral_then_normal_feedback_resumes(self):
        c = self.controller()
        c.update(-5.771, .45, .4, 28.52, 28.52, .720)
        actual = c.update(-5.762, .45, 1., 28.54, 28.54, c.value)
        self.assertEqual(c.support_weight, 1.)
        self.assertEqual(c.reference, -5.762)
        self.assertTrue(c.reference_tracking)
        self.assertEqual(c.correction, 0.)
        self.assertEqual(c.target_speed, 0.)
        self.assertAlmostEqual(c.requested, c.base)
        c.update(-5.753, .45, 1., 28.56, 28.56, actual)
        self.assertFalse(c.reference_tracking)
        self.assertEqual(c.reference, -5.762)
        self.assertLess(c.target_speed, 0.)
        self.assertLess(c.correction, 0.)

    def test_direct_level_initialization_preserves_height_error_feedback(self):
        c = self.controller()
        self.assertFalse(c.reference_tracking)
        c.update(-5.88, -.2, 1., 28.52, 28.52, .720)
        self.assertEqual(c.reference, -5.78)
        self.assertFalse(c.reference_tracking)
        self.assertGreater(c.target_speed, 0.)
        self.assertGreater(c.correction, 0.)

    def test_return_to_partial_support_clears_preexisting_feedback(self):
        c = self.controller()
        c.update(-5.78, 0., 1., 28.52, 28.52, .515)
        c.integral = .04
        c.update(-5.48, -.2, .4, 28.54, 28.54, c.value)
        self.assertTrue(c.reference_tracking)
        self.assertEqual(c.reference, -5.48)
        self.assertEqual(c.integral, 0.)
        self.assertEqual(c.correction, 0.)
        self.assertEqual(c.target_speed, 0.)

    def test_support_tracking_keeps_slew_and_duplicate_tick_contract(self):
        c = self.controller()
        first = c.update(-5., .45, .4, 29., 29., .720)
        self.assertLessEqual(abs(first-.720), .025+1.e-9)
        self.assertEqual(c.update(-4., -.5, 1., 29., 29., first), first)
        self.assertEqual(c.reference, -5.)
        with self.assertRaises(ValueError):
            c.update(-5., .45, .4, 28.99, 28.99, first)

    def test_recorded_odom_prefix_avoids_low_thrust_before_range_lock(self):
        fixture = recorded_fixture()
        _, _, outputs = replay_odom_prefix(fixture)
        window = [value for seconds, value in outputs if seconds >= 31.]
        self.assertGreater(len(window), 30)
        self.assertGreaterEqual(min(window), .50)
        self.assertLessEqual(max(window), .85)
        self.assertTrue(any(row['recorded']['actual_thrust'] < .45
                            for row in fixture['ticks'] if 31. <= row['relative_s'] <= 32.5))


@unittest.skipUnless(ROS_AVAILABLE, 'requires ROS message classes and rospy')
class RangeSupportHandoffTests(unittest.TestCase):
    def setUp(self):
        self.f = range_fixtures.RangeDetachNodeTests()
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.n = self.f.n

    def seed_hold(self, actual=.442, angle=0., seconds=10.27, invalid=False, base=.515):
        self.f.frame(10.25, angle=angle)
        n = self.n
        n.detach_vertical_controller.base = base
        self.f.seconds = seconds
        self.f.fx.feedback(n, seconds, tilt=angle)
        self.f.fx.output(n, seconds, tilt=angle, thrust=actual)
        if invalid:
            snapshot = replace(n.detach_range.read_snapshot(seconds), valid=False)
            patch = mock.patch.object(n.detach_range, 'read_snapshot', return_value=snapshot)
            patch.start()
            self.addCleanup(patch.stop)
            n.detach_range.invalid_since = 10.25
        return n.detach_vertical_controller

    def thrust(self, seconds=None):
        return self.n._range_vertical_thrust(range_fixtures.at(
            self.f.seconds if seconds is None else seconds))

    def test_first_range_without_velocity_raises_low_actual_toward_hover(self):
        c = self.seed_hold()
        c.integral = -.04
        reference = c.reference
        value = self.thrust()
        self.assertAlmostEqual(value, .452, delta=2.e-9)
        self.assertAlmostEqual(c.requested, .515)
        self.assertEqual(c.reference, reference)
        self.assertEqual(c.integral, 0.)
        self.assertEqual(c.correction, 0.)
        self.assertTrue(math.isnan(c.target_speed))
        self.assertFalse(self.n.detach_range.velocity_valid)
        self.assertFalse(self.n.detach_range.fast_velocity_valid)
        self.assertTrue(c.slew_limited)
        self.assertTrue(self.n.recovery_support_valid)
        self.assertIn('range P', self.n.detach_vertical_reason)

    def test_short_invalid_range_raises_low_actual_without_reanchoring(self):
        c = self.seed_hold(invalid=True)
        reference = c.reference
        value = self.thrust()
        self.assertAlmostEqual(value, .452, delta=2.e-9)
        self.assertAlmostEqual(c.requested, .515)
        self.assertEqual(c.reference, reference)
        self.assertEqual(c.correction, 0.)
        self.assertTrue(math.isnan(c.target_speed))
        self.assertTrue(self.n.recovery_support_valid)

    def test_hold_preserves_actual_above_compensated_hover(self):
        c = self.seed_hold(actual=.61, angle=9.)
        self.assertAlmostEqual(self.thrust(), .60)
        self.assertAlmostEqual(c.requested, .515/math.cos(math.radians(9.)))
        self.assertTrue(c.slew_limited)

    def test_hold_compensates_measured_tilt(self):
        c = self.seed_hold(actual=.520, angle=9.)
        expected = .515/math.cos(math.radians(9.))
        self.assertAlmostEqual(self.thrust(), expected)
        self.assertAlmostEqual(c.requested, expected)
        self.assertFalse(c.slew_limited)

    def test_hold_caps_hover_request_at_maximum_thrust(self):
        c = self.seed_hold(actual=.840, angle=9., base=.845)
        self.assertGreater(c.base/math.cos(math.radians(9.)), c.max_thrust)
        self.assertGreater(self.thrust(), .84)
        self.assertLessEqual(c.value, c.max_thrust)
        self.assertTrue(c.saturated)

    def test_hold_caps_elapsed_slew_and_does_not_step_twice_at_same_time(self):
        self.seed_hold(seconds=10.45)
        first = self.thrust()
        self.assertAlmostEqual(first, .467, delta=2.e-9)
        self.f.fx.output(self.n, 10.45, tilt=0., thrust=first)
        self.assertEqual(self.thrust(), first)
        self.f.seconds = 10.47
        self.f.fx.output(self.n, 10.47, tilt=0., thrust=first)
        self.assertAlmostEqual(self.thrust(), first+.010, delta=2.e-9)

    def test_hold_still_requires_fresh_owned_offboard_output(self):
        self.seed_hold()
        n = self.n
        for name, obj, attribute, value in (
                ('offboard', n.flight_state, 'mode', 'POSCTL'),
                ('armed', n.flight_state, 'armed', False),
                ('connected', n.flight_state, 'connected', False),
                ('owner', n, 'horizontal_output_owner', 0),
                ('state freshness', n, 'flight_state_received_at', range_fixtures.at(8.)),
                ('output freshness', n, 'horizontal_output_at', range_fixtures.at(9.))):
            with self.subTest(gate=name), mock.patch.object(obj, attribute, value):
                self.assertTrue(math.isnan(self.thrust()))
                self.assertFalse(n.recovery_support_valid)

    def test_fresh_height_without_velocity_keeps_p_control(self):
        self.seed_hold()
        self.assertTrue(math.isfinite(self.thrust()))
        self.f.seconds = 10.56
        self.f.fx.feedback(self.n, self.f.seconds, tilt=0.)
        self.f.fx.output(self.n, self.f.seconds, tilt=0., thrust=.515)
        self.assertTrue(math.isfinite(self.thrust()))
        self.assertFalse(self.n.detach_fast_velocity_fault)
        self.assertTrue(self.n.recovery_support_valid)

    def test_short_invalid_range_support_timeout_is_not_extended(self):
        self.seed_hold(seconds=12.26, invalid=True)
        self.assertTrue(math.isnan(self.thrust()))
        self.assertFalse(self.n.recovery_support_valid)

    def test_range_target_remains_fixed_when_high_base_needs_partial_support(self):
        c = self.seed_hold(actual=.84, angle=9., base=.84)
        reference = c.reference
        n = self.n
        self.f.seconds = 10.28  # A new source frame accepts the changed tilt.
        self.f.fx.feedback(n, self.f.seconds, tilt=15.)
        self.f.fx.output(n, self.f.seconds, tilt=15., thrust=.84)
        snapshot = replace(n.detach_range.read_snapshot(self.f.seconds),
            height=reference+.005, velocity=0., velocity_valid=True,
            fast_velocity=0., fast_velocity_valid=True, fast_velocity_age=.01,
            fast_velocity_source=self.f.seconds-.01)
        with mock.patch.object(n.detach_range, 'read_snapshot', return_value=snapshot):
            self.assertTrue(math.isfinite(self.thrust()))
        self.assertLess(c.support_weight, 1.)
        self.assertEqual(c.reference, reference)
        self.assertFalse(c.reference_tracking)
        self.assertLess(c.correction, 0.)

    def test_recorded_handoff_replay_keeps_command_above_point_five(self):
        fixture = recorded_fixture()
        epoch = fixture['metadata']['relative_epoch_unix_s']
        c, actual, outputs = replay_odom_prefix(fixture)
        n = self.n
        self.f.frame(10.25)  # Establish the existing range task/message fixture.
        r = n.detach_range
        r.config = RangeHeightConfig(**fixture['range_config'])
        template = r.read_snapshot(10.25)
        n.detach_vertical_controller = c
        n._reset_range_control_velocity()
        r.started = fixture['initial']['t']-epoch
        r.velocity_ready_at = None
        first = True
        held_without_velocity = fast_feedback_samples = 0
        observed_unsafe_descent = False

        def finite_or_nan(value):
            return math.nan if value is None else value

        for row in fixture['ticks']:
            recorded = row['range']
            if not recorded['target_locked']:
                continue
            now = range_fixtures.at(row['relative_s'])
            seconds = now.to_sec()  # Use the same nanosecond clock as the adapter.
            self.f.seconds = seconds
            angle = math.degrees(math.acos(max(-1., min(1., row['vertical_fraction']))))
            self.f.fx.feedback(n, seconds, tilt=angle, height=row['height'],
                               vz=row['velocity'], source=row['odom_source']-epoch)
            self.f.fx.output(n, seconds, tilt=angle, thrust=actual)
            if first:
                # Equivalent atomic controller seed at the first recorded lock;
                # retain the newly replayed actual, not log61's old low output.
                r.target = recorded['target_m']
                r.lock_time = r.action_started = seconds
                r.lock_source = recorded['lock_source_time']-epoch
                r.lock_received = recorded['lock_received_time']-epoch
                c.begin(r.target, fixture['initial']['base'], actual, seconds)
                n.detach_range_synced = True
                n.detach_fast_invalid_since = seconds
                first = False
            if recorded['velocity_valid'] and r.velocity_ready_at is None:
                r.velocity_ready_at = seconds
            source = recorded['source_time']-epoch
            snapshot = replace(template, target=recorded['target_m'],
                height=finite_or_nan(recorded['height_m']),
                velocity=finite_or_nan(recorded['velocity_mps']),
                velocity_valid=recorded['velocity_valid'], valid=recorded['valid'],
                source=source, received=recorded['received_time']-epoch,
                raw_distance=finite_or_nan(recorded['raw_m']),
                projected_height=finite_or_nan(recorded['projected_m']),
                sample_tilt_deg=finite_or_nan(recorded['sample_tilt_deg']),
                age=seconds-source, reason=recorded['block_reason'],
                fast_velocity=finite_or_nan(recorded['fast_velocity_mps']),
                fast_velocity_valid=recorded['fast_velocity_valid'],
                fast_velocity_source=source if recorded['fast_velocity_valid'] else 0.,
                fast_velocity_age=finite_or_nan(recorded['fast_velocity_age_s']))
            previous = actual
            with mock.patch.object(r, 'read_snapshot', return_value=snapshot):
                actual = n._range_vertical_thrust(now)
                # Recorded motion is unchanged: the original dangerous descent
                # must still fail its safety gate despite the new command.
                if snapshot.fast_velocity_valid and snapshot.fast_velocity < -.8:
                    self.assertFalse(n._vertical_speed_safe())
                    observed_unsafe_descent = True
            self.assertTrue(math.isfinite(actual), row['relative_s'])
            self.assertLessEqual(abs(actual-previous), .025+1.e-9)
            self.assertLessEqual(actual, c.max_thrust)
            self.assertEqual(c.reference, r.target)
            outputs.append((row['relative_s'], actual))
            if snapshot.fast_velocity_valid:
                fast_feedback_samples += 1
            else:
                held_without_velocity += 1
                self.assertTrue(math.isnan(c.target_speed))

        window = [value for seconds, value in outputs if 31. <= seconds <= 32.5]
        self.assertGreater(len(window), 60)
        self.assertGreaterEqual(min(window), .50)
        self.assertGreater(held_without_velocity, 0)
        self.assertGreater(fast_feedback_samples, 0)
        self.assertTrue(observed_unsafe_descent)


if __name__ == '__main__':
    unittest.main()
