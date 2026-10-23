"""Numerical contracts for measured-rate detach braking and smooth replanning.

These tests validate the commanded trajectory. They do not model contact,
PX4 integral state, or prove that the vehicle can produce the assumed braking.
"""
import math
import unittest

from px4_wall_ceiling_control.domain.detach_braking import (
    BrakingTrajectory,
    QuinticSegment,
    ReturnAxis,
    allowed_rate,
    braking_goal,
)
from px4_wall_ceiling_control.domain.geometry import (
    hover_and_pitched_quaternions,
    quaternion_multiply,
)


DEG = math.pi / 180.0


class StoppingEnvelopeTests(unittest.TestCase):
    def test_allowed_rate_accounts_for_delay_and_angle_margin(self):
        remaining, decel, delay, margin = 20 * DEG, 60 * DEG, .15, 3 * DEG
        speed = allowed_rate(remaining, decel, delay, margin, 80 * DEG)
        # A command on the envelope can consume at most the available angle.
        consumed = speed * delay + speed * speed / (2 * decel)
        self.assertAlmostEqual(consumed, remaining - margin, places=10)
        self.assertLess(speed, allowed_rate(remaining, decel, 0., margin, 80 * DEG))

    def test_envelope_reaches_zero_inside_margin_and_respects_rate_cap(self):
        self.assertEqual(allowed_rate(2 * DEG, 60 * DEG, .1, 3 * DEG, 40 * DEG), 0.)
        self.assertEqual(allowed_rate(3 * DEG, 60 * DEG, .1, 3 * DEG, 40 * DEG), 0.)
        self.assertAlmostEqual(allowed_rate(90 * DEG, 60 * DEG, .1, 3 * DEG,
                                           20 * DEG), 20 * DEG)

    def test_envelope_falls_as_level_approaches(self):
        speeds = [allowed_rate(angle * DEG, 60 * DEG, .15, 3 * DEG, 60 * DEG)
                  for angle in (60, 40, 20, 10, 5, 3)]
        self.assertTrue(all(a >= b for a, b in zip(speeds, speeds[1:])))

    def test_high_measured_rate_retracts_target_and_can_request_reverse_error(self):
        common = dict(remaining=20 * DEG, allowed=20 * DEG,
                      attitude_gain=4., rate_damping=1.,
                      max_lead=15 * DEG, max_angle=90 * DEG)
        slow = braking_goal(closing_rate=10 * DEG, **common)
        fast = braking_goal(closing_rate=60 * DEG, **common)
        self.assertLess(slow, common['remaining'])
        self.assertGreater(fast, common['remaining'])
        self.assertGreater(fast, slow)

    def test_braking_target_respects_lead_and_attitude_limits(self):
        for remaining in (0., 2 * DEG, 45 * DEG, 89 * DEG, 90 * DEG):
            for closing in (-180 * DEG, 0., 180 * DEG):
                goal = braking_goal(remaining, closing, 30 * DEG, 4., 1.,
                                    10 * DEG, 90 * DEG)
                self.assertGreaterEqual(goal, 0.)
                self.assertLessEqual(goal, 90 * DEG)
                self.assertLessEqual(abs(goal - remaining), 10 * DEG + 1.e-12)


class QuinticSegmentTests(unittest.TestCase):
    def test_log64_outward_seed_acceleration_is_continuously_captured(self):
        p = BrakingTrajectory(29.55*DEG, -50.90*DEG, -126.64*DEG)
        p.step(.02, 44.8*DEG)
        self.assertTrue(p.capture)
        self.assertAlmostEqual(p.segment.evaluate(0.)[2], -126.64*DEG)
        self.assertLess(abs(p.accel), 126.64*DEG)
        for _ in range(150):
            p.step(.02, 44.8*DEG)
            self.assertTrue(p.segment.within_bounds(0., p.max_angle, p.max_rate, p.max_accel))
        self.assertFalse(p.capture)
        self.assertAlmostEqual(p.angle, 44.8*DEG)
        self.assertAlmostEqual(p.rate, 0.)

    def test_connects_position_velocity_acceleration_to_rest_endpoint(self):
        segment = QuinticSegment(30 * DEG, -45 * DEG, 20 * DEG, 44 * DEG, 1.5)
        for actual, expected in zip(segment.evaluate(0.),
                                    (30 * DEG, -45 * DEG, 20 * DEG)):
            self.assertAlmostEqual(actual, expected, places=10)
        for actual, expected in zip(segment.evaluate(1.5), (44 * DEG, 0., 0.)):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_bound_check_accepts_rest_to_rest_curve_inside_all_limits(self):
        segment = QuinticSegment(0., 0., 0., 1., 1.)
        self.assertTrue(segment.within_bounds(0., 1., 2., 6.))
        for index in range(1001):
            angle, rate, accel = segment.evaluate(index / 1000.)
            self.assertGreaterEqual(angle, -1.e-10)
            self.assertLessEqual(angle, 1. + 1.e-10)
            self.assertLessEqual(abs(rate), 2.)
            self.assertLessEqual(abs(accel), 6.)

    def test_bound_check_catches_interior_rate_and_acceleration_peaks(self):
        segment = QuinticSegment(0., 0., 0., 1., 1.)
        # Endpoints are at rest, so endpoint-only bounds would miss both.
        self.assertFalse(segment.within_bounds(0., 1., 1.5, 6.))
        self.assertFalse(segment.within_bounds(0., 1., 2., 5.))

    def test_bound_check_catches_interior_position_excursion(self):
        segment = QuinticSegment(.25, 1., 0., .25, 1.)
        self.assertFalse(segment.within_bounds(0., .30, 2., 20.))


class BrakingTrajectoryTests(unittest.TestCase):
    def assert_bounded(self, trajectory):
        self.assertGreaterEqual(trajectory.angle, -1.e-9)
        self.assertLessEqual(trajectory.angle, 90 * DEG + 1.e-9)
        self.assertLessEqual(abs(trajectory.rate), 56.25 * DEG + 1.e-9)
        self.assertLessEqual(abs(trajectory.accel), 180 * DEG + 1.e-9)

    def test_retract_from_log64_like_state_without_target_jump(self):
        trajectory = BrakingTrajectory(30 * DEG, rate=-45 * DEG)
        previous = trajectory.angle
        seen_reverse = False
        for _ in range(1500):
            angle = trajectory.step(.01, 44 * DEG)
            self.assertAlmostEqual(angle, trajectory.angle)
            self.assertLessEqual(abs(angle - previous), 56.25 * DEG * .01 + 1.e-9)
            self.assert_bounded(trajectory)
            seen_reverse = seen_reverse or trajectory.rate > 1 * DEG
            previous = angle
        self.assertTrue(seen_reverse)
        self.assertGreaterEqual(trajectory.elapsed, trajectory.duration)
        self.assertFalse(trajectory.done)  # Only the final horizontal goal is done.
        self.assertAlmostEqual(trajectory.angle, 44 * DEG, places=9)
        self.assertAlmostEqual(trajectory.rate, 0., places=9)
        self.assertAlmostEqual(trajectory.accel, 0., places=9)

    def test_completes_at_level_with_zero_endpoint_rate_and_acceleration(self):
        trajectory = BrakingTrajectory(30 * DEG, rate=-20 * DEG)
        for _ in range(1000):
            trajectory.step(.01, 0.)
            self.assert_bounded(trajectory)
        self.assertTrue(trajectory.done)
        self.assertAlmostEqual(trajectory.angle, 0., places=9)
        self.assertAlmostEqual(trajectory.rate, 0., places=9)
        self.assertAlmostEqual(trajectory.accel, 0., places=9)

    def test_changed_goal_preserves_position_velocity_and_acceleration(self):
        trajectory = BrakingTrajectory(30 * DEG, rate=-20 * DEG)
        for _ in range(20):
            trajectory.step(.01, 20 * DEG)
        before = (trajectory.angle, trajectory.rate, trajectory.accel)
        # After the replan interval, a new goal must reconnect to current P/V/A.
        trajectory.step(1.e-7, 40 * DEG)
        after = (trajectory.angle, trajectory.rate, trajectory.accel)
        self.assertAlmostEqual(trajectory.goal, 40 * DEG)
        self.assertLess(abs(after[0] - before[0]), 1.e-6)
        self.assertLess(abs(after[1] - before[1]), 1.e-6)
        self.assertLess(abs(after[2] - before[2]), 1.e-4)
        for _ in range(1000):
            trajectory.step(.01, 40 * DEG)
            self.assert_bounded(trajectory)
        self.assertGreaterEqual(trajectory.elapsed, trajectory.duration)
        self.assertAlmostEqual(trajectory.angle, 40 * DEG, places=9)

    def test_changing_measured_goal_stays_bounded_then_settles(self):
        trajectory = BrakingTrajectory(30 * DEG, rate=-30 * DEG)
        previous = trajectory.angle
        for index in range(400):
            goal = (32. + 5. * math.sin(index * .02)) * DEG
            trajectory.step(.01, goal)
            self.assertLessEqual(abs(trajectory.angle - previous),
                                 56.25 * DEG * .01 + 1.e-9)
            self.assert_bounded(trajectory)
            previous = trajectory.angle
        for _ in range(1000):
            trajectory.step(.01, 0.)
            self.assert_bounded(trajectory)
        self.assertTrue(trajectory.done)
        self.assertAlmostEqual(trajectory.angle, 0., places=9)

    def test_zero_dt_preserves_output_and_long_dt_is_capped(self):
        normal = BrakingTrajectory(30 * DEG, rate=-20 * DEG)
        delayed = BrakingTrajectory(30 * DEG, rate=-20 * DEG)
        normal.step(.01, 0.)
        delayed.step(.01, 0.)
        before = (normal.angle, normal.rate, normal.accel)
        normal.step(0., 0.)
        self.assertEqual((normal.angle, normal.rate, normal.accel), before)
        normal.step(.05, 0.)
        delayed.step(1., 0.)
        for left, right in zip((normal.angle, normal.rate, normal.accel),
                               (delayed.angle, delayed.rate, delayed.accel)):
            self.assertAlmostEqual(left, right, places=10)

    def test_infeasible_initial_splice_fails_instead_of_jumping_target(self):
        trajectory = BrakingTrajectory(1 * DEG, rate=-45 * DEG)
        # At the configured acceleration limit there is insufficient angle
        # to stop this already moving reference before it crosses zero.
        with self.assertRaises(RuntimeError):
            trajectory.step(.01, 0.)
        self.assertAlmostEqual(trajectory.angle, 1 * DEG)
        self.assertAlmostEqual(trajectory.rate, -45 * DEG)

    def test_infeasible_replan_retains_previous_bounded_segment(self):
        trajectory = BrakingTrajectory(45 * DEG, max_duration=.5)
        for _ in range(20):
            trajectory.step(.01, 44 * DEG)
        segment = trajectory.segment
        previous = trajectory.angle
        trajectory.step(.01, 0.)
        self.assertTrue(trajectory.replan_rejected)
        self.assertIs(trajectory.segment, segment)
        self.assertAlmostEqual(trajectory.goal, 44 * DEG)
        self.assertLessEqual(abs(trajectory.angle - previous), 56.25 * DEG * .01)
        self.assert_bounded(trajectory)

    def test_finished_segment_accepts_small_goal_change_near_recovery_cone(self):
        # A deadband intended to suppress noise during an active curve must
        # not leave a finished reference stranded just outside the 5 deg gate.
        for angle in (5.01, 5.1, 5.24):
            with self.subTest(angle=angle):
                trajectory = BrakingTrajectory(angle * DEG)
                for _ in range(100):
                    trajectory.step(.01, angle * DEG)
                self.assertGreaterEqual(trajectory.elapsed, trajectory.duration)
                previous_replans = trajectory.replan_count
                goal = (angle - .1) * DEG
                for _ in range(100):
                    trajectory.step(.01, goal)
                self.assertGreater(trajectory.replan_count, previous_replans)
                self.assertAlmostEqual(trajectory.angle, goal, places=9)
                self.assertAlmostEqual(trajectory.rate, 0., places=9)
                self.assertAlmostEqual(trajectory.accel, 0., places=9)


class ReturnAxisTests(unittest.TestCase):
    def test_signed_return_coordinate_stays_continuous_at_ninety_degrees(self):
        for wall_pitch in (90 * DEG, -90 * DEG):
            hover, seed = hover_and_pitched_quaternions(73 * DEG, wall_pitch)
            axis = ReturnAxis(hover, seed)
            for angle in (-5 * DEG, 0., 30 * DEG, 89.9 * DEG, 90 * DEG):
                self.assertAlmostEqual(axis.angle(axis.quaternion(angle)), angle)
            self.assertAlmostEqual(axis.angle(seed), 90 * DEG)

    def test_body_rate_sign_and_quaternion_sign_are_invariant_to_heading(self):
        for yaw in (0., 73 * DEG, -150 * DEG):
            for wall_pitch in (90 * DEG, -90 * DEG):
                hover, seed = hover_and_pitched_quaternions(yaw, wall_pitch)
                axis = ReturnAxis(hover, seed)
                q = axis.quaternion(30 * DEG)
                rate = (0., -math.copysign(.6, wall_pitch), 0.)
                self.assertAlmostEqual(axis.closing_rate(q, rate), .6)
                opposite = tuple(-value for value in q)
                self.assertAlmostEqual(axis.angle(opposite), axis.angle(q))
                self.assertAlmostEqual(axis.closing_rate(opposite, rate), .6)

    def test_off_axis_closing_rate_matches_actual_quaternion_motion(self):
        hover, seed = hover_and_pitched_quaternions(73 * DEG, 90 * DEG)
        axis = ReturnAxis(hover, seed)
        roll = (math.sin(15 * DEG / 2.), 0., 0., math.cos(15 * DEG / 2.))
        q = quaternion_multiply(axis.quaternion(30 * DEG), roll)
        rate = (.2, -.6, .1)
        norm = math.sqrt(sum(value * value for value in rate))
        dt = 1.e-6
        increment = tuple(value / norm * math.sin(norm * dt / 2.)
                          for value in rate) + (math.cos(norm * dt / 2.),)
        next_q = quaternion_multiply(q, increment)
        finite_difference = -(axis.angle(next_q) - axis.angle(q)) / dt
        self.assertAlmostEqual(axis.closing_rate(q, rate), finite_difference, places=6)


if __name__ == '__main__':
    unittest.main()
