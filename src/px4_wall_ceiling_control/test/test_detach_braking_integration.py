"""ROS fixture integration for normal-detach braking and original safety gates.

Published-output feedback is explicit; these are command/state-machine tests,
not a simulation of the aircraft or PX4's rate integral.
"""
import math
import unittest
from unittest import mock

from px4_wall_ceiling_control.domain.geometry import quaternion_angle
from px4_wall_ceiling_control.msg import WallPerchStatus as W
import test_detach_state_machine as fixtures


at = fixtures.at
quaternion = fixtures.quaternion


class DetachBrakingIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.fx = fixtures.DetachStateMachineTests()

    def node(self, **extra):
        params = {'~detach_brake_enabled': True}
        params.update(extra)
        node = self.fx.node(**params)
        self.fx.enter(node, W.STATE_DETACH_ROTATE, 10.)
        self.assertEqual(node.state, W.STATE_DETACH_ROTATE)
        return node

    def frame(self, node, seconds, sent, measured=54., rate=-54., source=None):
        self.fx.feedback(node, seconds, tilt=measured, rate=rate)
        self.fx.output(node, seconds, tilt=sent)
        if source is not None:
            node.horizontal_output.header.stamp = at(source)
        node._advance_detach_progress(at(seconds))

    def engage(self):
        node = self.node()
        # Three final publications establish a constant -45 deg/s reference.
        for seconds, sent in ((10.02, 31.8), (10.04, 30.9), (10.06, 30.)):
            self.frame(node, seconds, sent)
        self.assertIsNotNone(node.detach_brake_planner)
        return node

    @staticmethod
    def state(planner):
        return (planner.angle, planner.rate, planner.accel, planner.elapsed,
                planner.replan_count)

    def test_trigger_uses_final_published_target_instead_of_candidate(self):
        node = self.node()
        node.detach_elapsed_s = 2.2
        # Nominal candidate has crossed 30 deg, but final output has not.
        self.frame(node, 10.02, sent=40., measured=45.)
        candidate = node._detach_target_quaternion()
        self.assertLess(quaternion_angle(candidate, node.q_hover), math.radians(30.))
        self.assertIsNone(node.detach_brake_planner)
        self.assertIsNone(node.detach_brake_target)

    def test_engagement_inherits_exact_published_angle_and_reference_velocity(self):
        node = self.engage()
        planner = node.detach_brake_planner
        self.assertAlmostEqual(math.degrees(planner.angle), 30.)
        self.assertAlmostEqual(math.degrees(planner.rate), -45., delta=1.e-4)
        self.assertAlmostEqual(planner.accel, 0., delta=1.e-5)
        self.assertEqual(planner.elapsed, 0.)
        self.assertLess(quaternion_angle(node._detach_target_quaternion(),
                                         quaternion(30.)), 1.e-7)
        # Actual vehicle is moving at -54 deg/s; it must not replace the
        # derivative of the actual reference being spliced.
        self.assertNotAlmostEqual(planner.rate, node.odom.twist.twist.angular.y)

    def test_missing_publication_history_holds_target_without_inventing_velocity(self):
        node = self.node()
        self.frame(node, 10.02, sent=30.)
        self.assertIsNone(node.detach_brake_planner)
        self.assertLess(quaternion_angle(node.detach_brake_target, quaternion(30.)), 1.e-7)
        self.frame(node, 10.04, sent=30., source=10.02)
        self.assertEqual(len(node.detach_brake_history), 1)
        self.assertIsNone(node.detach_brake_planner)
        self.frame(node, 10.06, sent=29.1)
        self.assertIsNotNone(node.detach_brake_planner)
        self.assertAlmostEqual(math.degrees(node.detach_brake_planner.rate), -22.5, delta=1.e-4)

    def test_seed_at_reference_rate_cap_discards_outward_acceleration(self):
        node = self.node()
        for seconds, sent in ((10.02, 31.8), (10.04, 30.9), (10.06, 29.7)):
            self.frame(node, seconds, sent)
        planner = node.detach_brake_planner
        self.assertIsNotNone(planner)
        self.assertAlmostEqual(planner.rate, -node.detach_brake_max_rate)
        self.assertAlmostEqual(planner.accel, 0.)
        self.frame(node, 10.08, sent=29.7)
        self.assertEqual(node.detach_brake_fault, '')
        self.assertIsNotNone(planner.segment)
        self.assertTrue(planner.segment.within_bounds(0., planner.max_angle,
                                                     planner.max_rate, planner.max_accel))

    def test_same_now_candidate_does_not_advance_braking_twice(self):
        node = self.engage()
        self.frame(node, 10.08, sent=30.)
        before = self.state(node.detach_brake_planner)
        first = node._candidate(at(10.08))
        second = node._candidate(at(10.08))
        self.assertEqual(first[1], second[1])
        self.assertEqual(first[3], second[3])
        self.assertEqual(self.state(node.detach_brake_planner), before)

    def test_stale_final_output_freezes_then_resumes_without_time_catchup(self):
        node = self.engage()
        self.frame(node, 10.08, sent=30.)
        planner = node.detach_brake_planner
        before, target = self.state(planner), node.detach_brake_target
        self.fx.feedback(node, 10.40, tilt=54., rate=-54.)
        node._advance_detach_progress(at(10.40))
        self.assertTrue(node.detach_brake_paused)
        self.assertEqual(self.state(planner), before)
        self.assertEqual(node.detach_brake_target, target)
        self.assertLess(node._detach_progress(), 1.)
        self.frame(node, 10.42, sent=math.degrees(planner.angle))
        self.assertFalse(node.detach_brake_paused)
        self.assertAlmostEqual(node.detach_step_dt, .02)
        self.assertLessEqual(abs(planner.angle - before[0]),
                             node.detach_brake_max_rate * .02 + 1.e-9)

    def test_reversed_publication_source_freezes_braking_state(self):
        node = self.engage()
        before = self.state(node.detach_brake_planner)
        target = node.detach_brake_target
        self.frame(node, 10.08, sent=29.1, source=10.04)
        self.assertTrue(node.detach_brake_paused)
        self.assertEqual(self.state(node.detach_brake_planner), before)
        self.assertEqual(node.detach_brake_target, target)
        self.assertAlmostEqual(node.detach_brake_history[-1][0], 10.06)

    def test_new_detach_clears_braking_history_target_and_fault(self):
        node = self.engage()
        node.detach_brake_fault = 'old trajectory fault'
        self.fx.enter(node, W.STATE_WALL_HOLD, 10.5)
        self.fx.feedback(node, 10.6, tilt=90.)
        self.fx.output(node, 10.6, tilt=90.)
        self.fx.enter(node, W.STATE_DETACH_ROTATE, 10.6)
        self.assertEqual(node.state, W.STATE_DETACH_ROTATE)
        self.assertIsNone(node.detach_brake_planner)
        self.assertIsNone(node.detach_brake_target)
        self.assertEqual(node.detach_brake_history, [])
        self.assertEqual(node.detach_brake_fault, '')
        self.assertFalse(node.detach_brake_paused)

    def test_high_rate_near_level_blocks_horizontal_overlay_and_recover(self):
        node = self.engage()
        self.frame(node, 10.08, sent=30., measured=3., rate=-54.)
        self.assertFalse(node._detach_rotation_pose_valid())
        self.assertFalse(node._horizontal_phase_allowed(at(10.08)))
        self.assertFalse(node._detach_rotation_ready(at(10.08)))
        self.assertGreater(node.detach_brake_planner.goal, 0.)
        self.fx.tick(node, 10.08)
        self.assertEqual(node.state, W.STATE_DETACH_ROTATE)

    def test_zero_goal_finishes_then_original_measured_gate_allows_recover(self):
        node = self.engage()
        self.frame(node, 10.08, sent=30., measured=0., rate=0.)
        self.assertTrue(node._detach_rotation_pose_valid())
        self.assertFalse(node.detach_brake_planner.done)
        self.assertFalse(node._detach_rotation_ready(at(10.08)))
        self.assertFalse(node._horizontal_phase_allowed(at(10.08)))
        for index in range(1, 201):
            seconds = 10.08 + index * .02
            self.frame(node, seconds, sent=math.degrees(node.detach_brake_planner.angle),
                       measured=0., rate=0.)
            if node.detach_brake_planner.done:
                break
        self.assertTrue(node.detach_brake_planner.done)
        self.assertAlmostEqual(node._detach_progress(), 1.)
        self.assertAlmostEqual(node.detach_brake_planner.angle, 0., places=9)
        self.assertGreaterEqual(node.detach_recovered_samples, 2)
        self.fx.output(node, seconds, tilt=0.)
        self.assertTrue(node._horizontal_phase_allowed(at(seconds)))
        self.assertTrue(node._detach_rotation_ready(at(seconds)))
        self.fx.tick(node, seconds)
        self.assertEqual(node.state, W.STATE_RECOVER)

    def test_bench_keeps_original_timed_interpolation(self):
        node = self.node(**{'~bench_wait_for_ranges': True})
        node._height_hold_thrust = lambda: .6
        node._advance_detach_braking = mock.Mock(side_effect=AssertionError('bench uses original trajectory'))
        self.fx.feedback(node, 11.5, tilt=45.)
        candidate = node._candidate(at(11.5))
        self.assertLess(quaternion_angle(candidate[1], quaternion(45.)), 1.e-7)
        node._advance_detach_braking.assert_not_called()
        self.assertIsNone(node.detach_brake_planner)
        self.assertEqual(node.detach_brake_history, [])

    def test_abort_immediately_commands_hover_despite_active_braking_curve(self):
        node = self.engage()
        self.fx.feedback(node, 10.08, tilt=54., rate=-54.)
        planner = node.detach_brake_planner
        before = self.state(planner)
        self.fx.enter(node, W.STATE_ABORT, 10.08)
        candidate = node._candidate(at(10.08))
        self.assertEqual(node.state, W.STATE_ABORT)
        self.assertLess(quaternion_angle(candidate[1], node.q_hover), 1.e-7)
        self.assertEqual(self.state(planner), before)


if __name__ == '__main__':
    unittest.main()
