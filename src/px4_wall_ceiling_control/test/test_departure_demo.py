"""Command-to-level demo contract; ROS is mocked and no flight nodes are started."""
import json
import math
import unittest
from unittest import mock

from std_msgs.msg import Bool
from px4_wall_ceiling_control.domain.departure_height import DepartureHeightController
from px4_wall_ceiling_control.domain.range_height_feedback import RangeHeightFeedback, RangeHeightConfig
from px4_wall_ceiling_control.adapters.wall_horizontal_velocity import WallHorizontalVelocityMixin
from px4_wall_ceiling_control.msg import WallPerchStatus as W
import test_range_height_integration as fixtures
from test_detach_state_machine import quaternion, at


class RangePDTests(unittest.TestCase):
    def test_pd_terms_and_no_integral_or_hidden_velocity_target(self):
        c = DepartureHeightController()
        c.begin(1., .5, .5, 10.)
        c.update_range(.8, -.3, 1., 10.02, .5, .15, .1)
        self.assertAlmostEqual(c.correction, .06)
        self.assertAlmostEqual(c.requested, .56)
        self.assertAlmostEqual(c.value, .51)
        self.assertEqual(c.integral, 0.)
        self.assertEqual(c.target_speed, 0.)
        c.update_range(1.2, None, 1., 10.04, .51, .15, .1)
        self.assertAlmostEqual(c.correction, -.03)
        self.assertAlmostEqual(c.requested, .47)
        self.assertTrue(math.isnan(c.target_speed))
        self.assertEqual(c.reference, 1.)

    def test_twenty_and_fifty_hz_range_build_velocity(self):
        from dataclasses import replace
        for dt in (.02, .05):
            r = RangeHeightFeedback(replace(RangeHeightConfig(), min_distance_m=.02,
                                           max_distance_m=6., geometry_confirmed=True))
            r.begin(1, 10.)
            for i in range(1, 30):
                t = 10.+i*dt
                r.observe_pose(t, t, quaternion(0.))
                r.observe_range(t, t, 1.+.1*(t-10.))
            self.assertTrue(r.read_snapshot(t).fast_velocity_valid)
            self.assertAlmostEqual(r.fast_velocity, .1)


class DepartureDemoTests(unittest.TestCase):
    def setUp(self):
        self.fx = fixtures.RangeDetachNodeTests()
        self.fx.setUp()
        self.addCleanup(self.fx.doCleanups)
        self.n = self.fx.n

    def test_request_starts_without_stability_window_or_horizontal_reference(self):
        n = self.n
        self.fx.fx.enter(n, W.STATE_WALL_HOLD, 10.01)
        self.fx.seconds = 10.02
        self.fx.fx.feedback(n, 10.02, tilt=72., rate=30., height=math.nan, vz=math.nan)
        self.fx.fx.output(n, 10.02, tilt=72.)
        n.horizontal_equilibrium.ready = n.horizontal_equilibrium.frozen = False
        n.horizontal_takeover_yaw = None
        n.detach_stable_since = None
        with mock.patch.object(n, '_height_feedback', side_effect=AssertionError('local height used')):
            n._on_detach(Bool(data=True))
            n._update_state_machine(at(10.02))
        self.assertEqual(n.state, W.STATE_DETACH_ROTATE)
        self.assertAlmostEqual(n.detach_range.started, 10.02)
        self.assertTrue(math.isnan(n.detach_vertical_controller.reference))

    def test_timed_trajectory_does_not_pause_for_lag_or_replan_braking(self):
        n = self.n
        n.detach_brake_enabled = True
        with mock.patch.object(n, '_advance_detach_braking', side_effect=AssertionError('adaptive brake')):
            for i in range(1, 151):
                t = 10.+i*.02
                self.fx.seconds = t
                self.fx.fx.feedback(n, t, tilt=90., height=math.nan, vz=math.nan)
                self.fx.fx.output(n, t, tilt=90.)
                n._advance_detach_progress(at(t))
        self.assertAlmostEqual(n._detach_progress(), 1.)
        self.assertFalse(n.detach_tracking_limited)
        self.assertIsNone(n.detach_brake_planner)
        self.assertAlmostEqual(n._detach_target_quaternion()[3], 1.)

    def test_horizontal_correction_cannot_modify_rotation_even_near_level(self):
        n = self.n
        n.horizontal_velocity_enabled = True
        with mock.patch.object(n, '_prepare_horizontal_velocity', side_effect=AssertionError('horizontal control')):
            q = quaternion(3.)
            self.assertEqual(WallHorizontalVelocityMixin._horizontal_attitude(n, at(10.), q), q)
        self.assertFalse(n._horizontal_phase_allowed(at(10.)))

    def test_rotation_safety_uses_authority_attitude_and_severe_rate_only(self):
        n = self.n
        self.fx.seconds = 10.2
        self.fx.fx.feedback(n, 10.2, tilt=30., height=math.nan, vz=math.nan)
        with mock.patch.object(n, '_top_preflight_ready', side_effect=AssertionError('top range gate')), \
             mock.patch.object(n, '_vertical_speed_safe', side_effect=AssertionError('descent gate')):
            self.assertTrue(type(n)._safety_ok(n, at(10.2)))
            n.odom.twist.twist.angular.y = n.recover_max_rate_rps+.1
            self.assertFalse(type(n)._safety_ok(n, at(10.2)))
        self.assertFalse(type(n)._safety_ok(n, at(12.)))

    def test_command_to_level_stability_does_not_wait_for_range_or_flow(self):
        n = self.n
        n._horizontal_recovery_ready = lambda now: False
        n._departure_clearance_ready = lambda now: False
        with mock.patch.object(n, '_height_feedback', side_effect=AssertionError('local height used')):
            for i in range(1, 181):
                t = 10.+i*.02
                self.fx.seconds = t
                angle = max(0., 90.*(1.-(t-10.)/3.))
                self.fx.fx.feedback(n, t, tilt=angle, rate=0. if angle == 0. else 30.,
                                    height=math.nan, vz=math.nan)
                self.fx.fx.output(n, t, tilt=angle)
                n._update_state_machine(at(t))
                if n.state == W.STATE_RECOVER:
                    break
        self.assertEqual(n.state, W.STATE_RECOVER)
        self.assertFalse(n._range_locked())
        self.assertIsNotNone(n.detach_level_stable_at)
        self.assertLessEqual(n.detach_level_stable_at, 13.36)
        self.assertFalse(n.detach_completed)  # Full handover is a later outcome.
        status = n._build_status(at(self.fx.seconds))
        candidate = n._build_control_candidate(status, n.maneuver_id, 1)
        self.assertTrue(candidate.departure_support_only)
        self.assertEqual(candidate.detach_task_phase, 1)
        mux_fx = fixtures.mux_fixtures.CandidateLatencyTest()
        mux_fx.setUp()
        self.addCleanup(mux_fx.doCleanups)
        self.assertEqual(mux_fx.node._wall_candidate_problem(candidate, at(self.fx.seconds)), '')

    def test_oscillation_across_old_fifteen_degree_gate_keeps_control(self):
        n = self.n
        self.fx.frame(10.2, distance=1.)
        target = n.detach_range.target
        for i in range(1, 61):
            t = 10.2+i*.1
            angle = 18. if i % 2 else 12.
            self.fx.frame(t, angle=angle, distance=1./math.cos(math.radians(angle)),
                          height=math.nan, vz=math.nan)
            value = n._detach_vertical_thrust(at(t), .02)
            self.assertTrue(math.isfinite(value))
            self.assertFalse(n.detach_range_faulted)
            self.assertTrue(n.detach_range.read_snapshot(t).fast_velocity_valid)
        self.assertEqual(n.detach_range.target, target)
        self.assertAlmostEqual(n.detach_range.height, 1.)

    def test_missing_velocity_uses_height_p_without_point_three_second_fault(self):
        n = self.n
        self.fx.frame(10.2, distance=1.)
        for i in range(1, 16):
            t = 10.2+i*.1
            self.fx.frame(t, distance=.9)
            n.detach_range._invalidate_velocity(t, 'synthetic unavailable')
            value = n._detach_vertical_thrust(at(t), .02)
            self.assertTrue(math.isfinite(value))
            self.assertGreater(n.detach_vertical_controller.correction, 0.)
            self.assertFalse(n.detach_fast_velocity_fault)
            self.assertFalse(n.detach_range_faulted)

    def test_one_rejected_frame_does_not_reset_good_velocity(self):
        n = self.n
        self.fx.frame(10.2, distance=1.)
        self.fx.frame(10.3, distance=1.)
        self.assertTrue(n.detach_range.fast_velocity_valid)
        self.fx.frame(10.4, distance=-1.)
        self.assertTrue(n.detach_range.read_snapshot(10.4).fast_velocity_valid)
        self.assertEqual(n.detach_range.source, 10.3)
        self.assertFalse(n.detach_range_faulted)

    def test_real_height_loss_recovers_and_only_prolonged_loss_faults(self):
        n = self.n
        self.fx.frame(10.2, distance=1.)
        self.fx.frame(10.3, distance=1.)
        target = n.detach_range.target
        self.fx.seconds = 11.6
        self.fx.fx.feedback(n, 11.6, tilt=0., height=math.nan, vz=math.nan)
        self.fx.fx.output(n, 11.6, tilt=0., thrust=.55)
        self.assertTrue(math.isfinite(n._detach_vertical_thrust(at(11.6), .02)))
        self.assertFalse(n.detach_range_faulted)
        self.fx.frame(11.7, distance=1.1)
        self.assertEqual(n.detach_range.target, target)
        self.assertFalse(n.detach_range_faulted)
        self.fx.seconds = 14.31
        n._prepare_range_feedback(at(14.31))
        self.assertTrue(n.detach_range_faulted)
        self.assertIn('测距失效', n.detach_vertical_fault)

    def test_fifteen_second_budget_does_not_interrupt_live_control(self):
        n = self.n
        self.fx.frame(10.2, distance=1.)
        self.fx.prepare_stability()
        n._range_px4_ready = lambda now: False
        for i in range(1, 101):
            t = 10.2+i*.2
            self.fx.frame(t, distance=1.)
            n._prepare_range_feedback(at(t))
            self.assertFalse(n.detach_range_faulted)
        self.assertEqual(n.detach_range.target, 1.)

    def test_trace_separates_candidate_actual_and_terms_and_is_rate_limited(self):
        n = self.n
        self.fx.frame(10.2, distance=1.)
        self.fx.frame(10.3, distance=.98)
        status = n._build_status(at(10.3))
        n._publish_departure_trace(at(10.3), status)
        row = json.loads(n.departure_trace_publisher.publish.call_args.args[0].data)
        self.assertEqual(row['version'], 'departure_demo_v1')
        self.assertEqual(row['mode'], 'RANGE_PD')
        self.assertAlmostEqual(row['height_p'], .15*row['range_error_m'])
        self.assertAlmostEqual(row['velocity_d'], -.18*row['range_velocity_mps'])
        self.assertEqual(row['sent_source_time_s'], 10.3)
        self.assertIn('candidate_q', row)
        self.assertIn('sent_q', row)
        count = n.departure_trace_publisher.publish.call_count
        n._publish_departure_trace(at(10.32), status)
        self.assertEqual(n.departure_trace_publisher.publish.call_count, count)


if __name__ == '__main__':
    unittest.main()
