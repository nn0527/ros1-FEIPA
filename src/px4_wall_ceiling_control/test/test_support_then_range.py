"""No local-height dependency from range departure admission to handover."""
import math
import unittest
from unittest import mock

from px4_wall_ceiling_control.domain.departure_height import DepartureHeightController
import test_range_height_integration as fixtures
from px4_wall_ceiling_control.msg import AttachmentControlCandidate as C, WallPerchStatus as W


class AttitudeSupportTests(unittest.TestCase):
    def test_no_height_reference_or_feedback_even_after_full_vertical_authority(self):
        c = DepartureHeightController()
        c.begin_support(.50, .648, 10.)
        actual = .648
        for i in range(1, 201):
            actual = c.update_support(1., 10.+i*.02, actual)
            self.assertEqual(c.correction, 0.)
            self.assertEqual(c.integral, 0.)
            self.assertTrue(math.isnan(c.reference))
            self.assertTrue(math.isnan(c.target_speed))
        self.assertAlmostEqual(actual, .50)

    def test_vertical_wall_keeps_pressure_and_tilt_compensation_is_bounded(self):
        c = DepartureHeightController()
        c.begin_support(.50, .648, 10.)
        self.assertAlmostEqual(c.update_support(0., 10.02, .648), .648)
        actual = .648
        for i in range(2, 102):
            previous = actual
            actual = c.update_support(.7, 10.+i*.02, actual)
            self.assertLessEqual(abs(actual-previous), .010000001)
        self.assertAlmostEqual(actual, .50/.7)
        self.assertAlmostEqual(c.update_support(.7, 12.02, actual), actual)

    def test_range_begin_is_only_place_to_create_height_target(self):
        c = DepartureHeightController()
        c.begin_support(.50, .648, 10.)
        c.update_support(.8, 10.1, .648)
        c.begin(.90, c.base, c.value, 10.2)
        self.assertEqual(c.reference, .90)
        self.assertEqual(c.integral, 0.)


class SupportThenRangeNodeTests(unittest.TestCase):
    def setUp(self):
        self.fx = fixtures.RangeDetachNodeTests()
        self.fx.setUp()
        self.addCleanup(self.fx.doCleanups)
        self.n = self.fx.n

    def support_frame(self, height, vz, angle=20.):
        self.fx.seconds = 10.2
        self.fx.fx.feedback(self.n, 10.2, height=height, vz=vz, tilt=angle)
        self.fx.fx.output(self.n, 10.2, tilt=angle, thrust=.70)
        return self.n._detach_vertical_thrust(fixtures.at(10.2), .02)

    def test_nan_local_height_and_velocity_do_not_enter_support_calculation(self):
        with mock.patch.object(self.n, '_height_feedback', side_effect=AssertionError('local height read')), \
             mock.patch.object(self.n, '_snapshot_departure_feedback', side_effect=AssertionError('legacy snapshot')), \
             mock.patch.object(self.n.detach_height_continuation, 'observe', side_effect=AssertionError('reanchor')):
            value = self.support_frame(math.nan, math.nan)
        self.assertTrue(math.isfinite(value))
        self.assertEqual(self.n.detach_vertical_controller.correction, 0.)
        self.assertTrue(self.n._vertical_speed_safe())

    def test_support_output_independent_of_local_height_and_velocity(self):
        a = self.support_frame(-1e6, 100.)
        self.n = self.fx.new_node()
        b = self.support_frame(1e6, -100.)
        self.assertEqual(a, b)

    def test_admission_ignores_z_vz_and_vertical_estimator_flags(self):
        n = self.n
        n.horizontal_estimator.velocity_vert_status_flag = False
        n.horizontal_estimator.pos_vert_abs_status_flag = False
        self.support_frame(math.nan, math.nan, angle=90.)
        with mock.patch.object(n, '_height_feedback', side_effect=AssertionError('admission read Z')):
            self.assertTrue(n._detach_pose_valid())
            self.assertTrue(n._detach_vertical_feedback_ok(fixtures.at(10.2)))
            actual = n._detach_vertical_output(fixtures.at(10.2))
            self.assertTrue(n._begin_detach_vertical(fixtures.at(10.2), actual))
        self.assertIsNone(n.detach_altitude_m)
        self.assertTrue(math.isnan(n.detach_vertical_controller.reference))

    def test_support_does_not_claim_height_hold_or_normal_completion(self):
        self.support_frame(1e6, -100.)
        s = self.n._build_status(fixtures.at(10.2))
        self.assertFalse(s.vertical_reference_valid)
        self.assertFalse(s.vertical_control_active)
        self.assertTrue(math.isnan(s.vertical_reference_m))
        self.assertTrue(math.isnan(s.detach_height_error_m))
        self.assertTrue(s.recovery_support_valid)
        self.assertFalse(self.n._detach_height_settled())

    def test_range_lock_uses_only_range_and_never_reverts_to_local_z(self):
        self.fx.frame(10.25, distance=1.24, height=math.nan, vz=math.nan)
        self.assertTrue(self.n._range_locked())
        self.assertAlmostEqual(self.n.detach_vertical_controller.reference, 1.24)
        self.fx.frame(10.5, distance=1.20, height=1e6, vz=-100.)
        value = self.n._detach_vertical_thrust(fixtures.at(10.5), .02)
        self.assertTrue(math.isfinite(value))
        self.assertAlmostEqual(self.n.detach_vertical_controller.reference, 1.24)
        self.assertLess(self.n._height_feedback()[0], 2.)
        self.fx.seconds = 13.11
        self.n._prepare_range_feedback(fixtures.at(13.11))
        self.assertIsNone(self.n._height_feedback())
        self.assertTrue(self.n.detach_range_faulted)

    def test_attitude_and_owner_freshness_still_gate_support(self):
        self.support_frame(math.nan, math.nan)
        self.n.horizontal_output_owner_at = fixtures.at(9.)
        self.assertTrue(math.isnan(self.n._detach_vertical_thrust(fixtures.at(10.2), .02)))
        self.assertFalse(self.n.recovery_support_valid)

    def test_mux_accepts_support_without_local_z_and_checks_phase(self):
        f = fixtures.mux_fixtures.CandidateLatencyTest()
        f.setUp()
        self.addCleanup(f.doCleanups)
        m = f.node
        m.owner = m.OWNER_WALL
        c = C(source=C.SOURCE_WALL, kind=C.KIND_ATTITUDE_THRUST, valid=True,
              phase_valid=True, control_phase=W.STATE_DETACH_ROTATE,
              maneuver_id='support-test', control_sequence=1,
              departure_support_only=True, detach_task_phase=1, detach_range_episode=1)
        c.header.stamp = fixtures.at(10.)
        c.attitude_setpoint.w = 1.
        c.thrust_normalized = .65
        self.assertEqual(m._wall_candidate_problem(c, fixtures.at(10.)), '')
        m.wall_candidate_snapshot = (c, fixtures.at(10.))
        m._wall_fresh = lambda now: True
        m.odom_twist_in_child_frame = False
        m.odom.pose.pose.position.z = math.nan
        m.odom.twist.twist.linear.z = math.nan
        self.assertTrue(m._odom_fresh(fixtures.at(10.)))
        m.wall_maneuver_id = c.maneuver_id
        c.recovery_support_valid = True
        c.recovery_thrust_limit = .85
        f.seed_publication(angle=0., thrust=.65, age=.02)
        self.assertEqual(m._publish_candidate(fixtures.at(10.), c), m.OUTPUT_ATTITUDE_THRUST)
        sent = m.attitude_publisher.publish.call_args.args[0]
        self.assertAlmostEqual(sent.thrust, .65)
        self.assertAlmostEqual(fixtures.tilt(sent), 0.)
        self.assertEqual(sent.type_mask, 7)
        c.control_phase = W.STATE_SLOW_APPROACH
        self.assertEqual(m._wall_candidate_problem(c, fixtures.at(10.)), 'departure_support_payload_mismatch')


if __name__ == '__main__':
    unittest.main()
