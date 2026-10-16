#!/usr/bin/env python3
"""Post-detach RECOVER bridge at the real (mocked) publication boundary."""
import math
import unittest
from unittest import mock

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget
from px4_wall_ceiling_control.msg import AttachmentControlCandidate as C, WallPerchStatus as W
import test_candidate_latency as fixture
from test_detach_near_level_latch import q_rpy, q_tuple, steps_degrees


class DetachAbortMuxTest(unittest.TestCase):
    def setUp(self):
        self.fx = fixture.CandidateLatencyTest()
        self.fx.setUp()
        self.addCleanup(self.fx.doCleanups)
        self.n = self.fx.node
        self.fx.accept(self.fx.candidate())
        self.n.owner = fixture.Mux.OWNER_WALL
        self.n.output_enabled = True
        self.n.detach_abort_recovery_enabled = True
        self.sequence = 1
        self.fx.advance()
        self.measured(q_rpy(pitch=40.))

    def measured(self, q, source_age=0., receipt_age=0.):
        self.n.odom.pose.pose.orientation = Quaternion(*q)
        self.n.odom.header.stamp = self.fx.now-rospy.Duration.from_sec(source_age)
        self.n.odom_at = self.fx.now-rospy.Duration.from_sec(receipt_age)
        self.n.flight_state_at = self.fx.now

    def seed(self, q, thrust=.75, age=.02, maneuver='run-a', source=C.SOURCE_WALL):
        target = AttitudeTarget(orientation=Quaternion(*q), thrust=thrust)
        self.n._remember_publication(target, source, self.fx.now-rospy.Duration.from_sec(age),
                                    maneuver_id=maneuver)

    def command(self, q=None, phase=W.STATE_RECOVER, marked=True, accept=True, thrust=.82):
        self.sequence += 1
        candidate = self.fx.candidate(phase=phase, sequence=self.sequence, acquire=False)
        candidate.attitude_setpoint = Quaternion(*tuple(q or q_rpy()))
        candidate.thrust_normalized = thrust
        candidate.recovery_support_valid = True
        candidate.recovery_thrust_limit = .85
        candidate.detach_abort_recovery = marked
        if accept:
            self.fx.accept(candidate)
        return candidate

    def publish(self, candidate):
        self.assertEqual(self.n._publish_candidate(self.fx.now, candidate), fixture.Mux.OUTPUT_ATTITUDE_THRUST,
                         self.n.wall_candidate_rejection)
        return self.n.attitude_publisher.publish.call_args.args[0]

    def test_bridge_keeps_outer_cone_target_and_real_publication_history(self):
        before = q_rpy(pitch=50.)
        self.seed(before)
        target = self.publish(self.command(q_rpy(pitch=40.)))
        self.assertAlmostEqual(fixture.tilt(target), 49.8, places=6)
        self.assertAlmostEqual(target.thrust, .76)
        diagnostic = self.n.last_wall_command['publication_slew']
        self.assertEqual(diagnostic['rate_mode'], 'detach_abort_recovery')
        self.assertTrue(diagnostic['detach_abort_recovery'])
        self.assertFalse(diagnostic['abort_level_immediate'])

    def test_bridge_does_not_project_an_unchanged_large_axis_to_five_degrees(self):
        self.seed(q_rpy(pitch=40.))
        target = self.publish(self.command(q_rpy(pitch=40.)))
        self.assertAlmostEqual(fixture.tilt(target), 40., places=6)

    def test_axis_heading_and_thrust_remain_bounded_across_multiple_publications(self):
        before = q_rpy(roll=15., pitch=50., yaw=179.)
        self.seed(before)
        thrust = .75
        for _ in range(8):
            target = self.publish(self.command(q_rpy(yaw=-170.), thrust=.4))
            axis_step, heading_step = steps_degrees(before, q_tuple(target))
            self.assertLessEqual(axis_step, .200001)
            self.assertLessEqual(abs(heading_step), .400001)
            self.assertLessEqual(abs(target.thrust-thrust), .010001)
            before, thrust = q_tuple(target), target.thrust
            self.fx.advance()
            self.measured(q_rpy(pitch=40.))

    def test_bridge_rates_cannot_exceed_ten_twenty_or_point_five(self):
        self.n.recovery_tilt_slew_deg_s = self.n.recovery_yaw_slew_deg_s = 90.
        self.n.wall_return_thrust_slew_per_s = 2.
        before = q_rpy(pitch=40.)
        self.seed(before)
        target = self.publish(self.command(q_rpy(yaw=90.), thrust=.4))
        axis_step, heading_step = steps_degrees(before, q_tuple(target))
        self.assertLessEqual(axis_step, .200001)
        self.assertLessEqual(abs(heading_step), .400001)
        self.assertAlmostEqual(target.thrust, .74)

    def test_unmarked_recover_and_exit_keep_existing_outer_cone_rejection(self):
        for phase in (W.STATE_RECOVER, W.STATE_EXIT):
            with self.subTest(phase=phase):
                self.seed(q_rpy(pitch=40.))
                candidate = self.command(phase=phase, marked=False)
                self.assertEqual(self.n._publish_candidate(self.fx.now, candidate), fixture.Mux.OUTPUT_NONE)
                self.assertEqual(self.n.wall_candidate_rejection, 'recovery_entry_published_axis_outside_cone')
                self.fx.advance()

    def test_unmarked_abort_still_levels_immediately(self):
        self.seed(q_rpy(pitch=40.))
        target = self.publish(self.command(q_rpy(pitch=20.), phase=W.STATE_ABORT, marked=False))
        self.assertLessEqual(fixture.tilt(target), 5.000001)
        self.assertEqual(self.n.last_wall_command['publication_slew']['rate_mode'], 'abort_immediate')

    def test_marker_requires_feature_and_recover_phase(self):
        self.n.detach_abort_recovery_enabled = False
        candidate = self.command(accept=False)
        self.assertEqual(self.n._wall_candidate_problem(candidate, self.fx.now),
                         'invalid_detach_abort_recovery_marker')
        self.n.detach_abort_recovery_enabled = True
        for phase in (W.STATE_DETACH_ROTATE, W.STATE_ABORT, W.STATE_EXIT, W.STATE_WALL_HOLD):
            candidate = self.command(phase=phase, accept=False)
            self.assertEqual(self.n._wall_candidate_problem(candidate, self.fx.now),
                             'invalid_detach_abort_recovery_marker')
        candidate.source = C.SOURCE_CEILING
        self.assertFalse(self.n._detach_abort_recovery_active(candidate))
        target = AttitudeTarget(orientation=Quaternion(*q_rpy(pitch=30.)), thrust=.7)
        unchanged = self.n._limit_final_recovery_target(self.fx.now, target, C.SOURCE_CEILING, candidate)
        self.assertAlmostEqual(fixture.tilt(unchanged), 30.)

    def test_old_stale_or_prestream_history_cannot_be_replaced_by_measurement(self):
        for age, maneuver, source in ((.6, 'run-a', C.SOURCE_WALL),
                                      (.02, 'old-run', C.SOURCE_WALL),
                                      (.02, 'run-a', C.SOURCE_NONE)):
            with self.subTest(age=age, maneuver=maneuver, source=source):
                self.seed(q_rpy(pitch=40.), age=age, maneuver=maneuver, source=source)
                echo = AttitudeTarget(orientation=Quaternion(*q_rpy(pitch=5.)), thrust=.70)
                echo.header.stamp = self.fx.now
                self.n.wall_continuity_fcu_snapshot = (echo, self.fx.now)
                candidate = self.command()
                history = self.n.last_published_control
                self.assertEqual(self.n._publish_candidate(self.fx.now, candidate), fixture.Mux.OUTPUT_NONE)
                self.assertEqual(self.n.wall_candidate_rejection, 'detach_abort_recovery_history_stale')
                self.assertIs(self.n.last_published_control, history)
                self.fx.advance()
                self.measured(q_rpy(pitch=40.))

    def test_stale_measured_pose_cannot_admit_bridge_output(self):
        for source_age, receipt_age in ((.6, 0.), (0., .6)):
            self.seed(q_rpy(pitch=40.))
            self.measured(q_rpy(pitch=40.), source_age, receipt_age)
            self.assertEqual(self.n._publish_candidate(self.fx.now, self.command()), fixture.Mux.OUTPUT_NONE)
            self.assertEqual(self.n.wall_candidate_rejection, 'detach_abort_recovery_feedback_invalid')
            self.fx.advance()

    def test_authority_lost_before_final_publish_does_not_commit_output(self):
        self.seed(q_rpy(pitch=40.))
        history = self.n.last_published_control
        with mock.patch.object(self.n, '_near_level_publication_authorized', side_effect=[True, False]):
            self.assertEqual(self.n._publish_candidate(self.fx.now, self.command()), fixture.Mux.OUTPUT_NONE)
        self.assertEqual(self.n.wall_candidate_rejection, 'detach_abort_recovery_invalid_before_publication')
        self.n.attitude_publisher.publish.assert_not_called()
        self.assertIs(self.n.last_published_control, history)

    def test_expired_candidate_does_not_commit_bridge_history(self):
        self.seed(q_rpy(pitch=40.))
        history = self.n.last_published_control
        original = self.n._limit_wall_publication
        def delayed(*args):
            target = original(*args)
            self.fx.advance(.21)
            return target
        with mock.patch.object(self.n, '_limit_wall_publication', side_effect=delayed):
            self.assertEqual(self.n._publish_candidate(self.fx.now, self.command()), fixture.Mux.OUTPUT_NONE)
        self.assertEqual(self.n.wall_candidate_rejection, 'candidate_expired_before_publication')
        self.n.attitude_publisher.publish.assert_not_called()
        self.assertIs(self.n.last_published_control, history)

    def test_revoked_marker_cannot_publish_previously_bypassed_cone(self):
        self.seed(q_rpy(pitch=40.))
        candidate = self.command(q_rpy(pitch=30.))
        original = self.n._limit_wall_publication
        def revoke(*args):
            target = original(*args)
            candidate.detach_abort_recovery = False
            return target
        with mock.patch.object(self.n, '_limit_wall_publication', side_effect=revoke):
            self.assertEqual(self.n._publish_candidate(self.fx.now, candidate), fixture.Mux.OUTPUT_NONE)
        self.assertEqual(self.n.wall_candidate_rejection, 'detach_abort_recovery_invalid_before_publication')
        self.n.attitude_publisher.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
