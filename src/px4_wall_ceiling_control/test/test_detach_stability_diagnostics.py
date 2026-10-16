"""Measured completion diagnostics retain the original departure gates."""
import math
import unittest
from unittest import mock

import rospy

from px4_wall_ceiling_control.msg import WallPerchStatus as W
import test_detach_state_machine as fixtures

at = fixtures.at


class DetachStabilityDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.DetachStateMachineTests()
        self.n = self.f.node()
        self.f.enter(self.n, W.STATE_DETACH_ROTATE, 10.)
        self.n.detach_elapsed_s = self.n.detach_time_s
        self.n._detach_height_loop_ready = lambda: True

    def sample(self, stamp, rates=(0., 0., 0.)):
        # Build the immutable ROS sample before it reaches the decision callback.
        from nav_msgs.msg import Odometry
        odom = Odometry()
        odom.header.stamp = at(stamp)
        odom.pose.pose.orientation.w = 1.
        odom.pose.pose.position.z = 2.4
        for axis, value in zip('xyz', rates):
            setattr(odom.twist.twist.angular, axis, math.radians(value))
        with mock.patch.object(rospy.Time, 'now', return_value=at(stamp)):
            self.n._on_odom(odom)
        self.f.output(self.n, stamp, tilt=0.)

    def stable_window(self):
        for stamp in (10.1, 10.2, 10.3, 10.4, 10.5):
            self.sample(stamp, (3., 4., 0.))

    def test_rates_norm_and_sample_duration_match_unchanged_completion_gate(self):
        self.stable_window()
        evidence = self.n._detach_rotation_diagnostics(at(10.5))
        for actual, expected in zip(evidence['rates_deg_s'], (3., 4., 0.)):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(evidence['rate_norm_deg_s'], 5.)
        self.assertAlmostEqual(evidence['stable_s'], .4)
        self.assertEqual(evidence['failed'], ())
        self.assertTrue(self.n._detach_rotation_ready(at(10.5)))
        # Reading diagnostics never lengthens the window or creates new samples.
        before = self.n.detach_recovered_samples
        self.n._detach_rotation_diagnostics(at(10.51))
        self.assertEqual(self.n.detach_recovered_samples, before)

    def test_axis_rates_below_ten_but_norm_above_ten_still_fail(self):
        self.stable_window()
        self.sample(10.52, (7., 7., 7.))
        evidence = self.n._detach_rotation_diagnostics(at(10.52))
        self.assertAlmostEqual(evidence['rate_norm_deg_s'], math.sqrt(147.))
        self.assertEqual(evidence['stable_s'], 0.)
        self.assertIn('angular_rate_norm', evidence['failed'])
        self.assertIn('continuous_stability', evidence['failed'])
        self.assertFalse(self.n._detach_rotation_ready(at(10.52)))

    def test_stale_odometry_does_not_report_old_stability_as_current(self):
        self.stable_window()
        evidence = self.n._detach_rotation_diagnostics(at(12.))
        self.assertEqual(evidence['stable_s'], 0.)
        self.assertIn('fresh_odometry', evidence['failed'])
        self.assertIn('published_target', evidence['failed'])
        self.assertFalse(self.n._detach_rotation_ready(at(12.)))

    def test_missing_height_or_trajectory_gate_and_published_cone_are_identified(self):
        self.stable_window()
        self.n._detach_height_loop_ready = lambda: False
        self.n.detach_elapsed_s = self.n.detach_time_s-.01
        self.f.output(self.n, 10.5, tilt=6.)
        evidence = self.n._detach_rotation_diagnostics(at(10.5))
        self.assertEqual(set(evidence['failed']), {'height_closed_loop', 'trajectory_complete', 'published_target'})
        self.assertFalse(self.n._detach_rotation_ready(at(10.5)))

    def test_transition_log_retains_rates_and_failed_conditions(self):
        self.sample(10.1, (0., 0., 20.))
        with mock.patch.object(rospy, 'loginfo') as log:
            self.n._enter_state(W.STATE_ABORT, at(10.1), 'detach rotation timeout')
        reason = log.call_args[0][-1]
        self.assertIn('detach rotation timeout', reason)
        self.assertIn('rate_xyz_deg_s=(0.00,0.00,20.00)', reason)
        self.assertIn('rate_norm_deg_s=20.00', reason)
        self.assertIn('stable_s=0.000', reason)
        self.assertIn('angular_rate_norm', reason)


if __name__ == '__main__':
    unittest.main()
