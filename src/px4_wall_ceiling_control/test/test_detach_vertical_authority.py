import math
import unittest
from unittest import mock

import rospy
from mavros_msgs.msg import State, AttitudeTarget, EstimatorStatus
from nav_msgs.msg import Odometry
from px4_wall_ceiling_control.adapters.wall_detach_vertical import WallDetachVerticalMixin
from px4_wall_ceiling_control.msg import WallPerchStatus


class DepartureAuthorityTests(unittest.TestCase):
    def node(self):
        n = WallDetachVerticalMixin()
        n.max_thrust, n.hover_thrust = .85, .49
        n.state_timeout_s, n.odom_timeout_s = 1.5, .5
        n.horizontal_velocity_enabled = True
        n.approach_base_vertical_thrust = .53
        with mock.patch.object(rospy, 'get_param', side_effect=lambda name, default=None: default):
            n._init_detach_vertical()
        n.odom = Odometry()
        n.odom.pose.pose.orientation.w = 1.
        n.odom.pose.pose.position.z = 2.6
        n._height_feedback = lambda: (n.odom.pose.pose.position.z, n.odom.twist.twist.linear.z)
        n._current_quaternion = lambda: (0., 0., 0., 1.)
        n._odom_fresh = lambda now: 0 <= (now-n.odom.header.stamp).to_sec() <= .5
        n.horizontal_estimator = EstimatorStatus(attitude_status_flag=True,
            velocity_vert_status_flag=True, pos_vert_abs_status_flag=True)
        n.flight_state = State(connected=True, armed=True, mode='OFFBOARD')
        n.horizontal_output_owner = 2
        n.horizontal_output = AttitudeTarget(thrust=.75)
        n.horizontal_output.orientation.w = 1.
        self.refresh(n, 10.)
        return n

    def refresh(self, n, seconds, thrust=None):
        now = rospy.Time.from_sec(seconds)
        n.odom.header.stamp = now
        n.horizontal_estimator_at = n.flight_state_received_at = now
        n.horizontal_output_at = n.horizontal_output_owner_at = now
        n.horizontal_output.header.stamp = now
        if thrust is not None:
            n.horizontal_output.thrust = thrust
        return now

    def test_begin_captures_height_support_and_actual_output(self):
        n = self.node()
        self.assertTrue(n._begin_detach_vertical(rospy.Time.from_sec(10), n.horizontal_output))
        self.assertEqual(n.recovery_altitude_m, 2.6)
        self.assertEqual(n.detach_vertical_controller.base, .53)
        self.assertEqual(n.detach_vertical_value, .75)
        self.assertFalse(n.recovery_wait_for_level)

    def test_begin_rejects_stale_or_wrong_owner_and_invalid_vertical_estimate(self):
        for field, value in [('horizontal_output_owner', 1), ('horizontal_output_at', rospy.Time.from_sec(9.))]:
            n = self.node(); setattr(n, field, value)
            self.assertFalse(n._begin_detach_vertical(rospy.Time.from_sec(10), n.horizontal_output))
        n = self.node(); n.horizontal_estimator.pos_vert_abs_status_flag = False
        self.assertFalse(n._begin_detach_vertical(rospy.Time.from_sec(10), n.horizontal_output))
        n = self.node(); n.flight_state.mode = 'POSCTL'
        self.assertFalse(n._begin_detach_vertical(rospy.Time.from_sec(10), n.horizontal_output))

    def test_first_level_and_recover_keep_same_reference_and_continuity(self):
        n = self.node(); n.horizontal_output.thrust = .68
        self.assertTrue(n._begin_detach_vertical(rospy.Time.from_sec(10), n.horizontal_output))
        n.odom.pose.pose.position.z = 2.4; n.odom.twist.twist.linear.z = -.3
        now = self.refresh(n, 10.02)
        value = n._detach_vertical_thrust(now, .02)
        self.assertGreaterEqual(value, .67-1e-9)
        self.assertEqual(n.recovery_altitude_m, 2.6)
        n.horizontal_output.thrust = value
        n.state = WallPerchStatus.STATE_RECOVER
        later = self.refresh(n, 10.04)
        following = n._detach_vertical_thrust(later, .02)
        self.assertLessEqual(abs(following-value), .0100001)
        self.assertEqual(n.recovery_altitude_m, 2.6)

    def test_missing_feedback_clears_integral_and_never_emits_fabricated_thrust(self):
        n = self.node(); n._begin_detach_vertical(rospy.Time.from_sec(10), n.horizontal_output)
        n.detach_vertical_controller.integral = .04
        n.horizontal_output_owner = 0
        self.assertTrue(math.isnan(n._detach_vertical_thrust(rospy.Time.from_sec(10.02), .02)))
        self.assertEqual(n.detach_vertical_controller.integral, 0.)
        self.assertFalse(n.recovery_support_valid)

    def test_coordinate_reset_is_latched_and_refused(self):
        n = self.node(); n._begin_detach_vertical(rospy.Time.from_sec(10), n.horizontal_output)
        n.odom.pose.pose.position.z = 1.
        now = self.refresh(n, 10.02)
        self.assertTrue(math.isnan(n._detach_vertical_thrust(now, .02)))
        self.assertIn('跳变', n.detach_vertical_fault)
        now = self.refresh(n, 10.04)
        self.assertTrue(math.isnan(n._detach_vertical_thrust(now, .02)))

    def test_short_echo_gap_continues_without_integral_then_recovers(self):
        n = self.node(); n._begin_detach_vertical(rospy.Time.from_sec(10), n.horizontal_output)
        n.detach_vertical_controller.integral = .04
        now = self.refresh(n, 10.22)
        n.horizontal_output.header.stamp = n.horizontal_output_at = rospy.Time.from_sec(10.)
        value = n._detach_vertical_thrust(now, .02)
        self.assertTrue(math.isfinite(value))
        self.assertTrue(n.detach_vertical_output_degraded)
        self.assertEqual(n.detach_vertical_controller.integral, 0.)
        self.assertLessEqual(abs(value-.75), .02500001)
        now = self.refresh(n, 10.24, value)
        self.assertTrue(math.isfinite(n._detach_vertical_thrust(now, .02)))
        self.assertFalse(n.detach_vertical_output_degraded)

    def test_fresh_fcu_echo_can_reseed_but_stale_or_wrong_owner_cannot(self):
        n = self.node(); n._begin_detach_vertical(rospy.Time.from_sec(10), n.horizontal_output)
        now = self.refresh(n, 10.6)
        n.horizontal_output.header.stamp = n.horizontal_output_at = rospy.Time.from_sec(10.)
        self.assertTrue(math.isnan(n._detach_vertical_thrust(now, .02)))
        n.detach_fcu_target = AttitudeTarget(thrust=.64)
        n.detach_fcu_target.orientation.w = 1.
        n.detach_fcu_target.header.stamp = n.detach_fcu_target_at = now
        value = n._detach_vertical_thrust(now, .02)
        self.assertTrue(math.isfinite(value))
        self.assertLessEqual(abs(value-.64), .02500001)
        self.assertEqual(n.detach_vertical_controller.integral, 0.)
        n.horizontal_output_owner = 0
        self.assertTrue(math.isnan(n._detach_vertical_thrust(now, .02)))

    def test_coordinate_reset_abort_reseeds_only_after_consistent_new_frame(self):
        n = self.node(); n._begin_detach_vertical(rospy.Time.from_sec(10), n.horizontal_output)
        n.odom.pose.pose.position.z = 1.
        now = self.refresh(n, 10.02)
        self.assertTrue(math.isnan(n._detach_vertical_thrust(now, .02)))
        n.state = WallPerchStatus.STATE_ABORT
        self.assertTrue(math.isnan(n._detach_vertical_thrust(now, .02)))
        now = self.refresh(n, 10.04)
        value = n._detach_vertical_thrust(now, .02)
        self.assertTrue(math.isfinite(value))
        self.assertEqual(value, .75)
        self.assertEqual(n.detach_vertical_controller.reference, 1.)
        self.assertEqual(n.detach_vertical_controller.integral, 0.)
        self.assertEqual(n.vertical_reference_source, 'fault_recovery_height')
        self.assertEqual(n.detach_vertical_fault, '')

    def test_status_carries_original_height_and_bounded_cap(self):
        n = self.node(); n._begin_detach_vertical(rospy.Time.from_sec(10), n.horizontal_output)
        status = WallPerchStatus()
        n._fill_detach_vertical_status(status)
        self.assertTrue(status.recovery_support_valid)
        self.assertEqual(status.recovery_thrust_limit, .85)
        self.assertEqual(status.vertical_reference_m, 2.6)
        self.assertEqual(status.detach_vertical_base_thrust, .53)
        n._reset_detach_vertical()
        n._fill_detach_vertical_status(status)
        self.assertFalse(status.detach_vertical_active)
        self.assertFalse(status.recovery_support_valid)


if __name__ == '__main__':
    unittest.main()
