"""Mocked ROS checks for heading continuity and isolated lateral damping."""
import math
import unittest

from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion
from px4_wall_ceiling_control.domain.recovery_attitude_limits import attitude_from_axis_yaw
from px4_wall_ceiling_control.msg import WallPerchStatus as W
import test_recover_retreat as fixtures
from test_detach_state_machine import at


class StraightRetreatIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.fx = fixtures.RetreatTests()
        self.fx.setUp()
        self.addCleanup(self.fx.doCleanups)
        self.n = self.fx.n

    def test_recovery_does_not_replace_frozen_heading_with_disturbed_yaw(self):
        n = self.n
        n.state = W.STATE_DETACH_ROTATE
        n.horizontal_takeover_yaw = 1.2
        self.fx.frame(13.1)
        q = attitude_from_axis_yaw((0., 0., 1.), 1.5)
        for key, value in zip('xyzw', q):
            setattr(n.odom.pose.pose.orientation, key, value)
        n._enter_state(W.STATE_RECOVER, at(13.1), 'heading test')
        self.assertAlmostEqual(n.departure_yaw, 1.2)
        # During a second capture for approach, q_hover uses the same frozen
        # reference instead of following the current pose again.
        n._capture_attitude_references()
        self.assertAlmostEqual(euler_from_quaternion(n.q_hover)[2], 1.2)

    def test_missing_takeover_reference_uses_existing_hover_heading(self):
        n = self.n
        n.state = W.STATE_DETACH_ROTATE
        n.horizontal_takeover_yaw = None
        n.q_hover = attitude_from_axis_yaw((0., 0., 1.), -.7)
        self.fx.frame(13.1)
        n._enter_state(W.STATE_RECOVER, at(13.1), 'heading fallback test')
        self.assertAlmostEqual(n.departure_yaw, -.7)

    def retreat(self, lateral_speed, yaw=0.):
        n = self.n
        n.departure_yaw = yaw
        n.departure_attitude_controller.reset((0., 0., 1.), yaw)
        n.recover_baseline_axis = (0., 0., 1.)
        n.recovery_horizontal_controller.reset()
        n.departure_clear_confirmed = False
        n.recovery_control_at = n.recovery_attitude_at = at(13.)
        n._prepare_simple_recovery_velocity = lambda now: None
        n.horizontal_velocity_ready = n.departure_feedback_valid = True
        n.horizontal_measured = (-lateral_speed*math.sin(yaw), lateral_speed*math.cos(yaw))
        for i in range(1, 151):
            q = n._simple_recovery_attitude(at(13.+i*.02))
        return q

    def test_sideways_speed_does_not_change_captured_target(self):
        for yaw in (0., 1.3, -2.7):
            with self.subTest(yaw=yaw):
                self.n.recover_entry_attitude = attitude_from_axis_yaw((0., 0., 1.), yaw)
                neutral = self.retreat(0., yaw)
                disturbed = self.retreat(.4, yaw)
                self.assertEqual(disturbed, neutral)
                self.assertEqual(disturbed, self.n.recover_entry_attitude)
                self.assertAlmostEqual(euler_from_quaternion(disturbed)[2], yaw)
                self.assertEqual(self.n.recovery_horizontal_controller.target, (0., 0.))
                self.assertFalse(self.n.horizontal_velocity_active)

    def test_feedback_loss_does_not_reset_roll_or_catch_up_elapsed_time(self):
        q = self.retreat(.4)
        previous = euler_from_quaternion(q)[0]
        self.n.horizontal_velocity_ready = False
        for t in (16.02, 17., 17.):
            q = self.n._simple_recovery_attitude(at(t))
            roll, _, yaw = euler_from_quaternion(q)
            self.assertLessEqual(abs(roll-previous), math.radians(.050001))
            self.assertAlmostEqual(yaw, 0.)
            self.assertFalse(self.n.horizontal_velocity_active)
            previous = roll


if __name__ == '__main__':
    unittest.main()
