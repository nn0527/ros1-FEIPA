import importlib.util
import pathlib
import math
import unittest
from unittest import mock
import rospy
from px4_wall_ceiling_control.msg import WallPerchStatus
p = pathlib.Path(__file__).resolve().parents[1] / "scripts/decisions/wall_perch_decision_node.py"
spec = importlib.util.spec_from_file_location("wall_recovery_test", str(p))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

class RecoveryLimitsTest(unittest.TestCase):
    def node(self, altitude=0.0, speed=0.0, tilt=0.0):
        n = m.WallPerchDecisionNode.__new__(m.WallPerchDecisionNode)
        n.hover_thrust, n.max_thrust, n.recover_thrust_multiplier = .45, .85, 1.1
        n.recovery_max_thrust, n.recovery_positive_compensation_max = .60, .08
        n.recovery_max_up_speed_mps, n.recovery_max_down_speed_mps = .10, .15
        n.recovery_height_to_speed_kp, n.recovery_velocity_kp = .5, .3
        n.detach_altitude_m = 0.0
        n.q_hover = (0.0, 0.0, 0.0, 1.0)
        n._height_feedback = lambda: (altitude, speed)
        n._current_quaternion = lambda: (0.0, math.sin(tilt/2), 0.0, math.cos(tilt/2))
        return n

    def test_climb_target_saturates_at_point_one(self):
        n = self.node(altitude=-10)
        self.assertAlmostEqual(n._recovery_height_thrust(), .48)
        n = self.node(altitude=-.1)
        self.assertAlmostEqual(n._recovery_height_thrust(), .465)

    def test_descent_target_saturates_at_point_fifteen(self):
        n = self.node(altitude=10)
        self.assertAlmostEqual(n._recovery_height_thrust(), .405)

    def test_descent_braking_is_included_in_positive_cap(self):
        n = self.node(altitude=-10, speed=-5)
        self.assertAlmostEqual(n._recovery_height_thrust(), .53)

    def test_upward_overspeed_reduces_thrust(self):
        n = self.node(altitude=-10, speed=.5)
        self.assertAlmostEqual(n._recovery_height_thrust(), .33)

    def test_tilt_compensation_and_fallback_obey_final_cap(self):
        n = self.node(altitude=-10, speed=-5, tilt=math.radians(80))
        self.assertAlmostEqual(n._recovery_height_thrust(), .6)
        n._height_feedback = lambda: None
        n.hover_thrust = .8
        self.assertAlmostEqual(n._recovery_height_thrust(), .6)

    def test_only_recover_and_abort_use_new_loop_and_exit_unchanged(self):
        n = self.node()
        n._recovery_height_thrust = mock.Mock(return_value=.7)
        for state in (WallPerchStatus.STATE_RECOVER, WallPerchStatus.STATE_ABORT):
            n.state = state
            self.assertAlmostEqual(n._candidate(rospy.Time.from_sec(1))[2], .6)
        self.assertEqual(n._recovery_height_thrust.call_count, 2)
        n.state = WallPerchStatus.STATE_EXIT
        self.assertAlmostEqual(n._candidate(rospy.Time.from_sec(1))[2], .45)
        self.assertEqual(n._recovery_height_thrust.call_count, 2)

if __name__ == "__main__":
    unittest.main()
