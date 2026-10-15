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
        n.detach_altitude_m = n.recovery_altitude_m = 0.0
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
        self.assertTrue(math.isnan(n._recovery_height_thrust()))

    def test_recover_abort_and_exit_keep_same_bounded_loop(self):
        n = self.node()
        n._recovery_height_thrust = mock.Mock(return_value=.7)
        for state in (WallPerchStatus.STATE_RECOVER, WallPerchStatus.STATE_ABORT):
            n.state = state
            self.assertAlmostEqual(n._candidate(rospy.Time.from_sec(1))[2], .6)
        self.assertEqual(n._recovery_height_thrust.call_count, 2)
        n.state = WallPerchStatus.STATE_EXIT
        self.assertAlmostEqual(n._candidate(rospy.Time.from_sec(1))[2], .6)
        self.assertEqual(n._recovery_height_thrust.call_count, 3)

    def test_approach_uses_takeover_height_and_brakes_descent(self):
        n = self.node(altitude=.5, speed=-.69)
        n.approach_altitude_m = 1.6
        n.state = WallPerchStatus.STATE_SLOW_APPROACH
        n.approach_pitch_rad = 0.
        self.assertAlmostEqual(n._candidate(rospy.Time.from_sec(1))[2], .53)
        self.assertEqual(n.vertical_target_speed, .10)
        self.assertAlmostEqual(n.vertical_correction, .08)

    def test_abort_without_detach_never_uses_fixed_boost(self):
        n = self.node(altitude=1.6, speed=1.9)
        n.recovery_altitude_m = 1.6
        self.assertAlmostEqual(n._recovery_height_thrust(), .1)
        n.recovery_altitude_m = None
        self.assertAlmostEqual(n._recovery_height_thrust(), .1)
        self.assertEqual(n.vertical_target_speed, 0.)

    def test_first_level_reference_latches_once_not_departure_height(self):
        n = self.node(altitude=.9, tilt=math.radians(6))
        n.detach_altitude_m = 2.
        n.recovery_altitude_m = None
        n.recovery_wait_for_level = True
        n._odom_fresh = lambda _: True
        n._prepare_vertical_reference(rospy.Time.from_sec(1))
        self.assertIsNone(n.recovery_altitude_m)
        n._current_quaternion = lambda: (0.,0.,0.,1.)
        n._prepare_vertical_reference(rospy.Time.from_sec(2))
        self.assertAlmostEqual(n.recovery_altitude_m, .9)
        n._height_feedback = lambda: (.6,-.1)
        n._prepare_vertical_reference(rospy.Time.from_sec(3))
        self.assertAlmostEqual(n.recovery_altitude_m, .9)
        self.assertEqual(n.detach_altitude_m, 2.)

    def test_first_level_rejects_stale_odometry_and_uses_total_tilt(self):
        n = self.node(altitude=1.)
        n.recovery_altitude_m = None; n.recovery_wait_for_level = True
        n._odom_fresh = lambda _: False
        n._prepare_vertical_reference(rospy.Time.from_sec(1))
        self.assertIsNone(n.recovery_altitude_m)
        n._odom_fresh = lambda _: True
        # Roll=4deg and pitch=4deg exceed a 5deg total cone.
        a=math.radians(4)/2
        n._current_quaternion=lambda:(math.sin(a)*math.cos(a),math.cos(a)*math.sin(a),
                                      -math.sin(a)**2, math.cos(a)**2)
        n._prepare_vertical_reference(rospy.Time.from_sec(2))
        self.assertIsNone(n.recovery_altitude_m)

    @mock.patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(10))
    def test_takeover_height_is_not_reset_by_repeated_state_heartbeat(self, _clock):
        from mavros_msgs.msg import State
        n = self.node(altitude=1.6)
        n.flight_state = State(mode='POSCTL')
        n._odom_fresh = lambda _: True
        n._on_state(State(connected=True,armed=True,mode='OFFBOARD'))
        self.assertEqual(n.approach_altitude_m, 1.6)
        n._height_feedback = lambda: (.5,-.6)
        n._on_state(State(connected=True,armed=True,mode='OFFBOARD'))
        self.assertEqual(n.approach_altitude_m, 1.6)
        n._on_state(State(connected=True,armed=True,mode='POSCTL'))
        n._on_state(State(connected=True,armed=True,mode='OFFBOARD'))
        self.assertEqual(n.approach_altitude_m, .5)

    def test_abort_entry_from_approach_preserves_takeover_reference(self):
        n = self.node(altitude=.5)
        n.state=WallPerchStatus.STATE_SLOW_APPROACH
        n.STATE_NAMES=m.WallPerchDecisionNode.STATE_NAMES
        n.approach_altitude_m=1.6
        n._enter_state(WallPerchStatus.STATE_ABORT,rospy.Time.from_sec(1),'test')
        self.assertEqual(n.recovery_altitude_m,1.6)
        self.assertFalse(n.recovery_wait_for_level)

    def test_abort_from_wall_waits_for_level_and_never_uses_old_round(self):
        n = self.node(altitude=1.,tilt=math.pi/2)
        n.state=WallPerchStatus.STATE_WALL_HOLD
        n.STATE_NAMES=m.WallPerchDecisionNode.STATE_NAMES
        n.recovery_altitude_m=7.
        n._odom_fresh=lambda _:True
        n._enter_state(WallPerchStatus.STATE_ABORT,rospy.Time.from_sec(1),'test')
        self.assertIsNone(n.recovery_altitude_m)
        n._current_quaternion=lambda:(0.,0.,0.,1.)
        n._prepare_vertical_reference(rospy.Time.from_sec(2))
        self.assertEqual(n.recovery_altitude_m,1.)
        n._enter_state(WallPerchStatus.STATE_IDLE,rospy.Time.from_sec(3),'test')
        self.assertIsNone(n.approach_altitude_m)
        self.assertIsNone(n.recovery_altitude_m)

    def test_missing_feedback_is_not_clamped_into_valid_fixed_thrust(self):
        n=self.node();n.state=WallPerchStatus.STATE_ABORT
        n._height_feedback=lambda:None
        self.assertTrue(math.isnan(n._candidate(rospy.Time.from_sec(1))[2]))

    def test_handover_height_settling_uses_first_level_reference(self):
        n=self.node(altitude=.9,speed=.05)
        n.detach_altitude_m=2.;n.recovery_altitude_m=.9
        n.detach_altitude_tolerance_m=.10;n.detach_vertical_speed_tolerance_mps=.20
        self.assertTrue(n._detach_height_settled())
        n._height_feedback=lambda:(.9,.25)
        self.assertFalse(n._detach_height_settled())


if __name__ == "__main__":
    unittest.main()
