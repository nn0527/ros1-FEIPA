"""Fixed-heading geometry, lateral sign, cone and roll slew without ROS."""
import math
import unittest

from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion, rotate_vector
from px4_wall_ceiling_control.domain.recovery_attitude_limits import attitude_from_axis_yaw
from px4_wall_ceiling_control.domain.straight_retreat import StraightRetreatAttitude


class StraightRetreatTests(unittest.TestCase):
    def command(self, c, yaw=0., velocity=1., dt=.02, baseline=(0., 0., 1.), pitch=-.02):
        axis = (math.sin(pitch)*math.cos(yaw), math.sin(pitch)*math.sin(yaw), math.cos(pitch))
        return c.attitude(baseline, attitude_from_axis_yaw(axis, yaw), velocity, yaw, dt,
                          math.radians(5.), .30)

    def test_lateral_damping_keeps_heading_and_pitch_in_all_directions(self):
        for yaw in (0., 1.4, -2.8, math.pi-.001, -math.pi+.001):
            for speed in (-1., 1.):
                c = StraightRetreatAttitude()
                for _ in range(150):
                    q = self.command(c, yaw, speed)
                roll, pitch, actual_yaw = euler_from_quaternion(q)
                self.assertGreater(roll*speed, 0.)
                self.assertLessEqual(abs(roll), math.radians(2.)+1e-9)
                self.assertAlmostEqual(pitch, -.02)
                self.assertAlmostEqual(math.atan2(math.sin(actual_yaw-yaw), math.cos(actual_yaw-yaw)), 0.)
                left = (-math.sin(yaw), math.cos(yaw))
                axis = rotate_vector(q, (0., 0., 1.))
                self.assertLess(sum(a*b for a, b in zip(axis, left))*speed, 0.)

    def test_alternating_noise_is_slow_and_long_gap_cannot_catch_up(self):
        c = StraightRetreatAttitude()
        previous = 0.
        for speed, dt in [(1., .02), (-1., .02), (1., 10.), (-1., 0.), (-1., -.1)]*30:
            q = self.command(c, velocity=speed, dt=dt)
            roll = euler_from_quaternion(q)[0]
            self.assertLessEqual(abs(roll-previous), math.radians(1.)*min(.05, max(0., dt))+1e-9)
            previous = roll

    def test_trusted_hover_roll_is_retained_when_lateral_speed_is_zero(self):
        # A nonzero equilibrium is a feedforward reference, not drift error.
        baseline = (0., -math.sin(.04), math.cos(.04))
        c = StraightRetreatAttitude()
        c.reset(baseline, 0.)
        q = self.command(c, velocity=0., baseline=baseline)
        self.assertAlmostEqual(euler_from_quaternion(q)[0], .04)

    def test_roll_correction_is_bounded_relative_to_hover_baseline(self):
        baseline = (0., -math.sin(.02), math.cos(.02))
        c = StraightRetreatAttitude(lateral_kp=1., roll_correction=math.radians(1.))
        c.reset(baseline, 0.)
        for _ in range(100):
            q = self.command(c, velocity=10., baseline=baseline)
        self.assertAlmostEqual(euler_from_quaternion(q)[0], .02+math.radians(1.))

    def test_combined_tilt_preserves_existing_five_degree_cone(self):
        c = StraightRetreatAttitude()
        for _ in range(300):
            q = self.command(c, pitch=-math.radians(5.))
            self.assertGreaterEqual(rotate_vector(q, (0., 0., 1.))[2], math.cos(math.radians(5.))-1e-9)

    def test_entry_roll_returns_slowly_without_recapturing_heading(self):
        c = StraightRetreatAttitude()
        axis = (0., -math.sin(math.radians(7.)), math.cos(math.radians(7.)))
        c.reset(axis, .5)
        previous = c.roll
        for _ in range(300):
            q = self.command(c, yaw=.5, velocity=0.)
            roll, _, yaw = euler_from_quaternion(q)
            self.assertLessEqual(abs(roll-previous), math.radians(.020001))
            self.assertAlmostEqual(yaw, .5)
            previous = roll

    def test_invalid_settings_and_feedback_are_rejected(self):
        for kwargs in ({'lateral_kp':float('nan')}, {'roll_rate':0.},
                       {'roll_correction':math.radians(6.)}):
            with self.assertRaises(ValueError):
                StraightRetreatAttitude(**kwargs)
        with self.assertRaises(ValueError):
            self.command(StraightRetreatAttitude(), velocity=float('inf'))


if __name__ == '__main__':
    unittest.main()
