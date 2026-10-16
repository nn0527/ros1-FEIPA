"""Departure height contracts; the numeric model is not a flight guarantee."""
import math
import unittest
from px4_wall_ceiling_control.domain.departure_height import DepartureHeightController


class DepartureHeightTests(unittest.TestCase):
    def controller(self, base=.53, seed=.75):
        c = DepartureHeightController()
        c.begin(2.6, base, seed, 10.)
        return c

    def test_entry_is_actual_pressure_not_nominal_hover_multiplier(self):
        c = self.controller()
        self.assertEqual(c.update(2.6, 0., 0., 10., 10., .75), .75)
        self.assertEqual(c.update(2.6, 0., 0., 10.02, 10.02, .75), .75)
        self.assertEqual(c.base, .53)
        self.assertEqual(c.integral, 0.)

    def test_first_level_preserves_height_and_bounds_thrust_change(self):
        c = self.controller(base=.49, seed=.68)
        actual = .68
        result = c.update(2.4, -.3, 1., 10.02, 10.02, actual)
        self.assertEqual(c.reference, 2.6)
        self.assertGreaterEqual(result, actual-.01000001)
        self.assertGreater(result, .57)
        self.assertAlmostEqual(c.target_speed, .10)

    def test_stall_and_duplicate_evaluation_cannot_catch_up(self):
        c = self.controller(seed=.60)
        first = c.update(2.6, 0., 1., 10.6, 10.6, .60)
        self.assertGreaterEqual(first, .575-1e-9)
        self.assertEqual(c.update(1., -2., 1., 10.6, 10.6, first), first)
        with self.assertRaises(ValueError):
            c.update(2.6, 0., 1., 10.5, 10.5, first)

    def test_duplicate_source_cannot_accumulate_integral(self):
        c = self.controller(seed=.56)
        for index in range(30):
            c.update(2.5, -.05, 1., 10.02+.02*index, 10.02, .56)
        self.assertEqual(c.integral, 0.)

    def test_no_integral_while_wall_or_output_saturated(self):
        c = self.controller(seed=.75)
        for index in range(100):
            now = 10.02+.02*index
            c.update(2., -.2, .2, now, now, .75)
        self.assertEqual(c.integral, 0.)
        for index in range(100):
            now = 12.02+.02*index
            c.update(2., -5., 1., now, now, .85)
        self.assertEqual(c.integral, 0.)

    def test_integral_can_release_in_opposite_direction(self):
        c = self.controller(seed=.59)
        c.integral = .06
        c.last_source_stamp = 10.
        c.update(2.6, .10, 1., 10.02, 10.02, .59)
        self.assertLess(c.integral, .06)

    def test_invalid_feedback_is_rejected(self):
        for bad in (math.nan, math.inf):
            c = self.controller()
            with self.assertRaises(ValueError):
                c.update(bad, 0., 1., 10.02, 10.02, .75)
        with self.assertRaises(ValueError):
            self.controller().update(2.6, 0., 1., 10.02, 0., .75)

    def test_support_is_bounded_at_every_measured_tilt(self):
        c = self.controller()
        actual = .75
        for index in range(301):
            fraction = math.cos(math.radians(90.*(1.-index/300.)))
            now = 10.02+.02*index
            value = c.update(2.6, 0., fraction, now, now, actual)
            self.assertTrue(.1 <= value <= .85)
            self.assertLessEqual(abs(value-actual), .01000001)
            actual = value
        self.assertAlmostEqual(actual, .53, places=5)

    def test_captured_equilibrium_holds_height_in_level_model(self):
        c = self.controller(seed=.53)
        height, speed, actual = 2.6, 0., .53
        for index in range(600):
            now = 10.02+.02*index
            actual = c.update(height, speed, 1., now, now, actual)
            speed += 9.80665*(actual/.53-1.)*.02
            height += speed*.02
        self.assertAlmostEqual(height, 2.6, places=6)
        self.assertAlmostEqual(speed, 0., places=6)

    def test_bounded_integral_removes_small_base_error_in_level_model(self):
        c = self.controller(base=.51, seed=.53)
        height, speed, actual = 2.6, 0., .53
        minimum = height
        for index in range(3000):
            now = 10.02+.02*index
            actual = c.update(height, speed, 1., now, now, actual)
            speed += 9.80665*(actual/.53-1.)*.02
            height += speed*.02
            minimum = min(height, minimum)
            self.assertLessEqual(abs(c.integral), .0600001)
        self.assertLess(abs(height-2.6), .01)
        self.assertLess(abs(speed), .01)
        self.assertGreater(minimum, 2.45)


if __name__ == '__main__':
    unittest.main()
