"""Range-only gain selection must not mutate legacy control or safety limits."""
import math
import unittest

from px4_wall_ceiling_control.domain.departure_height import DepartureHeightController
from px4_wall_ceiling_control.domain.range_height_feedback import RangeHeightConfig


class RangeControlGainTests(unittest.TestCase):
    def node(self):
        c = DepartureHeightController()
        c.begin(1., .525, .525, 10.)
        return c

    def test_range_gain_reduces_request_without_changing_base_or_legacy_gain(self):
        old, new = self.node(), self.node()
        old.update(1., .35, 1., 10.02, 10.02, .525, integrate=False)
        new.update(1., .35, 1., 10.02, 10.02, .525, integrate=False, velocity_kp=.18)
        self.assertAlmostEqual(old.correction, -.105)
        self.assertAlmostEqual(new.correction, -.063)
        self.assertEqual(new.velocity_kp, .30)
        self.assertEqual(new.base, .525)
        self.assertEqual(new.correction_limit, .12)
        self.assertEqual(new.thrust_slew, .50)

    def test_override_does_not_leak_to_next_legacy_update(self):
        c = self.node()
        c.update(1., .2, 1., 10.02, 10.02, .525, integrate=False, velocity_kp=.18)
        c.update(1., .2, 1., 10.04, 10.04, .525, integrate=False)
        self.assertAlmostEqual(c.correction, -.06)

    def test_override_used_consistently_after_integral_trial(self):
        c = self.node()
        c.update(1., -.02, 1., 10.02, 10.02, .525, velocity_kp=.18)
        c.update(1., -.02, 1., 10.04, 10.04, .525, velocity_kp=.18)
        self.assertTrue(c.integral_active)
        self.assertAlmostEqual(c.correction, .18*.02+c.integral)

    def test_override_keeps_source_and_output_slew_protection(self):
        c = self.node()
        value = c.update(1., -2., 1., 10.5, 10.5, .525, velocity_kp=.18)
        self.assertLessEqual(value-.525, .02500001)
        self.assertEqual(c.correction, .12)
        integral = c.integral
        c.update(1., -2., 1., 10.52, 10.5, value, velocity_kp=.18)
        self.assertEqual(c.integral, integral)

    def test_invalid_override_is_rejected_before_output_mutation(self):
        for gain in (0., -.1, math.nan, math.inf):
            c = self.node()
            with self.assertRaises(ValueError):
                c.update(1., .1, 1., 10.02, 10.02, .525, velocity_kp=gain)
            self.assertEqual(c.value, .525)
            self.assertEqual(c.at, 10.)

    def test_range_configuration_has_independent_validated_gain(self):
        self.assertEqual(RangeHeightConfig().control_velocity_kp, .18)
        for gain in (0., -.1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                RangeHeightConfig(control_velocity_kp=gain)


if __name__ == '__main__':
    unittest.main()
