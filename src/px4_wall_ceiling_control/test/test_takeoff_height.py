import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))
from px4_wall_ceiling_control.adapters.takeoff_height import TakeoffHeightReference


class TakeoffHeightTests(unittest.TestCase):
    def test_recorded_negative_ground_origin_does_not_block_real_climb(self):
        reference = TakeoffHeightReference()
        reference.observe_state(True, False, 1)
        reference.observe_odom(-1.50466, 1.1, True)
        reference.observe_state(True, True, 1.2)
        self.assertAlmostEqual(reference.height(-0.149), 1.35566)
        self.assertLess(reference.height(-1.30), 0.30)
        reference.observe_odom(-0.1, 2, False)
        self.assertAlmostEqual(reference.ground_z, -1.50466)

    def test_start_in_air_and_stale_ground_cannot_create_reference(self):
        reference = TakeoffHeightReference()
        reference.observe_state(True, True, 1)
        reference.observe_odom(10, 1, False)
        self.assertIsNone(reference.height(11))
        reference.observe_state(True, False, 2)
        reference.observe_odom(0, 2.1, True)
        reference.observe_state(True, True, 3)
        self.assertIsNone(reference.height(5))

    def test_disconnect_is_not_disarm_and_next_flight_gets_new_ground(self):
        reference = TakeoffHeightReference()
        reference.observe_state(True, False, 1)
        reference.observe_odom(-1.5, 1.1, True)
        reference.observe_state(True, True, 1.2)
        reference.observe_state(False, False, 2)
        reference.observe_odom(0, 2.1, False)
        reference.observe_state(True, True, 2.2)
        self.assertAlmostEqual(reference.height(0), 1.5)
        reference.observe_state(True, False, 3)
        self.assertIsNone(reference.height(0))
        reference.observe_odom(-2, 3.1, True)
        reference.observe_state(True, True, 3.2)
        self.assertAlmostEqual(reference.height(-1), 1)

    def test_invalid_odometry_does_not_replace_ground(self):
        reference = TakeoffHeightReference()
        reference.observe_state(True, False, 1)
        reference.observe_odom(-1.5, 1.1, True)
        reference.observe_odom(float('nan'), 1.15, True)
        reference.observe_state(True, True, 1.2)
        self.assertIsNone(reference.height(float('nan')))
        self.assertAlmostEqual(reference.height(0), 1.5)


if __name__ == '__main__':
    unittest.main()
