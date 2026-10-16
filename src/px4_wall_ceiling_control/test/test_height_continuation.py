"""Bounded support acceptance, budgets, source ordering, and thrust continuity."""
import math
import unittest

from px4_wall_ceiling_control.domain.height_continuation import HeightContinuation
from px4_wall_ceiling_control.domain.departure_height import DepartureHeightController


class HeightContinuationTests(unittest.TestCase):
    def start(self, sign=1., tilt=20.):
        c = HeightContinuation()
        c.observe(10., 10., 2., 0., tilt)
        c.observe(10.1, 10.1, 2.+sign*.4, 0., tilt)
        self.assertEqual(c.state, 'SUPPORT')
        return c

    def stable(self, c, start=10.2, height=2.4, count=7):
        for i in range(count):
            t = start+i*.1
            c.observe(t, t, height, 0., 5.)

    def test_positive_and_negative_jump_need_window_and_blend(self):
        for sign in (-1., 1.):
            c = self.start(sign)
            self.stable(c, height=2.+sign*.4, count=4)
            self.assertEqual(c.state, 'BLENDING')
            self.assertEqual(c.reanchor_height, 2.+sign*.4)
            self.assertFalse(c.ready)
            self.stable(c, start=10.6, height=2.+sign*.4, count=3)
            self.assertTrue(c.ready)
            self.assertAlmostEqual(c.total_used, .7)

    def test_angle_above_30_refuses_continuation(self):
        c = HeightContinuation(); c.observe(10., 10., 2., 0., 31.)
        c.observe(10.1, 10.1, 1.6, 0., 31.)
        self.assertEqual(c.state, 'FAILED')

    def test_existing_support_fails_if_angle_leaves_cone(self):
        c = self.start(); c.observe(10.2, 10.2, 2.4, 0., 30.01)
        self.assertEqual(c.state, 'FAILED')

    def test_exact_detection_threshold_does_not_trigger(self):
        c = HeightContinuation(); c.observe(10., 10., 2., 0., 0.)
        c.observe(10.1, 10.1, 2.25, 0., 0.)
        self.assertTrue(c.ready)

    def test_duplicates_and_old_sources_cannot_advance_confirmation(self):
        c = self.start()
        c.observe(10.2, 10.2, 2.4, 0., 5.)
        for now, source in ((10.3, 10.2), (10.4, 10.1), (10.5, 10.2)):
            c.observe(now, source, 2.4, 0., 5.)
        self.assertEqual(len(c.window), 1)
        self.assertEqual(c.previous[0], 10.2)
        self.assertEqual(c.state, 'SUPPORT')
        c.observe(11.31, 10.2, 2.4, 0., 5.)
        self.assertEqual(c.state, 'FAILED')

    def test_queued_burst_does_not_count_as_continuous_recovery(self):
        c = self.start()
        for i in range(5):
            c.observe(10.6+i*.001, 10.2+i*.1, 2.4, 0., 5.)
        self.assertEqual(c.state, 'SUPPORT')

    def test_fewer_than_four_samples_or_large_gap_resets_window(self):
        c = self.start()
        for t in (10.2, 10.35, 10.5):
            c.observe(t, t, 2.4, 0., 5.)
        self.assertEqual(c.state, 'SUPPORT')
        c.observe(10.71, 10.71, 2.4, 0., 5.)
        self.assertEqual(len(c.window), 1)

    def test_signed_cancellation_cannot_hide_window_residual(self):
        c = self.start()
        for i in range(7):
            t = 10.2+i*.1
            c.observe(t, t, 2.4+(i%2)*.03, 0., 5.)
        self.assertEqual(c.state, 'SUPPORT')

    def test_repeated_anomalies_do_not_reset_episode_deadline(self):
        c = self.start()
        for i in range(1, 14):
            t = 10.1+i*.1
            c.observe(t, t, 2.4+(i%2)*.4, 0., 20.)
        self.assertEqual(c.state, 'FAILED')
        self.assertAlmostEqual(c.episode_at, 10.1)

    def test_recovery_speed_and_tilt_gates_remain_strict(self):
        for v, tilt in ((.201, 5.), (0., 10.01)):
            c = self.start()
            for i in range(8):
                t = 10.2+i*.1
                c.observe(t, t, 2.4+v*(t-10.2), v, tilt)
            self.assertEqual(c.state, 'SUPPORT')

    def test_blending_fault_uses_same_episode_and_budget(self):
        c = self.start(); self.stable(c, count=4)
        self.assertEqual(c.state, 'BLENDING')
        c.observe(10.6, 10.6, 2.8, 0., 5.)
        self.assertEqual(c.state, 'SUPPORT')
        self.assertAlmostEqual(c.episode_at, 10.1)
        self.assertAlmostEqual(c.total_used, .5)

    def test_separate_episodes_share_total_budget(self):
        c = self.start(); self.stable(c)
        self.assertTrue(c.ready)
        c.observe(12., 12., 2.8, 0., 20.)
        self.assertEqual(c.state, 'SUPPORT')
        c.observe(12.81, 12.81, 2.8, 0., 20.)
        self.assertEqual(c.state, 'FAILED')
        self.assertLess(c.episode_age(12.81), c.episode_s)

    def test_nonfinite_and_reversed_clock_fail(self):
        for now, z in ((10.05, 2.4), (10.2, math.nan)):
            c = self.start(); c.observe(now, 10.2, z, 0., 5.)
            self.assertEqual(c.state, 'FAILED')

    def test_support_ignores_height_and_speed_correction_with_output_slew(self):
        ctl = DepartureHeightController(); ctl.begin(2., .53, .74, 10.)
        ctl.integral = .04
        value = ctl.update(100., 10., math.cos(math.radians(20)), 10.02, 10.02,
                           .74, integrate=False, correction_weight=0.)
        self.assertLessEqual(abs(value-.74), .01000001)
        self.assertEqual(ctl.correction, 0.)
        self.assertEqual(ctl.target_speed, 0.)
        self.assertEqual(ctl.integral, 0.)
        self.assertFalse(ctl.integral_active)
        self.assertAlmostEqual(ctl.requested, .53/math.cos(math.radians(20)))

    def test_reanchor_preserves_pressure_base_output_and_blend_no_integral(self):
        ctl = DepartureHeightController(); ctl.begin(2., .53, .74, 10.)
        before = (ctl.base, ctl.wall_pressure, ctl.value, ctl.at)
        ctl.reanchor(1.5)
        self.assertEqual(before, (ctl.base, ctl.wall_pressure, ctl.value, ctl.at))
        self.assertEqual(ctl.reference, 1.5)
        ctl.update(1.5, -.2, 1., 10.02, 10.02, .74, correction_weight=.5)
        self.assertAlmostEqual(ctl.correction, .03)
        self.assertFalse(ctl.integral_active)


if __name__ == '__main__':
    unittest.main()
