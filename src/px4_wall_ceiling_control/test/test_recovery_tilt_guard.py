"""Pure source-time and hysteresis tests for early attitude-recovery priority."""
import math
import unittest
from px4_wall_ceiling_control.domain.recovery_tilt_guard import RecoveryTiltGuard


class RecoveryTiltGuardTests(unittest.TestCase):
    def setUp(self):
        self.g = RecoveryTiltGuard(*map(math.radians,(5.,8.,10.)),.30,.50,math.radians(30.))

    def update(self,t,tilt=0.,rate=0.,source=None):
        return self.g.update(t,t if source is None else source,math.radians(tilt),math.radians(rate))

    def test_weight_boundaries_and_partial_correction(self):
        for t,tilt,weight in ((10.,0.,1.),(10.1,5.,1.),(10.2,6.5,.5),(10.3,8.,0.)):
            self.assertAlmostEqual(self.update(t,tilt),weight)
        self.assertTrue(self.g.paused)

    def test_latch_does_not_reopen_at_seven_degrees(self):
        self.update(10.,8.)
        for t in (10.1,10.3,10.5):self.assertEqual(self.update(t,7.),0.)

    def test_resume_requires_level_and_continuous_low_rate_samples(self):
        self.update(10.,8.)
        self.assertEqual(self.update(10.1,5.),0.)
        self.assertEqual(self.update(10.3,5.),0.)
        self.assertEqual(self.update(10.4,5.),1.)
        self.assertFalse(self.g.paused)

    def test_rate_excursion_restarts_confirmation(self):
        self.update(10.,8.);self.update(10.1,4.);self.update(10.3,4.,31.)
        self.assertEqual(self.update(10.4,4.),0.)
        self.assertEqual(self.update(10.7,4.),1.)

    def test_tilt_excursion_restarts_confirmation(self):
        self.update(10.,8.);self.update(10.1,4.);self.update(10.3,5.1)
        self.assertEqual(self.update(10.4,4.),0.)
        self.assertEqual(self.update(10.7,4.),1.)

    def test_duplicate_source_cannot_complete_confirmation(self):
        self.update(10.,8.);self.update(10.1,4.)
        self.assertEqual(self.update(10.5,4.,source=10.1),0.)
        self.assertEqual(self.g.samples,1)

    def test_queued_source_samples_cannot_bypass_real_time(self):
        self.update(10.,8.);self.update(10.1,4.,source=10.1)
        self.assertEqual(self.update(10.12,4.,source=10.4),0.)

    def test_long_sample_gap_starts_new_confirmation(self):
        self.update(10.,8.);self.update(10.1,4.)
        self.assertEqual(self.update(10.8,4.),0.)
        self.assertEqual(self.update(11.1,4.),1.)

    def test_clock_or_source_reversal_cannot_reopen(self):
        for now,source in ((10.05,10.2),(10.2,10.05)):
            self.g.reset(False);self.update(10.,8.);self.update(10.1,4.)
            self.assertEqual(self.update(now,4.,source=source),0.)
            self.assertEqual(self.g.samples,0)

    def test_feedback_reset_preserves_pending_level_latch(self):
        self.update(10.,8.);self.update(10.1,4.);self.g.reset()
        self.assertTrue(self.g.paused)
        self.assertIsNone(self.g.since_source)
        self.g.reset(False)
        self.assertFalse(self.g.paused)

    def test_nonfinite_feedback_fails_closed(self):
        self.assertEqual(self.update(10.,math.nan),0.)
        self.assertTrue(self.g.paused)

    def test_invalid_configuration_is_rejected(self):
        for soft,stop,hard,hold in ((8.,5.,10.,.3),(5.,10.,10.,.3),(5.,8.,10.,0.),(5.,8.,10.,math.nan)):
            with self.assertRaises(ValueError):
                RecoveryTiltGuard(*map(math.radians,(soft,stop,hard)),hold,.5,.5)


if __name__ == '__main__':
    unittest.main()
