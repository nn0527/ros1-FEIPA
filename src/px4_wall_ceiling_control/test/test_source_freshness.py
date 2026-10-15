"""Source time is independent from receipt time; no live ROS master needed."""
import unittest
from unittest import mock
import rospy
from px4_wall_ceiling_control.freshness import is_fresh_source


class SourceFreshnessTests(unittest.TestCase):
    def test_stale_zero_and_missing_source_are_rejected(self):
        now = rospy.Time.from_sec(10.)
        for stamp in (None, rospy.Time(), rospy.Time.from_sec(9.49)):
            self.assertFalse(is_fresh_source(stamp, .5, now))
        self.assertTrue(is_fresh_source(rospy.Time.from_sec(9.5), .5, now))

    def test_only_bounded_fc_mapping_lead_is_allowed(self):
        now = rospy.Time.from_sec(10.)
        with mock.patch.object(rospy.Time, 'now', return_value=now):
            self.assertTrue(is_fresh_source(rospy.Time.from_sec(10.05), .5, now))
            self.assertFalse(is_fresh_source(rospy.Time.from_sec(10.11), .5, now))
            self.assertFalse(is_fresh_source(rospy.Time.from_sec(10.05), .5, now, 0.))

    def test_callback_during_tick_does_not_fake_future_command(self):
        with mock.patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(10.001)):
            self.assertTrue(is_fresh_source(rospy.Time.from_sec(10.0005), .2,
                                           rospy.Time.from_sec(10.), 0.))
            self.assertFalse(is_fresh_source(rospy.Time.from_sec(10.01), .2,
                                            rospy.Time.from_sec(10.), 0.))

    def test_invalid_limits_fail_closed(self):
        for timeout, future in [(-1., .1), (.5, -1.), (float('nan'), .1), (.5, float('inf'))]:
            self.assertFalse(is_fresh_source(rospy.Time.from_sec(10.), timeout,
                                            rospy.Time.from_sec(10.), future))


if __name__ == '__main__':
    unittest.main()
