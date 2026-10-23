"""Single raw forward-range flip admission and retained safety gates."""
import io
import unittest
from unittest import mock

import rospy
from sensor_msgs.msg import Range
from px4_wall_ceiling_control.msg import WallPerchStatus as S
import test_front_only_late_top as fixtures
from test_attachment_safety import WallDecision


class SingleFrameFlipTests(unittest.TestCase):
    def node(self):
        n = fixtures.FrontOnlyLateTopTests().node()
        n.flip_trigger_hold_s = 0.0
        n.filter_alpha = .3
        n._filter_range = WallDecision._filter_range.__get__(n)
        return n

    def feed(self, n, distance, received=10., stamp=None, wire=False):
        sample = Range(range=distance, min_range=.03, max_range=40.)
        sample.header.stamp = rospy.Time.from_sec(received if stamp is None else stamp)
        if wire:
            buffer = io.BytesIO()
            sample.serialize(buffer)
            sample = Range().deserialize(buffer.getvalue())
        with mock.patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(received)):
            n._on_front_range(sample)

    def test_first_frame_at_or_below_threshold_enters_flip(self):
        for distance in (.2, .19):
            with self.subTest(distance=distance):
                n = self.node()
                self.feed(n, distance)
                n._update_state_machine(rospy.Time.from_sec(10.))
                self.assertEqual(n.state, S.STATE_FLIP_TO_WALL)

    def test_wire_float32_threshold_is_inclusive(self):
        n = self.node()
        self.feed(n, .2, wire=True)
        self.assertGreater(n.front_range.range, .2)  # Float32 representation.
        n._update_state_machine(rospy.Time.from_sec(10.))
        self.assertEqual(n.state, S.STATE_FLIP_TO_WALL)
        n = self.node()
        self.feed(n, .200001, wire=True)
        n._update_state_machine(rospy.Time.from_sec(10.))
        self.assertEqual(n.state, S.STATE_SLOW_APPROACH)

    def test_raw_near_frame_does_not_wait_for_filtered_distance(self):
        n = self.node()
        self.feed(n, .8)
        self.feed(n, .2, received=10.02)
        self.assertGreater(n.front_filtered_m, .2)
        n._update_state_machine(rospy.Time.from_sec(10.02))
        self.assertEqual(n.state, S.STATE_FLIP_TO_WALL)

    def test_above_threshold_never_flips_by_waiting(self):
        n = self.node()
        self.feed(n, .2001)
        for time in (10., 10.3, 10.5):
            n._update_state_machine(rospy.Time.from_sec(time))
            self.assertEqual(n.state, S.STATE_SLOW_APPROACH)

    def test_invalid_old_future_and_duplicate_frames_do_not_authorize_flip(self):
        for distance, stamp in ((float('nan'), 10.02), (.01, 10.02),
                                (.2, 9.), (.2, 10.1), (.2, 10.)):
            with self.subTest(distance=distance, stamp=stamp):
                n = self.node()
                self.feed(n, .8)
                self.feed(n, distance, received=10.02, stamp=stamp)
                n._update_state_machine(rospy.Time.from_sec(10.02))
                self.assertEqual(n.state, S.STATE_SLOW_APPROACH)

    def test_expired_near_frame_does_not_flip(self):
        n = self.node()
        self.feed(n, .2)
        self.assertFalse(n._flip_ready(rospy.Time.from_sec(10.61)))
        n._update_state_machine(rospy.Time.from_sec(10.61))
        self.assertEqual(n.state, S.STATE_SLOW_APPROACH)

    def test_invalid_front_health_blocks_single_frame(self):
        n = self.node()
        n.sensor_health.front_valid = False
        self.feed(n, .2)
        n._update_state_machine(rospy.Time.from_sec(10.))
        self.assertEqual(n.state, S.STATE_SLOW_APPROACH)

    def test_near_frame_does_not_bypass_flight_or_velocity_gates(self):
        for cause in ('flight', 'velocity'):
            with self.subTest(cause=cause):
                n = self.node()
                self.feed(n, .2)
                if cause == 'flight':
                    n._safety_ok = lambda now: False
                else:
                    n.horizontal_velocity_enabled = True
                    n.horizontal_velocity_ready = False
                    n.horizontal_velocity_reason = 'test missing feedback'
                n._update_state_machine(rospy.Time.from_sec(10.))
                self.assertEqual(n.state, S.STATE_ABORT if cause == 'flight' else S.STATE_SLOW_APPROACH)


if __name__ == '__main__':
    unittest.main()
