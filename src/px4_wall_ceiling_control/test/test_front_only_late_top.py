"""Forward-only flip entry and one-frame top capture in the final 10%."""
import math
import unittest
from unittest import mock

import rospy
from sensor_msgs.msg import Range
from px4_wall_ceiling_control.msg import SensorHealth, WallPerchStatus as S
from px4_wall_ceiling_control.domain.geometry import quaternion_angle
import test_auto_wall_approach as fixtures
from test_attachment_safety import WallDecision


class FrontOnlyLateTopTests(unittest.TestCase):
    def node(self):
        n = fixtures.AutoDecisionTests().node()
        n._auto_approach_ranges_ready = WallDecision._auto_approach_ranges_ready.__get__(n)
        n._flip_ready = WallDecision._flip_ready.__get__(n)
        n._front_fresh = WallDecision._front_fresh.__get__(n)
        n._sensor_health_fresh = lambda now: True
        n.sensor_health = SensorHealth(front_valid=True, ceiling_valid=False)
        n.sensor_timeout_s = .6
        n.front_received_at = n.top_received_at = None
        n.front_range = n.top_range = None
        n.front_filtered_m = n.top_filtered_m = None
        n.front_ready_since = n.front_ready_last_sample_at = None
        n.flip_ready_since = n.flip_ready_last_sample_at = None
        n.front_ready_distance_m = .5
        n.front_ready_hold_s = .4
        n.flip_trigger_distance_m = .2
        n.flip_trigger_hold_s = .3
        n._filter_range = lambda message, previous: message.range
        n.flip_time_s = 2.
        n.flip_top_start_fraction = .9
        n.flip_duration_timeout_s = 4.
        n.flip_max_tracking_error_rad = math.radians(15.)
        n.flip_elapsed_s = n.flip_step_dt = 0.
        n.flip_step_at = rospy.Time.from_sec(10.)
        n.flip_clock_reversed = False
        n.q_hover, n.q_wall = n._hover_and_wall_quaternions(0., math.pi/2)
        n.q_flip_start = n.q_hover
        # These tests isolate the range gate; fresh measured attitude follows
        # the last bounded target, as a successful attitude tracker would.
        n._odom_fresh = lambda now: True
        n._current_quaternion = lambda: n._quat_slerp(
            n.q_flip_start, n.q_wall, n._smoothstep5(n._flip_progress()))
        n.top_hold_accept_after = None
        n.top_hold_frame_at = None
        n.top_contact_since = n.top_contact_last_sample_at = None
        n.top_clear_since = n.top_clear_last_sample_at = None
        n.top_capture_samples = []
        n.top_capture_near_samples = 0
        n.top_capture_window_samples = 3
        n.top_contact_distance_m = .056
        n.top_clear_distance_m = .1
        n.filter_alpha = n.top_flip_filter_alpha = n.top_hold_filter_alpha = 1.
        n._pin_triggered = lambda: False
        n.capture_timeout_s = 8.
        return n

    def advance_flip(self, n, seconds):
        """Run actual bounded control steps; a single wall-clock jump is no flip."""
        end = rospy.Time.from_sec(seconds)
        while n.flip_step_at < end:
            now = min(n.flip_step_at + rospy.Duration.from_sec(.02), end)
            n._advance_flip_progress(now)
        return end

    def sample(self, n, t, value=.04, top=True, stamp=None):
        message = Range(range=value, min_range=.03, max_range=40.)
        message.header.stamp = rospy.Time.from_sec(t if stamp is None else stamp)
        with mock.patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(t)):
            (n._on_top_range if top else n._on_front_range)(message)

    def test_top_invalid_does_not_reset_front_continuous_confirmation(self):
        n = self.node()
        for t in [10., 10.1, 10.2, 10.31]:
            self.sample(n, t, .1, top=False)
            self.sample(n, t, float('nan'))
            with mock.patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(t)):
                n._on_sensor_health(SensorHealth(front_valid=True, ceiling_valid=False))
        self.assertTrue(n._flip_ready(rospy.Time.from_sec(10.31)))
        n._update_state_machine(rospy.Time.from_sec(10.31))
        self.assertEqual(n.state, S.STATE_FLIP_TO_WALL)

    def test_front_invalid_or_stale_still_blocks_flip(self):
        for invalid_health in [False, True]:
            n = self.node()
            for t in [10., 10.1, 10.2, 10.31]:
                self.sample(n, t, .1, top=False)
            if invalid_health:
                n.sensor_health.front_valid = False
                now = rospy.Time.from_sec(10.32)
            else:
                now = rospy.Time.from_sec(11.)
            n._update_state_machine(now)
            self.assertEqual(n.state, S.STATE_SLOW_APPROACH)

    def test_distance_above_threshold_still_blocks_flip(self):
        n = self.node()
        for t in [10., 10.1, 10.2, 10.4]:
            self.sample(n, t, .21, top=False)
        n._update_state_machine(rospy.Time.from_sec(10.4))
        self.assertEqual(n.state, S.STATE_SLOW_APPROACH)

    def test_top_input_before_90_percent_is_not_consumed(self):
        n = self.node(); n.state = S.STATE_FLIP_TO_WALL
        self.advance_flip(n, 11.79)
        self.sample(n, 11.79)
        self.assertIsNone(n.top_received_at)
        self.assertFalse(n._late_top_frame_ready(rospy.Time.from_sec(11.8)))
        n._update_state_machine(rospy.Time.from_sec(11.8))
        self.assertEqual(n.state, S.STATE_FLIP_TO_WALL)

    def test_one_valid_frame_at_90_percent_enters_hold_without_distance_gate(self):
        n = self.node(); n.state = S.STATE_FLIP_TO_WALL
        self.advance_flip(n, 11.8)
        self.sample(n, 11.8, 2.)  # Valid measurement, not a near-distance requirement.
        n._update_state_machine(rospy.Time.from_sec(11.8))
        self.assertEqual(n.state, S.STATE_WALL_HOLD)
        self.assertEqual(n.top_capture_near_samples, 0)

    def test_invalid_or_old_header_frame_cannot_authorize_hold(self):
        for value, stamp in [(float('nan'), 11.8), (.02, 11.8), (.04, 11.7), (.04, 10.)]:
            n = self.node(); n.state = S.STATE_FLIP_TO_WALL
            self.advance_flip(n, 11.8)
            self.sample(n, 11.8, value, stamp=stamp)
            n._update_state_machine(rospy.Time.from_sec(11.8))
            self.assertEqual(n.state, S.STATE_FLIP_TO_WALL, (value, stamp))

    def test_cached_preflip_top_range_cannot_authorize_hold(self):
        n = self.node(); n.state = S.STATE_FLIP_TO_WALL
        n.top_range = Range(range=.04, min_range=.03, max_range=40.)
        n.top_range.header.stamp = rospy.Time.from_sec(11.79)
        n.top_received_at = rospy.Time.from_sec(11.79)
        n.top_filtered_m = .04
        self.advance_flip(n, 11.8)
        n._update_state_machine(rospy.Time.from_sec(11.8))
        self.assertEqual(n.state, S.STATE_FLIP_TO_WALL)

    def test_flip_completes_without_top_then_waits_for_one_frame(self):
        n = self.node(); n.state = S.STATE_FLIP_TO_WALL
        self.advance_flip(n, 12.)
        n._update_state_machine(rospy.Time.from_sec(12.))
        self.assertEqual(n.state, S.STATE_WALL_CAPTURE)
        self.sample(n, 12.1, .2)
        n._update_state_machine(rospy.Time.from_sec(12.1))
        self.assertEqual(n.state, S.STATE_WALL_HOLD)

    def test_capture_still_times_out_without_any_valid_top_frame(self):
        n = self.node(); n.state = S.STATE_FLIP_TO_WALL
        self.advance_flip(n, 12.)
        n._update_state_machine(rospy.Time.from_sec(12.))
        n._update_state_machine(rospy.Time.from_sec(20.1))
        self.assertEqual(n.state, S.STATE_ABORT)
        self.assertEqual(n.last_reason, 'wall capture timeout')

    def test_invalid_frame_after_one_valid_frame_does_not_erase_latched_success(self):
        n = self.node(); n.state = S.STATE_FLIP_TO_WALL
        self.advance_flip(n, 11.8)
        self.sample(n, 11.8)
        self.sample(n, 11.81, float('nan'))
        n._update_state_machine(rospy.Time.from_sec(11.82))
        self.assertEqual(n.state, S.STATE_WALL_HOLD)

    def test_top_absence_does_not_bypass_flight_odom_or_rate_failure(self):
        n = self.node()
        n._safety_ok = WallDecision._safety_ok.__get__(n)
        n._flight_ready = n._odom_fresh = n._rates_safe = n._vertical_speed_safe = lambda *a: True
        n.flip_max_rate_rps = n.capture_max_rate_rps = 4.5
        for state in [S.STATE_FLIP_TO_WALL, S.STATE_WALL_CAPTURE]:
            n.state = state
            self.assertTrue(n._safety_ok(rospy.Time.from_sec(11.8)))
            for name in ['_flight_ready', '_odom_fresh', '_rates_safe', '_vertical_speed_safe']:
                original = getattr(n, name); setattr(n, name, lambda *a: False)
                self.assertFalse(n._safety_ok(rospy.Time.from_sec(11.8)), (state, name))
                setattr(n, name, original)

    def test_direct_flip_to_hold_preserves_attitude_target(self):
        n = self.node(); n.state = S.STATE_FLIP_TO_WALL
        n.hover_thrust = .5; n.flip_thrust_multiplier = 1.2
        n.hold_thrust_multiplier = 1.5; n.hold_entry_blend_s = .5
        n.q_hover, n.q_wall = n._hover_and_wall_quaternions(0., math.pi/2)
        n.q_contact = n.q_wall; n.odom = None
        now = self.advance_flip(n, 11.8)
        before = n._candidate(now)[1]
        WallDecision._enter_state(n, S.STATE_WALL_HOLD, now, 'late frame')
        after = n._candidate(now)[1]
        self.assertLess(quaternion_angle(before, after), 1e-6)


if __name__ == '__main__':
    unittest.main()
