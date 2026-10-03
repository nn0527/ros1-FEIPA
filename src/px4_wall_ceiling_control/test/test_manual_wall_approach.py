"""Mode ownership and timing regression tests using ROS messages, no flight."""
import unittest
import math
from unittest import mock
import rospy
from mavros_msgs.msg import State
from std_msgs.msg import Bool
from sensor_msgs.msg import Range
from px4_wall_ceiling_control.msg import OperatorCommand, SensorHealth, WallPerchStatus
from test_flight_handover import Manager, OutputMux
from test_attachment_safety import WallDecision


class ManualApproachTests(unittest.TestCase):
    def setUp(self):
        self.now = rospy.Time.from_sec(10)
        self.clock = 100.0
        n = self.node = Manager.__new__(Manager)
        n.phase, n.selected_mode, n.phase_since = Manager.IDLE, None, self.clock
        n.wall_manual_approach = True
        n.manual_wall_wait_timeout_s = 60
        n.manual_wall_wait_started = None
        n.manual_wait_canceled = False
        n.stream_since = None
        n.prestream_s = 1.5
        n.state_timeout_s = n.sensor_timeout_s = n.status_timeout_s = n.output_timeout_s = .5
        n.flight, n.flight_at = State(connected=True, armed=True, mode="ALTCTL"), self.now
        n.sensor, n.sensor_at = SensorHealth(front_valid=True, ceiling_valid=True), self.now
        n.wall = WallPerchStatus(state=WallPerchStatus.STATE_IDLE, flip_ready=False,
                                front_range_fresh=True, top_range_fresh=True,
                                top_preflight_ready=True, entry_height_ready=True)
        n.wall_at = self.now
        n.owner, n.owner_at, n.output_mode, n.output_at = 0, self.now, 2, self.now
        n._requested_mode = mock.Mock(return_value=OperatorCommand.MODE_WALL)
        n._preflight_ready = mock.Mock(return_value=True)
        n._preflight_failure_reason = mock.Mock(return_value=None)
        n._request_mode = mock.Mock()
        n._handover_ready = mock.Mock(return_value=True)
        n.cancel_pub = mock.Mock()
        n.prestream_pub = mock.Mock()
        n.phase_pub = mock.Mock()
        n.phase_reason_pub = mock.Mock()
        n.kill_guard_enabled = False

    def tick(self):
        with mock.patch.object(rospy.Time, "now", return_value=self.now), mock.patch("time.monotonic", return_value=self.clock):
            self.node._on_timer(None)

    def test_request_warms_stream_in_altctl_without_mode_command(self):
        self.tick()
        self.clock += 30
        self.tick()
        self.assertEqual(self.node.phase, Manager.WAITING_WALL)
        self.node._request_mode.assert_not_called()
        self.assertTrue(self.node.prestream_pub.publish.call_args[0][0].data)
        self.assertEqual(self.node.flight.mode, "ALTCTL")

    def test_ready_has_no_extra_three_second_delay(self):
        self.tick()
        self.node.wall.flip_ready = True
        self.tick()  # Same monotonic time: no added reaction delay.
        self.assertEqual(self.node.phase, Manager.WAITING_WALL)
        self.tick()
        self.clock += 1.6
        self.tick()
        self.assertEqual(self.node.phase, Manager.ENTERING)
        self.node._request_mode.assert_called_once_with("OFFBOARD")

    def test_stale_or_invalid_range_cannot_start_takeover(self):
        self.tick()
        self.node.wall.flip_ready = True
        self.node.wall_at = rospy.Time.from_sec(8)
        self.tick()
        self.assertEqual(self.node.phase, Manager.WAITING_WALL)
        self.node.wall_at = self.now
        self.node.sensor.front_valid = False
        self.tick()
        self.assertEqual(self.node.phase, Manager.WAITING_WALL)
        self.node._request_mode.assert_not_called()

    def test_distance_changes_do_not_restart_parallel_stream_or_extend_deadline(self):
        self.tick()
        self.node.wall.flip_ready = True
        self.tick()
        self.tick()
        self.node.wall.flip_ready = False
        self.clock += 1
        self.tick()
        self.assertEqual(self.node.phase, Manager.WAITING_WALL)
        self.assertEqual(self.node.manual_wall_wait_started, 100)
        self.assertEqual(self.node.stream_since, 100)
        self.node._request_mode.assert_not_called()
        self.clock += 1
        self.node.wall.flip_ready = True
        self.tick()
        self.assertEqual(self.node.phase, Manager.ENTERING)
        self.node._request_mode.assert_called_once_with("OFFBOARD")

    def test_warm_stream_takes_over_on_distance_gate_without_additional_wait(self):
        self.tick()
        self.tick()
        self.clock += 10
        self.tick()
        self.node._request_mode.assert_not_called()
        self.node.wall.flip_ready = True
        self.tick()
        self.assertEqual(self.node.phase, Manager.ENTERING)
        self.node._request_mode.assert_called_once_with("OFFBOARD")

    def test_output_stream_loss_restarts_only_stream_confirmation(self):
        self.tick()
        self.tick()
        self.clock += 5
        self.node.output_mode = 0
        self.tick()
        self.assertIsNone(self.node.stream_since)
        self.node.output_mode = 2
        self.node.wall.flip_ready = True
        self.tick()
        self.assertEqual(self.node.phase, Manager.WAITING_WALL)
        self.clock += 1.4
        self.tick()
        self.node._request_mode.assert_not_called()
        self.clock += .2
        self.tick()
        self.node._request_mode.assert_called_once_with("OFFBOARD")

    def test_timeout_cancels_intent_without_changing_flight_mode(self):
        self.tick()
        self.clock += 60
        self.tick()
        self.assertEqual(self.node.phase, Manager.LOCKED)
        self.node._request_mode.assert_not_called()
        self.assertEqual(self.node.cancel_pub.publish.call_args[0][0].data, "cancel")

    def test_wait_cancel_only_applies_before_takeover(self):
        self.tick()
        self.node._on_wait_cancel(Bool(data=True))
        self.tick()
        self.assertEqual(self.node.phase, Manager.LOCKED)
        self.node._request_mode.assert_not_called()
        self.node.phase = Manager.ENTERING
        self.node.manual_wait_canceled = False
        self.node._on_wait_cancel(Bool(data=True))
        self.assertFalse(self.node.manual_wait_canceled)

    def test_offboard_confirmed_after_gate_loss_returns_without_flip(self):
        self.tick()
        self.node.phase = Manager.ENTERING
        self.node.flight.mode = "OFFBOARD"
        self.tick()
        self.assertEqual(self.node.phase, Manager.RETURNING)
        self.tick()
        self.node._request_mode.assert_called_once_with("ALTCTL")

    def test_rpc_rechecks_distance_after_async_dispatch(self):
        self.tick()
        self.node.phase = Manager.ENTERING
        self.node.set_mode = mock.Mock()
        self.node.request_in_flight = True
        with mock.patch.object(rospy.Time, "now", return_value=self.now), mock.patch.object(rospy, "wait_for_service"):
            self.node._send_mode_request("OFFBOARD")
        self.node.set_mode.assert_not_called()
        self.assertFalse(self.node.request_in_flight)

    def test_decision_does_not_take_output_while_pilot_is_in_altctl(self):
        n = WallDecision.__new__(WallDecision)
        n.state = WallPerchStatus.STATE_IDLE
        n.manual_approach = True
        n.cancel_requested = n.rearm_required = False
        n._mode_requested = n._entry_height_ready = lambda *args: True
        n._safety_ok = lambda *_: False  # OFFBOARD requirement fails in ALTCTL.
        n._enter_state = mock.Mock()
        n._update_state_machine(self.now)
        n._enter_state.assert_not_called()
        self.assertFalse(n._acquire_requested(self.now))

    def test_decision_skips_automatic_approach_and_requests_output_owner(self):
        n = WallDecision.__new__(WallDecision)
        n.state = WallPerchStatus.STATE_IDLE
        n.manual_approach = True
        n.cancel_requested = n.rearm_required = False
        n._mode_requested = n._entry_height_ready = n._safety_ok = lambda *args: True
        n._flip_ready = n._top_fresh = n._sensor_health_fresh = lambda *_: True
        n.sensor_health = SensorHealth(front_valid=True, ceiling_valid=True)
        transitions = []
        def enter(state, *_):
            transitions.append(state)
            n.state = state
        n._enter_state = enter
        n._update_state_machine(self.now)
        self.assertEqual(transitions, [WallPerchStatus.STATE_FRONT_WALL_DETECT, WallPerchStatus.STATE_FLIP_TO_WALL])
        self.assertTrue(n._acquire_requested(self.now))
        n._safety_ok = lambda *_: False
        self.assertFalse(n._acquire_requested(self.now))


class FlipThrustTests(unittest.TestCase):
    def node(self, degrees):
        n = WallDecision.__new__(WallDecision)
        n.hover_thrust, n.flip_thrust_multiplier = .45, 1.35
        n.wall_angle_rad = math.pi / 2
        n.flip_thrust_follow_tilt = True
        angle = math.radians(degrees)
        n._current_quaternion = lambda: (0, math.sin(angle/2), 0, math.cos(angle/2))
        return n

    def test_level_vehicle_does_not_receive_flip_boost(self):
        self.assertAlmostEqual(self.node(0)._flip_thrust(), .45)
        self.assertLess(self.node(6)._flip_thrust(), .451)

    def test_boost_follows_actual_tilt_and_is_bounded(self):
        values = [self.node(angle)._flip_thrust() for angle in [0, 10, 20, 45, 70, 90, 120]]
        self.assertEqual(values, sorted(values))
        self.assertAlmostEqual(values[3], .52875)
        self.assertAlmostEqual(values[-1], .6075)

    def test_yaw_alone_does_not_increase_flip_thrust(self):
        n = self.node(0)
        n._current_quaternion = lambda: (0, 0, math.sin(.7), math.cos(.7))
        self.assertAlmostEqual(n._flip_thrust(), .45)

    def test_invalid_attitude_does_not_request_flip_boost(self):
        n = self.node(0)
        n._current_quaternion = lambda: None
        self.assertAlmostEqual(n._flip_thrust(), .45)

    def test_legacy_entry_retains_existing_flip_thrust(self):
        n = self.node(0)
        n.flip_thrust_follow_tilt = False
        self.assertAlmostEqual(n._flip_thrust(), .6075)

    def test_takeover_first_output_matches_altctl_and_blends_without_jump(self):
        mux = OutputMux.__new__(OutputMux)
        mux.match_fcu_thrust_on_handover = True
        mux.handover_start_thrust, mux.handover_started_at = .46, None
        mux.thrust_handover_ramp_s = 1
        first = mux._apply_thrust_handover(rospy.Time.from_sec(10), self.node(0)._flip_thrust())
        midpoint = mux._apply_thrust_handover(rospy.Time.from_sec(10.5), self.node(6)._flip_thrust())
        final = mux._apply_thrust_handover(rospy.Time.from_sec(11), self.node(20)._flip_thrust())
        self.assertAlmostEqual(first, .46)
        self.assertLess(abs(midpoint-first), .01)
        self.assertLess(final, .47)


class BenchRangeGraceTests(unittest.TestCase):
    def node(self):
        n = WallDecision.__new__(WallDecision)
        n.state = WallPerchStatus.STATE_FLIP_TO_WALL
        n.bench_wait_for_ranges = True
        n.require_top_sensor_on_acquire = True
        n.top_range = Range(range=.04, min_range=.03, max_range=40)
        n.top_filtered_m = .04
        n.top_received_at = rospy.Time.from_sec(10)
        n.sensor_timeout_s, n.sensor_health_timeout_s = .6, .3
        n.sensor_health = SensorHealth(ceiling_valid=False, front_valid=True)
        n.sensor_health_received_at = rospy.Time.from_sec(10)
        n.top_contact_since = n.top_contact_last_sample_at = rospy.Time.from_sec(10)
        n.top_contact_hold_s, n.top_contact_distance_m = 1, .056
        n.top_clear_distance_m = .1
        n.top_clear_since = n.top_clear_last_sample_at = None
        n._filter_range = lambda message, previous: message.range
        n._flight_ready = n._odom_fresh = n._vertical_speed_safe = n._rates_safe = lambda *args: True
        n.flip_max_rate_rps, n.capture_max_rate_rps = 6, 3
        return n

    def test_brief_error_keeps_bench_flip_capture_and_hold_safe(self):
        n = self.node()
        for state in [WallPerchStatus.STATE_FLIP_TO_WALL, WallPerchStatus.STATE_WALL_CAPTURE,
                      WallPerchStatus.STATE_WALL_HOLD, WallPerchStatus.STATE_WALL_PIN]:
            n.state = state
            self.assertTrue(n._safety_ok(rospy.Time.from_sec(10.1)))

    def test_persistent_range_loss_still_stops_after_original_timeout(self):
        n = self.node()
        now = rospy.Time.from_sec(10.61)
        n.sensor_health_received_at = now
        self.assertFalse(n._safety_ok(now))

    def test_sensor_manager_loss_still_stops_even_with_valid_range(self):
        n = self.node()
        n.top_received_at = rospy.Time.from_sec(10.4)
        self.assertFalse(n._safety_ok(rospy.Time.from_sec(10.4)))

    def test_normal_flight_keeps_instantaneous_health_requirement(self):
        n = self.node()
        n.bench_wait_for_ranges = False
        self.assertFalse(n._safety_ok(rospy.Time.from_sec(10.1)))

    def test_brief_sensor_error_does_not_bypass_flight_or_rate_protection(self):
        n = self.node()
        n._rates_safe = lambda *args: False
        self.assertFalse(n._safety_ok(rospy.Time.from_sec(10.1)))
        n._rates_safe = lambda *args: True
        n._flight_ready = lambda *args: False
        self.assertFalse(n._safety_ok(rospy.Time.from_sec(10.1)))

    def test_invalid_sample_never_refreshes_distance_and_restarts_confirmation(self):
        n = self.node()
        with mock.patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(10.2)):
            n._on_top_range(Range(range=float('nan'), min_range=.03, max_range=40))
        self.assertEqual(n.top_received_at, rospy.Time.from_sec(10))
        self.assertEqual(n.top_filtered_m, .04)
        self.assertFalse(n._top_contact_ready(rospy.Time.from_sec(10.2)))
        for t in [10.3, 10.5, 10.7, 10.9, 11.1, 11.3, 11.5]:
            now = rospy.Time.from_sec(t)
            with mock.patch.object(rospy.Time, 'now', return_value=now):
                n._on_top_range(Range(range=.04, min_range=.03, max_range=40))
            if t < 11.3:
                self.assertFalse(n._top_contact_ready(now))
        self.assertTrue(n._top_contact_ready(now))


class BenchDetachSupportTests(unittest.TestCase):
    def node(self):
        n = WallDecision.__new__(WallDecision)
        n.bench_wait_for_ranges = True
        n.bench_detach_hover_thrust = .48
        n.hover_thrust, n.max_thrust = .45, .85
        n.detach_altitude_m = 3
        n._height_feedback = lambda: (3, 0)
        n._current_quaternion = lambda: (0, 0, 0, 1)
        n.q_hover = (0, 0, 0, 1)
        return n

    def test_level_recovery_and_exit_keep_same_support_until_handover(self):
        n = self.node()
        for state in [WallPerchStatus.STATE_RECOVER, WallPerchStatus.STATE_EXIT]:
            n.state = state
            self.assertAlmostEqual(n._candidate(rospy.Time.from_sec(10))[2], .48)

    def test_bench_support_ignores_height_drift_and_respects_tilt_and_cap(self):
        n = self.node()
        n._height_feedback = lambda: (-100, 20)
        self.assertAlmostEqual(n._height_hold_thrust(), .48)
        a = math.radians(60)
        n._current_quaternion = lambda: (0, math.sin(a/2), 0, math.cos(a/2))
        self.assertAlmostEqual(n._height_hold_thrust(), .85)

    def test_normal_flight_and_pre_detach_exit_retain_hover_setting(self):
        n = self.node()
        n.state = WallPerchStatus.STATE_EXIT
        n.bench_wait_for_ranges = False
        self.assertAlmostEqual(n._candidate(rospy.Time.from_sec(10))[2], .45)
        n.bench_wait_for_ranges = True
        n.detach_altitude_m = None
        self.assertAlmostEqual(n._candidate(rospy.Time.from_sec(10))[2], .45)


if __name__ == "__main__": unittest.main()
