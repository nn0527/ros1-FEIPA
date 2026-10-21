"""Normal wall departure: bounded trajectory, measured completion and deadlines."""
import math
import struct
import unittest
from unittest import mock

import rospy
from mavros_msgs.msg import AttitudeTarget, EstimatorStatus, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from px4_wall_ceiling_control.msg import AttachmentControlCandidate as C, WallPerchStatus as W, SensorHealth
from px4_wall_ceiling_control.domain.geometry import quaternion_angle
import test_attachment_safety as fixtures


def at(seconds):
    return rospy.Time.from_sec(seconds)


def quaternion(degrees):
    a = math.radians(degrees)/2.
    return (0., math.sin(a), 0., math.cos(a))


class DetachStateMachineTests(unittest.TestCase):
    def node(self, **extra):
        values = {'~auto_approach': True, '~horizontal_velocity_enabled': True,
                  '~hover_thrust': .48, '~detach_time_s': 3., '~recover_time_s': 1.,
                  '~top_clear_hold_s': .5, '~ignore_range_loss_during_attachment': True,
                  '~ignore_range_loss_during_hold': True, '~ignore_down_velocity_during_hold': True}
        values.update(extra)
        with mock.patch.object(rospy.Time, 'now', return_value=at(10.)), \
             mock.patch.object(rospy, 'get_param', side_effect=lambda key, default=None: values.get(key, default)), \
             mock.patch.object(rospy, 'Publisher'), mock.patch.object(rospy, 'Subscriber'), \
             mock.patch.object(rospy, 'Timer'), mock.patch.object(rospy, 'logwarn'):
            n = fixtures.WallDecision()
        n.state = W.STATE_WALL_HOLD; n.state_entered_at = at(9.)
        n.q_hover = quaternion(0.); n.q_wall = n.q_contact = n.q_hold_reference = quaternion(90.)
        n.hold_entry_blend_s = .5
        n.top_hold_filter_alpha = 1.
        n.approach_base_vertical_thrust = .518
        # This fixture represents an acquisition with its POSCTL equilibrium
        # and heading already captured, before the vehicle turned to the wall.
        n.horizontal_equilibrium.ready = n.horizontal_equilibrium.frozen = True
        n.horizontal_takeover_yaw = 0.
        n.flight_state = State(connected=True, armed=True, mode='OFFBOARD')
        n.flight_state_received_at = at(10.)
        n.horizontal_estimator = EstimatorStatus(attitude_status_flag=True,
            velocity_vert_status_flag=True, pos_vert_abs_status_flag=True)
        n.horizontal_estimator_at = at(10.)
        n.odom_twist_in_child_frame = False
        n._mode_requested = n._flight_ready = n._safety_ok = lambda now: True
        n._horizontal_attitude = lambda now, q: q
        n._begin_departure_clearance = mock.Mock()
        n._departure_clearance_ready = lambda now: True
        n._reset_recovery_horizontal = mock.Mock()
        n._horizontal_recovery_ready = lambda now: True
        for t in (9.4, 9.6, 9.8, 10.):
            self.feedback(n, t)
        self.output(n, 10.)
        return n

    def feedback(self, n, seconds, tilt=90., vz=0., height=2.4, rate=0., source=None):
        o = Odometry(); o.header.stamp = at(seconds if source is None else source)
        for key, value in zip('xyzw', quaternion(tilt)):
            setattr(o.pose.pose.orientation, key, value)
        o.pose.pose.position.z = height
        o.twist.twist.linear.z = vz
        o.twist.twist.angular.y = math.radians(rate)
        with mock.patch.object(rospy.Time, 'now', return_value=at(seconds)):
            n._on_odom(o)
        n.flight_state_received_at = n.horizontal_estimator_at = at(seconds)
        return o

    def output(self, n, seconds, tilt=90., thrust=.74):
        target = AttitudeTarget(thrust=thrust); target.header.stamp = at(seconds)
        for key, value in zip('xyzw', quaternion(tilt)):
            setattr(target.orientation, key, value)
        n.horizontal_output = target
        n.horizontal_output_at = n.horizontal_output_owner_at = at(seconds)
        n.horizontal_output_owner = C.SOURCE_WALL

    def enter(self, n, state, seconds):
        with mock.patch.object(rospy, 'loginfo'):
            n._enter_state(state, at(seconds), 'test transition')

    def tick(self, n, seconds):
        with mock.patch.object(rospy, 'loginfo'):
            n._update_state_machine(at(seconds))

    def top(self, n, seconds, value, source=None):
        r = Range(range=value, min_range=.03, max_range=40.)
        r.header.stamp = at(seconds if source is None else source)
        with mock.patch.object(rospy.Time, 'now', return_value=at(seconds)):
            n._on_top_range(r)

    def test_enabled_admission_rejects_vz_which_would_immediately_abort(self):
        n = self.node(**{"~detach_vertical_speed_check_enabled": True})
        self.assertEqual(n.detach_max_abs_vertical_speed_mps, .2)
        for t, vz, expected in ((10.1, -.9, False), (10.2, -.21, False), (10.3, .19, True)):
            self.feedback(n, t, vz=vz)
            self.assertEqual(n._detach_pose_valid(), expected)
        with self.assertRaises(ValueError):
            self.node(**{'~detach_max_abs_vertical_speed_mps': .9})

    def test_disabled_speed_gate_allows_recorded_bias_and_relaxed_angle(self):
        n = self.node()
        self.assertFalse(n.detach_vertical_speed_check_enabled)
        self.assertAlmostEqual(math.degrees(n.detach_axis_error_rad), 15.)
        for t in (10.1, 10.3, 10.5, 10.7):
            self.feedback(n, t, tilt=78., vz=-.32)
            self.output(n, t, tilt=78.)
        self.assertTrue(n._detach_start_ready(at(10.7)))
        self.assertNotIn('竖直速度估算', n._detach_block_reason(at(10.7)))
        n.detach_requested = True; n.detach_requested_at = at(10.)
        self.tick(n, 10.7)
        self.assertEqual(n.state, W.STATE_DETACH_ROTATE)

    def test_relaxed_admission_still_rejects_large_angle_and_rate(self):
        n = self.node()
        self.feedback(n, 10.1, tilt=74., vz=-.32)
        self.assertFalse(n._detach_pose_valid())
        self.feedback(n, 10.2, tilt=90., vz=-.32, rate=26.)
        self.assertFalse(n._detach_pose_valid())

    def test_repeated_request_does_not_restart_deadline_and_expiry_precedes_ready(self):
        n = self.node()
        for t in (10., 17.):
            with mock.patch.object(rospy.Time, 'now', return_value=at(t)):
                n._on_detach(Bool(data=True))
        self.assertEqual(n.detach_requested_at, at(10.))
        n._detach_pose_ready = lambda now: True
        self.tick(n, 18.)
        self.assertEqual(n.state, W.STATE_WALL_HOLD)
        self.assertFalse(n.detach_requested)
        self.assertIn('超时', n.last_reason)

    def test_stale_top_neither_changes_pressure_nor_triggers_contact_loss(self):
        n = self.node()
        self.top(n, 10., .20)
        n.wall_pressure_thrust_command = .73
        self.assertAlmostEqual(n._wall_pressure_thrust(at(11.)), .73)
        self.tick(n, 11.)
        self.assertEqual(n.state, W.STATE_WALL_HOLD)

    def test_single_far_top_frame_waits_for_continuous_fresh_confirmation(self):
        n = self.node()
        self.top(n, 10., .20)
        self.tick(n, 10.)
        self.assertEqual(n.state, W.STATE_WALL_HOLD)
        self.top(n, 10.2, .20)
        self.tick(n, 10.2)
        self.assertEqual(n.state, W.STATE_WALL_HOLD)
        self.top(n, 10.5, .20)
        self.tick(n, 10.5)
        self.assertEqual(n.state, W.STATE_ABORT)

    def test_detach_starts_at_actual_published_q_thrust_and_locks_departure_height(self):
        n = self.node()
        self.output(n, 10., tilt=88., thrust=.74)
        self.enter(n, W.STATE_DETACH_ROTATE, 10.)
        self.assertEqual(n.state, W.STATE_DETACH_ROTATE)
        _, q, thrust, progress = n._candidate(at(10.))
        self.assertLess(quaternion_angle(q, quaternion(88.)), 1e-7)
        self.assertAlmostEqual(thrust, .74)
        self.assertEqual(progress, 0.)
        self.assertEqual(n.detach_altitude_m, 2.4)
        self.assertEqual(n.recovery_altitude_m, 2.4)
        self.assertFalse(n.recovery_wait_for_level)
        n._begin_departure_clearance.assert_called_once()

    def test_missing_final_output_keeps_pending_request_in_hold(self):
        n = self.node(); n.horizontal_output = None
        n.detach_requested = True; n.detach_requested_at = at(10.)
        self.tick(n, 10.)
        self.assertEqual(n.state, W.STATE_WALL_HOLD)
        self.assertTrue(n.detach_requested)
        self.assertIn('最终', n.last_reason)

    def test_stall_discards_excess_time_and_same_tick_is_idempotent(self):
        n = self.node(); self.enter(n, W.STATE_DETACH_ROTATE, 10.)
        for gap in (.1, .2):
            self.feedback(n, 10.+gap)
            self.output(n, 10.+gap)
            before = n.detach_elapsed_s
            first = n._candidate(at(10.+gap))
            second = n._candidate(at(10.+gap))
            self.assertEqual(first, second)
            self.assertAlmostEqual(n.detach_elapsed_s-before, .05)
            self.assertAlmostEqual(n.detach_step_dt, .05)

    def test_tracking_lag_limits_target_while_bounded_time_keeps_advancing(self):
        n = self.node(); self.enter(n, W.STATE_DETACH_ROTATE, 10.)
        for i in range(1, 101):
            t = 10.+i*.02
            self.feedback(n, t, tilt=90.)
            n._advance_detach_progress(at(t))
        q = n._quat_slerp(n.q_detach_start, n.q_hover, n._smoothstep5(n._detach_progress()))
        self.assertTrue(n.detach_tracking_limited)
        self.assertLessEqual(quaternion_angle(q, quaternion(90.)), math.radians(30.)+1e-8)
        self.assertGreater(quaternion_angle(q, quaternion(90.)), math.radians(29.))
        progress = n._detach_progress()
        self.feedback(n, 12.02, tilt=75.)
        n._advance_detach_progress(at(12.02))
        self.assertGreater(n._detach_progress(), progress)
        self.assertAlmostEqual(n.detach_step_dt, .02)

    def test_nominal_three_seconds_does_not_complete_without_measured_return(self):
        n = self.node(); self.enter(n, W.STATE_DETACH_ROTATE, 10.)
        self.feedback(n, 13.1, tilt=60.)
        self.output(n, 13.1, tilt=60.)
        self.tick(n, 13.1)
        self.assertEqual(n.state, W.STATE_DETACH_ROTATE)

    def test_recover_requires_completed_trajectory_stable_sources_and_final_output(self):
        n = self.node(); self.enter(n, W.STATE_DETACH_ROTATE, 10.)
        n.detach_elapsed_s = n.detach_time_s
        self.feedback(n, 13., tilt=0.)
        self.output(n, 13., tilt=0.)
        self.assertFalse(n._detach_rotation_ready(at(13.)))
        self.feedback(n, 13.2, tilt=0.)
        self.assertFalse(n._detach_rotation_ready(at(13.2)))
        self.feedback(n, 13.31, tilt=0.)
        self.output(n, 13.31, tilt=15.)
        self.assertFalse(n._detach_rotation_ready(at(13.31)))
        self.output(n, 13.31, tilt=0.)
        self.assertTrue(n._detach_rotation_ready(at(13.31)))
        self.tick(n, 13.31)
        self.assertEqual(n.state, W.STATE_RECOVER)
        self.assertEqual(n.recovery_altitude_m, 2.4)

    def test_duplicate_odom_cannot_fill_return_confirmation(self):
        n = self.node(); self.enter(n, W.STATE_DETACH_ROTATE, 10.)
        n.detach_elapsed_s = n.detach_time_s
        self.feedback(n, 13., tilt=0.)
        self.feedback(n, 13.31, tilt=0., source=13.)
        self.output(n, 13.31, tilt=0.)
        self.assertEqual(n.detach_recovered_samples, 1)
        self.assertFalse(n._detach_rotation_ready(at(13.31)))

    def test_return_confirmation_rejects_diagonal_tilt_and_rate_spikes(self):
        n = self.node(); self.enter(n, W.STATE_DETACH_ROTATE, 10.)
        self.feedback(n, 11., tilt=5.1)
        self.assertFalse(n._detach_rotation_pose_valid())
        self.feedback(n, 11.1, tilt=0., rate=10.1)
        self.assertFalse(n._detach_rotation_pose_valid())
        self.feedback(n, 11.2, tilt=0., rate=9.9)
        self.assertTrue(n._detach_rotation_pose_valid())

    def test_rotation_timeout_aborts_and_does_not_mark_normal_completion(self):
        n = self.node(); self.enter(n, W.STATE_DETACH_ROTATE, 10.)
        self.feedback(n, 18., tilt=60.)
        self.tick(n, 18.)
        self.assertEqual(n.state, W.STATE_ABORT)
        self.assertFalse(n.recovery_completed)
        self.assertFalse(n.detach_completed)
        self.assertIn('timeout', n.last_reason)

    def test_recovery_timeout_aborts_without_success_but_retains_height_reference(self):
        n = self.node(); self.enter(n, W.STATE_DETACH_ROTATE, 10.)
        self.enter(n, W.STATE_RECOVER, 13.)
        self.tick(n, 28.)
        self.assertEqual(n.state, W.STATE_ABORT)
        self.assertFalse(n.recovery_completed)
        self.assertEqual(n.recovery_altitude_m, 2.4)
        self.assertTrue(n.detach_vertical_active)

    def test_first_level_does_not_replace_departure_height(self):
        n = self.node(); self.enter(n, W.STATE_DETACH_ROTATE, 10.)
        self.feedback(n, 10.1, tilt=0., height=2.2)
        with mock.patch.object(rospy, 'loginfo'):
            n._prepare_vertical_reference(at(10.1))
        self.assertEqual(n.recovery_altitude_m, 2.4)
        self.assertEqual(n.vertical_reference_source, 'detach_start_height')

    def test_abort_exit_records_fault_recovery_only(self):
        n = self.node(); self.enter(n, W.STATE_ABORT, 10.)
        self.enter(n, W.STATE_EXIT, 11.)
        self.assertFalse(n.recovery_completed)
        self.assertFalse(n.detach_completed)
        self.assertTrue(n.fault_recovery_completed)
        self.tick(n, 11.02)
        self.assertIn('fault recovery', n.last_reason)

    def test_normal_exit_records_normal_completion(self):
        n = self.node(); self.enter(n, W.STATE_DETACH_ROTATE, 10.)
        self.enter(n, W.STATE_RECOVER, 13.)
        self.enter(n, W.STATE_EXIT, 15.)
        self.assertTrue(n.recovery_completed)
        self.assertTrue(n.detach_completed)
        self.assertFalse(n.fault_recovery_completed)

    def test_normal_recovery_waits_for_departure_clearance(self):
        n = self.node(); self.enter(n, W.STATE_DETACH_ROTATE, 10.)
        self.enter(n, W.STATE_RECOVER, 13.)
        self.feedback(n, 14., tilt=0.)
        n._departure_clearance_ready = lambda now: False
        self.tick(n, 14.)
        self.assertEqual(n.state, W.STATE_RECOVER)
        n._departure_clearance_ready = lambda now: True
        self.tick(n, 14.)
        self.assertEqual(n.state, W.STATE_EXIT)

    def test_offboard_exit_cancels_active_departure_clearance(self):
        n = self.node(); self.enter(n, W.STATE_DETACH_ROTATE, 10.)
        n._reset_recovery_horizontal.reset_mock()
        with mock.patch.object(rospy.Time, 'now', return_value=at(10.1)):
            n._on_state(State(connected=True, armed=True, mode='POSCTL'))
        n._reset_recovery_horizontal.assert_called_once()
        self.assertTrue(n._reset_recovery_horizontal.call_args.kwargs['clear_departure'])

    def test_backwards_clock_aborts_detach_instead_of_rewinding(self):
        n = self.node(); self.enter(n, W.STATE_DETACH_ROTATE, 10.)
        self.tick(n, 9.9)
        self.assertEqual(n.state, W.STATE_ABORT)
        self.assertIn('clock reversed', n.last_reason)
        self.assertEqual(n.recovery_altitude_m, 2.4)

    def test_fault_exit_waits_for_fresh_published_target_inside_recovery_cone(self):
        n = self.node(); self.enter(n, W.STATE_ABORT, 10.)
        n.recovery_altitude_m = 2.4
        n.recovery_wait_for_level = False
        self.feedback(n, 11., tilt=0.)
        self.output(n, 11., tilt=15.)
        self.tick(n, 11.)
        self.assertEqual(n.state, W.STATE_ABORT)
        self.output(n, 11., tilt=0.)
        self.tick(n, 11.)
        self.assertEqual(n.state, W.STATE_EXIT)
        self.assertTrue(n.fault_recovery_completed)
        self.assertFalse(n.recovery_completed)

    def test_start_readiness_includes_final_output_and_vertical_estimator(self):
        n = self.node()
        self.assertTrue(n._detach_start_ready(at(10.)))
        n.horizontal_output = None
        self.assertFalse(n._detach_start_ready(at(10.)))
        self.assertIn('最终', n._detach_block_reason(at(10.)))
        self.output(n, 10.)
        n.horizontal_estimator.velocity_vert_status_flag = False
        self.assertFalse(n._detach_start_ready(at(10.)))
        self.assertIn('竖直', n._detach_block_reason(at(10.)))
        self.assertFalse(n.detach_vertical_active)

    def test_float32_maximum_output_roundtrip_is_accepted_without_raising_cap(self):
        n = self.node()
        value = struct.unpack('f', struct.pack('f', .85))[0]
        self.assertGreater(value, n.max_thrust)
        self.output(n, 10., thrust=value)
        self.assertIs(n._valid_wall_output(at(10.)), n.horizontal_output)
        self.assertTrue(n._detach_start_ready(at(10.)))
        self.output(n, 10., thrust=n.max_thrust + 2.e-6)
        self.assertIsNone(n._valid_wall_output(at(10.)))

    def test_detach_requires_frozen_horizontal_reference_but_not_wall_facing_flow(self):
        for field, value in (('horizontal_velocity_enabled', False),
                             ('horizontal_takeover_yaw', None),
                             ('horizontal_takeover_yaw', math.nan)):
            n = self.node(); setattr(n, field, value)
            self.assertFalse(n._detach_start_ready(at(10.)))
            self.assertIn('退墙', n._detach_block_reason(at(10.)))
            self.enter(n, W.STATE_DETACH_ROTATE, 10.)
            self.assertEqual(n.state, W.STATE_WALL_HOLD)
        n = self.node(); n.horizontal_equilibrium.frozen = False
        self.assertFalse(n._detach_start_ready(at(10.)))
        n.horizontal_equilibrium.frozen = True
        n.horizontal_velocity_active = False
        n.horizontal_flow = None
        self.assertTrue(n._detach_start_ready(at(10.)))
        n.bench_wait_for_ranges = True
        n.horizontal_velocity_enabled = False
        n.horizontal_takeover_yaw = None
        self.assertTrue(n._detach_start_ready(at(10.)))

    def normal_exit(self):
        n = self.node()
        self.enter(n, W.STATE_DETACH_ROTATE, 10.)
        self.enter(n, W.STATE_RECOVER, 13.)
        self.enter(n, W.STATE_EXIT, 15.)
        return n

    def front_clearance(self, n, seconds, distance):
        # Use the real front-facing evidence gate for the EXIT reentry case.
        n.departure_active = True
        n.departure_yaw = 0.
        n.departure_level_at = at(14.)
        n.horizontal_velocity_ready = True
        n._departure_clearance_ready = lambda now: fixtures.WallDecision._departure_clearance_ready(n, now)
        self.feedback(n, seconds, tilt=0., height=2.2)
        r = Range(range=distance, min_range=.03, max_range=40.)
        r.header.stamp = at(seconds)
        with mock.patch.object(rospy.Time, 'now', return_value=at(seconds)):
            n._on_front_range(r)
            n._on_sensor_health(SensorHealth(front_valid=True))

    def test_normal_exit_distance_point_two_nine_resumes_existing_recovery(self):
        n = self.normal_exit()
        vertical = n.detach_vertical_controller
        horizontal = n.recovery_horizontal_controller
        vertical.integral = .02
        horizontal.integral = (.03, -.01)
        n._reset_recovery_horizontal.reset_mock()
        self.front_clearance(n, 15.02, .29)
        self.tick(n, 15.02)
        self.assertEqual(n.state, W.STATE_RECOVER)
        self.assertFalse(n.detach_completed or n.recovery_completed)
        self.assertFalse(n.fault_recovery_completed)
        self.assertEqual(n.recovery_started_at, at(13.))
        self.assertEqual(n.recovery_altitude_m, 2.4)
        self.assertIs(n.detach_vertical_controller, vertical)
        self.assertEqual(vertical.reference, 2.4)
        self.assertAlmostEqual(vertical.base, .518)
        self.assertAlmostEqual(vertical.integral, .02)
        self.assertIs(n.recovery_horizontal_controller, horizontal)
        self.assertEqual(horizontal.integral, (.03, -.01))
        n._begin_departure_clearance.assert_called_once()
        n._reset_recovery_horizontal.assert_not_called()

    def test_reentry_does_not_restart_total_recovery_deadline(self):
        n = self.normal_exit()
        n._departure_clearance_ready = lambda now: False
        self.tick(n, 27.9)
        self.assertEqual(n.state, W.STATE_RECOVER)
        self.assertEqual(n.state_entered_at, at(27.9))
        self.assertEqual(n.recovery_started_at, at(13.))
        self.tick(n, 28.)
        self.assertEqual(n.state, W.STATE_ABORT)
        self.assertIn('recovery timeout', n.last_reason)
        self.assertFalse(n.detach_completed or n.recovery_completed)
        self.assertEqual(n.recovery_altitude_m, 2.4)

    def test_expired_exit_with_lost_clearance_aborts_without_reentry(self):
        n = self.normal_exit()
        n._departure_clearance_ready = lambda now: False
        self.tick(n, 28.)
        self.assertEqual(n.state, W.STATE_ABORT)
        self.assertIn('timeout after departure clearance lost', n.last_reason)
        self.assertFalse(n.detach_completed or n.recovery_completed)
        self.assertEqual(n.recovery_started_at, at(13.))

    def test_fault_and_bench_exit_do_not_restart_normal_retreat(self):
        n = self.node()
        self.enter(n, W.STATE_ABORT, 10.)
        self.enter(n, W.STATE_EXIT, 11.)
        n._departure_clearance_ready = mock.Mock(return_value=False)
        self.tick(n, 30.)
        self.assertEqual(n.state, W.STATE_EXIT)
        self.assertTrue(n.fault_recovery_completed)
        n._departure_clearance_ready.assert_not_called()
        n = self.normal_exit()
        n.bench_wait_for_ranges = True
        n._departure_clearance_ready = mock.Mock(return_value=False)
        self.tick(n, 30.)
        self.assertEqual(n.state, W.STATE_EXIT)
        n._departure_clearance_ready.assert_not_called()

    def test_new_detach_and_idle_clear_total_recovery_deadline(self):
        n = self.normal_exit()
        self.enter(n, W.STATE_WALL_HOLD, 20.)
        self.feedback(n, 20.)
        self.output(n, 20.)
        self.enter(n, W.STATE_DETACH_ROTATE, 20.)
        self.assertEqual(n.state, W.STATE_DETACH_ROTATE)
        self.assertIsNone(n.recovery_started_at)
        self.enter(n, W.STATE_RECOVER, 23.)
        self.assertEqual(n.recovery_started_at, at(23.))
        self.enter(n, W.STATE_IDLE, 24.)
        self.assertIsNone(n.recovery_started_at)


if __name__ == '__main__':
    unittest.main()
