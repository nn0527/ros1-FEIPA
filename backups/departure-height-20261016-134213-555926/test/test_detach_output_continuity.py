#!/usr/bin/env python3
"""Exercise the actual mocked MAVROS publication boundary, without ROS output."""
import math
import struct
import unittest
from unittest import mock

import rospy
from mavros_msgs.msg import AttitudeTarget, EstimatorStatus, State
from nav_msgs.msg import Odometry
from px4_wall_ceiling_control.msg import OperatorCommand, WallPerchStatus as Wall
import test_candidate_latency as fixture
import test_flight_handover as handover_fixture


class DetachOutputContinuityTest(unittest.TestCase):
    def setUp(self):
        self.fx = fixture.CandidateLatencyTest()
        self.fx.setUp()
        self.addCleanup(self.fx.doCleanups)
        self.node = self.fx.node
        self.fx.accept(self.fx.candidate())
        self.sequence = 1

    def command(self, phase, angle=0., thrust=.72, support=False, horizontal=False):
        self.sequence += 1
        candidate = self.fx.candidate(phase=phase, sequence=self.sequence, acquire=False, angle=angle)
        candidate.thrust_normalized = thrust
        candidate.horizontal_velocity_active = horizontal
        candidate.recovery_support_valid = support
        candidate.recovery_thrust_limit = .85 if support else 0.
        self.fx.accept(candidate)
        return candidate

    def publish(self, candidate):
        self.assertEqual(self.node._publish_candidate(self.fx.now, candidate), fixture.Mux.OUTPUT_ATTITUDE_THRUST,
                         self.node.wall_candidate_rejection)
        return self.node.attitude_publisher.publish.call_args.args[0]

    def test_hold_to_detach_keeps_actual_pressure_and_angle_continuous(self):
        self.fx.seed_publication(angle=90., thrust=.75)
        self.fx.advance()
        target = self.publish(self.command(Wall.STATE_DETACH_ROTATE, angle=80., thrust=.72))
        self.assertAlmostEqual(fixture.tilt(target), 90.-56.25*.02)
        self.assertAlmostEqual(target.thrust, .74)
        self.assertEqual(self.node.last_published_control[2], self.fx.now)

    def test_mux_stalls_with_fresh_detach_candidate_keep_shared_dt_cap(self):
        for delay in (.10, .20):
            with self.subTest(delay=delay):
                self.fx.seed_publication(angle=60., thrust=.85)
                self.fx.advance(delay)
                target = self.publish(self.command(Wall.STATE_DETACH_ROTATE, angle=30., thrust=.6))
                self.assertAlmostEqual(fixture.tilt(target), 60.-56.25*.05)
                self.assertAlmostEqual(target.thrust, .825)
                self.assertAlmostEqual(self.node.last_wall_command['publication_slew']['dt_s'], .05)
                self.assertEqual(self.node.last_wall_command['attitude_xyzw'],
                                 [getattr(target.orientation, k) for k in 'xyzw'])

    def test_missing_horizontal_ready_does_not_bypass_detach_slew(self):
        self.fx.seed_publication(angle=40., thrust=.75)
        self.fx.advance(.20)
        candidate = self.command(Wall.STATE_DETACH_ROTATE, angle=0., horizontal=False)
        candidate.horizontal_velocity_enabled = False
        target = self.publish(candidate)
        self.assertAlmostEqual(fixture.tilt(target), 40.-56.25*.05)

    def test_detach_near_level_projection_cannot_jump_actual_large_angle(self):
        self.fx.seed_publication(angle=30., thrust=.75)
        self.fx.advance()
        target = self.publish(self.command(Wall.STATE_DETACH_ROTATE, angle=60., horizontal=True))
        self.assertAlmostEqual(fixture.tilt(target), 30.-56.25*.02)
        preview = self.node.recovery_target_publisher.publish.call_args.args[0]
        self.assertAlmostEqual(preview.vector.x, fixture.tilt(target))
        self.assertAlmostEqual(preview.vector.z, target.thrust)

    def test_recover_and_exit_use_published_history_not_unsent_helper_history(self):
        self.fx.seed_publication(angle=4., thrust=.75)
        for phase in (Wall.STATE_RECOVER, Wall.STATE_EXIT):
            before = self.node.last_published_control
            self.node.recovery_previous_target = (0., 0., 0., 1.)
            self.node.recovery_previous_at = self.fx.now
            self.fx.advance()
            target = self.publish(self.command(phase, angle=-5., thrust=.74, support=True))
            self.assertAlmostEqual(fixture.tilt(target), math.degrees(math.acos(1.-2.*before[0][1]**2))-.2)
            self.assertGreater(target.thrust, .60)
            self.assertLessEqual(abs(target.thrust-before[1]), .010001)

    def test_normal_recovery_rejects_phase_entry_from_actual_large_axis(self):
        for phase in (Wall.STATE_RECOVER, Wall.STATE_EXIT):
            self.fx.seed_publication(angle=30., thrust=.8)
            before = self.node.last_published_control
            self.fx.advance()
            candidate = self.command(phase, support=True)
            self.assertEqual(self.node._publish_candidate(self.fx.now, candidate), fixture.Mux.OUTPUT_NONE)
            self.assertEqual(self.node.wall_candidate_rejection, 'recovery_entry_published_axis_outside_cone')
            self.assertIs(self.node.last_published_control, before)
        self.node.attitude_publisher.publish.assert_not_called()

    def test_support_cap_disappearance_ramps_down_instead_of_cutting_to_point_six(self):
        self.fx.seed_publication(angle=1., thrust=.8)
        self.fx.advance()
        target = self.publish(self.command(Wall.STATE_RECOVER, thrust=.8, support=False))
        self.assertAlmostEqual(target.thrust, .79)
        self.assertEqual(self.node.last_wall_command['publication_slew']['requested_thrust'], .60)

    def test_abort_levels_immediately_but_keeps_support_thrust_continuous(self):
        self.fx.seed_publication(angle=80., thrust=.84)
        self.fx.advance()
        target = self.publish(self.command(Wall.STATE_ABORT, angle=20., thrust=.49))
        self.assertLessEqual(fixture.tilt(target), 5.000001)
        self.assertAlmostEqual(target.thrust, .83)
        self.assertTrue(self.node.last_wall_command['publication_slew']['abort_level_immediate'])

    def test_abort_without_history_does_not_suppress_valid_safety_command(self):
        self.fx.advance()
        target = self.publish(self.command(Wall.STATE_ABORT, angle=30., thrust=.49))
        self.assertLessEqual(fixture.tilt(target), 5.000001)
        self.assertAlmostEqual(target.thrust, .49)
        self.assertFalse(self.node.last_wall_command['publication_slew']['seed_available'])

    def test_hard_maximum_has_priority_over_continuity_even_in_abort(self):
        self.fx.seed_publication(angle=40., thrust=.95)
        self.fx.advance()
        target = self.publish(self.command(Wall.STATE_ABORT, thrust=.9, support=True))
        self.assertLessEqual(target.thrust, self.node.max_thrust)

    def test_invalid_support_allowance_is_rejected(self):
        self.fx.advance()
        for value in (float('nan'), float('inf'), -.1, .90):
            candidate = self.fx.candidate(sequence=2, phase=Wall.STATE_RECOVER, acquire=False)
            candidate.recovery_support_valid = True
            candidate.recovery_thrust_limit = value
            self.node._on_wall_candidate(candidate)
            self.assertEqual(self.node.wall_candidate_rejection, 'invalid_recovery_support_limit')

    def test_expired_candidate_does_not_commit_any_continuity_history(self):
        self.fx.seed_publication(angle=4., thrust=.8)
        before = self.node.last_published_control
        previous_q, previous_at = self.node.recovery_previous_target, self.node.recovery_previous_at
        self.fx.advance()
        candidate = self.command(Wall.STATE_RECOVER, support=True)
        original = self.node._limit_wall_publication
        def delayed(*args):
            result = original(*args)
            self.fx.advance(.21)
            return result
        with mock.patch.object(self.node, '_limit_wall_publication', side_effect=delayed):
            self.assertEqual(self.node._publish_candidate(self.fx.now, candidate), fixture.Mux.OUTPUT_NONE)
        self.node.attitude_publisher.publish.assert_not_called()
        self.node.recovery_target_publisher.publish.assert_not_called()
        self.assertIs(self.node.last_published_control, before)
        self.assertEqual((self.node.recovery_previous_target, self.node.recovery_previous_at), (previous_q, previous_at))

    def test_offboard_fcu_callback_resumes_actual_output_without_relearning_handover(self):
        n = self.node
        pilot_target = AttitudeTarget(orientation=fixture.tilted(1.), thrust=.52)
        pilot_target.header.stamp = self.fx.now
        n.fcu_attitude_target, n.fcu_attitude_target_at = pilot_target, self.fx.now
        self.fx.seed_publication(angle=30., thrust=.75)
        self.fx.advance(.6)
        n.odom.header.stamp = n.odom_at = self.fx.now
        n.odom.pose.pose.orientation = fixture.tilted(4.)
        echo = AttitudeTarget(orientation=fixture.tilted(5.), thrust=.64)
        echo.header.stamp = self.fx.now
        n._on_fcu_attitude_target(echo)
        self.assertIs(n.fcu_attitude_target, pilot_target)
        self.assertEqual(n.fcu_attitude_target_at, rospy.Time.from_sec(10.))
        target = self.publish(self.command(Wall.STATE_DETACH_ROTATE, angle=0., thrust=.72))
        self.assertAlmostEqual(target.thrust, .64)
        self.assertAlmostEqual(fixture.tilt(target), 4.)
        self.fx.advance()
        target = self.publish(self.command(Wall.STATE_DETACH_ROTATE, angle=0., thrust=.72))
        self.assertAlmostEqual(target.thrust, .65)
        self.assertEqual(n.last_published_control[2], self.fx.now)

    def test_fresh_receipt_cannot_revive_stale_offboard_echo(self):
        n = self.node
        self.fx.seed_publication(angle=0., thrust=.75)
        self.fx.advance(.6)
        n.odom.header.stamp = n.odom_at = self.fx.now
        echo = AttitudeTarget(orientation=fixture.tilted(0.), thrust=.64)
        echo.header.stamp = rospy.Time.from_sec(10.)
        n._on_fcu_attitude_target(echo)
        candidate = self.command(Wall.STATE_DETACH_ROTATE)
        self.assertEqual(n._publish_candidate(self.fx.now, candidate), fixture.Mux.OUTPUT_NONE)
        self.assertEqual(n.wall_candidate_rejection, 'final_slew_seed_unavailable')
        n.attitude_publisher.publish.assert_not_called()

    def test_serialized_maximum_echo_is_clamped_and_retired_on_mode_exit(self):
        n = self.node
        self.fx.advance(.02)
        n.odom.header.stamp = n.odom_at = self.fx.now
        echo = AttitudeTarget(orientation=fixture.tilted(0.),
                              thrust=struct.unpack('f', struct.pack('f', .85))[0])
        echo.header.stamp = self.fx.now
        n._on_fcu_attitude_target(echo)
        target = self.publish(self.command(Wall.STATE_DETACH_ROTATE, thrust=.85))
        self.assertEqual(target.thrust, n.max_thrust)
        n._on_state(State(connected=True, armed=True, mode='POSCTL'))
        self.assertIsNone(n.wall_continuity_fcu_snapshot)


class WallReturnOutcomeTest(unittest.TestCase):
    def setUp(self):
        self.now = rospy.Time.from_sec(10.)
        self.node = n = handover_fixture.Manager.__new__(handover_fixture.Manager)
        n.selected_mode = OperatorCommand.MODE_WALL
        n.owner = 2
        n.owner_at = n.wall_at = self.now
        n.owner_seen = n.detach_seen = n.recovery_seen = n.completed = n.fault_seen = False
        n.wall_fault_return_ready = False
        n.wall_failure_reason = ''
        n.output_timeout_s = n.status_timeout_s = .3
        n.wall = Wall()
        n._pilot_mode = mock.Mock(return_value='POSCTL')

    def test_normal_result_requires_both_explicit_milestones(self):
        n = self.node
        for phase in (Wall.STATE_DETACH_ROTATE, Wall.STATE_RECOVER, Wall.STATE_EXIT):
            n.wall.state = phase
            n._observe_completion(self.now)
        self.assertFalse(n.completed)
        n.wall.recovery_completed = True
        n._observe_completion(self.now)
        self.assertFalse(n.completed)
        n.wall.detach_completed = True
        n._observe_completion(self.now)
        self.assertTrue(n.completed)
        self.assertFalse(n.wall_fault_return_ready)

    def test_abort_exit_is_fault_return_and_never_success(self):
        n = self.node
        n.wall.state = Wall.STATE_ABORT
        n.wall.reason = '脱离超时'
        n._observe_completion(self.now)
        n.wall.state = Wall.STATE_EXIT
        n.wall.fault_recovery_completed = True
        n.wall.recovery_completed = n.wall.detach_completed = True  # Even inconsistent success flags cannot override fault.
        n._observe_completion(self.now)
        self.assertTrue(n.wall_fault_return_ready)
        self.assertFalse(n.completed)
        self.assertIn('故障', n._wall_return_reason(confirmed=True))
        self.assertIn('脱离超时', n._wall_return_reason(confirmed=True))

    def test_missed_abort_sample_still_yields_fault_result_from_explicit_flag(self):
        n = self.node
        n.wall.state = Wall.STATE_EXIT
        n.wall.fault_recovery_completed = True
        n.wall.reason = '故障回平完成'
        n._observe_completion(self.now)
        self.assertTrue(n.wall_fault_return_ready)
        self.assertFalse(n.completed)

    def test_fault_return_keeps_stream_until_posctl_confirmation_and_preserves_result(self):
        n = self.node
        n.phase = n.ACTIVE
        n.flight = State(mode='OFFBOARD', armed=True, connected=True)
        n.flight_at = self.now
        n.state_timeout_s = 1.5
        n.wall.state = Wall.STATE_EXIT
        n.wall.fault_recovery_completed = True
        n.wall.reason = '脱离超时后故障回平'
        n._requested_mode = mock.Mock(return_value=OperatorCommand.MODE_WALL)
        n._handover_ready = mock.Mock(return_value=True)
        n._request_mode = mock.Mock()
        n.prestream_pub = mock.Mock()
        n.phase_pub = mock.Mock()
        n.phase_reason_pub = mock.Mock()
        with mock.patch.object(rospy.Time, 'now', return_value=self.now), mock.patch.object(rospy, 'loginfo'):
            n._on_timer(None)
            self.assertEqual(n.phase, n.RETURNING)
            self.assertFalse(n.completed)
            self.assertIn('故障', n.phase_reason)
            n._on_timer(None)
            n._request_mode.assert_called_once_with('POSCTL')
            self.assertTrue(n.prestream_pub.publish.call_args.args[0].data)
            n.flight.mode = 'POSCTL'
            n._on_timer(None)
        self.assertEqual(n.phase, n.LOCKED)
        self.assertIn('故障', n.phase_reason)
        self.assertIn('已安全交回POSCTL', n.phase_reason)
        self.assertFalse(n.prestream_pub.publish.call_args.args[0].data)

    def test_centering_disabled_still_requires_fresh_valid_rc(self):
        n = self.node
        n.output_mode = 2
        n.output_at = n.estimator_at = n.odom_at = self.now
        n.estimator_timeout_s = n.odom_timeout_s = .5
        n.estimator = EstimatorStatus(attitude_status_flag=True, velocity_vert_status_flag=True)
        n.odom = Odometry(); n.odom.pose.pose.orientation.w = 1.
        n.odom_twist_in_child_frame = True
        n.handover_max_tilt_deg = 10.
        n.handover_max_vertical_speed_mps = .2
        n.require_rc_center = False
        n.rc_timeout_s = .2
        n.rc_at = self.now
        n.rc_channels = (1200, 1500, 1700, 1500)
        n._wall_horizontal_ready = mock.Mock(return_value=True)
        self.assertTrue(n._handover_ready(self.now))
        n.rc_at = rospy.Time.from_sec(9.)
        self.assertFalse(n._handover_ready(self.now))
        n.rc_at = self.now
        n.rc_channels = (1500, 1500, 0, 1500)
        self.assertFalse(n._handover_ready(self.now))

    def test_revoked_exit_blocks_timer_and_service_until_new_valid_exit(self):
        n = self.node
        n.phase = n.ACTIVE
        n.flight = State(mode='OFFBOARD', armed=True, connected=True)
        n.flight_at = self.now
        n.state_timeout_s = 1.5
        n.wall.state = Wall.STATE_EXIT
        n.wall.detach_completed = n.wall.recovery_completed = True
        n._requested_mode = mock.Mock(return_value=OperatorCommand.MODE_WALL)
        n._request_mode = mock.Mock()
        n.set_mode = mock.Mock()
        n.prestream_pub = mock.Mock()
        n.phase_pub = mock.Mock()
        n.phase_reason_pub = mock.Mock()
        n.output_mode = 2
        n.output_at = n.estimator_at = n.odom_at = self.now
        n.estimator_timeout_s = n.odom_timeout_s = .5
        n.estimator = EstimatorStatus(attitude_status_flag=True, velocity_vert_status_flag=True)
        n.odom = Odometry(); n.odom.pose.pose.orientation.w = 1.
        n.odom_twist_in_child_frame = True
        n.handover_max_tilt_deg = 10.
        n.handover_max_vertical_speed_mps = .2
        n.require_rc_center = False
        n.rc_timeout_s = .2
        n.rc_at = self.now
        n.rc_channels = (1500, 1500, 1500, 1500)
        n._wall_horizontal_ready = mock.Mock(return_value=True)
        with mock.patch.object(rospy.Time, 'now', return_value=self.now), mock.patch.object(rospy, 'loginfo'):
            n._on_timer(None)
            self.assertEqual(n.phase, n.RETURNING)
            self.assertTrue(n.completed)
            n.wall.state = Wall.STATE_RECOVER
            n.wall.detach_completed = n.wall.recovery_completed = False
            # A callback may revoke EXIT after scheduling but before the mode
            # service worker runs, before the next manager timer observes it.
            with mock.patch.object(rospy, 'wait_for_service'):
                n._send_mode_request('POSCTL')
            n.set_mode.assert_not_called()
            n._on_timer(None)
            self.assertFalse(n.completed)
            self.assertEqual(n.phase, n.RETURNING)
            n._request_mode.assert_not_called()
            self.assertTrue(n.prestream_pub.publish.call_args.args[0].data)
            n.wall.state = Wall.STATE_EXIT
            n.wall.detach_completed = n.wall.recovery_completed = True
            n.wall_at = rospy.Time.from_sec(9.)
            n._on_timer(None)
            n._request_mode.assert_not_called()
            n.wall_at = self.now
            n._on_timer(None)
            n._request_mode.assert_called_once_with('POSCTL')
            self.assertTrue(n.completed)
            self.assertTrue(n.prestream_pub.publish.call_args.args[0].data)


if __name__ == '__main__':
    unittest.main()
