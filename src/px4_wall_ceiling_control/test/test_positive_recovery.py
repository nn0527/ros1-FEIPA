"""Signed detach branching, captured recovery and asynchronous PX4 landing.

All ROS transports and mode RPCs are mocked; these tests cannot command a vehicle.
"""
import math
import unittest
from types import SimpleNamespace
from unittest import mock
import rospy
from mavros_msgs.msg import State, AttitudeTarget, OpticalFlowRad
from geometry_msgs.msg import Quaternion
from px4_wall_ceiling_control.msg import WallPerchStatus as W
from px4_wall_ceiling_control.domain.simple_recovery import within_margins, attitude_within_margins
from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion
from px4_wall_ceiling_control.adapters.wall_horizontal_velocity import WallHorizontalVelocityMixin
import test_range_height_integration as fixtures
import test_candidate_latency as mux_fixtures
from test_detach_state_machine import at, quaternion
import test_flight_handover as manager_module


class PositiveRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.fx = fixtures.RangeDetachNodeTests()
        self.fx.setUp()
        self.addCleanup(self.fx.doCleanups)
        self.n = self.fx.n

    def finish(self, pitch):
        n = self.n
        self.fx.seconds = 13.
        self.fx.fx.feedback(n, 13., tilt=pitch, rate=18., height=math.nan, vz=math.nan)
        self.fx.fx.output(n, 13., tilt=0.)
        n._advance_detach_progress = lambda now: None
        n._detach_progress = lambda: 1.
        n._update_state_machine(at(13.))

    def test_entry_boundaries_capture_actual_not_command_without_dwell(self):
        for pitch in (0., 1., 4.9, 5., 8., 9.9, 9.999999):
            with self.subTest(pitch=pitch):
                self.n = self.fx.new_node()
                self.finish(pitch)
                self.assertEqual(self.n.state, W.STATE_RECOVER)
                self.assertTrue(self.n.recover_attitude_locked)
                for actual, expected in zip(self.n.recover_entry_attitude, quaternion(pitch)):
                    self.assertAlmostEqual(actual, expected, places=14)
                self.assertFalse(self.n.abort_land_requested)

    def test_entry_ten_or_more_waits(self):
        for pitch in (10., 10.000001, 12.):
            with self.subTest(pitch=pitch):
                self.n = self.fx.new_node()
                self.finish(pitch)
                self.assertEqual(self.n.state, W.STATE_DETACH_ROTATE)
                self.assertFalse(self.n.recover_attitude_locked)

    def test_entry_uses_total_tilt_not_pitch_alone(self):
        from px4_wall_ceiling_control.domain.geometry import quaternion_multiply
        n = self.n
        self.finish(4.)
        # A small pitch cannot admit an aircraft whose roll puts its thrust
        # axis outside the ten-degree cone.
        self.n = n = self.fx.new_node()
        self.fx.seconds = 13.
        odom = self.fx.fx.feedback(n, 13., tilt=4.)
        roll = (math.sin(math.radians(10.)/2.), 0., 0., math.cos(math.radians(10.)/2.))
        q = quaternion_multiply(quaternion(4.), roll)
        for name, value in zip('xyzw', q):
            setattr(odom.pose.pose.orientation, name, value)
        self.fx.fx.output(n, 13., tilt=0.)
        n._advance_detach_progress = lambda now: None
        n._detach_progress = lambda: 1.
        n._update_state_machine(at(13.))
        self.assertEqual(n.state, W.STATE_DETACH_ROTATE)
        self.assertFalse(n.recover_attitude_locked)

    def test_negative_only_after_trajectory_completion(self):
        n = self.n
        self.fx.seconds = 10.2
        self.fx.fx.feedback(n, 10.2, tilt=-1.)
        self.fx.fx.output(n, 10.2, tilt=1.)
        n._update_state_machine(at(10.2))
        self.assertEqual(n.state, W.STATE_DETACH_ROTATE)
        self.finish(-.01)
        self.assertEqual(n.state, W.STATE_ABORT)
        self.assertTrue(n.abort_land_requested)
        n._detach_height_settled = mock.Mock(side_effect=AssertionError('landing waited for height'))
        n._update_state_machine(at(13.1))
        self.assertEqual(n.state, W.STATE_ABORT)

    def test_negative_result_lands_even_when_range_has_faulted(self):
        self.n.detach_vertical_fault = 'range unavailable'
        self.finish(-1.)
        self.assertTrue(self.n.abort_land_requested)
        self.assertEqual(self.n.state, W.STATE_ABORT)

    def test_land_mode_stops_candidate_even_with_no_range(self):
        self.finish(-1.)
        n = self.n
        n._flight_ready = lambda now: False
        result = n._candidate(at(13.1))
        self.assertIsNone(result[1])
        self.assertTrue(math.isnan(result[2]))

    def test_recover_keeps_heading_initialization_without_automatic_retreat(self):
        n = self.n
        n._begin_departure_clearance.assert_not_called()
        self.finish(4.)
        n._begin_departure_clearance.assert_called_once_with(at(13.))
        self.assertAlmostEqual(n.departure_yaw, 0.)

    def test_small_negative_during_recover_does_not_trigger_landing(self):
        self.finish(3.)
        self.fx.frame(13.1, angle=-1.)
        self.n._update_state_machine(at(13.1))
        self.assertEqual(self.n.state, W.STATE_RECOVER)
        self.assertFalse(self.n.abort_land_requested)

    def test_attitude_stability_uses_new_poses_not_range_or_speed_convergence(self):
        self.finish(5.)
        n = self.n
        n.recover_height_captured = True  # Isolate stability from height admission.
        n._prepare_simple_recovery_velocity = lambda now: None
        n.horizontal_velocity_ready = True
        n.horizontal_measured = (.20, 0.)
        n._departure_clearance_ready = lambda now: True
        n.recovery_horizontal_controller.target = (0., 0.)
        snapshot = SimpleNamespace(valid=True, target=1., height=.82, source=13.)
        n.detach_range.read_snapshot = lambda now: snapshot
        n._range_control_velocity = lambda s, now: (.23, True)
        for t, expected in ((13., False), (13., False), (13.25, False), (13.51, True)):
            snapshot.source = t
            self.fx.fx.feedback(n, t, tilt=8., rate=18., height=math.nan, vz=math.nan)
            self.fx.fx.output(n, t, tilt=8.)
            self.assertEqual(n._simple_recovery_stability(at(t)), expected)
        self.assertEqual(n.detach_range_stable_samples, 3)
        n.horizontal_measured = (2., 0.)
        snapshot.valid = False
        snapshot.height = math.nan
        n._range_control_velocity = mock.Mock(side_effect=AssertionError('range velocity used'))
        self.assertTrue(n._simple_recovery_stability(at(13.51)))
        self.fx.fx.feedback(n, 13.52, tilt=10.1, rate=18.)
        self.fx.fx.output(n, 13.52, tilt=8.)
        self.assertFalse(n._simple_recovery_stability(at(13.52)))
        self.assertFalse(n.recover_stable)

    def test_attitude_handover_never_reads_range_and_resets_on_rate_excursion(self):
        self.finish(5.)
        n = self.n
        n.detach_range.read_snapshot = mock.Mock(side_effect=AssertionError('range snapshot used'))
        n._prepare_simple_recovery_velocity = mock.Mock(side_effect=AssertionError('flow required'))
        for t, rate, expected in ((13., 18., False), (13.25, 18., False),
                                  (13.51, 21., False), (13.6, 18., False),
                                  (13.85, 18., False), (14.11, 18., True)):
            self.fx.fx.feedback(n, t, tilt=8., rate=rate)
            self.fx.fx.output(n, t, tilt=8.)
            self.assertEqual(n._simple_recovery_stability(at(t)), expected)

    def test_duplicate_and_stale_pose_cannot_fill_attitude_dwell(self):
        self.finish(5.)
        n = self.n
        self.fx.fx.feedback(n, 13., tilt=8., rate=18.)
        self.fx.fx.output(n, 13., tilt=8.)
        self.assertFalse(n._simple_recovery_stability(at(13.)))
        self.fx.fx.output(n, 13.2, tilt=8.)
        self.assertFalse(n._simple_recovery_stability(at(13.2)))
        self.assertEqual(n.detach_range_stable_samples, 1)
        self.fx.fx.output(n, 14., tilt=8.)
        self.assertFalse(n._simple_recovery_stability(at(14.)))
        self.assertEqual(n.detach_range_stable_samples, 0)

    def test_range_pd_remains_active_in_recover_without_local_z(self):
        self.fx.frame(10.2, distance=1.)
        self.fx.frame(10.3, distance=1.)
        self.finish(5.)
        self.fx.frame(13.1, distance=.9, angle=5., height=math.nan, vz=math.nan)
        n = self.n
        with mock.patch.object(n, '_height_feedback', side_effect=AssertionError('local Z used')):
            thrust = n._detach_vertical_thrust(at(13.1), .02)
        self.assertTrue(math.isfinite(thrust))
        self.assertGreater(n.detach_vertical_controller.correction, 0.)
        self.assertAlmostEqual(n.detach_vertical_controller.reference, 1.)

    def test_fresh_flow_accepts_four_point_nine_without_reacquire_or_tilt_guard(self):
        self.finish(4.9)
        n = self.n
        n._wall_fusion_ready = lambda now: True
        n.horizontal_estimator.velocity_horiz_status_flag = True
        n._update_departure_clearance = lambda now: None
        flow = OpticalFlowRad(distance=1., quality=100, integration_time_us=20000)
        flow.header.stamp = at(13.)
        n.horizontal_flow_snapshot = (flow, at(13.), '')
        n.horizontal_estimator.header.stamp = at(13.)
        n._prepare_simple_recovery_velocity(at(13.))
        self.assertTrue(n.horizontal_velocity_ready)
        n.horizontal_flow_snapshot = (flow, at(12.), '')
        n._prepare_simple_recovery_velocity(at(13.))
        self.assertFalse(n.horizontal_velocity_ready)

    def test_rotation_end_target_is_fixed_despite_velocity_and_baseline_changes(self):
        self.finish(4.9)
        n = self.n
        n.recover_height_captured = True
        n.recovery_horizontal_controller.attitude = mock.Mock(side_effect=AssertionError('attitude changed'))
        expected = n.recover_entry_attitude
        for t, velocity in ((13., (0., 0.)), (14., (2., -.5)), (120., (-1., 1.))):
            n.horizontal_measured = velocity
            n.horizontal_equilibrium.frozen = False
            self.assertEqual(n._simple_recovery_attitude(at(t)), expected)
            self.assertFalse(n.horizontal_velocity_active)
            self.assertEqual(n.recovery_horizontal_controller.target, (0., 0.))

    def test_complete_recovery_without_retreat_or_clearance_reaches_exit(self):
        from sensor_msgs.msg import Range
        from px4_wall_ceiling_control.msg import SensorHealth
        n = self.n
        for name in ('_begin_departure_clearance', '_reset_recovery_horizontal',
                     '_departure_clearance_ready', '_horizontal_attitude'):
            delattr(n, name)  # Exercise the actual adapters, not fixture stubs.
        n._wall_fusion_ready = lambda now: True
        n.horizontal_estimator.velocity_horiz_status_flag = True
        self.finish(4.9)
        self.assertTrue(n.departure_active)
        saw_retreat = False
        for i in range(1, 26):
            t = 13.+i*.1
            self.fx.frame(t, distance=1., angle=8., height=math.nan, vz=math.nan)
            n.odom.twist.twist.angular.y = math.radians(18.)
            n.odom.twist.twist.linear.x = .20
            n.horizontal_estimator.header.stamp = at(t)
            flow = OpticalFlowRad(distance=1., quality=100, integration_time_us=20000)
            flow.header.stamp = at(t)
            n.horizontal_flow_snapshot = (flow, at(t), '')
            front = Range(range=.1, min_range=.03, max_range=40.)
            front.header.stamp = at(t)
            n.front_range = front
            n.front_received_at = at(t)
            n.front_filtered_m = front.range
            n.sensor_health = SensorHealth(front_valid=True)
            n.sensor_health_received_at = at(t)
            n._update_state_machine(at(t))
            n._candidate(at(t))
            saw_retreat |= n.recovery_horizontal_controller.target[0] < 0.
            if n.state == W.STATE_EXIT and n.recover_stable:
                break
        self.assertFalse(saw_retreat)
        self.assertEqual(n.state, W.STATE_EXIT)
        self.assertTrue(n.recover_stable)
        self.assertFalse(n.departure_clear_confirmed)
        n.flight_state.mode = 'POSCTL'
        n.flight_state.header.stamp = n.flight_state_received_at = at(t+.1)
        self.fx.seconds = t+.1
        n._update_state_machine(at(t+.1))
        self.assertEqual(n.state, W.STATE_IDLE)
        self.assertTrue(n.detach_completed)
        self.assertIsNotNone(n.detach_range_posctl_confirmed_at)

    def test_mux_keeps_captured_eight_degrees_in_both_output_limits(self):
        fx = mux_fixtures.CandidateLatencyTest()
        fx.setUp(); self.addCleanup(fx.doCleanups)
        n = fx.node
        candidate = fx.candidate(phase=W.STATE_RECOVER, angle=8.)
        candidate.recover_attitude_locked = True
        target = AttitudeTarget(thrust=.5, orientation=candidate.attitude_setpoint)
        target.header.stamp = fx.now
        n._remember_publication(target, candidate.SOURCE_WALL, fx.now)
        fx.advance(.02)
        bounded = n._limit_final_recovery_target(fx.now, target, candidate.SOURCE_WALL, candidate)
        self.assertAlmostEqual(mux_fixtures.tilt(bounded), 8.)
        bounded = n._limit_wall_publication(fx.now, bounded, candidate)
        self.assertIsNotNone(bounded)
        self.assertAlmostEqual(mux_fixtures.tilt(bounded), 8.)

    def test_exit_stays_exit_when_stability_wobbles_and_revokes_handover(self):
        self.fx.frame(12.9, distance=1., angle=5.)
        self.finish(5.)
        n = self.n
        n.state = W.STATE_EXIT
        n._range_stability_ready = mock.Mock(return_value=False)
        n.flight_state.header.stamp = at(13.)
        n._update_state_machine(at(13.))
        self.assertEqual(n.state, W.STATE_EXIT)
        n._range_stability_ready.assert_called_once()


class LandingManagerTests(unittest.TestCase):
    def setUp(self):
        self.fx = fixtures.RangeModeManagerTests()
        self.fx.setUp(); self.addCleanup(self.fx.doCleanups)
        n = self.n = self.fx.n
        self.now = at(10.)
        n.wall.state = W.STATE_ABORT
        n.wall.abort_land_requested = True
        n.wall.detach_range_maneuver_id = 'land-test'
        n.flight = State(connected=True, armed=True, mode='OFFBOARD')
        n.flight.header.stamp = n.flight_at = self.now
        n.state_timeout_s = 1.
        n.phase = n.ACTIVE
        n.phase_reason = ''
        n.takeover_attempt = 3
        n.wall_pilot_mode = 'POSCTL'
        n.prestream_pub = n.phase_pub = n.phase_reason_pub = mock.Mock()
        n._request_mode = mock.Mock()
        n.set_mode = mock.Mock(return_value=SimpleNamespace(mode_sent=True))
        n.request_in_flight = False
        n._reset = mock.Mock()
        self.patch = mock.patch.object(manager_module.rospy, 'wait_for_service')
        self.patch.start(); self.addCleanup(self.patch.stop)

    def test_abort_land_requests_immediately_without_range_flow_stability(self):
        n = self.n
        n.wall.detach_range_valid = n.wall.detach_range_velocity_valid = False
        n._handover_ready = mock.Mock(side_effect=AssertionError('land waited for handover'))
        n._wall_fusion_handover_ready = mock.Mock(side_effect=AssertionError('land waited for fusion'))
        self.assertTrue(n._tick_wall_abort_land(self.now))
        n._request_mode.assert_called_once_with('AUTO.LAND')
        n._send_mode_request('AUTO.LAND', attempt=3)
        self.assertEqual(n.set_mode.call_args.args[0].custom_mode, 'AUTO.LAND')
        self.assertFalse(n.wall_land_confirmed)

    def test_worker_rechecks_attempt_and_pilot_mode_after_service_wait(self):
        n = self.n
        n._tick_wall_abort_land(self.now)
        n._send_mode_request('AUTO.LAND', attempt=2)
        n.set_mode.assert_not_called()
        with mock.patch.object(manager_module.rospy, 'wait_for_service', side_effect=lambda *a, **kw: setattr(n.flight, 'mode', 'POSCTL')):
            n._send_mode_request('AUTO.LAND', attempt=3)
        n.set_mode.assert_not_called()

    def test_worker_rechecks_maneuver_after_service_wait(self):
        n = self.n
        n._tick_wall_abort_land(self.now)
        n.wall.detach_range_maneuver_id = 'new-flight'
        n._send_mode_request('AUTO.LAND', attempt=3)
        n.set_mode.assert_not_called()

    def test_rejected_service_can_retry_and_confirmation_needs_new_heartbeat(self):
        n = self.n
        n._tick_wall_abort_land(self.now)
        n.set_mode.return_value.mode_sent = False
        with mock.patch.object(rospy, 'logwarn'):
            n._send_mode_request('AUTO.LAND', attempt=3)
        n.set_mode.return_value.mode_sent = True
        n._send_mode_request('AUTO.LAND', attempt=3)
        self.assertEqual(n.set_mode.call_count, 2)
        n.flight.mode = 'AUTO.LAND'
        n._tick_wall_abort_land(self.now)
        self.assertFalse(n.wall_land_confirmed)
        n.flight.header.stamp = n.flight_at = at(10.1)
        n._tick_wall_abort_land(at(10.1))
        self.assertTrue(n.wall_land_confirmed)
        self.assertEqual(n.phase, 'WALL_LANDING')

    def test_manual_mode_is_respected(self):
        n = self.n
        n._tick_wall_abort_land(self.now)
        n._request_mode.reset_mock()
        n.flight.mode = 'POSCTL'
        n._tick_wall_abort_land(self.now)
        n._request_mode.assert_not_called()
        n._reset.assert_called_once()

    def test_stale_abort_or_disarmed_flight_cannot_send_land(self):
        n = self.n
        n.wall.header.stamp = at(8.)
        self.assertFalse(n._tick_wall_abort_land(self.now))
        n.wall.header.stamp = self.now
        n._tick_wall_abort_land(self.now)
        n.flight.armed = False
        n._send_mode_request('AUTO.LAND', attempt=3)
        n.set_mode.assert_not_called()


class RecoveryMarginTests(unittest.TestCase):
    def test_attitude_only_thresholds_reject_nonfinite_and_excess(self):
        self.assertTrue(attitude_within_margins(math.radians(10.), math.radians(20.)))
        for tilt, rate in ((math.radians(10.01), 0.), (0., math.radians(20.01)),
                           (math.nan, 0.), (0., math.inf)):
            self.assertFalse(attitude_within_margins(tilt, rate))

    def test_nonzero_motion_within_margins_is_accepted(self):
        self.assertTrue(within_margins(math.radians(8.), math.radians(18.), .18, .23, .20))
        for index, excess in enumerate((math.radians(10.1), math.radians(20.1), .201, .251, .251)):
            values = [math.radians(8.), math.radians(18.), .18, .23, .20]
            values[index] = excess
            self.assertFalse(within_margins(*values))
            values[index] = math.nan
            self.assertFalse(within_margins(*values))

    def test_manager_uses_attitude_margins_without_height_or_speed_thresholds(self):
        fx = fixtures.RangeModeManagerTests()
        fx.setUp(); self.addCleanup(fx.doCleanups)
        n = fx.n
        n.wall.recover_attitude_locked = n.wall.recover_stable = True
        from px4_wall_ceiling_control.msg import OperatorCommand as Op
        n.operator_timeout_s = .5
        n.operator_at = at(10.)
        n.wall.detach_rotation_start_time = at(9.)
        n.operator = Op(mode=Op.MODE_WALL, enable_control=True, source='terminal',
                        pilot_handover_request_time=at(9.9))
        n.operator.header.stamp = at(10.)
        n.odom.header.stamp = at(10.)
        n.wall.detach_range_fast_velocity_valid = True
        n.wall.detach_range_fast_velocity_mps = .23
        n.wall.detach_range_height_m = 1.06
        n.wall.detach_range_velocity_mps = 99.
        q = quaternion(8.)
        n.odom.pose.pose.orientation = Quaternion(*q)
        n.odom.pose.pose.position.z = math.nan
        n.odom.twist.twist.linear.z = math.nan
        n.odom.twist.twist.linear.x = .20
        n.odom.twist.twist.angular.y = math.radians(18.)
        n._wall_horizontal_ready = mock.Mock(side_effect=AssertionError('legacy stability gate'))
        self.assertTrue(n._handover_ready(at(10.)))
        n.wall.detach_range_valid = n.wall.detach_range_fast_velocity_valid = False
        n.wall.detach_range_config_json = ''
        n.wall.detach_range_height_m = n.wall.detach_range_fast_velocity_mps = math.nan
        n.odom.twist.twist.linear.x = 2.
        self.assertTrue(n._handover_ready(at(10.)))
        n.odom.twist.twist.angular.y = math.radians(20.1)
        self.assertFalse(n._handover_ready(at(10.)))
        n.odom.twist.twist.angular.y = math.radians(18.)
        n.wall.recover_stable = False
        self.assertFalse(n._handover_ready(at(10.)))

    def test_manager_keeps_rc_estimator_fusion_and_output_safety_checks(self):
        fx = fixtures.RangeModeManagerTests()
        fx.setUp(); self.addCleanup(fx.doCleanups)
        n = fx.n
        n.wall.recover_attitude_locked = n.wall.recover_stable = True
        from px4_wall_ceiling_control.msg import OperatorCommand as Op
        n.operator_timeout_s = .5
        n.operator_at = at(10.)
        n.wall.detach_rotation_start_time = at(9.)
        n.operator = Op(mode=Op.MODE_WALL, enable_control=True, source='terminal',
                        pilot_handover_request_time=at(9.9))
        n.operator.header.stamp = at(10.)
        n.odom.header.stamp = at(10.)
        self.assertTrue(n._handover_ready(at(10.)))
        n.rc_at = at(9.)
        self.assertFalse(n._handover_ready(at(10.)))
        n.rc_at = at(10.)
        n.estimator.pos_horiz_rel_status_flag = False
        self.assertFalse(n._handover_ready(at(10.)))
        n.estimator.pos_horiz_rel_status_flag = True
        n._wall_fusion_handover_ready = lambda now: False
        self.assertFalse(n._handover_ready(at(10.)))
        n._wall_fusion_handover_ready = lambda now: True
        n.output_at = at(9.)
        self.assertFalse(n._handover_ready(at(10.)))
        n.output_at = at(10.)
        n.odom.header.stamp = at(9.)
        self.assertFalse(n._handover_ready(at(10.)))
