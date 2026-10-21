"""Mocked node boundaries for the three range departure task phases.

Uses ROS message classes but never starts flight nodes, transports or services.
"""
import json
import math
import os
import sys
import unittest
from unittest import mock
from dataclasses import asdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rospy
from mavros_msgs.msg import OpticalFlowRad, EstimatorStatus, State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from px4_wall_ceiling_control.msg import WallPerchStatus as W, OperatorCommand
from px4_wall_ceiling_control.domain.range_height_feedback import RangeHeightConfig
import test_detach_state_machine as state_fixtures
at = state_fixtures.at
from test_flight_handover import Manager
import test_candidate_latency as mux_fixtures
import test_wall_console as console_fixtures
tilt = mux_fixtures.tilt


class RangeDetachNodeTests(unittest.TestCase):
    def setUp(self):
        self.seconds = 10.
        self.params = {'~detach_range_height_enabled': True, '~detach_range_min_distance_m': .02,
                       '~detach_range_max_distance_m': 6., '~detach_range_geometry_confirmed': True}
        patch = mock.patch.object(rospy.Time, 'now', side_effect=lambda: at(self.seconds))
        patch.start(); self.addCleanup(patch.stop)
        patch = mock.patch.object(rospy, 'get_param', side_effect=lambda key, default=None: self.params.get(key, default))
        patch.start(); self.addCleanup(patch.stop)
        patch = mock.patch.object(rospy, 'loginfo')
        patch.start(); self.addCleanup(patch.stop)
        self.fx = state_fixtures.DetachStateMachineTests()
        self.n = self.new_node()

    def new_node(self):
        n = self.fx.node(**self.params)
        n._on_detach(Bool(data=True))
        self.fx.enter(n, W.STATE_DETACH_ROTATE, 10.)
        self.assertEqual(n.state, W.STATE_DETACH_ROTATE)
        return n

    def frame(self, seconds, distance=1.24, angle=0., height=2.4, vz=0., node=None):
        n = node or self.n
        self.seconds = seconds
        self.fx.feedback(n, seconds, tilt=angle, height=height, vz=vz)
        self.fx.output(n, seconds, tilt=angle, thrust=.70)
        m = OpticalFlowRad(distance=distance, quality=0)
        m.header.stamp = at(seconds)
        n._on_detach_range(m)
        return m

    def prepare_stability(self, node=None):
        n = node or self.n
        self.fx.enter(n, W.STATE_RECOVER, self.seconds)
        n._detach_progress = lambda: 1.
        n._wall_fusion_samples_ready = lambda now: True
        n._range_px4_ready = lambda now: True

    def stable_exit(self):
        self.frame(10.25)
        self.prepare_stability()
        for t in (10.25, 10.5, 10.75, 11., 11.25, 11.5):
            if t > 10.25:
                self.frame(t)
            self.n._update_state_machine(at(t))
        self.assertEqual(self.n.state, W.STATE_EXIT)

    def test_first_frame_locks_during_rotate_without_fusion_or_flow_readiness(self):
        n = self.n
        n.horizontal_velocity_enabled = False
        n.wall_sensor_fusion_enabled = True
        n.horizontal_estimator.velocity_vert_status_flag = False
        self.frame(10.25, angle=9.)
        self.assertEqual(n.state, W.STATE_DETACH_ROTATE)
        self.assertEqual(n.detach_task_phase, 'RANGE_HEIGHT_CONTROL')
        self.assertAlmostEqual(n.detach_vertical_controller.reference, 1.24*math.cos(math.radians(9.)))
        self.assertEqual(n.detach_vertical_controller.integral, 0.)
        self.assertFalse(n.detach_range.velocity_valid)
        self.assertTrue(math.isnan(n.detach_range.velocity))

    def test_old_z_and_world_vz_cannot_change_range_control_or_height_gate(self):
        outputs = []
        for height, vz in ((2.4, .4), (-999., -99.), (math.nan, math.nan)):
            n = self.new_node()
            self.frame(10.25, height=height, vz=vz, node=n)
            self.frame(10.5, distance=1.23, height=height, vz=vz, node=n)
            self.seconds = 10.52
            self.fx.output(n, self.seconds, tilt=0., thrust=.70)
            value = n._detach_vertical_thrust(at(self.seconds), .02)
            self.assertTrue(math.isfinite(value))
            self.assertTrue(n._detach_height_settled())
            outputs.append(value)
        self.assertEqual(outputs[0], outputs[1])
        self.assertEqual(outputs[1], outputs[2])

    def test_short_loss_holds_actual_output_long_loss_faults_and_never_reanchors(self):
        n = self.n
        self.frame(10.25)
        reference = n.detach_range.target
        self.seconds = 10.87
        self.fx.feedback(n, self.seconds, tilt=0., height=999., vz=99.)
        self.fx.output(n, self.seconds, tilt=0., thrust=.68)
        self.assertAlmostEqual(n._detach_vertical_thrust(at(self.seconds), .02), .68)
        self.assertFalse(n.detach_range.snapshot(self.seconds).velocity_valid)
        self.seconds = 11.17
        n._prepare_detach_vertical_feedback(at(self.seconds))
        self.assertEqual(n.detach_task_phase, 'FAULT')
        self.assertTrue(n.detach_vertical_fault)
        self.fx.enter(n, W.STATE_ABORT, self.seconds)
        self.frame(11.2, distance=1.3)
        self.assertEqual(n.detach_range.target, reference)
        self.assertEqual(n.detach_vertical_controller.reference, reference)
        self.assertEqual(n.detach_task_phase, 'FAULT')
        self.assertFalse(n.detach_range.velocity_valid)

    def test_phase_three_can_be_revoked_without_target_or_budget_reset(self):
        self.stable_exit()
        n = self.n
        reference, locked_at = n.detach_range.target, n.detach_range.lock_time
        wait_started = n.detach_range_handover_started
        self.assertEqual(n.detach_task_phase, 'HANDOVER_READY')
        self.assertFalse(n.detach_completed)
        self.frame(11.75, distance=1.6)
        n._update_state_machine(at(self.seconds))
        self.assertEqual(n.state, W.STATE_RECOVER)
        self.assertEqual(n.detach_task_phase, 'RANGE_HEIGHT_CONTROL')
        self.assertEqual(n.detach_range.target, reference)
        self.assertEqual(n.detach_range.lock_time, locked_at)
        self.assertEqual(n.detach_range_handover_started, wait_started)
        self.assertEqual(n.detach_range_stable_samples, 0)

    def test_only_fresh_posctl_confirmation_finishes_and_releases(self):
        self.stable_exit()
        n = self.n
        self.seconds = 11.55
        n.detach_range_posctl_request_at = 11.54  # Service submitted, no mode confirmation.
        n._update_state_machine(at(self.seconds))
        self.assertEqual(n.state, W.STATE_EXIT)
        self.assertFalse(n.detach_completed)
        self.assertTrue(math.isfinite(n._detach_vertical_thrust(at(self.seconds), .02)))
        n.flight_state.mode = 'POSCTL'
        n.flight_state.header.stamp = at(11.56)
        n.flight_state_received_at = at(11.56)
        self.seconds = 11.56
        n._update_state_machine(at(self.seconds))
        self.assertEqual(n.state, W.STATE_IDLE)
        self.assertEqual(n.detach_task_phase, 'COMPLETED')
        self.assertTrue(n.detach_completed)
        self.assertFalse(n.detach_vertical_active)
        self.assertEqual(n.detach_range_posctl_confirmed_at, 11.56)

    def test_manual_takeover_before_ready_releases_without_normal_completion(self):
        self.frame(10.25)
        self.n.flight_state.mode = 'POSCTL'
        self.n.flight_state.header.stamp = self.n.flight_state_received_at = at(10.3)
        self.seconds = 10.3
        self.n._update_state_machine(at(self.seconds))
        self.assertEqual(self.n.state, W.STATE_IDLE)
        self.assertEqual(self.n.detach_task_phase, 'FAULT')
        self.assertFalse(self.n.detach_completed)

    def test_stale_or_replayed_posctl_heartbeat_cannot_complete(self):
        self.stable_exit()
        message = State(connected=True, armed=True, mode='POSCTL')
        message.header.stamp = at(9.)
        self.seconds = 11.55
        self.n._on_state(message)
        self.n._update_state_machine(at(self.seconds))
        self.assertEqual(self.n.state, W.STATE_EXIT)
        self.assertFalse(self.n.detach_completed)

    def test_fault_exit_posctl_releases_without_becoming_completed(self):
        self.frame(10.25)
        self.fx.enter(self.n, W.STATE_ABORT, 10.25)
        self.fx.enter(self.n, W.STATE_EXIT, 10.26)
        self.seconds = 10.3
        self.n.flight_state.mode = 'POSCTL'
        self.n.flight_state.header.stamp = self.n.flight_state_received_at = at(10.3)
        self.n._update_state_machine(at(self.seconds))
        self.assertEqual(self.n.state, W.STATE_IDLE)
        self.assertEqual(self.n.detach_task_phase, 'FAULT')
        self.assertFalse(self.n.detach_completed)

    def test_disarm_marks_fault_and_stops_departure_support(self):
        self.frame(10.25)
        self.n._on_state(State(connected=True, armed=False, mode='POSCTL'))
        self.assertEqual(self.n.detach_task_phase, 'FAULT')
        self.assertFalse(self.n.detach_vertical_active)
        self.assertFalse(self.n.detach_completed)

    def test_fault_recovery_never_becomes_normal_stage_three(self):
        self.frame(10.25)
        self.fx.enter(self.n, W.STATE_ABORT, self.seconds)
        self.assertEqual(self.n.detach_task_phase, 'FAULT')
        self.frame(10.5)
        self.assertFalse(self.n._range_stability_ready(at(self.seconds)))
        self.assertFalse(self.n.detach_completed)

    def test_duplicate_timer_ticks_do_not_fill_stable_sample_count(self):
        self.frame(10.25)
        self.frame(10.5)
        self.prepare_stability()
        for t in (10.5, 10.52, 10.54, 10.56):
            self.seconds = t
            self.n._range_stability_ready(at(t))
        self.assertEqual(self.n.detach_range_stable_samples, 1)

    def test_total_timeout_survives_phase_changes(self):
        self.stable_exit()
        self.frame(25.3)
        self.n._prepare_detach_vertical_feedback(at(self.seconds))
        self.assertEqual(self.n.detach_task_phase, 'FAULT')
        self.assertIn('总时间', self.n.detach_vertical_fault)

    def test_parameter_update_is_deferred_to_next_episode(self):
        self.params['~detach_range_entry_tilt_deg'] = 8.
        self.frame(10.25, angle=9.)
        self.assertTrue(self.n.detach_range.locked)
        self.assertEqual(self.n.detach_range.config.entry_tilt_deg, 10.)

    def test_status_and_candidate_share_episode_phase_target_and_invalid_velocity(self):
        self.frame(10.25, angle=8.)
        s = W(); s.header.stamp = at(self.seconds)
        self.n._fill_detach_vertical_status(s)
        c = self.n._build_control_candidate(s, self.n.maneuver_id, 1)
        self.assertEqual(s.detach_task_phase_name, 'RANGE_HEIGHT_CONTROL')
        self.assertEqual(c.detach_task_phase, s.detach_task_phase)
        self.assertEqual(c.detach_range_episode, s.detach_range_episode)
        self.assertTrue(c.range_height_control)
        self.assertTrue(c.range_feedback_valid)
        self.assertTrue(math.isnan(s.detach_range_velocity_mps))
        self.assertTrue(math.isnan(s.vertical_target_enu_mps))


class RangeMuxTests(unittest.TestCase):
    def setUp(self):
        self.fx = mux_fixtures.CandidateLatencyTest()
        self.fx.setUp()
        self.addCleanup(self.fx.doCleanups)
        self.n = self.fx.node
        self.fx.accept(self.fx.candidate())

    def candidate(self):
        c = self.fx.candidate(phase=W.STATE_DETACH_ROTATE, sequence=2, angle=0., acquire=False)
        c.range_height_control = c.range_feedback_valid = True
        c.detach_task_phase, c.detach_range_episode = 2, 1
        c.recovery_support_valid, c.recovery_thrust_limit = True, .85
        return c

    def test_range_entry_does_not_project_actual_ten_degree_output_to_five(self):
        self.fx.seed_publication(angle=10., thrust=.74)
        self.fx.advance()
        c = self.candidate(); self.fx.accept(c)
        self.assertEqual(self.n._publish_candidate(self.fx.now, c), self.n.OUTPUT_ATTITUDE_THRUST)
        target = self.n.attitude_publisher.publish.call_args.args[0]
        self.assertGreater(tilt(target), 5.)
        self.assertAlmostEqual(tilt(target), 10.-56.25*.02)
        self.assertLessEqual(abs(target.thrust-.74), .0100001)

    def test_only_range_candidate_can_ignore_unrelated_odom_z(self):
        self.fx.advance()
        c = self.candidate(); self.fx.accept(c)
        self.n.owner = self.n.OWNER_WALL
        self.n.odom_twist_in_child_frame = False
        self.n.odom.pose.pose.position.z = math.nan
        self.n.odom.twist.twist.linear.z = math.nan
        self.assertTrue(self.n._odom_fresh(self.fx.now))
        c.range_height_control = False
        self.assertFalse(self.n._odom_fresh(self.fx.now))
        c.range_height_control = True
        self.n.odom.twist.twist.linear.x = math.nan
        self.assertFalse(self.n._odom_fresh(self.fx.now))


class RangeModeManagerTests(unittest.TestCase):
    def setUp(self):
        self.now = at(10.)
        patch = mock.patch.object(rospy.Time, 'now', return_value=self.now)
        patch.start(); self.addCleanup(patch.stop)
        n = self.n = Manager.__new__(Manager)
        n.selected_mode = OperatorCommand.MODE_WALL
        n.wall = W(state=W.STATE_EXIT, detach_vertical_active=True,
            detach_range_height_enabled=True, detach_range_target_locked=True,
            detach_task_phase_name='HANDOVER_READY', detach_range_handover_ready=True,
            detach_range_valid=True, detach_range_velocity_valid=True,
            detach_range_velocity_mps=.01, detach_range_height_m=1.24, detach_range_target_m=1.24,
            detach_range_episode=1, detach_range_maneuver_id='test')
        n.wall.header.stamp = n.wall.detach_range_source_time = n.wall.detach_range_received_time = self.now
        n.wall.detach_range_config_json = json.dumps(asdict(RangeHeightConfig()))
        n.wall_at = n.output_at = n.estimator_at = n.odom_at = n.rc_at = self.now
        n.status_timeout_s = n.output_timeout_s = n.estimator_timeout_s = n.odom_timeout_s = .5
        n.output_mode = 2
        n.estimator = EstimatorStatus(attitude_status_flag=True, velocity_vert_status_flag=True,
            pos_vert_abs_status_flag=True, velocity_horiz_status_flag=True, pos_horiz_rel_status_flag=True)
        n.odom = Odometry(); n.odom.pose.pose.orientation.w = 1.
        n.odom_twist_in_child_frame = False
        n.handover_max_tilt_deg, n.handover_max_vertical_speed_mps = 10., .20
        n.rc_channels = (1500, 1500, 1500, 1500)
        n.rc_roll_channel_index, n.rc_pitch_channel_index, n.rc_throttle_channel_index = 0, 1, 2
        n.rc_timeout_s, n.require_rc_center = .2, False
        n.wall_return_episode = True
        n._wall_fusion_handover_ready = lambda now: True
        n._wall_horizontal_ready = lambda now, stable=False: True

    def test_handover_uses_range_velocity_and_separate_px4_readiness(self):
        n = self.n
        n.odom.pose.pose.position.z = math.nan
        n.odom.twist.twist.linear.z = -99.
        self.assertTrue(n._handover_ready(self.now))
        n.wall.detach_range_velocity_mps = -.3
        self.assertFalse(n._handover_ready(self.now))
        n.wall.detach_range_velocity_mps = 0.
        n.estimator.pos_vert_abs_status_flag = False
        self.assertFalse(n._handover_ready(self.now))

    def test_revoked_or_source_stale_range_status_cannot_authorize_request(self):
        n = self.n
        self.assertTrue(n._handover_ready(self.now))
        n.wall.detach_range_handover_ready = False
        self.assertFalse(n._handover_ready(self.now))
        n.wall.detach_range_handover_ready = True
        n.wall.header.stamp = at(9.)
        self.assertFalse(n._handover_ready(self.now))

    def test_worker_rechecks_episode_after_service_wait(self):
        n = self.n
        n.phase = n.RETURNING
        n.wall_pilot_mode = 'POSCTL'
        n.takeover_attempt = 3
        n.flight = State(connected=True, armed=True, mode='OFFBOARD')
        n.flight_at, n.state_timeout_s = self.now, 1.
        n.set_mode = mock.Mock()
        n.request_in_flight = True
        old_identity = n._wall_range_identity()
        def wait(*args, **kwargs):
            n.wall.detach_range_episode += 1
        with mock.patch.object(rospy, 'wait_for_service', side_effect=wait):
            n._send_mode_request('POSCTL', attempt=3, range_identity=old_identity)
        n.set_mode.assert_not_called()

    def test_pre_request_posctl_heartbeat_is_not_rpc_confirmation(self):
        n = self.n
        n.flight = State(connected=True, armed=True, mode='OFFBOARD')
        n.flight_at = self.now
        n.phase, n.state_timeout_s = n.RETURNING, 1.
        n.range_posctl_request = (n._wall_range_identity(), 9.95)
        old = State(connected=True, armed=True, mode='POSCTL')
        old.header.stamp = at(9.9)
        n._on_flight(old)
        self.assertEqual(n.flight.mode, 'OFFBOARD')


class RangeTelemetryTests(unittest.TestCase):
    def test_console_serializes_source_times_and_unavailable_velocity(self):
        runtime = object.__new__(console_fixtures.server.RemoteRuntime)
        runtime.state = mock.Mock()
        message = W(detach_range_height_enabled=True, detach_range_target_locked=True,
            detach_task_phase_name='RANGE_HEIGHT_CONTROL', detach_range_velocity_mps=math.nan,
            detach_range_velocity_valid=False, detach_range_lock_raw_m=1.24,
            detach_range_lock_height_m=1.2386)
        message.detach_range_lock_source_time = at(10.123)
        message.detach_range_lock_received_time = at(10.14)
        runtime.callback(message, 'wall')
        name, value = runtime.state.update.call_args.args
        self.assertEqual(name, 'wall')
        self.assertEqual(value['detach_range_lock_source_time'], 10.123)
        self.assertIsNone(value['detach_range_velocity_mps'])
        self.assertFalse(value['detach_range_velocity_valid'])
        self.assertEqual(value['detach_range_lock_raw_m'], 1.24)
        json.dumps(value, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
