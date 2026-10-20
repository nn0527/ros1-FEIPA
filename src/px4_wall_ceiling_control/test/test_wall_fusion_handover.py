"""Parameter ACK readiness cannot substitute for post-restoration feedback."""
import math
import os
import sys
import unittest
from unittest import mock

# catkin invokes nose with -P, so explicitly expose the shared test fixtures.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from mavros_msgs.msg import EstimatorStatus, OpticalFlowRad, State
from nav_msgs.msg import Odometry
from px4_wall_ceiling_control.msg import (
    OperatorCommand, SensorHealth, WallPerchStatus as W, WallSensorFusionStatus,
)
from test_flight_handover import Manager
import test_wall_horizontal_velocity as horizontal_fixtures
import test_auto_wall_approach as approach_fixtures


def status_at(t, ready=10., episode='wall-1', pending=False):
    message = WallSensorFusionStatus(enabled=True, snapshot_valid=True,
        parameters_restored=not pending, control_ready=not pending,
        restore_required=pending, episode_id=episode)
    message.header.stamp = rospy.Time.from_sec(t)
    message.ready_after = rospy.Time.from_sec(ready)
    return message


def feedback_at(t):
    now = rospy.Time.from_sec(t)
    odom = Odometry()
    odom.header.stamp = now
    odom.pose.pose.orientation.w = 1.
    estimator = EstimatorStatus(attitude_status_flag=True, velocity_vert_status_flag=True,
        velocity_horiz_status_flag=True, pos_horiz_rel_status_flag=True)
    estimator.header.stamp = now
    flow = OpticalFlowRad(quality=80, integration_time_us=20000, distance=1.)
    flow.header.stamp = now
    return odom, estimator, flow


class FusionHorizontalGateTests(unittest.TestCase):
    def setUp(self):
        self.fixture = horizontal_fixtures.HorizontalFlightTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.n = self.fixture.n
        self.n.wall_sensor_fusion_enabled = True
        self.n.wall_fusion_epoch = None

    def feed(self, t, ready=10., episode='wall-1', pending=False):
        self.fixture.time = t
        self.n._on_wall_fusion_status(status_at(t, ready, episode, pending))
        odom, estimator, flow = feedback_at(t)
        self.n._on_odom(odom)
        self.n._on_horizontal_estimator(estimator)
        self.n._on_horizontal_flow(flow)
        self.n._prepare_horizontal_velocity(rospy.Time.from_sec(t))

    def establish(self):
        for t in (10.1, 10.3, 10.5):
            self.feed(t)
        self.assertTrue(self.n.horizontal_velocity_ready)

    def test_pending_parameters_disable_loop_and_clear_integrals(self):
        self.establish()
        self.n.horizontal_controller.integral = (.1, .1)
        self.feed(10.6, pending=True)
        q = self.n.q_hover
        self.assertEqual(self.n._horizontal_attitude(rospy.Time.now(), q), q)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertEqual(self.n.horizontal_controller.integral, (0., 0.))

    def test_source_stale_status_rejected_even_with_new_receipt(self):
        self.establish()
        self.n._on_wall_fusion_status(status_at(9.))
        self.n._prepare_horizontal_velocity(rospy.Time.now())
        self.assertFalse(self.n.horizontal_velocity_ready)

    def test_new_epoch_requires_new_continuous_history(self):
        self.establish()
        self.feed(10.6, ready=10.55, episode='wall-2')
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.feed(10.8, ready=10.55, episode='wall-2')
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.feed(11., ready=10.55, episode='wall-2')
        self.assertTrue(self.n.horizontal_velocity_ready)

    def test_post_buffer_source_samples_and_complete_flow_interval_required(self):
        self.feed(10.01)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertIn('完整采集', self.n.horizontal_velocity_reason)
        self.feed(10.1)
        self.n.horizontal_estimator.header.stamp = rospy.Time.from_sec(9.9)
        self.n._prepare_horizontal_velocity(rospy.Time.now())
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertIn('新里程计', self.n.horizontal_velocity_reason)

    def test_hold_attitude_remains_available_while_restore_is_pending(self):
        self.n.state = W.STATE_WALL_HOLD
        self.feed(10.1, pending=True)
        q = (0., math.sin(math.pi / 4.), 0., math.cos(math.pi / 4.))
        self.assertEqual(self.n._horizontal_attitude(rospy.Time.now(), q), q)
        self.assertFalse(self.n.horizontal_velocity_ready)

    def test_disabled_configuration_still_blocks_unknown_corrupt_journal(self):
        self.n.wall_sensor_fusion_enabled = False
        status = WallSensorFusionStatus(enabled=True, phase='ERROR',
            snapshot_valid=False, restore_required=False, control_ready=False)
        status.header.stamp = rospy.Time.now()
        self.n._on_wall_fusion_status(status)
        self.assertFalse(self.n._wall_fusion_ready(rospy.Time.now()))
        self.assertFalse(self.n.horizontal_velocity_ready)

    def test_disabled_horizontal_recovery_still_checks_parameter_manager(self):
        self.n.horizontal_velocity_enabled = False
        self.feed(10.1, pending=True)
        self.assertFalse(self.n._horizontal_recovery_ready(rospy.Time.now()))
        self.feed(10.2)
        self.assertTrue(self.n._horizontal_recovery_ready(rospy.Time.now()))


class FusionDecisionEntryTests(unittest.TestCase):
    def node(self):
        node = approach_fixtures.AutoDecisionTests().node()
        node.state = W.STATE_IDLE
        node.wall_sensor_fusion_enabled = True
        node.horizontal_velocity_enabled = False
        node._reset_horizontal_gate = mock.Mock()
        return node

    def test_idle_cannot_begin_new_maneuver_with_unknown_or_pending_parameters(self):
        for status in (None, status_at(10.1, pending=True)):
            n = self.node()
            now = rospy.Time.from_sec(10.1)
            if status is not None:
                n.wall_fusion_status_snapshot = (status, now)
            n._update_state_machine(now)
            self.assertEqual(n.state, W.STATE_IDLE)
            n._enter_state.assert_not_called()
            self.assertIn('参数就绪', n.last_reason)

    def test_idle_rejects_stale_ready_then_accepts_fresh_ready_without_horizontal_loop(self):
        n = self.node()
        now = rospy.Time.from_sec(10.7)
        n.wall_fusion_status_snapshot = (status_at(10.1), now)
        n._update_state_machine(now)
        self.assertEqual(n.state, W.STATE_IDLE)
        n._enter_state.assert_not_called()
        n.wall_fusion_status_snapshot = (status_at(10.7), now)
        n._update_state_machine(now)
        self.assertEqual(n.state, W.STATE_SLOW_APPROACH)
        self.assertEqual([call[0][0] for call in n._enter_state.call_args_list],
                         [W.STATE_FRONT_WALL_DETECT, W.STATE_SLOW_APPROACH])


class FusionManagerGateTests(unittest.TestCase):
    def setUp(self):
        self.time = 10.1
        timer = mock.patch.object(rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(self.time))
        timer.start()
        self.addCleanup(timer.stop)
        n = self.n = Manager.__new__(Manager)
        n.enabled = True
        n.wall_sensor_fusion_enabled = True
        n.wall_horizontal_velocity_enabled = False
        n.wall_fusion_epoch = None
        n.selected_mode = OperatorCommand.MODE_WALL
        n.wall_pilot_mode = 'POSCTL'
        n.phase = Manager.RETURNING
        n.wall = W(state=W.STATE_EXIT, detach_completed=True, recovery_completed=True)
        n.wall_return_episode = True
        n.wall_approach_timeout_returning = False
        n.output_mode, n.owner = 2, 2
        n.flight = State(connected=True, armed=True, mode='OFFBOARD')
        n.sensor = SensorHealth(front_valid=True)
        n.rc_channels = [1500] * 8
        n.state_timeout_s = n.odom_timeout_s = n.estimator_timeout_s = .5
        n.status_timeout_s = n.sensor_timeout_s = n.output_timeout_s = n.rc_timeout_s = .5
        n.handover_max_tilt_deg = 10.
        n.handover_max_vertical_speed_mps = .2
        n.odom_twist_in_child_frame = False
        n.kill_guard_enabled = False
        n.set_mode = mock.Mock(return_value=type('Reply', (), {'mode_sent': True})())
        self.feed(10.1)

    def feed(self, t, ready=10., pending=False, epoch='wall-1'):
        self.time = t
        now = rospy.Time.from_sec(t)
        self.n._on_wall_fusion_status(status_at(t, ready, epoch, pending))
        odom, estimator, flow = feedback_at(t)
        self.n._on_odom(odom)
        self.n._on_estimator(estimator)
        self.n._on_wall_fusion_flow(flow)
        for key in ('flight_at', 'rc_at', 'wall_at', 'owner_at', 'output_at', 'sensor_at'):
            setattr(self.n, key, now)
        return now

    def establish(self):
        for t in (10.1, 10.3, 10.5):
            self.n._wall_fusion_ready(self.feed(t))
        self.assertTrue(self.n._wall_fusion_ready(rospy.Time.now()))

    def test_preflight_denies_pending_even_without_horizontal_loop(self):
        self.feed(10.1, pending=True)
        reason = self.n._preflight_failure_reason(rospy.Time.now(), OperatorCommand.MODE_WALL)
        self.assertIn('融合参数', reason)

    def test_all_return_paths_wait_for_own_restoration(self):
        self.establish()
        self.feed(10.6, pending=True)
        for name in ('_handover_ready', '_entry_cancel_handover_ready',
                     '_offboard_exit_request_ready', '_wall_timeout_handover_ready'):
            self.assertFalse(getattr(self.n, name)(rospy.Time.now()), name)

    def test_ready_after_exit_needs_post_buffer_new_data_and_confirmation(self):
        self.feed(10.1, pending=True)
        self.assertFalse(self.n._handover_ready(rospy.Time.now()))
        self.feed(10.3, ready=10.2)
        self.assertFalse(self.n._handover_ready(rospy.Time.now()))
        self.feed(10.5, ready=10.2)
        self.assertFalse(self.n._handover_ready(rospy.Time.now()))
        self.feed(10.7, ready=10.2)
        self.assertTrue(self.n._handover_ready(rospy.Time.now()))

    def test_stale_receipt_or_source_denies_and_clears_prior_confirmation(self):
        self.establish()
        self.n._on_wall_fusion_status(status_at(9.))
        self.assertFalse(self.n._handover_ready(rospy.Time.now()))
        self.feed(10.6)
        self.assertFalse(self.n._handover_ready(rospy.Time.now()))
        self.time = 11.2
        self.assertFalse(self.n._handover_ready(rospy.Time.now()))

    def test_old_estimator_and_odom_sources_do_not_count_as_new_feedback(self):
        for name in ('estimator', 'odom'):
            self.establish()
            getattr(self.n, name).header.stamp = rospy.Time.from_sec(9.9)
            self.assertFalse(self.n._handover_ready(rospy.Time.now()), name)
            self.assertEqual(self.n.wall_fusion_samples, 0)

    def test_duplicate_flow_does_not_complete_new_sample_confirmation(self):
        self.n._wall_fusion_ready(rospy.Time.now())
        old = self.n.wall_fusion_flow_snapshot
        self.feed(10.5)
        self.n.wall_fusion_flow_snapshot = old
        self.assertFalse(self.n._handover_ready(rospy.Time.now()))

    def test_worker_rechecks_when_pending_arrives_after_outer_check(self):
        self.establish()
        self.assertTrue(self.n._handover_ready(rospy.Time.now()))
        with mock.patch.object(rospy, 'wait_for_service',
                side_effect=lambda *args, **kwargs: self.feed(10.6, pending=True)):
            self.n._send_mode_request('POSCTL')
        self.n.set_mode.assert_not_called()

    def test_worker_accepts_ready_without_treating_ack_as_mode_confirmation(self):
        self.establish()
        with mock.patch.object(rospy, 'wait_for_service'):
            self.n._send_mode_request('POSCTL')
        self.n.set_mode.assert_called_once()
        self.assertEqual(self.n.flight.mode, 'OFFBOARD')

    def test_known_restore_debt_survives_disabled_feature_and_cleared_selection(self):
        self.feed(10.2, pending=True)
        self.n.wall_sensor_fusion_enabled = False
        self.n.selected_mode = None
        self.assertFalse(self.n._wall_fusion_handover_ready(rospy.Time.now()))
        self.n.selected_mode = OperatorCommand.MODE_CEILING
        self.assertFalse(self.n._wall_fusion_handover_ready(rospy.Time.now()))
        self.n._on_wall_fusion_status(status_at(10.2))
        self.assertTrue(self.n._wall_fusion_handover_ready(rospy.Time.now()))

    def test_unknown_corrupt_journal_blocks_even_after_selection_is_cleared(self):
        self.n.wall_sensor_fusion_enabled = False
        status = WallSensorFusionStatus(enabled=True, phase='ERROR',
            snapshot_valid=False, restore_required=False, control_ready=False)
        status.header.stamp = rospy.Time.now()
        self.n._on_wall_fusion_status(status)
        for selected in (OperatorCommand.MODE_WALL, None, OperatorCommand.MODE_CEILING):
            self.n.selected_mode = selected
            self.assertFalse(self.n._wall_fusion_handover_ready(rospy.Time.now()))

    def test_ready_manager_does_not_add_wall_sample_gate_to_ceiling(self):
        self.n.selected_mode = OperatorCommand.MODE_CEILING
        self.n.wall_fusion_flow_snapshot = None
        self.assertTrue(self.n._wall_fusion_handover_ready(rospy.Time.now()))

    def test_feature_disabled_retains_legacy_behavior(self):
        self.n.wall_sensor_fusion_enabled = False
        self.n.wall_fusion_status_snapshot = None
        self.assertTrue(self.n._handover_ready(rospy.Time.now()))


if __name__ == '__main__':
    unittest.main()
