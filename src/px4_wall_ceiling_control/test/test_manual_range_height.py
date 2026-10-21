"""Fixed-target admission, bounded recovery and stale-handover regression tests."""
import unittest
from types import SimpleNamespace
from unittest import mock

from px4_wall_ceiling_control.msg import WallPerchStatus as W
from px4_wall_ceiling_control.srv import SetRangeHeightRequest
import test_range_height_integration as fixtures


class ManualRangeHeightTests(unittest.TestCase):
    def setUp(self):
        self.fx = fixtures.RangeDetachNodeTests()
        self.fx.setUp()
        self.addCleanup(self.fx.doCleanups)
        self.n = self.fx.n
        self.frame(10.25)
        self.frame(10.5)
        self.fx.prepare_stability()

    def frame(self, seconds, distance=1.24, angle=0.):
        self.fx.frame(seconds, distance=distance, angle=angle)
        self.n.flight_state.header.stamp = fixtures.at(seconds)

    def request(self, identifier='height-1', **kwargs):
        n = self.n
        values = dict(request_id=identifier, maneuver_id=n.maneuver_id,
                      episode=n.detach_range.episode, expected_target_revision=n.detach_range.target_revision,
                      requested_at=fixtures.at(self.fx.seconds))
        values.update(kwargs)
        return SetRangeHeightRequest(**values)

    def submit(self, **kwargs):
        return self.n._on_set_range_height(self.request(**kwargs))

    def timeout(self):
        self.frame(25.3)
        self.frame(25.5)
        self.n._prepare_range_feedback(fixtures.at(25.5))
        self.n._update_state_machine(fixtures.at(25.5))
        self.assertEqual(self.n.state, W.STATE_ABORT)
        self.n._front_fresh = self.n._sensor_health_fresh = lambda now: True
        self.n.front_range = SimpleNamespace(range=1.)
        self.n.sensor_health = SimpleNamespace(front_valid=True)

    def test_fixed_target_preserves_measurements_first_frame_and_actual_thrust(self):
        n, r = self.n, self.n.detach_range
        evidence = (r.lock_source, r.lock_time, r.lock_height, r.lock_raw, r.height, r.velocity, r.source)
        n.detach_vertical_controller.integral = .04
        result = self.submit()
        self.assertTrue(result.success, result.message)
        self.assertEqual(r.target, 1.5)
        self.assertEqual(r.target_revision, 1)
        self.assertEqual(r.target_source, 'manual_fixed_range_height')
        self.assertEqual(evidence, (r.lock_source, r.lock_time, r.lock_height, r.lock_raw, r.height, r.velocity, r.source))
        self.assertEqual(n.detach_vertical_controller.reference, 1.5)
        self.assertEqual(n.detach_vertical_controller.integral, 0.)
        self.assertAlmostEqual(n._detach_vertical_thrust(fixtures.at(10.5), .02), .70)
        self.assertEqual(n.detach_task_phase, 'RANGE_HEIGHT_CONTROL')

    def test_request_retry_is_idempotent_and_new_click_cannot_extend_budget(self):
        request = self.request()
        self.assertTrue(self.n._on_set_range_height(request).success)
        started = self.n.detach_range.action_started
        self.frame(10.75)
        self.assertTrue(self.n._on_set_range_height(request).success)
        self.assertFalse(self.submit(identifier='new-click').success)
        self.assertEqual(self.n.detach_range.action_started, started)
        self.assertEqual(self.n.detach_range.target_revision, 1)

    def test_stale_wrong_episode_or_revision_requests_are_rejected(self):
        for kwargs in (dict(requested_at=fixtures.at(9.)), dict(episode=9), dict(maneuver_id='old'),
                       dict(expected_target_revision=1), dict(request_id='')):
            result = self.submit(**kwargs)
            self.assertFalse(result.success, kwargs)
            self.assertEqual(self.n.detach_range.target_revision, 0)

    def test_missing_velocity_range_or_actual_owner_cannot_start(self):
        n = self.n
        n.detach_range.velocity_valid = False
        self.assertFalse(self.submit().success)
        self.frame(10.75)
        n.horizontal_output_owner = 0
        self.assertFalse(self.submit().success)
        self.assertEqual(n.detach_range.target_revision, 0)

    def test_invalid_collective_seed_is_rejected_without_partial_target_change(self):
        self.n.detach_vertical_controller.max_thrust = .65
        original = self.n.detach_range.target
        result = self.submit()
        self.assertFalse(result.success)
        self.assertEqual(self.n.detach_range.target, original)
        self.assertEqual(self.n.detach_range.target_revision, 0)
        self.assertFalse(self.n.detach_range_manual_request_id)

    def test_unlevel_pose_and_kill_or_canceled_authority_cannot_start(self):
        self.frame(10.75, angle=8.)
        self.assertFalse(self.submit().success)
        self.frame(11.)
        self.n._flight_ready = lambda now: False
        self.assertFalse(self.submit().success)
        self.n._flight_ready = lambda now: True
        self.n.cancel_requested = True
        self.assertFalse(self.submit().success)

    def test_posctl_and_disarmed_flight_cannot_start(self):
        self.n.flight_state.mode = 'POSCTL'
        self.assertFalse(self.submit().success)
        self.n.flight_state.mode = 'OFFBOARD'
        self.n.flight_state.armed = False
        self.assertFalse(self.submit().success)

    def test_unrelated_fused_height_and_vertical_velocity_cannot_block_fixed_target(self):
        self.fx.frame(10.75, height=float('nan'), vz=float('nan'))
        self.n.flight_state.header.stamp = fixtures.at(10.75)
        self.assertTrue(self.submit().success)
        self.assertEqual(self.n.detach_range.target, 1.5)

    def test_fixed_parameter_update_does_not_change_current_episode_target(self):
        self.fx.params['~detach_range_manual_target_m'] = 2.
        self.assertTrue(self.submit().success)
        self.assertEqual(self.n.detach_range.target, 1.5)

    def test_stage_three_override_revokes_stability_and_old_readiness(self):
        for seconds in (10.75, 11., 11.25, 11.5, 11.75):
            self.frame(seconds)
            self.n._update_state_machine(fixtures.at(seconds))
        self.assertEqual(self.n.state, W.STATE_EXIT)
        self.n.manager_handover_snapshot = ({'ready': True, 'target_revision': 0}, fixtures.at(self.fx.seconds))
        result = self.submit()
        self.assertTrue(result.success, result.message)
        self.assertEqual(self.n.state, W.STATE_RECOVER)
        self.assertEqual(self.n.detach_range_stable_samples, 0)
        self.assertIsNone(self.n.detach_range_handover_started)
        self.assertIsNone(self.n.manager_handover_snapshot)

    def test_already_sent_mode_request_blocks_override(self):
        self.n.detach_range_posctl_request_at = 10.49
        self.assertFalse(self.submit().success)

    def test_timeout_requires_new_admission_and_preserves_original_failure(self):
        self.timeout()
        lock = self.n.detach_range.lock_time
        result = self.submit()
        self.assertTrue(result.success, result.message)
        self.assertEqual(self.n.state, W.STATE_RECOVER)
        self.assertFalse(self.n.detach_range_faulted)
        self.assertTrue(self.n.detach_range_manual_recovery)
        self.assertIn('总时间', self.n.detach_range_previous_fault)
        self.assertEqual(self.n.detach_range.action_started, 25.5)
        self.assertEqual(self.n.detach_range.lock_time, lock)
        self.n._prepare_range_feedback(fixtures.at(25.5))
        self.assertFalse(self.n.detach_vertical_fault)

    def test_fault_readmission_rejects_unknown_clearance_and_non_timeout_failure(self):
        self.timeout()
        self.n.front_range.range = .01
        self.assertFalse(self.submit().success)
        self.n.front_range.range = 1.
        self.n.detach_range_failure_cause = 'cancel or wall mode released'
        self.assertFalse(self.submit().success)

    def test_first_non_timeout_failure_is_not_overwritten_by_later_timeout(self):
        self.n._enter_state(W.STATE_ABORT, fixtures.at(10.5), 'cancel or wall mode released')
        self.frame(26.)
        self.frame(26.2)
        self.n._prepare_range_feedback(fixtures.at(26.2))
        self.assertEqual(self.n.detach_range_failure_cause, 'cancel or wall mode released')
        self.assertFalse(self.submit().success)

    def test_manual_action_has_new_bounded_timeout_and_no_automatic_retry(self):
        self.assertTrue(self.submit().success)
        self.frame(25.6)
        self.n._prepare_range_feedback(fixtures.at(25.6))
        self.assertTrue(self.n.detach_range_faulted)
        self.assertFalse(self.submit(identifier='retry-after-timeout').success)

    def test_fault_manual_recovery_can_finish_but_never_becomes_original_success(self):
        self.timeout()
        self.assertTrue(self.submit().success)
        for seconds in (25.75, 26., 26.25, 26.5, 26.75, 27., 27.25, 27.5, 27.75, 28.):
            self.frame(seconds, distance=1.5)
            self.n._update_state_machine(fixtures.at(seconds))
        self.assertEqual(self.n.state, W.STATE_EXIT)
        self.assertTrue(self.n.fault_recovery_completed)
        self.assertFalse(self.n.detach_completed)
        self.frame(28.1, distance=1.5)
        self.n.flight_state.mode = 'POSCTL'
        self.n._update_state_machine(fixtures.at(28.1))
        self.assertEqual(self.n.state, W.STATE_IDLE)
        self.assertEqual(self.n.detach_task_phase, 'COMPLETED')
        self.assertFalse(self.n.detach_completed)
        self.assertFalse(self.n.recovery_completed)
        self.assertTrue(self.n.fault_recovery_completed)

    def test_status_reports_active_target_and_immutable_original_evidence(self):
        self.assertTrue(self.submit().success)
        status = W(); status.header.stamp = fixtures.at(10.5)
        self.n._fill_detach_vertical_status(status)
        self.assertEqual(status.detach_range_target_m, 1.5)
        self.assertAlmostEqual(status.detach_range_lock_height_m, 1.24)
        self.assertEqual(status.detach_range_target_revision, 1)
        self.assertEqual(status.vertical_reference_source, 'manual_fixed_range_height')
        self.assertFalse(status.detach_range_manual_available)


class ManualHandoverTests(unittest.TestCase):
    def test_mode_worker_rejects_old_target_version_after_wait(self):
        fx = fixtures.RangeModeManagerTests(); fx.setUp()
        self.addCleanup(fx.doCleanups)
        n = fx.n
        n.phase = n.RETURNING
        n.wall_pilot_mode = 'POSCTL'; n.takeover_attempt = 3
        n.flight = fixtures.State(connected=True, armed=True, mode='OFFBOARD')
        n.flight_at, n.state_timeout_s = fx.now, 1.
        n.set_mode = mock.Mock(); n.request_in_flight = True
        identity = n._wall_range_identity()
        def wait(*args, **kwargs):
            n.wall.detach_range_target_revision += 1
        with mock.patch.object(fixtures.rospy, 'wait_for_service', side_effect=wait):
            n._send_mode_request('POSCTL', attempt=3, range_identity=identity)
        n.set_mode.assert_not_called()


if __name__ == '__main__':
    unittest.main()
