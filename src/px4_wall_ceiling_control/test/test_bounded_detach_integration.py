"""ROS adapter and state-machine checks, including measured log replay.

Stable suffixes below are synthetic software stimuli, not recorded flight.
"""
import json
import math
import pathlib
import threading
import unittest
from unittest import mock

import rospy
from px4_wall_ceiling_control.msg import WallPerchStatus as W
import test_detach_state_machine as fsmfixtures
from test_detach_state_machine import at


class BoundedDetachIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.f = fsmfixtures.DetachStateMachineTests()

    def begun(self, tilt=20.):
        n = self.f.node()
        n.odom_source_stamp = None
        self.f.feedback(n, 10., tilt=tilt)
        self.f.output(n, 10., tilt=tilt, thrust=.7)
        self.f.enter(n, W.STATE_DETACH_ROTATE, 10.)
        return n

    def tick(self, n, t, height, tilt=5., vz=0., rate=0., source=None):
        self.f.feedback(n, t, tilt=tilt, height=height, vz=vz, rate=rate, source=source)
        previous = n.horizontal_output.thrust
        self.f.output(n, t, tilt=0., thrust=previous)
        with mock.patch.object(rospy, 'loginfo'):
            n._update_state_machine(at(t))
            candidate = n._candidate(at(t))
        # Emulate output echo of the candidate, not vehicle physics.
        _, q, thrust, _ = candidate
        if math.isfinite(thrust):
            self.assertLessEqual(abs(thrust-previous), .0250001)
            self.assertGreaterEqual(thrust, .1)
            self.assertLessEqual(thrust, .85)
            n.horizontal_output.thrust = thrust
        return candidate

    def test_anomaly_detected_before_fsm_without_nan_candidate(self):
        n = self.begun(); n.detach_vertical_controller.integral = .03
        self.tick(n, 10.02, 2.8, tilt=20.)
        self.assertEqual(n.state, W.STATE_DETACH_ROTATE)
        self.assertEqual(n.detach_height_continuation.state, 'SUPPORT')
        self.assertTrue(math.isfinite(n.detach_vertical_value))
        self.assertTrue(n.recovery_support_valid)
        self.assertEqual(n.detach_vertical_controller.integral, 0.)
        self.assertEqual(n.vertical_correction, 0.)
        status = W(); n._fill_detach_vertical_status(status)
        self.assertFalse(status.vertical_control_active)
        self.assertTrue(status.recovery_support_valid)
        self.assertIn('高度闭环暂停', status.detach_vertical_reason)

    def test_three_references_reanchor_without_resetting_progress_or_pressure(self):
        n = self.begun()
        self.tick(n, 10.02, 2.8)
        pressure, base = n.detach_vertical_controller.wall_pressure, n.detach_vertical_controller.base
        progress = n.detach_elapsed_s
        for i in range(1, 5):
            self.tick(n, 10.02+i*.1, 2.8)
        self.assertEqual(n.detach_height_continuation.state, 'BLENDING')
        self.assertEqual((n.detach_altitude_m, n.recovery_altitude_m, n.detach_vertical_controller.reference), (2.8,)*3)
        self.assertEqual(n.detach_vertical_controller.wall_pressure, pressure)
        self.assertEqual(n.detach_vertical_controller.base, base)
        self.assertGreaterEqual(n.detach_elapsed_s, progress)
        self.assertEqual(n.vertical_reference_source, 'bounded_continuation_reanchor')
        self.assertFalse(n._detach_rotation_ready(at(10.42)))

    def test_output_permission_loss_and_feedback_expiry_abort(self):
        for lost in ('owner', 'estimator', 'offboard', 'odom'):
            n = self.begun(); self.tick(n, 10.02, 2.8, tilt=20.)
            self.f.feedback(n, 10.04, tilt=20., height=2.8)
            self.f.output(n, 10.04, thrust=n.horizontal_output.thrust)
            if lost == 'owner': n.horizontal_output_owner = 0
            if lost == 'estimator': n.horizontal_estimator_at = at(8.)
            if lost == 'offboard': n.flight_state.mode = 'POSCTL'
            if lost == 'odom': n.odom_received_at = at(8.)
            self.f.tick(n, 10.04)
            self.assertEqual(n.state, W.STATE_ABORT, lost)

    def test_tilt_exit_and_budget_failure_abort_in_same_tick(self):
        for tilt, time in ((31., 10.04), (20., 11.23)):
            n = self.begun(); self.tick(n, 10.02, 2.8, tilt=20.)
            self.tick(n, time, 2.8, tilt=tilt)
            self.assertEqual(n.state, W.STATE_ABORT)

    def test_missing_feedback_abort_requires_two_new_frames_then_supports_handover(self):
        n = self.begun(); self.tick(n, 10.02, 2.8)
        self.f.feedback(n, 10.04, tilt=5., height=2.8)
        self.f.output(n, 10.04, thrust=n.horizontal_output.thrust)
        n.horizontal_estimator_at = at(8.)
        self.f.tick(n, 10.04)
        self.assertEqual(n.state, W.STATE_ABORT)
        self.assertIsNone(n.detach_vertical_fault_feedback)
        self.assertTrue(math.isnan(self.tick(n, 10.06, 2.8)[2]))
        self.assertTrue(math.isfinite(self.tick(n, 10.08, 2.8)[2]))
        self.assertFalse(n.detach_vertical_fault)
        self.assertFalse(n.detach_completed or n.recovery_completed)

    def test_cancel_during_continuation_keeps_original_abort_path(self):
        n = self.begun(); self.tick(n, 10.02, 2.8)
        n.cancel_requested = True
        self.tick(n, 10.04, 2.8)
        self.assertEqual(n.state, W.STATE_ABORT)
        self.assertIn('cancel', n.last_reason)

    def test_new_maneuver_resets_budget_but_stage_change_does_not(self):
        n = self.begun(); self.tick(n, 10.02, 2.8, tilt=20.)
        self.tick(n, 10.12, 2.8, tilt=20.)
        used = n.detach_height_continuation.total_used
        self.f.enter(n, W.STATE_RECOVER, 10.12)
        self.assertEqual(n.detach_height_continuation.total_used, used)
        self.f.enter(n, W.STATE_IDLE, 10.13)
        self.assertEqual(n.detach_height_continuation.total_used, 0.)

    def test_callback_cannot_replace_odom_mid_control_cycle(self):
        n = self.begun(); old = n.odom
        seen, entered, finished = [], threading.Event(), threading.Event()
        from nav_msgs.msg import Odometry
        new = Odometry(); new.header.stamp = at(10.02); new.pose.pose.orientation.w = 1.
        new.pose.pose.position.z = 99.
        def cycle(_):
            seen.append(n._snapshot_departure_feedback())
            entered.set()
            self.assertFalse(finished.wait(.03))
            self.assertIs(n.odom, old)
            seen.append(n._snapshot_departure_feedback())
        def callback():
            entered.wait(1.)
            with mock.patch.object(rospy.Time, 'now', return_value=at(10.02)):
                n._on_odom(new)
            finished.set()
        n._control_cycle = cycle
        thread = threading.Thread(target=callback); thread.start()
        n._on_timer(None); thread.join(1.)
        self.assertTrue(finished.is_set())
        self.assertEqual(seen[0], seen[1])
        self.assertIs(n.odom, new)

    def test_height_anomaly_revokes_provisional_handover_without_reset(self):
        n = self.begun()
        self.f.enter(n, W.STATE_RECOVER, 10.)
        self.f.enter(n, W.STATE_EXIT, 10.01)
        n.detach_completed = n.recovery_completed = True
        self.tick(n, 10.02, 2.8)
        self.assertEqual(n.state, W.STATE_RECOVER)
        self.assertFalse(n.detach_completed or n.recovery_completed)
        self.assertAlmostEqual(n.detach_height_continuation.episode_at, 10.02)
        self.assertEqual(n.recovery_started_at, at(10.))
        self.tick(n, 10.12, 2.8)
        self.assertAlmostEqual(n.detach_height_continuation.total_used, .1)

    def test_height_failure_at_normal_exit_aborts_but_fault_exit_still_hands_over(self):
        n = self.begun()
        self.f.enter(n, W.STATE_RECOVER, 10.)
        self.f.enter(n, W.STATE_EXIT, 10.01)
        n.detach_completed = n.recovery_completed = True
        self.tick(n, 10.02, 2.8, tilt=31.)
        self.assertEqual(n.state, W.STATE_ABORT)
        self.assertFalse(n.detach_completed or n.recovery_completed)
        # Let the existing ABORT controller adopt two consistent new frames.
        self.tick(n, 10.03, 2.8)
        self.assertFalse(n.detach_vertical_fault)
        self.f.enter(n, W.STATE_EXIT, 10.03)
        for t in (10.04, 10.06, 10.08):
            _, _, thrust, _ = self.tick(n, t, 2.8)
            self.assertEqual(n.state, W.STATE_EXIT)
            self.assertTrue(n.fault_recovery_completed)
            self.assertTrue(math.isfinite(thrust))
            self.assertFalse(n.detach_completed or n.recovery_completed)
        n._flight_ready = lambda now: False
        n.flight_state.mode = 'POSCTL'
        self.tick(n, 10.1, 2.8)
        self.assertEqual(n.state, W.STATE_IDLE)

    def raw(self):
        return json.loads((pathlib.Path(__file__).parent/'fixtures'/'detach_height_fault_20261016.json').read_text())['samples']

    def replay_prefix(self):
        rows = self.raw(); n = self.begun(rows[0]['tilt'])
        # Begin at the actual first replay height and velocity, with a trusted
        # output fixture. Subsequent feedback values are unmodified raw samples.
        n.odom_source_stamp = None
        self.f.feedback(n, 10., tilt=rows[0]['tilt'], height=rows[0]['z'], vz=rows[0]['vz'])
        n._begin_detach_vertical(at(10.), n.horizontal_output)
        n.detach_elapsed_s = 2.8
        fault_index = None
        for i, row in enumerate(rows[1:], 1):
            t = 10.+row['t']-rows[0]['t']
            self.tick(n, t, row['z'], row['tilt'], row['vz'],
                      math.degrees(math.sqrt(sum(v*v for v in row['rates']))),
                      source=10.+row['stamp']-rows[0]['stamp'])
            if n.detach_height_continuation.state == 'SUPPORT':
                fault_index = i
                break
        self.assertIsNotNone(fault_index)
        self.assertAlmostEqual(n.detach_height_continuation.residual, .285454626, places=5)
        self.assertEqual(n.state, W.STATE_DETACH_ROTATE)
        return n, rows, fault_index

    def test_raw_fault_prefix_plus_explicit_synthetic_stable_suffix_enters_recover(self):
        n, rows, i = self.replay_prefix()
        row = rows[i]; t0 = 10.+row['t']-rows[0]['t']
        # SYNTHETIC: fixed valid height, level attitude, zero speed/rate.
        # The logged future does not satisfy these conditions.
        for k in range(1, 41):
            self.tick(n, t0+k*.02, row['z'], tilt=0., vz=0., rate=0.)
        self.assertTrue(n._detach_height_loop_ready())
        self.assertEqual(n.state, W.STATE_RECOVER)
        self.assertEqual(n.detach_altitude_m, n.recovery_altitude_m)
        self.assertEqual(n.detach_altitude_m, n.detach_vertical_controller.reference)

    def test_original_unstable_suffix_cannot_complete_normally(self):
        n, rows, i = self.replay_prefix()
        for row in rows[i+1:]:
            self.tick(n, 10.+row['t']-rows[0]['t'], row['z'], row['tilt'], row['vz'],
                      math.degrees(math.sqrt(sum(v*v for v in row['rates']))),
                      source=10.+row['stamp']-rows[0]['stamp'])
            self.assertNotEqual(n.state, W.STATE_RECOVER)
            if n.state == W.STATE_ABORT:
                break
        self.assertEqual(n.state, W.STATE_ABORT)


if __name__ == '__main__':
    unittest.main()
