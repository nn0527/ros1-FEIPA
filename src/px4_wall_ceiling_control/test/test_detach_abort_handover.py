"""The detach RECOVER bridge cannot automatically hand back before ABORT."""
import unittest
from unittest import mock

import rospy
from px4_wall_ceiling_control.msg import WallPerchStatus as W
from test_flight_handover import Manager
import test_wall_level_handover as fixture


class DetachAbortHandoverTest(unittest.TestCase):
    def setUp(self):
        self.fx = fixture.WallLevelHandoverTest()
        self.fx.setUp()
        self.n = self.fx.n
        self.n.wall.detach_abort_recovery = True
        self.n.wall.detach_rotation_succeeded = False

    def test_bridge_blocks_convergence_and_legacy_handover_paths(self):
        for enabled in (True, False):
            with self.subTest(enabled=enabled):
                self.setUp()
                n = self.n
                n.wall_level_handover_enabled = enabled
                for t in (0., .1, .2, .3, .4):
                    self.fx.tick(t)
                n._request_mode.assert_not_called()
                self.assertEqual(n.phase, Manager.ACTIVE)
                self.assertIsNone(n.wall_level_window)
                self.assertFalse(n._handover_ready(self.fx.fx.f.now))
                self.assertFalse(n._entry_cancel_handover_ready(self.fx.fx.f.now))
                self.assertFalse(n._wall_pilot_request_phase_ready('POSCTL', self.fx.fx.f.now))
                self.assertIn('等待进入ABORT', n.phase_reason)

    def test_bridge_blocks_owner_acquisition_fallback(self):
        for enabled in (True, False):
            with self.subTest(enabled=enabled):
                self.setUp()
                n = self.n
                n.wall_level_handover_enabled = enabled
                n.owner = 0
                n.owner_seen = False
                n.phase_since = 0.
                self.fx.tick(0.)
                self.assertEqual(n.phase, Manager.ACTIVE)
                n._request_mode.assert_not_called()

    def test_bridge_revokes_pending_returning_and_its_worker(self):
        for enabled in (True, False):
            with self.subTest(enabled=enabled):
                self.setUp()
                n = self.n
                n.wall_level_handover_enabled = enabled
                n.phase = Manager.RETURNING
                n.wall_level_returning = True
                n.wall_level_window = (7, 0, 0, 400000000, 400000000, 5)
                n.set_mode = mock.Mock()
                with mock.patch.object(rospy, 'wait_for_service'), mock.patch.object(
                        rospy.Time, 'now', return_value=self.fx.fx.f.now):
                    n._send_mode_request('POSCTL', 7)
                n.set_mode.assert_not_called()
                self.fx.tick(0.)
                self.assertEqual(n.phase, Manager.ACTIVE)
                self.assertFalse(n.wall_level_returning)
                self.assertIsNone(n.wall_level_window)

    def test_request_and_worker_final_recheck_block_new_bridge(self):
        for enabled in (True, False):
            with self.subTest(enabled=enabled):
                self.setUp()
                n = self.n
                n.wall_level_handover_enabled = enabled
                n.phase = Manager.RETURNING
                n.wall.detach_abort_recovery = False
                calls = []

                def ready(_now):
                    calls.append(True)
                    n.wall.detach_abort_recovery = True
                    return True

                # Simulate a later status arriving after worker gate evaluation.
                n._handover_ready = ready
                n.set_mode = mock.Mock()
                with mock.patch.object(rospy, 'wait_for_service'), mock.patch.object(
                        rospy.Time, 'now', return_value=self.fx.fx.f.now):
                    n._send_mode_request('POSCTL', 7)
                    with mock.patch('threading.Thread') as thread:
                        Manager._request_mode(n, 'POSCTL')
                        thread.assert_not_called()
                self.assertTrue(calls)
                n.set_mode.assert_not_called()

    def test_abort_starts_new_confirmation_window_before_handover(self):
        self.fx.tick(0.)
        self.fx.tick(.1)
        self.n.wall.state = W.STATE_ABORT
        self.n.wall.detach_abort_recovery = False
        for t in (.2, .3, .4):
            self.fx.tick(t)
        self.n._request_mode.assert_not_called()
        self.fx.tick(.5)
        self.n._request_mode.assert_called_once_with('POSCTL')
        self.assertEqual(self.n.phase, Manager.RETURNING)

    def test_default_marker_false_retains_existing_recover_handback(self):
        self.n.wall.detach_abort_recovery = False
        self.fx.confirm()
        self.n._request_mode.assert_called_once_with('POSCTL')

    def test_bridge_marker_does_not_override_abort_phase(self):
        # Phase is authoritative; a delayed marker reset cannot hold ABORT.
        self.n.wall.state = W.STATE_ABORT
        self.fx.confirm()
        self.n._request_mode.assert_called_once_with('POSCTL')

    def test_stale_bridge_status_never_opens_legacy_fallback(self):
        self.n.wall_level_handover_enabled = False
        self.fx.refresh(.1)
        self.n.wall_at = rospy.Time.from_sec(1.)
        self.assertFalse(self.n._handover_ready(self.fx.fx.f.now))
        self.assertFalse(self.n._wall_pilot_request_phase_ready('POSCTL', self.fx.fx.f.now))

    def test_kill_and_pilot_mode_change_keep_existing_priority(self):
        self.n.kill_guard_enabled = True
        self.n.kill_status = {'valid': True, 'active': True}
        self.n.kill_status_at = self.fx.fx.f.now
        self.fx.tick(0.)
        self.assertEqual(self.n.phase, Manager.LOCKED)
        self.assertFalse(self.n.prestream_pub.publish.call_args.args[0].data)
        self.n._request_mode.assert_not_called()

        self.setUp()
        self.n.flight.mode = 'AUTO.LAND'
        self.fx.tick(0.)
        self.assertEqual(self.n.phase, Manager.CANCELING_ENTRY)
        self.assertTrue(self.n.offboard_exit_cleanup)
        self.n._request_mode.assert_not_called()

    def test_already_posctl_can_clear_offboard_intention_without_retake(self):
        self.n.flight.mode = 'POSCTL'
        self.assertTrue(self.n._wall_pilot_request_phase_ready('POSCTL', self.fx.fx.f.now))
        self.fx.tick(0.)
        self.assertEqual(self.n.phase, Manager.CANCELING_ENTRY)
        self.n._request_mode.assert_not_called()

    def test_completed_rotation_via_abort_does_not_claim_full_detach_success(self):
        self.n.wall.state = W.STATE_ABORT
        self.n.wall.detach_abort_recovery = False
        self.n.wall.detach_rotation_succeeded = True
        self.n.wall.fault_detected = False
        self.n._observe_completion(self.fx.fx.f.now)
        self.assertFalse(self.n.fault_seen)
        self.assertFalse(self.n.completed)
        reason = self.n._wall_level_return_reason()
        self.assertIn('脱离旋转已完成', reason)
        self.assertIn('未等待完整离墙验收', reason)
        self.n.wall.fault_detected = True
        self.n._observe_completion(self.fx.fx.f.now)
        self.assertTrue(self.n.fault_seen)

    def test_completed_rotation_abort_exit_retains_gates_without_false_failure(self):
        self.n.wall.state = W.STATE_EXIT
        self.n.wall.detach_abort_recovery = False
        self.n.wall.detach_rotation_succeeded = True
        self.n.wall.fault_detected = False
        self.n.wall.fault_recovery_completed = True
        self.n._observe_completion(self.fx.fx.f.now)
        self.assertFalse(self.n.fault_seen)
        self.assertFalse(self.n.completed)
        self.assertTrue(self.n.wall_fault_return_ready)
        self.assertIn('脱离旋转已完成', self.n._wall_return_reason())
        self.assertNotIn('侧吸未正常完成', self.n._wall_return_reason())
        self.fx.confirm()
        self.n._request_mode.assert_called_once_with('POSCTL')

        self.setUp()
        self.n.wall.state = W.STATE_EXIT
        self.n.wall.detach_rotation_succeeded = True
        self.n.wall.fault_recovery_completed = True
        self.n.fault_seen = True
        self.n._observe_completion(self.fx.fx.f.now)
        self.assertTrue(self.n.fault_seen)
        self.assertIn('侧吸未正常完成', self.n._wall_return_reason())


if __name__ == '__main__':
    unittest.main()
