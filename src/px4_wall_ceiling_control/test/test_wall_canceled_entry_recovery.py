"""Canceled entry reports ABORT while the existing owner-zero controller runs."""
import importlib.util
import math
import pathlib
import unittest
from unittest import mock

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from px4_wall_ceiling_control.msg import AttachmentControlCandidate as C
from px4_wall_ceiling_control.msg import OperatorCommand, WallPerchStatus as W

path = pathlib.Path(__file__).resolve().parents[1] / "scripts/decisions/wall_perch_decision_node.py"
spec = importlib.util.spec_from_file_location("wall_canceled_entry_recovery_test", str(path))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class CanceledEntryRecoveryTests(unittest.TestCase):
    def setUp(self):
        patch = mock.patch.object(rospy, "loginfo")
        patch.start()
        self.addCleanup(patch.stop)

    def node(self):
        n = module.WallPerchDecisionNode.__new__(module.WallPerchDecisionNode)
        now = rospy.Time.from_sec(10.)
        n.state = W.STATE_IDLE
        n.decision_enabled = True
        n.cancel_requested = True
        n.cancel_requested_at = now
        n.canceled_entry_recovery = False
        n.manager_phase = "CANCELING_ENTRY"
        n.manager_phase_received_at = now
        n.manager_phase_timeout_s = .5
        n.flight_state = State(connected=True, armed=True, mode="OFFBOARD")
        n.flight_state_received_at = now
        n.state_timeout_s = 1.5
        n.require_armed = n.require_offboard = True
        n.operator = OperatorCommand(mode=OperatorCommand.MODE_STANDBY)
        n.operator_received_at = now
        n.operator_timeout_s = .5
        n.horizontal_output_owner = C.SOURCE_NONE
        n.horizontal_output_owner_at = now
        n.horizontal_velocity_enabled = False
        n.odom = Odometry()
        n.odom.header.stamp = now
        n.odom.pose.pose.position.z = 2.4
        n.odom.pose.pose.orientation.z = math.sin(.7 / 2.)
        n.odom.pose.pose.orientation.w = math.cos(.7 / 2.)
        n.odom_received_at = now
        n.odom_timeout_s = .5
        n.odom_twist_in_child_frame = False
        n.wall_angle_rad = math.pi / 2.
        n.fault_count = 0
        n.q_hover = (0., 0., 0., 1.)
        n.maneuver_id = "idle-before-canceled-entry"
        n.control_sequence = 4
        return n, now

    def test_idle_explicit_cancel_enters_abort_with_current_heading(self):
        n, now = self.node()
        n._update_state_machine(now)
        self.assertEqual(n.state, W.STATE_ABORT)
        self.assertTrue(n.canceled_entry_recovery)
        self.assertAlmostEqual(n._euler_from_quaternion(n.q_hover)[2], .7)
        self.assertEqual(n.fault_count, 1)
        self.assertEqual(n.maneuver_id, "idle-before-canceled-entry")
        self.assertFalse(n.detach_completed if hasattr(n, "detach_completed") else False)

    def test_requires_explicit_wall_cancel_not_manager_phase_alone(self):
        n, now = self.node()
        n.cancel_requested = False
        self.assertFalse(n._update_canceled_entry_recovery(now))
        self.assertEqual(n.state, W.STATE_IDLE)
        with mock.patch.object(rospy.Time, "now", return_value=now):
            n._on_cancel(Bool(data=False))
            self.assertFalse(n.cancel_requested)
            n._on_cancel(Bool(data=True))
        self.assertEqual(n.cancel_requested_at, now)
        self.assertTrue(n._update_canceled_entry_recovery(now))

    def test_wrong_stale_or_inactive_authority_never_enters_abort(self):
        for fault in ("disabled", "not_idle", "cancel_old", "phase", "phase_old",
                      "missing_fcu", "disconnected", "disarmed", "posctl", "fcu_old",
                      "ceiling", "owner_ceiling", "owner_wall", "owner_old", "kill"):
            with self.subTest(fault=fault):
                n, now = self.node()
                if fault == "disabled": n.decision_enabled = False
                elif fault == "not_idle": n.state = W.STATE_WALL_HOLD
                elif fault == "cancel_old": n.cancel_requested_at = rospy.Time.from_sec(9.)
                elif fault == "phase": n.manager_phase = "ACTIVE"
                elif fault == "phase_old": n.manager_phase_received_at = rospy.Time.from_sec(9.)
                elif fault == "missing_fcu": n.flight_state = None
                elif fault == "disconnected": n.flight_state.connected = False
                elif fault == "disarmed": n.flight_state.armed = False
                elif fault == "posctl": n.flight_state.mode = "POSCTL"
                elif fault == "fcu_old": n.flight_state_received_at = rospy.Time.from_sec(8.)
                elif fault == "ceiling": n.operator.mode = OperatorCommand.MODE_CEILING
                elif fault == "owner_ceiling": n.horizontal_output_owner = C.SOURCE_CEILING
                elif fault == "owner_wall": n.horizontal_output_owner = C.SOURCE_WALL
                elif fault == "owner_old": n.horizontal_output_owner_at = rospy.Time.from_sec(9.)
                elif fault == "kill":
                    n.kill_guard_enabled = True
                    n.kill_status_at = now
                    n.kill_status = {"valid": True, "active": True}
                self.assertFalse(n._canceled_entry_recovery_ready(now))

    def test_invalid_or_stale_odometry_never_enters_abort(self):
        for fault in ("missing", "receipt", "source", "nan_height", "nan_velocity", "bad_attitude"):
            with self.subTest(fault=fault):
                n, now = self.node()
                if fault == "missing": n.odom = None
                elif fault == "receipt": n.odom_received_at = rospy.Time.from_sec(9.)
                elif fault == "source": n.odom.header.stamp = rospy.Time.from_sec(9.)
                elif fault == "nan_height": n.odom.pose.pose.position.z = math.nan
                elif fault == "nan_velocity": n.odom.twist.twist.linear.x = math.nan
                else: n.odom.pose.pose.orientation.w = n.odom.pose.pose.orientation.z = 0.
                self.assertFalse(n._canceled_entry_recovery_ready(now))

    def test_waiting_for_odometry_preserves_cancel_request(self):
        n, now = self.node()
        n.odom_received_at = rospy.Time.from_sec(8.)
        self.assertFalse(n._update_canceled_entry_recovery(now))
        self.assertTrue(n.cancel_requested)
        n.odom_received_at = now
        self.assertTrue(n._update_canceled_entry_recovery(now))

    def test_special_abort_never_takes_output_ownership_or_fabricates_reference(self):
        n, now = self.node()
        n._update_state_machine(now)
        status = n._build_status(now)
        candidate = n._build_control_candidate(status, n.maneuver_id, n.control_sequence)
        self.assertEqual(status.state, W.STATE_ABORT)
        self.assertTrue(status.fault_detected)
        self.assertFalse(status.enabled)
        self.assertFalse(status.acquire_request)
        self.assertFalse(status.keep_ownership)
        self.assertFalse(status.vertical_control_active)
        self.assertFalse(status.vertical_reference_valid)
        self.assertEqual(status.vertical_reference_source, "output_mux_cancel_vertical_control")
        self.assertFalse(candidate.valid)
        self.assertFalse(candidate.acquire_request)
        self.assertFalse(candidate.keep_ownership)
        self.assertEqual(candidate.kind, C.KIND_NONE)
        self.assertIsNone(n._candidate(now)[1])
        self.assertTrue(math.isnan(n._candidate(now)[2]))

    def test_repeated_ticks_stay_abort_without_resetting_seed_or_normal_completion(self):
        n, now = self.node()
        n._update_state_machine(now)
        state_entered = n.state_entered_at
        n._prepare_detach_vertical_feedback = mock.Mock(side_effect=AssertionError("must retain mux controller"))
        n._update_state_machine(rospy.Time.from_sec(10.1))
        self.assertEqual(n.state, W.STATE_ABORT)
        self.assertEqual(n.state_entered_at, state_entered)
        self.assertEqual(n.fault_count, 1)
        self.assertFalse(n._acquire_requested(now))

    def test_release_requires_fresh_connected_pilot_mode_or_disarm(self):
        for change in ("fresh_posctl", "fresh_disarm", "stale_posctl", "disconnected"):
            with self.subTest(change=change):
                n, now = self.node()
                n._update_state_machine(now)
                if change == "fresh_disarm": n.flight_state.armed = False
                else: n.flight_state.mode = "POSCTL"
                if change == "stale_posctl": n.flight_state_received_at = rospy.Time.from_sec(8.)
                if change == "disconnected": n.flight_state.connected = False
                n._update_state_machine(now)
                released = change in ("fresh_posctl", "fresh_disarm")
                self.assertEqual(n.state, W.STATE_IDLE if released else W.STATE_ABORT)
                self.assertEqual(n.canceled_entry_recovery, not released)
                if released:
                    self.assertIsNone(n.cancel_requested_at)


if __name__ == "__main__":
    unittest.main()
