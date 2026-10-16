#!/usr/bin/env python3
"""Candidate age, atomic phase and maneuver lifecycle at the final output boundary."""
import importlib.util
import math
import pathlib
import unittest
from unittest import mock

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget, State
from nav_msgs.msg import Odometry
from px4_wall_ceiling_control.msg import AttachmentControlCandidate as Candidate, WallPerchStatus as Wall

_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts/outputs/attachment_output_mux_node.py"
_SPEC = importlib.util.spec_from_file_location("candidate_latency_mux", str(_PATH))
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
Mux = _MODULE.AttachmentOutputMuxNode


def tilted(degrees):
    angle = math.radians(degrees) / 2.0
    return Quaternion(y=math.sin(angle), w=math.cos(angle))


def tilt(target):
    q = target.orientation
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0-2.0*(q.x*q.x+q.y*q.y)))))


class CandidateLatencyTest(unittest.TestCase):
    def setUp(self):
        self.now = rospy.Time.from_sec(10.0)
        patch = mock.patch.object(rospy.Time, "now", side_effect=lambda: self.now)
        patch.start(); self.addCleanup(patch.stop)
        patch = mock.patch.object(rospy, "logwarn_throttle")
        patch.start(); self.addCleanup(patch.stop)
        n = self.node = Mux.__new__(Mux)
        n.owner = Mux.OWNER_NONE
        n.wall_output_enabled = True
        n.wall_candidate_timeout_s = .20
        n.wall_candidate = n.wall_candidate_at = n.wall_candidate_snapshot = None
        n.wall_maneuver_id = None
        n.wall_retired_maneuvers = set()
        n.wall_maneuver_closed = False
        n.wall_last_sequence = 0
        n.wall_last_source_stamp = None
        n.flight_state = State(connected=True, armed=True, mode="OFFBOARD")
        n.flight_state_at = self.now
        n.state_timeout_s = 1.5
        n.odom = Odometry()
        n.odom.header.stamp = self.now
        n.odom.pose.pose.orientation.w = 1.0
        n.odom_at = self.now
        n.odom_timeout_s = .5
        n.odom_twist_in_child_frame = True
        n.rc_assist_enabled = False
        n.approach_attitude_limits_enabled = n.recovery_attitude_limits_enabled = True
        n.approach_max_tilt_deg = n.recovery_max_tilt_deg = 5.0
        n.approach_tilt_slew_deg_s = n.recovery_tilt_slew_deg_s = 10.0
        n.approach_yaw_slew_deg_s = n.recovery_yaw_slew_deg_s = 20.0
        n.recovery_final_thrust_cap = .60
        n.approach_previous_target = n.recovery_previous_target = None
        n.approach_previous_at = n.recovery_previous_at = None
        n.approach_target_publisher = mock.Mock()
        n.recovery_target_publisher = mock.Mock()
        n.recovery_horizontal_publisher = mock.Mock()
        n.attitude_publisher = mock.Mock()
        n.status_timeout_s = .30
        n.wall_status = Wall(state=Wall.STATE_SLOW_APPROACH)
        n.wall_status_at = self.now
        n.max_velocity_z_mps = .30
        n.min_thrust, n.max_thrust, n.hover_thrust = .10, .85, .48
        n.vertical_speed_kp = .15
        n.local_frame_id = "map"
        n.match_fcu_thrust_on_handover = False

    def candidate(self, phase=Wall.STATE_SLOW_APPROACH, sequence=1, maneuver="run-a", stamp=None,
                  acquire=True, keep=True, angle=0.0):
        result = Candidate(source=Candidate.SOURCE_WALL, kind=Candidate.KIND_ATTITUDE_THRUST,
                           acquire_request=acquire, keep_ownership=keep, valid=True,
                           phase_valid=True, control_phase=phase, maneuver_id=maneuver,
                           control_sequence=sequence, attitude_setpoint=tilted(angle),
                           thrust_normalized=.70, horizontal_velocity_enabled=True)
        result.header.stamp = self.now if stamp is None else rospy.Time.from_sec(stamp)
        return result

    def advance(self, seconds=.02):
        self.now += rospy.Duration.from_sec(seconds)

    def accept(self, candidate):
        self.node._on_wall_candidate(candidate)
        self.assertIs(self.node.wall_candidate, candidate, self.node.wall_candidate_rejection)

    def seed_publication(self, angle=0, thrust=.5, age=0.0, source=Candidate.SOURCE_WALL):
        target = AttitudeTarget(orientation=tilted(angle), thrust=thrust)
        self.node._remember_publication(target, source, self.now-rospy.Duration.from_sec(age))

    def test_stale_source_cannot_be_revived_by_fresh_receipt(self):
        self.node._on_wall_candidate(self.candidate(stamp=9.5))
        self.assertIsNone(self.node.wall_candidate)
        self.assertFalse(self.node._wall_fresh(self.now))
        self.assertEqual(self.node.wall_candidate_rejection, "source_stamp_stale_zero_or_future")

    def test_zero_future_and_missing_metadata_rejected(self):
        for stamp in (0.0, 10.001):
            self.node._on_wall_candidate(self.candidate(stamp=stamp))
            self.assertIsNone(self.node.wall_candidate)
        candidate = self.candidate()
        candidate.phase_valid = False
        self.node._on_wall_candidate(candidate)
        self.assertIsNone(self.node.wall_candidate)

    def test_reordered_sequence_repeated_stamp_and_replayed_receipt_are_rejected(self):
        first = self.candidate(sequence=3)
        self.accept(first)
        first_receipt = self.node.wall_candidate_at
        self.advance()
        for sequence, stamp in ((2, 10.02), (3, 10.02), (4, 10.0), (5, 9.99)):
            self.node._on_wall_candidate(self.candidate(sequence=sequence, stamp=stamp))
            self.assertIs(self.node.wall_candidate, first)
            self.assertEqual(self.node.wall_candidate_at, first_receipt)
        self.advance(.20)
        self.assertFalse(self.node._wall_fresh(self.now))

    def test_source_timeout_rechecked_at_publication(self):
        candidate = self.candidate()
        self.accept(candidate)
        self.node.wall_candidate_at = self.now + rospy.Duration.from_sec(.15)
        self.advance(.21)
        self.assertEqual(self.node._publish_candidate(self.now, candidate), Mux.OUTPUT_NONE)
        self.node.attitude_publisher.publish.assert_not_called()

    def test_candidate_expiring_during_output_processing_is_not_sent(self):
        candidate = self.candidate()
        self.accept(candidate)
        original = self.node._attitude_target
        def delayed(*args, **kwargs):
            self.advance(.21)
            return original(*args, **kwargs)
        self.node._attitude_target = delayed
        self.assertEqual(self.node._publish_candidate(rospy.Time.from_sec(10), candidate), Mux.OUTPUT_NONE)
        self.node.attitude_publisher.publish.assert_not_called()
        self.assertEqual(self.node.wall_candidate_rejection, "candidate_expired_before_publication")

    def test_flip_candidate_not_limited_by_old_approach_status(self):
        candidate = self.candidate(phase=Wall.STATE_FLIP_TO_WALL, angle=60)
        self.accept(candidate)
        self.seed_publication(angle=60, thrust=.70, age=.02)
        self.assertEqual(self.node._publish_candidate(self.now, candidate), Mux.OUTPUT_ATTITUDE_THRUST)
        target = self.node.attitude_publisher.publish.call_args.args[0]
        self.assertAlmostEqual(tilt(target), 60.0)
        self.assertAlmostEqual(target.thrust, .70)
        self.node.rc_free_max_angle_deg = 8
        self.node.rc_yaw_max_deg = 15
        self.assertEqual(self.node._roll_yaw_limits(self.now, Candidate.SOURCE_WALL, candidate), (0, 0))

    def test_recovery_candidate_enforces_cone_despite_old_flip_status(self):
        self.accept(self.candidate())
        self.advance()
        candidate = self.candidate(phase=Wall.STATE_RECOVER, sequence=2, acquire=False, angle=60)
        self.accept(candidate)
        self.node.wall_status.state = Wall.STATE_FLIP_TO_WALL
        self.seed_publication(angle=4, thrust=.60, age=.02)
        self.assertEqual(self.node._publish_candidate(self.now, candidate), Mux.OUTPUT_ATTITUDE_THRUST)
        target = self.node.attitude_publisher.publish.call_args.args[0]
        self.assertLessEqual(tilt(target), 5.000001)
        self.assertLessEqual(target.thrust, .60)

    def test_detach_recovery_selection_uses_same_candidate_flags(self):
        self.accept(self.candidate())
        self.advance()
        candidate = self.candidate(phase=Wall.STATE_DETACH_ROTATE, sequence=2, acquire=False, angle=60)
        candidate.horizontal_velocity_active = True
        self.accept(candidate)
        self.node.wall_status.horizontal_velocity_active = False
        self.node.wall_status.state = Wall.STATE_FLIP_TO_WALL
        self.seed_publication(angle=4, thrust=.60, age=.02)
        self.node._publish_candidate(self.now, candidate)
        self.assertLessEqual(tilt(self.node.attitude_publisher.publish.call_args.args[0]), 5.000001)

    def test_new_maneuver_cannot_replace_active_owned_command(self):
        first = self.candidate()
        self.accept(first)
        self.node.owner = Mux.OWNER_WALL
        self.advance()
        self.node._on_wall_candidate(self.candidate(maneuver="restarted-node"))
        self.assertIs(self.node.wall_candidate, first)
        self.assertEqual(self.node.wall_candidate_rejection, "new_maneuver_while_previous_active")

    def test_release_allows_new_acquisition_but_rejects_retired_maneuver(self):
        self.accept(self.candidate())
        self.advance()
        self.accept(self.candidate(phase=Wall.STATE_IDLE, sequence=2, acquire=False, keep=False))
        self.advance()
        self.node._on_wall_candidate(self.candidate(sequence=3))
        self.assertEqual(self.node.wall_candidate_rejection, "closed_maneuver_cannot_reacquire")
        new = self.candidate(maneuver="run-b")
        self.accept(new)
        self.advance()
        self.node._on_wall_candidate(self.candidate(sequence=4))
        self.assertIs(self.node.wall_candidate, new)
        self.assertEqual(self.node.wall_candidate_rejection, "retired_maneuver")

    def test_offboard_exit_permits_restart_after_safe_rearm(self):
        self.accept(self.candidate())
        self.node.owner = Mux.OWNER_WALL
        self.node._on_state(State(connected=True, armed=False, mode="POSCTL"))
        self.assertIsNone(self.node.wall_candidate)
        self.assertIn("run-a", self.node.wall_retired_maneuvers)
        self.node.owner = Mux.OWNER_NONE  # Owner arbiter releases on missing candidate.
        self.advance()
        self.accept(self.candidate(maneuver="run-b"))

    def test_idle_metadata_does_not_block_new_acquisition(self):
        self.accept(self.candidate(phase=Wall.STATE_IDLE, acquire=False, keep=False))
        self.advance()
        self.accept(self.candidate(maneuver="actual-acquisition"))

    def test_publish_time_and_original_age_both_retained(self):
        candidate = self.candidate(stamp=9.95)
        self.accept(candidate)
        self.node._publish_candidate(self.now, candidate)
        target = self.node.attitude_publisher.publish.call_args.args[0]
        self.assertEqual(target.header.stamp, self.now)
        self.assertEqual(candidate.header.stamp, rospy.Time.from_sec(9.95))
        diagnostic = self.node.last_wall_command
        self.assertEqual(diagnostic["control_sequence"], 1)
        self.assertEqual(diagnostic["control_phase"], Wall.STATE_SLOW_APPROACH)
        self.assertEqual(diagnostic["maneuver_id"], "run-a")
        self.assertAlmostEqual(diagnostic["source_age_s"], .05)
        self.assertEqual(diagnostic["published_at_s"], 10)

    def test_owner_intent_and_freshness_use_one_atomic_pair(self):
        candidate = self.candidate()
        self.accept(candidate)
        # Simulate the callback's legacy convenience fields being ahead of the
        # atomic pair; owner arbitration must use the pair for both decisions.
        self.node.wall_candidate = self.candidate(acquire=False, keep=False)
        self.node.interlock_active = False
        self.node.interlock_clear_since = None
        self.node.handoff_dwell_s = .50
        from px4_wall_ceiling_control.msg import OperatorCommand
        self.node._operator_mode = lambda now: OperatorCommand.MODE_WALL
        self.node._update_owner(self.now)
        self.assertEqual(self.node.owner, Mux.OWNER_WALL)

    def test_wall_odom_requires_source_freshness_but_ceiling_is_unchanged(self):
        self.node.odom.header.stamp = rospy.Time.from_sec(9.0)
        self.node.owner = Mux.OWNER_WALL
        self.assertFalse(self.node._odom_fresh(self.now))
        self.node.owner = Mux.OWNER_CEILING
        self.assertTrue(self.node._odom_fresh(self.now))

    def test_mux_only_stalls_bound_both_attitude_and_thrust_with_same_dt(self):
        self.accept(self.candidate())
        for duration, sequence in ((.10, 2), (.20, 3)):
            self.seed_publication(angle=40, thrust=.5)
            self.advance(duration)
            candidate = self.candidate(phase=Wall.STATE_FLIP_TO_WALL, sequence=sequence,
                                       acquire=False, angle=80)
            self.accept(candidate)
            self.node._publish_candidate(self.now, candidate)
            target = self.node.attitude_publisher.publish.call_args.args[0]
            self.assertAlmostEqual(tilt(target), 40+84.375*.05, places=5)
            self.assertAlmostEqual(target.thrust, .525)
            self.assertAlmostEqual(self.node.last_wall_command["publication_slew"]["dt_s"], .05)
            self.assertEqual(target.header.stamp, self.now)
            self.assertAlmostEqual(self.node.last_wall_command["thrust"], target.thrust)

    def test_phase_boundary_keeps_last_actually_published_step_history(self):
        self.accept(self.candidate())
        self.seed_publication(angle=4, thrust=.51, source=Candidate.SOURCE_NONE)
        self.advance()
        flip = self.candidate(phase=Wall.STATE_FLIP_TO_WALL, sequence=2, acquire=False, angle=60)
        self.accept(flip)
        self.node._publish_candidate(self.now, flip)
        first = self.node.attitude_publisher.publish.call_args.args[0]
        self.advance()
        capture = self.candidate(phase=Wall.STATE_WALL_CAPTURE, sequence=3, acquire=False, angle=90)
        self.accept(capture)
        self.node._publish_candidate(self.now, capture)
        second = self.node.attitude_publisher.publish.call_args.args[0]
        self.assertLessEqual(tilt(second)-tilt(first), 84.375*.02+1e-5)
        self.assertAlmostEqual(second.thrust-first.thrust, .50*.02)
        self.assertEqual(self.node.last_published_control[2], self.now)

    def test_abort_projection_is_not_delayed_by_maneuver_slew(self):
        self.accept(self.candidate())
        self.seed_publication(angle=60, thrust=.7)
        self.advance()
        candidate = self.candidate(phase=Wall.STATE_ABORT, sequence=2, acquire=False, angle=60)
        self.accept(candidate)
        self.node._publish_candidate(self.now, candidate)
        target = self.node.attitude_publisher.publish.call_args.args[0]
        self.assertLessEqual(tilt(target), 5.000001)
        self.assertAlmostEqual(target.thrust, .69)
        # The soft cap is approached continuously, not abandoned or applied as
        # a discontinuous pressure cut on the first safe-level command.
        for sequence in range(candidate.control_sequence+1, candidate.control_sequence+12):
            self.advance()
            next_candidate = self.candidate(phase=Wall.STATE_ABORT, sequence=sequence, acquire=False, angle=60)
            self.accept(next_candidate)
            previous = target.thrust
            self.node._publish_candidate(self.now, next_candidate)
            target = self.node.attitude_publisher.publish.call_args.args[0]
            self.assertLessEqual(tilt(target), 5.000001)
            self.assertLessEqual(previous-target.thrust, .010001)
        self.assertAlmostEqual(target.thrust, .60)

    def test_no_valid_initial_slew_seed_inhibits_flip(self):
        candidate = self.candidate(phase=Wall.STATE_FLIP_TO_WALL, angle=50)
        self.accept(candidate)
        self.assertEqual(self.node._publish_candidate(self.now, candidate), Mux.OUTPUT_NONE)
        self.node.attitude_publisher.publish.assert_not_called()
        self.assertEqual(self.node.wall_candidate_rejection, "final_slew_seed_unavailable")

    def test_fresh_measured_attitude_and_fcu_thrust_are_valid_initial_seed(self):
        candidate = self.candidate(phase=Wall.STATE_FLIP_TO_WALL, angle=50)
        self.accept(candidate)
        self.node.odom.pose.pose.orientation = tilted(4)
        self.node.fcu_attitude_target = AttitudeTarget(orientation=tilted(4), thrust=.518)
        self.node.fcu_attitude_target.header.stamp = self.now
        self.node.fcu_attitude_target_at = self.now
        self.assertEqual(self.node._publish_candidate(self.now, candidate), Mux.OUTPUT_ATTITUDE_THRUST)
        target = self.node.attitude_publisher.publish.call_args.args[0]
        self.assertAlmostEqual(tilt(target), 4)
        self.assertAlmostEqual(target.thrust, .518)

    def test_rejected_command_does_not_advance_publication_history(self):
        candidate = self.candidate(phase=Wall.STATE_FLIP_TO_WALL, angle=50)
        self.accept(candidate)
        self.seed_publication(angle=10, thrust=.518)
        previous = self.node.last_published_control
        original = self.node._limit_wall_publication
        def delayed(*args):
            result = original(*args)
            self.advance(.21)
            return result
        self.node._limit_wall_publication = delayed
        self.assertEqual(self.node._publish_candidate(rospy.Time.from_sec(10), candidate), Mux.OUTPUT_NONE)
        self.assertEqual(previous, self.node.last_published_control)
        self.node.attitude_publisher.publish.assert_not_called()

    def test_paused_nonhorizontal_approach_cannot_apply_preflip_roll_bias(self):
        candidate = self.candidate()
        candidate.horizontal_velocity_enabled = False
        candidate.approach_paused = True
        self.accept(candidate)
        self.node.preflip_right_roll_deg = 3
        from px4_wall_ceiling_control.msg import OperatorCommand
        self.node._operator_mode = lambda now: OperatorCommand.MODE_WALL
        output = self.node._preflip_bias_attitude(self.now, tilted(0), Candidate.SOURCE_WALL, candidate)
        self.assertAlmostEqual(output.x, 0)
        self.assertAlmostEqual(self.node.preflip_bias_rad, 0)

    def test_ceiling_protocol_does_not_require_wall_metadata(self):
        candidate = Candidate(source=Candidate.SOURCE_CEILING, kind=Candidate.KIND_ATTITUDE_THRUST,
                              attitude_setpoint=tilted(0), thrust_normalized=.5)
        self.assertEqual(self.node._publish_candidate(self.now, candidate), Mux.OUTPUT_ATTITUDE_THRUST)
        self.assertAlmostEqual(self.node.attitude_publisher.publish.call_args.args[0].thrust, .5)


if __name__ == "__main__":
    unittest.main()
