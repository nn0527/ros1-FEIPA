"""Wheel correction retains wall pressure until a stable, manual departure."""
import json
import math
import pathlib
import unittest
from unittest import mock

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from px4_wall_ceiling_control.msg import WallPerchStatus as W
from px4_wall_ceiling_control.domain.geometry import quaternion_angle, rotate_vector
from test_attachment_safety import WallDecision
import test_detach_state_machine as detach_fixtures
from test_detach_state_machine import at


class ManualWallRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.f = detach_fixtures.DetachStateMachineTests()

    def node(self):
        n = self.f.node(**{'~manual_wall_recovery_enabled': True})
        n._safety_ok = WallDecision._safety_ok.__get__(n)
        return n

    def test_slip_and_large_rotation_keep_wall_pressure_without_level_command(self):
        n = self.node()
        self.f.feedback(n, 10.02, tilt=55., vz=-1.4, rate=350.)
        n.wall_pressure_thrust_command = .74
        self.f.tick(n, 10.02)
        self.assertEqual(n.state, W.STATE_WALL_HOLD)
        self.assertEqual(n.fault_count, 0)
        _, q, thrust, _ = n._candidate(at(10.02))
        self.assertLess(quaternion_angle(q, n.q_contact), 1.e-7)
        self.assertAlmostEqual(thrust, .74)
        self.assertIn('保持压墙推力', n._manual_wall_recovery_reason(at(10.02)))
        self.assertFalse(n._detach_start_ready(at(10.02)))

    def test_range_increase_during_wheel_correction_does_not_force_abort(self):
        n = self.node()
        self.f.top(n, 10.02, .25)
        self.f.feedback(n, 10.6)
        self.f.top(n, 10.6, .25)
        self.assertTrue(n._top_clear_ready(at(10.6)))
        self.f.tick(n, 10.6)
        self.assertEqual(n.state, W.STATE_WALL_HOLD)
        self.assertIn('顶部距离偏大', n._manual_wall_recovery_reason(at(10.6)))

    def test_waiting_does_not_automatically_detach_even_after_pose_recovers(self):
        n = self.node(); n.hold_time_s = .1
        for t in (10.1, 10.3, 10.5, 10.7, 11.):
            self.f.feedback(n, t); self.f.output(n, t)
            self.f.tick(n, t)
        self.assertTrue(n._detach_start_ready(at(11.)))
        self.assertEqual(n.state, W.STATE_WALL_HOLD)
        self.assertFalse(n.detach_requested)

    def test_manual_request_waits_for_recovery_then_uses_normal_departure(self):
        n = self.node()
        self.f.feedback(n, 10.02, tilt=55., vz=-1.2, rate=350.)
        self.f.output(n, 10.02, thrust=.74)
        with mock.patch.object(rospy.Time, 'now', return_value=at(10.02)):
            n._on_detach(Bool(data=True))
        self.f.tick(n, 10.02)
        self.assertEqual(n.state, W.STATE_WALL_HOLD)
        self.assertTrue(n.detach_requested)
        for t in (10.1, 10.3, 10.5, 10.7):
            self.f.feedback(n, t, tilt=90., vz=0., rate=0.)
            self.f.output(n, t, thrust=.74)
            self.f.tick(n, t)
        self.assertEqual(n.state, W.STATE_DETACH_ROTATE)
        _, q, thrust, _ = n._candidate(at(10.7))
        self.assertLess(quaternion_angle(q, n.q_contact), 1.e-7)
        self.assertAlmostEqual(thrust, .74)

    def test_flight_or_odom_failure_still_aborts(self):
        for gate in ('_flight_ready', '_odom_fresh'):
            n = self.node()
            setattr(n, gate, lambda now: False)
            self.f.tick(n, 10.02)
            self.assertEqual(n.state, W.STATE_ABORT, gate)

    def test_nonfinite_feedback_cannot_be_hidden_by_manual_recovery(self):
        for component in ('position', 'velocity', 'angular'):
            n = self.node()
            if component == 'position': n.odom.pose.pose.position.z = math.nan
            elif component == 'velocity': n.odom.twist.twist.linear.x = math.nan
            else: n.odom.twist.twist.angular.z = math.nan
            self.assertFalse(n._safety_ok(at(10.02)), component)

    def test_no_top_frame_does_not_bypass_prehold_descent_protection(self):
        for phase in (W.STATE_FLIP_TO_WALL, W.STATE_WALL_CAPTURE):
            n = self.node(); n.state = phase
            n.top_hold_accept_after = at(10.); n.top_hold_frame_at = None
            self.f.feedback(n, 10.02, vz=-1.2)
            self.assertFalse(n._manual_wall_recovery_active(at(10.02)))
            self.assertFalse(n._safety_ok(at(10.02)))

    def test_late_top_frame_admits_hold_before_kinematic_abort(self):
        for phase in (W.STATE_FLIP_TO_WALL, W.STATE_WALL_CAPTURE):
            n = self.node(); n.state = phase; n.state_entered_at = at(10.)
            n.top_hold_accept_after = at(10.); n.top_hold_frame_at = None
            n.flip_elapsed_s = n.flip_time_s
            n.flip_step_at = at(10.02)
            n.q_flip_start = n.q_hover
            self.f.feedback(n, 10.02, vz=-1.2, rate=350.)
            self.f.top(n, 10.02, .26)
            self.assertTrue(n._manual_wall_recovery_active(at(10.02)))
            self.f.tick(n, 10.02)
            self.assertEqual(n.state, W.STATE_WALL_HOLD)

    def test_disabling_parameter_keeps_original_hold_abort(self):
        n = self.node(); n.manual_wall_recovery_enabled = False
        self.f.feedback(n, 10.02, rate=350.)
        self.f.tick(n, 10.02)
        self.assertEqual(n.state, W.STATE_ABORT)

    def test_wheel_turn_keeps_thrust_axis_toward_wall(self):
        from px4_wall_ceiling_control.domain.geometry import quaternion_multiply
        n = self.node()
        turn = (0., 0., math.sin(math.radians(60)), math.cos(math.radians(60)))
        q = quaternion_multiply(n.q_contact, turn)
        for k, v in zip('xyzw', q):setattr(n.odom.pose.pose.orientation, k, v)
        followed = n._hold_follow_quaternion()
        self.assertLess(quaternion_angle(followed, q), 1.e-7)
        for a, b in zip(rotate_vector(followed, (0, 0, 1)), rotate_vector(n.q_contact, (0, 0, 1))):
            self.assertAlmostEqual(a, b)

    def test_recorded_1224_capture_window_reaches_hold_instead_of_abort(self):
        path = pathlib.Path(__file__).parent/'fixtures/manual_wall_recovery_20261020_1224.json'
        data = json.loads(path.read_text())
        n = self.node(); n.state = W.STATE_WALL_CAPTURE
        n.state_entered_at = at(data['capture_header_unix_s'])
        n.top_hold_accept_after = at(data['top_accept_after_unix_s'])
        n.top_hold_frame_at = None
        n.odom_source_stamp = None
        n.top_source_stamp = None
        n.q_contact = tuple(data['last_vertical_target_xyzw'])
        n.q_hold_reference = n.q_contact
        n.wall_pressure_thrust_command = .648
        for e in data['events']:
            now = at(e['receipt_unix_s'])
            with mock.patch.object(rospy.Time, 'now', return_value=now):
                if e['kind'] == 'odom':
                    m = Odometry(); m.header.stamp = at(e['header_unix_s'])
                    for k, v in zip('xyzw', e['q_xyzw']):setattr(m.pose.pose.orientation, k, v)
                    for k, v in zip('xyz', e['position']):setattr(m.pose.pose.position, k, v)
                    for k, v in zip('xyz', e['linear_body']):setattr(m.twist.twist.linear, k, v)
                    for k, v in zip('xyz', e['angular_body']):setattr(m.twist.twist.angular, k, v)
                    n.odom_twist_in_child_frame = True
                    n._on_odom(m)
                else:
                    m = Range(range=e['range'], min_range=e['min_range'], max_range=e['max_range'])
                    m.header.stamp = at(e['header_unix_s']); n._on_top_range(m)
        now = at(data['original_abort_header_unix_s'])
        self.f.tick(n, now.to_sec())
        self.assertEqual(n.state, W.STATE_WALL_HOLD)
        _, q, thrust, _ = n._candidate(now)
        self.assertLess(quaternion_angle(q, n.q_contact), 1.e-7)
        self.assertGreaterEqual(thrust, .648)


if __name__ == '__main__':
    unittest.main()
