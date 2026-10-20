import math
import unittest
import threading
from types import SimpleNamespace as NS
from unittest import mock
import rospy
from std_msgs.msg import Bool
from px4_wall_ceiling_control.domain.rc_takeover import RCTakeover
from px4_wall_ceiling_control.domain.geometry import quaternion_angle
from px4_wall_ceiling_control.adapters.rc_takeover import RCTakeoverOutput, RCTakeoverManager


class TransitionTest(unittest.TestCase):
    def test_starts_from_measured_attitude_and_thrust(self):
        q = (0., math.sin(math.pi/4), 0., math.cos(math.pi/4))
        control = RCTakeover(q, .83)
        actual, thrust, _ = control.step(0., 0., 0., .49, 0.)
        self.assertLess(quaternion_angle(q, actual), 1e-7)
        self.assertEqual(thrust, .83)

    def test_large_angle_and_stall_are_rate_limited(self):
        control = RCTakeover((0., 1., 0., 0.), .8)
        previous = control.attitude
        q, thrust, _ = control.step(0., 0., 0., .1, 8.)
        self.assertLessEqual(quaternion_angle(previous, q), math.radians(1.) + 1e-9)
        self.assertAlmostEqual(thrust, .8-.15*.05)

    def test_follows_moving_stick_not_initial_snapshot(self):
        control = RCTakeover((0., 0., 0., 1.), .49)
        for _ in range(200):
            q, _, target = control.step(.15, -.1, 0., .6, .02)
        self.assertLess(quaternion_angle(q, target), 1e-6)
        old = q
        for _ in range(200):
            q, _, target = control.step(-.15, .1, 0., .4, .02)
        self.assertLess(quaternion_angle(q, target), 1e-6)
        self.assertGreater(quaternion_angle(old, q), .3)
        self.assertAlmostEqual(control.thrust, .4)

    def test_opposite_quaternion_sign_has_same_path(self):
        a = RCTakeover((0., .7, 0., .7), .5)
        b = RCTakeover((0., -.7, 0., -.7), .5)
        for _ in range(100):
            qa, _, _ = a.step(0., 0., 0., .5, .02)
            qb, _, _ = b.step(0., 0., 0., .5, .02)
            self.assertLess(quaternion_angle(qa, qb), 1e-7)

    def test_invalid_input_rejected(self):
        with self.assertRaises(ValueError):
            RCTakeover((0., 0., 0., 0.), .5)
        control = RCTakeover((0., 0., 0., 1.), .5)
        with self.assertRaises(ValueError):
            control.step(float('nan'), 0., 0., .5, .02)

    def test_clock_rollback_has_no_step(self):
        control = RCTakeover((0., 0., 0., 1.), .5)
        q, thrust, _ = control.step(1., 1., 1., .8, -1.)
        self.assertEqual(q, (0., 0., 0., 1.))
        self.assertEqual(thrust, .5)


class CoordinationTest(unittest.TestCase):
    def manager(self):
        node = RCTakeoverManager()
        now = rospy.Time.from_sec(10.)
        node.enabled = True
        node.flight = NS(connected=True, armed=True, mode='OFFBOARD')
        node.flight_at = node.rc_takeover_ready_at = node.rc_takeover_active_at = node.output_at = now
        node.estimator_at = now
        node.state_timeout_s = node.estimator_timeout_s = 1.
        node.output_timeout_s = .3
        node.output_mode = 2
        node.rc_takeover_ready = node.rc_takeover_requested = True
        node.estimator = NS(attitude_status_flag=True, velocity_vert_status_flag=True,
                            velocity_horiz_status_flag=True, pos_horiz_rel_status_flag=True)
        node._pilot_mode = mock.Mock(return_value='POSCTL')
        node._wall_fusion_handover_ready = mock.Mock(return_value=True)
        return node, now

    def test_ready_requires_fresh_mux_and_valid_native_estimator(self):
        node, now = self.manager()
        self.assertTrue(node._rc_takeover_handover_ready(now))
        node.rc_takeover_ready_at = rospy.Time.from_sec(9.)
        self.assertFalse(node._rc_takeover_handover_ready(now))
        node.rc_takeover_ready_at = now
        node.estimator.pos_horiz_rel_status_flag = False
        self.assertFalse(node._rc_takeover_handover_ready(now))
        node._pilot_mode.return_value = 'ALTCTL'
        self.assertTrue(node._rc_takeover_handover_ready(now))

    def test_no_handover_on_kill_or_fusion_restore_failure(self):
        node, now = self.manager()
        node._wall_fusion_handover_ready.return_value = False
        self.assertFalse(node._rc_takeover_handover_ready(now))
        node._wall_fusion_handover_ready.return_value = True
        node.kill_guard_enabled = True
        node.kill_status_at = now
        node.kill_status = {'valid': True, 'active': True}
        self.assertFalse(node._rc_takeover_handover_ready(now))

    def output(self):
        node = RCTakeoverOutput()
        now = rospy.Time.from_sec(10.)
        node.rc_takeover_pending = True
        node.rc_takeover_controller = None
        node.rc_takeover_failed = False
        node.flight_state = NS(armed=True, mode='OFFBOARD')
        node.flight_state_at = node.manager_phase_at = now
        node.state_timeout_s = 1.
        node.manager_phase = 'RC_TAKEOVER'
        node._takeover_axes = mock.Mock(return_value=dict(roll=0., pitch=0., yaw=0., throttle=0.))
        node._takeover_inputs_ready = mock.Mock(return_value=True)
        for name in ('rc_takeover_active_pub', 'rc_takeover_ready_pub', 'rc_takeover_status_pub',
                     'mode_publisher', 'attitude_publisher'):
            setattr(node, name, mock.Mock())
        return node, now

    def test_abort_first_output_uses_measured_attitude_and_previous_thrust(self):
        node, now = self.output()
        node.wall_status = NS(state='ABORT')
        q = (0., math.sin(math.pi/4), 0., math.cos(math.pi/4))
        orientation = NS(**dict(zip('xyzw', q)))
        node.odom = NS(pose=NS(pose=NS(orientation=orientation)),
                       twist=NS(twist=NS(linear=NS(x=0., y=0., z=0.))))
        node.odom_twist_in_child_frame = True
        node.hover_thrust, node.vertical_speed_kp = .49, .15
        node.min_thrust, node.max_thrust = .1, .85
        node.last_published_control = ((0., 0., 0., 1.), .81, now, 2)
        node._attitude_target = mock.Mock(return_value=NS(thrust=0.))
        node._remember_publication = mock.Mock()
        node.owner_publisher = mock.Mock()
        self.assertTrue(node._tick_rc_takeover(now))
        args = node._attitude_target.call_args[0]
        actual = tuple(getattr(args[1], k) for k in 'xyzw')
        self.assertLess(quaternion_angle(actual, q), 1e-7)
        self.assertAlmostEqual(node.attitude_publisher.publish.call_args[0][0].thrust, .81)
        self.assertFalse(node.rc_takeover_ready_pub.publish.call_args[0][0].data)

    def test_repeat_request_does_not_reset_controller(self):
        node, _ = self.output()
        node.rc_takeover_lock = threading.RLock()
        controller = object()
        node.rc_takeover_controller = controller
        response = node._request_rc_takeover(None)
        self.assertTrue(response.success)
        self.assertIs(node.rc_takeover_controller, controller)

    def test_handshake_keeps_existing_stream_until_manager_ack(self):
        node, now = self.output()
        node.manager_phase = 'ACTIVE'
        self.assertFalse(node._tick_rc_takeover(now))
        node.attitude_publisher.publish.assert_not_called()

    def test_abort_does_not_gate_takeover(self):
        node, now = self.output()
        node.wall_status = NS(state='ABORT')
        # This reaches thrust-seed validation; abort status itself is not a gate.
        self.assertTrue(node._tick_rc_takeover(now))
        self.assertTrue(node.rc_takeover_failed)
        self.assertIn('推力起点', node.rc_takeover_status_pub.publish.call_args[0][0].data)

    def test_lost_rc_latches_no_fallback_or_automatic_resume(self):
        node, now = self.output()
        node.rc_takeover_controller = object()
        node._takeover_inputs_ready.return_value = False
        self.assertTrue(node._tick_rc_takeover(now))
        node._takeover_inputs_ready.return_value = True
        self.assertTrue(node._tick_rc_takeover(now))
        self.assertTrue(node.rc_takeover_failed)
        node.attitude_publisher.publish.assert_not_called()

    def test_pilot_mode_change_stops_output_immediately(self):
        node, now = self.output()
        node.flight_state.mode = 'ALTCTL'
        self.assertTrue(node._tick_rc_takeover(now))
        self.assertFalse(node.rc_takeover_pending)
        node.attitude_publisher.publish.assert_not_called()

    def test_manager_latches_active_until_native_mode_confirmed(self):
        node, now = self.manager()
        with mock.patch.object(rospy.Time, 'now', return_value=now):
            node._on_rc_takeover_active(Bool(data=False))
        self.assertTrue(node.rc_takeover_requested)


if __name__ == '__main__':
    unittest.main()
