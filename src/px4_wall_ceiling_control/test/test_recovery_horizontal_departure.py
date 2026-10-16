"""Recovery PI authority and measured, front-facing departure evidence."""
import math
import unittest

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget, EstimatorStatus, OpticalFlowRad, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import UInt8

import test_wall_horizontal_velocity as fixtures
from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion, rotate_vector
from px4_wall_ceiling_control.domain.horizontal_velocity import HorizontalVelocityController
from px4_wall_ceiling_control.domain.recovery_attitude_limits import attitude_from_axis_yaw
from px4_wall_ceiling_control.msg import SensorHealth, WallPerchStatus as W


class RecoveryHorizontalDepartureTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.HorizontalFlightTests()
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.n = self.f.n
        self.f.ready()
        self.n.state = W.STATE_RECOVER
        self.n.horizontal_takeover_yaw = 0.
        self.n._flight_ready = lambda now: True  # independent kill gate tests
        self.n.odom_twist_in_child_frame = False
        self.q = (0., 0., 0., 1.)
        self.n._begin_departure_clearance(rospy.Time.now())
        self.n._prepare_horizontal_velocity(rospy.Time.now())

    def tick(self, t, distance=.2, vx=0., vy=0., yaw=0., tilt=0., quality=80,
             owner=2, output=True, offset=0., mode='OFFBOARD', odom_stamp=None,
             front_stamp=None, flow=True):
        self.f.time = t
        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        axis = (math.sin(math.radians(tilt)), 0., math.cos(math.radians(tilt)))
        odom.pose.pose.orientation = Quaternion(*attitude_from_axis_yaw(axis, yaw))
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = vx, vy
        self.n._on_odom(odom)
        if odom_stamp is not None:
            self.n.odom.header.stamp = rospy.Time.from_sec(odom_stamp)
        if flow:
            msg = OpticalFlowRad(quality=quality, integration_time_us=20000, distance=1.)
            msg.header.stamp = now
            self.n._on_horizontal_flow(msg)
        self.n._on_horizontal_estimator(EstimatorStatus(
            attitude_status_flag=True, velocity_horiz_status_flag=True))
        self.n._on_sensor_health(SensorHealth(front_valid=True, ceiling_valid=False))
        if distance is not None:
            msg = Range(range=distance, min_range=.01, max_range=4.)
            msg.header.stamp = now if front_stamp is None else rospy.Time.from_sec(front_stamp)
            self.n._on_front_range(msg)
        self.n._on_state(State(connected=True, armed=True, mode=mode))
        self.n._on_horizontal_output_owner(UInt8(data=owner))
        if output:
            q = self.q if not offset else (0., math.sin(math.radians(offset)/2), 0., math.cos(math.radians(offset)/2))
            msg = AttitudeTarget(orientation=Quaternion(*q))
            msg.header.stamp = now
            self.n._on_horizontal_output(msg)
        self.q = self.n._horizontal_attitude(now, self.n.q_hover)
        return self.n.recovery_horizontal_controller

    def prime_integral(self):
        for t in (10.52, 10.54, 10.56, 10.58):
            self.tick(t, vx=.03)
        self.assertLess(self.n.recovery_horizontal_controller.integral[0], 0.)

    def test_recovery_has_independent_integral_and_clears_approach_history(self):
        self.n.horizontal_controller.integral = (.18, -.04)
        self.prime_integral()
        self.assertEqual(self.n.horizontal_controller.integral, (0., 0.))
        self.assertIsNot(self.n.horizontal_controller, self.n.recovery_horizontal_controller)
        self.assertLess(math.hypot(*self.n.recovery_horizontal_controller.integral), .01)
        self.n._reset_recovery_horizontal('new action', clear_departure=True)
        self.assertEqual(self.n.recovery_horizontal_controller.integral, (0., 0.))
        self.assertEqual(self.n.recovery_horizontal_controller.target, (0., 0.))
        self.assertFalse(self.n.departure_active)

    def test_wrong_owner_stale_output_and_bad_flow_clear_integral(self):
        for index, kwargs in enumerate(({'owner': 1}, {'output': False}, {'quality': 0})):
            self.n.recovery_horizontal_controller.integral = (.1, .1)
            if kwargs.get('output') is False:
                self.n.horizontal_output_at = rospy.Time.from_sec(9.)
            control = self.tick(10.52 + index*.02, **kwargs)
            self.assertEqual(control.integral, (0., 0.))
            self.assertFalse(control.integral_active)

    def test_mode_exit_clears_integral_and_cancels_departure(self):
        self.prime_integral()
        self.tick(10.60, mode='POSCTL')
        self.assertEqual(self.n.recovery_horizontal_controller.integral, (0., 0.))
        self.assertFalse(self.n.departure_active)

    def test_repeated_velocity_source_never_integrates_twice(self):
        self.prime_integral()
        before = self.n.recovery_horizontal_controller.integral
        self.tick(10.60, vx=.03, odom_stamp=10.58)
        self.assertEqual(self.n.recovery_horizontal_controller.integral, before)
        self.assertIn('新的速度样本', self.n.recovery_integral_reason)
        self.tick(10.62, vx=.03)
        self.assertLess(self.n.recovery_horizontal_controller.integral[0], before[0])

    def test_output_axis_mismatch_holds_without_catchup(self):
        self.prime_integral()
        before = self.n.recovery_horizontal_controller.integral
        self.tick(10.60, vx=.03, offset=4.)
        self.assertEqual(self.n.recovery_horizontal_controller.integral, before)
        self.assertFalse(self.n.recovery_horizontal_controller.integral_active)
        self.tick(10.62, vx=.03)
        self.assertEqual(self.n.recovery_horizontal_controller.integral, before)
        self.tick(10.64, vx=.03)
        self.assertLess(self.n.recovery_horizontal_controller.integral[0], before[0])

    def test_recovery_dt_budget_limits_ramp_and_integral_after_delayed_tick(self):
        self.prime_integral()
        control = self.n.recovery_horizontal_controller
        target, integral = control.target, control.integral
        self.tick(10.78, vx=.03)
        self.assertLessEqual(abs(control.target[0]-target[0]), .00500001)
        self.assertLessEqual(abs(control.integral[0]-integral[0]), .3*.08*.05+1.e-9)

    def test_retreat_direction_uses_frozen_takeover_heading(self):
        for yaw in (0., math.pi/2, math.pi, -1.3):
            self.n._reset_horizontal_gate('new heading')
            self.n.horizontal_takeover_yaw = yaw
            self.n.q_hover = attitude_from_axis_yaw((0., 0., 1.), yaw)
            self.q = self.n.q_hover
            self.n._begin_departure_clearance(rospy.Time.now())
            start = self.f.time + .02
            for i in range(23):
                self.tick(start + i*.05, yaw=yaw)
            target = self.n.recovery_horizontal_controller.target
            forward = (math.cos(yaw), math.sin(yaw))
            self.assertLess(sum(a*b for a,b in zip(target, forward)), -.049)
            self.assertAlmostEqual(target[0]*forward[1]-target[1]*forward[0], 0., places=7)
            axis = rotate_vector(self.q, (0., 0., 1.))
            self.assertLess(sum(a*b for a,b in zip(axis[:2], forward)), 0.)
            self.assertGreaterEqual(axis[2], math.cos(math.radians(5.))-1.e-9)

    def test_three_different_headings_keep_hover_attitude_and_departure_velocity(self):
        hover_yaw, takeover_yaw, departure_yaw = map(math.radians, (35., -80., 110.))
        self.n.q_hover = attitude_from_axis_yaw((0., 0., 1.), hover_yaw)
        self.n.horizontal_takeover_yaw = takeover_yaw
        self.n.departure_yaw = departure_yaw
        for i in range(23):
            self.tick(10.52+i*.05, yaw=departure_yaw)
        actual_yaw = euler_from_quaternion(self.q)[2]
        self.assertAlmostEqual(actual_yaw, hover_yaw, places=6)
        target = self.n.recovery_horizontal_controller.target
        direction = (math.cos(departure_yaw), math.sin(departure_yaw))
        self.assertLess(sum(a*b for a,b in zip(target, direction)), -.049)
        self.assertAlmostEqual(target[0]*direction[1]-target[1]*direction[0], 0., places=7)
        self.assertGreater(abs(actual_yaw-takeover_yaw), 1.)

    def test_normal_detach_and_exit_keep_hover_yaw_but_fault_path_is_unchanged(self):
        hover_yaw, takeover_yaw = math.radians(35.), math.radians(-80.)
        self.n.q_hover = attitude_from_axis_yaw((0., 0., 1.), hover_yaw)
        self.n.horizontal_takeover_yaw = takeover_yaw
        self.n.q_detach_start = (0., math.sin(math.pi/4), 0., math.cos(math.pi/4))
        self.n._detach_progress = lambda: 1.
        for stage, fault, expected in ((W.STATE_DETACH_ROTATE, False, hover_yaw),
                (W.STATE_RECOVER, False, hover_yaw), (W.STATE_EXIT, False, hover_yaw),
                (W.STATE_ABORT, False, takeover_yaw), (W.STATE_EXIT, True, takeover_yaw)):
            self.n.state = stage
            self.n.fault_recovery_completed = fault
            self.tick(self.f.time+.02, distance=None)
            self.assertTrue(self.n.horizontal_velocity_active)
            self.assertAlmostEqual(euler_from_quaternion(self.q)[2], expected, places=6)

    def test_approach_uses_takeover_yaw_for_both_attitude_and_forward_direction(self):
        hover_yaw, takeover_yaw = math.radians(35.), math.radians(-80.)
        self.n.q_hover = attitude_from_axis_yaw((0., 0., 1.), hover_yaw)
        self.n.horizontal_takeover_yaw = takeover_yaw
        self.n.departure_yaw = math.radians(110.)
        self.n.state = W.STATE_SLOW_APPROACH
        self.tick(10.52)
        self.assertAlmostEqual(euler_from_quaternion(self.q)[2], takeover_yaw, places=6)
        target = self.n.horizontal_controller.target
        direction = (math.cos(takeover_yaw), math.sin(takeover_yaw))
        self.assertGreater(sum(a*b for a,b in zip(target, direction)), 0.)
        self.assertAlmostEqual(target[0]*direction[1]-target[1]*direction[0], 0., places=7)

    def test_unknown_front_distance_does_not_retreat_or_complete(self):
        for t in (10.52, 10.54, 10.56):
            self.tick(t, distance=None)
        self.assertFalse(self.n.departure_feedback_valid)
        self.assertEqual(self.n.recovery_horizontal_controller.target, (0., 0.))
        self.assertEqual(self.n.recovery_horizontal_controller.integral, (0., 0.))
        self.assertFalse(self.n._departure_clearance_ready(rospy.Time.now()))
        self.assertFalse(self.n._horizontal_recovery_ready(rospy.Time.now()))

    def test_rotated_top_range_is_not_departure_evidence(self):
        self.n.top_filtered_m = 3.
        self.n.top_received_at = rospy.Time.now()
        self.n.ignore_range_loss_during_attachment = True
        self.tick(10.52, distance=.18)
        self.assertTrue(self.n.departure_feedback_valid)
        self.assertFalse(self.n._departure_clearance_ready(rospy.Time.now()))
        self.assertLess(self.n.recovery_horizontal_controller.target[0], 0.)

    def test_front_heading_mismatch_resets_clearance_evidence(self):
        for t in (10.52, 10.72, 10.92):
            self.tick(t, distance=.4)
        self.assertTrue(self.n.departure_clear_confirmed)
        self.tick(10.94, distance=.4, yaw=math.radians(11.))
        self.assertFalse(self.n.departure_feedback_valid)
        self.assertFalse(self.n.departure_clear_confirmed)
        self.assertIsNone(self.n.departure_level_at)

    def test_front_frame_must_follow_latest_return_to_level(self):
        self.tick(10.52, tilt=6., distance=.4)
        self.tick(10.54, tilt=0., distance=.4)
        self.assertFalse(self.n.departure_feedback_valid)  # same frame as level qualification
        self.tick(10.56, distance=.4)
        self.assertTrue(self.n.departure_feedback_valid)
        self.assertFalse(self.n.departure_clear_confirmed)

    def test_repeated_front_frame_does_not_accumulate_hold_time(self):
        self.tick(10.52, distance=.4)
        for t in (10.62, 10.72, 10.92):
            self.tick(t, distance=.4, front_stamp=10.52)
        self.assertEqual(self.n.departure_clear_samples, 1)
        self.assertFalse(self.n.departure_clear_confirmed)
        self.tick(10.94, distance=.4)
        self.assertTrue(self.n.departure_clear_confirmed)

    def test_clearance_then_stop_target_then_one_second_stability(self):
        for i in range(30):
            self.tick(10.52+i*.02)
        self.assertAlmostEqual(self.n.recovery_horizontal_controller.target[0], -.05)
        for i in range(18):
            self.tick(11.12+i*.02, distance=.4)
        self.assertTrue(self.n.departure_clear_confirmed)
        self.assertLess(self.n.recovery_horizontal_controller.target[0], 0.)
        self.assertFalse(self.n.horizontal_velocity_stable)
        for i in range(80):
            self.tick(11.48+i*.02, distance=.4)
        self.assertEqual(self.n.recovery_horizontal_controller.target, (0., 0.))
        self.assertTrue(self.n._horizontal_recovery_ready(rospy.Time.now()))
        self.tick(13.10, distance=.2)
        self.assertFalse(self.n.horizontal_velocity_stable)
        self.assertFalse(self.n._horizontal_recovery_ready(rospy.Time.now()))

    def test_detach_tail_needs_trajectory_and_actual_level(self):
        self.n.state = W.STATE_DETACH_ROTATE
        self.n.q_detach_start = (0., math.sin(math.pi/4), 0., math.cos(math.pi/4))
        self.n._detach_progress = lambda: 1.
        self.tick(10.52, tilt=6.)
        self.assertFalse(self.n._horizontal_phase_allowed(rospy.Time.now()))
        self.n._detach_progress = lambda: .5
        self.tick(10.54, tilt=0.)
        self.assertFalse(self.n._horizontal_phase_allowed(rospy.Time.now()))
        self.n._detach_progress = lambda: 1.
        self.tick(10.56, tilt=4.)
        self.assertTrue(self.n._horizontal_phase_allowed(rospy.Time.now()))
        self.assertEqual(self.n.recovery_horizontal_controller.target, (0., 0.))

    def test_abort_cancels_retreat_and_stabilizes_without_range(self):
        self.prime_integral()
        self.n._reset_recovery_horizontal('ABORT', clear_departure=True)
        self.n.state = W.STATE_ABORT
        self.n.front_range = self.n.front_received_at = None
        for i in range(60):
            self.tick(10.60+i*.02, distance=None)
        self.assertFalse(self.n.departure_active)
        self.assertEqual(self.n.recovery_horizontal_controller.target, (0., 0.))
        self.assertTrue(self.n._horizontal_recovery_ready(rospy.Time.now()))

    def test_recovery_baseline_does_not_jump_to_frozen_tilt(self):
        self.n.horizontal_equilibrium.axis = (math.sin(math.radians(4.)), 0., math.cos(math.radians(4.)))
        self.tick(10.52, distance=None)
        axis = self.n.recovery_baseline_axis
        self.assertLessEqual(math.degrees(math.acos(axis[2])), .200001)
        self.tick(10.54, distance=None)
        axis = self.n.recovery_baseline_axis
        self.assertLessEqual(math.degrees(math.acos(axis[2])), .400001)

    def test_status_identifies_recovery_target_and_integral(self):
        self.prime_integral()
        status = W()
        status.header.stamp = rospy.Time.now()
        self.n._fill_horizontal_status(status)
        self.assertLess(status.horizontal_forward_target_mps, 0.)
        self.assertLess(status.recovery_horizontal_integral_x, 0.)
        self.assertEqual(status.horizontal_integral_x_enu_mps2, 0.)
        self.assertAlmostEqual(status.departure_clearance_distance_m, .2, places=6)
        self.assertFalse(status.departure_clearance_ready)
        self.assertTrue(status.departure_clearance_reason)


class RecoveryBiasModelTests(unittest.TestCase):
    def test_zero_velocity_bias_rejection_bounded_in_simplified_plant(self):
        # This mathematical plant checks the controller; it is no flight claim.
        for bias_deg in (-1., -.5, .5, 1.):
            control = HorizontalVelocityController()
            speed = 0.
            disturbance = 9.80665 * math.tan(math.radians(bias_deg))
            for _ in range(750):
                q = control.attitude((0., 0.), (speed, 0.), 0., .02,
                    integrate=True, integral_dt=.02, max_tilt=math.radians(5.))
                speed += (control.acceleration[0]-disturbance)*.02
                self.assertLessEqual(math.hypot(*control.integral), .20000001)
                self.assertLessEqual(math.hypot(*control.acceleration), .30000001)
                self.assertGreaterEqual(rotate_vector(q, (0., 0., 1.))[2], math.cos(math.radians(5.))-1.e-9)
            self.assertLess(abs(speed), .005)


if __name__ == '__main__':
    unittest.main()
