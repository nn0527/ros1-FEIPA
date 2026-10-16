#!/usr/bin/env python3
import math
import unittest
from unittest import mock

import rospy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import OpticalFlowRad, EstimatorStatus, State
from std_msgs.msg import String
from px4_wall_ceiling_control.domain.horizontal_velocity import HorizontalVelocityController
from px4_wall_ceiling_control.domain.geometry import rotate_vector
from px4_wall_ceiling_control.msg import WallPerchStatus as W, OperatorCommand
from test_attachment_safety import WallDecision
from test_flight_handover import Manager, Terminal, OutputMux


class HorizontalMathTests(unittest.TestCase):
    def test_forward_and_left_drift_braking_directions(self):
        control=HorizontalVelocityController()
        q=control.attitude((.05,0),(0,.2),0,.02)
        axis=rotate_vector(q,(0,0,1))
        self.assertGreater(axis[0],0)
        self.assertLess(axis[1],0)
        self.assertLessEqual(math.hypot(*control.acceleration),.3+1e-9)

    def test_target_ramp_vector_cap_and_reset(self):
        control=HorizontalVelocityController()
        control.attitude((.05,0),(0,0),0,.02)
        self.assertAlmostEqual(control.target[0],.002)
        for _ in range(100):control.attitude((.05,0),(-5,5),0,.02)
        self.assertAlmostEqual(control.target[0],.05)
        self.assertLessEqual(math.hypot(*control.acceleration),.3+1e-9)
        control.reset();self.assertEqual(control.target,(0.,0.))
        with self.assertRaises(ValueError):control.attitude((0,0),(float('nan'),0),0,.02)


class HorizontalFlightTests(unittest.TestCase):
    def setUp(self):
        self.time=10.
        timer=mock.patch.object(rospy.Time,'now',side_effect=lambda:rospy.Time.from_sec(self.time))
        timer.start();self.addCleanup(timer.stop)
        params={'~horizontal_velocity_enabled':True,'~auto_approach':True}
        with mock.patch.object(rospy,'get_param',side_effect=lambda k,d=None:params.get(k,d)),mock.patch.object(rospy,'Publisher'),mock.patch.object(rospy,'Subscriber'),mock.patch.object(rospy,'Timer'):
            self.n=WallDecision()
        # These tests isolate flow/phase behavior after a valid equilibrium capture.
        self.n.horizontal_equilibrium.ready=self.n.horizontal_equilibrium.frozen=True
        self.n.state=W.STATE_SLOW_APPROACH
        self.n.q_hover=(0.,0.,0.,1.)
        self.n._auto_approach_ranges_ready=lambda now:True
        self.n.control_dt=.02
        self.feed(10.)

    def feed(self,t,vx=0.,vy=0.,tilt=0.,quality=80,const=False,rate=0.,flow=True,estimator=True):
        self.time=t;now=rospy.Time.now()
        odom=Odometry();odom.header.stamp=now;odom.pose.pose.orientation.y=math.sin(math.radians(tilt)/2);odom.pose.pose.orientation.w=math.cos(math.radians(tilt)/2)
        odom.twist.twist.linear.x=vx;odom.twist.twist.linear.y=vy;odom.twist.twist.angular.y=rate
        self.n._on_odom(odom)
        if flow:
            message=OpticalFlowRad(quality=quality,integration_time_us=20000,distance=1.)
            message.header.stamp=now;self.n._on_horizontal_flow(message)
        if estimator:self.n._on_horizontal_estimator(EstimatorStatus(attitude_status_flag=True,velocity_horiz_status_flag=True,const_pos_mode_status_flag=const))
        self.n._prepare_horizontal_velocity(now)

    def ready(self,start=10.):
        for t in (start,start+.25,start+.5):
            if self.n.horizontal_flow_at == rospy.Time.from_sec(t):
                self.n._prepare_horizontal_velocity(rospy.Time.now())
            else:self.feed(t)
        self.assertTrue(self.n.horizontal_velocity_ready)

    def test_new_continuous_samples_required_then_actual_velocity_feedback(self):
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.ready()
        q=self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
        self.assertTrue(self.n.horizontal_velocity_active)
        self.assertGreater(rotate_vector(q,(0,0,1))[0],0)
        self.feed(10.75,vx=.2,vy=.2)
        q=self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
        axis=rotate_vector(q,(0,0,1));self.assertLess(axis[0],0);self.assertLess(axis[1],0)

    def test_bad_flow_estimate_and_stale_frames_remove_forward_command(self):
        for index,fault in enumerate(('quality','const','stale')):
            start=20.+index*3
            self.n._reset_horizontal_gate('test');self.ready(start)
            self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
            self.assertTrue(self.n.horizontal_velocity_active)
            self.feed(start+1.2,quality=0 if fault=='quality' else 80,const=fault=='const',flow=fault!='stale')
            q=self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
            self.assertFalse(self.n.horizontal_velocity_ready)
            self.assertFalse(self.n.horizontal_velocity_active)
            self.assertEqual(q,self.n.q_hover)
            self.assertEqual(self.n.horizontal_controller.target,(0.,0.))

    def test_flip_capture_hold_ignore_velocity_and_clear_history(self):
        self.ready()
        self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
        for stage in (W.STATE_FLIP_TO_WALL,W.STATE_WALL_CAPTURE,W.STATE_WALL_HOLD):
            self.n.state=stage
            original=(0.,math.sin(.7),0.,math.cos(.7))
            self.assertEqual(self.n._horizontal_attitude(rospy.Time.now(),original),original)
            self.assertFalse(self.n.horizontal_velocity_active)
            self.assertFalse(self.n.horizontal_velocity_ready)
            self.assertEqual(self.n.horizontal_controller.target,(0.,0.))

    def test_rotation_tail_requires_measured_level_and_new_samples(self):
        self.ready();self.n.state=W.STATE_FLIP_TO_WALL
        self.n._prepare_horizontal_velocity(rospy.Time.now())
        self.n.state=W.STATE_DETACH_ROTATE
        self.n.q_detach_start=(0.,math.sin(math.pi/4),0.,math.cos(math.pi/4))
        self.n.detach_time_s=3.;self.n.state_entered_at=rospy.Time.from_sec(10.)
        self.n.detach_elapsed_s=2.
        self.feed(12.,tilt=20.)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.n.detach_elapsed_s=2.8
        self.feed(12.8,tilt=9.)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.n.detach_elapsed_s=3.
        self.feed(13.05,tilt=4.);self.feed(13.3,tilt=3.);self.feed(13.55,tilt=3.)
        self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
        self.assertTrue(self.n.horizontal_velocity_active)
        self.assertEqual(self.n.horizontal_controller.target,(0.,0.))

    def test_recovery_needs_stable_measured_speed_attitude_and_new_odometry(self):
        self.n.state=W.STATE_RECOVER;self.ready()
        self.feed(10.75,vx=.25)
        self.assertFalse(self.n._horizontal_recovery_ready(rospy.Time.now()))
        for t in (11.,11.25,11.5,11.75):self.feed(t)
        self.assertFalse(self.n.horizontal_velocity_stable)
        self.feed(12.)
        self.assertTrue(self.n.horizontal_velocity_stable)
        self.feed(12.25,rate=.3)
        self.assertFalse(self.n.horizontal_velocity_stable)
        for t in (12.5,12.75,13.,13.25,13.5):self.feed(t)
        self.assertTrue(self.n.horizontal_velocity_stable)
        self.time=14.2
        self.assertFalse(self.n._horizontal_recovery_ready(rospy.Time.now()))

    def test_world_velocity_rotates_body_twist_once(self):
        self.ready()
        odom=self.n.odom;odom.pose.pose.orientation.z=math.sin(math.pi/4);odom.pose.pose.orientation.w=math.cos(math.pi/4)
        odom.twist.twist.linear.x=.3
        self.n._prepare_horizontal_velocity(rospy.Time.now())
        self.assertAlmostEqual(self.n.horizontal_measured[0],0.,places=6)
        self.assertAlmostEqual(self.n.horizontal_measured[1],.3,places=6)

    def test_received_old_source_timestamp_cannot_enable_loop(self):
        self.ready();self.feed(11.)
        self.n.horizontal_flow.header.stamp=rospy.Time.from_sec(8.)
        self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
        self.assertFalse(self.n.horizontal_velocity_active)
        self.assertIn('源时间戳',self.n.horizontal_velocity_reason)

    def test_pose_gate_rejects_fast_rotation_even_near_horizontal(self):
        self.ready();self.feed(11.,tilt=3.,rate=1.)
        self.assertFalse(self.n.horizontal_velocity_ready)


class PilotModeTests(unittest.TestCase):
    def setUp(self):
        timer=mock.patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(10.))
        timer.start();self.addCleanup(timer.stop)

    def test_wall_posctl_ceiling_altctl_and_rpc_does_not_accept_old_return_mode(self):
        manager=Manager.__new__(Manager);manager.wall_pilot_mode='POSCTL';manager.selected_mode=OperatorCommand.MODE_WALL
        self.assertEqual(manager._pilot_mode(),'POSCTL')
        self.assertEqual(manager._pilot_mode(OperatorCommand.MODE_CEILING),'ALTCTL')
        manager.phase=Manager.RETURNING;manager.set_mode=mock.Mock();manager.request_in_flight=True
        manager._handover_ready=lambda now:True
        with mock.patch.object(rospy,'wait_for_service'),mock.patch.dict(Manager._send_mode_request.__globals__,kill_clear=lambda *args:True):
            manager._send_mode_request('POSCTL')
        self.assertEqual(manager.set_mode.call_args.args[0].custom_mode,'POSCTL')

    def test_posctl_requires_position_and_stability_and_rejects_stale_feedback(self):
        manager=Manager.__new__(Manager);manager.wall_pilot_mode='POSCTL';manager.selected_mode=OperatorCommand.MODE_WALL
        manager.wall_horizontal_velocity_enabled=True
        manager.wall=W(horizontal_velocity_enabled=True,horizontal_velocity_ready=True,horizontal_velocity_stable=False)
        manager.wall_at=manager.estimator_at=rospy.Time.now();manager.status_timeout_s=.5;manager.estimator_timeout_s=1.
        manager.estimator=EstimatorStatus(velocity_horiz_status_flag=True,pos_horiz_rel_status_flag=True)
        self.assertTrue(manager._wall_horizontal_ready(rospy.Time.now()))
        self.assertFalse(manager._wall_horizontal_ready(rospy.Time.now(),stable=True))
        manager.wall.horizontal_velocity_stable=True
        self.assertTrue(manager._wall_horizontal_ready(rospy.Time.now(),stable=True))
        manager.estimator.pos_horiz_rel_status_flag=False
        self.assertFalse(manager._wall_horizontal_ready(rospy.Time.now(),stable=True))
        manager.estimator.pos_horiz_rel_status_flag=True;manager.wall_at=rospy.Time.from_sec(8.)
        self.assertFalse(manager._wall_horizontal_ready(rospy.Time.now(),stable=True))

    def test_terminal_wall_posctl_return_clears_request_but_ceiling_keeps_altctl(self):
        with mock.patch.object(rospy,'get_param',side_effect=lambda k,d=None:'POSCTL' if k=='/flight_mode_manager_node/wall_pilot_mode' else d),mock.patch.object(rospy,'Publisher'),mock.patch.object(rospy,'Subscriber'),mock.patch.object(rospy,'Timer'),mock.patch.object(rospy,'set_param'):
            terminal=Terminal();terminal._on_state(State(connected=True,armed=True,mode='POSCTL'))
            terminal._apply_command(String(data='wall'))
            self.assertEqual(terminal.mode,OperatorCommand.MODE_WALL)
            terminal._on_state(State(connected=True,armed=True,mode='OFFBOARD'))
            terminal._on_state(State(connected=True,armed=True,mode='POSCTL'))
            self.assertEqual(terminal.mode,OperatorCommand.MODE_STANDBY)
            terminal._apply_command(String(data='ceiling'))
            self.assertEqual(terminal.mode,OperatorCommand.MODE_STANDBY)


if __name__=='__main__':unittest.main()
