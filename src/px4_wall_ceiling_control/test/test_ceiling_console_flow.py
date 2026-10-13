#!/usr/bin/env python3
import unittest
from unittest import mock
import rospy
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus as S, OperatorCommand, SensorHealth
from test_attachment_safety import CeilingDecision
from test_flight_handover import Manager, Terminal
from std_msgs.msg import String

class CeilingConsoleFlowTests(unittest.TestCase):
    def setUp(self):
        self.clock = 10.0
        self.time = mock.patch.object(rospy.Time, "now", side_effect=lambda: rospy.Time.from_sec(self.clock));self.time.start();self.addCleanup(self.time.stop)
        overrides={"~console_contact_profile":True,"~activation_distance_m":1.0,"~activation_hold_s":.3,"~wall_contact_distance_m":.056,"~wall_contact_hold_s":1.,"~surface_hold_thrust":.5,"~hover_thrust":.45,"~detach_drop_m":.2,"~detach_safe_distance_m":.1,"~require_mechanism_ack":False,"~detach_timeout_s":10.}
        with mock.patch.object(rospy,"get_param",side_effect=lambda k,d=None:overrides.get(k,d)),mock.patch.object(rospy,"Publisher"),mock.patch.object(rospy,"Subscriber"),mock.patch.object(rospy,"Timer"):
            self.n=CeilingDecision()
        self.n._flight_ready=self.n._arm_requested=self.n._mode_requested=lambda *a:True
        self.odom(2.)
    def odom(self,z,vz=0):
        m=Odometry();m.pose.pose.position.z=z;m.pose.pose.orientation.w=1;m.twist.twist.linear.z=vz
        self.n._on_odom(m)
    def sample(self,t,d):
        self.clock=t;self.n._on_range(Range(range=d,min_range=.03,max_range=40.))
    def test_one_metre_requires_continuous_samples_for_point_three_seconds(self):
        for t in [10,10.1,10.2]:self.sample(t,1.)
        self.assertFalse(self.n._activation_ready(rospy.Time.now()))
        self.sample(10.3,1.);self.assertTrue(self.n._activation_ready(rospy.Time.now()))
        self.sample(10.4,1.01);self.assertFalse(self.n._activation_ready(rospy.Time.now()))
    def test_invalid_and_stale_data_restart_entry_confirmation(self):
        for t in [10,10.1,10.2,10.3]:self.sample(t,.8)
        self.sample(10.4,255);self.assertFalse(self.n._activation_ready(rospy.Time.now()))
        self.sample(10.5,.8);self.assertFalse(self.n._activation_ready(rospy.Time.now()))
        self.sample(12,.8);self.assertFalse(self.n._activation_ready(rospy.Time.now()))
    def test_health_invalid_resets_gate_between_valid_ranges(self):
        for t in [10,10.1,10.2,10.3]:self.sample(t,.8)
        self.n._on_sensor_health(SensorHealth(ceiling_valid=False))
        self.assertFalse(self.n._activation_ready(rospy.Time.now()))
        self.sample(10.4,.8);self.assertFalse(self.n._activation_ready(rospy.Time.now()))
    def test_click_restarts_gate_even_if_top_was_already_near(self):
        for t in [10,10.1,10.2,10.3]:self.sample(t,.8)
        op=OperatorCommand(mode=OperatorCommand.MODE_CEILING,enable_control=True)
        self.n._on_operator(op);self.assertFalse(self.n._activation_ready(rospy.Time.now()))
    def test_wall_contact_threshold_and_one_second_hold(self):
        self.n.state=S.STATE_ATTACH_CONTROL
        for t in [10,10.1,10.2,10.3,10.4,10.5,10.6,10.7,10.8,10.9]:self.sample(t,.056)
        self.assertFalse(self.n._attach_confirmed(rospy.Time.now()))
        self.sample(11.,.056);self.assertTrue(self.n._attach_confirmed(rospy.Time.now()))
        self.sample(11.1,.06);self.assertFalse(self.n._attach_confirmed(rospy.Time.now()))
    def test_hold_uses_exact_point_five_thrust(self):
        self.assertAlmostEqual(self.n._surface_hold_thrust(),.5)
        self.n.state=S.STATE_SURFACE_HOLD;self.sample(10,.05)
        status=self.n._build_status(rospy.Time.now(),.02)
        candidate=self.n._build_control_candidate(status)
        self.assertAlmostEqual(candidate.thrust_normalized,.5)
    def test_detach_captures_reference_and_requires_twenty_cm(self):
        self.n.state=S.STATE_SURFACE_HOLD;self.sample(10,.05)
        self.n._enter_state(S.STATE_DETACH,rospy.Time.now(),"detach")
        self.assertAlmostEqual(self.n.detach_start_height_m,2.)
        self.sample(10.1,.11);self.odom(1.9)
        self.assertFalse(self.n._detach_height_ready(rospy.Time.now()))
        for t in [10.2,10.3,10.4]:
            self.clock=t;self.odom(1.8);ready=self.n._detach_height_ready(rospy.Time.now())
        self.assertTrue(ready)
    def test_stale_height_and_fast_descent_block_handover(self):
        self.n.state=S.STATE_DETACH;self.n.detach_start_height_m=2.
        for t in [10,10.1,10.2]:
            self.clock=t;self.odom(1.8,-.5);self.assertFalse(self.n._detach_height_ready(rospy.Time.now()))
        self.clock=12.;self.assertFalse(self.n._detach_height_ready(rospy.Time.now()))
    def test_repeated_timer_without_new_odom_does_not_confirm_descent(self):
        self.n.state=S.STATE_DETACH;self.n.detach_start_height_m=2.;self.odom(1.8)
        self.assertFalse(self.n._detach_height_ready(rospy.Time.now()))
        self.clock=10.3;self.assertFalse(self.n._detach_height_ready(rospy.Time.now()))
    def test_height_controller_reduces_thrust_then_brakes_descent(self):
        self.n.detach_start_height_m=2.
        self.odom(2.,0);self.assertLess(-self.n._detach_thrust(.02),.45)
        self.odom(1.8,-.5);self.assertGreater(-self.n._detach_thrust(.02),.45)
        self.assertLessEqual(-self.n._detach_thrust(.02),.53)
    def test_detach_state_waits_for_height_then_recovers(self):
        self.n.state=S.STATE_SURFACE_HOLD;self.sample(10,.05)
        self.n._enter_state(S.STATE_DETACH,rospy.Time.now(),"detach")
        for t in [10.1,10.2,10.3,10.4]:
            self.sample(t,.26);self.odom(1.9);self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_DETACH)
        for t in [10.5,10.6,10.7,10.8]:
            self.sample(t,.26);self.odom(1.8);self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_RECOVERY_HOVER)
        self.n.recovery_hover_time_s=.2
        self.clock=11.01;self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_NORMAL_FLIGHT)
    def test_height_missing_and_timeout_latch_fault(self):
        for failure in ["height", "timeout"]:
            self.n.state=S.STATE_DETACH;self.n.state_entered_at=rospy.Time.from_sec(10)
            self.n.detach_start_height_m=2.;self.sample(10,.26)
            if failure=="height":self.odom(float("nan"))
            else:
                self.clock=21.;self.sample(21.,.26);self.odom(2.)
            self.n._update_state_machine(rospy.Time.now())
            self.assertEqual(self.n.state,S.STATE_FAULT)
            self.assertTrue(self.n.detach_failed)
    def test_rpc_does_not_request_offboard_after_gate_is_lost(self):
        m=Manager.__new__(Manager);m.phase=Manager.ENTERING;m.selected_mode=OperatorCommand.MODE_CEILING
        m._requested_mode=lambda *a:OperatorCommand.MODE_CEILING
        m._preflight_ready=m._stream_ready=lambda *a:True;m._gated_wall_selected=lambda:False
        m._ceiling_takeover_ready=lambda *a:False;m.set_mode=mock.Mock()
        with mock.patch.object(rospy,"wait_for_service"),mock.patch.dict(Manager._send_mode_request.__globals__,kill_clear=lambda *a:True):
            m._send_mode_request("OFFBOARD")
        m.set_mode.assert_not_called()
    def test_front_mask_persists_after_release_and_blocks_wall(self):
        n=Terminal.__new__(Terminal);n.mode=OperatorCommand.MODE_STANDBY;n.detaching=False;n.was_offboard=False;n.flight_mode="ALTCTL";n.front_sensor_masked=False
        with mock.patch.object(rospy,"set_param") as write:
            n._on_command(String(data="ceiling"))
            write.assert_called_with("/attachment/front_sensor_masked",True)
        self.assertTrue(n.front_sensor_masked)
        n._release_intent();n._on_command(String(data="wall"))
        self.assertTrue(n.front_sensor_masked);self.assertEqual(n.mode,OperatorCommand.MODE_STANDBY)
    def test_operator_restart_clears_front_mask(self):
        with mock.patch.object(rospy,"get_param",side_effect=lambda k,d=None:d),mock.patch.object(rospy,"set_param") as write,mock.patch.object(rospy,"Publisher"),mock.patch.object(rospy,"Subscriber"),mock.patch.object(rospy,"Timer"):
            n=Terminal()
        self.assertFalse(n.front_sensor_masked);write.assert_called_with("/attachment/front_sensor_masked",False)
    def test_manager_requires_fresh_activation_confirmation(self):
        m=Manager.__new__(Manager);m.ceiling_distance_gate_enabled=True;m.selected_mode=OperatorCommand.MODE_CEILING
        m.status_timeout_s=.5;m.ceiling_at=rospy.Time.now();m.ceiling=S(activation_ready=False)
        self.assertFalse(m._ceiling_takeover_ready(rospy.Time.now()))
        m.ceiling.activation_ready=True;self.assertTrue(m._ceiling_takeover_ready(rospy.Time.now()))
        self.clock=11.;self.assertFalse(m._ceiling_takeover_ready(rospy.Time.now()))

if __name__=="__main__":unittest.main()
