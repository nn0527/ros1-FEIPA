"""Late OFFBOARD after cancellation must stabilize and permit a safe POSCTL return."""
import math
import unittest
from unittest import mock
import rospy
from mavros_msgs.msg import State, EstimatorStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from test_flight_handover import Manager, OutputMux
from px4_wall_ceiling_control.msg import OperatorCommand
from px4_wall_ceiling_control.domain.recovery_attitude_limits import axis_yaw, attitude_from_axis_yaw

class CancelVerticalTests(unittest.TestCase):
    def node(self):
        n=OutputMux.__new__(OutputMux)
        n.owner=0;n.flight_state=State(mode='OFFBOARD',connected=True,armed=True)
        n.manager_phase='CANCELING_ENTRY';n.manager_phase_at=rospy.Time.from_sec(10)
        n.odom=Odometry();n.odom.pose.pose.orientation.w=1.;n.odom.pose.pose.position.z=2.078
        n.odom_at=rospy.Time.from_sec(10);n.odom_timeout_s=.5;n.odom_twist_in_child_frame=False
        n.handover_start_thrust=.5063993335;n.min_thrust=.1;n.max_thrust=.85
        n.max_velocity_z_mps=.3;n.local_frame_id='map'
        n._assisted_attitude=lambda now,q,source:q
        return n
    def target(self,n,t=10):
        now=rospy.Time.from_sec(t);n.odom_at=now
        with mock.patch.object(rospy,'logwarn'):
            return n._prestream_attitude_target(now)
    def test_normal_start_captures_once_then_brakes_upward_motion(self):
        n=self.node();a=self.target(n)
        self.assertAlmostEqual(a.thrust,.5063993335)
        n.odom.pose.pose.position.z+=.4;n.odom.twist.twist.linear.z=.3
        b=self.target(n,10.02)
        self.assertLess(b.thrust,a.thrust)
        self.assertAlmostEqual(n.cancel_vertical_telemetry['target_vertical_speed_mps'],-.15)
        self.assertAlmostEqual(n.cancel_vertical_telemetry['height_reference_m'],2.078)
    def test_all_positive_correction_including_descent_braking_is_capped(self):
        n=self.node();self.target(n)
        n.odom.pose.pose.position.z-=3.;n.odom.twist.twist.linear.z=-2.
        b=self.target(n,10.02)
        self.assertAlmostEqual(n.cancel_vertical_telemetry['correction'],.08)
        self.assertAlmostEqual(n.cancel_vertical_telemetry['target_vertical_speed_mps'],.1)
        self.assertLessEqual(b.thrust,.6)
    def test_final_cap_survives_high_capture_and_rc_tilt(self):
        n=self.node();n.handover_start_thrust=.8
        q=attitude_from_axis_yaw((math.sin(.4),0,math.cos(.4)),1.)
        n._assisted_attitude=lambda *args:Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
        b=self.target(n);axis,_=axis_yaw(tuple(getattr(b.orientation,k) for k in 'xyzw'))
        self.assertLessEqual(b.thrust,.6);self.assertLessEqual(math.acos(axis[2]),math.radians(5)+1e-9)
    def test_tilt_and_yaw_slew_include_rc_correction(self):
        n=self.node();a=self.target(n)
        q=attitude_from_axis_yaw((math.sin(.3),0,math.cos(.3)),.6)
        n._assisted_attitude=lambda *args:Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
        b=self.target(n,10.02);axis,yaw=axis_yaw(tuple(getattr(b.orientation,k) for k in 'xyzw'))
        self.assertLessEqual(math.acos(axis[2]),math.radians(10)*.02+1e-7)
        self.assertLessEqual(abs(yaw),math.radians(20)*.02+1e-7)
    def test_stale_or_replaced_phase_cannot_reenable_fixed_thrust(self):
        n=self.node();self.target(n)
        n.manager_phase='IDLE';n.odom.twist.twist.linear.z=.3
        b=self.target(n,12)
        self.assertLess(b.thrust,n.handover_start_thrust)
        self.assertTrue(n.cancel_vertical_latched)
    def test_invalid_or_stale_odometry_or_missing_capture_stops_output(self):
        for fault in ('stale','nan','capture'):
            n=self.node()
            if fault=='stale':n.odom_at=rospy.Time.from_sec(8)
            elif fault=='nan':n.odom.twist.twist.linear.z=math.nan
            else:n.handover_start_thrust=None
            self.assertIsNone(n._prestream_attitude_target(rospy.Time.from_sec(10)),fault)
    def test_non_cancel_phase_never_activates_feedback(self):
        for phase in ('PRESTREAM','ENTERING','ACTIVE'):
            n=self.node();n.manager_phase=phase
            self.assertFalse(n._cancel_vertical_active(rospy.Time.from_sec(10)))
    def test_owner_or_pilot_mode_ends_cancel_latch(self):
        for mode,owner in (('POSCTL',0),('ALTCTL',0),('OFFBOARD',2),('OFFBOARD',1)):
            n=self.node();self.target(n)
            n.flight_state.mode=mode;n.owner=owner
            self.assertFalse(n._cancel_vertical_active(rospy.Time.from_sec(10)))
            self.assertIsNone(n.cancel_vertical_reference)
    def test_disarm_clears_reference_and_repeated_attempt_recaptures(self):
        n=self.node();self.target(n);n.flight_state.armed=False
        self.assertFalse(n._cancel_vertical_active(rospy.Time.from_sec(10)))
        n.flight_state.armed=True;n.odom.pose.pose.position.z=4
        self.target(n,10.02)
        self.assertEqual(n.cancel_vertical_reference[0],4)
    def test_callback_records_phase_receipt(self):
        n=self.node()
        with mock.patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(20)):
            n._on_manager_phase(String(data='CANCELING_ENTRY'))
        self.assertEqual(n.manager_phase_at.to_sec(),20)

class CancelReturnTests(unittest.TestCase):
    def node(self):
        n=Manager.__new__(Manager);now=rospy.Time.from_sec(10)
        n.require_rc_center=False;n.rc_roll_channel_index=n.rc_pitch_channel_index=n.rc_throttle_channel_index=-1
        n.rc_channels=[1500]*8;n.rc_at=now;n.rc_timeout_s=.2
        n.owner=0;n.owner_at=n.output_at=n.flight_at=n.estimator_at=n.odom_at=now
        n.output_mode=2;n.output_timeout_s=.3;n.state_timeout_s=1.5;n.odom_timeout_s=.5;n.estimator_timeout_s=1.5
        n.flight=State(mode='OFFBOARD',connected=True,armed=True)
        n.estimator=EstimatorStatus(attitude_status_flag=True,velocity_vert_status_flag=True,
            velocity_horiz_status_flag=True,pos_horiz_rel_status_flag=True)
        n.odom=Odometry();n.odom.pose.pose.orientation.w=1.
        n.odom_twist_in_child_frame=False;n.handover_max_vertical_speed_mps=.2
        n.wall_pilot_mode='POSCTL';n.selected_mode=OperatorCommand.MODE_WALL
        n.cancel_entry_stable_since=None;n._request_mode=mock.Mock()
        return n,now
    def test_production_minus_one_indices_no_longer_block_safe_return(self):
        n,now=self.node();self.assertTrue(n._entry_cancel_handover_ready(now))
        n._tick_entry_cancel(now,100,True,'OFFBOARD');n._request_mode.assert_not_called()
        n._tick_entry_cancel(now,100.31,True,'OFFBOARD');n._request_mode.assert_called_once_with('POSCTL')
    def test_missing_stale_short_and_invalid_pwm_still_block(self):
        for fault in ('missing','stale','short','pwm','nan'):
            n,now=self.node()
            if fault=='missing':n.rc_channels=None
            elif fault=='stale':n.rc_at=rospy.Time.from_sec(9)
            elif fault=='short':n.rc_channels=[1500]*3
            elif fault=='pwm':n.rc_channels[3]=0
            else:n.rc_channels[0]=math.nan
            self.assertFalse(n._entry_cancel_handover_ready(now),fault)
    def test_optional_centering_still_requires_distinct_configured_channels(self):
        n,now=self.node();n.require_rc_center=True
        self.assertFalse(n._rc_fresh(now))
        n.rc_roll_channel_index=0;n.rc_pitch_channel_index=1;n.rc_throttle_channel_index=2
        self.assertTrue(n._rc_fresh(now))
        n.rc_throttle_channel_index=1;self.assertFalse(n._rc_fresh(now))
    def test_posctl_estimator_tilt_and_vertical_motion_gates_remain(self):
        for fault in ('estimate','tilt','vertical'):
            n,now=self.node()
            if fault=='estimate':n.estimator.pos_horiz_rel_status_flag=False
            elif fault=='tilt':n.odom.pose.pose.orientation.y=.1
            else:n.odom.twist.twist.linear.z=.4
            self.assertFalse(n._entry_cancel_handover_ready(now),fault)
    def test_pilot_third_mode_never_overridden(self):
        n,now=self.node();n._tick_entry_cancel(now,100,True,'ALTCTL')
        n._request_mode.assert_not_called()

if __name__=='__main__':unittest.main()
