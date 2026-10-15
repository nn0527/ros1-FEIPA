"""Bumpless approach-to-flip support, bounded feedback and unchanged protected phases."""
import math
import unittest
from unittest import mock
import rospy
from mavros_msgs.msg import AttitudeTarget, State
from px4_wall_ceiling_control.msg import WallPerchStatus as W, AttachmentControlCandidate as C
from test_attachment_safety import WallDecision

class FlipVerticalSupportTests(unittest.TestCase):
    def node(self, thrust=.518, base=.518, z=2.4, vz=0., tilt=0.):
        n=WallDecision.__new__(WallDecision)
        n.state=W.STATE_SLOW_APPROACH;n.auto_approach=True;n.flip_vertical_support_enabled=True
        n.flip_thrust_follow_tilt=True;n.flip_support_fade_start_rad=math.radians(20)
        n.flip_support_fade_end_rad=math.radians(60);n.flip_thrust_slew_per_s=.5
        n.wall_angle_rad=math.pi/2;n.hover_thrust=.48;n.flip_thrust_multiplier=1.35;n.max_thrust=.85;n.recovery_max_thrust=.60
        n.recovery_max_up_speed_mps=.10;n.recovery_max_down_speed_mps=.15
        n.recovery_height_to_speed_kp=.5;n.recovery_velocity_kp=.30;n.recovery_positive_compensation_max=.08
        n.approach_use_takeover_thrust=True;n.approach_base_vertical_thrust=base;n.approach_altitude_m=2.4
        n.z=z;n.vz=vz;n.tilt=tilt;n.feedback_valid=True;n.odom_valid=True
        n._height_feedback=lambda:(n.z,n.vz) if n.feedback_valid else None
        n._current_quaternion=lambda:(0.,math.sin(math.radians(n.tilt)/2),0.,math.cos(math.radians(n.tilt)/2))
        n._odom_fresh=lambda now:n.odom_valid
        n._top_fresh=lambda now:False;n.horizontal_velocity_enabled=False
        n._horizontal_attitude=lambda now,q:q
        n.horizontal_output=AttitudeTarget(thrust=thrust);n.horizontal_output.orientation.w=1.;n.horizontal_output.header.stamp=rospy.Time.from_sec(9.99)
        n.horizontal_output_at=n.horizontal_output_owner_at=rospy.Time.from_sec(9.99)
        n.horizontal_output_owner=C.SOURCE_WALL
        n.q_hover=(0.,0.,0.,1.);n.q_wall=(0.,math.sin(math.pi/4),0.,math.cos(math.pi/4))
        n.flip_time_s=2.;n.fault_count=0;n.state_entered_at=rospy.Time.from_sec(9.)
        return n
    def start(self,n):
        with mock.patch.object(rospy,'loginfo'):
            n._enter_state(W.STATE_FLIP_TO_WALL,rospy.Time.from_sec(10),'test')
        self.assertEqual(n.state,W.STATE_FLIP_TO_WALL)
    def step(self,n,t):return n._flip_thrust(rospy.Time.from_sec(t))
    def test_first_flip_candidate_matches_actual_sent_approach_not_old_hover(self):
        n=self.node(thrust=.527)
        self.start(n)
        target=n._candidate(rospy.Time.from_sec(10))
        self.assertAlmostEqual(target[2],.527)
        self.assertGreater(target[2],n.hover_thrust)
        self.assertEqual(n.flip_altitude_m,2.4)
        self.assertAlmostEqual(n.flip_base_vertical_thrust,.518)
    def test_baseline_and_existing_correction_are_not_counted_twice(self):
        n=self.node(thrust=.548,vz=-.1)
        self.start(n);self.step(n,10)
        for i in range(1,21):u=self.step(n,10+i*.02)
        self.assertAlmostEqual(u,.548)
        self.assertAlmostEqual(n.vertical_correction,.03)
    def test_near_level_height_and_velocity_feedback_brake_descent(self):
        n=self.node();self.start(n);self.step(n,10)
        n.z=2.2;n.vz=-.3
        for i in range(1,21):u=self.step(n,10+i*.02)
        self.assertGreater(u,.518)
        self.assertAlmostEqual(n.vertical_target_speed,.1)
        self.assertAlmostEqual(n.vertical_correction,.08)
        self.assertLessEqual(u,.60)
    def test_upward_velocity_reduces_thrust_and_target_velocity_stays_bounded(self):
        n=self.node();self.start(n);self.step(n,10)
        n.z=2.8;n.vz=.3
        for i in range(1,31):u=self.step(n,10+i*.02)
        self.assertLess(u,.518);self.assertAlmostEqual(n.vertical_target_speed,-.15)
    def test_tilt_projection_uses_captured_vertical_support(self):
        n=self.node();self.start(n);self.step(n,10);n.tilt=20
        for i in range(1,31):u=self.step(n,10+i*.02)
        self.assertAlmostEqual(u,.518/math.cos(math.radians(20)))
        self.assertAlmostEqual(n.flip_support_weight,1.)
    def test_fade_is_smooth_finite_and_monotonic_with_no_reenable(self):
        n=self.node();self.start(n);self.step(n,10);last=1.
        for i,angle in enumerate(range(0,91)):
            n.tilt=angle;u=self.step(n,10.02+i*.02)
            self.assertTrue(math.isfinite(u));self.assertLessEqual(u,.85)
            self.assertLessEqual(n.flip_support_weight,last);last=n.flip_support_weight
        self.assertEqual(n.flip_support_weight,0.)
        n.tilt=10;self.step(n,12);self.assertEqual(n.flip_support_weight,0.)
        self.assertTrue(math.isnan(n.vertical_correction))
    def test_slew_applies_to_both_feedback_and_wall_pressure_transition(self):
        n=self.node();self.start(n);previous=self.step(n,10)
        for i in range(1,101):
            n.tilt=0 if i<30 else 90;n.vz=-2.
            value=self.step(n,10+i*.02)
            self.assertLessEqual(abs(value-previous),.5*.02+1e-9);previous=value
        self.assertAlmostEqual(previous,.48*1.35)
    def test_bad_feedback_never_falls_back_to_fixed_hover(self):
        for fault in ('odom','height','baseline'):
            n=self.node();self.start(n)
            if fault=='odom':n.odom_valid=False
            elif fault=='height':n.feedback_valid=False
            else:n.flip_base_vertical_thrust=None
            self.assertTrue(math.isnan(self.step(n,10.02)),fault)
    def test_missing_stale_future_or_foreign_final_output_rejects_flip(self):
        for fault in ('missing','stale','header','future','owner','owner_age','mask','nan'):
            n=self.node()
            if fault=='missing':n.horizontal_output=None
            elif fault=='stale':n.horizontal_output_at=rospy.Time.from_sec(9)
            elif fault=='header':n.horizontal_output.header.stamp=rospy.Time.from_sec(9)
            elif fault=='future':n.horizontal_output.header.stamp=rospy.Time.from_sec(11)
            elif fault=='owner':n.horizontal_output_owner=C.SOURCE_CEILING
            elif fault=='owner_age':n.horizontal_output_owner_at=rospy.Time.from_sec(9)
            elif fault=='mask':n.horizontal_output.type_mask=AttitudeTarget.IGNORE_THRUST
            else:n.horizontal_output.thrust=math.nan
            self.assertFalse(n._prepare_flip_vertical_support(rospy.Time.from_sec(10)),fault)
    def test_real_state_transition_reports_missing_source_as_abort(self):
        n=self.node();n.horizontal_output=None
        n._prepare_vertical_reference=lambda now:None
        with mock.patch.object(rospy,'loginfo'):
            n._enter_state(W.STATE_FLIP_TO_WALL,rospy.Time.from_sec(10),'test')
        self.assertEqual(n.state,W.STATE_ABORT);self.assertEqual(n.fault_count,1)
        self.assertIn('source missing',n.last_reason)
    def test_disarm_and_next_attempt_clear_all_captured_values(self):
        n=self.node();self.start(n)
        with mock.patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(10)):
            n._on_state(State(connected=True,armed=False,mode='POSCTL'))
        self.assertIsNone(n.flip_base_vertical_thrust)
        n.approach_base_vertical_thrust=.53;n.approach_altitude_m=3.;n.state=W.STATE_SLOW_APPROACH
        self.start(n);self.assertEqual(n.flip_altitude_m,3.)
    def test_legacy_and_other_phases_remain_separate(self):
        n=self.node();n.flip_vertical_support_enabled=False;n.tilt=0
        self.assertAlmostEqual(self.step(n,10),.48)
        n.tilt=90;self.assertAlmostEqual(self.step(n,10),.648)
        n.state=W.STATE_RECOVER;n.z=2.4;n.vz=0;n.tilt=0
        self.assertAlmostEqual(n._vertical_height_thrust(2.4),.48)
    def test_old_drop_profile_no_longer_reduces_thrust_to_point_four_eight(self):
        n=self.node();self.start(n);self.step(n,10)
        for i in range(1,43):
            n.tilt=14.*(i/42.)**3;n.z=2.4-.35*(i/42.)**2;n.vz=-.887*(i/42.)
            u=self.step(n,10+i*.02)
            self.assertGreaterEqual(u,.518-1e-9)
            self.assertLessEqual(n.vertical_correction,.08+1e-9)
    def test_real_node_status_exposes_early_flip_feedback_and_reference(self):
        from nav_msgs.msg import Odometry
        clock=mock.patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(10))
        clock.start();self.addCleanup(clock.stop)
        values={'~auto_approach':True,'~flip_vertical_support_enabled':True}
        with mock.patch.object(rospy,'get_param',side_effect=lambda key,default=None:values.get(key,default)), \
             mock.patch.object(rospy,'Publisher'),mock.patch.object(rospy,'Subscriber'), \
             mock.patch.object(rospy,'Timer'),mock.patch.object(rospy,'logwarn'):
            n=WallDecision()
        n.state=W.STATE_SLOW_APPROACH
        n.approach_altitude_m=2.4;n.approach_base_vertical_thrust=.518
        n.odom=Odometry();n.odom.pose.pose.orientation.w=1.;n.odom.pose.pose.position.z=2.4
        n.odom.header.stamp=rospy.Time.from_sec(10)
        n.odom_received_at=rospy.Time.from_sec(10)
        n.horizontal_output=AttitudeTarget(thrust=.518);n.horizontal_output.orientation.w=1.;n.horizontal_output.header.stamp=rospy.Time.from_sec(9.99)
        n.horizontal_output_at=n.horizontal_output_owner_at=rospy.Time.from_sec(9.99)
        n.horizontal_output_owner=C.SOURCE_WALL
        n._flight_ready=lambda now:True
        n._top_fresh=lambda now:False
        n._horizontal_attitude=lambda now,q:q
        self.start(n)
        status=n._build_status(rospy.Time.from_sec(10))
        self.assertTrue(status.flip_vertical_support_enabled)
        self.assertTrue(status.vertical_control_active)
        self.assertAlmostEqual(status.thrust_normalized,.518,places=6)
        self.assertAlmostEqual(status.vertical_reference_m,2.4,places=6)
        self.assertAlmostEqual(status.flip_entry_thrust,.518,places=6)
        self.assertAlmostEqual(status.flip_vertical_support_weight,1.)
        self.assertTrue(n._build_control_candidate(status).valid)
    def test_original_downward_speed_protection_is_retained_in_flip(self):
        from nav_msgs.msg import Odometry
        n=self.node();self.start(n)
        n.odom=Odometry();n.odom.pose.pose.orientation.w=1.
        n.odom.twist.twist.linear.z=-.887;n.max_down_velocity_mps=.8
        n.odom_twist_in_child_frame=True
        self.assertFalse(n._vertical_speed_safe())

    def test_simple_vertical_plant_removes_the_baseline_switch_drop(self):
        heights=[]
        for enabled in (False,True):
            n=self.node();n.flip_vertical_support_enabled=enabled
            if enabled:self.start(n);self.step(n,10)
            for i in range(1,43):
                n.tilt=14.*(i/42.)**3
                u=self.step(n,10+i*.02)
                acceleration=9.80665*(u*math.cos(math.radians(n.tilt))/.518-1.)
                n.vz+=acceleration*.02;n.z+=n.vz*.02
            heights.append(n.z)
        self.assertLess(heights[0],2.2)
        self.assertGreater(heights[1],2.38)

if __name__=='__main__':unittest.main()
