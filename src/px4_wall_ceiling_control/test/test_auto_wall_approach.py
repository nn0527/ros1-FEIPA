"""Automatic approach gate, owner, pause deadline and final-setpoint regression tests."""
import math
import unittest
from unittest import mock
import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget
from nav_msgs.msg import Odometry
from px4_wall_ceiling_control.msg import WallPerchStatus as S, AttachmentControlCandidate as C, OperatorCommand, SensorHealth
from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion, rotate_vector
from px4_wall_ceiling_control.domain.recovery_attitude_limits import axis_yaw
from test_manual_wall_approach import ManualApproachTests
from test_flight_handover import Manager, PreflipBiasTest, OutputMux, wall_control_fixture
from test_attachment_safety import WallDecision
from test_recovery_attitude_limits import q, tilt

class AutoManagerTests(unittest.TestCase):
    def setUp(self):
        self.fixture = ManualApproachTests(); self.fixture.setUp()
        self.n = self.fixture.node
        self.n.wall_manual_approach = False; self.n.wall_auto_approach = True
        self.n.wall.front_ready = True; self.n.wall.sensor_health_fresh = True
        self.n.enter_timeout_s = 8
    def tick(self): self.fixture.tick()
    def test_takeover_on_front_ready_not_flip_ready(self):
        self.tick(); self.tick(); self.fixture.clock += 2; self.tick()
        self.assertEqual(self.n.phase, Manager.ENTERING)
        self.assertFalse(self.n.wall.flip_ready)
        self.n._request_mode.assert_called_once_with('OFFBOARD')
    def test_no_takeover_with_invalid_or_unconfirmed_front_gate(self):
        self.n.selected_mode = OperatorCommand.MODE_WALL
        for field in ('front_ready','front_range_fresh','sensor_health_fresh'):
            setattr(self.n.wall,field,False)
            self.assertFalse(self.n._wall_takeover_ready(self.fixture.now),field)
            setattr(self.n.wall,field,True)
        self.n.sensor.ceiling_valid = False
        self.n.wall.top_range_fresh = False
        self.n.wall.top_preflight_ready = False
        self.assertTrue(self.n._wall_takeover_ready(self.fixture.now))
    def test_range_bypass_enters_without_any_sensor_feedback(self):
        self.n.ignore_range_loss_during_attachment=True
        for field in ('front_ready','flip_ready','front_range_fresh','top_range_fresh',
                      'sensor_health_fresh','top_preflight_ready'):
            setattr(self.n.wall,field,False)
        self.n.sensor=None
        self.tick();self.tick();self.fixture.clock+=2;self.tick()
        self.assertEqual(self.n.phase,Manager.ENTERING)
        self.n._request_mode.assert_called_once_with('OFFBOARD')
        self.assertFalse(self.n.wall.flip_ready)

    def test_range_bypass_preserves_height_baseline_and_status_freshness(self):
        self.n.selected_mode=OperatorCommand.MODE_WALL
        self.n.ignore_range_loss_during_attachment=True
        self.n.wall.horizontal_equilibrium_enabled=True
        self.n.wall.horizontal_equilibrium_valid=True
        self.assertTrue(self.n._wall_takeover_ready(self.fixture.now))
        self.n.wall.horizontal_equilibrium_valid=False
        self.assertFalse(self.n._wall_takeover_ready(self.fixture.now))
        self.n.wall.horizontal_equilibrium_valid=True
        self.n.wall.entry_height_ready=False
        self.assertFalse(self.n._wall_takeover_ready(self.fixture.now))
        self.n.wall.entry_height_ready=True
        self.n.wall_at=rospy.Time.from_sec(1)
        self.assertFalse(self.n._wall_takeover_ready(self.fixture.now))

    def test_confirmed_offboard_does_not_wait_for_flip_gate(self):
        self.n.selected_mode = OperatorCommand.MODE_WALL
        self.n.phase = Manager.ENTERING; self.n.flight.mode = 'OFFBOARD'
        self.n.offboard_request = (getattr(self.n,'takeover_attempt',0),OperatorCommand.MODE_WALL,self.fixture.clock)
        self.n.wall.front_ready = self.n.wall.flip_ready = False
        self.tick()
        self.assertEqual(self.n.phase, Manager.ACTIVE)
    def test_unrequested_offboard_does_not_authorize_approach(self):
        self.n.selected_mode = OperatorCommand.MODE_WALL
        self.n.phase = Manager.ENTERING; self.n.flight.mode = 'OFFBOARD'
        self.n.offboard_request = None
        self.tick()
        self.assertEqual(self.n.phase, Manager.CANCELING_ENTRY)
        self.n.cancel_pub.publish.assert_called()
    def test_rpc_rechecks_front_gate(self):
        self.n.selected_mode = OperatorCommand.MODE_WALL; self.n.phase = Manager.ENTERING
        self.n.set_mode = mock.Mock();self.n.request_in_flight = True
        self.n.wall.front_ready = False
        with mock.patch.object(rospy.Time,'now',return_value=self.fixture.now),mock.patch.object(rospy,'wait_for_service'):
            self.n._send_mode_request('OFFBOARD')
        self.n.set_mode.assert_not_called()

class AutoDecisionTests(unittest.TestCase):
    @staticmethod
    def range_sample(distance):
        from sensor_msgs.msg import Range
        sample = Range(range=distance, min_range=.03, max_range=40)
        sample.header.stamp = rospy.Time.now()  # Distinct real sensor sample, not a zero-stamp replay.
        return sample

    def node(self):
        n=WallDecision.__new__(WallDecision)
        n.auto_approach=True;n.manual_approach=False;n.state=S.STATE_SLOW_APPROACH
        n.cancel_requested=n.rearm_required=False;n.fault_count=0
        n.state_entered_at=rospy.Time.from_sec(10);n.approach_timeout_s=20
        n.flip_trigger_distance_m=.2;n.flip_trigger_hold_s=.3
        n._mode_requested=n._entry_height_ready=n._safety_ok=n._auto_takeover_confirmed=lambda *a:True
        n._approach_far_distance=lambda *a:None
        n._odom_fresh=lambda *a:False;n.recover_time_s=1
        n._auto_approach_ranges_ready=lambda *a:True;n._flip_ready=lambda *a:False
        n.flip_ready_since=rospy.Time.from_sec(9);n.flip_ready_last_sample_at=rospy.Time.from_sec(10)
        n._vertical_height_thrust=mock.Mock(return_value=.47);n.recovery_max_thrust=.6
        n.q_hover=(0,0,0,1);n.approach_pitch_rad=math.radians(3);n.approach_thrust=.47;n.max_thrust=.85
        def enter(state,now,reason):n.state=state;n.state_entered_at=now;n.last_reason=reason
        n._enter_state=mock.Mock(side_effect=enter)
        return n
    def test_entry_is_slow_approach_not_direct_flip(self):
        n=self.node();n.state=S.STATE_IDLE
        n._update_state_machine(rospy.Time.from_sec(10))
        self.assertEqual([a[0][0] for a in n._enter_state.call_args_list],[S.STATE_FRONT_WALL_DETECT,S.STATE_SLOW_APPROACH])
        self.assertTrue(n._acquire_requested(rospy.Time.from_sec(10)))
    def test_entry_requires_fresh_manager_active_confirmation(self):
        n=self.node();n.state=S.STATE_IDLE
        n._auto_takeover_confirmed=WallDecision._auto_takeover_confirmed.__get__(n)
        n.manager_phase_timeout_s=.5;n.manager_phase_received_at=rospy.Time.from_sec(10)
        for phase,t in [('ENTERING',10),('ACTIVE',10.6)]:
            n.manager_phase=phase;n._update_state_machine(rospy.Time.from_sec(t))
            self.assertEqual(n.state,S.STATE_IDLE)
        n.manager_phase='ACTIVE';n._update_state_machine(rospy.Time.from_sec(10.1))
        self.assertEqual(n.state,S.STATE_SLOW_APPROACH)
    def test_pause_keeps_owner_level_candidate_and_deadline(self):
        n=self.node();n._auto_approach_ranges_ready=lambda *a:False
        n._update_state_machine(rospy.Time.from_sec(15))
        self.assertEqual(n.state,S.STATE_SLOW_APPROACH)
        self.assertEqual(n.state_entered_at,rospy.Time.from_sec(10))
        self.assertIsNone(n.flip_ready_since)
        self.assertTrue(n._acquire_requested(rospy.Time.from_sec(15)))
        neutral,v,thrust,_=n._candidate(rospy.Time.from_sec(15))
        self.assertFalse(neutral);self.assertAlmostEqual(tilt(v),0);self.assertAlmostEqual(thrust,.47)
        n._update_state_machine(rospy.Time.from_sec(30))
        self.assertEqual(n.state,S.STATE_ABORT)
        self.assertIn('超时',n.last_reason)
    def test_range_bypass_keeps_approaching_without_false_flip(self):
        n=self.node();n.ignore_range_loss_during_attachment=True
        n._auto_approach_ranges_ready=lambda *a:False
        n._flip_ready=lambda *a:True  # A cached distance must not authorize flip.
        now=rospy.Time.from_sec(15)
        n._update_state_machine(now)
        self.assertEqual(n.state,S.STATE_SLOW_APPROACH)
        self.assertIsNone(n.flip_ready_since)
        self.assertIn('继续自动接近',n.last_reason)
        neutral,v,thrust,_=n._candidate(now)
        self.assertFalse(neutral);self.assertAlmostEqual(tilt(v),3)
        self.assertTrue(n._approach_motion_allowed(now))
        self.assertEqual(n.state_entered_at,rospy.Time.from_sec(10))
        n._update_state_machine(rospy.Time.from_sec(30))
        self.assertEqual(n.state,S.STATE_ABORT)

    def test_range_bypass_covers_detach_and_recovery(self):
        n=self.node();n.ignore_range_loss_during_attachment=True
        for state in (S.STATE_DETACH_ROTATE,S.STATE_RECOVER,S.STATE_ABORT,S.STATE_EXIT):
            n.state=state
            self.assertTrue(n._ignore_attachment_range_loss())
            self.assertFalse(n._top_required())
            self.assertTrue(n._top_preflight_ready(rospy.Time.from_sec(15)))

    def test_valid_recovery_restores_three_degree_forward(self):
        n=self.node();n._auto_approach_ranges_ready=lambda *a:False
        n._update_state_machine(rospy.Time.from_sec(15))
        n._auto_approach_ranges_ready=lambda *a:True
        n._update_state_machine(rospy.Time.from_sec(16))
        self.assertEqual(n.state,S.STATE_SLOW_APPROACH)
        neutral,v,thrust,_=n._candidate(rospy.Time.from_sec(16))
        self.assertGreater(rotate_vector(v,(0,0,1))[0],0)
        self.assertAlmostEqual(math.degrees(euler_from_quaternion(v)[1]),3)
        self.assertAlmostEqual(thrust,.47)
    def test_valid_flip_confirmation_is_still_required(self):
        n=self.node();n._flip_ready=lambda *a:True
        n._auto_approach_ranges_ready=lambda *a:False
        n._update_state_machine(rospy.Time.from_sec(15));self.assertEqual(n.state,S.STATE_SLOW_APPROACH)
        n._auto_approach_ranges_ready=lambda *a:True
        n._update_state_machine(rospy.Time.from_sec(16));self.assertEqual(n.state,S.STATE_FLIP_TO_WALL)
    def test_far_distance_and_flight_failure_still_abort(self):
        for cause in ('far','flight','cancel'):
            n=self.node()
            if cause=='far':n._approach_far_distance=lambda *a:1.81
            if cause=='flight':n._safety_ok=lambda *a:False
            if cause=='cancel':n.cancel_requested=True
            n._update_state_machine(rospy.Time.from_sec(15));self.assertEqual(n.state,S.STATE_ABORT,cause)
    def test_sensor_pause_does_not_mask_other_safety(self):
        n=self.node();n._safety_ok=WallDecision._safety_ok.__get__(n)
        n._flight_ready=n._odom_fresh=n._rates_safe=n._vertical_speed_safe=lambda *a:True
        self.assertTrue(n._safety_ok(rospy.Time.from_sec(15)))
        for field in ('_flight_ready','_odom_fresh','_rates_safe','_vertical_speed_safe'):
            original=getattr(n,field);setattr(n,field,lambda *a:False)
            self.assertFalse(n._safety_ok(rospy.Time.from_sec(15)),field);setattr(n,field,original)
    def test_distance_boundaries_and_continuous_hold_windows(self):
        from sensor_msgs.msg import Range
        n=self.node();n.front_ready_distance_m=.5;n.front_ready_hold_s=.4
        n.flip_trigger_distance_m=.2;n.flip_trigger_hold_s=.3;n.sensor_timeout_s=.6
        n.front_received_at=None;n.front_filtered_m=None
        n.front_ready_since=n.front_ready_last_sample_at=None
        n.flip_ready_since=n.flip_ready_last_sample_at=None
        n._filter_range=lambda m,previous:m.range
        n._flip_ready=WallDecision._flip_ready.__get__(n)
        for t in [10.,10.1,10.2,10.3]:
            with mock.patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(t)):
                n._on_front_range(self.range_sample(.5))
        self.assertFalse(n._front_ready(rospy.Time.from_sec(10.3)))
        with mock.patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(10.4)):
            n._on_front_range(self.range_sample(.5))
        self.assertTrue(n._front_ready(rospy.Time.from_sec(10.4)))
        self.assertFalse(n._flip_ready(rospy.Time.from_sec(10.4)))
        for t in [10.5,10.6,10.7]:
            with mock.patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(t)):
                n._on_front_range(self.range_sample(.2))
        self.assertFalse(n._flip_ready(rospy.Time.from_sec(10.7)))
        with mock.patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(10.8)):
            n._on_front_range(self.range_sample(.2))
        self.assertTrue(n._flip_ready(rospy.Time.from_sec(10.8)))
        n._update_state_machine(rospy.Time.from_sec(10.8))
        self.assertEqual(n.state,S.STATE_FLIP_TO_WALL)

    def test_pause_then_valid_data_requires_new_flip_samples(self):
        from sensor_msgs.msg import Range
        n=self.node();n.front_ready_distance_m=.5;n.front_ready_hold_s=.4
        n.flip_trigger_distance_m=.2;n.flip_trigger_hold_s=.3;n.sensor_timeout_s=.6
        n.front_received_at=rospy.Time.from_sec(10);n.front_filtered_m=.2
        n.front_ready_since=n.front_ready_last_sample_at=None
        n._filter_range=lambda m,previous:m.range
        n._flip_ready=WallDecision._flip_ready.__get__(n)
        n._auto_approach_ranges_ready=lambda *a:False
        n._update_state_machine(rospy.Time.from_sec(10.1))
        n._auto_approach_ranges_ready=lambda *a:True
        with mock.patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(10.2)):
            n._on_front_range(self.range_sample(.2))
        self.assertFalse(n._flip_ready(rospy.Time.from_sec(10.2)))
        self.assertEqual(n.flip_ready_since,rospy.Time.from_sec(10.2))

    def test_invalid_health_restarts_continuous_confirmation(self):
        n=self.node();n.front_ready_since=n.front_ready_last_sample_at=rospy.Time.from_sec(9)
        with mock.patch.object(rospy.Time,"now",return_value=rospy.Time.from_sec(10)):
            n._on_sensor_health(SensorHealth(front_valid=False,ceiling_valid=True))
        self.assertIsNone(n.front_ready_since);self.assertIsNone(n.flip_ready_since)

class AutoOutputTests(unittest.TestCase):
    def node(self):
        n,now=PreflipBiasTest().node();n.preflip_right_roll_deg=1.7
        n.approach_attitude_limits_enabled=True;n.approach_max_tilt_deg=5
        n.approach_tilt_slew_deg_s=10;n.approach_yaw_slew_deg_s=20
        n.approach_target_publisher=mock.Mock();n.approach_previous_target=q();n.approach_previous_at=now
        n.wall_status.state=S.STATE_SLOW_APPROACH;n.odom=Odometry();n.odom.pose.pose.orientation.w=1
        n.wall_output_enabled=True;n.wall_candidate_timeout_s=.2;n.owner=OutputMux.OWNER_NONE
        with mock.patch.object(rospy.Time,'now',return_value=now):
            n._on_wall_candidate(wall_control_fixture(S.STATE_SLOW_APPROACH,now))
        self.assertTrue(n._wall_fresh(now))
        return n,now
    def test_sum_then_total_cone_limit_preserves_direction(self):
        n,now=self.node();n.preflip_bias_rad=math.radians(1.7)
        # A 5-degree RC roll is combined with the bias before the shared cone clamp.
        from px4_wall_ceiling_control.domain.geometry import quaternion_multiply
        base=quaternion_multiply((0,math.sin(math.radians(3)/2),0,math.cos(math.radians(3)/2)),(math.sin(math.radians(5)/2),0,0,math.cos(math.radians(5)/2)))
        desired=n._preflip_bias_attitude(now,Quaternion(x=base[0],y=base[1],z=base[2],w=base[3]),C.SOURCE_WALL)
        raw=(desired.x,desired.y,desired.z,desired.w)
        self.assertAlmostEqual(math.degrees(euler_from_quaternion(raw)[0]),6.7)
        # Use enough history time to isolate the cone projection from the slew budget.
        n.approach_previous_target=q(5,math.degrees(math.atan2(axis_yaw(raw)[0][1],axis_yaw(raw)[0][0])))
        target=n._limit_final_approach_target(now,AttitudeTarget(orientation=desired,thrust=.47),C.SOURCE_WALL)
        final=tuple(getattr(target.orientation,k) for k in 'xyzw')
        self.assertLessEqual(tilt(final),5+1e-6)
        a,b=axis_yaw(raw)[0],axis_yaw(final)[0]
        self.assertAlmostEqual(a[0]/a[1],b[0]/b[1],places=5)
    def test_axis_and_yaw_rates_and_thrust_preserved(self):
        n,now=self.node();v=q(5,0,90)
        target=AttitudeTarget(orientation=Quaternion(x=v[0],y=v[1],z=v[2],w=v[3]),thrust=.47)
        out=n._limit_final_approach_target(rospy.Time.from_sec(10.02),target,C.SOURCE_WALL)
        final=tuple(getattr(out.orientation,k) for k in 'xyzw')
        self.assertAlmostEqual(tilt(final),.2,places=5)
        self.assertAlmostEqual(math.degrees(axis_yaw(final)[1]),.4)
        self.assertAlmostEqual(out.thrust,.47)
    def test_pause_clears_bias_and_slews_toward_level(self):
        n,now=self.node();n.preflip_bias_rad=math.radians(1.7);n.wall_status.approach_paused=True
        n.wall_candidate.approach_paused=True
        v=n._preflip_bias_attitude(now,Quaternion(w=1),C.SOURCE_WALL)
        self.assertEqual(n.preflip_bias_rad,0);self.assertEqual(v.x,0)
        n.approach_previous_target=q(3.45,0)
        out=n._limit_final_approach_target(rospy.Time.from_sec(10.02),AttitudeTarget(orientation=v,thrust=.47),C.SOURCE_WALL)
        final=tuple(getattr(out.orientation,k) for k in 'xyzw')
        self.assertAlmostEqual(tilt(final),3.25,places=5)
    def test_limit_and_bias_excluded_from_protected_phases(self):
        for state in (S.STATE_FLIP_TO_WALL,S.STATE_WALL_CAPTURE,S.STATE_WALL_HOLD,S.STATE_DETACH_ROTATE,S.STATE_RECOVER,S.STATE_ABORT,S.STATE_EXIT):
            n,now=self.node();n.wall_status.state=state
            n.wall_candidate.control_phase=state
            v=q(80,0);target=AttitudeTarget(orientation=Quaternion(x=v[0],y=v[1],z=v[2],w=v[3]),thrust=.7)
            out=n._limit_final_approach_target(now,target,C.SOURCE_WALL)
            self.assertAlmostEqual(tilt(tuple(getattr(out.orientation,k) for k in 'xyzw')),80)
            corrected=n._preflip_bias_attitude(now,Quaternion(w=1),C.SOURCE_WALL)
            self.assertEqual(corrected.x,0)
    def test_prestream_does_not_inject_auto_bias_and_seeds_history(self):
        n,now=self.node();n.wall_status.state=S.STATE_IDLE
        corrected=n._preflip_bias_attitude(now,Quaternion(w=1),C.SOURCE_NONE)
        self.assertEqual(corrected.x,0)
        n.approach_previous_target=None
        target=n._limit_final_approach_target(now,AttitudeTarget(orientation=corrected,thrust=.48),C.SOURCE_NONE)
        self.assertIsNone(n.approach_previous_target)
        n.attitude_publisher=mock.Mock()
        n.attitude_publisher.publish(target)
        n._remember_publication(target,C.SOURCE_NONE,now)
        self.assertEqual(n.approach_previous_target,(0,0,0,1))
    def test_publish_path_applies_limits_after_rc(self):
        n,now=self.node();raw=q(12,45)
        n._assisted_attitude=mock.Mock(return_value=Quaternion(x=raw[0],y=raw[1],z=raw[2],w=raw[3]))
        n._attitude_target=lambda stamp,v,u:AttitudeTarget(orientation=v,thrust=u)
        n._limit_final_recovery_target=lambda stamp,v,source,candidate=None:v
        n.attitude_publisher=mock.Mock()
        candidate=n.wall_candidate
        with mock.patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(10.02)):
            n._publish_candidate(rospy.Time.from_sec(10.02),candidate)
        sent=n.attitude_publisher.publish.call_args[0][0]
        self.assertLessEqual(tilt(tuple(getattr(sent.orientation,k) for k in 'xyzw')),.2+1e-6)

class ApproachConsoleTests(unittest.TestCase):
    def test_pause_reason_change_is_visible_without_state_transition(self):
        from px4_wall_ceiling_control.wall_console import ConsoleState
        state=ConsoleState()
        state.update('wall',{'state_name':'SLOW_APPROACH','reason':'自动接近'})
        before=len(state.events)
        state.update('wall',{'state_name':'SLOW_APPROACH','reason':'自动接近暂停：雷达无效'})
        self.assertEqual(len(state.events),before+1)
        self.assertIn('暂停',state.events[-1]['text'])
        state.update('wall',{'state_name':'SLOW_APPROACH','reason':'自动接近暂停：雷达无效'})
        self.assertEqual(len(state.events),before+1)

if __name__=='__main__':unittest.main()
