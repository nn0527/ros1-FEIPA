import math
import unittest
from unittest import mock
import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import State,AttitudeTarget,EstimatorStatus
from px4_wall_ceiling_control.msg import WallPerchStatus as W,OperatorCommand
from px4_wall_ceiling_control.domain.geometry import rotate_vector,quaternion_multiply,euler_from_quaternion
from px4_wall_ceiling_control.domain.recovery_attitude_limits import attitude_from_axis_yaw
from px4_wall_ceiling_control.domain.horizontal_equilibrium import HorizontalEquilibrium
from px4_wall_ceiling_control.domain.horizontal_velocity import HorizontalVelocityController
import test_wall_horizontal_velocity as legacy
from test_flight_handover import Manager,OutputMux

def pitch(deg):return (0.,math.sin(math.radians(deg)/2),0.,math.cos(math.radians(deg)/2))
def tilt(q):return math.degrees(math.acos(max(-1,min(1,rotate_vector(q,(0.,0.,1.))[2]))))

class EquilibriumMathTests(unittest.TestCase):
    def test_capture_needs_unique_continuous_stable_samples(self):
        c=HorizontalEquilibrium()
        c.observe(10.,pitch(2.5),True);c.observe(10.,pitch(2.5),True)
        self.assertFalse(c.ready)
        for t in (10.25,10.5,10.75,11.):c.observe(t,pitch(2.5),True)
        self.assertTrue(c.ready);self.assertAlmostEqual(tilt(attitude_from_axis_yaw(c.axis,0)),2.5)
        self.assertTrue(c.freeze(11.05))
        c.observe(11.25,pitch(-3),False)
        self.assertTrue(c.frozen);self.assertAlmostEqual(tilt(attitude_from_axis_yaw(c.axis,0)),2.5)
    def test_invalid_motion_gap_or_large_target_resets_window(self):
        for fault in ('motion','gap','tilt','spread'):
            c=HorizontalEquilibrium()
            for t in (10.,10.25,10.5,10.75,11.):c.observe(t,pitch(2.5),True)
            self.assertTrue(c.ready)
            c.observe(11.5 if fault=='gap' else 11.25,pitch(7 if fault=='tilt' else -2 if fault=='spread' else 2.5),fault!='motion')
            self.assertFalse(c.ready,fault)
    def test_stale_equilibrium_cannot_freeze_and_new_attempt_clears_bias(self):
        c=HorizontalEquilibrium()
        for t in (10.,10.25,10.5,10.75,11.):c.observe(t,pitch(2.5),True)
        self.assertFalse(c.freeze(12.));self.assertFalse(c.ready)
        c.clear();self.assertEqual(c.axis,(0.,0.,1.))
    def test_feedback_is_increment_not_absolute_one_point_seven_five_degree_cap(self):
        c=HorizontalVelocityController();axis=rotate_vector(pitch(2.5),(0.,0.,1.))
        q=c.attitude((0.,0.),(0.,0.),0.,.02,axis)
        self.assertAlmostEqual(tilt(q),2.5)
        q=c.attitude((.05,0.),(-.5,0.),0.,.02,axis)
        self.assertGreater(tilt(q),4.)
        self.assertLessEqual(math.hypot(*c.acceleration),.30)
    def test_final_sum_with_rc_still_obeys_shared_five_degree_cone(self):
        from px4_wall_ceiling_control.domain.recovery_attitude_limits import limit_recovery_attitude
        c=HorizontalVelocityController();axis=rotate_vector(pitch(4),(0.,0.,1.))
        q=c.attitude((.05,0.),(-1.,.5),0.,.02,axis)
        q=quaternion_multiply(q,(math.sin(math.radians(5)/2),0.,0.,math.cos(math.radians(5)/2)))
        q,_=limit_recovery_attitude(q,None,.02,math.radians(5),math.radians(10),math.radians(20))
        self.assertLessEqual(tilt(q),5.000001)

    def test_closed_loop_retains_known_disturbance_balance_instead_of_retreating(self):
        # Ideal attitude tracking plant with a constant horizontal disturbance.
        # This is a regression model, not a prediction of real flight performance.
        equilibrium=rotate_vector(pitch(2.5),(0.,0.,1.))
        control=HorizontalVelocityController();velocity=0.;minimum=0.
        for i in range(300):
            q=control.attitude((.05 if i>=50 else 0.,0.),(velocity,0.),0.,.02,equilibrium)
            axis=rotate_vector(q,(0.,0.,1.))
            acceleration=9.80665*(axis[0]/axis[2]-equilibrium[0]/equilibrium[2])
            velocity+=acceleration*.02;minimum=min(minimum,velocity)
        self.assertGreaterEqual(minimum,-1.e-8)
        self.assertAlmostEqual(velocity,.05,places=3)

class EquilibriumHandoverTests(unittest.TestCase):
    def setUp(self):
        self.f=legacy.HorizontalFlightTests();self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.n=self.f.n;self.n.horizontal_equilibrium.clear();self.n.state=W.STATE_IDLE
        self.n._mode_requested=lambda now:True
        self.n.manager_phase='WAITING_WALL'
        self.n._on_state(State(connected=True,armed=True,mode='POSCTL'))
    def sample(self,t,velocity=0.,target=2.5,actual=None,rates=(0.,0.,0.)):
        # A fixture update at the same time must not replay the initial flow frame.
        self.f.feed(t,vx=velocity,tilt=target if actual is None else actual,
                    flow=self.n.horizontal_flow_at != rospy.Time.from_sec(t))
        for axis,value in zip("xyz",rates):setattr(self.n.odom.twist.twist.angular,axis,math.radians(value))
        self.n._prepare_horizontal_velocity(rospy.Time.now())
        self.n.flight_state_received_at=rospy.Time.now();self.n.manager_phase_received_at=rospy.Time.now()
        self.n._on_horizontal_fcu_target(AttitudeTarget(orientation=Quaternion(*pitch(target)),thrust=.50))
        self.n._prepare_horizontal_equilibrium(rospy.Time.now())
    def capture(self):
        for t in (10.,10.25,10.5,10.75,11.,11.25,11.5):self.sample(t)
        self.assertTrue(self.n.horizontal_equilibrium.ready)
        self.assertTrue(self.n.horizontal_velocity_ready)
    def test_equilibrium_requires_half_second_below_twenty_centimetres_per_second(self):
        self.assertAlmostEqual(self.n.horizontal_equilibrium_speed,.20)
        self.assertAlmostEqual(self.n.horizontal_equilibrium.window,.5)
        for t in (10.,10.25,10.5,10.75,11.,11.25,11.5):self.sample(t,velocity=.201)
        self.assertFalse(self.n.horizontal_equilibrium.ready)
        for t in (11.75,12.):self.sample(t,velocity=.20)
        self.assertFalse(self.n.horizontal_equilibrium.ready)
        self.sample(12.25,velocity=.20)
        self.assertTrue(self.n.horizontal_equilibrium.ready)

    def test_entry_uses_only_pitch_rate_with_twenty_degree_limit(self):
        self.capture()
        for i, (rate, expected) in enumerate(((20., True), (-20., True), (20.01, False), (-20.01, False), (float('nan'), False))):
            with mock.patch.object(self.n.horizontal_equilibrium, 'observe') as observe:
                self.sample(11.75+i*.25, rates=(60., rate, -60.))
                self.assertEqual(observe.call_args[0][2], expected)
            if expected:self.assertTrue(self.n.horizontal_velocity_ready)

    def test_entry_tracking_accepts_below_five_degrees_and_rejects_above(self):
        self.capture()
        for i,(actual,expected) in enumerate(((-2.4,True),(-2.6,False))):
            with mock.patch.object(self.n.horizontal_equilibrium,'observe') as observe:
                self.sample(11.75+i*.25,target=2.5,actual=actual)
                self.assertEqual(observe.call_args[0][2],expected)

    def test_no_entry_baseline_from_moving_or_untracked_attitude(self):
        for t in (10.,10.25,10.5,10.75,11.):self.sample(t,velocity=.3)
        self.assertFalse(self.n.horizontal_equilibrium.ready)
        for t in (11.25,11.5,11.75,12.,12.25):self.sample(t,actual=-3.)
        self.assertFalse(self.n.horizontal_equilibrium.ready)
    def test_prestream_keeps_nonzero_balance_and_starts_feedback_before_active(self):
        self.capture();self.n._prepare_horizontal_prestream(rospy.Time.now())
        self.assertAlmostEqual(tilt(self.n.horizontal_prestream_attitude),2.5)
        self.f.feed(11.75,vx=-.1,tilt=2.5)
        self.n._prepare_horizontal_prestream(rospy.Time.now())
        self.assertGreater(tilt(self.n.horizontal_prestream_attitude),2.5)
        self.assertEqual(self.n.horizontal_controller.target,(0.,0.))
    def test_entering_freezes_and_front_transition_retains_fresh_velocity_ready(self):
        self.capture();self.n.manager_phase='ENTERING'
        self.n._prepare_horizontal_equilibrium(rospy.Time.now())
        self.assertTrue(self.n.horizontal_equilibrium.frozen)
        self.n._on_state(State(connected=True,armed=True,mode='OFFBOARD'))
        now=rospy.Time.now()
        self.n._enter_state(W.STATE_FRONT_WALL_DETECT,now,'confirmed')
        self.n._enter_state(W.STATE_SLOW_APPROACH,now,'approach')
        q=self.n._horizontal_attitude(now,self.n.q_hover)
        self.assertTrue(self.n.horizontal_velocity_active)
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.assertGreater(tilt(q),2.5)
        self.assertGreater(self.n.horizontal_controller.target[0],0.)
    def test_offboard_echo_cannot_relearn_equilibrium(self):
        self.capture();self.n.manager_phase='ENTERING';self.n._prepare_horizontal_equilibrium(rospy.Time.now())
        old=self.n.horizontal_equilibrium.axis
        self.n._on_state(State(connected=True,armed=True,mode='OFFBOARD'))
        self.n._on_horizontal_fcu_target(AttitudeTarget(orientation=Quaternion(*pitch(-4.))))
        self.n._prepare_horizontal_equilibrium(rospy.Time.now())
        self.assertEqual(self.n.horizontal_equilibrium.axis,old)
    def test_flip_pauses_feedback_but_retains_balance_for_recovery(self):
        self.capture();self.n.manager_phase='ENTERING';self.n._prepare_horizontal_equilibrium(rospy.Time.now())
        self.n.state=W.STATE_FLIP_TO_WALL
        q=self.n._horizontal_attitude(rospy.Time.now(),pitch(60.))
        self.assertAlmostEqual(tilt(q),60.)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertTrue(self.n.horizontal_equilibrium.frozen)
    def test_invalid_flow_falls_back_to_balance_not_euler_zero(self):
        self.capture();self.n.manager_phase='ENTERING';self.n._prepare_horizontal_equilibrium(rospy.Time.now())
        self.n.state=W.STATE_SLOW_APPROACH
        self.f.feed(11.75,quality=0,tilt=2.5)
        q=self.n._horizontal_attitude(rospy.Time.now(),(0.,0.,0.,1.))
        self.assertAlmostEqual(tilt(q),2.5)
        self.assertFalse(self.n.horizontal_velocity_active)
    def test_idle_return_clears_previous_attempt(self):
        self.capture();self.n.state=W.STATE_EXIT
        self.n._enter_state(W.STATE_IDLE,rospy.Time.now(),'returned')
        self.assertFalse(self.n.horizontal_equilibrium.ready)
        self.assertIsNone(self.n.horizontal_takeover_yaw)

class EquilibriumAdmissionTests(unittest.TestCase):
    def test_manager_blocks_without_new_equilibrium(self):
        n=Manager.__new__(Manager);n.wall_horizontal_velocity_enabled=True
        n.selected_mode=OperatorCommand.MODE_WALL;n.wall_pilot_mode='POSCTL'
        n.wall=W(horizontal_velocity_enabled=True,horizontal_velocity_ready=True,horizontal_equilibrium_enabled=True)
        n.wall_at=n.estimator_at=rospy.Time.from_sec(10.);n.status_timeout_s=.5;n.estimator_timeout_s=1.
        n.estimator=EstimatorStatus(velocity_horiz_status_flag=True,pos_horiz_rel_status_flag=True)
        self.assertFalse(n._wall_horizontal_ready(rospy.Time.from_sec(10.)))
        n.wall.horizontal_equilibrium_valid=True
        self.assertTrue(n._wall_horizontal_ready(rospy.Time.from_sec(10.)))
    def test_waiting_can_collect_baseline_but_cannot_request_takeover_without_it(self):
        import test_manual_wall_approach as fixtures
        f=fixtures.ManualApproachTests();f.setUp();n=f.node
        n.wall_auto_approach=True;n.wall_manual_approach=False;n.phase=Manager.WAITING_WALL
        n.selected_mode=OperatorCommand.MODE_WALL
        n.wall.front_ready=n.wall.sensor_health_fresh=True
        n.wall.horizontal_equilibrium_enabled=True;n.wall.horizontal_equilibrium_valid=False
        self.assertFalse(n._wall_takeover_ready(f.now))
        n.wall.horizontal_equilibrium_valid=True
        self.assertTrue(n._wall_takeover_ready(f.now))

    def test_mux_uses_supplied_prestream_attitude_and_cancels_without_offboard_baseline(self):
        n=OutputMux.__new__(OutputMux);now=rospy.Time.from_sec(10.)
        n.match_fcu_thrust_on_handover=True;n.flight_state=State(mode='OFFBOARD')
        n.handover_start_thrust=.50
        n.wall_status=W(horizontal_equilibrium_enabled=True,horizontal_equilibrium_valid=True,
            horizontal_equilibrium_frozen=True,horizontal_prestream_valid=True,
            horizontal_prestream_attitude=Quaternion(*pitch(3.)))
        n.wall_status_at=now;n.status_timeout_s=.5
        n._operator_mode=lambda now:OperatorCommand.MODE_WALL
        n._level_attitude=lambda:Quaternion(w=1.)
        n._assisted_attitude=lambda now,q,source:q
        n._attitude_target=lambda now,q,thrust,apply_handover:(q,thrust)
        target=n._prestream_attitude_target(now)
        self.assertAlmostEqual(tilt(tuple(getattr(target[0],k) for k in 'xyzw')),3.)
        n.wall_status.horizontal_prestream_valid=False
        n.wall_status.horizontal_equilibrium_valid=False
        with mock.patch.object(rospy,'logerr_throttle'):
            self.assertIsNone(n._prestream_attitude_target(now))


    def test_mux_prestream_limits_balance_plus_feedback_and_rc_before_mode_heartbeat(self):
        from test_auto_wall_approach import AutoOutputTests
        from mavros_msgs.msg import AttitudeTarget
        from px4_wall_ceiling_control.msg import AttachmentControlCandidate as C
        n,now=AutoOutputTests().node()
        n.flight_state.mode='POSCTL'  # Actual OFFBOARD can precede its heartbeat.
        n.wall_status.state=W.STATE_IDLE
        n.wall_status.horizontal_equilibrium_enabled=True
        n.operator=OperatorCommand(mode=OperatorCommand.MODE_WALL,enable_control=True)
        n.operator_at=now;n.operator_timeout_s=.5
        q=quaternion_multiply(pitch(4.),(math.sin(math.radians(5)/2),0.,0.,math.cos(math.radians(5)/2)))
        target=AttitudeTarget(orientation=Quaternion(*q),thrust=.5)
        final=n._limit_final_approach_target(rospy.Time.from_sec(10.02),target,C.SOURCE_NONE)
        finalq=tuple(getattr(final.orientation,k) for k in 'xyzw')
        self.assertLessEqual(tilt(finalq),.200001)
        # The final sum is cone-limited even when previous history has reached its edge.
        n.approach_previous_at=rospy.Time.from_sec(10.0)
        n.approach_previous_target=q
        with mock.patch.object(rospy,'logwarn_throttle'):
            final=n._limit_final_approach_target(rospy.Time.from_sec(10.1),AttitudeTarget(orientation=Quaternion(*q),thrust=.5),C.SOURCE_NONE)
        self.assertLessEqual(tilt(tuple(getattr(final.orientation,k) for k in 'xyzw')),5.000001)

if __name__=='__main__':unittest.main()
