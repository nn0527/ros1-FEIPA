"""Approach-only PI: geometry, bounded biased plant, and runtime authority."""
import math
import unittest
from px4_wall_ceiling_control.domain.horizontal_velocity import HorizontalVelocityController
from px4_wall_ceiling_control.domain.geometry import rotate_vector
from px4_wall_ceiling_control.domain.recovery_attitude_limits import attitude_from_axis_yaw


class HorizontalPIMathTests(unittest.TestCase):
    def test_nonzero_balance_and_heading_preserve_world_error_direction(self):
        for yaw in (0.,math.pi/2,math.pi,-1.3):
            f=(math.cos(yaw),math.sin(yaw));left=(-f[1],f[0])
            balance=(.03*f[0]+.02*left[0],.03*f[1]+.02*left[1],1.)
            norm=math.sqrt(sum(v*v for v in balance));balance=tuple(v/norm for v in balance)
            for forward_speed in (-.1,.2):
                c=HorizontalVelocityController()
                measured=tuple(forward_speed*a+.1*b for a,b in zip(f,left))
                for _ in range(30):
                    q=c.attitude(tuple(.05*v for v in f),measured,yaw,.02,balance,
                                 integrate=True,integral_dt=.02,max_tilt=math.radians(5))
                axis=rotate_vector(q,(0,0,1))
                delta=tuple(axis[i]/axis[2]-balance[i]/balance[2] for i in (0,1))
                self.assertGreater(sum(a*b for a,b in zip(delta,f))*(1 if forward_speed<0 else -1),0)
                self.assertLess(sum(a*b for a,b in zip(delta,left)),0)
                self.assertLessEqual(math.degrees(math.acos(axis[2])),5.000001)

    def test_biased_simple_plant_converges_and_integral_stays_bounded(self):
        for bias_deg in (-1.,-.5,.5,1.):
            for yaw in (0.,1.7):
                f=(math.cos(yaw),math.sin(yaw));c=HorizontalVelocityController();v=[-.02*a for a in f]
                baseline=tuple(math.tan(math.radians(2))*a for a in f)+(1.,)
                disturbance=9.80665*math.tan(math.radians(bias_deg))
                for _ in range(1000):
                    c.attitude(tuple(.05*a for a in f),v,yaw,.02,baseline,
                               integrate=True,integral_dt=.02,max_tilt=math.radians(5))
                    v=[x+(a-disturbance*b)*.02 for x,a,b in zip(v,c.acceleration,f)]
                    self.assertLessEqual(math.hypot(*c.integral),.20000001)
                    self.assertLessEqual(math.hypot(*c.acceleration),.30000001)
                self.assertAlmostEqual(sum(a*b for a,b in zip(v,f)),.05,delta=.003)

    def test_combined_and_cone_saturation_block_growth_but_allow_release(self):
        c=HorizontalVelocityController();c.target=(.05,0)
        for _ in range(200):c.attitude((.05,0),(-2.,0),0,.02,integrate=True,integral_dt=.02,max_tilt=math.radians(5))
        self.assertEqual(c.integral,(0.,0.));self.assertTrue(c.saturated)
        c.integral=(.15,0)
        c.attitude((.05,0),(.5,0),0,.02,integrate=True,integral_dt=.02,max_tilt=math.radians(5))
        self.assertLess(c.integral[0],.15)
        c.reset();c.target=(.05,0);balance=(math.tan(math.radians(5)),0,1)
        for _ in range(100):
            q=c.attitude((.05,0),(-.05,0),0,.02,balance,integrate=True,integral_dt=.02,max_tilt=math.radians(5))
        self.assertEqual(c.integral,(0.,0.));self.assertTrue(c.saturated)
        self.assertAlmostEqual(math.degrees(math.acos(rotate_vector(q,(0,0,1))[2])),5.)
        c.integral=(.1,0)
        c.attitude((.05,0),(.06,0),0,.02,balance,integrate=True,integral_dt=.02,max_tilt=math.radians(5))
        self.assertLess(c.integral[0],.1)

    def test_actual_mux_cone_limit_blocks_outward_integral_before_candidate_limit(self):
        c=HorizontalVelocityController();c.target=(.05,0)
        base=(math.tan(math.radians(4)),0,1)
        applied=(math.sin(math.radians(5)),0,math.cos(math.radians(5)))
        c.attitude((.05,0),(0,0),0,.02,base,integrate=True,integral_dt=.02,
                   max_tilt=math.radians(5),applied_axis=applied)
        self.assertEqual(c.integral,(0.,0.));self.assertTrue(c.saturated)
        c.integral=(.1,0)
        c.attitude((.05,0),(.1,0),0,.02,base,integrate=True,integral_dt=.02,
                   max_tilt=math.radians(5),applied_axis=applied)
        self.assertLess(c.integral[0],.1)

    def test_unique_sample_interval_preserves_ki_at_ten_hz(self):
        c=HorizontalVelocityController(kp=.1);c.target=(.05,0)
        for _ in range(10):c.attitude((.05,0),(0,0),0,.02,integrate=True,integral_dt=.1)
        self.assertAlmostEqual(c.integral[0],.015)

    def test_integral_cap_and_reset(self):
        c=HorizontalVelocityController()
        for _ in range(3000):c.attitude((.05,0),(0,0),0,.02,integrate=True,integral_dt=.02)
        self.assertAlmostEqual(c.integral[0],.2);self.assertTrue(c.saturated)
        c.clear_integral();self.assertEqual(c.integral,(0.,0.))
        self.assertEqual(c.target,(.05,0.))
        c.reset();self.assertEqual(c.target,(0.,0.))


# ROS adapters are exercised in the Noetic container, without a ROS master.
try:
    import rospy
except ImportError:
    rospy=None

if rospy is not None:
    import test_wall_horizontal_velocity as fixtures
    from geometry_msgs.msg import Quaternion
    from mavros_msgs.msg import AttitudeTarget,State
    from std_msgs.msg import UInt8
    from px4_wall_ceiling_control.msg import WallPerchStatus as W

    class HorizontalPIAuthorityTests(unittest.TestCase):
        def setUp(self):
            self.f=fixtures.HorizontalFlightTests();self.f.setUp();self.addCleanup(self.f.doCleanups)
            self.n=self.f.n;self.f.ready();self.n.horizontal_takeover_yaw=0.
            self.n._flight_ready=lambda now:True  # kill gate has separate safety regressions
            self.q=(0.,0.,0.,1.)
            for t in (10.52,10.54,10.56,10.58):self.tick(t)
            self.assertGreater(self.n.horizontal_controller.integral[0],0.)

        def tick(self,t,owner=2,output=True,offset=0.,mode='OFFBOARD',manager='ACTIVE',quality=80,stamp=None):
            self.f.feed(t,vx=-.03,quality=quality)
            now=rospy.Time.now();self.n.odom.header.stamp=now if stamp is None else rospy.Time.from_sec(stamp)
            self.n._on_state(State(connected=True,armed=True,mode=mode))
            self.n.manager_phase=manager;self.n.manager_phase_received_at=now
            self.n._on_horizontal_output_owner(UInt8(data=owner))
            if output:
                q=self.q if not offset else (0.,math.sin(math.radians(offset)/2),0.,math.cos(math.radians(offset)/2))
                msg=AttitudeTarget(orientation=Quaternion(*q));msg.header.stamp=now
                self.n._on_horizontal_output(msg)
            self.q=self.n._horizontal_attitude(now,self.n.q_hover)
            return self.n.horizontal_controller.integral

        def test_duplicate_or_out_of_order_velocity_never_integrates_twice(self):
            before=self.n.horizontal_controller.integral
            self.assertEqual(self.tick(10.60,stamp=10.58),before)
            self.assertIn('重复或倒序',self.n.horizontal_integral_reason)
            self.assertEqual(self.tick(10.62,stamp=10.57),before)
            self.assertGreater(self.tick(10.64)[0],before[0])

        def test_output_gap_wrong_owner_and_mode_exit_clear_integral(self):
            for index,kwargs in enumerate(({'output':False},{'owner':1},{'mode':'POSCTL'},{'manager':'ENTERING'})):
                self.n.horizontal_controller.integral=(.1,0)
                self.n.horizontal_output_at=rospy.Time.from_sec(9.)
                self.assertEqual(self.tick(10.60+index*.02,**kwargs),(0.,0.))

        def test_output_mismatch_holds_and_resumes_without_catchup(self):
            before=self.n.horizontal_controller.integral
            self.assertEqual(self.tick(10.60,offset=3.),before)
            self.assertIn('超过1度',self.n.horizontal_integral_reason)
            self.assertEqual(self.tick(10.62),before)
            after=self.tick(10.64)
            self.assertGreater(after[0],before[0]);self.assertLess(after[0]-before[0],.001)

        def test_pause_flow_loss_and_flip_abort_recovery_clear_integral(self):
            self.n._auto_approach_ranges_ready=lambda now:False
            self.assertEqual(self.tick(10.60),(0.,0.))
            self.assertLessEqual(math.hypot(*self.n.horizontal_controller.target),.0081)
            self.n._auto_approach_ranges_ready=lambda now:True
            self.n.horizontal_controller.integral=(.1,0)
            self.tick(10.62,quality=0)
            self.assertEqual(self.n.horizontal_controller.integral,(0.,0.))
            for phase in (W.STATE_FLIP_TO_WALL,W.STATE_ABORT,W.STATE_RECOVER,W.STATE_DETACH_ROTATE,W.STATE_IDLE):
                self.n.horizontal_controller.integral=(.1,0)
                self.n._enter_state(phase,rospy.Time.now(),'PI lifecycle test')
                self.assertEqual(self.n.horizontal_controller.integral,(0.,0.))

        def test_expired_flow_and_velocity_source_clear_integral(self):
            self.tick(10.60,stamp=9.)
            self.assertEqual(self.n.horizontal_controller.integral,(0.,0.))
            self.assertIn('速度源时间戳',self.n.horizontal_integral_reason)
            self.n.horizontal_controller.integral=(.1,0)
            self.f.time=11.3
            self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
            self.assertEqual(self.n.horizontal_controller.integral,(0.,0.))

        def test_final_output_cone_limit_is_forwarded_to_antiwindup(self):
            self.n.horizontal_equilibrium.axis=(math.sin(math.radians(4)),0,math.cos(math.radians(4)))
            self.n.horizontal_controller.clear_integral()
            self.q=attitude_from_axis_yaw(self.n.horizontal_equilibrium.axis,0)
            self.n.horizontal_candidate_axis=self.n.horizontal_equilibrium.axis
            self.tick(10.60,offset=5.)
            self.tick(10.62,offset=5.)
            self.assertEqual(self.n.horizontal_controller.integral,(0.,0.))
            self.assertTrue(self.n.horizontal_controller.saturated)

        def test_status_exposes_integral_and_forward_measurement(self):
            status=W();status.header.stamp=rospy.Time.now()
            self.n._fill_horizontal_status(status)
            self.assertGreater(status.horizontal_integral_x_enu_mps2,0)
            self.assertAlmostEqual(status.horizontal_forward_measured_mps,-.03)
            self.assertGreater(status.horizontal_forward_target_mps,0)
            self.assertTrue(status.horizontal_integral_reason)

if __name__=='__main__':unittest.main()
