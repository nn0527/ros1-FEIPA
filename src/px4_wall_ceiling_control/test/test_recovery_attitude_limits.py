import math
import unittest
import random
from unittest import mock
import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import State, AttitudeTarget
from px4_wall_ceiling_control.msg import AttachmentControlCandidate, WallPerchStatus
from px4_wall_ceiling_control.domain.geometry import rotate_vector
from px4_wall_ceiling_control.domain.recovery_attitude_limits import (
    axis_yaw, attitude_from_axis_yaw, limit_recovery_attitude)
from test_flight_handover import OutputMux


def q(tilt=0, direction=0, yaw=0):
    t, d, y = map(math.radians, (tilt,direction,yaw))
    return attitude_from_axis_yaw((math.sin(t)*math.cos(d), math.sin(t)*math.sin(d), math.cos(t)),y)


def tilt(value):
    return math.degrees(math.acos(max(-1,min(1,axis_yaw(value)[0][2]))))

class GeometryLimitsTest(unittest.TestCase):
    def limit(self, desired, previous=None, dt=.02):
        return limit_recovery_attitude(desired, previous, dt, math.radians(5), math.radians(10), math.radians(20))

    def test_combined_tilt_is_capped_and_heading_preserved(self):
        final,_=self.limit(q(7,45,30))
        self.assertAlmostEqual(tilt(final),5)
        self.assertAlmostEqual(math.degrees(axis_yaw(final)[1]),30)

    def test_thrust_axis_and_yaw_have_separate_slew_budgets(self):
        final,_=self.limit(q(5,0,90),q())
        self.assertAlmostEqual(tilt(final),.2,places=6)
        self.assertAlmostEqual(math.degrees(axis_yaw(final)[1]),.4)

    def test_heading_wrap_uses_shortest_direction(self):
        final,_=self.limit(q(0,0,-179),q(0,0,179))
        self.assertAlmostEqual(math.degrees(axis_yaw(final)[1]),179.4)

    def test_tilt_direction_changes_are_also_slew_limited(self):
        final,_=self.limit(q(5,180),q(5,0))
        a=axis_yaw(final)[0];b=axis_yaw(q(5,0))[0]
        self.assertLessEqual(math.acos(max(-1,min(1,sum(x*y for x,y in zip(a,b))))),math.radians(.2)+1e-8)

    def test_entry_outside_cone_is_explicitly_projected(self):
        final,projected=self.limit(q(),q(80,20,10))
        self.assertTrue(projected)
        self.assertLessEqual(tilt(final),5+1e-8)

    def test_zero_elapsed_time_does_not_move_in_cone_target(self):
        final,_=self.limit(q(5,80,90),q(2,30,10),0)
        self.assertAlmostEqual(tilt(final),2)
        self.assertAlmostEqual(math.degrees(axis_yaw(final)[1]),10)

    def test_random_sequences_stay_in_cone_and_slew(self):
        rng=random.Random(42);previous=q()
        for _ in range(300):
            final,projected=self.limit(q(rng.uniform(0,40),rng.uniform(-180,180),rng.uniform(-180,180)),previous)
            self.assertFalse(projected)
            self.assertLessEqual(tilt(final),5+1e-7)
            a,ya=axis_yaw(previous);b,yb=axis_yaw(final)
            self.assertLessEqual(math.acos(max(-1,min(1,sum(x*y for x,y in zip(a,b))))),math.radians(.2)+1e-7)
            self.assertLessEqual(abs(math.atan2(math.sin(yb-ya),math.cos(yb-ya))),math.radians(.4)+1e-7)
            previous=final

class FinalTargetLimitsTest(unittest.TestCase):
    def node(self):
        n=OutputMux.__new__(OutputMux)
        n.recovery_attitude_limits_enabled=True
        n.recovery_max_tilt_deg=5;n.recovery_tilt_slew_deg_s=10;n.recovery_yaw_slew_deg_s=20
        n.recovery_final_thrust_cap=.6
        n.recovery_previous_target=None;n.recovery_previous_at=None
        n.wall_status=WallPerchStatus(state=WallPerchStatus.STATE_RECOVER)
        n.wall_status_at=n.flight_state_at=rospy.Time.from_sec(10)
        n.status_timeout_s=.3;n.state_timeout_s=1
        n.flight_state=State(mode="OFFBOARD",connected=True)
        n.recovery_target_publisher=mock.Mock();n.recovery_horizontal_publisher=mock.Mock()
        return n

    def target(self):
        x,y,z,w=q(12,45,30)
        return AttitudeTarget(orientation=Quaternion(x=x,y=y,z=z,w=w),thrust=.85)

    def test_recover_abort_final_thrust_and_horizontal_proxy(self):
        for state in (WallPerchStatus.STATE_RECOVER,WallPerchStatus.STATE_ABORT):
            n=self.node();n.wall_status.state=state
            t=n._limit_final_recovery_target(rospy.Time.from_sec(10),self.target(),AttachmentControlCandidate.SOURCE_WALL)
            self.assertAlmostEqual(t.thrust,.6,places=6)
            self.assertAlmostEqual(tilt(tuple(getattr(t.orientation,k) for k in "xyzw")),5)
            value=n.recovery_horizontal_publisher.publish.call_args[0][0].data
            self.assertLessEqual(value,.6*math.sin(math.radians(5))+1e-7)

    def test_protected_states_are_unchanged(self):
        for state in (WallPerchStatus.STATE_FLIP_TO_WALL,WallPerchStatus.STATE_WALL_CAPTURE,
                      WallPerchStatus.STATE_WALL_HOLD,WallPerchStatus.STATE_DETACH_ROTATE,WallPerchStatus.STATE_EXIT):
            n=self.node();n.wall_status.state=state;t=self.target();before=t.orientation
            result=n._limit_final_recovery_target(rospy.Time.from_sec(10),t,AttachmentControlCandidate.SOURCE_WALL)
            self.assertEqual(result.orientation,before);self.assertAlmostEqual(result.thrust,.85,places=6)
            n.recovery_target_publisher.publish.assert_not_called()

    def test_altctl_and_ceiling_outputs_are_unchanged(self):
        for mode,source in [("ALTCTL",AttachmentControlCandidate.SOURCE_WALL),("OFFBOARD",AttachmentControlCandidate.SOURCE_CEILING)]:
            n=self.node();n.flight_state.mode=mode
            t=n._limit_final_recovery_target(rospy.Time.from_sec(10),self.target(),source)
            self.assertAlmostEqual(t.thrust,.85,places=6)

    def test_recover_to_abort_keeps_slew_history(self):
        n=self.node();n.recovery_previous_target=q();n.recovery_previous_at=rospy.Time.from_sec(9.98)
        t=n._limit_final_recovery_target(rospy.Time.from_sec(10),self.target(),AttachmentControlCandidate.SOURCE_WALL)
        n.wall_status.state=WallPerchStatus.STATE_ABORT
        t=n._limit_final_recovery_target(rospy.Time.from_sec(10.02),self.target(),AttachmentControlCandidate.SOURCE_WALL)
        self.assertLessEqual(tilt(tuple(getattr(t.orientation,k) for k in "xyzw")),.4+1e-6)

if __name__=="__main__": unittest.main()
