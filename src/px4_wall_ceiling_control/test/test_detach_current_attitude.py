#!/usr/bin/env python3
import importlib.util,pathlib,unittest,math
from unittest import mock
import rospy
from px4_wall_ceiling_control.msg import WallPerchStatus
path=pathlib.Path(__file__).resolve().parents[1]/'scripts/decisions/wall_perch_decision_node.py'
spec=importlib.util.spec_from_file_location('detach_current_attitude',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class DetachCurrentAttitudeTest(unittest.TestCase):
    def setUp(self):
        self.node=module.WallPerchDecisionNode.__new__(module.WallPerchDecisionNode)
        n=self.node;n.q_contact=(0,0,0,1);n.q_hold_reference=n.q_contact
        self.current=(0,0,math.sin(math.radians(9)),math.cos(math.radians(9)))
        n._current_quaternion=lambda:self.current
        n._hold_follow_quaternion=lambda:self.current
        n._height_feedback=lambda:(1.,0.)
        n._odom_fresh=lambda now:True
        n._rates_safe=lambda limit:True
        n.detach_reference_angle_rad=math.radians(10)
        n.detach_axis_error_rad=math.radians(5);n.detach_max_rate_rps=.15
        n.detach_max_abs_vertical_speed_mps=1.;n.bench_wait_for_ranges=False
        n.max_down_velocity_mps=.8
        n.detach_stable_hold_s=.5
        n.odom_twist_in_child_frame=True
        from types import SimpleNamespace
        n.odom=SimpleNamespace(twist=SimpleNamespace(twist=SimpleNamespace(angular=SimpleNamespace(x=0.,y=0.,z=0.))))

    def test_old_reference_difference_no_longer_blocks(self):
        with mock.patch.object(module,'vertical_speed_enu_mps',return_value=0.):
            self.assertTrue(self.node._detach_pose_valid())
            self.assertNotIn('参考姿态偏差',self.node._detach_block_reason(rospy.Time.from_sec(10)))

    def test_axis_rate_and_velocity_gates_remain(self):
        n=self.node
        n.detach_vertical_speed_check_enabled=True
        with mock.patch.object(module,'vertical_speed_enu_mps',return_value=0.):
            n._hold_follow_quaternion=lambda:n.q_contact
            self.assertFalse(n._detach_pose_valid())
            n._hold_follow_quaternion=lambda:self.current
            n._rates_safe=lambda limit:False
            self.assertFalse(n._detach_pose_valid())
            n._rates_safe=lambda limit:True
        with mock.patch.object(module,'vertical_speed_enu_mps',return_value=1.1):
            self.assertFalse(n._detach_pose_valid())

    def test_detach_captures_final_output_instead_of_stale_hold_reference(self):
        import test_detach_state_machine as departure
        f = departure.DetachStateMachineTests()
        n = f.node()
        f.output(n, 10., tilt=88.)
        f.enter(n, WallPerchStatus.STATE_DETACH_ROTATE, 10.)
        self.assertEqual(n.state, WallPerchStatus.STATE_DETACH_ROTATE)
        self.assertLess(module.quaternion_angle(n.q_detach_start, departure.quaternion(88.)), 1.e-7)
        self.assertEqual(n.detach_altitude_m,2.4)

if __name__=='__main__':unittest.main()
