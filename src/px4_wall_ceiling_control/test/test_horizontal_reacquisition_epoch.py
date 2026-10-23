"""Independent attitude and fusion fences must not add a second sensor cycle."""
import math
import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from px4_wall_ceiling_control.msg import WallPerchStatus as W
from test_wall_fusion_handover import feedback_at, status_at
import test_wall_horizontal_velocity as horizontal_fixtures


class HorizontalReacquisitionEpochTests(unittest.TestCase):
    def setUp(self):
        self.fixture = horizontal_fixtures.HorizontalFlightTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.n = self.fixture.n
        self.n.wall_sensor_fusion_enabled = True
        self.n.wall_fusion_epoch = None
        self.n.state = W.STATE_ABORT

    def tick(self, t, *, odom=True, estimator=False, flow=False, tilt=0.,
             rate=0., ready=10., episode='wall-1', pending=False,
             estimator_source=None, flow_source=None, const=False):
        self.fixture.time = t
        now = rospy.Time.from_sec(t)
        self.n._on_wall_fusion_status(status_at(t, ready, episode, pending))
        o, e, f = feedback_at(t)
        o.pose.pose.orientation.y = math.sin(math.radians(tilt) / 2.)
        o.pose.pose.orientation.w = math.cos(math.radians(tilt) / 2.)
        o.twist.twist.angular.y = math.radians(rate)
        if odom:
            self.n._on_odom(o)
        if estimator:
            if estimator_source is not None:
                e.header.stamp = rospy.Time.from_sec(estimator_source)
            e.const_pos_mode_status_flag = const
            self.n._on_horizontal_estimator(e)
        if flow:
            if flow_source is not None:
                f.header.stamp = rospy.Time.from_sec(flow_source)
            self.n._on_horizontal_flow(f)
        self.n._prepare_horizontal_velocity(now)

    def establish(self):
        self.tick(10.01)
        self.tick(10.04, estimator=True)
        self.tick(10.15, flow=True)
        self.tick(10.38, flow=True)
        self.tick(10.46)
        self.assertTrue(self.n.horizontal_velocity_ready)

    def test_estimator_before_first_flow_does_not_need_another_estimator(self):
        self.tick(10.01)
        gate = self.n.horizontal_gate_at
        self.assertEqual(gate, rospy.Time.from_sec(10.01))
        self.tick(10.04, estimator=True)
        self.assertEqual(self.n.horizontal_gate_at, gate)
        self.tick(10.15, flow=True)
        self.assertEqual(self.n.horizontal_gate_at, gate)
        self.assertEqual(self.n.horizontal_flow_samples, 1)
        self.assertIn('连续有效确认', self.n.horizontal_velocity_reason)
        self.tick(10.38, flow=True)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.tick(10.46)
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.assertEqual(self.n.horizontal_estimator_at, rospy.Time.from_sec(10.04))
        self.n._horizontal_attitude(rospy.Time.now(), self.n.q_hover)
        self.assertTrue(self.n.horizontal_velocity_active)

    def test_single_flow_frame_never_completes_confirmation(self):
        self.tick(10.01)
        self.tick(10.04, estimator=True)
        self.tick(10.15, flow=True)
        self.tick(10.46)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertEqual(self.n.horizontal_flow_samples, 1)

    def test_post_ready_complete_flow_and_new_source_are_still_required(self):
        self.tick(10.005)
        gate = self.n.horizontal_gate_at
        self.tick(10.01, estimator=True, flow=True)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertIn('完整采集', self.n.horizontal_velocity_reason)
        self.assertEqual(self.n.horizontal_gate_at, gate)
        self.tick(10.1, estimator=True, flow=True, estimator_source=9.9)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertIn('新里程计', self.n.horizontal_velocity_reason)
        self.assertEqual(self.n.horizontal_gate_at, gate)
        self.tick(10.2, estimator=True, flow=True)
        self.tick(10.4, flow=True)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.tick(10.51)
        self.assertTrue(self.n.horizontal_velocity_ready)

    def assert_pose_fault_restarts_epoch(self, **fault):
        self.establish()
        self.tick(10.5, estimator=True, flow=True, **fault)
        self.assertIsNone(self.n.horizontal_gate_at)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertEqual(self.n.horizontal_flow_samples, 0)
        self.tick(10.6)
        gate = self.n.horizontal_gate_at
        self.assertEqual(gate, rospy.Time.from_sec(10.6))
        self.assertIn('恢复水平后的新光流', self.n.horizontal_velocity_reason)
        self.tick(10.65, flow=True)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertEqual(self.n.horizontal_flow_samples, 0)
        self.tick(10.7, estimator=True)
        self.tick(10.9, flow=True)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.tick(10.96)
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.assertEqual(self.n.horizontal_gate_at, gate)

    def test_tilt_fault_restarts_epoch_and_requires_post_pose_feedback(self):
        self.assert_pose_fault_restarts_epoch(tilt=11.)

    def test_rate_only_suspension_preserves_an_established_healthy_epoch(self):
        self.establish()
        gate = self.n.horizontal_gate_at
        self.tick(10.5,estimator=True,flow=True,tilt=3.,rate=46.)
        self.assertEqual(self.n.horizontal_gate_at,gate)
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.assertEqual(self.n.recovery_horizontal_rate_weight,0.)
        self.assertFalse(self.n.horizontal_velocity_stable)
        self.tick(10.6,estimator=True,flow=True)
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.assertEqual(self.n.horizontal_gate_at,gate)

    def test_pending_parameters_and_new_epoch_discard_previous_history(self):
        self.establish()
        self.tick(10.5, pending=True)
        self.assertIsNone(self.n.horizontal_gate_at)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.tick(10.6, ready=10.55, episode='wall-2')
        self.assertEqual(self.n.horizontal_gate_at, rospy.Time.from_sec(10.6))
        self.assertEqual(self.n.horizontal_flow_samples, 0)
        self.tick(10.65, ready=10.55, episode='wall-2', estimator=True)
        self.tick(10.7, ready=10.55, episode='wall-2', flow=True)
        self.tick(10.9, ready=10.55, episode='wall-2', flow=True)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.tick(11.01, ready=10.55, episode='wall-2')
        self.assertTrue(self.n.horizontal_velocity_ready)

    def test_invalid_estimator_and_stale_receipts_keep_loop_disabled(self):
        self.establish()
        self.tick(10.5, estimator=True, flow=True, const=True)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertIn('恒位置', self.n.horizontal_velocity_reason)
        self.tick(10.6, estimator=True, flow=True)
        self.tick(10.8, flow=True)
        self.tick(10.91)
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.tick(11.7)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertFalse(self.n.horizontal_velocity_active)

    def test_epoch_change_during_sample_check_never_mixes_confirmation(self):
        self.tick(10.01)
        self.tick(10.04, estimator=True)
        original = self.n._wall_fusion_samples_ready

        def changed_epoch(now):
            self.n._on_wall_fusion_status(status_at(10.15, ready=10.1, episode='wall-2'))
            return original(now)

        with mock.patch.object(self.n, '_wall_fusion_samples_ready', side_effect=changed_epoch):
            self.tick(10.15, estimator=True, flow=True)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertIsNone(self.n.horizontal_gate_at)
        self.assertEqual(self.n.horizontal_flow_samples, 0)


if __name__ == '__main__':
    unittest.main()
