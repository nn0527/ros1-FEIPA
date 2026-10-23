"""A-version: parallel level confirmation and bounded healthy soft reentry."""
import math
import unittest
from unittest import mock

import rospy
from std_msgs.msg import UInt8
from px4_wall_ceiling_control.msg import WallPerchStatus as W
import test_recovery_continuous_control as continuous


class RecoveryAResponseTests(unittest.TestCase):
    setUp = continuous.RecoveryContinuousFeedbackTests.setUp
    tick = continuous.RecoveryContinuousFeedbackTests.tick
    warm = continuous.RecoveryContinuousFeedbackTests.warm

    def test_reset_without_horizontal_initialization_remains_safe(self):
        from test_attachment_safety import WallDecision
        node = WallDecision.__new__(WallDecision)
        node._reset_recovery_horizontal('initialization/disabled', clear_departure=True)
        self.assertAlmostEqual(node.recovery_blend_duration_s,.30)
        self.assertFalse(node.recovery_blend_established)
        self.assertFalse(node.recovery_soft_reentry)
        self.assertFalse(node.departure_active)

    def test_level_hold_runs_while_feedback_is_being_confirmed(self):
        self.warm()
        self.tick(11.,tilt=11.)
        self.assertTrue(self.n.recovery_tilt_guard.paused)
        self.assertFalse(self.n.horizontal_velocity_ready)
        for i in range(16):
            self.tick(11.1+i*.02,tilt=4.)
        # Both existing 300ms confirmations are met in parallel. The output
        # still starts its full cold blend; no sensor fence has been relaxed.
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.assertFalse(self.n.recovery_tilt_guard.paused)
        self.assertTrue(self.n.horizontal_velocity_active)
        self.assertFalse(self.n.recovery_soft_reentry)
        self.assertAlmostEqual(self.n.recovery_blend_duration_s,.30)
        self.assertLess(self.n.recovery_horizontal_weight,.02)

    def test_soft_reentry_reaches_full_blend_faster_than_cold_reentry(self):
        self.warm()
        gate = self.n.horizontal_gate_at
        self.tick(11.,rate=60.)
        self.assertEqual(self.n.recovery_horizontal_weight,0.)
        for i in range(1,9):
            self.tick(11.+i*.02)
        self.assertEqual(self.n.horizontal_gate_at,gate)
        self.assertTrue(self.n.recovery_soft_reentry)
        self.assertAlmostEqual(self.n.recovery_horizontal_weight,1.)
        self.assertAlmostEqual(self.n.recovery_blend_duration_s,.15)

    def test_short_pause_before_cold_blend_established_still_uses_cold_path(self):
        self.tick(10.52)
        self.tick(10.54,rate=60.)
        self.tick(10.56)
        self.assertFalse(self.n.recovery_soft_reentry)
        self.assertAlmostEqual(self.n.recovery_blend_duration_s,.30)

    def test_long_soft_pause_does_not_receive_fast_reentry(self):
        self.warm()
        for i in range(11):
            self.tick(11.+i*.1,rate=60.)
        self.tick(12.02)
        self.assertFalse(self.n.recovery_soft_reentry)
        self.assertAlmostEqual(self.n.recovery_blend_duration_s,.30)
        self.assertLess(self.n.recovery_horizontal_weight,.02)

    def test_real_feedback_fault_discards_soft_reentry_entitlement(self):
        self.warm()
        self.tick(11.,rate=60.)
        self.tick(11.02,quality=0)
        self.assertFalse(self.n.recovery_blend_established)
        self.assertIsNone(self.n.recovery_soft_pause_at)
        for i in range(18):
            self.tick(11.04+i*.02)
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.assertFalse(self.n.recovery_soft_reentry)
        self.assertAlmostEqual(self.n.recovery_blend_duration_s,.30)

    def test_owner_loss_during_soft_pause_forces_full_reentry(self):
        self.warm()
        self.tick(11.,rate=60.)
        self.f.feed(11.02,rate=math.radians(60.))
        self.n._on_horizontal_output_owner(UInt8(data=0))
        self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
        self.assertFalse(self.n.recovery_blend_established)
        self.tick(11.04)
        self.assertFalse(self.n.recovery_soft_reentry)
        self.assertAlmostEqual(self.n.recovery_blend_duration_s,.30)

    def test_stale_actual_output_does_not_receive_fast_reentry(self):
        self.warm()
        self.tick(11.,rate=60.)
        self.f.feed(11.02)
        self.n.horizontal_output.header.stamp=rospy.Time.from_sec(10.)
        self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
        self.assertFalse(self.n.recovery_soft_reentry)
        self.assertFalse(self.n.recovery_blend_established)
        self.assertAlmostEqual(self.n.recovery_blend_duration_s,.30)

    def test_mode_loss_does_not_receive_fast_reentry(self):
        self.warm()
        self.tick(11.,rate=60.)
        self.n.flight_state.mode='POSCTL'
        self.tick(11.02)
        self.assertFalse(self.n.recovery_soft_reentry)
        self.assertFalse(self.n.recovery_blend_established)

    def test_invalid_flow_still_clears_parallel_attitude_confirmation(self):
        self.warm()
        self.tick(11.,tilt=11.)
        self.tick(11.1,tilt=4.)
        self.assertIsNotNone(self.n.recovery_tilt_guard.since_source)
        self.tick(11.2,tilt=4.,quality=0)
        self.assertIsNone(self.n.recovery_tilt_guard.since_source)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertFalse(self.n.horizontal_velocity_active)

    def test_delayed_soft_reentry_tick_is_capped_and_cannot_finish_at_once(self):
        self.warm()
        self.tick(11.,rate=60.)
        self.tick(11.5)
        self.assertTrue(self.n.recovery_soft_reentry)
        self.assertAlmostEqual(self.n.recovery_blend_elapsed,.05)
        self.assertLess(self.n.recovery_horizontal_weight,.3)

    def test_invalid_soft_pause_duration_is_rejected(self):
        from test_attachment_safety import WallDecision
        for value in (0.,float('nan'),2.01):
            with mock.patch.object(rospy,'get_param',side_effect=lambda key,default=None:
                    value if key=='~recovery_horizontal_soft_pause_max_s' else default), \
                    mock.patch.object(rospy,'Publisher'), mock.patch.object(rospy,'Subscriber'), \
                    mock.patch.object(rospy,'Timer'):
                with self.assertRaises(ValueError):
                    WallDecision()

    def test_duplicate_control_tick_cannot_accumulate_fast_reentry(self):
        self.warm()
        self.tick(11.,rate=60.)
        self.tick(11.02)
        elapsed = self.n.recovery_blend_elapsed
        weight = self.n.recovery_horizontal_weight
        for _ in range(10):
            self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
        self.assertEqual(self.n.recovery_blend_elapsed,elapsed)
        self.assertEqual(self.n.recovery_horizontal_weight,weight)

    def test_large_tilt_still_resets_and_cannot_fast_reenter(self):
        self.warm()
        self.tick(11.,tilt=11.)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertFalse(self.n.horizontal_velocity_active)
        self.assertFalse(self.n.recovery_blend_established)
        self.assertIsNone(self.n.recovery_soft_pause_at)

    def test_no_single_flow_frame_or_pending_level_latch_can_enable_output(self):
        self.warm()
        self.tick(11.,tilt=11.)
        self.tick(11.1,tilt=7.)
        for i in range(1,16):
            self.f.feed(11.1+i*.02,tilt=7.,flow=False)
            self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertFalse(self.n.horizontal_velocity_active)
        self.assertTrue(self.n.recovery_tilt_guard.paused)

    def test_approach_controller_and_acceleration_limits_are_unchanged(self):
        self.warm()
        self.n.state = W.STATE_SLOW_APPROACH
        self.tick(11.,vx=.8)
        self.assertTrue(self.n.horizontal_velocity_active)
        self.assertFalse(self.n.recovery_soft_reentry)
        self.assertAlmostEqual(self.n.horizontal_controller.max_acceleration,.30)
        self.assertAlmostEqual(self.n.recovery_horizontal_controller.max_acceleration,.30)
        self.assertAlmostEqual(self.n.horizontal_controller.kp,1.5)
        self.assertAlmostEqual(self.n.horizontal_controller.ki,.30)

    def test_invalid_soft_reentry_config_fails_closed(self):
        for value in (0.,float('nan'),.31):
            def param(key,default=None):
                if key == '~recovery_horizontal_soft_reentry_blend_s':
                    return value
                return default
            with mock.patch.object(rospy,'get_param',side_effect=param), \
                    mock.patch.object(rospy,'Publisher'), mock.patch.object(rospy,'Subscriber'), \
                    mock.patch.object(rospy,'Timer'):
                from test_attachment_safety import WallDecision
                with self.assertRaises(ValueError):
                    WallDecision()


if __name__ == '__main__':
    unittest.main()
