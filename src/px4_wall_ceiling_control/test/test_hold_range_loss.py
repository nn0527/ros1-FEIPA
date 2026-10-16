#!/usr/bin/env python3
import importlib.util
import pathlib
import unittest
import rospy
from px4_wall_ceiling_control.msg import WallPerchStatus

path = pathlib.Path(__file__).resolve().parents[1] / 'scripts/decisions/wall_perch_decision_node.py'
spec = importlib.util.spec_from_file_location('wall_hold_loss', path)
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)

class HoldRangeLossTest(unittest.TestCase):
    def setUp(self):
        self.node = module.WallPerchDecisionNode.__new__(module.WallPerchDecisionNode)
        self.node.ignore_range_loss_during_hold = True
        self.node.bench_wait_for_ranges = False
        self.node.require_top_sensor_on_acquire = True
        self.node._bench_waiting = lambda: False
        self.node._sensor_health_fresh = lambda now: False
        self.node._top_fresh = lambda now: False
        self.node._front_required = lambda: False
        self.node._flight_ready = lambda now: True
        self.node._odom_fresh = lambda now: True
        self.node._rates_safe = lambda limit: True
        self.node._vertical_speed_safe = lambda: True
        self.node.capture_max_rate_rps = 4.5
        self.node.flip_max_rate_rps = 6.
        self.node.recover_max_rate_rps = 3.
        self.node.max_rate_rps = 3.
        self.now = rospy.Time.from_sec(10)

    def test_loss_ignored_only_in_established_hold(self):
        for state in (WallPerchStatus.STATE_WALL_HOLD, WallPerchStatus.STATE_WALL_PIN):
            self.node.state = state
            self.assertTrue(self.node._safety_ok(self.now))
        for state in (WallPerchStatus.STATE_FLIP_TO_WALL, WallPerchStatus.STATE_WALL_CAPTURE):
            self.node.state = state
            self.assertTrue(self.node._safety_ok(self.now))
        for state in (WallPerchStatus.STATE_DETACH_ROTATE, WallPerchStatus.STATE_RECOVER):
            self.node.state = state
            self.assertFalse(self.node._safety_ok(self.now))

    def test_other_safety_checks_are_retained(self):
        self.node.state = WallPerchStatus.STATE_WALL_HOLD
        for name in ('_flight_ready', '_odom_fresh', '_rates_safe', '_vertical_speed_safe'):
            original = getattr(self.node, name)
            setattr(self.node, name, lambda *args: False)
            self.assertFalse(self.node._safety_ok(self.now), name)
            setattr(self.node, name, original)

    def test_disable_parameter_restores_loss_protection(self):
        self.node.state = WallPerchStatus.STATE_WALL_HOLD
        self.node.ignore_range_loss_during_hold = False
        self.assertFalse(self.node._safety_ok(self.now))

    def test_real_contact_loss_still_enters_abort(self):
        self.node.state = WallPerchStatus.STATE_WALL_HOLD
        self.node.top_filtered_m = .12; self.node.top_clear_distance_m = .10
        self.node.cancel_requested = False; self.node.fault_count = 0
        self.node._mode_requested = lambda now: True
        entered = []
        self.node._enter_state = lambda state, now, reason: entered.append((state, reason))
        self.node.top_clear_hold_s = .5
        self.node.top_clear_since = rospy.Time.from_sec(9.5)
        self.node.top_clear_last_sample_at = self.now
        self.node._top_fresh = lambda now: True
        self.node._update_state_machine(self.now)
        self.assertEqual(entered, [(WallPerchStatus.STATE_ABORT, 'wall contact or safety lost')])

if __name__ == '__main__':
    unittest.main()
