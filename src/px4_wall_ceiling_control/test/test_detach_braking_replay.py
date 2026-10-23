"""Open-loop log64 command replay; recorded motion is NOT predicted new motion."""
import json
import math
from pathlib import Path
import unittest

import test_detach_state_machine as fixtures
from px4_wall_ceiling_control.domain.geometry import quaternion_angle, rotate_vector
from px4_wall_ceiling_control.msg import WallPerchStatus as W


def replay():
    samples = json.loads((Path(__file__).parent/'fixtures'/'detach_brake_log64.json').read_text())['samples']
    fx = fixtures.DetachStateMachineTests()
    node = fx.node(**{'~detach_brake_enabled': True})
    fx.enter(node, W.STATE_DETACH_ROTATE, 10.)
    node.q_hover = tuple(samples[-1]['sent_q'])
    node.q_detach_start = tuple(samples[0]['sent_q'])
    node.odom_twist_in_child_frame = True
    published = None
    result, trigger = [], None
    for sample in samples:
        now = 10.+sample['t']
        odom = fx.feedback(node, now)
        for key, value in zip('xyzw', sample['q']):
            setattr(odom.pose.pose.orientation, key, value)
        for key, value in zip('xyz', sample['body_rate']):
            setattr(odom.twist.twist.angular, key, value)
        # Before engagement use actual logged output/history. Afterwards feed
        # back the preceding new command, never the counterfactual old target.
        sent = sample['sent_q'] if published is None else published
        fx.output(node, now)
        for key, value in zip('xyzw', sent):
            setattr(node.horizontal_output.orientation, key, value)
        if published is None:
            node.horizontal_output.header.stamp = fixtures.at(10.+sample['sent_t'])
        node._advance_detach_progress(fixtures.at(now))
        if node.detach_brake_fault:
            raise AssertionError(node.detach_brake_fault)
        planner = node.detach_brake_planner
        if planner is None:
            continue
        if trigger is None:
            trigger = sample['t']
            if quaternion_angle(node.detach_brake_target, sent) > 1.e-7:
                raise AssertionError('engagement jumped from published target')
        published = node.detach_brake_target
        result.append(dict(t=sample['t'],
            recorded_tilt_deg=math.degrees(node.detach_brake_axis.angle(sample['q'])),
            recorded_rate_deg_s=math.degrees(node.detach_brake_axis.closing_rate(sample['q'],sample['body_rate'])),
            old_target_deg=math.degrees(node.detach_brake_axis.angle(sample['sent_q'])),
            new_target_deg=math.degrees(planner.angle),
            new_target_rate_deg_s=math.degrees(planner.rate),
            new_target_accel_deg_s2=math.degrees(planner.accel),
            replan_count=planner.replan_count, paused=node.detach_brake_paused,
            reason=node.detach_brake_reason))
    return trigger, result, node


class DetachBrakingReplayTests(unittest.TestCase):
    def test_log64_engages_and_retracts_acceleration_demand_with_bounded_commands(self):
        trigger, rows, node = replay()
        self.assertAlmostEqual(trigger, 2.550527, delta=.02)
        self.assertGreater(len(rows), 30)
        self.assertTrue(all(0. <= r['new_target_deg'] <= 90.1 for r in rows))
        self.assertLessEqual(max(abs(r['new_target_rate_deg_s']) for r in rows), 56.25001)
        self.assertLessEqual(max(abs(r['new_target_accel_deg_s2']) for r in rows), 180.00001)
        # The reference turns back toward the measured attitude, rather than
        # freezing a far-ahead command or continuing monotonically to zero.
        self.assertTrue(any(r['new_target_rate_deg_s'] > 5. for r in rows))
        near = min(rows, key=lambda r: abs(r['t']-3.5))
        self.assertGreater(near['new_target_deg'], near['old_target_deg']+5.)
        self.assertEqual(node.detach_brake_fault, '')


if __name__ == '__main__':
    unittest.main()
