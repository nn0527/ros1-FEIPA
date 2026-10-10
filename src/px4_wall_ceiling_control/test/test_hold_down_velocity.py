#!/usr/bin/env python3
import importlib.util,pathlib,unittest,math
from unittest import mock
from px4_wall_ceiling_control.msg import WallPerchStatus
path=pathlib.Path(__file__).resolve().parents[1]/'scripts/decisions/wall_perch_decision_node.py'
spec=importlib.util.spec_from_file_location('hold_down_velocity',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class HoldDownVelocityTest(unittest.TestCase):
    def test_hold_bypass_and_other_phases(self):
        node=module.WallPerchDecisionNode.__new__(module.WallPerchDecisionNode)
        node.odom=object();node.bench_wait_for_ranges=False
        node.max_down_velocity_mps=.8;node.ignore_down_velocity_during_hold=True
        with mock.patch.object(module,'vertical_speed_enu_mps',return_value=-.8213):
            for state in (WallPerchStatus.STATE_WALL_HOLD,WallPerchStatus.STATE_WALL_PIN):
                node.state=state;self.assertTrue(node._vertical_speed_safe())
            for state in (WallPerchStatus.STATE_FLIP_TO_WALL,WallPerchStatus.STATE_WALL_CAPTURE,WallPerchStatus.STATE_DETACH_ROTATE,WallPerchStatus.STATE_RECOVER):
                node.state=state;self.assertFalse(node._vertical_speed_safe())
            node.state=WallPerchStatus.STATE_WALL_HOLD
            node.ignore_down_velocity_during_hold=False
            self.assertFalse(node._vertical_speed_safe())
        node.ignore_down_velocity_during_hold=True
        with mock.patch.object(module,'vertical_speed_enu_mps',return_value=math.nan):
            self.assertFalse(node._vertical_speed_safe())

if __name__=='__main__':unittest.main()
