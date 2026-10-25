#!/usr/bin/env python3
"""CH12 edge handling and mixed web/RC operator cycles; no flight hardware needed."""
import importlib.util
import pathlib
import threading
import unittest
from unittest import mock

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import RCIn, State
from px4_wall_ceiling_control.msg import OperatorCommand as Op, WallPerchStatus as Wall
from px4_wall_ceiling_control.domain.momentary_switch import MomentarySwitch

path = pathlib.Path(__file__).resolve().parents[1] / 'scripts/operator/terminal_operator_node.py'
spec = importlib.util.spec_from_file_location('ch12_terminal', str(path))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
Terminal = module.TerminalOperatorNode


class SwitchTest(unittest.TestCase):
    def test_startup_high_and_reconnect_require_release(self):
        s = MomentarySwitch()
        for t in [0., .1, .2]: self.assertFalse(s.update(1999, t))
        s.update(999, .3); self.assertFalse(s.update(999, .4))
        s.update(1999, .5); self.assertTrue(s.update(1999, .6))
        self.assertFalse(s.update(1999, .7))
        self.assertFalse(s.update(1999, 2.))
        self.assertFalse(s.update(1999, 2.1))

    def test_bounce_missing_and_backwards_time(self):
        s = MomentarySwitch()
        s.update(999, 0.); s.update(999, .1)
        s.update(1999, .2); s.update(999, .21)
        self.assertFalse(s.update(1999, .22))
        self.assertTrue(s.update(1999, .3))
        s.update(999, .4); s.update(999, .5)
        self.assertFalse(s.update(None, .6))
        s.update(1999, .7); self.assertFalse(s.update(1999, .8))
        s.update(999, 1.); s.update(999, 1.1)
        self.assertFalse(s.update(1999, .1))
        self.assertFalse(s.update(1999, .2))


class MixedOperatorTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.clock = mock.patch.object(rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(self.now))
        self.clock.start(); self.addCleanup(self.clock.stop)
        self.params = mock.patch.object(rospy, 'get_param', side_effect=lambda name, default=None: default)
        self.params.start(); self.addCleanup(self.params.stop)
        n = self.node = Terminal.__new__(Terminal)
        n.intent_lock = threading.RLock()
        n.mode = Op.MODE_STANDBY
        n.detaching = n.detach_pulse = n.was_offboard = False
        n.front_sensor_masked = False
        n.flight_mode = 'ALTCTL'
        n.flight_connected = n.flight_armed = True
        n.flight_at = rospy.Time.now()
        n.kill_guard_enabled = False
        n.rc_wall_toggle_enabled = True
        n.rc_wall_channel_index = 11
        n.wall_switch = MomentarySwitch()
        for attr in ['command_pub', 'ceiling_detach_pub', 'wall_detach_pub']:
            setattr(n, attr, mock.Mock())
        self.status(Wall.STATE_IDLE)

    def status(self, state, pending=False):
        w = Wall(state=state, detach_pending=pending)
        w.header.stamp = rospy.Time.now()
        self.node._on_wall_status(w)

    def press(self):
        n = self.node
        for pwm in [999, 999, 1999, 1999]:
            with mock.patch.object(module.time, 'monotonic', return_value=self.now):
                n._on_rc(RCIn(channels=[1500]*11+[pwm]))
            self.now += .1

    def test_rc_start_web_detach_and_next_cycle(self):
        n = self.node
        self.press(); self.assertEqual(n.mode, Op.MODE_WALL)
        n._on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        self.status(Wall.STATE_WALL_HOLD)
        n._on_command(String(data='detach')); n._on_timer(None)
        n.wall_detach_pub.publish.assert_called_once()
        self.status(Wall.STATE_DETACH_ROTATE)
        self.press(); self.assertEqual(n.wall_detach_pub.publish.call_count, 1)
        n._on_state(State(connected=True, armed=True, mode='ALTCTL'))
        self.status(Wall.STATE_IDLE)
        self.press(); self.assertEqual(n.mode, Op.MODE_WALL)

    def test_web_start_rc_detach_and_retry_rejected_request(self):
        n = self.node
        n._on_command(String(data='wall'))
        self.status(Wall.STATE_SLOW_APPROACH)
        self.press(); self.assertFalse(n.detach_pulse)
        self.status(Wall.STATE_WALL_HOLD)
        n.flight_at = rospy.Time.now()
        self.press(); self.assertTrue(n.detach_pulse)
        n._on_timer(None); n.wall_detach_pub.publish.assert_called_once()
        # Decision rejects the request, leaving HOLD and no pending request.
        self.status(Wall.STATE_WALL_HOLD)
        n.flight_at = rospy.Time.now()
        self.press(); n._on_timer(None)
        self.assertEqual(n.wall_detach_pub.publish.call_count, 2)

    def test_stale_disarmed_kill_and_other_action_block(self):
        n = self.node
        self.now += 1.; n._on_wall_toggle(); self.assertEqual(n.mode, Op.MODE_STANDBY)
        self.status(Wall.STATE_IDLE); n.flight_at = rospy.Time.now()
        n.flight_armed = False; n._on_wall_toggle(); self.assertEqual(n.mode, Op.MODE_STANDBY)
        n.flight_armed = True; n.kill_guard_enabled = True
        n._kill_report = lambda: {'valid':True,'active':True}
        n._on_wall_toggle(); self.assertEqual(n.mode, Op.MODE_STANDBY)
        n.kill_guard_enabled = False; n.mode = Op.MODE_CEILING
        n._on_wall_toggle(); self.assertEqual(n.mode, Op.MODE_CEILING)

    def test_pending_request_blocks_rc_duplicate(self):
        n = self.node
        n.mode = Op.MODE_WALL
        self.status(Wall.STATE_WALL_HOLD, pending=True)
        self.press(); self.assertFalse(n.detach_pulse)


if __name__ == '__main__':
    unittest.main()
