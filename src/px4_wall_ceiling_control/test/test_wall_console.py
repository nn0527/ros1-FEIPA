#!/usr/bin/env python3
"""Action admission and live-state regression tests without hardware or ROS."""
import pathlib
import sys
import unittest
from unittest import mock
import xml.etree.ElementTree as ET
import importlib.util
import threading
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))
from px4_wall_ceiling_control.wall_console import ConsoleState, DEFAULT_CONFIG, launch_arguments, validate_config

server_path = pathlib.Path(__file__).resolve().parents[1] / "scripts/operator/wall_console_server.py"
spec = importlib.util.spec_from_file_location("wall_console_server", str(server_path))
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class ConsoleTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.state = ConsoleState(clock=lambda: self.now)
        self.publisher = mock.Mock()

    def ready(self):
        for name, value in {"runtime": {"enabled": True}, "state": {"connected": True, "armed": True, "mode": "ALTCTL"},
               "health": {"ceiling_valid": True, "front_valid": True}, "estimator": {"attitude": True, "vertical_velocity": True},
               "odom": {"finite": True}, "phase": "IDLE", "owner": 0, "operator": {"mode": 1, "enabled": False}}.items():
            self.state.update(name, value)

    def test_ready_dispatches_exactly_one_wall(self):
        self.ready()
        self.state.send_wall(self.publisher)
        self.publisher.assert_called_once_with("wall")
        with self.assertRaises(ValueError):
            self.state.send_wall(self.publisher)
        self.publisher.assert_called_once()

    def test_stale_estimator_blocks_even_when_flag_remains_true(self):
        self.ready()
        self.now += 1.6
        for name in ("state", "runtime", "health", "odom", "phase", "owner", "operator"):
            self.state.update(name, self.state.samples[name]["value"])
        with self.assertRaisesRegex(ValueError, "估计器"):
            self.state.send_wall(self.publisher)
        self.publisher.assert_not_called()

    def test_disarmed_wrong_mode_invalid_sensor_and_existing_intent_block(self):
        mutations = [("state", {"connected": True, "armed": False, "mode": "ALTCTL"}),
                     ("state", {"connected": True, "armed": True, "mode": "OFFBOARD"}),
                     ("health", {"ceiling_valid": False, "front_valid": True}),
                     ("operator", {"mode": 3, "enabled": True}), ("runtime", {"enabled": False})]
        for name, value in mutations:
            with self.subTest(name=name):
                self.ready()
                self.state.update(name, value)
                with self.assertRaises(ValueError):
                    self.state.send_wall(self.publisher)
        self.publisher.assert_not_called()

    def test_sent_or_stale_hold_is_not_success(self):
        self.ready()
        self.state.send_wall(self.publisher)
        self.assertFalse(self.state.snapshot()["success"])
        self.state.update("wall", {"state_name": "WALL_HOLD", "reason": "contact confirmed"})
        self.assertFalse(self.state.snapshot()["success"])
        self.state.update("phase", "ACTIVE")
        self.state.update("owner", 2)
        self.state.update("state", {"connected": True, "armed": True, "mode": "OFFBOARD"})
        self.state.update("setpoint", {"thrust": 0.6})
        self.assertTrue(self.state.snapshot()["success"])
        self.now += 0.6
        self.assertFalse(self.state.snapshot()["success"])

    def test_no_rc_mapping_needed_and_shell_input_rejected(self):
        self.assertEqual(validate_config(DEFAULT_CONFIG), DEFAULT_CONFIG)
        for change in ({"fcu_port": "/dev/ttyUSB0; reboot"}, {"front_port": "/dev/ttyUSB1"},
                       {"hover_thrust": float("nan")}, {"baud": True}, {"flight_enabled": False}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_config(dict(DEFAULT_CONFIG, **change))

    def test_launch_uses_dedicated_entry_with_explicit_real_output(self):
        args = launch_arguments(DEFAULT_CONFIG)
        self.assertEqual(args[:3], ["roslaunch", "px4_wall_ceiling_control", "wall_console_flight.launch"])
        path = pathlib.Path(__file__).resolve().parents[1] / "launch/system/wall_console_flight.launch"
        values = {arg.attrib["name"]: arg.attrib["value"] for arg in ET.parse(str(path)).findall("./include/arg")}
        for key in ("flight_mode_enabled", "output_enabled", "wall_enabled"):
            self.assertEqual(values[key], "true")
        for key in ("rc_assist_enabled", "require_rc_center", "ceiling_enabled"):
            self.assertEqual(values[key], "false")
        self.assertEqual(values["wall_hold_time_s"], "0.0")

    def test_no_subscriber_cannot_be_reported_as_sent(self):
        self.ready()
        self.publisher.side_effect = ValueError("未订阅")
        with self.assertRaises(ValueError):
            self.state.send_wall(self.publisher)
        self.assertIsNone(self.state.pending_since)

    def test_transport_loss_clears_success_and_admission(self):
        self.ready()
        bridge = object.__new__(server.SSHTransport)
        bridge.lock = threading.RLock()
        bridge.snapshot = self.state.snapshot()
        bridge.snapshot["success"] = True
        bridge.arrived, bridge.connected, bridge.error = 100.0, True, ""
        bridge.args = SimpleNamespace(ssh_host="test-host", ssh_user="pi")
        with mock.patch.object(server.time, "monotonic", return_value=103.0):
            status = bridge.status(DEFAULT_CONFIG)
        self.assertFalse(status["success"])
        self.assertFalse(status["can_wall"])
        self.assertTrue(all(not row["ok"] for row in status["checks"]))

    def test_transport_age_invalidates_estimator_before_ssh_disconnect(self):
        self.ready()
        self.state.samples["estimator"]["at"] = 98.6
        bridge = object.__new__(server.SSHTransport)
        bridge.lock = threading.RLock()
        bridge.snapshot = self.state.snapshot()
        bridge.arrived, bridge.connected, bridge.error = 100.0, True, ""
        bridge.args = SimpleNamespace(ssh_host="test-host", ssh_user="pi")
        with mock.patch.object(server.time, "monotonic", return_value=100.2):
            status = bridge.status(DEFAULT_CONFIG)
        self.assertTrue(status["ssh"]["connected"])
        self.assertFalse(status["can_wall"])
        self.assertFalse(next(c["ok"] for c in status["checks"] if c["id"] == "estimator"))

    def test_running_pipeline_is_not_restarted(self):
        self.ready()
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        with mock.patch.object(server.threading, "Thread") as thread:
            reply = runtime.start(DEFAULT_CONFIG)
        self.assertIn("无需重复启动", reply["message"])
        thread.assert_not_called()

    def test_armed_vehicle_blocks_new_pipeline_start(self):
        self.ready()
        self.state.update("runtime", {"enabled": False})
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        with mock.patch.object(server.threading, "Thread") as thread, self.assertRaisesRegex(ValueError, "未解锁"):
            runtime.start(DEFAULT_CONFIG)
        thread.assert_not_called()


if __name__ == "__main__":
    unittest.main()
