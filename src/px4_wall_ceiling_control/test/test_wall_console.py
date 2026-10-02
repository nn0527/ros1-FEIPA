#!/usr/bin/env python3
"""Action admission and live-state regression tests without hardware or ROS."""
import pathlib
import sys
import unittest
from unittest import mock
import xml.etree.ElementTree as ET
import importlib.util
import threading
import os
import subprocess
import tempfile
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))
from px4_wall_ceiling_control.wall_console import ConsoleState, DEFAULT_CONFIG, launch_arguments, validate_config

server_path = pathlib.Path(__file__).resolve().parents[1] / "scripts/operator/wall_console_server.py"
spec = importlib.util.spec_from_file_location("wall_console_server", str(server_path))
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class ConsoleTests(unittest.TestCase):
    def telemetry_runtime(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        runtime.ros = mock.Mock()
        return runtime

    def test_telemetry_configuration_rechecks_disarmed_after_service_wait(self):
        runtime = self.telemetry_runtime()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        runtime.ros.wait_for_service.side_effect = lambda *a, **k: self.state.update(
            "state", {"connected": True, "armed": True, "mode": "ALTCTL"})
        with mock.patch.dict(sys.modules, {"mavros_msgs.srv": SimpleNamespace(MessageInterval=object)}):
            with self.assertRaises(ValueError):
                runtime.configure_telemetry()
        runtime.ros.ServiceProxy.assert_not_called()

    def test_telemetry_configuration_requires_fcu_confirmation(self):
        runtime = self.telemetry_runtime()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        runtime.ros.ServiceProxy.return_value.return_value.success = False
        with mock.patch.dict(sys.modules, {"mavros_msgs.srv": SimpleNamespace(MessageInterval=object)}):
            with self.assertRaises(ValueError):
                runtime.configure_telemetry()
        runtime.ros.ServiceProxy.return_value.assert_called_once_with(message_id=230, message_rate=5.0)

    def test_bridge_keeps_catkin_generated_messages_importable(self):
        # Reproduce a sourced Catkin workspace in a fresh interpreter. Importing
        # the bridge must not shadow its relay with the plain source package.
        source = server_path.parents[2] / "src/px4_wall_ceiling_control"
        with tempfile.TemporaryDirectory() as directory:
            relay = pathlib.Path(directory) / "px4_wall_ceiling_control"
            (relay / "msg").mkdir(parents=True)
            (relay / "__init__.py").write_text(
                "__path__ = [" + repr(str(relay)) + ", " + repr(str(source)) + "]\n",
                encoding="utf-8")
            (relay / "msg/__init__.py").write_text("generated = True\n", encoding="utf-8")
            code = ("import runpy; runpy.run_path(" + repr(str(server_path)) + "); "
                    "from px4_wall_ceiling_control.msg import generated; assert generated")
            result = subprocess.run([sys.executable, "-c", code],
                env=dict(os.environ, PYTHONPATH=directory), capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

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
        self.state.launch_error = "上一次失败的启动错误"
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        runtime.ros = mock.Mock()
        runtime.ros.get_param.return_value = DEFAULT_CONFIG["fcu_port"] + ":" + str(DEFAULT_CONFIG["baud"])
        with mock.patch.object(server.threading, "Thread") as thread:
            reply = runtime.start(DEFAULT_CONFIG)
        self.assertIn("无需重复启动", reply["message"])
        self.assertEqual(self.state.launch_error, "")
        thread.assert_not_called()

    def test_armed_vehicle_blocks_new_pipeline_start(self):
        self.ready()
        self.state.update("runtime", {"enabled": False})
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        with mock.patch.object(server.threading, "Thread") as thread, self.assertRaisesRegex(ValueError, "未解锁"):
            runtime.start(DEFAULT_CONFIG)
        thread.assert_not_called()

    def test_lidar_startup_resets_both_measurements_only_while_disarmed(self):
        self.ready()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        factory = mock.MagicMock()
        yaml = SimpleNamespace(safe_load=lambda text: {
            name: {"input_format": "l1_ascii", "baudrate": 38400}
            for name in ("ceiling_lidar_node", "front_lidar_node")})
        with mock.patch.object(runtime, "port_owners", return_value=[]), \
             mock.patch.dict(sys.modules, {"serial": SimpleNamespace(Serial=factory), "yaml": yaml}), \
             mock.patch.object(server.time, "sleep"):
            runtime.prepare_lidars()
            self.assertEqual(factory.call_count, 2)
            self.assertEqual([call.args[0] for call in factory.call_args_list],
                             ["/dev/ttyUSB1", "/dev/ttyUSB2"])
            device = factory.return_value.__enter__.return_value
            self.assertEqual(device.write.call_args_list, [mock.call(b"iHALT"), mock.call(b"iHALT")])
            self.state.update("state", {"connected": True, "armed": True, "mode": "ALTCTL"})
            factory.reset_mock()
            with self.assertRaisesRegex(ValueError, "未解锁"):
                runtime.prepare_lidars()
            factory.assert_not_called()

    def test_orphan_serial_reader_blocks_startup_reset(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        with mock.patch.object(runtime, "port_owners", return_value=[1234]), \
             self.assertRaisesRegex(ValueError, "旧 roslaunch.*1234"):
            runtime.prepare_lidars()


if __name__ == "__main__":
    unittest.main()
