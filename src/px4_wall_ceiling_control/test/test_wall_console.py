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
    def test_password_auth_consumes_environment_without_persisting(self):
        args = SimpleNamespace(ssh_auth="password")
        with mock.patch.dict(os.environ, {"WALL_CONSOLE_SSH_PASSWORD": "fixture-secret"}), mock.patch.object(server.threading, "Thread"):
            transport = server.SSHTransport(args)
            self.assertEqual(transport.password, "fixture-secret")
            self.assertNotIn("WALL_CONSOLE_SSH_PASSWORD", os.environ)
            self.assertNotIn("password", transport.snapshot)

    def test_key_auth_does_not_use_password(self):
        with mock.patch.object(server.threading, "Thread"):
            transport = server.SSHTransport(SimpleNamespace(ssh_auth="key"))
            self.assertIsNone(transport.password)

    def telemetry_runtime(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        runtime.ros = mock.Mock()
        runtime.telemetry_lock = threading.Lock()
        runtime.telemetry_attempt_at = -float("inf")
        runtime.telemetry_connected = False
        return runtime

    def test_reconnect_restores_telemetry_without_start_button(self):
        runtime = self.telemetry_runtime()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        self.state.update("estimator", {"attitude": True, "vertical_velocity": True})
        runtime.configure_telemetry = mock.Mock()
        runtime.maintain_telemetry()
        runtime.configure_telemetry.assert_called_once()
        runtime.configure_telemetry.reset_mock()
        runtime.maintain_telemetry()
        runtime.configure_telemetry.assert_not_called()
        self.state.update("state", {"connected": False, "armed": False, "mode": "ALTCTL"})
        runtime.maintain_telemetry()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        runtime.maintain_telemetry()
        runtime.configure_telemetry.assert_called_once()

    def test_stale_stream_retries_disarmed_with_backoff(self):
        runtime = self.telemetry_runtime()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        runtime.telemetry_connected = True
        runtime.configure_telemetry = mock.Mock()
        with mock.patch.object(server.time, "monotonic", return_value=100):
            runtime.telemetry_attempt_at = 95
            runtime.maintain_telemetry()
            runtime.configure_telemetry.assert_not_called()
            runtime.telemetry_attempt_at = 89
            runtime.maintain_telemetry()
            runtime.configure_telemetry.assert_called_once()

    def test_telemetry_maintenance_never_configures_armed_or_stale_state(self):
        runtime = self.telemetry_runtime()
        runtime.configure_telemetry = mock.Mock()
        self.state.update("state", {"connected": True, "armed": True, "mode": "ALTCTL"})
        runtime.maintain_telemetry()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        self.now += 2
        runtime.maintain_telemetry()
        runtime.configure_telemetry.assert_not_called()

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
               "odom": {"finite": True}, "wall": {"state_name": "IDLE", "reason": "initialized",
                   "altitude_reference_valid": True, "height_above_takeoff_m": 1.2,
                   "min_entry_height_m": 0.3, "entry_height_ready": True},
               "phase": "IDLE", "owner": 0, "operator": {"mode": 1, "enabled": False}}.items():
            self.state.update(name, value)
        self.state.update("kill", {"valid": True, "active": False, "channel": 7})

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
        for name in ("state", "runtime", "health", "odom", "wall", "phase", "owner", "operator", "kill"):
            self.state.update(name, self.state.samples[name]["value"])
        with self.assertRaisesRegex(ValueError, "估计器"):
            self.state.send_wall(self.publisher)
        self.publisher.assert_not_called()

    def test_height_and_sensor_failures_are_visible_and_block_dispatch(self):
        self.ready()
        self.state.update("wall", dict(self.state.samples["wall"]["value"],
                                      height_above_takeoff_m=0.2, entry_height_ready=False))
        with self.assertRaisesRegex(ValueError, "相对起飞高度"):
            self.state.send_wall(self.publisher)
        self.ready()
        self.state.update("health", {"front_valid": False, "ceiling_valid": True})
        self.state.update("front_diagnostic", {"valid": False, "error": "L1 error 255 (weak reflection)"})
        with self.assertRaisesRegex(ValueError, "前向雷达.*E255"):
            self.state.send_wall(self.publisher)
        self.publisher.assert_not_called()

    def test_restrained_bench_ignores_entry_height_and_sensor_not_flight_feedback(self):
        self.ready()
        self.state.config["test_profile"] = "restrained_bench"
        self.state.update("health", {"front_valid": False, "ceiling_valid": False})
        self.state.update("wall", dict(self.state.samples["wall"]["value"], entry_height_ready=False))
        self.assertTrue(self.state.snapshot()["can_wall"])
        self.state.update("state", {"connected": False, "armed": False, "mode": ""})
        self.assertFalse(self.state.snapshot()["can_wall"])
        self.assertIn("restrained_bench:=true", launch_arguments(self.state.config))

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

    def holding(self):
        self.ready()
        self.state.update("state", {"connected": True, "armed": True, "mode": "OFFBOARD"})
        self.state.update("phase", "ACTIVE")
        self.state.update("owner", 2)
        self.state.update("operator", {"mode": 3, "enabled": True})
        self.state.update("setpoint", {"thrust": 0.6})
        self.state.update("wall", dict(self.state.samples["wall"]["value"],
            state_name="WALL_HOLD", hold_achieved=True, reached_state_name="WALL_HOLD",
            detach_ready=True, detach_completed=False, fault_detected=False))

    def test_detach_dispatches_once_only_in_current_ready_hold(self):
        self.holding()
        self.assertTrue(self.state.snapshot()["can_detach"])
        self.state.send_detach(self.publisher)
        self.publisher.assert_called_once_with("detach")
        with self.assertRaises(ValueError):
            self.state.send_detach(self.publisher)
        self.state.update("wall", dict(self.state.samples["wall"]["value"], state_name="DETACH_ROTATE"))
        self.now += 6
        self.assertFalse(self.state.snapshot()["can_detach"])

    def test_historical_success_never_authorizes_detach_after_abort(self):
        self.holding()
        self.state.update("wall", dict(self.state.samples["wall"]["value"], state_name="ABORT", fault_detected=True))
        result = self.state.snapshot()
        self.assertTrue(result["status"]["wall"]["value"]["hold_achieved"])
        self.assertFalse(result["success"])
        with self.assertRaises(ValueError):
            self.state.send_detach(self.publisher)
        self.publisher.assert_not_called()

    def test_detach_rejects_unsettled_pose_stale_sensor_and_lost_ssh(self):
        self.holding()
        self.state.update("wall", dict(self.state.samples["wall"]["value"], detach_ready=False))
        self.assertTrue(self.state.snapshot()["can_detach"])
        self.holding()
        self.state.samples["health"]["at"] = 99.0
        self.assertFalse(self.state.snapshot()["can_detach"])
        self.holding()
        bridge = object.__new__(server.SSHTransport)
        bridge.lock = threading.RLock()
        bridge.snapshot = self.state.snapshot()
        bridge.arrived, bridge.connected, bridge.error = 100.0, True, ""
        bridge.args = SimpleNamespace(ssh_host="test-host", ssh_user="pi")
        with mock.patch.object(server.time, "monotonic", return_value=103.0):
            result = bridge.status(DEFAULT_CONFIG)
        self.assertFalse(result["can_detach"])
        self.assertTrue(result["status"]["wall"]["value"]["hold_achieved"])

    def test_failed_detach_publish_is_retryable(self):
        self.holding()
        self.publisher.side_effect = ValueError("未订阅")
        with self.assertRaises(ValueError):
            self.state.send_detach(self.publisher)
        self.assertIsNone(self.state.detach_pending_since)

    def test_kill_switch_blocks_detach_before_armed_flag_catches_up(self):
        self.holding()
        self.state.update("kill", {"valid": True, "active": True, "channel": 7})
        status = self.state.snapshot()
        self.assertTrue(status["status"]["state"]["value"]["armed"])
        self.assertFalse(status["success"])
        self.assertFalse(status["can_detach"])
        with self.assertRaisesRegex(ValueError, "急停"):
            self.state.send_detach(self.publisher)
        self.publisher.assert_not_called()

    def test_controller_queued_detach_blocks_duplicate_after_http_pending_expires(self):
        self.holding()
        self.state.update("wall", dict(self.state.samples["wall"]["value"], detach_pending=True))
        self.assertFalse(self.state.snapshot()["can_detach"])

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
        with mock.patch.object(server.threading, "Thread") as thread, \
             mock.patch.object(runtime, "inspect_runtime", return_value={"enabled": True}):
            reply = runtime.start(DEFAULT_CONFIG)
        self.assertIn("无需重复启动", reply["message"])
        self.assertEqual(self.state.launch_error, "")
        thread.assert_not_called()

    def test_new_computer_adopts_running_configuration(self):
        self.ready()
        running = dict(DEFAULT_CONFIG, test_profile="restrained_bench", hover_thrust=0.52,
                       fcu_port="/dev/ttyUSB3", front_port="/dev/ttyUSB4")
        self.state.config = running
        bridge = object.__new__(server.SSHTransport)
        bridge.lock = threading.RLock()
        bridge.snapshot = self.state.snapshot()
        bridge.arrived, bridge.connected, bridge.error = 100.0, True, ""
        bridge.args = SimpleNamespace(ssh_host="test-host", ssh_user="pi")
        with mock.patch.object(server.time, "monotonic", return_value=100.1):
            result = bridge.status(DEFAULT_CONFIG)
        self.assertEqual(result["config"], running)

    def test_runtime_reads_pi_settings_for_another_computer(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        names = {"/mavros", "/terminal_operator_node", "/flight_mode_manager_node",
                 "/attachment_output_mux_node", "/wall_perch_decision_node", "/sensor_range_manager_node",
                 "/ceiling_lidar_node", "/front_lidar_node"}
        values = {"/mavros/fcu_url": "/dev/ttyUSB3:230400",
                  "/ceiling_lidar_node/serial_port": "/dev/ttyUSB1",
                  "/front_lidar_node/serial_port": "/dev/ttyUSB4",
                  "/wall_perch_decision_node/hover_thrust": 0.52}
        runtime.ros = SimpleNamespace(get_param=lambda name, default=False:
            values.get(name, False if name.endswith(("require_rc_center", "rc_assist_enabled")) else True))
        with mock.patch.object(runtime, "nodes", return_value=names):
            self.assertTrue(runtime.inspect_runtime()["enabled"])
        self.assertEqual(runtime.state.config["test_profile"], "restrained_bench")
        self.assertEqual(runtime.state.config["fcu_port"], "/dev/ttyUSB3")
        self.assertEqual(runtime.state.config["hover_thrust"], 0.52)

    def test_armed_vehicle_blocks_new_pipeline_start(self):
        self.ready()
        self.state.update("runtime", {"enabled": False})
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        runtime.ros = None
        with mock.patch.object(runtime, "nodes", return_value=set()), \
             mock.patch.object(server.threading, "Thread") as thread, self.assertRaisesRegex(ValueError, "未解锁"):
            runtime.start(DEFAULT_CONFIG)
        thread.assert_not_called()

    def test_reconnect_start_inspects_existing_nodes_before_first_monitor_tick(self):
        self.ready()
        self.state.update("runtime", {"enabled": False})
        runtime = self.telemetry_runtime()
        runtime.ros.get_param.return_value = DEFAULT_CONFIG["fcu_port"] + ":" + str(DEFAULT_CONFIG["baud"])
        with mock.patch.object(runtime, "inspect_runtime", return_value={"enabled": True}) as inspect, \
             mock.patch.object(server.threading, "Thread") as thread:
            self.assertIn("无需重复启动", runtime.start(DEFAULT_CONFIG)["message"])
        inspect.assert_called_once()
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
