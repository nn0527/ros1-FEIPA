#!/usr/bin/env python3
"""Exercise passive logging failure cases without a ROS master."""

import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("flight_logger", str(
    PACKAGE / "scripts/test_support/attachment_flight_logger.py"))
logger = importlib.util.module_from_spec(spec)
spec.loader.exec_module(logger)


def orientation(pitch=0):
    return NS(x=0., y=math.sin(pitch / 2), z=0., w=math.cos(pitch / 2))


def header():
    return NS(stamp=NS(to_sec=lambda: 10.), frame_id="map")


def odom(pitch=0):
    return NS(header=header(), child_frame_id="base_link",
              pose=NS(pose=NS(position=NS(x=0., y=0., z=2.), orientation=orientation(pitch))),
              twist=NS(twist=NS(linear=NS(x=1., y=0., z=0.), angular=NS(x=0., y=0., z=0.))))


class FakeRos:
    Time = NS(now=lambda: NS(to_sec=lambda: 10.))
    AnyMsg = object

    def __init__(self, root):
        self.values = {"~directory": str(root / "records"), "/run_id": "test-run",
                       "/": {"wall_perch_decision_node": {"detach_time_s": 4.}},
                       "/wall_perch_decision_node/odom_twist_in_child_frame": True}
        self.hooks = []

    def get_param(self, key, default=None):
        return self.values.get(key, default)

    def Subscriber(self, *_, **__):
        return NS(unregister=lambda: None)

    def on_shutdown(self, hook):
        self.hooks.append(hook)

    def is_shutdown(self):
        return False

    def loginfo(self, *_, **__):
        pass

    logerr = loginfo


class LoggerTests(unittest.TestCase):
    def test_quaternion_sign_and_invalid_attitudes(self):
        q = orientation(math.pi / 2)
        minus_q = NS(**{axis: -getattr(q, axis) for axis in ("x", "y", "z", "w")})
        self.assertAlmostEqual(logger.attitude_error_deg(q, minus_q), 0)
        self.assertAlmostEqual(logger.attitude_error_deg(q, orientation()), 90)
        self.assertIsNone(logger.attitude_error_deg(NS(x=0, y=0, z=0, w=0), q))

    def test_velocity_frame_conversion(self):
        self.assertAlmostEqual(logger.world_vertical_speed(odom(math.pi / 2), True), -1)
        self.assertEqual(logger.world_vertical_speed(odom(math.pi / 2), False), 0)

    def test_readable_csv_handles_missing_stale_and_rate_only_targets(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            summary = logger.SignalSummary(root, FakeRos(root), True)
            summary.write()  # No messages yet: CSV columns must remain stable.
            summary.remember("/mavros/local_position/odom", odom(math.pi / 2))
            summary.remember("/mavros/setpoint_raw/attitude", NS(
                header=header(), orientation=orientation(), type_mask=7, thrust=.45))
            summary.write()
            with summary.lock:
                item = summary.latest["/mavros/setpoint_raw/attitude"]
                summary.latest["/mavros/setpoint_raw/attitude"] = (item[0], item[1] - 2)
            summary.write()
            summary.remember("/mavros/setpoint_raw/attitude", NS(
                header=header(), orientation=orientation(), type_mask=128, thrust=.45))
            summary.close()
            with (root / "signals.csv").open(encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["sent_error_deg"], "")
            self.assertAlmostEqual(float(rows[1]["sent_error_deg"]), 90)
            self.assertEqual(rows[2]["sent_error_deg"], "")
            self.assertEqual(rows[3]["sent_error_deg"], "")

    def test_event_changes_and_repeated_state_are_distinguishable(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            summary = logger.SignalSummary(root, FakeRos(root), True)
            state = NS(connected=True, armed=True, mode="OFFBOARD")
            summary.remember("/mavros/state", state)
            summary.remember("/mavros/state", state)
            summary.remember("/mavros/state", NS(connected=True, armed=True, mode="ALTCTL"))
            summary.close()
            events = (root / "events.log").read_text(encoding="utf-8")
            self.assertEqual(events.count("mode=OFFBOARD"), 1)
            self.assertEqual(events.count("mode=ALTCTL"), 1)
            self.assertEqual(summary.counts["/mavros/state"], 3)

    def test_snapshot_and_traceback_copy_preserve_originals(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source"
            source.mkdir()
            original = source / "wall-stdout.log"
            original.write_text("Traceback: test failure\n", encoding="utf-8")
            self.assertEqual(logger.copy_run_logs(source, root / "saved"), [])
            self.assertEqual(original.read_text(), (root / "saved" / original.name).read_text())
            logger.snapshot_package(PACKAGE, root)
            hashes = json.loads((root / "source_sha256.json").read_text(encoding="utf-8"))
            self.assertIn("scripts/decisions/wall_perch_decision_node.py", hashes)

    def test_auto_start_and_stop_finalize_without_flight_commands(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            ros = FakeRos(root)
            run_logs = root / "roslog" / "test-run"
            run_logs.mkdir(parents=True)
            (run_logs / "wall-stdout.log").write_text("Traceback\n", encoding="utf-8")
            process = NS(pid=999, poll=lambda: None)
            capture = {}

            def popen(command, **kwargs):
                capture["command"] = command
                capture["options"] = kwargs
                prefix = Path(command[command.index("-O") + 1])
                prefix.with_name(prefix.name + "_0.bag").write_bytes(b"mock bag")
                process.wait = lambda timeout: 0
                return process

            instance = logger.FlightLogger(ros)
            with patch.dict(os.environ, {"ROS_LOG_DIR": str(root / "roslog")}), patch.dict(
                    sys.modules, {"rospkg": NS(RosPack=lambda: NS(get_path=lambda _: str(PACKAGE)))}), patch.object(
                    logger.subprocess, "Popen", side_effect=popen), patch.object(logger.os, "killpg", create=True) as interrupt:
                instance.start()
                instance.request_stop()
                instance.run()
                instance.stop()  # Idempotent.
                self.assertEqual(interrupt.call_count, 1)
            self.assertTrue(capture["options"]["start_new_session"])
            self.assertIn("--duration=30", capture["command"])
            session = json.loads((instance.session / "session.json").read_text(encoding="utf-8"))
            self.assertEqual(session["status"], "stopped")
            self.assertTrue((instance.session / "signals.csv").exists())
            self.assertTrue((instance.session / "ros_parameters.json").exists())
            self.assertTrue((instance.session / "ros_logs" / "wall-stdout.log").exists())

    def test_low_disk_does_not_spawn_recorder(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            instance = logger.FlightLogger(FakeRos(root))
            with patch.object(logger.shutil, "disk_usage", return_value=NS(free=0)), patch.object(
                    logger.subprocess, "Popen") as spawn:
                with self.assertRaises(RuntimeError):
                    instance.start()
                instance.stop()
                spawn.assert_not_called()

    def test_launch_is_passive_and_keeps_terminal_interactive(self):
        launch = ET.parse(PACKAGE / "launch/system/attachment_system_mavros.launch").getroot()
        recorder = next(node for node in launch.findall("node") if node.get("name") == "attachment_flight_logger")
        self.assertEqual(recorder.get("required"), "false")
        terminal = next(node for node in launch.findall("node") if node.get("name") == "terminal_operator_node")
        self.assertEqual(terminal.get("output"), "screen")
        self.assertEqual(next(arg for arg in launch.findall("arg") if arg.get("name") == "record_logs").get("default"), "true")


if __name__ == "__main__":
    unittest.main()
