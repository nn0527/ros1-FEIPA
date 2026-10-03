#!/usr/bin/env python3
"""Local HTTP page + SSH JSON bridge. --bridge runs only on the ROS machine."""
import argparse
import json
import math
import os
from pathlib import Path
import secrets
import shlex
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PACKAGE = Path(__file__).resolve().parents[2]
# Keep Catkin's relay package first: it exposes both generated msg/ and source
# modules. The source fallback is only needed on the Windows HTTP host.
sys.path.append(str(PACKAGE / "src"))
from px4_wall_ceiling_control.wall_console import ConsoleState, DEFAULT_CONFIG, finite_json, launch_arguments, validate_config


class RemoteRuntime:
    """ROS actions remain on the Pi. Starting never arms or requests a mode."""
    def __init__(self):
        self.state = ConsoleState()
        self.children = {}
        self.ros = None
        self.init_lock = threading.Lock()
        self.telemetry_lock = threading.Lock()
        self.telemetry_attempt_at = -math.inf
        self.telemetry_connected = False
        self.publisher = None
        self.subscriptions = []
        self.log_dir = Path.home() / ".ros" / "wall_console"
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.state.event("SSH 已连接；等待启动基础程序")
        threading.Thread(target=self.observe_existing, daemon=True).start()

    @staticmethod
    def nodes():
        result = subprocess.run(["rosnode", "list"], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                text=True, timeout=3)
        return set(result.stdout.split()) if result.returncode == 0 else set()

    def spawn(self, name, args):
        log = self.log_dir / (name + ".log")
        with log.open("ab") as out:
            self.children[name] = subprocess.Popen(args, stdout=out, stderr=out, start_new_session=True)
        self.state.event("启动 " + name + "；日志 " + str(log))

    @staticmethod
    def port_owners(port):
        owners = set()
        port = str(Path(port).resolve())
        for fd in Path("/proc").glob("[0-9]*/fd/*"):
            try:
                if os.readlink(str(fd)) == port:
                    owners.add(int(fd.parts[2]))
            except OSError:
                continue
        return sorted(owners)

    def prepare_lidars(self):
        # Only the disarmed startup path may reset a measurement. Old readers
        # can survive a ROS master restart, so node names alone are insufficient.
        ports = [("ceiling_port", "ceiling_lidar_node"), ("front_port", "front_lidar_node")]
        for key, _ in ports:
            owners = self.port_owners(self.state.config[key])
            if owners:
                raise ValueError("雷达串口仍被旧进程占用，请结束整个旧 roslaunch: "
                                 + self.state.config[key] + " PID " + str(owners))
        import yaml
        import serial
        sensors = yaml.safe_load((PACKAGE / "config/sensors.yaml").read_text(encoding="utf-8"))
        for key, node in ports:
            flight = self.state.snapshot()["status"]["state"]
            if not flight["fresh"] or not flight["value"].get("connected") or flight["value"].get("armed"):
                raise ValueError("雷达启动复位要求新鲜、已连接且未解锁的飞控状态")
            settings = sensors[node]
            if settings.get("input_format", "l1_ascii") != "l1_ascii":
                continue
            with serial.Serial(self.state.config[key], int(settings.get("baudrate", 38400)),
                               timeout=0.2, exclusive=True) as device:
                device.write(b"iHALT")
                device.flush()
                time.sleep(0.2)
                device.reset_input_buffer()
            self.state.event("已复位雷达测量: " + self.state.config[key] + "；新节点将重新启动测距")

    def observe_existing(self):
        while self.ros is None:
            try:
                if self.nodes():
                    self.init_ros()
                    return
            except Exception:
                pass
            time.sleep(1)

    def configure_telemetry(self):
        with self.telemetry_lock:
            self.telemetry_attempt_at = time.monotonic()
            self._configure_telemetry()

    def _configure_telemetry(self):
        # Lower serial bandwidth scales PX4's default 1 Hz estimator stream
        # below the existing freshness limit. Request more samples, keeping
        # the controller's timeout unchanged. Never configure an armed FCU.
        def require_disarmed():
            flight = self.state.snapshot()["status"]["state"]
            if not flight["fresh"] or not flight["value"].get("connected") or flight["value"].get("armed"):
                raise ValueError("遥测配置要求新鲜、已连接且未解锁的飞控状态")
        require_disarmed()
        from mavros_msgs.srv import MessageInterval
        self.ros.wait_for_service("/mavros/set_message_interval", timeout=3)
        require_disarmed()
        request = self.ros.ServiceProxy("/mavros/set_message_interval", MessageInterval)
        if not request(message_id=230, message_rate=5.0).success:
            raise ValueError("飞控未确认估计器遥测频率，请查看 MAVROS 日志后重试")
        self.state.event("已请求 ESTIMATOR_STATUS 5 Hz；实际频率仍由飞控链路带宽决定")

    def maintain_telemetry(self):
        """Restore the volatile stream request after FCU/SSH reconnect, disarmed only."""
        status = self.state.snapshot()["status"]
        flight = status["state"]
        connected = bool(flight["fresh"] and flight["value"].get("connected"))
        reconnected = connected and not self.telemetry_connected
        self.telemetry_connected = connected
        if not connected or flight["value"].get("armed"):
            return
        # Retry a rejected/lost request or a stream that fell back to its low
        # default rate. Do not flood commands while the FCU is booting.
        if time.monotonic() - self.telemetry_attempt_at < 10.0:
            return
        if reconnected or not status["estimator"]["fresh"]:
            try:
                self.configure_telemetry()
            except Exception as error:
                self.state.event("估计器遥测频率恢复失败，将在未解锁时重试: " + str(error)[:200])

    def init_ros(self):
        # Only one thread initializes rospy, including a concurrent start request.
        with self.init_lock:
            if self.ros is not None:
                return
            import rospy
            from mavros_msgs.msg import State, RCIn, EstimatorStatus, AttitudeTarget
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import Range, BatteryState
            from std_msgs.msg import String, UInt8
            from rosgraph_msgs.msg import Log
            from px4_wall_ceiling_control.msg import SensorHealth, WallPerchStatus, OperatorCommand
            rospy.init_node("wall_console_bridge", anonymous=False, disable_signals=True)
            self.publisher = rospy.Publisher("/attachment/terminal_command", String, queue_size=1)
            topics = {"state": ("/mavros/state", State), "rc": ("/mavros/rc/in", RCIn),
                      "estimator": ("/mavros/estimator_status", EstimatorStatus),
                      "odom": ("/mavros/local_position/odom", Odometry),
                      "health": ("/sensor/health", SensorHealth), "wall": ("/wall_perch/status", WallPerchStatus),
                      "phase": ("/attachment/flight_phase", String), "reason": ("/attachment/flight_phase_reason", String),
                      "owner": ("/attachment/output_owner", UInt8), "operator": ("/operator/command", OperatorCommand),
                      "front": ("/sensor/front/range", Range), "ceiling": ("/sensor/ceiling/range", Range),
                      "front_diagnostic": ("/sensor/front/diagnostic", String),
                      "ceiling_diagnostic": ("/sensor/ceiling/diagnostic", String),
                      "kill": ("/attachment/kill_switch", String),
                      "setpoint": ("/mavros/setpoint_raw/attitude", AttitudeTarget), "battery": ("/mavros/battery", BatteryState)}
            self.subscriptions = [rospy.Subscriber(topic, kind, self.callback, callback_args=name,
                                    queue_size=1, tcp_nodelay=True) for name, (topic, kind) in topics.items()]
            self.subscriptions.append(rospy.Subscriber("/rosout_agg", Log, self.roslog, queue_size=20))
            self.ros = rospy
            threading.Thread(target=self.monitor, daemon=True).start()

    def callback(self, msg, name):
        if name == "state":
            value = {"connected": msg.connected, "armed": msg.armed, "mode": msg.mode}
        elif name == "rc":
            value = {"channels": list(msg.channels), "rssi": msg.rssi}
        elif name == "estimator":
            value = {"attitude": msg.attitude_status_flag, "vertical_velocity": msg.velocity_vert_status_flag}
        elif name == "health":
            value = {"ceiling_valid": msg.ceiling_valid, "front_valid": msg.front_valid,
                     "ceiling_age_s": msg.ceiling_age_s, "front_age_s": msg.front_age_s}
        elif name == "wall":
            value = {key: getattr(msg, key) for key in ("state_name", "reason", "progress", "fault_detected",
                      "front_distance_filtered_m", "top_distance_filtered_m", "thrust_normalized", "top_contact_ready",
                      "altitude_reference_valid", "height_above_takeoff_m", "min_entry_height_m", "entry_height_ready",
                      "hold_achieved", "reached_state_name", "detach_ready", "detach_completed", "recovery_completed",
                      "detach_pending", "detach_block_reason")}
        elif name in ("front_diagnostic", "ceiling_diagnostic", "kill"):
            value = json.loads(msg.data)
        elif name in ("phase", "reason", "owner"):
            value = msg.data
        elif name == "operator":
            value = {"mode": msg.mode, "enabled": msg.enable_control}
        elif name == "setpoint":
            value = {"thrust": msg.thrust, "type_mask": msg.type_mask}
        elif name == "battery":
            value = {"voltage": msg.voltage, "percentage": msg.percentage}
        elif name in ("ceiling", "front"):
            value = {"range_m": msg.range}
        elif name == "odom":
            from px4_wall_ceiling_control.adapters.odometry import odometry_is_finite, vertical_speed_enu_mps
            from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion
            q = msg.pose.pose.orientation
            finite = odometry_is_finite(msg)
            value = {"finite": finite, "z": msg.pose.pose.position.z,
                     "vertical_speed": vertical_speed_enu_mps(msg) if finite else None,
                     "angles": [math.degrees(v) for v in euler_from_quaternion((q.x, q.y, q.z, q.w))] if finite else None}
        else:
            return
        self.state.update(name, finite_json(value))

    def roslog(self, msg):
        if msg.level >= 4 and msg.name in ("/flight_mode_manager_node", "/wall_perch_decision_node",
                "/attachment_output_mux_node", "/terminal_operator_node", "/mavros"):
            # Keep repeated MAVROS warnings from burying operator events.
            now = time.monotonic()
            if not hasattr(self, "last_warning") or now - self.last_warning > 2:
                self.last_warning = now
                self.state.event(msg.name + ": " + msg.msg[:350], "error" if msg.level >= 8 else "warning")

    def inspect_runtime(self):
        required = {"/mavros", "/terminal_operator_node", "/flight_mode_manager_node",
                    "/attachment_output_mux_node", "/wall_perch_decision_node", "/sensor_range_manager_node",
                    "/ceiling_lidar_node", "/front_lidar_node"}
        nodes = self.nodes()
        wall_bench = bool(self.ros.get_param("/wall_perch_decision_node/bench_wait_for_ranges", False))
        manager_bench = bool(self.ros.get_param("/flight_mode_manager_node/wall_bench_wait_for_ranges", False))
        enabled = (required <= nodes and self.ros.get_param("/flight_mode_manager_node/enabled", False)
                   and all(self.ros.get_param(name + "/kill_guard_enabled", False) for name in
                           ("/terminal_operator_node", "/flight_mode_manager_node",
                            "/attachment_output_mux_node", "/wall_perch_decision_node"))
                   and wall_bench == manager_bench
                   and self.ros.get_param("/attachment_output_mux_node/output_enabled", False)
                   and self.ros.get_param("/wall_perch_decision_node/decision_enabled", False)
                   and not self.ros.get_param("/flight_mode_manager_node/require_rc_center", False)
                   and not self.ros.get_param("/attachment_output_mux_node/rc_assist_enabled", False)
                   and not (nodes & {"/ceiling_attachment_decision_node", "/ceiling_motor_bench_output_node",
                                    "/flight_supervisor_node", "/setpoint_mux_node", "/ceiling_attachment_output_node"}))
        profile = "restrained_bench" if wall_bench and manager_bench else "flight"
        with self.state.lock:
            if required <= nodes:
                running = dict(self.state.config, test_profile=profile)
                url = self.ros.get_param("/mavros/fcu_url", "")
                try:
                    port, baud = url.rsplit(":", 1)
                    running.update(fcu_port=port, baud=int(baud),
                        ceiling_port=self.ros.get_param("/ceiling_lidar_node/serial_port", running["ceiling_port"]),
                        front_port=self.ros.get_param("/front_lidar_node/serial_port", running["front_port"]),
                        hover_thrust=self.ros.get_param("/wall_perch_decision_node/hover_thrust", running["hover_thrust"]))
                    self.state.config = validate_config(running)
                except (ValueError, TypeError):
                    enabled = False
        return {"enabled": bool(enabled), "test_profile": profile, "nodes": sorted(nodes),
                "missing": sorted(required - nodes), "logger": "/attachment_flight_logger" in nodes}

    def monitor(self):
        while not self.ros.is_shutdown():
            try:
                self.state.update("runtime", self.inspect_runtime())
                self.maintain_telemetry()
                for name, child in list(self.children.items()):
                    if child.poll() is not None and not self.state.starting:
                        self.state.launch_error = name + " 已退出，请查看启动日志"
            except Exception as error:
                self.state.update("runtime", {"enabled": False, "error": str(error)[:200]})
            time.sleep(1)

    def start(self, config):
        config = validate_config(config)
        # After SSH reconnection, inspect the already-running nodes before
        # the first monitor tick; never attempt a duplicate launch.
        if self.ros is None and self.nodes():
            self.init_ros()
        with self.state.lock:
            if self.state.starting:
                return {"message": "基础程序正在启动"}
            if self.ros is not None:
                self.state.update("runtime", self.inspect_runtime())
            if self.state.snapshot()["status"]["runtime"]["fresh"] and self.state.snapshot()["status"]["runtime"]["value"].get("enabled"):
                if self.state.config != config:
                    raise ValueError("基础程序正在运行，当前页面不支持在运行中变更配置")
                url = config["fcu_port"] + ":" + str(config["baud"])
                if self.ros.get_param("/mavros/fcu_url", "") != url:
                    raise ValueError("已运行 MAVROS 的串口配置与页面不一致")
                flight = self.state.snapshot()["status"]["state"]
                if flight["fresh"] and flight["value"].get("connected") and not flight["value"].get("armed"):
                    self.configure_telemetry()
                self.state.launch_error = ""
                return {"message": "基础程序已运行，无需重复启动"}
            flight = self.state.snapshot()["status"]["state"]
            if flight["value"] and (not flight["fresh"] or flight["value"].get("armed")):
                raise ValueError("请在未解锁且飞控状态有效时启动基础程序")
            self.state.config = config
            self.state.starting = True
            self.state.launch_error = ""
        threading.Thread(target=self.start_worker, daemon=True).start()
        return {"message": "开始启动 MAVROS 与贴墙基础程序"}

    def start_worker(self):
        try:
            nodes = self.nodes()
            conflicting = nodes & {"/attachment_output_mux_node", "/wall_perch_decision_node", "/flight_mode_manager_node",
                                   "/ceiling_motor_bench_output_node", "/flight_supervisor_node", "/setpoint_mux_node",
                                   "/ceiling_attachment_output_node"}
            if conflicting:
                raise ValueError("已有控制节点在运行，请先结束原启动入口: " + ", ".join(sorted(conflicting)))
            for key in ("fcu_port", "ceiling_port", "front_port"):
                if not Path(self.state.config[key]).is_char_device():
                    raise ValueError("找不到串口设备: " + self.state.config[key])
            if not nodes:
                self.spawn("roscore", ["roscore"])
                deadline = time.monotonic() + 10
                while not self.nodes():
                    if time.monotonic() > deadline:
                        raise ValueError("ROS master 启动超时")
                    time.sleep(0.4)
            self.init_ros()
            nodes = self.nodes()
            url = self.state.config["fcu_port"] + ":" + str(self.state.config["baud"])
            if "/mavros" not in nodes:
                self.spawn("mavros", ["roslaunch", "mavros", "px4.launch", "fcu_url:=" + url])
            elif self.ros.get_param("/mavros/fcu_url", "") != url:
                raise ValueError("已运行 MAVROS 的串口配置与页面不一致")
            else:
                self.state.event("复用已运行的唯一 MAVROS 实例")
            # A connected flight state must establish disarmed status before output-capable nodes launch.
            deadline = time.monotonic() + 25
            while True:
                item = self.state.snapshot()["status"]["state"]
                if item["fresh"] and item["value"]["connected"]:
                    if item["value"]["armed"]:
                        raise ValueError("飞控已解锁，基础控制链路未启动；请先着陆上锁")
                    break
                if time.monotonic() > deadline:
                    raise ValueError("MAVROS 连接超时，检查飞控串口、波特率与心跳")
                time.sleep(0.2)
            self.configure_telemetry()
            self.prepare_lidars()
            # Starting sensors takes time; recheck before creating output nodes.
            item = self.state.snapshot()["status"]["state"]
            if not item["fresh"] or not item["value"]["connected"] or item["value"]["armed"]:
                raise ValueError("飞控状态已变化，贴墙基础链路未启动；请在未解锁状态重试")
            self.spawn("wall_flight", launch_arguments(self.state.config))
            deadline = time.monotonic() + 15
            while not (self.state.snapshot()["status"]["runtime"]["value"] or {}).get("enabled", False):
                child = self.children["wall_flight"]
                if child.poll() is not None:
                    raise ValueError("贴墙链路启动失败，查看 " + str(self.log_dir / "wall_flight.log"))
                if time.monotonic() > deadline:
                    raise ValueError("贴墙节点启动未完成，请查看缺失节点和启动日志")
                time.sleep(0.3)
            self.state.launch_error = ""
            self.state.event("基础程序已就绪；等待飞手在 ALTCTL 起飞并稳定")
        except Exception as error:
            self.state.launch_error = str(error)
            self.state.event(str(error), "error")
        finally:
            self.state.starting = False

    def wall(self):
        def publish(command):
            if not self.publisher or self.publisher.get_num_connections() == 0:
                raise ValueError("终端指令节点未订阅，消息未发送")
            self.publisher.publish(command)
        self.state.send_wall(publish)
        return {"message": "wall 已发送，等待贴墙阶段反馈"}

    def detach(self):
        def publish(command):
            if not self.publisher or self.publisher.get_num_connections() == 0:
                raise ValueError("终端指令节点未订阅，消息未发送")
            self.publisher.publish(command)
        self.state.send_detach(publish)
        return {"message": "离墙请求已发送；控制节点等待连续稳定后旋转，8 秒未满足会取消"}


def bridge_main():
    output = sys.stdout
    sys.stdout = sys.stderr  # ROS logging cannot corrupt the JSON-line transport.
    write_lock = threading.Lock()
    def send(value):
        with write_lock:
            output.write(json.dumps(finite_json(value), ensure_ascii=False, allow_nan=False) + "\n")
            output.flush()
    runtime = RemoteRuntime()
    running = threading.Event()
    running.set()
    def stream():
        while running.is_set():
            try:
                send({"type": "snapshot", "data": runtime.state.snapshot()})
            except (BrokenPipeError, OSError):
                return
            time.sleep(0.1)
    threading.Thread(target=stream, daemon=True).start()
    for line in sys.stdin:
        request = {}
        try:
            request = json.loads(line)
            action = request.get("action")
            if action == "start":
                result = runtime.start(request["config"])
            elif action == "wall":
                result = runtime.wall()
            elif action == "detach":
                result = runtime.detach()
            else:
                raise ValueError("未知操作")
            send({"type": "reply", "id": request["id"], "ok": True, "data": result})
        except Exception as error:
            send({"type": "reply", "id": request.get("id"), "ok": False, "error": str(error)})
    running.clear()
    # Do not stop flight processes because the local UI or SSH connection closes.


class SSHTransport:
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.snapshot = ConsoleState().snapshot()
        self.connected = False
        self.error = "正在连接树莓派"
        self.arrived = 0.0
        self.pending = {}
        self.writer = None
        self.client = None
        self.stderr_tail = ""
        threading.Thread(target=self.connect_loop, daemon=True).start()

    def connect_loop(self):
        import paramiko
        while True:
            client = paramiko.SSHClient()
            try:
                client.load_host_keys(str(Path(getattr(self.args, "known_hosts", Path.home() / ".ssh" / "known_hosts")).expanduser()))
                client.set_missing_host_key_policy(paramiko.RejectPolicy())
                client.connect(self.args.ssh_host, port=self.args.ssh_port, username=self.args.ssh_user,
                               key_filename=self.args.ssh_key, look_for_keys=False, allow_agent=False,
                               timeout=8, auth_timeout=8)
                client.get_transport().set_keepalive(5)
                source = self.args.remote_workspace + "/devel/setup.bash"
                worker = self.args.remote_workspace + "/src/px4_wall_ceiling_control/scripts/operator/wall_console_server.py"
                script = "source /opt/ros/noetic/setup.bash && source " + shlex.quote(source) + " && exec python3 " + shlex.quote(worker) + " --bridge"
                command = "docker exec -i " + shlex.quote(self.args.container) + " bash -lc " + shlex.quote(script)
                stdin, stdout, stderr = client.exec_command(command)
                with self.lock:
                    self.client, self.writer = client, stdin
                def drain(stream):
                    for line in stream:
                        self.stderr_tail = (self.stderr_tail + line)[-2000:]
                threading.Thread(target=drain, args=(stderr,), daemon=True).start()
                for line in stdout:
                    message = json.loads(line)
                    with self.lock:
                        if message.get("type") == "snapshot":
                            self.snapshot = message["data"]
                            self.connected, self.error, self.arrived = True, "", time.monotonic()
                        elif message.get("type") == "reply" and message.get("id") in self.pending:
                            slot = self.pending[message["id"]]
                            slot["reply"] = message
                            slot["event"].set()
                raise ConnectionError("SSH 桥接已结束" + (": " + self.stderr_tail[-300:] if self.stderr_tail else ""))
            except Exception as error:
                with self.lock:
                    self.connected, self.writer, self.error = False, None, str(error)
                    for slot in self.pending.values():
                        slot["reply"] = {"ok": False, "error": "SSH 已断开；操作结果未知，请核对实时状态"}
                        slot["event"].set()
            finally:
                client.close()
            time.sleep(3)

    def status(self, config):
        with self.lock:
            result = json.loads(json.dumps(self.snapshot))
            age = time.monotonic() - self.arrived if self.arrived else None
            connected = self.connected and age is not None and age < 2
            runtime = result["status"]["runtime"]
            if connected and runtime["fresh"] and runtime["value"] and runtime["value"].get("enabled"):
                config = validate_config(result["config"])
            # Account for transport delay in every ROS freshness decision.
            for item in result["status"].values():
                if item["age_s"] is not None:
                    item["age_s"] += age or 0
                    item["fresh"] = connected and item["age_s"] <= item["limit_s"]
            required_feedback = ("runtime", "state", "odom", "estimator", "operator", "phase", "owner", "wall", "kill")
            if config["test_profile"] != "restrained_bench":
                required_feedback += ("health",)
            stale = any(not result["status"][n]["fresh"] for n in required_feedback)
            check_sources = {"runtime": ("runtime",), "connected": ("state",), "armed": ("state",),
                             "mode": ("state",), "odom": ("odom",), "estimator": ("estimator",),
                             "kill": ("kill",),
                             "health": () if config["test_profile"] == "restrained_bench" else ("health",),
                             "height": () if config["test_profile"] == "restrained_bench" else ("wall", "odom"), "idle": ("phase", "owner", "operator")}
            for check in result["checks"]:
                if any(not result["status"][name]["fresh"] for name in check_sources.get(check["id"], ())):
                    check["ok"] = False
            if stale and connected:
                labels = {"runtime": "基础程序", "state": "飞控心跳", "odom": "里程计", "estimator": "估计器", "health": "雷达", "operator": "操作节点", "phase": "控制阶段", "owner": "输出归属", "wall": "贴墙决策", "kill": "急停开关"}
                stale_reasons = [labels[name] + "反馈缺失或已过期" for name in required_feedback if not result["status"][name]["fresh"]]
                result["blockers"] = list(dict.fromkeys(result["blockers"] + stale_reasons))
            if not result["status"]["wall"]["fresh"] or not result["status"]["setpoint"]["fresh"]:
                result["success"] = False
            if not connected or stale:
                result["can_wall"] = False
                result["success"] = False
                if not connected:
                    result["blockers"] = ["SSH 反馈中断；历史数据不能用于判断贴墙成功"]
                    for check in result["checks"]:
                        check["ok"] = False
            detach_sources = required_feedback + ("health", "setpoint")
            if not connected or any(not result["status"][n]["fresh"] for n in detach_sources):
                if not connected or result["can_detach"]:
                    result["detach_blockers"] = ["离墙所需实时反馈中断或过期"]
                result["can_detach"] = False
            result["config"] = config
            result["ssh"] = {"connected": connected, "error": self.error, "age_s": age,
                             "host": self.args.ssh_user + "@" + self.args.ssh_host}
            result["connection"] = {name: getattr(self.args, name, None) for name in
                                    ("ssh_port", "container", "remote_workspace")}
            return result

    def request(self, action, config=None):
        identifier = secrets.token_hex(12)
        event = threading.Event()
        with self.lock:
            if not self.connected or not self.writer or time.monotonic() - self.arrived >= 2:
                raise ValueError("SSH 未连接或反馈过期，操作未发送")
            slot = {"event": event}
            self.pending[identifier] = slot
            try:
                self.writer.write(json.dumps({"id": identifier, "action": action, "config": config}) + "\n")
                self.writer.flush()
            except Exception:
                self.pending.pop(identifier, None)
                raise
        try:
            if not event.wait(6):
                raise ValueError("SSH 操作确认超时；不会自动重发，请先核对状态")
            reply = slot["reply"]
            if not reply["ok"]:
                raise ValueError(reply["error"])
            return reply["data"]
        finally:
            with self.lock:
                self.pending.pop(identifier, None)


def http_main(args):
    settings = Path(args.settings).expanduser()
    config = dict(DEFAULT_CONFIG)
    if settings.exists():
        config = validate_config(json.loads(settings.read_text(encoding="utf-8")))
    transport = SSHTransport(args)
    token = secrets.token_urlsafe(32)
    class Handler(BaseHTTPRequestHandler):
        def respond(self, code, payload, mime="application/json; charset=utf-8"):
            if not isinstance(payload, bytes):
                payload = json.dumps(finite_json(payload), ensure_ascii=False, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", mime)
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
        def valid_host(self):
            return self.headers.get("Host") in ("127.0.0.1:" + str(args.port), "localhost:" + str(args.port))
        def do_GET(self):
            if not self.valid_host():
                return self.respond(403, {"error": "Invalid host"})
            if self.path == "/api/status":
                value = transport.status(config)
                value["token"] = token
                return self.respond(200, value)
            assets = {"/": ("index.html", "text/html; charset=utf-8"),
                      "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                      "/style.css": ("style.css", "text/css; charset=utf-8")}
            if self.path in assets:
                name, mime = assets[self.path]
                return self.respond(200, (PACKAGE / "web" / "wall_console" / name).read_bytes(), mime)
            return self.respond(404, {"error": "Not found"})
        def do_POST(self):
            nonlocal config
            origin = self.headers.get("Origin")
            if not self.valid_host() or (origin and origin != "http://" + self.headers.get("Host", "")) or not secrets.compare_digest(self.headers.get("X-Console-Token", ""), token):
                return self.respond(403, {"error": "操作来源校验失败"})
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 8192:
                    raise ValueError("请求长度无效")
                data = json.loads(self.rfile.read(size))
                if self.path == "/api/start":
                    proposed = validate_config(data)
                    result = transport.request("start", proposed)
                    settings.parent.mkdir(parents=True, exist_ok=True)
                    temporary = settings.with_suffix(".tmp")
                    temporary.write_text(json.dumps(proposed, ensure_ascii=False, indent=2), encoding="utf-8")
                    temporary.replace(settings)
                    config = proposed
                elif self.path == "/api/wall":
                    result = transport.request("wall")
                elif self.path == "/api/detach":
                    result = transport.request("detach")
                else:
                    return self.respond(404, {"error": "Not found"})
                return self.respond(200, result)
            except (ValueError, TypeError, KeyError, OSError) as error:
                return self.respond(409, {"error": str(error)})
        def log_message(self, *unused):
            pass
    print("Wall console: http://127.0.0.1:" + str(args.port), flush=True)
    ThreadingHTTPServer(("127.0.0.1", args.port), Handler).serve_forever()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge", action="store_true")
    parser.add_argument("--port", type=int, default=8787)
    parser.add_argument("--ssh-host", default="192.168.50.66")
    parser.add_argument("--ssh-port", type=int, default=22)
    parser.add_argument("--ssh-user", default="pi")
    parser.add_argument("--ssh-key", default=str(Path.home() / ".ssh/id_ed25519_wallrobot"))
    parser.add_argument("--known-hosts", default=str(Path.home() / ".ssh/known_hosts"))
    parser.add_argument("--container", default="ros1_noetic")
    parser.add_argument("--remote-workspace", default="/root/ros_ws")
    parser.add_argument("--settings", default=str(Path.home() / ".codex/wall-console/settings.json"))
    options = parser.parse_args()
    if options.bridge:
        bridge_main()
    else:
        http_main(options)
