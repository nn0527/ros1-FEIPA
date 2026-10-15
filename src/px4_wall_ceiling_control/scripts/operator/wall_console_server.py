#!/usr/bin/env python3
"""Local HTTP page + SSH JSON bridge. --bridge runs only on the ROS machine."""
import argparse
import json
import math
import os
from pathlib import Path
import secrets
import signal
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
from px4_wall_ceiling_control.wall_console import ConsoleState, DEFAULT_CONFIG, STABLE_PORTS, finite_json, launch_arguments, validate_config, migrate_legacy_ports


class RemoteRuntime:
    """ROS actions remain on the Pi. Starting never arms or requests a mode."""
    def __init__(self):
        self.state = ConsoleState()
        self.command_ack_lock = threading.Lock()
        self.command_acks = {}
        self.children = {}
        self.stop_requested = False
        self.stopping = False
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
        if getattr(self, "stop_requested", False):
            raise ValueError("启动已取消：已请求终止现有进程")
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
            try:
                with serial.Serial(self.state.config[key], int(settings.get("baudrate", 38400)),
                                   timeout=0.2, exclusive=True) as device:
                    device.write(b"iHALT")
                    device.flush()
                    time.sleep(0.2)
                    device.reset_input_buffer()
            except OSError as error:
                if key != "front_port":
                    raise
                self.state.event("前部雷达启动故障：" + str(error) + "；贴顶可继续，侧吸需等待前距恢复", "warning")
                continue
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
        require_disarmed()
        if not request(message_id=83, message_rate=10.0).success:
            raise ValueError("飞控未确认推力目标遥测频率，无法确认平滑接管所需反馈")
        require_disarmed()
        if request(message_id=105, message_rate=10.0).success:
            self.state.event("已请求 HIGHRES_IMU 10 Hz，获取气压计原始压力；以实际新鲜气压反馈判断离顶")
        else:
            self.state.event("气压遥测频率请求未确认；气压缺失时离顶按钮禁用")
        self.state.event("已请求 ESTIMATOR_STATUS 5 Hz；实际频率仍由飞控链路带宽决定")
        self.state.event("已请求 ATTITUDE_TARGET 10 Hz，用于匹配 ALTCTL 接管推力")

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
            from std_msgs.msg import String, UInt8, Bool
            from rosgraph_msgs.msg import Log
            from px4_wall_ceiling_control.msg import SensorHealth, WallPerchStatus, CeilingAttachmentStatus, OperatorCommand
            rospy.init_node("wall_console_bridge", anonymous=True, disable_signals=True)
            self.publisher = rospy.Publisher("/attachment/terminal_command", String, queue_size=1)
            self.subscriptions_ack = rospy.Subscriber("/attachment/terminal_ack", String, self._on_terminal_ack, queue_size=10)
            self.wait_cancel_publisher = rospy.Publisher("/wall_perch/cancel_wait", Bool, queue_size=1)
            topics = {"state": ("/mavros/state", State), "rc": ("/mavros/rc/in", RCIn),
                      "estimator": ("/mavros/estimator_status", EstimatorStatus),
                      "odom": ("/mavros/local_position/odom", Odometry),
                      "health": ("/sensor/health", SensorHealth), "wall": ("/wall_perch/status", WallPerchStatus),
                      "ceiling_status": ("/ceiling_attachment/status", CeilingAttachmentStatus),
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
            value = {"attitude": msg.attitude_status_flag, "vertical_velocity": msg.velocity_vert_status_flag, "horizontal_velocity": msg.velocity_horiz_status_flag}
        elif name == "health":
            value = {"ceiling_valid": msg.ceiling_valid, "front_valid": msg.front_valid,
                     "ceiling_age_s": msg.ceiling_age_s, "front_age_s": msg.front_age_s}
        elif name == "ceiling_status":
            value = {key: getattr(msg, key) for key in ("state_name", "reason", "fault_detected", "attach_confirmed", "distance_stable", "range_fresh", "odom_fresh", "candidate_valid", "ceiling_distance_filtered_m", "compression_m", "target_distance_m", "detach_failed", "activation_ready", "detach_start_height_m", "descended_height_m", "detach_height_ready", "height_source", "baro_fresh", "baro_reference_ready", "baro_pressure_pa", "hold_pressure_pa")}
        elif name == "wall":
            value = {key: getattr(msg, key) for key in ("state_name", "reason", "progress", "fault_detected",
                      "front_distance_filtered_m", "top_distance_filtered_m", "thrust_normalized", "top_contact_ready",
                      "altitude_reference_valid", "height_above_takeoff_m", "min_entry_height_m", "entry_height_ready",
                      "hold_achieved", "reached_state_name", "detach_ready", "detach_completed", "recovery_completed",
                      "detach_pending", "detach_block_reason", "flip_ready", "front_range_fresh", "top_range_fresh",
                      "auto_approach_enabled", "approach_paused", "horizontal_velocity_enabled", "horizontal_velocity_ready", "horizontal_velocity_active", "horizontal_velocity_stable", "horizontal_velocity_reason", "horizontal_forward_target_mps", "horizontal_forward_measured_mps", "horizontal_integral_x_enu_mps2", "horizontal_integral_y_enu_mps2", "horizontal_integral_active", "horizontal_integral_reason", "horizontal_control_saturated",  "horizontal_velocity_x_enu_mps", "horizontal_velocity_y_enu_mps", "horizontal_target_x_enu_mps", "horizontal_target_y_enu_mps", "optical_flow_quality", "horizontal_equilibrium_enabled", "horizontal_equilibrium_valid", "horizontal_equilibrium_frozen", "horizontal_equilibrium_reason", "horizontal_prestream_valid")}
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
        if msg.level >= 4 and msg.name in ("/ceiling_attachment_decision_node", "/flight_mode_manager_node", "/wall_perch_decision_node",
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
        if "/ceiling_attachment_decision_node" in nodes:
            required.discard("/front_lidar_node")
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
                   and (not self.ros.get_param("/attachment_output_mux_node/rc_assist_enabled", False)
                        or self.ros.get_param("/attachment_output_mux_node/rc_roll_yaw_enabled", False))
                   and not (nodes & {"/ceiling_motor_bench_output_node",
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
                    for name, alias in STABLE_PORTS.items():
                        if Path(running[name]).resolve() == Path(alias).resolve():
                            running[name] = alias
                    self.state.config = validate_config(migrate_legacy_ports(running))
                except (ValueError, TypeError):
                    enabled = False
        return {"wall_pilot_mode": self.ros.get_param("/flight_mode_manager_node/wall_pilot_mode","ALTCTL"), "front_sensor_masked": self.ros.get_param("/attachment/front_sensor_masked", False), "ceiling_distance_gate_enabled": self.ros.get_param("/flight_mode_manager_node/ceiling_distance_gate_enabled", False), "ceiling_enabled": "/ceiling_attachment_decision_node" in nodes,
                "ceiling_activation_distance_m": self.ros.get_param("/ceiling_attachment_decision_node/activation_distance_m", .5),
                "enabled": bool(enabled), "test_profile": profile, "nodes": sorted(nodes),
                "missing": sorted(required - nodes), "logger": "/attachment_flight_logger" in nodes}

    def update_process_errors(self, runtime):
        """Report the live ROS instance rather than an obsolete launcher PID."""
        mavros_error = "mavros 无响应，请查看启动日志"
        mavros_present = "/mavros" in runtime.get("nodes", [])
        expected = self.state.config["fcu_port"] + ":" + str(self.state.config["baud"])
        mavros_live = mavros_present and self.mavros_alive() and self.fcu_url_matches(expected)
        with self.state.lock:
            for name, child in list(self.children.items()):
                if child.poll() is None or self.state.starting:
                    continue
                error = name + " 已退出，请查看启动日志"
                replaced = ((name == "wall_flight" and runtime.get("enabled")) or
                            (name == "mavros" and mavros_live))
                if replaced:
                    self.children.pop(name, None)
                    if self.state.launch_error == error:
                        self.state.launch_error = ""
                else:
                    self.state.launch_error = error
            # Continue checking a replacement even after its old PID is discarded.
            if mavros_present and not mavros_live and not self.state.starting:
                if not self.state.launch_error or self.state.launch_error == mavros_error:
                    self.state.launch_error = mavros_error
            elif mavros_live and self.state.launch_error in (
                    mavros_error, "mavros 已退出，请查看启动日志"):
                self.state.launch_error = ""

    def monitor(self):
        while not self.ros.is_shutdown():
            try:
                runtime = self.inspect_runtime()
                self.state.update("runtime", runtime)
                self.maintain_telemetry()
                if not getattr(self, "stopping", False):
                    self.update_process_errors(runtime)
            except Exception as error:
                self.state.update("runtime", {"enabled": False, "error": str(error)[:200]})
            time.sleep(1)

    def start(self, config):
        if getattr(self, "stopping", False):
            raise ValueError("正在终止现有进程，请等待结束")
        self.stop_requested = False
        config = validate_config(migrate_legacy_ports(config))
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
                if not self.fcu_url_matches(url):
                    raise ValueError("已运行 MAVROS 的串口配置与页面不一致")
                flight = self.state.snapshot()["status"]["state"]
                if flight["fresh"] and flight["value"].get("connected") and not flight["value"].get("armed"):
                    self.require_mavros_task_isolation()
                    self.configure_telemetry()
                self.state.launch_error = ""
                return {"message": "基础程序已运行，无需重复启动"}
            flight = self.state.snapshot()["status"]["state"]
            # Stopped MAVROS leaves a cached heartbeat. Bootstrap MAVROS first;
            # start_worker still requires NEW connected/disarmed feedback before controls.
            if flight["fresh"] and flight["value"] and flight["value"].get("armed"):
                raise ValueError("请在未解锁且飞控状态有效时启动基础程序")
            self.state.config = config
            self.state.starting = True
            self.state.launch_error = ""
        threading.Thread(target=self.start_worker, daemon=True).start()
        return {"message": "开始启动 MAVROS 与贴墙基础程序"}

    def require_mavros_task_isolation(self):
        blacklist = self.ros.get_param("/mavros/plugin_blacklist", [])
        if not isinstance(blacklist, list) or not {"waypoint", "geofence", "rallypoint"}.issubset(blacklist):
            raise ValueError("MAVROS未停用任务同步插件；请在上锁后终止并重新启动基础程序")

    def start_worker(self):
        try:
            nodes = self.nodes()
            conflicting = nodes & {"/attachment_output_mux_node", "/wall_perch_decision_node", "/flight_mode_manager_node",
                                   "/ceiling_motor_bench_output_node", "/flight_supervisor_node", "/setpoint_mux_node",
                                   "/ceiling_attachment_output_node"}
            if conflicting:
                raise ValueError("已有控制节点在运行，请先结束原启动入口: " + ", ".join(sorted(conflicting)))
            for key in ("fcu_port", "ceiling_port", "front_port"):
                if key == "front_port" and not Path(self.state.config[key]).is_char_device():
                    self.state.event("前雷达设备缺失；启动贴顶链路，侧吸等待前距恢复", "warning")
                    continue
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
            if "/mavros" in nodes and not self.mavros_alive():
                owners = self.port_owners(self.state.config["fcu_port"])
                if owners:
                    raise ValueError("MAVROS 无响应，但飞控串口仍被进程占用，不能重复启动: " + str(owners))
                nodes.discard("/mavros")
                self.state.event("检测到已退出的 MAVROS 残留注册；重新启动 MAVROS")
            if "/mavros" not in nodes:
                self.spawn("mavros", ["roslaunch", "px4_wall_ceiling_control", "wall_console_mavros.launch", "fcu_url:=" + url])
            elif not self.fcu_url_matches(url):
                raise ValueError("已运行 MAVROS 的串口配置与页面不一致")
            else:
                self.state.event("复用已运行的唯一 MAVROS 实例")
            # A connected flight state must establish disarmed status before output-capable nodes launch.
            deadline = time.monotonic() + 25
            while True:
                if self.stop_requested:
                    raise ValueError("启动已取消：已请求终止现有进程")
                item = self.state.snapshot()["status"]["state"]
                if item["fresh"] and item["value"]["connected"]:
                    if item["value"]["armed"]:
                        raise ValueError("飞控已解锁，基础控制链路未启动；请先着陆上锁")
                    break
                if time.monotonic() > deadline:
                    raise ValueError("MAVROS 连接超时，检查飞控串口、波特率与心跳")
                time.sleep(0.2)
            self.require_mavros_task_isolation()
            self.configure_telemetry()
            self.prepare_lidars()
            # Starting sensors takes time; recheck before creating output nodes.
            item = self.state.snapshot()["status"]["state"]
            if not item["fresh"] or not item["value"]["connected"] or item["value"]["armed"]:
                raise ValueError("飞控状态已变化，贴墙基础链路未启动；请在未解锁状态重试")
            self.spawn("wall_flight", launch_arguments(self.state.config))
            deadline = time.monotonic() + 45
            while not (self.state.snapshot()["status"]["runtime"]["value"] or {}).get("enabled", False):
                child = self.children["wall_flight"]
                if child.poll() is not None:
                    raise ValueError("贴墙链路启动失败，查看 " + str(self.log_dir / "wall_flight.log"))
                if time.monotonic() > deadline:
                    raise ValueError("贴墙节点启动未完成，请查看缺失节点和启动日志")
                time.sleep(0.3)
            self.state.launch_error = ""
            self.state.event("基础程序已就绪；侧吸用POSCTL、吸顶用ALTCTL起飞并稳定")
        except Exception as error:
            self.state.launch_error = str(error)
            self.state.event(str(error), "error")
        finally:
            self.state.starting = False

    @staticmethod
    def mavros_alive():
        """A ROS master registration can outlive the actual serial reader."""
        try:
            result = subprocess.run(["rosnode", "ping", "-c", "1", "/mavros"],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=4)
            return result.returncode == 0
        except subprocess.TimeoutExpired:
            return False

    def fcu_url_matches(self, expected):
        actual = self.ros.get_param("/mavros/fcu_url", "")
        if actual == expected:
            return True
        try:
            actual_port, actual_baud = actual.rsplit(":", 1)
            expected_port, expected_baud = expected.rsplit(":", 1)
            return (actual_baud == expected_baud and
                    Path(actual_port).resolve() == Path(expected_port).resolve())
        except (ValueError, TypeError):
            return False

    def _on_terminal_ack(self, message):
        try:reply=json.loads(message.data)
        except (ValueError,TypeError):return
        if not isinstance(reply,dict):return
        with self.command_ack_lock:
            slot=self.command_acks.get(reply.get("id"))
            if slot is not None:
                slot["reply"]=reply
                slot["event"].set()  # Do not acquire ConsoleState.lock: the request holds it while awaiting this ACK.

    def _publish_ceiling_request(self, command):
        if not self.publisher or self.publisher.get_num_connections()==0:
            raise ValueError("终端指令节点未订阅，吸顶消息未发送")
        request_id=secrets.token_hex(12)
        slot={"event":threading.Event(),"reply":None}
        with self.command_ack_lock:self.command_acks[request_id]=slot
        try:
            self.publisher.publish(json.dumps({"id":request_id,"command":command}))
            if not slot["event"].wait(2.5):
                raise ValueError("未收到吸顶节点回执，执行情况未确认；请检查实时阶段，勿重复点击。若为旧节点，请在落地上锁后重启基础程序")
            reply=slot["reply"]
            if reply.get("accepted") is not True or reply.get("mode") != 2 or reply.get("enable_control") is not True:
                error=ValueError("吸顶节点拒绝："+str(reply.get("reason","未提供原因")))
                error.command_rejected=True
                raise error
            return reply
        finally:
            with self.command_ack_lock:self.command_acks.pop(request_id,None)

    def ceiling(self):
        self.state.send_ceiling(self._publish_ceiling_request)
        return {"message":"吸顶节点已确认接收；正在ALTCTL下等待距离和接管条件", "request":self.state.ceiling_request}

    def ceiling_detach(self):
        def publish(command):
            if not self.publisher or self.publisher.get_num_connections() == 0:
                raise ValueError("终端指令节点未订阅，消息未发送")
            self.publisher.publish(command)
        self.state.send_ceiling_detach(publish)
        return {"message": "离顶请求已提交；等待吸顶节点确认"}

    def wall(self):
        def publish(command):
            if not self.publisher or self.publisher.get_num_connections() == 0:
                raise ValueError("终端指令节点未订阅，消息未发送")
            self.publisher.publish(command)
        self.state.send_wall(publish)
        return {"message": "贴墙请求已提交；保持遥控模式稳定飞行，距离及速度反馈条件满足后接管接近"}

    def detach(self):
        def publish(command):
            if not self.publisher or self.publisher.get_num_connections() == 0:
                raise ValueError("终端指令节点未订阅，消息未发送")
            self.publisher.publish(command)
        self.state.send_detach(publish)
        return {"message": "离墙请求已发送；控制节点等待连续稳定后旋转，8 秒未满足会取消"}

    def cancel_wait(self):
        with self.state.lock:
            if not self.state.snapshot()["can_cancel_wait"]:
                raise ValueError("当前不是 ALTCTL 手动接近等待阶段，取消未发送")
            if not self.wait_cancel_publisher.get_num_connections():
                raise ValueError("模式管理节点未订阅取消等待消息")
            self.wait_cancel_publisher.publish(True)
            self.state.event("已请求取消手动接近等待；以模式阶段反馈为准")
        return {"message": "取消等待已提交；仅等待阶段接受，开始接管后不会中断动作"}

    def stop(self, config=None):
        """Stop this console's test pipeline, preserving ROS master and SSH."""
        with self.state.lock:
            if getattr(self, "stopping", False):
                raise ValueError("正在终止现有进程，请等待结束")
            snapshot = self.state.snapshot()
            flight = snapshot["status"]["state"]
            value = flight.get("value") or {}
            runtime = snapshot["status"]["runtime"].get("value") or {}
            confirmed = (config or {}).get("grounded_confirmed") is True
            if flight["fresh"] and value.get("armed"):
                raise ValueError("飞控仍处于解锁状态；请先落地并上锁，再终止程序")
            if not flight["fresh"] and not confirmed:
                raise ValueError("飞控反馈过期；需明确确认飞机已落地且电机停止，才可清理异常进程")
            self.stopping = True
            self.stop_requested = True
        try:
            targets = {}
            for item in Path("/proc").glob("[0-9]*/cmdline"):
                try:
                    args = item.read_bytes().split(b"\0")
                    text_args = [a.decode(errors="replace") for a in args if a]
                    is_launch = any("roslaunch" in a for a in text_args) and (
                        "wall_console_flight.launch" in text_args or "wall_console_mavros.launch" in text_args or
                        ("mavros" in text_args and "px4.launch" in text_args))
                    is_node = any(a.startswith(str(PACKAGE / "scripts") + "/") or
                                  a.startswith(str(PACKAGE.parents[1] / "devel/lib/px4_wall_ceiling_control") + "/")
                                  for a in text_args) and not any("wall_console_server.py" in a for a in text_args)
                    is_mavros = "/opt/ros/noetic/lib/mavros/mavros_node" in text_args
                    is_record = any("rosbag" in a for a in text_args) and any(
                        a.startswith(("/root/wall_test_records/", "/root/ceiling_test/")) for a in text_args)
                    if is_launch or is_node or is_mavros or is_record:
                        targets[int(item.parent.name)] = item.read_bytes()
                except (OSError, ValueError):
                    continue
            def alive(pid, original):
                try:
                    return Path('/proc', str(pid), 'cmdline').read_bytes() == original
                except OSError:
                    return False
            for sig, delay in ((signal.SIGINT, 3.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, .3)):
                for pid, original in list(targets.items()):
                    if alive(pid, original):
                        try: os.kill(pid, sig)
                        except ProcessLookupError: pass
                deadline = time.monotonic() + delay
                while time.monotonic() < deadline and any(alive(pid, original) for pid, original in targets.items()):
                    time.sleep(.1)
            remaining = [pid for pid, original in targets.items() if alive(pid, original)]
            for name in ("wall_flight", "mavros"):
                self.children.pop(name, None)
            with self.state.lock:
                self.state.pending_since = self.state.detach_pending_since = None
                self.state.starting = False
                self.state.launch_error = "终止失败，残留 PID: " + str(remaining) if remaining else ""
            if remaining:
                raise ValueError(self.state.launch_error)
            with self.state.lock:
                self.state.samples.clear()  # Removed producers must not leave stale action state.
            self.state.update("runtime", self.inspect_runtime() if self.ros else {"enabled": False, "nodes": [], "missing": []})
            self.state.event("已终止试验控制、MAVROS、雷达及记录进程；可以重新启动")
            return {"message": "现有试验进程已终止，可以重新启动基础程序"}
        finally:
            self.stopping = False


def bridge_main():
    from px4_wall_ceiling_control.shared_bridge import relay
    relay(str(Path(__file__).resolve()))


class SSHTransport:
    def __init__(self, args):
        self.args = args
        self.password = os.environ.pop("WALL_CONSOLE_SSH_PASSWORD", None) if getattr(args, "ssh_auth", "key") == "password" else None
        self.lock = threading.RLock()
        self.snapshot = ConsoleState().snapshot()
        self.connected = False
        self.error = "正在连接树莓派"
        self.arrived = 0.0
        self.heartbeat = 0.0
        self.push_enabled = True
        self.session_id = secrets.token_hex(24)
        self.session = {"control": False, "owner": "", "clients": 0, "push": True}
        threading.Thread(target=self.keepalive_loop, daemon=True).start()
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
                               key_filename=self.args.ssh_key if getattr(self.args, "ssh_auth", "key") == "key" else None,
                               password=self.password, look_for_keys=False, allow_agent=False,
                               timeout=8, auth_timeout=8)
                client.get_transport().set_keepalive(5)
                source = self.args.remote_workspace + "/devel/setup.bash"
                worker = self.args.remote_workspace + "/src/px4_wall_ceiling_control/scripts/operator/wall_console_server.py"
                script = "source /opt/ros/noetic/setup.bash && source " + shlex.quote(source) + " && exec python3 " + shlex.quote(worker) + " --bridge"
                command = "docker exec -i " + shlex.quote(self.args.container) + " bash -lc " + shlex.quote(script)
                stdin, stdout, stderr = client.exec_command(command)
                with self.lock:
                    self.client, self.writer = client, stdin
                    self.session = {"control": False, "owner": "", "clients": 0, "push": self.push_enabled}
                    self.arrived = 0.0
                    stdin.write(json.dumps({"client": self.session_id, "label": os.environ.get("COMPUTERNAME", "Windows"), "push": self.push_enabled}) + "\n")
                    stdin.flush()
                def drain(stream):
                    for line in stream:
                        self.stderr_tail = (self.stderr_tail + line)[-2000:]
                threading.Thread(target=drain, args=(stderr,), daemon=True).start()
                for line in stdout:
                    message = json.loads(line)
                    with self.lock:
                        if message.get("type") == "session":
                            self.session = message["data"]
                            self.connected, self.error, self.heartbeat = True, "", time.monotonic()
                        elif message.get("type") == "snapshot" and self.push_enabled:
                            self.snapshot = message["data"]
                            self.connected, self.error, self.arrived = True, "", time.monotonic()
                            self.heartbeat = self.arrived
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

    def keepalive_loop(self):
        while True:
            time.sleep(2)
            with self.lock:
                if self.writer:
                    try:
                        self.writer.write('{"action":"ping"}\n')
                        self.writer.flush()
                    except OSError:
                        pass

    def status(self, config):
        with self.lock:
            result = json.loads(json.dumps(self.snapshot))
            age = time.monotonic() - self.arrived if self.arrived else None
            link_connected = self.connected and time.monotonic() - self.heartbeat < 3
            connected = link_connected and self.push_enabled and age is not None and age < 2
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
                    result["blockers"] = ["实时反馈已暂停，动作禁用" if not self.push_enabled else "SSH 反馈中断；历史数据不能用于判断贴墙成功"]
                    for check in result["checks"]:
                        check["ok"] = False
            detach_sources = required_feedback + ("health", "setpoint")
            if not connected or any(not result["status"][n]["fresh"] for n in detach_sources):
                if not connected or result["can_detach"]:
                    result["detach_blockers"] = ["离墙所需实时反馈中断或过期"]
                result["can_detach"] = False
            ceiling_sources = ("runtime", "state", "odom", "estimator", "operator", "phase", "owner", "kill", "ceiling_status")
            if not (result["status"]["runtime"]["value"] or {}).get("ceiling_distance_gate_enabled"):
                ceiling_sources += ("health","ceiling")
            if not connected or not self.session.get("control") or any(not result["status"][n]["fresh"] for n in ceiling_sources):
                result["can_ceiling"] = False
                result["ceiling_blockers"] = list(dict.fromkeys(result.get("ceiling_blockers", []) + ["吸顶所需实时反馈或控制权缺失"]))
            ceiling_detach_sources = tuple(n for n in ceiling_sources if n not in ("health", "ceiling")) + ("setpoint",)
            if not connected or not self.session.get("control") or any(not result["status"][n]["fresh"] for n in ceiling_detach_sources):
                result["can_ceiling_detach"] = False
            result["config"] = config
            if not connected or stale:
                result["can_cancel_wait"] = False
            result["session"] = dict(self.session, push=self.push_enabled, feedback_live=connected)
            if not self.session.get("control"):
                result["can_wall"] = result["can_detach"] = result["can_cancel_wait"] = False
            result["ssh"] = {"connected": link_connected, "error": self.error, "age_s": age,
                             "host": self.args.ssh_user + "@" + self.args.ssh_host}
            result["connection"] = {name: getattr(self.args, name, None) for name in
                                    ("ssh_port", "container", "remote_workspace")}
            return result

    def request(self, action, config=None):
        identifier = secrets.token_hex(12)
        event = threading.Event()
        with self.lock:
            if not self.connected or not self.writer or time.monotonic() - self.heartbeat >= 3:
                raise ValueError("SSH 未连接，操作未发送")
            if action == "stop" and not self.session.get("control"):
                raise ValueError("需要控制权，终止未发送")
            if action in ("start", "wall", "detach", "cancel_wait") and (not self.push_enabled or not self.session.get("control") or time.monotonic() - self.arrived >= 2):
                raise ValueError("需要控制权和有效的实时反馈，操作未发送")
            if action == "push":
                self.push_enabled = bool(config["enabled"])
                self.arrived = 0.0
            slot = {"event": event}
            self.pending[identifier] = slot
            try:
                self.writer.write(json.dumps({"id": identifier, "action": action, "config": config}) + "\n")
                self.writer.flush()
            except Exception:
                self.pending.pop(identifier, None)
                raise
        try:
            if not event.wait(15 if action == "stop" else 6):
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
        config = validate_config(migrate_legacy_ports(json.loads(settings.read_text(encoding="utf-8"))))
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
                value["console_protocol"] = 2
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
                    proposed = validate_config(migrate_legacy_ports(data))
                    result = transport.request("start", proposed)
                    settings.parent.mkdir(parents=True, exist_ok=True)
                    temporary = settings.with_suffix(".tmp")
                    temporary.write_text(json.dumps(proposed, ensure_ascii=False, indent=2), encoding="utf-8")
                    temporary.replace(settings)
                    config = proposed
                elif self.path in ("/api/acquire", "/api/release"):
                    result = transport.request(self.path.rsplit("/", 1)[1])
                elif self.path == "/api/push":
                    if type(data.get("enabled")) is not bool:
                        raise ValueError("enabled 必须为布尔值")
                    result = transport.request("push", data)
                elif self.path in ("/api/ceiling", "/api/ceiling-detach"):
                    result = transport.request("ceiling" if self.path == "/api/ceiling" else "ceiling_detach")
                elif self.path == "/api/wall":
                    result = transport.request("wall")
                elif self.path == "/api/cancel-wait":
                    result = transport.request("cancel_wait")
                elif self.path == "/api/stop":
                    result = transport.request("stop", data)
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
    parser.add_argument("--shared-service", action="store_true")
    parser.add_argument("--port", type=int, default=8787)
    parser.add_argument("--ssh-host", default="192.168.50.66")
    parser.add_argument("--ssh-port", type=int, default=22)
    parser.add_argument("--ssh-user", default="pi")
    parser.add_argument("--ssh-auth", choices=("key", "password"), default="key")
    parser.add_argument("--ssh-key", default=str(Path.home() / ".ssh/id_ed25519_wallrobot"))
    parser.add_argument("--known-hosts", default=str(Path.home() / ".ssh/known_hosts"))
    parser.add_argument("--container", default="ros1_noetic")
    parser.add_argument("--remote-workspace", default="/root/ros_ws")
    parser.add_argument("--settings", default=str(Path.home() / ".codex/wall-console/settings.json"))
    options = parser.parse_args()
    if options.shared_service:
        from px4_wall_ceiling_control.shared_bridge import serve
        serve(RemoteRuntime, str(Path.home() / ".ros/wall_console/shared.sock"))
    elif options.bridge:
        bridge_main()
    else:
        http_main(options)
