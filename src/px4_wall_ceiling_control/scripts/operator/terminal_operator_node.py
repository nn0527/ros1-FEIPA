#!/usr/bin/env python3
"""One-shot terminal commands for a persistent attachment operator intent."""

import json
import threading
import time
import rospy
from mavros_msgs.msg import State, RCIn
from mavros_msgs.srv import ParamGet
from std_msgs.msg import Bool, String

from px4_wall_ceiling_control.msg import OperatorCommand
from px4_wall_ceiling_control.adapters.kill_switch import KillSwitchConfig


class TerminalOperatorNode:
    def __init__(self):
        self.front_sensor_masked = False
        self.front_mask_seen_offboard = False
        rospy.set_param("/attachment/front_sensor_masked", False)
        self.mode = OperatorCommand.MODE_STANDBY
        self.detaching = False
        self.detach_pulse = False
        self.was_offboard = False
        self.flight_mode = ""
        self.kill_guard_enabled = bool(rospy.get_param("~kill_guard_enabled", False))
        self.kill_config = None
        self.kill_config_loading = False
        self.kill_config_attempt_at = -float("inf")
        self.rc_channels, self.rc_at = (), None
        self.flight_at = None
        self.flight_connected = self.flight_armed = False
        self.request_acks = {}
        self.command_ack_pub = rospy.Publisher("/attachment/terminal_ack", String, queue_size=10)
        self.kill_pub = rospy.Publisher("/attachment/kill_switch", String, queue_size=1)
        rospy.Subscriber("/mavros/rc/in", RCIn, self._on_rc, queue_size=1)
        self.command_pub = rospy.Publisher(
            "/operator/command", OperatorCommand, queue_size=10
        )
        self.ceiling_detach_pub = rospy.Publisher(
            "/ceiling_attachment/detach", Bool, queue_size=10
        )
        self.wall_detach_pub = rospy.Publisher(
            "/wall_perch/detach", Bool, queue_size=10
        )
        rospy.Subscriber(
            "/attachment/terminal_command", String, self._on_command, queue_size=10
        )
        rospy.Subscriber("/mavros/state", State, self._on_state, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._on_timer)

    def _on_command(self, message):
        if not message.data.lstrip().startswith("{"):
            return self._apply_command(message)  # Preserve existing terminal commands.
        request_id = ""
        try:
            request = json.loads(message.data)
            request_id = request.get("id", "")
            command = request.get("command", "")
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64 or command != "ceiling":
                raise ValueError("请求格式无效")
            if request_id in self.request_acks:
                self.command_ack_pub.publish(String(data=json.dumps(self.request_acks[request_id],ensure_ascii=False)))
                return
            age = (rospy.Time.now()-self.flight_at).to_sec() if self.flight_at is not None else float('inf')
            safety = self._kill_report() if self.kill_guard_enabled else {"valid":True,"active":False}
            reason = ("急停开关已生效或反馈无效" if not safety['valid'] or safety['active'] else
                      "飞控未连接、心跳过期或尚未解锁" if not (self.flight_connected and self.flight_armed and 0 <= age <= 1.5) else
                      "吸顶请求需从ALTCTL发起" if self.flight_mode != "ALTCTL" else
                      "已有动作请求，不能重复启动" if self.mode != OperatorCommand.MODE_STANDBY or self.detaching else "")
            if reason:
                accepted = False
            else:
                self._apply_command(String(data="ceiling"))
                accepted = self.mode == OperatorCommand.MODE_CEILING
                reason = "吸顶节点已接收：等待顶距≤1m连续0.30秒及接管条件" if accepted else "节点安全条件变化，请检查急停和飞控反馈"
                if accepted:
                    self._publish_intent()  # Confirmed intent is sent immediately, not on the next timer tick.
            reply = {"id":request_id,"command":"ceiling","accepted":accepted,"reason":reason,
                     "mode":self.mode,"enable_control":self.mode==OperatorCommand.MODE_CEILING,
                     "ros_time":rospy.Time.now().to_sec()}
            if len(self.request_acks) >= 32:self.request_acks.pop(next(iter(self.request_acks)))
            self.request_acks[request_id] = reply
        except Exception as error:
            reply = {"id":request_id,"command":"ceiling","accepted":False,"reason":str(error)}
        self.command_ack_pub.publish(String(data=json.dumps(reply,ensure_ascii=False)))
        rospy.logwarn("ceiling request acknowledgement: %s",json.dumps(reply,ensure_ascii=False))

    def _apply_command(self, message):
        command = message.data.strip().lower()
        if command in ("ceiling", "wall", "detach") and getattr(self, "kill_guard_enabled", False):
            safety = self._kill_report()
            if not safety["valid"] or safety["active"]:
                rospy.logwarn("terminal operator: kill switch active or unavailable; command rejected")
                return
        if command == "wall" and getattr(self, "front_sensor_masked", False):
            rospy.logwarn("front sensor masked during ceiling cycle; wait for confirmed ALTCTL handover before wall testing")
            return
        if command in ("ceiling", "wall"):
            if self.mode != OperatorCommand.MODE_STANDBY or self.detaching:
                rospy.logwarn("terminal operator: finish current cycle before %s", command)
                return
            pilot_mode = (str(rospy.get_param("/flight_mode_manager_node/wall_pilot_mode","ALTCTL")).upper() if command == "wall" else "ALTCTL")
            if self.flight_mode != pilot_mode:
                rospy.logwarn("terminal operator: start only from PX4 %s",pilot_mode)
                return
            self.mode = (
                OperatorCommand.MODE_CEILING
                if command == "ceiling"
                else OperatorCommand.MODE_WALL
            )
            self.was_offboard = False
            if command == "ceiling":
                self.front_sensor_masked = True
                self.front_mask_seen_offboard = False
                rospy.set_param("/attachment/front_sensor_masked", True)
                rospy.logwarn("ceiling requested: front sensor excluded from control until confirmed ALTCTL handover")
            rospy.loginfo("terminal operator: %s requested", command)
        elif command == "detach":
            if self.mode not in (
                OperatorCommand.MODE_CEILING,
                OperatorCommand.MODE_WALL,
            ):
                rospy.logwarn("terminal operator: no attachment to detach")
                return
            self.detaching = True
            self.detach_pulse = True
            rospy.loginfo("terminal operator: detach requested")
        elif command == "cancel":
            # Release the behavior request. Decision nodes then execute their
            # recovery path; this does not force a premature mode change.
            self.mode = OperatorCommand.MODE_STANDBY
            self.detaching = False
            self.detach_pulse = False
            rospy.logwarn("terminal operator: behavior cancel requested")
        else:
            rospy.logwarn("terminal operator: use ceiling, wall, detach or cancel")

    def _on_state(self, message):
        self.flight_at = rospy.Time.now()
        self.flight_connected, self.flight_armed = message.connected, message.armed
        if getattr(self, "kill_guard_enabled", False) and message.connected and not message.armed and not self.kill_config_loading and time.monotonic() - self.kill_config_attempt_at >= 10:
            self.kill_config_loading = True
            self.kill_config_attempt_at = time.monotonic()
            threading.Thread(target=self._load_kill_config, daemon=True).start()
        if not message.connected:
            self.kill_config = None
        if getattr(self, "kill_guard_enabled", False) and not message.armed:
            self._release_intent()
        self.flight_mode = message.mode.upper()
        if message.connected and getattr(self, "front_sensor_masked", False):
            if self.flight_mode == "OFFBOARD":
                self.front_mask_seen_offboard = True
            elif self.flight_mode == "ALTCTL" and (getattr(self, "front_mask_seen_offboard", False) or self.mode == OperatorCommand.MODE_STANDBY):
                self.front_sensor_masked = False
                self.front_mask_seen_offboard = False
                rospy.set_param("/attachment/front_sensor_masked", False)
                rospy.loginfo("confirmed ALTCTL handover: front sensor control input restored")
        if self.flight_mode == "OFFBOARD":
            self.was_offboard = True
        elif self.was_offboard and message.connected and self.flight_mode == (str(rospy.get_param("/flight_mode_manager_node/wall_pilot_mode","ALTCTL")).upper() if self.mode == OperatorCommand.MODE_WALL else "ALTCTL"):
            self.mode = OperatorCommand.MODE_STANDBY
            self.detaching = False
            self.detach_pulse = False
            self.was_offboard = False

    def _release_intent(self):
        self.mode = OperatorCommand.MODE_STANDBY
        self.detaching = self.detach_pulse = self.was_offboard = False

    def _load_kill_config(self):
        try:
            get = rospy.ServiceProxy("/mavros/param/get", ParamGet)
            def read(name, integer=False):
                result = get(param_id=name)
                if not result.success:
                    raise ValueError("FCU parameter unavailable: " + name)
                return result.value.integer if integer else result.value.real
            channel = read("RC_MAP_KILL_SW", True)
            config = (KillSwitchConfig(channel, read("RC_KILLSWITCH_TH"), read("RC%d_MIN" % channel),
                read("RC%d_MAX" % channel), read("RC%d_REV" % channel)) if channel else KillSwitchConfig(0))
            self.kill_config = config
        except Exception as error:
            rospy.logwarn("kill-switch configuration read failed: %s", error)
        finally:
            self.kill_config_loading = False

    def _on_rc(self, message):
        self.rc_channels, self.rc_at = tuple(message.channels), rospy.Time.now()
        if self.kill_guard_enabled and self._kill_report().get("active"):
            self._release_intent()

    def _kill_report(self):
        now = rospy.Time.now()
        if self.kill_config is None:
            return {"valid": False, "active": False, "channel": None}
        fresh = self.rc_at is not None and 0 <= (now - self.rc_at).to_sec() <= 0.25
        return self.kill_config.report(self.rc_channels, fresh)

    def _on_timer(self, _event):
        if getattr(self, "kill_guard_enabled", False):
            safety = self._kill_report()
            if not safety["valid"] or safety["active"]:
                self._release_intent()
            self.kill_pub.publish(String(data=json.dumps(safety)))
        self._publish_intent()
        if self.detach_pulse:
            if self.mode == OperatorCommand.MODE_CEILING:
                self.ceiling_detach_pub.publish(Bool(data=True))
            elif self.mode == OperatorCommand.MODE_WALL:
                self.wall_detach_pub.publish(Bool(data=True))
            self.detach_pulse = False

    def _publish_intent(self):
        command = OperatorCommand()
        command.header.stamp = rospy.Time.now()
        command.mode = self.mode
        command.enable_control = self.mode in (
            OperatorCommand.MODE_CEILING,
            OperatorCommand.MODE_WALL,
        )
        command.source = "terminal"
        command.reason = "detach pending" if self.detaching else "terminal request"
        self.command_pub.publish(command)


if __name__ == "__main__":
    rospy.init_node("terminal_operator_node")
    TerminalOperatorNode()
    rospy.spin()
