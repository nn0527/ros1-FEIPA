#!/usr/bin/env python3
"""Manage optical-flow/range EKF parameters without blocking flight outputs."""
import fcntl
import hashlib
import math
import os
import queue
import threading
import time

import rospy
from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import ParamGet, ParamPull, ParamSet
from nav_msgs.msg import Odometry

from px4_wall_ceiling_control.domain.wall_sensor_fusion import (
    FLOW, RANGE, FusionPolicy, RestoreJournal,
)
from px4_wall_ceiling_control.msg import WallPerchStatus, WallSensorFusionStatus


class MavrosParameterTransport:
    """Used only by the one service worker; ParamGet alone is a local cache."""
    def __init__(self, namespace):
        self.namespace = namespace.rstrip("/")
        self.pull = rospy.ServiceProxy(self.namespace + "/param/pull", ParamPull)
        self.get = rospy.ServiceProxy(self.namespace + "/param/get", ParamGet)
        self.set = rospy.ServiceProxy(self.namespace + "/param/set", ParamSet)

    @staticmethod
    def _integer(value):
        # EKF2_*_CTRL are integer parameters.  Refuse real-valued responses,
        # missing values, and conversions that could hide a malformed reply.
        integer = value.integer
        real = value.real
        if (type(integer) is not int or not 0 <= integer <= 2147483647
                or not math.isfinite(real) or real != 0.):
            raise ValueError("expected an integer EKF parameter response")
        return integer

    def execute(self, operation):
        if operation.kind == "set":
            rospy.wait_for_service(self.namespace + "/param/set", timeout=1.)
            response = self.set(param_id=operation.parameter,
                                value=ParamValue(integer=operation.value, real=0.))
            if not response.success:
                raise RuntimeError("PARAM_SET rejected: " + operation.parameter)
            value = self._integer(response.value)
            if value != operation.value:
                raise RuntimeError("PARAM_SET returned a different value: " + operation.parameter)
            return {operation.parameter: value}
        rospy.wait_for_service(self.namespace + "/param/pull", timeout=1.)
        response = self.pull(force_pull=True)
        if not response.success or response.param_received <= 0:
            raise RuntimeError("forced PARAM_PULL failed or returned no parameters")
        rospy.wait_for_service(self.namespace + "/param/get", timeout=1.)
        values = {}
        for parameter in (FLOW, RANGE):
            response = self.get(param_id=parameter)
            if not response.success:
                raise RuntimeError("parameter unavailable after forced pull: " + parameter)
            values[parameter] = self._integer(response.value)
        return values


class SerialParameterWorker:
    """Never cancel/replace a slow call: its late result must be reconciled."""
    def __init__(self, execute, ros_clock, monotonic_clock=time.monotonic):
        self.execute = execute
        self.ros_clock = ros_clock
        self.monotonic_clock = monotonic_clock
        self.requests = queue.Queue(maxsize=1)
        self.results = queue.Queue()
        self.thread = threading.Thread(target=self._run, name="wall-fusion-parameters", daemon=True)
        self.thread.start()

    def submit(self, operation):
        self.requests.put_nowait(operation)

    def _run(self):
        while True:
            operation = self.requests.get()
            if operation is None:
                return
            try:
                values = self.execute(operation)
                result = (operation, True, values, self.ros_clock(), self.monotonic_clock(), "")
            except Exception as exc:  # ROS transport failures must retain the journal.
                result = (operation, False, {}, self.ros_clock(), self.monotonic_clock(),
                          "{}: {}".format(type(exc).__name__, exc))
            self.results.put(result)

    def close(self):
        # Do not wait for a stuck ROS service in shutdown, or claim restoration.
        # The durable journal is intentionally left for the next process.
        try:
            self.requests.put_nowait(None)
        except queue.Full:
            pass


class WallSensorFusionManagerNode:
    def __init__(self):
        self.lock = threading.RLock()
        namespace = str(rospy.get_param("~mavros_namespace", "/mavros")).rstrip("/")
        vehicle_id = str(rospy.get_param("~vehicle_id", "")).strip()
        if not vehicle_id:
            target = rospy.get_param(namespace + "/target_system_id", None)
            fcu_url = str(rospy.get_param(namespace + "/fcu_url", "")).strip()
            if target is None or not fcu_url:
                raise ValueError("set ~vehicle_id or MAVROS target_system_id and fcu_url; vehicle identity is required")
            vehicle_id = "{}|sys={}|fcu={}".format(namespace, int(target), fcu_url)
        identity_hash = hashlib.sha256(vehicle_id.encode("utf-8")).hexdigest()[:24]
        journal_directory = rospy.get_param("~journal_directory",
                                             os.path.expanduser("~/.ros/wall_sensor_fusion"))
        journal_path = rospy.get_param("~journal_path", os.path.join(
            journal_directory, identity_hash + ".json"))
        # Keep the process lock until process exit, including shutdown while
        # the worker still owns a service request.  Recovered pending requests
        # additionally use the MAVROS-version-specific quarantine below.
        journal_path = os.path.abspath(os.path.expanduser(journal_path))
        os.makedirs(os.path.dirname(journal_path), mode=0o700, exist_ok=True)
        self.journal_lock = open(journal_path + ".lock", "a")
        fcntl.flock(self.journal_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        settings = {key: float(rospy.get_param("~" + key, default)) for key, default in (
            ("restore_tilt_deg", 80.), ("application_settle_s", 1.2),
            ("odom_timeout_s", .3), ("status_timeout_s", .6), ("state_timeout_s", 1.5),
            ("future_tolerance_s", .1), ("request_timeout_s", 5.),
            ("retry_initial_s", 1.), ("retry_max_s", 8.), ("restart_quarantine_s", 10.))}
        self.policy = FusionPolicy(RestoreJournal(journal_path, vehicle_id), vehicle_id,
                                  enabled=bool(rospy.get_param("~enabled", False)), **settings)
        self.publisher = rospy.Publisher(
            rospy.get_param("~status_topic", "/wall_perch/sensor_fusion_status"),
            WallSensorFusionStatus, queue_size=5, latch=True)
        self.transport = MavrosParameterTransport(namespace)
        self.worker = SerialParameterWorker(self.transport.execute,
                                            lambda: rospy.Time.now().to_sec())
        self.state_names = {getattr(WallPerchStatus, "STATE_" + name): name for name in (
            "IDLE", "FRONT_WALL_DETECT", "STABILIZE_HOVER", "SLOW_APPROACH",
            "FLIP_TO_WALL", "WALL_CAPTURE", "WALL_HOLD", "WALL_PIN",
            "DETACH_ROTATE", "RECOVER", "EXIT", "ABORT")}
        self.odom_subscriber = rospy.Subscriber(
            rospy.get_param("~odom_topic", namespace + "/local_position/odom"),
            Odometry, self._on_odom, queue_size=1, tcp_nodelay=True)
        self.wall_subscriber = rospy.Subscriber(
            rospy.get_param("~wall_status_topic", "/wall_perch/status"),
            WallPerchStatus, self._on_wall, queue_size=1, tcp_nodelay=True)
        self.state_subscriber = rospy.Subscriber(
            rospy.get_param("~state_topic", namespace + "/state"),
            State, self._on_state, queue_size=1, tcp_nodelay=True)
        self.timer = rospy.Timer(rospy.Duration(.1), self._on_timer)
        rospy.on_shutdown(self.worker.close)
        rospy.loginfo("wall EKF parameter manager enabled=%s vehicle=%s journal=%s; status is not actual fusion evidence",
                      self.policy.enabled, vehicle_id, journal_path)

    @staticmethod
    def _times(message):
        return message.header.stamp.to_sec(), rospy.Time.now().to_sec(), time.monotonic()

    def _on_odom(self, message):
        q = message.pose.pose.orientation
        with self.lock:
            self.policy.observe_odom((q.x, q.y, q.z, q.w), *self._times(message))

    def _on_wall(self, message):
        with self.lock:
            # Numeric state is authoritative; a malformed textual label must not
            # create a false HOLD/recovery edge.
            self.policy.observe_wall(self.state_names.get(message.state, "UNKNOWN"),
                                     *self._times(message))

    def _on_state(self, message):
        with self.lock:
            self.policy.observe_fcu(message.connected, message.armed, *self._times(message))

    def _on_timer(self, _event):
        now, monotonic = rospy.Time.now().to_sec(), time.monotonic()
        with self.lock:
            while True:
                try:
                    result = self.worker.results.get_nowait()
                except queue.Empty:
                    break
                self.policy.complete(*result)
            operation = self.policy.tick(now, monotonic)
            if operation is not None:
                self.worker.submit(operation)
            data = self.policy.status(now, monotonic)
        message = WallSensorFusionStatus()
        message.header.stamp = rospy.Time.from_sec(now)
        for key, value in data.items():
            setattr(message, key, rospy.Time.from_sec(value)
                    if key in ("request_stamp", "confirmed_stamp", "ready_after") else value)
        self.publisher.publish(message)
        if data["phase"] == "ERROR":
            rospy.logwarn_throttle(2., "wall EKF parameter management: %s", data["reason"])


def main():
    rospy.init_node("wall_sensor_fusion_manager_node")
    try:
        WallSensorFusionManagerNode()
    except (ValueError, OSError) as exc:
        rospy.logfatal("wall EKF parameter manager startup failed: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
