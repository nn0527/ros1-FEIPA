"""Regression coverage for HOLD exit, dispatch cancellation and restore priority.

All parameter transport and ROS inputs are in-process fakes; these tests never
connect to a ROS master or a flight controller.
"""
import copy
import sys
import threading
import types
import unittest
from unittest import mock

from px4_wall_ceiling_control.domain.fusion_worker import (
    SerialParameterWorker, StagedRestoreJournal,
)
from px4_wall_ceiling_control.domain.wall_sensor_fusion import FLOW, RANGE
from test_wall_sensor_fusion import (
    Harness, MemoryJournal, load_node_without_ros, quaternion,
)


class FusionDispatchRecoveryTests(unittest.TestCase):
    def manager(self, harness):
        # The node's Linux process-lock import is unrelated to these callbacks.
        # No node constructor or fcntl operation is executed in these tests.
        with mock.patch.dict(sys.modules, {"fcntl": types.ModuleType("fcntl")}):
            module = load_node_without_ros()
        module.rospy.Time = types.SimpleNamespace(now=lambda:
            types.SimpleNamespace(to_sec=lambda: harness.now))
        module.time = types.SimpleNamespace(monotonic=lambda: harness.mono)
        node = module.WallSensorFusionManagerNode.__new__(module.WallSensorFusionManagerNode)
        node.lock = threading.RLock()
        node.policy = harness.policy
        node.state_names = {state: state for state in (
            "IDLE", "WALL_HOLD", "DETACH_ROTATE", "RECOVER", "ABORT")}
        return node

    def feed_node(self, node, harness, state="DETACH_ROTATE", tilt=85., armed=True):
        harness.now += .05
        harness.mono += .05
        header = types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: harness.now))
        node._on_state(types.SimpleNamespace(header=header, connected=True, armed=armed))
        node._on_wall(types.SimpleNamespace(header=header, state=state))
        q = types.SimpleNamespace(**dict(zip(("x", "y", "z", "w"), quaternion(tilt))))
        node._on_odom(types.SimpleNamespace(header=header,
            pose=types.SimpleNamespace(pose=types.SimpleNamespace(orientation=q))))

    def start_disable(self, harness=None):
        h = harness or Harness()
        h.start()
        h.feed(state="WALL_HOLD", tilt=90.)
        operation = h.tick()
        self.assertEqual((operation.parameter, operation.value), (FLOW, 0))
        return h, operation

    def worker(self, harness, execute, journal=None, authorize=None):
        worker = SerialParameterWorker(execute, lambda: harness.now,
            monotonic_clock=lambda: harness.mono, journal=journal,
            authorize=authorize)
        self.addCleanup(worker.close)
        return worker

    def test_new_disable_requires_armed_fresh_hold_and_connected_fcu(self):
        for invalid in ("disarmed", "stale_wall", "stale_fcu", "disconnected"):
            with self.subTest(invalid=invalid):
                h = Harness()
                h.start()
                h.feed(state="WALL_HOLD", tilt=90.,
                    armed=invalid != "disarmed", connected=invalid != "disconnected")
                if invalid == "stale_wall":
                    h.now += .61
                    h.mono += .61
                    h.policy.observe_fcu(True, True, h.now, h.now, h.mono)
                elif invalid == "stale_fcu":
                    h.now += 1.51
                    h.mono += 1.51
                    h.policy.observe_wall("WALL_HOLD", h.now, h.now, h.mono)
                self.assertIsNone(h.tick())
                self.assertFalse(h.policy.restore_required)

    def test_after_first_disable_leaving_hold_does_not_queue_the_second(self):
        for state in ("DETACH_ROTATE", "RECOVER", "ABORT", "WALL_PIN"):
            with self.subTest(state=state):
                h, operation = self.start_disable()
                h.complete(operation)
                for _ in range(5):
                    h.feed(state=state, tilt=85.)
                    self.assertIsNone(h.tick())
                self.assertTrue(h.policy.restore_required)
                self.assertEqual(h.policy.confirmed[FLOW], 0)

    def test_stale_hold_cannot_queue_another_disable_in_the_same_episode(self):
        h, operation = self.start_disable()
        h.complete(operation)
        h.now += .61
        h.mono += .61
        h.policy.observe_fcu(True, True, h.now, h.now, h.mono)
        self.assertIsNone(h.tick())

    def test_dispatch_rechecks_fresh_wall_evidence(self):
        h, operation = self.start_disable()
        node = self.manager(h)
        h.now += .61
        h.mono += .61
        h.policy.observe_fcu(True, True, h.now, h.now, h.mono)
        self.assertFalse(node._authorize(operation))

    def test_dispatch_detects_disarm_before_the_next_timer(self):
        h, operation = self.start_disable()
        node = self.manager(h)
        h.feed(armed=False)
        self.assertFalse(node._authorize(operation))
        self.assertTrue(h.policy.restore_intent)

    def test_dispatch_latches_fresh_recovery_threshold_before_next_timer(self):
        h, operation = self.start_disable()
        node = self.manager(h)
        h.feed(state="DETACH_ROTATE", tilt=80.)
        # No timer has run since either callback; live recovery still wins.
        self.assertFalse(node._authorize(operation))
        self.assertTrue(h.policy.restore_intent)
        self.assertEqual(h.journal.data["desired"], "restore")
        self.assertEqual(h.journal.data["pending"]["token"], operation.token)

    def test_fresh_threshold_crossing_survives_a_delayed_policy_timer(self):
        h = Harness()
        h.suspend()
        h.feed(state="DETACH_ROTATE", tilt=85.)
        h.feed(tilt=79.)
        # Subsequent valid poses are level, but no longer contain a new crossing.
        for _ in range(8):
            h.feed(dt=.05, tilt=79.)
        restore = h.tick()
        self.assertIsNotNone(restore)
        self.assertEqual((restore.parameter, restore.value), (RANGE, 2))

    def test_old_pose_and_pose_outside_recovery_cannot_authorize_restore(self):
        h = Harness()
        h.suspend()
        h.feed(state="WALL_HOLD", tilt=79.)
        self.assertIsNone(h.tick())
        self.assertFalse(h.policy.restore_intent)
        h.now += .31
        h.mono += .31
        h.policy.observe_fcu(True, True, h.now, h.now, h.mono)
        h.policy.observe_wall("DETACH_ROTATE", h.now, h.now, h.mono)
        self.assertIsNone(h.tick())
        self.assertFalse(h.policy.restore_intent)
        h.feed(state="DETACH_ROTATE", tilt=79.)
        self.assertEqual(h.tick().parameter, RANGE)

    def test_pre_dispatch_cancel_preserves_confirmation_and_retry_budget(self):
        h, operation = self.start_disable()
        confirmed = dict(h.policy.confirmed)
        h.feed(state="DETACH_ROTATE", tilt=85.)
        accepted = h.policy.complete(operation, False, {"cancelled": True},
            h.now, h.mono, "operation superseded before dispatch")
        self.assertTrue(accepted)
        self.assertEqual(h.policy.confirmed, confirmed)
        self.assertEqual(h.policy.failures, 0)
        self.assertEqual(h.status()["retry_count"], 0)
        self.assertEqual(h.policy.error, "")
        self.assertLessEqual(h.policy.next_attempt, h.mono)
        self.assertIsNone(h.tick())
        h.feed(tilt=80.)
        restore = h.tick()
        self.assertEqual((restore.parameter, restore.value), (RANGE, 2))

    def test_restore_threshold_bypasses_previous_disable_failure_backoff(self):
        h, operation = self.start_disable()
        h.complete(operation, success=False)
        self.assertGreater(h.policy.next_attempt, h.mono)
        h.feed(state="DETACH_ROTATE", tilt=80.)
        restore = h.tick()
        self.assertIsNotNone(restore)
        self.assertEqual((restore.parameter, restore.value), (RANGE, 2))
        h.complete(restore)
        h.feed()
        restore_flow = h.tick()
        self.assertEqual((restore_flow.parameter, restore_flow.value), (FLOW, 1))

    def test_restore_failure_still_uses_backoff(self):
        h = Harness()
        h.suspend()
        h.feed(state="DETACH_ROTATE", tilt=80.)
        restore = h.tick()
        h.complete(restore, success=False)
        h.feed(dt=.1, tilt=79.)
        self.assertIsNone(h.tick())
        self.assertGreater(h.policy.next_attempt, h.mono)
        h.feed(dt=1.01)
        retry = h.tick()
        self.assertEqual((retry.parameter, retry.value), (RANGE, 2))

    def test_late_failure_of_inflight_disable_does_not_delay_new_restore(self):
        h, operation = self.start_disable()
        h.feed(state="DETACH_ROTATE", tilt=80.)
        self.assertIsNone(h.tick())
        self.assertIs(h.policy.in_flight, operation)
        self.assertTrue(h.policy.restore_intent)
        h.complete(operation, success=False)
        # A real failed write remains unknown and is counted, unlike cancellation.
        self.assertNotIn(FLOW, h.policy.confirmed)
        self.assertEqual(h.status()["retry_count"], 1)
        restore = h.tick()
        self.assertIsNotNone(restore)
        self.assertEqual((restore.parameter, restore.value), (RANGE, 2))

    def test_late_disable_ack_mismatch_does_not_delay_new_restore(self):
        h, operation = self.start_disable()
        h.feed(state="DETACH_ROTATE", tilt=80.)
        self.assertIsNone(h.tick())
        self.assertIs(h.policy.in_flight, operation)
        self.assertTrue(h.policy.restore_intent)
        h.complete(operation, values={FLOW: 1})  # Old request wanted FLOW = 0.
        self.assertNotIn(FLOW, h.policy.confirmed)
        self.assertEqual(h.status()["retry_count"], 1)
        self.assertEqual(h.policy.restore_done, set())
        restore = h.tick()
        self.assertIsNotNone(restore)
        self.assertEqual((restore.parameter, restore.value), (RANGE, 2))

    def test_actual_inflight_disable_stays_serial_until_its_return(self):
        h, operation = self.start_disable()
        entered, release = threading.Event(), threading.Event()
        calls = []
        def execute(op):
            calls.append(op)
            if op == operation:
                entered.set()
                if not release.wait(3):
                    raise RuntimeError("test did not release fake transport")
            return {op.parameter: op.value}
        worker = self.worker(h, execute)
        self.addCleanup(release.set)
        worker.submit(operation)
        self.assertTrue(entered.wait(1))
        h.feed(dt=5.1, state="DETACH_ROTATE", tilt=80.)
        self.assertIsNone(h.tick())
        self.assertIs(h.policy.in_flight, operation)
        self.assertEqual(calls, [operation])
        self.assertTrue(worker.results.empty())
        release.set()
        self.assertTrue(h.policy.complete(*worker.results.get(timeout=1)))
        self.assertFalse(h.status()["control_ready"])
        for parameter, value in ((RANGE, 2), (FLOW, 1)):
            restore = h.tick()
            self.assertEqual((restore.parameter, restore.value), (parameter, value))
            worker.submit(restore)
            self.assertTrue(h.policy.complete(*worker.results.get(timeout=1)))
            h.feed()
        self.assertEqual([(op.parameter, op.value) for op in calls],
            [(FLOW, 0), (RANGE, 2), (FLOW, 1)])

    def test_slow_persist_keeps_callbacks_live_and_cancels_without_wire_write(self):
        disk = MemoryJournal()
        staged = StagedRestoreJournal(disk)
        h, operation = self.start_disable(Harness(journal=staged))
        node = self.manager(h)
        entered, release, callback_done = (threading.Event() for _ in range(3))
        original_save = disk.save
        def slow_save(record):
            entered.set()
            if not release.wait(3):
                raise RuntimeError("test did not release fake fsync")
            original_save(record)
        disk.save = slow_save
        calls = []
        worker = self.worker(h, lambda op: calls.append(op) or {op.parameter: op.value},
            journal=disk, authorize=node._authorize)
        self.addCleanup(release.set)
        worker.submit(operation, staged.load(), dict(h.policy.originals))
        self.assertTrue(entered.wait(1))
        self.assertIsNone(disk.data)
        self.assertEqual(calls, [])
        callback_errors = []
        def callbacks():
            try:
                self.feed_node(node, h, tilt=80.)
                with node.lock:
                    self.assertIsNone(h.tick())
                    self.assertTrue(h.policy.restore_intent)
                    self.assertFalse(h.status()["control_ready"])
            except BaseException as exc:
                callback_errors.append(exc)
            finally:
                callback_done.set()
        callback_thread = threading.Thread(target=callbacks, daemon=True)
        callback_thread.start()
        self.assertTrue(callback_done.wait(1), "callbacks blocked behind durable save")
        self.assertEqual(callback_errors, [])
        self.assertEqual(calls, [])
        release.set()
        result = worker.results.get(timeout=1)
        self.assertFalse(result[1])
        self.assertEqual(result[2], {"cancelled": True})
        self.assertIn("operation superseded before dispatch", result[-1])
        self.assertEqual(calls, [])
        self.assertEqual(disk.data["pending"]["token"], operation.token)
        self.assertTrue(h.policy.complete(*result))
        self.assertEqual(h.policy.confirmed, {FLOW: 1, RANGE: 2})
        self.assertEqual(h.status()["retry_count"], 0)
        restore = h.tick()
        self.assertEqual((restore.parameter, restore.value), (RANGE, 2))

    def test_successful_write_is_dispatched_only_after_matching_durable_intent(self):
        disk = MemoryJournal()
        h, operation = self.start_disable(Harness(journal=StagedRestoreJournal(disk)))
        node = self.manager(h)
        seen_records = []
        def execute(op):
            seen_records.append(copy.deepcopy(disk.data))
            return {op.parameter: op.value}
        worker = self.worker(h, execute, journal=disk, authorize=node._authorize)
        worker.submit(operation, h.journal.load(), dict(h.policy.originals))
        result = worker.results.get(timeout=1)
        self.assertTrue(result[1], result[-1])
        self.assertEqual(len(seen_records), 1)
        self.assertEqual(seen_records[0]["originals"], {FLOW: 1, RANGE: 2})
        self.assertEqual(seen_records[0]["pending"]["token"], operation.token)
        self.assertEqual(seen_records[0]["pending"]["value"], 0)


if __name__ == "__main__":
    unittest.main()
