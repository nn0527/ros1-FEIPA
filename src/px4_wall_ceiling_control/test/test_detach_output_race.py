"""Real-thread departure initialization and receipt-time regression tests."""
import copy
import threading
import unittest
from unittest import mock

import rospy
from std_msgs.msg import UInt8
from px4_wall_ceiling_control.msg import AttachmentControlCandidate as C, WallPerchStatus as W
from test_detach_state_machine import DetachStateMachineTests, at, quaternion


class ObservedLock:
    def __init__(self):
        self.lock = threading.RLock()
        self.attempted = threading.Event()

    def __enter__(self):
        if threading.current_thread().name == 'output-callback':
            self.attempted.set()
        self.lock.acquire()
        return self

    def __exit__(self, *args):
        self.lock.release()


class DetachOutputRaceTests(unittest.TestCase):
    def setUp(self):
        self.fx = DetachStateMachineTests()
        self.n = self.fx.node()
        self.n.feedback_lock = ObservedLock()
        self.a = self.n.horizontal_output
        self.b = copy.deepcopy(self.a)
        self.b.thrust = .69
        for k, v in zip('xyzw', quaternion(85.)):
            setattr(self.b.orientation, k, v)
        self.errors = []
        self.threads = []
        for name in ('loginfo', 'logwarn'):
            patch = mock.patch.object(rospy, name)
            patch.start()
            self.addCleanup(patch.stop)
        patch = mock.patch.object(rospy.Time, 'now', return_value=at(10.))
        patch.start()
        self.addCleanup(patch.stop)
        self.addCleanup(self.finish)

    def start(self, name, action):
        def run():
            try:
                action()
            except BaseException as error:
                self.errors.append(error)
        thread = threading.Thread(name=name, target=run, daemon=True)
        self.threads.append(thread)
        thread.start()
        return thread

    def finish(self):
        for thread in self.threads:
            thread.join(3.)
            self.assertFalse(thread.is_alive(), 'worker deadlocked')
        if self.errors:
            raise self.errors[0]

    def assert_seed(self, output):
        self.assertEqual(self.n.state, W.STATE_DETACH_ROTATE)
        self.assertTrue(self.n.detach_vertical_active)
        self.assertAlmostEqual(self.n.detach_vertical_controller.value, output.thrust)
        for actual, expected in zip(self.n.q_detach_start,
                                    [getattr(output.orientation, k) for k in 'xyzw']):
            self.assertAlmostEqual(actual, expected)

    def test_callback_attempt_after_read_cannot_replace_initialization_snapshot(self):
        read = threading.Event()
        done = threading.Event()
        original = self.n._valid_wall_output
        def read_a(now):
            output = original(now)
            read.set()
            self.assertTrue(self.n.feedback_lock.attempted.wait(3.))
            self.assertFalse(done.is_set())
            self.assertIs(self.n.horizontal_output, self.a)
            return output
        self.n._valid_wall_output = read_a
        self.n.detach_requested = True
        self.n.detach_requested_at = at(10.)
        def callback():
            self.n._on_horizontal_output(self.b)
            done.set()
        with mock.patch.object(rospy, 'get_param', side_effect=AssertionError('parameter I/O')):
            control = self.start('control-cycle', lambda: self.n._on_timer(None))
            self.assertTrue(read.wait(3.))
            update = self.start('output-callback', callback)
            control.join(3.)
            update.join(3.)
        self.finish()
        self.assert_seed(self.a)
        self.assertTrue(done.is_set())
        self.assertIs(self.n.horizontal_output, self.b)

    def test_callback_completed_first_seeds_b(self):
        update = self.start('output-callback', lambda: self.n._on_horizontal_output(self.b))
        update.join(3.)
        self.finish()
        self.n.detach_requested = True
        self.n.detach_requested_at = at(10.)
        self.n._on_timer(None)
        self.assert_seed(self.b)

    def test_waiting_callbacks_preserve_arrival_time_and_expire(self):
        for owner in (False, True):
            with self.subTest(owner=owner):
                self.n.feedback_lock.attempted.clear()
                with self.n.feedback_lock:
                    callback = (lambda: self.n._on_horizontal_output_owner(UInt8(data=C.SOURCE_WALL))) if owner else (lambda: self.n._on_horizontal_output(self.b))
                    worker = self.start('output-callback', callback)
                    self.assertTrue(self.n.feedback_lock.attempted.wait(3.))
                    # Callback has already sampled 10.0 and is blocked on lock.
                    rospy.Time.now.return_value = at(10.4)
                worker.join(3.)
                self.finish()
                received = self.n.horizontal_output_owner_at if owner else self.n.horizontal_output_at
                self.assertEqual(received, at(10.))
                self.assertIsNone(self.n._detach_vertical_output(at(10.4)))
                rospy.Time.now.return_value = at(10.)

    def test_invalid_output_and_owner_still_rejected(self):
        for fault in ('owner', 'owner_age', 'receipt_age', 'source_age', 'attitude', 'thrust', 'mask'):
            with self.subTest(fault=fault):
                self.n = self.fx.node()
                output = self.n.horizontal_output
                if fault == 'owner': self.n.horizontal_output_owner = C.SOURCE_CEILING
                elif fault == 'owner_age': self.n.horizontal_output_owner_at = at(9.6)
                elif fault == 'receipt_age': self.n.horizontal_output_at = at(9.7)
                elif fault == 'source_age': output.header.stamp = at(9.7)
                elif fault == 'attitude': output.orientation.w = float('nan')
                elif fault == 'thrust': output.thrust = float('nan')
                elif fault == 'mask': output.type_mask |= output.IGNORE_ATTITUDE
                self.assertFalse(self.n._begin_detach_vertical(at(10.), output))
                self.assertFalse(self.n.detach_vertical_active)
                self.assertNotEqual(self.n.detach_vertical_reason, '未开始正常脱离保高')

    def test_distinct_but_equal_objects_report_identity_failure(self):
        self.assertFalse(self.n._begin_detach_vertical(at(10.), copy.deepcopy(self.a)))
        self.assertIn('最终输出消息已变化', self.n.detach_vertical_reason)
        self.assertFalse(self.n.detach_vertical_active)


if __name__ == '__main__':
    unittest.main()
