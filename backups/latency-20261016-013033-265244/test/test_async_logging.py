"""Blocked logging backends must not stall state changes or control publication."""
import logging
import threading
import time
import unittest
from unittest import mock

from px4_wall_ceiling_control.adapters import async_logging as async_log


class RecordingSink(logging.Handler):
    def __init__(self):
        super().__init__()
        self.records = []

    def emit(self, record):
        self.records.append(record)


class BlockedSink(RecordingSink):
    def __init__(self):
        super().__init__()
        self.entered, self.release = threading.Event(), threading.Event()

    def emit(self, record):
        self.entered.set()
        self.release.wait(5)
        super().emit(record)


class AsyncLoggingTests(unittest.TestCase):
    def setUp(self):
        self.root, self.rosout = logging.getLogger(), logging.getLogger("rosout")
        self.saved = [(l, l.handlers[:], l.propagate, l.level) for l in (self.root, self.rosout)]
        self.old_dispatcher = async_log._installed
        async_log._installed = None
        self.sink, self.parent_sink = BlockedSink(), RecordingSink()
        self.rosout.handlers, self.rosout.propagate = [self.sink], True
        self.rosout.setLevel(logging.DEBUG)
        self.root.handlers = [self.parent_sink]
        self.dispatcher = None

    def tearDown(self):
        self.sink.release.set()
        if self.dispatcher:
            self.dispatcher.close()
            self.dispatcher.worker.join(2)
        for logger, handlers, propagate, level in self.saved:
            logger.handlers, logger.propagate = handlers, propagate
            logger.setLevel(level)
        async_log._installed = self.old_dispatcher

    def install(self, capacity=8):
        self.dispatcher = async_log.install_async_logging(capacity)
        self.rosout.info("backend blocking probe")
        self.assertTrue(self.sink.entered.wait(1))

    def test_blocked_backend_does_not_block_producer_and_queue_is_bounded(self):
        self.install()
        finished = threading.Event()

        def produce():
            for i in range(1000):
                self.rosout.warning("control event %d", i)
            finished.set()

        producer = threading.Thread(target=produce, daemon=True)
        producer.start()
        self.assertTrue(finished.wait(0.5), "producer waited for blocked sink")
        stats = self.dispatcher.snapshot()
        self.assertLessEqual(stats["queued"], 8)
        self.assertGreater(stats["dropped"], 0)
        self.assertFalse(self.sink.release.is_set())
        self.sink.release.set()
        producer.join(1)
        self.dispatcher.close()
        self.dispatcher.worker.join(2)
        self.assertTrue(any("overflow" in v.getMessage() for v in self.sink.records))

    def test_formats_only_in_writer_and_preserves_original_record_and_routing(self):
        self.install()
        formatted = threading.Event()

        class Message:
            def __str__(self):
                formatted.set()
                return "deferred message"

        record = logging.LogRecord("rosout", logging.ERROR, "wall.py", 1182,
                                   "%s", (Message(),), None, "_enter_state")
        record.created = 123.0
        self.rosout.handle(record)
        self.assertFalse(formatted.is_set())
        self.sink.release.set()
        self.dispatcher.close()
        self.dispatcher.worker.join(2)
        self.assertIs(self.sink.records[-1], record)
        self.assertEqual(record.created, 123.0)
        self.assertEqual(record.funcName, "_enter_state")
        self.assertEqual(self.parent_sink.records.count(record), 1)
        self.assertEqual(record.getMessage(), "deferred message")

    def test_handler_failure_is_contained_and_install_is_idempotent(self):
        class Broken(logging.Handler):
            def emit(self, record):
                raise OSError("disk or logging backend unavailable")

        self.sink.release.set()
        self.rosout.handlers.insert(0, Broken())
        self.install()
        self.assertIs(async_log.install_async_logging(), self.dispatcher)
        self.rosout.error("fault survives a broken sink")
        self.dispatcher.close()
        self.dispatcher.worker.join(2)
        self.assertGreater(self.dispatcher.snapshot()["handler_errors"], 0)
        self.assertTrue(any(v.getMessage() == "fault survives a broken sink"
                            for v in self.sink.records))

    def test_root_library_logs_are_also_deferred(self):
        self.install()
        self.root.warning("library warning")
        self.assertEqual(self.parent_sink.records, [])
        self.sink.release.set()
        self.dispatcher.close()
        self.dispatcher.worker.join(2)
        self.assertEqual(sum(v.getMessage() == "library warning"
                             for v in self.parent_sink.records), 1)

    def test_real_wall_capture_transition_keeps_publishing_while_log_sink_blocks(self):
        import rospy
        from test_attachment_safety import WallDecision
        from px4_wall_ceiling_control.msg import WallPerchStatus as W

        self.install(32)
        node = WallDecision.__new__(WallDecision)
        node.state, node.previous_update_at = W.STATE_WALL_CAPTURE, None
        node.horizontal_velocity_enabled = False
        node.q_contact, node.max_thrust = (0., 0., 0., 1.), .85
        node._current_quaternion = lambda: node.q_contact
        node._wall_pressure_nominal_thrust = lambda: .72
        node._wall_pressure_start_thrust = lambda: .648
        node._prepare_vertical_reference = node._prepare_horizontal_velocity = mock.Mock()
        node._prepare_horizontal_equilibrium = mock.Mock()
        node._update_state_machine = lambda now: node._enter_state(W.STATE_WALL_HOLD, now, "contact")
        node._build_status = node._build_control_candidate = mock.Mock()
        node.status_publisher, node.control_candidate_publisher = mock.Mock(), mock.Mock()
        finished = threading.Event()
        errors = []

        def control():
            try:
                for i in range(20):
                    with mock.patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(10 + i*.02)):
                        node._on_timer(None)
            except Exception as error:
                errors.append(error)
            finally:
                finished.set()

        # Use the real Python logging route while keeping ROS time deterministic.
        with mock.patch.object(rospy, "loginfo", side_effect=self.rosout.info):
            thread = threading.Thread(target=control, daemon=True)
            thread.start()
            self.assertTrue(finished.wait(.5), "capture transition waited for logging I/O")
            thread.join(1)
        self.assertEqual(errors, [])
        self.assertEqual(node.state, W.STATE_WALL_HOLD)
        self.assertEqual(node.control_candidate_publisher.publish.call_count, 20)
        self.assertFalse(self.sink.release.is_set())


if __name__ == "__main__":
    unittest.main()
