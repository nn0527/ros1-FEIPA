"""Keep ROS logging handlers and their I/O off control callback threads.

Install after rospy.init_node(), before constructing subscribers or timers.
Records retain their original creation time and caller information. Formatting,
ROS publication, parameter access and disk writes happen only in the worker.
The bounded queue drops new records under overload instead of waiting.
"""
import logging
import queue
import threading
import time


class AsyncLogDispatcher:
    def __init__(self, capacity=256):
        if not isinstance(capacity, int) or capacity < 1:
            raise ValueError("async log capacity must be a positive integer")
        self.queue = queue.Queue(maxsize=capacity)
        self.closed = threading.Event()
        self.stats_lock = threading.Lock()
        self.enqueued = self.dropped = self.delivered = self.handler_errors = 0
        self.max_handler_s = 0.0
        self.reported_dropped = 0
        self.worker = threading.Thread(target=self._run, name="ros-log-writer", daemon=True)
        self.worker.start()

    def submit(self, record, sinks):
        if self.closed.is_set():
            return
        try:
            self.queue.put_nowait((record, sinks))
        except queue.Full:
            with self.stats_lock:
                self.dropped += 1
        else:
            with self.stats_lock:
                self.enqueued += 1

    def snapshot(self):
        with self.stats_lock:
            return dict(enqueued=self.enqueued, dropped=self.dropped,
                        delivered=self.delivered, handler_errors=self.handler_errors,
                        max_handler_s=self.max_handler_s, queued=self.queue.qsize(),
                        capacity=self.queue.maxsize)

    def close(self):
        # Never join the writer from a ROS callback or shutdown hook.
        self.closed.set()

    def _deliver(self, record, sinks):
        started = time.monotonic()
        for sink in sinks:
            if record.levelno < sink.level:
                continue
            try:
                sink.handle(record)
            except Exception:
                # A broken logger cannot kill the writer or invoke more logging.
                with self.stats_lock:
                    self.handler_errors += 1
        with self.stats_lock:
            self.max_handler_s = max(self.max_handler_s, time.monotonic() - started)

    def _run(self):
        while not self.closed.is_set() or not self.queue.empty():
            try:
                record, sinks = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self._deliver(record, sinks)
                with self.stats_lock:
                    self.delivered += 1
                    dropped = self.dropped
                if dropped > self.reported_dropped:
                    summary = logging.LogRecord(
                        "rosout", logging.WARNING, __file__, 0,
                        "Async log queue overflow: dropped %d records; control callbacks did not wait",
                        (dropped - self.reported_dropped,), None)
                    self.reported_dropped = dropped
                    self._deliver(summary, sinks)
            finally:
                self.queue.task_done()


class NonBlockingLogHandler(logging.Handler):
    def __init__(self, dispatcher, sinks):
        super().__init__(level=logging.NOTSET)
        self.dispatcher, self.sinks = dispatcher, tuple(sinks)

    def emit(self, record):
        # Unlike QueueHandler.prepare(), do not format on the producer thread.
        self.dispatcher.submit(record, self.sinks)


_installed = None


def install_async_logging(capacity=256):
    """Route both rospy logs and library/root logs through one bounded worker."""
    global _installed
    if _installed is not None:
        return _installed
    root, rosout = logging.getLogger(), logging.getLogger("rosout")
    root_sinks = tuple(root.handlers)
    rosout_sinks = list(rosout.handlers)
    if rosout.propagate:
        parent = rosout.parent
        while parent is not None:
            rosout_sinks.extend(parent.handlers)
            if not parent.propagate:
                break
            parent = parent.parent
    dispatcher = AsyncLogDispatcher(capacity)
    root.handlers = [NonBlockingLogHandler(dispatcher, root_sinks)]
    rosout.handlers = [NonBlockingLogHandler(dispatcher, rosout_sinks)]
    rosout.propagate = False  # Original parent sinks are already in this route.
    _installed = dispatcher
    return dispatcher


def install_ros_async_logging(rospy):
    dispatcher = install_async_logging(int(rospy.get_param("~async_log_capacity", 256)))
    rospy.on_shutdown(dispatcher.close)
    # Startup-only parameter write; never perform this in the control timer.
    rospy.set_param("~async_logging_enabled", True)
    return dispatcher
