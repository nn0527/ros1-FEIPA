"""Serial parameter work, deliberately outside the policy/callback lock."""
import copy
import queue
import threading
import time


class OperationCancelled(RuntimeError):
    """The live policy revoked a queued operation before any FCU write."""


class MemoryRestoreJournal:
    """Keep the current cycle's originals and intent in memory only.

    A new node starts empty: no filesystem reads, writes or crash recovery.
    """
    def __init__(self):
        self.data = None

    def load(self):
        return copy.deepcopy(self.data)

    def save(self, data):
        self.data = copy.deepcopy(data)

    def clear(self):
        self.data = None


class StagedRestoreJournal:
    """Policy-owned memory image. The worker persists it BEFORE every write.

    An acknowledged write may leave an older pending record on disk until the
    next write/verified clear. This is conservative on restart: originals and
    the possible outstanding write survive, including the quarantine barrier.
    """
    def __init__(self, durable):
        self.data = durable.load()  # startup only, before subscribing to inputs

    def load(self):
        return copy.deepcopy(self.data)

    def save(self, data):
        self.data = copy.deepcopy(data)

    def clear(self):
        self.data = None  # real clear must already have succeeded in the worker


class SerialParameterWorker:
    def __init__(self, execute, ros_clock, monotonic_clock=time.monotonic,
                 journal=None, authorize=None):
        self.execute, self.ros_clock, self.monotonic_clock = execute, ros_clock, monotonic_clock
        self.journal, self.authorize = journal, authorize
        self.requests, self.results, self.progress, self.diagnostics = (queue.Queue(maxsize=1),
            queue.Queue(), queue.Queue(), queue.Queue())
        self.thread = threading.Thread(target=self._run, name='wall-fusion-parameters', daemon=True)
        self.thread.start()

    def submit(self, operation, record=None, originals=None):
        self.requests.put_nowait((operation, copy.deepcopy(record), copy.deepcopy(originals)))

    def _stage(self, operation, stage):
        self.progress.put((operation, stage, self.ros_clock(), self.monotonic_clock()))

    def _run(self):
        while True:
            task = self.requests.get()
            if task is None:
                return
            operation, record, originals = task
            timing = {'token': operation.token, 'kind': operation.kind,
                      'parameter': operation.parameter, 'value': operation.value,
                      'revision': operation.revision, 'queued_mono': operation.issued_mono,
                      'queue_s': max(0., self.monotonic_clock()-operation.issued_mono)}
            try:
                if self.authorize is not None and not self.authorize(operation):
                    raise OperationCancelled('operation superseded before dispatch')
                if operation.kind == 'set' and self.journal is not None:
                    pending = (record or {}).get('pending') or {}
                    if (pending.get('token') != operation.token
                            or pending.get('parameter') != operation.parameter
                            or pending.get('value') != operation.value
                            or set((record or {}).get('originals', {})) != {'EKF2_OF_CTRL', 'EKF2_RNG_CTRL'}):
                        raise ValueError('missing matching durable write intent')
                    self._stage(operation, 'persisting')
                    began = self.monotonic_clock()
                    self.journal.save(record)
                    timing['persist_s'] = self.monotonic_clock()-began
                if self.authorize is not None and not self.authorize(operation):
                    raise OperationCancelled('operation superseded before dispatch')
                self._stage(operation, 'executing')
                began = self.monotonic_clock()
                values = self.execute(operation)
                timing['wire_s'] = self.monotonic_clock()-began
                if operation.kind == 'verify' and values == originals and self.journal is not None:
                    self._stage(operation, 'clearing')
                    began = self.monotonic_clock()
                    self.journal.clear()
                    timing['clear_s'] = self.monotonic_clock()-began
                result = (operation, True, values, self.ros_clock(), self.monotonic_clock(), '')
                timing['success'] = True
            except Exception as exc:
                cancelled = isinstance(exc, OperationCancelled)
                result = (operation, False, {'cancelled': True} if cancelled else {},
                          self.ros_clock(), self.monotonic_clock(),
                          '{}: {}'.format(type(exc).__name__, exc))
                timing.update(success=False, cancelled=cancelled, reason=result[-1])
            self.diagnostics.put(timing)
            self.results.put(result)

    def close(self):
        try:
            self.requests.put_nowait(None)
        except queue.Full:
            pass  # durable intent survives a process exit during an operation
