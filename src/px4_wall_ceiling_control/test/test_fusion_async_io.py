import copy
import math
import struct
import threading
import time
import types
import unittest

from px4_wall_ceiling_control.domain.fusion_worker import StagedRestoreJournal, SerialParameterWorker
from px4_wall_ceiling_control.domain.wall_sensor_fusion import FLOW, RANGE, Operation
from px4_wall_ceiling_control.adapters.fusion_mavlink import decode_integer_parameter, MavrosParameterTransport
from test_wall_sensor_fusion import Harness, MemoryJournal


class AsyncJournalTests(unittest.TestCase):
    def record(self, operation):
        return dict(originals={FLOW: 1, RANGE: 1}, pending=dict(token=operation.token,
            parameter=operation.parameter, value=operation.value), desired='suspend')

    def test_blocked_fsync_does_not_block_policy_or_dispatch_write_early(self):
        entered, release = threading.Event(), threading.Event()
        disk = MemoryJournal()
        original_save = disk.save
        def slow_save(data):
            entered.set(); release.wait(2); original_save(data)
        disk.save = slow_save
        staged = StagedRestoreJournal(disk)
        calls = []
        worker = SerialParameterWorker(lambda op: calls.append(op) or {FLOW: 0},
            time.time, journal=disk)
        self.addCleanup(worker.close); self.addCleanup(release.set)
        op = Operation(1, 'set', 1, 10., 10., FLOW, 0)
        worker.submit(op, self.record(op))
        self.assertTrue(entered.wait(1)); self.assertEqual(calls, [])
        start = time.monotonic()
        h = Harness(journal=staged); h.start()
        for _ in range(20):h.feed(); h.tick(); h.status()
        self.assertLess(time.monotonic()-start, .1)
        release.set(); result = worker.results.get(timeout=1)
        self.assertTrue(result[1]); self.assertEqual(calls, [op])
        self.assertEqual(disk.data['pending']['token'], op.token)

    def test_disk_failure_never_changes_fcu(self):
        disk = MemoryJournal(); disk.fail_save = True; calls = []
        worker = SerialParameterWorker(lambda op: calls.append(op), time.time, journal=disk)
        self.addCleanup(worker.close)
        op = Operation(1, 'set', 1, 10., 10., FLOW, 0)
        worker.submit(op, self.record(op))
        self.assertFalse(worker.results.get(timeout=1)[1]); self.assertFalse(calls)

    def test_superseded_disable_is_not_sent_after_slow_save(self):
        disk = MemoryJournal(); calls = []
        entered, release, revoked = (threading.Event() for _ in range(3))
        original_save = disk.save
        def slow_save(data):
            entered.set()
            if not release.wait(3):
                raise RuntimeError('test did not release fake fsync')
            original_save(data)
        disk.save = slow_save
        worker = SerialParameterWorker(lambda op: calls.append(op), time.time,
            journal=disk, authorize=lambda op: not revoked.is_set())
        self.addCleanup(worker.close); self.addCleanup(release.set)
        op = Operation(1, 'set', 1, 10., 10., FLOW, 0)
        worker.submit(op, self.record(op))
        self.assertTrue(entered.wait(1))
        self.assertIsNone(disk.data); self.assertEqual(calls, [])
        revoked.set(); release.set()
        result = worker.results.get(timeout=1)
        self.assertFalse(result[1]); self.assertIn('superseded', result[-1]); self.assertFalse(calls)
        self.assertEqual(result[2], {'cancelled': True})
        self.assertIsNotNone(disk.data)  # conservative restart obligation retained
        self.assertEqual(disk.data['pending']['token'], op.token)

    def test_pre_save_cancellation_skips_disk_and_preserves_existing_obligation(self):
        for saved in (None, {'pending': 'previous unresolved write'}):
            with self.subTest(saved=saved):
                disk = MemoryJournal(saved=saved); calls = []
                worker = SerialParameterWorker(lambda op: calls.append(op), time.time,
                    journal=disk, authorize=lambda op: False)
                self.addCleanup(worker.close)
                op = Operation(1, 'set', 1, 10., 10., FLOW, 0)
                worker.submit(op, self.record(op))
                result = worker.results.get(timeout=1)
                self.assertFalse(result[1])
                self.assertEqual(result[2], {'cancelled': True})
                self.assertEqual(calls, [])
                self.assertEqual(disk.saves, [])
                self.assertEqual(disk.data, saved)

    def test_verify_result_waits_for_durable_clear_and_mismatch_retains_journal(self):
        for matches in (True, False):
            disk = MemoryJournal(saved={'pending': 'old'})
            entered, release = threading.Event(), threading.Event()
            original = disk.clear
            def clear():entered.set(); release.wait(2); original()
            disk.clear = clear
            values = {FLOW: 1, RANGE: 1 if matches else 0}
            worker = SerialParameterWorker(lambda op: values, time.time, journal=disk)
            self.addCleanup(worker.close); self.addCleanup(release.set)
            worker.submit(Operation(1, 'verify', 1, 10., 10.), originals={FLOW: 1, RANGE: 1})
            if matches:
                self.assertTrue(entered.wait(1)); self.assertTrue(worker.results.empty())
                release.set()
            result = worker.results.get(timeout=1); self.assertTrue(result[1])
            self.assertEqual(disk.data is None, matches)

    def test_persist_time_not_counted_as_wire_timeout(self):
        h = Harness(); h.start(); h.feed(state='WALL_HOLD', tilt=90.)
        op = h.tick(); h.policy.mark_operation_stage(op, 'persisting', h.now, h.mono)
        h.feed(dt=6.); self.assertIsNone(h.tick())
        self.assertNotIn('timed out', h.status()['reason'])
        h.policy.mark_operation_stage(op, 'executing', h.now, h.mono)
        h.feed(dt=5.1); h.tick(); self.assertIn('timed out', h.status()['reason'])

    def test_idle_after_verify_keeps_verified_snapshot(self):
        h = Harness(); h.suspend(); h.restore()
        h.feed(dt=1.201, state='IDLE'); op = h.tick(); self.assertEqual(op.kind, 'verify')
        h.complete(op); self.assertIsNone(h.tick())
        self.assertTrue(h.status()['control_ready']); self.assertTrue(h.status()['parameters_restored'])


def packet(name=FLOW, value=1, **kw):
    raw = struct.pack('<iHH16sB', value, 1000, 123, name.encode(), 6)
    padded = raw+b'\0'*((-len(raw)) % 8)
    data = dict(msgid=22, sysid=1, compid=1, framing_status=1, len=len(raw),
        payload64=struct.unpack('<{}Q'.format(len(padded)//8), padded),
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: time.monotonic())))
    data.update(kw); return types.SimpleNamespace(**data)


class NamedReadTests(unittest.TestCase):
    def transport(self):
        t = MavrosParameterTransport.__new__(MavrosParameterTransport)
        t.target_system=t.target_component=1;t.condition=threading.Condition();t.responses=[]
        t.timeout=.025;t.retries=0
        t.ros=types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda:
            types.SimpleNamespace(to_sec=time.monotonic)), is_shutdown=lambda: False)
        return t

    def test_px4_bytewise_int_decode_and_foreign_frames_rejected(self):
        self.assertEqual(decode_integer_parameter(packet(value=1),1,1),(FLOW,1))
        self.assertEqual(decode_integer_parameter(packet(value=0),1,1),(FLOW,0))
        for msg in [packet(sysid=2),packet(compid=2),packet(msgid=23),packet(framing_status=2),
                    packet(name='OTHER'),packet(value=-1)]:
            self.assertIsNone(decode_integer_parameter(msg,1,1))

    def test_only_post_dispatch_named_reply_is_accepted(self):
        t=self.transport()
        def publish(name, expected):
            t.responses.extend([(time.monotonic()-1,time.monotonic(),FLOW,7),
                (time.monotonic(),time.monotonic()-1,FLOW,8),
                (time.monotonic(),time.monotonic(),RANGE,9),
                (time.monotonic(),time.monotonic(),FLOW,1)])
        t._publish=publish
        self.assertEqual(t._exchange(FLOW),1)

    def test_cached_old_missing_or_wrong_write_value_times_out(self):
        for wrong in ('old','missing','wrong'):
            t=self.transport()
            def publish(name,expected):
                if wrong=='old':t.responses.append((time.monotonic()-1,time.monotonic()-1,FLOW,1))
                if wrong=='wrong':t.responses.append((time.monotonic(),time.monotonic(),FLOW,0))
            t._publish=publish
            with self.assertRaises(RuntimeError):t._exchange(FLOW,1)

    def test_retry_is_bounded_and_valid_callback_wakes_exchange(self):
        t=self.transport();t.retries=1;calls=[]
        def publish(name,expected):
            calls.append(name)
            if len(calls)==2:
                threading.Timer(.005,lambda:t._on_message(packet())).start()
        t._publish=publish
        self.assertEqual(t._exchange(FLOW),1);self.assertEqual(len(calls),2)


if __name__=='__main__':unittest.main()
