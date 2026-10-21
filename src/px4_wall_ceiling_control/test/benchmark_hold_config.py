"""Offline HOLD timing using real node logic, mocked ROS I/O and source clock.

Run inside ros1_noetic after sourcing the workspace. Never starts a ROS node,
connects a publisher/service, or contacts the parameter server. An optional
per-read delay models parameter-server latency; it is not a hardware measurement.
"""
import argparse
import json
import math
import platform
import time
from unittest import mock

import rospy
from px4_wall_ceiling_control.msg import WallPerchStatus as W
from test_hold_config_latency import HoldConfigLatencyTests


class Sink:
    def __init__(self):
        self.last = None
        self.count = 0

    def publish(self, message):
        self.last = message
        self.count += 1


def benchmark(samples=3000, warmup=100, parameter_delay_ms=0.):
    fixture = HoldConfigLatencyTests()
    try:
        fixture.setUp()
        node = fixture.n
        node.status_publisher = Sink()
        node.control_candidate_publisher = Sink()
        reads = 0

        def get_param(key, default=None):
            nonlocal reads
            reads += 1
            if parameter_delay_ms:
                time.sleep(parameter_delay_ms / 1000.)
            return fixture.params.get(key, default)

        durations = []
        with mock.patch.object(rospy, 'get_param', new=get_param):
            for i in range(warmup+samples):
                # Fresh feedback, but high angular rate blocks detach readiness:
                # this exercises the historical 78-read HOLD status path.
                fixture.refresh(10.+i*.02, rate=30.)
                started = time.perf_counter_ns()
                node._on_timer(None)
                elapsed_ms = (time.perf_counter_ns()-started)/1.e6
                if node.state != W.STATE_WALL_HOLD or not node.control_candidate_publisher.last.valid:
                    raise AssertionError('benchmark left HOLD or stopped publishing valid candidates')
                if i < warmup:
                    reads = 0
                else:
                    durations.append(elapsed_ms)
        ordered = sorted(durations)
        return dict(samples=samples, warmup=warmup, machine=platform.machine(),
                    python=platform.python_version(), simulated_parameter_delay_ms=parameter_delay_ms,
                    parameter_reads=reads, parameter_reads_per_cycle=reads/samples,
                    median_ms=(ordered[(samples-1)//2]+ordered[samples//2])/2.,
                    p95_ms=ordered[math.ceil(.95*samples)-1], max_ms=max(ordered),
                    candidate_count=node.control_candidate_publisher.count-warmup,
                    timing_scope='full _on_timer; mocked ROS I/O; excludes feedback injection')
    finally:
        fixture.doCleanups()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=3000)
    parser.add_argument('--warmup', type=int, default=100)
    parser.add_argument('--parameter-delay-ms', type=float, default=0.)
    args = parser.parse_args()
    if args.samples < 1 or args.warmup < 0 or not math.isfinite(args.parameter_delay_ms) or args.parameter_delay_ms < 0:
        parser.error('samples must be positive; warmup and parameter delay must be nonnegative')
    print(json.dumps(benchmark(args.samples, args.warmup, args.parameter_delay_ms), indent=2))
