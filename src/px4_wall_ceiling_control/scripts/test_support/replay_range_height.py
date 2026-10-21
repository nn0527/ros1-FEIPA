#!/usr/bin/env python3
"""Replay only range selection/geometry/velocity from recorded ROS1 bags.

Requires rosbags in the analysis Python environment, never starts ROS nodes.
The rotation source start time must be supplied from the recorded transition.
This cannot validate flight dynamics, PX4 takeover or actual hardware sample time.
"""
import argparse
import json
import math
from pathlib import Path

from px4_wall_ceiling_control.domain.range_height_feedback import RangeHeightConfig, RangeHeightFeedback


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1.e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', nargs='+', type=Path)
    parser.add_argument('--rotation-source-time', type=float, required=True)
    parser.add_argument('--entry-tilt-deg', type=float, default=10.)
    parser.add_argument('--operating-tilt-deg', type=float, default=15.)
    parser.add_argument('--device-min-m', type=float, required=True)
    parser.add_argument('--device-max-m', type=float, required=True)
    parser.add_argument('--sensor-axis-body', nargs=3, type=float, default=(0., 0., -1.))
    parser.add_argument('--reference-offset-body-m', nargs=3, type=float, default=(0., 0., 0.))
    args = parser.parse_args()
    from rosbags.highlevel import AnyReader
    tracker = RangeHeightFeedback(RangeHeightConfig(entry_tilt_deg=args.entry_tilt_deg,
        operating_tilt_deg=args.operating_tilt_deg, min_distance_m=args.device_min_m,
        max_distance_m=args.device_max_m, geometry_confirmed=True,
        sensor_axis_body=tuple(args.sensor_axis_body), reference_offset_body_m=tuple(args.reference_offset_body_m)))
    tracker.begin(1, args.rotation_source_time)
    poses, ranges, deltas = 0, 0, set()
    with AnyReader(args.bags) as reader:
        topics = ('/mavros/local_position/odom', '/mavros/px4flow/raw/optical_flow_rad')
        for connection, received_ns, raw in reader.messages(connections=[c for c in reader.connections if c.topic in topics]):
            received = received_ns*1.e-9
            if received < args.rotation_source_time-1.:
                continue
            # Select this departure, not a later task in the same session.
            if received > args.rotation_source_time+30.:
                break
            message = reader.deserialize(raw, connection.msgtype)
            source = seconds(message.header.stamp)
            if connection.topic.endswith('/odom'):
                q = message.pose.pose.orientation
                tracker.observe_pose(source, received, (q.x, q.y, q.z, q.w))
                poses += 1
            else:
                tracker.observe_range(source, received, float(message.distance))
                deltas.add(int(message.time_delta_distance_us))
                ranges += 1
    clean = lambda v: v if v is None or math.isfinite(v) else None
    result = dict(bags=[str(p) for p in args.bags], rotation_source_time=args.rotation_source_time,
        entry_tilt_deg=args.entry_tilt_deg, target_locked=tracker.locked,
        lock_source_time=tracker.lock_source, lock_received_time=tracker.lock_received,
        first_raw_m=clean(tracker.lock_raw), first_height_m=clean(tracker.lock_height),
        first_sample_tilt_deg=clean(tracker.lock_tilt), target_m=clean(tracker.target),
        accepted_range_messages=tracker.count, pose_messages=poses, range_messages=ranges,
        time_delta_distance_us_values=sorted(deltas), rejections=tracker.rejected,
        timestamp_semantics='first qualified new message; independent hardware range timestamp is not established')
    print(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))
    return 0 if tracker.locked else 1


if __name__ == '__main__':
    raise SystemExit(main())
