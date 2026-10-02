#!/usr/bin/env python3
"""Read ROS inputs for 15 seconds; never publish or invoke FCU services."""
import json
import time
import rospy
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu, Range
from nav_msgs.msg import Odometry

rospy.init_node('usb_input_check', anonymous=True)
received = {}
counts = {}

def receive(msg, topic):
    counts[topic] = counts.get(topic, 0) + 1
    received[topic] = {'received_at': time.monotonic()}
    if isinstance(msg, State):
        received[topic].update(connected=msg.connected, armed=msg.armed, mode=msg.mode)
    if isinstance(msg, Range):
        received[topic].update(range_m=msg.range, frame=msg.header.frame_id)

topics = {'/mavros/state': State, '/mavros/imu/data': Imu,
          '/mavros/local_position/odom': Odometry,
          '/sensor/ceiling/raw': Range, '/sensor/front/raw': Range}
subscribers = [rospy.Subscriber(topic, cls, receive, callback_args=topic) for topic, cls in topics.items()]
deadline = time.monotonic() + 15
while not rospy.is_shutdown() and time.monotonic() < deadline:
    time.sleep(0.1)
now = time.monotonic()
report = {}
for topic in topics:
    entry = dict(received.get(topic, {}))
    if 'received_at' in entry:
        entry['age_s'] = round(now - entry.pop('received_at'), 2)
    entry['messages'] = counts.get(topic, 0)
    report[topic] = entry
print(json.dumps(report, indent=2, ensure_ascii=False))
state = report['/mavros/state']
raise SystemExit(0 if state.get('connected') and state.get('age_s', 999) < 3 else 2)
