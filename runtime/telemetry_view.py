#!/usr/bin/env python3
"""Read-only MAVROS telemetry bridge and local chart page."""
import collections
import datetime
import json
import math
import pathlib
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import rospy
from mavros_msgs.msg import State, RCIn
from sensor_msgs.msg import Imu, BatteryState, Range
from nav_msgs.msg import Odometry
from px4_wall_ceiling_control.msg import SensorHealth

ROOT = pathlib.Path(__file__).resolve().parent
lock = threading.Lock()
latest = {}
arrivals = collections.defaultdict(lambda: collections.deque(maxlen=1000))
samples = collections.deque(maxlen=600)
start = time.monotonic()

def euler(q):
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-9:
        return None
    x, y, z, w = (q.x/norm, q.y/norm, q.z/norm, q.w/norm)
    return [math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))),
            math.degrees(math.asin(max(-1, min(1, 2*(w*y-z*x))))),
            math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))]

def vector(v):
    return [v.x, v.y, v.z]

def callback(msg, name):
    if name == 'state':
        value = dict(connected=msg.connected, armed=msg.armed, mode=msg.mode)
    elif name == 'imu':
        value = dict(attitude=euler(msg.orientation), accel=vector(msg.linear_acceleration),
                     gyro=[math.degrees(v) for v in vector(msg.angular_velocity)], frame=msg.header.frame_id)
    elif name == 'odom':
        value = dict(position=vector(msg.pose.pose.position), velocity=vector(msg.twist.twist.linear),
                     frame=msg.header.frame_id, child_frame=msg.child_frame_id)
    elif name == 'battery':
        value = dict(voltage=msg.voltage, current=msg.current, percentage=msg.percentage, present=msg.present)
    elif name == 'rc':
        value = dict(channels=list(msg.channels), rssi=msg.rssi)
    elif name == 'sensor_health':
        value = dict(com4_valid=msg.ceiling_valid, com5_valid=msg.front_valid,
                     com4_age_s=msg.ceiling_age_s, com5_age_s=msg.front_age_s,
                     com4_sequence=msg.ceiling_sequence, com5_sequence=msg.front_sequence,
                     com4_invalid_count=msg.ceiling_invalid_count, com5_invalid_count=msg.front_invalid_count)
    else:
        value = dict(range_m=msg.range, frame=msg.header.frame_id, min_range=msg.min_range, max_range=msg.max_range)
    with lock:
        now = time.monotonic()
        latest[name] = dict(value=value, received=now, stamp=msg.header.stamp.to_sec())
        arrivals[name].append(now)

def clean(value):
    if isinstance(value, float):
        return round(value, 5) if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, collections.deque)):
        return [clean(v) for v in value]
    return value

def snapshot(include_samples=True):
    with lock:
        now = time.monotonic()
        status = {}
        for name in TYPES:
            item = latest.get(name)
            times = [t for t in arrivals[name] if now-t < 5]
            hz = (len(times)-1)/(times[-1]-times[0]) if len(times) > 1 else 0
            age = now-item['received'] if item else None
            status[name] = dict(age_s=age, hz=hz, value=item['value'] if item else None,
                                source_stamp_ms=item['stamp']*1000 if item else None,
                                fresh=age is not None and age < (3 if name in ('state', 'battery', 'rc') else 1))
        rows=list(samples) if include_samples else None
    # Serialize outside the callback lock so chart requests cannot block ROS reception.
    result=dict(time=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),
                server_time_ms=time.time()*1000, duration_s=now-start, status=status)
    if rows is not None: result['samples']=rows
    return clean(result)

def sample_loop():
    while not rospy.is_shutdown():
        with lock:
            now = time.monotonic()
            row = dict(t=now-start)
            for name, fields in [('imu', ['attitude', 'accel', 'gyro']), ('odom', ['position', 'velocity'])]:
                item = latest.get(name)
                for field in fields:
                    row[field] = item['value'][field] if item and now-item['received'] < 1 else None
            row['distance'] = []
            health = latest.get('sensor_health')
            for name in ('com4', 'com5'):
                item = latest.get(name)
                valid = bool(health and now-health['received'] < 1 and health['value'][name+'_valid'])
                row['distance'].append(item['value']['range_m'] if item and valid and now-item['received'] < .6 else None)
            samples.append(row)
        time.sleep(0.1)

class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    def setup(self):
        super().setup()
        self.connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
    def do_GET(self):
        if self.path in ('/data', '/latest'):
            payload = json.dumps(snapshot(include_samples=self.path=='/data'), ensure_ascii=False, allow_nan=False).encode('utf-8')
            mime = 'application/json; charset=utf-8'
        elif self.path in ('/', '/index.html'):
            page = ROOT / 'telemetry_live.html'
            if not page.exists():
                self.send_error(503, 'Chart page is being prepared')
                return
            payload = page.read_bytes()
            mime = 'text/html; charset=utf-8'
        elif self.path == '/d3.min.js':
            payload = (ROOT/'d3.min.js').read_bytes()
            mime = 'application/javascript; charset=utf-8'
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', mime)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)
    def log_message(self, *args):
        pass

TYPES = {'state': ('/mavros/state', State), 'imu': ('/mavros/imu/data', Imu),
         'odom': ('/mavros/local_position/odom', Odometry), 'battery': ('/mavros/battery', BatteryState),
         'rc': ('/mavros/rc/in', RCIn), 'ceiling': ('/sensor/ceiling/raw', Range),
         'front': ('/sensor/front/raw', Range), 'com4': ('/sensor/com4/raw', Range),
         'com5': ('/sensor/com5/raw', Range), 'sensor_health': ('/sensor/usb_health', SensorHealth)}
if __name__ == '__main__':
    rospy.init_node('px4_telemetry_view', anonymous=False, disable_signals=True)
    subscriptions = [rospy.Subscriber(topic, cls, callback, callback_args=name, queue_size=1, tcp_nodelay=True) for name, (topic, cls) in TYPES.items()]
    threading.Thread(target=sample_loop, daemon=True).start()
    print('PX4 telemetry charts: http://127.0.0.1:8765 (read only)', flush=True)
    ThreadingHTTPServer(('127.0.0.1', 8765), Handler).serve_forever()
