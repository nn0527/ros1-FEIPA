import argparse
import collections
import json
import math
import pathlib
import statistics
import threading
import time
import rospy
from sensor_msgs.msg import Range
from px4_wall_ceiling_control.msg import SensorHealth

parser=argparse.ArgumentParser()
parser.add_argument('label')
parser.add_argument('--seconds',type=float,default=20)
args=parser.parse_args()
rospy.init_node('sensor_pipeline_measure',anonymous=True)
lock=threading.Lock()
events=collections.defaultdict(list)
health=[]
def receive(msg,key):
    with lock:
        events[key].append(dict(at=time.monotonic(),stamp=msg.header.stamp.to_sec(),
                               age_ms=(rospy.Time.now()-msg.header.stamp).to_sec()*1000,range_m=msg.range))
def receive_health(msg):
    with lock:
        health.append(dict(com4=msg.ceiling_valid,com5=msg.front_valid))
subs=[]
for channel in ('com4','com5'):
    for stage in ('raw','range'):
        topic='/sensor/'+channel+'/'+stage
        subs.append(rospy.Subscriber(topic,Range,receive,callback_args=channel+'/'+stage,queue_size=1,tcp_nodelay=True))
subs.append(rospy.Subscriber('/sensor/usb_health',SensorHealth,receive_health,queue_size=1,tcp_nodelay=True))
time.sleep(args.seconds)
def stats(vals):
    if not vals:return None
    vals=sorted(vals)
    return dict(p50=statistics.median(vals),p95=vals[min(len(vals)-1,math.ceil(.95*len(vals))-1)],max=max(vals))
report=dict(label=args.label,seconds=args.seconds,channels={})
with lock:
    for channel in ('com4','com5'):
        raw=events[channel+'/raw']; forwarded=events[channel+'/range']
        delta=[(b['at']-a['at'])*1000 for a,b in zip(raw,raw[1:])]
        lookup={e['stamp']:e['at'] for e in raw}
        relay=[(e['at']-lookup[e['stamp']])*1000 for e in forwarded if e['stamp'] in lookup]
        report['channels'][channel]=dict(raw_count=len(raw),valid_count=len(forwarded),
            interarrival_ms=stats(delta),raw_ros_age_ms=stats([e['age_ms'] for e in raw]),
            forwarded_ros_age_ms=stats([e['age_ms'] for e in forwarded]),relay_callback_delta_ms=stats(relay),
            gaps_over_600ms=sum(d>600 for d in delta),
            health_valid_fraction=sum(e[channel] for e in health)/len(health) if health else None,
            ranges=stats([e['range_m'] for e in raw]))
    report['events']=dict(events)
path=pathlib.Path(__file__).with_name('pipeline_'+args.label+'.json')
path.write_text(json.dumps(report,indent=2),encoding='utf-8')
report.pop('events')
print(json.dumps(report,indent=2))
