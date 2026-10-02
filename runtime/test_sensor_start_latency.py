"""Measure command-to-first-frame latency, never infer target-step latency."""
import concurrent.futures
import json
import math
import pathlib
import re
import statistics
import time
import serial

def read_frames(device, state):
    raw=device.read(max(1,device.in_waiting))
    state['buffer']+=raw
    frames=[]
    while b'#' in state['buffer'] or b'\n' in state['buffer']:
        ends=[i for i in (state['buffer'].find(b'#'),state['buffer'].find(b'\n')) if i>=0]
        end=min(ends)+1
        text=state['buffer'][:end].strip().decode('ascii',errors='replace')
        state['buffer']=state['buffer'][end:]
        if text:frames.append((time.perf_counter(),text))
    return frames

def distance(text):
    match=re.fullmatch(r'D=([+-]?(?:\d+(?:\.\d*)?|\.\d+))m(?:,\d+)?#?',text)
    return float(match.group(1)) if match else None

def run(port):
    cycles=[]
    with serial.Serial(port,38400,timeout=.005,write_timeout=1) as device:
        state={'buffer':b''}
        try:
            for cycle in range(30):
                stopped=time.perf_counter()
                device.write(b'iHALT');device.flush()
                last=stopped;stop_responses=[];remaining=0
                while time.perf_counter()-stopped<3:
                    for at,text in read_frames(device,state):
                        if distance(text) is not None:last=at;remaining+=1
                        else:stop_responses.append(text)
                    if time.perf_counter()-last>=.5:break
                quiet=time.perf_counter()-last
                if quiet<.5:
                    cycles.append(dict(cycle=cycle+1,stop_confirmed=False,stop_responses=stop_responses))
                    break
                # No stale complete measurement for 500 ms. Do not flush away the first response.
                state['buffer']=b''
                device.reset_input_buffer()
                send_begin=time.perf_counter()
                device.write(b'iACM');device.flush()
                send_end=time.perf_counter()
                first=None;responses=[]
                while time.perf_counter()-send_end<2:
                    for at,text in read_frames(device,state):
                        value=distance(text)
                        if value is not None and first is None:first=(at,value,text)
                        elif value is None:responses.append(text)
                    if first is not None:break
                cycles.append(dict(cycle=cycle+1,stop_confirmed=True,quiet_ms=quiet*1000,
                    residual_stop_frames=remaining,stop_responses=stop_responses,start_responses=responses,
                    host_write_ms=(send_end-send_begin)*1000,
                    command_begin_to_first_ms=(first[0]-send_begin)*1000 if first else None,
                    command_flush_to_first_ms=(first[0]-send_end)*1000 if first else None,
                    first_distance_m=first[1] if first else None,first_frame=first[2] if first else None))
                # Continue measuring briefly before the next stop so the two commands are separated.
                settle=time.perf_counter()
                while time.perf_counter()-settle<.25:read_frames(device,state)
        finally:
            device.write(b'iACM');device.flush()
    values=sorted(c['command_begin_to_first_ms'] for c in cycles if c.get('command_begin_to_first_ms') is not None)
    return dict(port=port,cycles=cycles,successes=len(values),attempts=len(cycles),
                p50_ms=statistics.median(values) if values else None,
                p95_ms=values[min(len(values)-1,math.ceil(.95*len(values))-1)] if values else None,
                min_ms=min(values) if values else None,max_ms=max(values) if values else None)

with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    rows=list(pool.map(run,['/dev/ttyUSB0','/dev/ttyUSB1']))
report=dict(test='iHALT -> 500ms without measurements -> iACM -> first valid frame',
            clock='host perf_counter; includes USB/IP and UART transport, not physical target-step latency',channels=rows)
pathlib.Path(__file__).with_name('sensor_start_latency.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
for row in rows:
    row['example_cycle']=row['cycles'][0] if row['cycles'] else None
    row.pop('cycles')
print(json.dumps(report,indent=2))
