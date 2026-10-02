import concurrent.futures
import json
import pathlib
import re
import statistics
import time
import serial
import argparse
parser=argparse.ArgumentParser()
parser.add_argument('--ports',nargs=2,default=['/dev/ttyUSB0','/dev/ttyUSB1'])
parser.add_argument('--label',default='baseline')
args=parser.parse_args()

def measure(port):
    chunks=[];frames=[];buf=b'';start=time.monotonic()
    with serial.Serial(port,38400,timeout=.02) as device:
        while time.monotonic()-start<20:
            raw=device.read(max(1,device.in_waiting))
            if not raw:continue
            at=time.monotonic()-start
            chunks.append(dict(t=at,raw=raw.decode('ascii',errors='backslashreplace')))
            buf+=raw
            while b'#' in buf or b'\n' in buf:
                ends=[i for i in (buf.find(b'#'),buf.find(b'\n')) if i>=0]
                end=min(ends)+1
                frame,buf=buf[:end],buf[end:]
                text=frame.strip()
                if not text:continue
                frames.append(dict(t=at,raw=text.decode('ascii',errors='backslashreplace')))
    valid=[f for f in frames if re.fullmatch(r'D=[+-]?[0-9.]+m(?:,\d+)?#?',f['raw'])]
    gaps=[(b['t']-a['t'])*1000 for a,b in zip(valid,valid[1:])]
    return dict(port=port,duration_s=time.monotonic()-start,valid_frames=len(valid),
                gap_median_ms=statistics.median(gaps) if gaps else None,max_gap_ms=max(gaps) if gaps else None,
                frame_examples=frames[:5],newline_chunks=sum('\n' in c['raw'] for c in chunks),
                hash_chunks=sum('#' in c['raw'] for c in chunks),chunks=chunks,frames=frames)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
    result=list(executor.map(measure,args.ports))
pathlib.Path(__file__).with_name('wire_'+args.label+'.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
for row in result:
    row.pop('chunks');row.pop('frames')
print(json.dumps(result,indent=2))
