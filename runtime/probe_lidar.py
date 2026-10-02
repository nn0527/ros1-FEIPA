"""Short independent UART receive test using the original L1 decoder."""
import concurrent.futures
import importlib.util
import json
import pathlib
import statistics
import time
import serial

spec = importlib.util.spec_from_file_location('original_lidar', '/home/wallrobot/catkin_ws_original/src/px4_wall_ceiling_control/scripts/sensors/lidar_distance_node.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

def probe(port):
    records, errors, raw_examples = [], [], []
    start = time.monotonic()
    with serial.Serial(port, 38400, timeout=0.2) as device:
        device.reset_input_buffer()
        device.write(b'iACM')
        device.flush()
        while time.monotonic()-start < 10:
            raw = device.readline()
            if not raw:
                continue
            if len(raw_examples) < 5:
                raw_examples.append(raw.decode('ascii', errors='replace').strip())
            try:
                distance = module.LidarDistanceNode.decode_l1_ascii(raw)
                records.append((time.monotonic(), distance))
            except (UnicodeError, ValueError) as exc:
                errors.append(str(exc))
    values = [value for _, value in records]
    return dict(port=port, baudrate=38400, duration_s=time.monotonic()-start,
                messages=len(records), hz=(len(records)-1)/(records[-1][0]-records[0][0]) if len(records)>1 else 0,
                min_m=min(values) if values else None, max_m=max(values) if values else None,
                mean_m=statistics.mean(values) if values else None,
                std_m=statistics.pstdev(values) if values else None,
                last_m=values[-1] if values else None, invalid_frames=len(errors),
                errors=errors[:5], raw_examples=raw_examples)

with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    results = list(pool.map(probe, ['/dev/ttyUSB0','/dev/ttyUSB1']))
report = dict(channels=results)
pathlib.Path(__file__).with_name('lidar_serial_test.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
