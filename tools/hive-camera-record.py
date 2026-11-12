#!/usr/bin/env python3
"""Read-only Mac/Limelight recorder; standard Python 3, no packages, no robot control.
A printout tests transport/visibility, never validates a real-hive readiness profile.
"""
import argparse, csv, json, math, time, urllib.request
from pathlib import Path


def selected(result, pipeline, tag):
    if result.get('pID') != pipeline or result.get('v') != 1:
        raise ValueError('wrong pipeline / no valid target')
    matches = [f for f in result.get('Fiducial', []) if f.get('fID') == tag]
    if len(matches) != 1:
        raise ValueError('selected tag absent or duplicated')
    pose = matches[0].get('t6t_cs', [])
    if len(pose) != 6 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose):
        raise ValueError('missing/nonfinite camera pose; enable 3-D output')
    stamp = result.get('ts', 0)
    if not isinstance(stamp, (float, int)) or not math.isfinite(stamp) or stamp <= 0 or not 0.05 < pose[2] < 15:
        raise ValueError('invalid camera timestamp / depth')
    return stamp, pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='limelight.local', help='camera hostname or USB IP, no port')
    parser.add_argument('--pipeline', type=int, default=0)
    parser.add_argument('--tag', type=int, required=True)
    parser.add_argument('--label', default='paper-test')
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if not 0 < args.seconds <= 600:
        parser.error('--seconds must be 0..600')
    url = 'http://' + args.host + ':5807/results'
    # Bypass any corporate HTTP proxy for the directly connected camera.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    rawpath = args.out.with_suffix('.ndjson')
    if args.out.exists() or rawpath.exists():
        parser.error('Output already exists; choose a new filename')
    count = 0
    last_stamp = None
    started = last_report = time.monotonic()
    with args.out.open('x', newline='') as out, rawpath.open('x') as raw:
        writer = csv.writer(out)
        writer.writerow(['host_elapsed_s', 'label', 'pipeline', 'tag', 'camera_timestamp', 'x_m', 'y_m', 'z_m', 'status'])
        while time.monotonic() - started < args.seconds:
            elapsed = time.monotonic() - started
            try:
                with opener.open(url, timeout=1) as response:
                    payload = json.load(response)
                result = payload.get('Results', payload)
                raw.write(json.dumps({'host_elapsed_s': elapsed, 'label': args.label, 'result': result}) + '\n')
                stamp, pose = selected(result, args.pipeline, args.tag)
                if stamp == last_stamp:
                    status = 'repeated camera frame'
                else:
                    last_stamp = stamp
                    count += 1
                    status = 'selected tag visible (NOT a readiness judgment)'
                    writer.writerow([elapsed, args.label, args.pipeline, args.tag, stamp, *pose[:3], 'valid'])
            except Exception as ex:
                status = str(ex)
                writer.writerow([elapsed, args.label, args.pipeline, args.tag, '', '', '', '', status])
            if time.monotonic() - last_report >= 1:
                print(f'{elapsed:5.1f}s frames={count}: {status}', flush=True)
                last_report = time.monotonic()
            time.sleep(.05)
    print(f'Saved {count} distinct usable frames to {args.out}; raw JSON: {rawpath}')


if __name__ == '__main__':
    main()
