#!/usr/bin/env python3
"""Summarize Hub benchmark JSONL without third-party packages. Raw logs remain authoritative."""
import argparse
import collections
import csv
import json
import math
from pathlib import Path
import statistics


def number(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def summarize(start, end, samples, session, source, complete):
    duration = max(0, end - start['t'])
    fresh = [s for s in samples if s.get('distinct')]
    hits = [s for s in fresh if s.get('raw_cell_hit')]
    accepted = [s for s in fresh if s.get('accepted')]
    hit_times = [start['t']] + [s['t'] for s in accepted] + [end]
    heights = [t['height_in'] for s in accepted for t in s.get('tags', [])
               if t.get('our_cell') and t.get('inside_side_gate') and number(t.get('height_in'))]
    times = [s['t'] for s in samples]
    loop_gaps = [b - a for a, b in zip([start['t']] + times, times + [end])]
    ready_seconds = covered = 0
    for i, s in enumerate(samples):
        until = samples[i+1]['t'] if i+1 < len(samples) else end
        # Conservative sampled estimate: never extrapolate an observation over a long loop stall.
        dt = max(0, min(.05, until - s['t']))
        covered += dt
        if s.get('state') == 'UP':
            ready_seconds += dt
    errors = collections.Counter(s.get('decode_error') for s in samples if s.get('decode_error'))
    states = collections.Counter(s.get('state', 'UNKNOWN') for s in fresh)
    gaps = [b-a for a, b in zip(hit_times, hit_times[1:])]
    up_times = [s['t'] for s in samples if s.get('state') == 'UP']
    pct = lambda a, b: round(100 * a / b, 2) if b else ''
    return {
        'file': source, 'trial': start['trial'], 'scenario': session.get('scenario', ''),
        'cell': session.get('cell'), 'pipeline': start['pipeline'], 'pass': start['pass'],
        'label': start['expected'], 'complete_trial': complete, 'seconds': round(duration, 3),
        'polls': len(samples), 'fresh_frames': len(fresh), 'fresh_fps': round(len(fresh)/duration, 2) if duration else 0,
        'raw_cell_hit_pct': pct(len(hits), len(fresh)), 'usable_cell_pct': pct(len(accepted), len(fresh)),
        'usable_fps': round(len(accepted)/duration, 2) if duration else 0,
        'max_usable_gap_s': round(max(gaps, default=duration), 3),
        'ready_time_pct_est': pct(ready_seconds, duration), 'observed_time_pct': pct(covered, duration),
        'first_ready_s': round(up_times[0]-start['t'], 3) if up_times else '',
        'false_up_polls': len(up_times) if start['expected'] == 'DOWN' else 0,
        'fresh_down_frames': states['DOWN'], 'fresh_unknown_frames': states['UNKNOWN'],
        'height_mean_in': round(statistics.mean(heights), 3) if heights else '',
        'height_sd_in': round(statistics.stdev(heights), 3) if len(heights)>1 else '',
        'max_poll_gap_s': round(max(loop_gaps, default=duration), 3),
        'decode_error_polls': sum(errors.values()), 'rejections': json.dumps(errors, sort_keys=True),
    }


def read_run(path):
    rows, samples, session, warnings = [], [], {}, []
    active = None
    last_t = 0
    stopped = False
    with path.open() as f:
        for lineno, line in enumerate(f, 1):
            try:
                record = json.loads(line)
            except (ValueError, TypeError):
                warnings.append(f'{path.name}:{lineno}: incomplete/invalid JSON line')
                continue
            event = record.get('event')
            last_t = record.get('t', last_t)
            if event == 'session':
                session = record
            elif event == 'record_start':
                if active:
                    rows.append(summarize(active, last_t, samples, session, path.name, False))
                active, samples = record, []
            elif event == 'sample' and record.get('phase') == 'RECORD' and active:
                samples.append(record)
            elif event == 'record_end' and active:
                rows.append(summarize(active, last_t, samples, session, path.name, True))
                active = None
            elif event == 'stop':
                stopped = True
                if not record.get('complete'):
                    warnings.append(f'{path.name}: session stopped early or cancelled; do not promote from this run alone')
                if 'FAILED' in record.get('restore', ''):
                    warnings.append(f'{path.name}: camera pipeline restore failed')
    if active:
        rows.append(summarize(active, last_t, samples, session, path.name, False))
    if not stopped:
        warnings.append(f'{path.name}: no STOP record; file may be truncated')
    return rows, warnings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--out', type=Path, default=Path('hive-analysis'))
    args = parser.parse_args()
    paths = sorted(args.folder.rglob('hive-*.jsonl')) if args.folder.is_dir() else [args.folder]
    rows, warnings = [], []
    for path in paths:
        r, w = read_run(path)
        rows.extend(r); warnings.extend(w)
    if not rows:
        parser.error('No recorded trials found. Check the downloaded benchmark folder.')
    args.out.mkdir(parents=True, exist_ok=False)
    with (args.out/'trials.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    lines = ['# Hive benchmark results', '',
             'Compare UP availability, usable frames/second and gaps; reject any false-UP result in a settled DOWN trial.',
             'These are labeled static trials, not certification of tipping/shooting behavior. Do not pool different cells or scenarios.', '',
             '| File / trial | Pipeline | Label | Fresh fps | Raw hit % | Usable % | Longest gap s | Ready time %* | False UP polls |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for r in rows:
        lines.append('| ' + ' | '.join((str(r[k]) + ' / ' + str(r['trial']) if k == 'file' else str(r[k])) for k in ['file', 'pipeline', 'label', 'fresh_fps',
                     'raw_cell_hit_pct', 'usable_cell_pct', 'max_usable_gap_s', 'ready_time_pct_est', 'false_up_polls']) + ' |')
    lines += ['', '*Readiness time is a conservative sampled estimate; a sample covers at most 50 ms. '
              'See observed_time_pct and max_poll_gap_s in CSV for logging stalls. '
              'Raw-hit denominator is distinct received frames from the requested pipeline, including invalid/empty results. '
              'No fresh frames produces blank percentages, not a passing result. Gaps include the beginning and end of each trial.', '',
              'Height SD pools accepted tag measurements; use raw per-ID poses to distinguish tag disagreement from jitter. '
              'A hidden DOWN tag can correctly block shooting while the detector reports UNKNOWN.', '', '## Warnings', '']
    lines += warnings or ['No file-integrity warnings. Still verify calibration, labels and pipeline configuration.']
    (args.out/'summary.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))
    print(f'\nSaved {args.out}/trials.csv and summary.md. Upload original JSONL and pipeline exports too.')


if __name__ == '__main__':
    main()
