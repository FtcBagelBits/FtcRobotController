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
    # The gate keeps UP for miss_hold seconds after the last frame that showed our tags, so this
    # is the most of the trial it could have stayed UP. It needs no calibration.
    hold = session.get('miss_hold', session.get('max_age', .2))
    held = sum(min(hold, b - a) for a, b in zip(hit_times[1:], hit_times[2:]))
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
        'hold_time_pct': pct(held, duration),
        'ready_time_pct_est': pct(ready_seconds, duration), 'observed_time_pct': pct(covered, duration),
        'first_ready_s': round(up_times[0]-start['t'], 3) if up_times else '',
        'false_up_polls': len(up_times) if start['expected'] == 'DOWN' else 0,
        'fresh_down_frames': states['DOWN'], 'fresh_unknown_frames': states['UNKNOWN'],
        'height_mean_in': round(statistics.mean(heights), 3) if heights else '',
        'height_sd_in': round(statistics.stdev(heights), 3) if len(heights)>1 else '',
        'max_poll_gap_s': round(max(loop_gaps, default=duration), 3),
        'decode_error_polls': sum(errors.values()), 'wrong_pipeline_polls': errors.get('Wrong pipeline', 0),
        'rejections': json.dumps(errors, sort_keys=True),
    }


def mean(values):
    values = [v for v in values if v != '']
    return round(statistics.mean(values), 1) if values else ''


def by_pipeline(rows):
    """One line per run and pipeline, best first: what to look at when choosing a pipeline."""
    groups = collections.OrderedDict()
    for r in rows:
        groups.setdefault((r['file'], r['pipeline']), []).append(r)
    out = []
    for (name, pipeline), trials in groups.items():
        up = [r for r in trials if r['label'] == 'UP']
        down = [r for r in trials if r['label'] == 'DOWN']
        false_up = sum(r['false_up_polls'] for r in down)
        problems = []
        if false_up:
            problems.append('REJECT: UP reported while DOWN')
        if any(r['fresh_frames'] == 0 for r in trials):
            problems.append('no camera frames')
        if any(r['wrong_pipeline_polls'] for r in trials):
            problems.append('camera on another pipeline')
        if not all(r['complete_trial'] for r in trials):
            problems.append('incomplete trial')
        out.append({
            'file': name, 'pipeline': pipeline, 'up_trials': len(up), 'down_trials': len(down),
            'up_fresh_fps': mean(r['fresh_fps'] for r in up), 'up_usable_fps': mean(r['usable_fps'] for r in up),
            'up_usable_pct': mean(r['usable_cell_pct'] for r in up),
            'up_worst_gap_s': max((r['max_usable_gap_s'] for r in up), default=''),
            'up_hold_pct': mean(r['hold_time_pct'] for r in up), 'up_ready_pct': mean(r['ready_time_pct_est'] for r in up),
            'down_usable_pct': mean(r['usable_cell_pct'] for r in down), 'down_false_up_polls': false_up,
            'notes': '; '.join(problems)})
    # Within a run: no false UP first, then the longest possible UP time (to the nearest percent),
    # then more usable frames per second.
    files = list(collections.OrderedDict.fromkeys(g['file'] for g in out))
    out.sort(key=lambda g: (files.index(g['file']), g['down_false_up_polls'] > 0,
                            -round(g['up_hold_pct'] or 0), -(g['up_usable_fps'] or 0)))
    return out


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
    groups = by_pipeline(rows)
    with (args.out/'pipelines.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(groups[0]))
        writer.writeheader(); writer.writerows(groups)
    runs = list(collections.OrderedDict.fromkeys(r['file'] for r in rows))
    run = lambda name: 'run ' + str(runs.index(name) + 1)
    lines = ['# Hive benchmark results', '',
             '## Pipelines, best first within each run', '',
             'Hold % = how much of the UP time was within the gate\'s hold time after a frame that showed our tags: '
             'the most the robot could have been allowed to shoot. It needs no calibration. '
             'Ready % = how much of the UP time the real gate said UP; it needs a calibrated camera tilt. '
             'Any UP while the cell was DOWN rejects a pipeline.', '',
             '| Run | Pipeline | UP hold % | UP ready % | UP usable fps | UP usable % | UP worst gap s | DOWN usable % | DOWN false UP | Notes |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for g in groups:
        lines.append('| ' + ' | '.join([run(g['file'])] + [str(g[k]) for k in [
            'pipeline', 'up_hold_pct', 'up_ready_pct', 'up_usable_fps', 'up_usable_pct', 'up_worst_gap_s',
            'down_usable_pct', 'down_false_up_polls', 'notes']]) + ' |')
    lines += [''] + [f'{run(name)} = {name}' for name in runs]
    lines += ['', '## Every trial', '',
              'Compare UP availability, usable frames/second and gaps; reject any false-UP result in a settled DOWN trial.',
              'These are labeled static trials, not certification of tipping/shooting behavior. Do not pool different cells or scenarios.', '',
              '| Run / trial | Pipeline | Label | Fresh fps | Raw hit % | Usable % | Longest gap s | Hold % | Ready time %* | False UP polls |',
              '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for r in rows:
        lines.append('| ' + ' | '.join((run(r[k]) + ' / ' + str(r['trial']) if k == 'file' else str(r[k])) for k in ['file', 'pipeline', 'label', 'fresh_fps',
                     'raw_cell_hit_pct', 'usable_cell_pct', 'max_usable_gap_s', 'hold_time_pct', 'ready_time_pct_est', 'false_up_polls']) + ' |')
        if r['wrong_pipeline_polls']:
            warnings.append(f"{r['file']} trial {r['trial']}: {r['wrong_pipeline_polls']} polls came from another pipeline; "
                            'do not compare this trial')
        if r['fresh_frames'] == 0:
            warnings.append(f"{r['file']} trial {r['trial']}: no camera frames from pipeline {r['pipeline']}")
    lines += ['', '*Readiness time is a conservative sampled estimate; a sample covers at most 50 ms. '
              'See observed_time_pct and max_poll_gap_s in CSV for logging stalls. '
              'Raw-hit denominator is distinct received frames from the requested pipeline, including invalid/empty results. '
              'No fresh frames produces blank percentages, not a passing result. Gaps include the beginning and end of each trial.', '',
              'Height SD pools accepted tag measurements; use raw per-ID poses to distinguish tag disagreement from jitter. '
              'A hidden DOWN tag can correctly block shooting while the detector reports UNKNOWN.', '', '## Warnings', '']
    lines += warnings or ['No file-integrity warnings. Still verify calibration, labels and pipeline configuration.']
    (args.out/'summary.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))
    print(f'\nSaved {args.out}/summary.md, pipelines.csv and trials.csv. Upload original JSONL and pipeline exports too.')


if __name__ == '__main__':
    main()
