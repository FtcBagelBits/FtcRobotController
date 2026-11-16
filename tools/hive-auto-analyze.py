#!/usr/bin/env python3
"""Summarize v7 traces without changing raw evidence. Numbers are observations, not diagnoses."""
import argparse
import csv
from pathlib import Path


def summarize(folder):
    lines = ['# Hive auto traces', '', 'Power readbacks are SDK values, not independent electrical measurements.', '']
    paths = sorted((folder / 'traces').glob('*.csv'))
    if not paths:
        return '\n'.join(lines + ['No v7 traces found. Older runs need reports and SDK logs.', ''])
    for path in paths:
        with path.open(newline='') as f:
            rows = list(csv.DictReader(f))
        lines.extend(['## ' + path.name, ''])
        if not rows:
            lines.append('No samples. Inspect the matching report for a start fault.')
            continue
        lines.extend(['| Time (s) | Phase | Reason |', '| --- | --- | --- |'])
        previous = None
        for row in rows:
            phase = row.get('phase', '')
            if phase != previous or row.get('kind') in ('STOP', 'FAULT'):
                reason = (row.get('reason') or '').replace('|', '/')
                lines.append(f"| {row.get('t_s', '?')} | {phase} | {reason} |")
                previous = phase
        coast = [r for r in rows if 'evaluated=COAST ' in (r.get('decision_inputs') or '')]
        if coast:
            speeds = [abs(float(r['decision_launcher_tps'])) for r in coast]
            lines.extend(['', f'Legacy COAST sampled speed range: {min(speeds):.1f}–{max(speeds):.1f} ticks/s.',
                          'Legacy COAST decision: `' + coast[-1]['decision_inputs'] + '`'])
        lines.extend(['', f'{len(rows)} samples. Last record: {rows[-1].get("kind", "?")}.', ''])
    return '\n'.join(lines)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    args = parser.parse_args()
    print(summarize(args.folder))
