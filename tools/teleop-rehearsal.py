#!/usr/bin/env python3
"""Turn rehearsals marked in PollenBot TeleOp recordings into draft autonomous waypoints.

In TeleOp, Circle marks the start of a rehearsal and Square marks its end (see
docs/POLLEN_LOGGING.md). This reads the recording(s), and for every rehearsal prints where the
robot went, measured from the spot where Circle was pressed, in the route directions every
autonomous uses: forward = toward the hive, left = left as seen looking at the hive, heading
counterclockwise. The driven path is reduced to a few waypoints, with what the intake and
launcher were doing on the way.

Usage: python3 tools/teleop-rehearsal.py PATH [--out FOLDER] [--tolerance-in 1.0] [--absolute]
PATH is one teleop CSV, a downloaded pollen-hive folder, or a downloaded session folder.

The output is a DRAFT made from one human drive. It is a starting point for route numbers,
not a tested route: check clearances, then test on the robot in DRIVE before LIVE.
"""
import argparse
import csv
import math
from pathlib import Path

CM = 2.54
INTAKE_ON = 0.1        # intake command above this counts as running
PAUSE_SPEED = 1.5      # inches per second: slower than this is "standing still"
PAUSE_SECONDS = 0.4    # standing still this long becomes a HOLD in the draft


def cm(inches):
    """Inches to centimetres with one decimal, and never "-0.0"."""
    return round(inches * CM, 1) + 0.0


def number(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return math.nan


def find_recordings(path):
    if path.is_file():
        return [path]
    for folder in (path / 'teleop', path / 'pollen-hive' / 'teleop', path):
        found = sorted(folder.glob('teleop-*.csv'))
        if found:
            return found
    return []


def wrap_degrees(angle):
    return (angle + 180) % 360 - 180


def relative_path(rows, absolute):
    """[(t, forward_in, left_in, heading_deg)] measured from the first row (or as recorded)."""
    x0, y0, h0 = (number(rows[0][k]) for k in ('x_in', 'y_in', 'heading_deg'))
    if absolute:
        x0 = y0 = h0 = 0.0
    cos, sin = math.cos(math.radians(h0)), math.sin(math.radians(h0))
    t0 = number(rows[0]['t_s'])
    points = []
    for row in rows:
        dx, dy = number(row['x_in']) - x0, number(row['y_in']) - y0
        points.append((number(row['t_s']) - t0, dx * cos + dy * sin, -dx * sin + dy * cos,
                       wrap_degrees(number(row['heading_deg']) - h0)))
    return points


def keep_by_distance(points, first, last, tolerance, keep):
    """Ramer-Douglas-Peucker: keep the point farthest from the straight line, if it is far enough."""
    if last <= first + 1:
        return
    (_, ax, ay, _), (_, bx, by, _) = points[first], points[last]
    length = math.hypot(bx - ax, by - ay)
    worst, index = 0.0, None
    for i in range(first + 1, last):
        _, px, py, _ = points[i]
        if length < 1e-9:
            distance = math.hypot(px - ax, py - ay)
        else:
            distance = abs((bx - ax) * (ay - py) - (ax - px) * (by - ay)) / length
        if distance > worst:
            worst, index = distance, i
    if index is not None and worst > tolerance:
        keep.add(index)
        keep_by_distance(points, first, index, tolerance, keep)
        keep_by_distance(points, index, last, tolerance, keep)


def pauses(points):
    """[(first_index, last_index)] where the robot stood still for PAUSE_SECONDS or longer."""
    found, start = [], None
    for i in range(1, len(points)):
        dt = points[i][0] - points[i - 1][0]
        speed = math.hypot(points[i][1] - points[i - 1][1], points[i][2] - points[i - 1][2]) / dt if dt > 0 else 0
        if speed < PAUSE_SPEED:
            start = i - 1 if start is None else start
        else:
            if start is not None and points[i - 1][0] - points[start][0] >= PAUSE_SECONDS:
                found.append((start, i - 1))
            start = None
    if start is not None and points[-1][0] - points[start][0] >= PAUSE_SECONDS:
        found.append((start, len(points) - 1))
    return found


def intervals(rows, t0, test):
    """[(start_s, end_s)] while test(row) is true."""
    found, start = [], None
    for row in rows:
        t = number(row['t_s']) - t0
        if test(row) and start is None:
            start = t
        elif not test(row) and start is not None:
            found.append((start, t))
            start = None
    if start is not None:
        found.append((start, number(rows[-1]['t_s']) - t0))
    return found


def waypoints(rows, points, tolerance):
    """Reduce the driven path to the points that matter: corners, pauses, intake on/off."""
    keep = {0, len(points) - 1}
    intake = [number(r['intake_cmd']) > INTAKE_ON for r in rows]
    keep.update(i for i in range(1, len(rows)) if intake[i] != intake[i - 1])
    still = pauses(points)
    for first, last in still:
        keep.update((first, last))
    ordered = sorted(keep)
    for a, b in zip(ordered, ordered[1:]):
        keep_by_distance(points, a, b, tolerance, keep)
    result, previous = [], None
    for i in sorted(keep):
        t, forward, left, heading = points[i]
        hold = next((points[b][0] - points[a][0] for a, b in still if a == i), 0.0)
        # Closer to the last waypoint than the path tolerance: the same spot, not a new leg.
        moved = previous is None or math.hypot(forward - previous['forward'], left - previous['left']) > max(0.5, tolerance)
        if not moved:
            # Same spot as the last waypoint: fold a pause or an intake change into it.
            previous['hold'] = max(previous['hold'], hold)
            previous['intake_after'] = intake[i]
            continue
        previous = dict(t=t, forward=forward, left=left, heading=heading, hold=hold,
                        intake_into=intake[i - 1] if i else False, intake_after=intake[i])
        result.append(previous)
    return result


def describe(recording, number_, rows, args):
    lines = [f'## Rehearsal {number_} in {recording.name}', '']
    t0 = number(rows[0]['t_s'])
    duration = number(rows[-1]['t_s']) - t0
    end_note = (rows[-1].get('note') or 'Square') if rows[-1]['kind'] == 'MARK_END' else 'the file stops mid-rehearsal'
    lines.append(f'Started {t0:.1f} s into the run, lasted **{duration:.1f} s** (ended by: {end_note}), {len(rows)} rows.')
    for label, test in (('intake in', lambda r: number(r['intake_cmd']) > INTAKE_ON),
                        ('intake out', lambda r: number(r['intake_cmd']) < -INTAKE_ON),
                        ('launcher on', lambda r: r['launching'] == 'true'),
                        ('feeding', lambda r: r['feeding'] == 'true'),
                        ('launcher reversed (L1)', lambda r: r['reversing'] == 'true')):
        spans = intervals(rows, t0, test)
        if spans:
            lines.append(f'- {label}: ' + ', '.join(f'{a:.1f}-{b:.1f} s' for a, b in spans))
    volts = [number(r['battery_v']) for r in rows if not math.isnan(number(r['battery_v']))]
    if volts:
        lines.append(f'- battery: {min(volts):.2f}-{max(volts):.2f} V')
    if math.isnan(number(rows[0]['x_in'])):
        lines += ['', f'**No position was recorded** (odometry: {rows[0]["odom_status"]}), so there are no '
                  'waypoints: only the timeline above. The Pinpoint must be READY in TeleOp INIT.', '']
        return lines, None
    bad = sorted({r['odom_status'] for r in rows if r['odom_status'] != 'READY'})
    if bad:
        lines.append(f'- **Odometry was not READY for the whole rehearsal** ({", ".join(bad)}): distrust the positions.')
    points = relative_path(rows, args.absolute)
    length = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(points, points[1:]))
    turned = max(abs(p[3]) for p in points)
    lines.append(f'- drove {length * CM:.0f} cm in all; ended {cm(points[-1][1]):+.1f} cm forward, '
                 f'{cm(points[-1][2]):+.1f} cm left, heading {points[-1][3] + 0.0:+.1f} deg from the mark')
    if turned > 5:
        lines.append(f'- **The robot turned up to {turned:.0f} deg.** The hive garden route holds one heading and '
                     'rejects turns (PollenHiveRoute), and the Odo legs do not turn: this needs turn steps, '
                     'or re-drive it without turning.')
    marks = waypoints(rows, points, args.tolerance_in)
    origin = 'the pose at INIT (--absolute)' if args.absolute else 'the spot and heading where Circle was pressed'
    lines += ['', f'Waypoints (cm from {origin}; forward = toward the hive, left = left looking at the hive;',
              f'path kept within {args.tolerance_in:g} in of what was driven):', '',
              '| # | at s | forward cm | left cm | heading deg | leg from previous | intake on this leg | then hold s |',
              '| --- | ---: | ---: | ---: | ---: | --- | --- | ---: |']
    previous = None
    for i, w in enumerate(marks):
        if previous is None:
            leg = '(start)'
        else:
            df, dl = w['forward'] - previous['forward'], w['left'] - previous['left']
            leg = f'{cm(math.hypot(df, dl)):.1f} cm @ {round(math.degrees(math.atan2(dl, df)), 1) + 0.0:+.1f} deg'
        lines.append(f"| {i} | {w['t']:.1f} | {cm(w['forward']):.1f} | {cm(w['left']):.1f} | {round(w['heading'], 1) + 0.0:+.1f} | {leg} | "
                     f"{'yes' if w['intake_into'] else 'no'} | {w['hold']:.1f} |")
        previous = w
    lines += ['', 'Draft for a hive-style route (PollenHiveRoute `move(name, forwardCm, leftCm, headingDeg, collect)`).',
              'It is a starting point: check clearances, then test in DRIVE with an empty robot.', '', '```java']
    for i, w in enumerate(marks[1:], 1):
        lines.append(f'move("R{number_} P{i}", {cm(w["forward"]):.1f}, {cm(w["left"]):.1f}, 0, {"true" if w["intake_into"] else "false"}),'
                     + (f'   // then hold {w["hold"]:.1f} s' if w['hold'] else ''))
    lines += ['```', '', f'The driver took {duration:.1f} s. Autonomous has its own speeds and ramps: read the planned time on INIT.', '']
    return lines, (points, marks)


def svg(points, marks, title):
    xs, ys = [-p[2] for p in points], [-p[1] for p in points]   # left is drawn to the left, forward up
    low_x, high_x, low_y, high_y = min(xs) - 6, max(xs) + 6, min(ys) - 6, max(ys) + 6
    scale = 600 / max(high_x - low_x, high_y - low_y)
    width, height = (high_x - low_x) * scale, (high_y - low_y) * scale + 30
    def at(left, forward):
        return (-left - low_x) * scale, (-forward - low_y) * scale + 30
    path = ' '.join(f'{x:.1f},{y:.1f}' for x, y in (at(p[2], p[1]) for p in points))
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" font-family="sans-serif" font-size="12">',
             '<rect width="100%" height="100%" fill="white"/>',
             f'<text x="6" y="16">{title}: up = toward the hive, left = left. Grey = driven, blue = waypoints.</text>',
             f'<polyline points="{path}" fill="none" stroke="#999" stroke-width="2"/>',
             '<polyline points="' + ' '.join('%.1f,%.1f' % at(w['left'], w['forward']) for w in marks)
             + '" fill="none" stroke="#1f6fb2" stroke-width="1.5" stroke-dasharray="6 4"/>']
    for i, w in enumerate(marks):
        x, y = at(w['left'], w['forward'])
        parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="{"#2a9d3a" if w["intake_into"] else "#1f6fb2"}"/>'
                     f'<text x="{x + 8:.1f}" y="{y + 4:.1f}">{i}</text>')
    return '\n'.join(parts + ['</svg>', ''])


def summarize(recording, args):
    with recording.open(newline='') as f:
        rows = list(csv.DictReader(f))
    lines = [f'# {recording.name}', '']
    if not rows:
        return lines + ['Empty file: the run never reached START.', ''], []
    start = next((r for r in rows if r['kind'] == 'START'), None)
    if start:
        lines += ['Settings at START: `' + start['note'] + '`', '']
    stopped = rows[-1]['kind'] == 'STOP'
    lines.append(f'{len(rows)} rows, {number(rows[-1]["t_s"]):.1f} s. '
                 + ('Ended with STOP.' if stopped else '**No STOP row: the file is incomplete** (per-run limit, crash, or power loss).'))
    slow = max((number(r['slowest_loop_ms']) for r in rows), default=math.nan)
    lines += [f'Slowest loop: {slow:.0f} ms.', '']
    numbers = sorted({int(r['rehearsal']) for r in rows if r['rehearsal'].isdigit() and r['rehearsal'] != '0'})
    if not numbers:
        lines += ['No rehearsals were marked in this run (Circle starts one, Square ends it).', '']
    outputs = []
    for n in numbers:
        part = [r for r in rows if r['rehearsal'] == str(n)]
        text, drawing = describe(recording, n, part, args)
        lines += text
        outputs.append((n, text, drawing))
    return lines, outputs


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('path', type=Path)
    parser.add_argument('--out', type=Path, help='also write one .md and one .svg per rehearsal here')
    parser.add_argument('--tolerance-in', type=float, default=1.0,
                        help='how far (inches) the simplified path may stray from the driven one')
    parser.add_argument('--absolute', action='store_true',
                        help='measure from the pose at TeleOp INIT instead of from the Circle press')
    args = parser.parse_args()
    recordings = find_recordings(args.path)
    if not recordings:
        print(f'No teleop-*.csv recordings under {args.path}. TeleOp recordings need build pollen-hive-v10 or later.')
        return
    for recording in recordings:
        lines, outputs = summarize(recording, args)
        print('\n'.join(lines))
        if args.out:
            args.out.mkdir(parents=True, exist_ok=True)
            for n, text, drawing in outputs:
                stem = f'{recording.stem}-rehearsal-{n}'
                (args.out / f'{stem}.md').write_text('\n'.join(text))
                if drawing:
                    (args.out / f'{stem}.svg').write_text(svg(*drawing, f'Rehearsal {n}'))


if __name__ == '__main__':
    main()
