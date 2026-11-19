#!/usr/bin/env python3
"""Build the run history in runs/ from downloaded sessions. Policy: docs/RUN_HISTORY.md.

    python3 tools/run-history.py SESSION [SESSION ...]   add downloads (folders or ZIPs)
    python3 tools/run-history.py ~/Desktop               add every session in a folder
    python3 tools/run-history.py                         rebuild index.csv and SUMMARY.md only

Reads the robot's own files under SESSION/pollen-hive and never changes them. Writes small
derived files only; raw recordings stay in the session ZIPs. Running it twice over the same
sessions changes nothing. notes.md files are created once and never overwritten.
"""
import argparse
import csv
import datetime
import hashlib
import io
import json
import re
import statistics
import sys
import zipfile
from collections import Counter
from pathlib import Path

TOOL_VERSION = 1  # Bump when derived numbers change; runs are re-derived where raw data is given.
ROOT = Path(__file__).resolve().parent.parent
try:
    from zoneinfo import ZoneInfo
    PACIFIC = ZoneInfo('America/Los_Angeles')
except Exception:  # No tz database: fall back to a fixed offset rather than fail.
    PACIFIC = datetime.timezone(datetime.timedelta(hours=-8))

HIVE_KINDS = ('match', 'shot_only', 'garden_return')
NOTES_KINDS = ('hive', 'collection_practice', 'odo_auto')
EXECUTION = {'0': 'DRY', '1': 'DRIVE', '2': 'LIVE'}
INDEX_COLUMNS = [
    'run_id', 'kind', 'hub_start', 'first_seen_session', 'build', 'installed', 'opmode', 'run',
    'secondary', 'make_room', 'execution', 'settings_id', 'cell', 'result', 'fault', 'warnings',
    'stopped_early', 'feed1_requested_s', 'feed1_start', 'feed1_end', 'feed2_requested_s',
    'feed2_start', 'feed2_end', 'both_volleys', 'feed2_cut_by_park', 'clear_out_s',
    'outbound_start', 'collect_start', 'collect_end', 'return_end', 'second_start',
    'park_wait_start', 'park_start', 'park_reason', 'done_at', 'spin_up_s', 'up_wait_first_s', 'up_wait_s',
    'settle_align_s', 'spare_before_park_s', 'return_error_in', 'return_heading_deg',
    'final_x_in', 'final_y_in', 'min_launcher_tps_feed1', 'min_launcher_tps_feed2',
    'dips_feed1', 'dips_feed2', 'dips_clear_out', 'last_dip_into_feed1_s',
    'last_dip_into_feed2_s', 'battery_start_v', 'battery_min_v', 'max_gap_s',
    'camera_status_last', 'trace_rows', 'trace_status', 'duration_s', 'r1_presses',
    'rehearsals', 'slowest_loop_ms', 'last_phase', 'notes_filled']
SOURCE_COLUMNS = ['session', 'session_time', 'path', 'bytes', 'md5', 'run_id']
SLIM_COLUMNS = ['t_s', 'kind', 'phase', 'execution', 'x_in', 'y_in', 'heading_deg',
                'requested_forward', 'requested_strafe', 'requested_turn',
                'requested_launcher_tps', 'measured_launcher_tps', 'applied_intake',
                'applied_agitator', 'feed_requested', 'hive_state', 'hive_reason', 'cell', 'battery_v',
                'launcher_current_a', 'clear_out']
NOTES_TEMPLATE = """# {run_id}

{context}

Filled in by people after the session. `tools/run-history.py` created this file once and never
changes it. Leave a line blank rather than guess. The logs cannot see balls; this file can.

- Event or practice (meet + match number, or practice + date):
- Field surface (tiles / vinyl):
- Partner and what they did (tipped our cell at about __ s?):
- Balls loaded at start:
- Balls scored, first volley / second volley:
- Balls left in the robot after the first volley (counted at the garden or flower):
- Balls collected:
- Jam or overload seen (where, when):
- Parked (yes / no / partly):
- Video (Drive link):
- Anything odd:
"""
REPORT_TRANSITION = re.compile(
    r'^(?P<t>[\d.]+)s (?P<phase>.*?) / (?P<hive>\S+) / feed=(?P<feed>true|false) '
    r'reason=(?P<reason>.*) poseIn=(?P<x>[-\d.NaInf]+),(?P<y>[-\d.NaInf]+) '
    r'headingDeg=(?P<h>[-\d.NaInf]+)$')


# ---------------------------------------------------------------- small helpers

def num(value):
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return None if f != f or f in (float('inf'), float('-inf')) else f


def fmt(value, places=2):
    """Blank for missing; never 0 for 'did not happen'."""
    if value is None or value == '':
        return ''
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, float):
        text = f'{value:.{places}f}'
        return '0.00' if text in ('-0.00', '-0.0') else text
    return str(value)


def base_phase(phase):
    return (phase or '').split(':')[0].strip()


def wrap_deg(deg):
    return None if deg is None else (deg + 180.0) % 360.0 - 180.0


def iso_from_ms(ms):
    ms = num(ms)
    if ms is None or ms < 1e11:
        return ''
    when = datetime.datetime.fromtimestamp(ms / 1000.0, PACIFIC)
    return when.replace(microsecond=0).isoformat()


def iso_from_stamp(stamp):
    """YYYYMMDD-HHMMSS in a file name: the Hub's local (Pacific) clock."""
    m = re.search(r'(20\d{6})-(\d{6})', stamp)
    if not m:
        return ''
    naive = datetime.datetime.strptime(m.group(1) + m.group(2), '%Y%m%d%H%M%S')
    return naive.replace(tzinfo=PACIFIC).isoformat()


def text_of(data):
    return data.decode('utf-8', errors='replace')


def csv_rows(data):
    return list(csv.reader(io.StringIO(text_of(data))))


def dict_rows(data):
    return list(csv.DictReader(io.StringIO(text_of(data))))


def write_if_changed(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.read_text() == content:
        return False
    path.write_text(content)
    return True


def write_csv(path, header, rows):
    out = io.StringIO()
    writer = csv.writer(out, lineterminator='\n')
    writer.writerow(header)
    writer.writerows(rows)
    return write_if_changed(path, out.getvalue())


def md_cell(value):
    return str(value).replace('|', '/').replace('\n', ' ')


# ---------------------------------------------------------------- reading sessions

class Session:
    """A downloaded session folder or its ZIP. Files are keyed by path under pollen-hive/."""

    def __init__(self, path):
        self.path = Path(path)
        self.name = self.path.stem if self.path.suffix == '.zip' else self.path.name
        self.time = iso_from_stamp(self.name)  # Laptop clock at download, from the folder name.
        self.files = {}
        if self.path.suffix == '.zip':
            with zipfile.ZipFile(self.path) as z:
                for info in z.infolist():
                    parts = info.filename.split('/')
                    if 'pollen-hive' in parts and not info.is_dir():
                        rel = '/'.join(parts[parts.index('pollen-hive') + 1:])
                        if rel:
                            self.files[rel] = z.read(info)
        else:
            hive = self.path / 'pollen-hive'
            if not hive.is_dir():
                raise SystemExit(f'{self.path}: no pollen-hive folder. A missing hive directory '
                                 'is a download failure, not an empty session.')
            for f in sorted(hive.rglob('*')):
                if f.is_file():
                    self.files[f.relative_to(hive).as_posix()] = f.read_bytes()


def classify(rel):
    """(kind, run_id, role) for a recording path, or None for settings and other files."""
    name = rel.split('/')[-1]
    folder = rel.split('/')[0] if '/' in rel else ''
    if folder == 'reports' and name.endswith('.txt'):
        prefix = name.split('-')[0]
        if prefix in HIVE_KINDS:
            return 'hive', name[:-4], 'report'
        if prefix == 'camera':
            return 'camera_check', name[:-4], 'report'
    if folder == 'traces' and name.endswith('.txt.csv'):
        return 'hive', name[:-8], 'trace'
    if folder == 'auto' and name.endswith('.csv'):
        return 'auto', name[:-4], 'main'
    if folder == 'teleop' and name.endswith('.csv'):
        return 'teleop', name[:-4], 'main'
    if folder == 'benchmark' and name.endswith('.jsonl'):
        return 'benchmark', name[:-6], 'main'
    return None


# ---------------------------------------------------------------- hive autos

def parse_report(text):
    info = {'transitions': [], 'faults': [], 'stop': '', 'lines': text.splitlines()}
    for line in info['lines']:
        body = line.split(' ', 1)[1] if ' ' in line else ''
        if line.startswith('0.000s build='):
            m = re.search(r'build=(\S+) run=(\S+)', line)
            if m:
                info['build'], info['run_kind'] = m.group(1), m.group(2)
        elif body.startswith('START'):
            for key in ('build', 'opmode', 'wall_ms', 'run', 'secondary', 'makeRoom'):
                m = re.search(r'\b' + key + r'=(\S+)', body)
                if m:
                    info['start_' + key] = m.group(1)
            m = re.search(r'settings=(.*?)(?: camera=| unsaved_route=|$)', body)
            info['settings'] = m.group(1).strip() if m else ''
        elif body.startswith('HUB '):
            for key in ('installed', 'trace'):
                m = re.search(key + r'=(NONE \([^)]*\)|\S+)', body)
                if m:
                    info[key] = m.group(1)
        elif body.startswith('FAULT'):
            info['faults'].append(body[len('FAULT'):].strip())
        elif body.startswith('STOP'):
            info['stop'] = body
        else:
            m = REPORT_TRANSITION.match(line)
            if m:
                info['transitions'].append(m.groupdict())
    return info


def settings_value(settings, key):
    m = re.search(r'(?:^|, )' + re.escape(key) + r'=([^,]+)', settings or '')
    return m.group(1).strip() if m else None


def dips(window, rows):
    """Drops below 90 % of the requested speed that recover above 95 %. A lower bound."""
    count, last, dipping, dip_t = 0, None, False, None
    for r in rows:
        req, meas, t = abs(num(r.get('requested_launcher_tps')) or 0), num(r.get('measured_launcher_tps')), num(r['t_s'])
        if req <= 0 or meas is None:
            continue
        meas = abs(meas)
        if not dipping and meas < 0.9 * req:
            dipping, dip_t = True, t
        elif dipping and meas > 0.95 * req:
            dipping, count, last = False, count + 1, dip_t
    if last is not None and window is not None:
        last -= window
    return count, last


def hive_metrics(run_id, report, trace):
    info = parse_report(text_of(report)) if report else {'transitions': [], 'faults': [], 'stop': '', 'lines': []}
    rows = dict_rows(trace) if trace else []
    for r in rows:
        r['_t'] = num(r.get('t_s'))
        r['_base'] = base_phase(r.get('phase'))
        r['_feed'] = r.get('feed_requested') == 'true'
        r['_clear'] = 'clear_out_s_left=' in (r.get('decision_inputs') or '')
    rows = [r for r in rows if r['_t'] is not None]
    if not rows and info['transitions']:
        # No trace (builds before v7, or the trace budget was full): use the report's transitions.
        # Phase times and feed windows still work; launcher speeds and battery stay blank.
        for t in info['transitions']:
            rows.append({'t_s': t['t'], 'kind': 'TRANSITION', 'phase': t['phase'].strip(), 'reason': t['reason'],
                         'feed_requested': t['feed'], 'hive_state': t['hive'], 'x_in': t['x'], 'y_in': t['y'],
                         'heading_deg': t['h'], '_t': num(t['t']), '_base': base_phase(t['phase']),
                         '_feed': t['feed'] == 'true', '_clear': False})
    settings = info.get('settings', '')
    m = {'run_id': run_id, 'kind': 'hive'}
    millis = re.search(r'-(\d{12,})-', run_id)
    m['hub_start'] = iso_from_ms(info.get('start_wall_ms') or (millis.group(1) if millis else None))
    m['build'] = info.get('start_build') or info.get('build', '')
    m['installed'] = info.get('installed', '')
    m['opmode'] = info.get('start_opmode', '')
    m['run'] = info.get('start_run') or (info.get('run_kind') or '').upper()
    m['secondary'] = info.get('start_secondary', '')
    m['make_room'] = info.get('start_makeRoom', '')
    m['settings'] = settings
    m['settings_id'] = hashlib.md5(settings.encode()).hexdigest()[:8] if settings else ''
    executions = Counter(str(int(num(r.get('execution')))) for r in rows if num(r.get('execution')) is not None)
    exec_code = executions.most_common(1)[0][0] if executions else None
    if exec_code is None and num(settings_value(settings, 'execution')) is not None:
        exec_code = str(int(num(settings_value(settings, 'execution'))))
    m['execution'] = EXECUTION.get(exec_code, '')
    cells = Counter(r.get('cell') for r in rows if (num(r.get('cell')) or -1) >= 0)
    m['cell'] = cells.most_common(1)[0][0] if cells else ''

    def first(*bases):
        return next((r for r in rows if r['_base'] in bases), None)

    def first_t(*bases):
        r = first(*bases)
        return r['_t'] if r else None

    done_row = first('DONE')
    transitions = info['transitions']
    done_tr = next((t for t in transitions if base_phase(t['phase']) == 'DONE'), None)
    if done_row is not None:
        m['result'] = done_row.get('reason', '')
    elif done_tr is not None:
        m['result'] = done_tr['reason']
    elif rows:
        m['result'] = rows[-1].get('reason', '')
    else:
        m['result'] = transitions[-1]['reason'] if transitions else ''
    faults = info['faults'] + [r.get('reason', '') for r in rows if r.get('kind') == 'FAULT']
    m['fault'] = ' ; '.join(dict.fromkeys(f for f in faults if f))
    m['stopped_early'] = bool(info['stop']) and done_row is None and done_tr is None and not faults

    for n, phase in ((1, 'FIRST'), (2, 'SECOND')):
        feed = [r for r in rows if r['_feed'] and r['_base'] == phase]
        m[f'feed{n}_start'] = feed[0]['_t'] if feed else None
        m[f'feed{n}_end'] = feed[-1]['_t'] if feed else None
        req = re.search(r'requested_s=([\d.]+)', feed[0].get('decision_inputs', '')) if feed else None
        m[f'feed{n}_requested_s'] = num(req.group(1)) if req else None
        window = [r for r in rows if feed and feed[0]['_t'] <= r['_t'] <= feed[-1]['_t']]
        speeds = [abs(num(r.get('measured_launcher_tps'))) for r in window if num(r.get('measured_launcher_tps')) is not None]
        m[f'min_launcher_tps_feed{n}'] = min(speeds) if speeds else None
        count, last = dips(m[f'feed{n}_start'], window)
        m[f'dips_feed{n}'] = count if feed else None
        m[f'last_dip_into_feed{n}_s'] = last
    m['both_volleys'] = m['feed1_start'] is not None and m['feed2_start'] is not None
    cut = None
    if m['feed2_start'] is not None:
        cut = any(r['_base'] == 'PARK' and r.get('reason', '').startswith('Park deadline during SECOND') for r in rows)
        cut = cut or any(a['_base'] == 'SECOND' and a['_feed'] and b['_base'] == 'PARK' for a, b in zip(rows, rows[1:]))
    m['feed2_cut_by_park'] = cut
    clear = [r for r in rows if r['_clear']]
    m['clear_out_s'] = (clear[-1]['_t'] - clear[0]['_t']) if clear else None
    count, _ = dips(None, clear)
    m['dips_clear_out'] = count if clear else None

    m['outbound_start'] = first_t('OUTBOUND')
    m['collect_start'] = first_t('GARDEN', 'FLOWER')
    m['collect_end'] = first_t('RETURN')
    m['return_end'] = first_t('SETTLE', 'ALIGN_SHOT', 'SECOND')
    m['second_start'] = first_t('SECOND')
    m['park_wait_start'] = first_t('PARK_WAIT', 'WAIT')
    m['park_start'] = first_t('PARK')
    m['done_at'] = done_row['_t'] if done_row else None
    park_row = first('PARK')
    m['park_reason'] = park_row.get('reason', '') if park_row else ''
    spin = next((r['_t'] for r in rows if abs(num(r.get('requested_launcher_tps')) or 0) > 0), None)
    m['spin_up_s'] = m['feed1_start'] - spin if m['feed1_start'] is not None and spin is not None else None
    first_start = first_t('FIRST')
    m['up_wait_first_s'] = m['feed1_start'] - first_start if m['feed1_start'] is not None and first_start is not None else None
    m['up_wait_s'] = m['feed2_start'] - m['second_start'] if m['feed2_start'] is not None and m['second_start'] is not None else None
    settle = first_t('SETTLE', 'ALIGN_SHOT')
    after = next((r['_t'] for r in rows if settle is not None and r['_t'] > settle
                  and r['_base'] not in ('SETTLE', 'ALIGN_SHOT')), None)
    m['settle_align_s'] = after - settle if settle is not None and after is not None else None
    m['spare_before_park_s'] = m['park_start'] - m['feed2_end'] if m['park_start'] is not None and m['feed2_end'] is not None else None
    second = first('SECOND')
    if second is not None and num(second.get('x_in')) is not None:
        m['return_error_in'] = (num(second['x_in']) ** 2 + num(second['y_in']) ** 2) ** 0.5
        m['return_heading_deg'] = wrap_deg(num(second.get('heading_deg')))
    else:
        m['return_error_in'] = m['return_heading_deg'] = None
    m['final_x_in'] = num(done_row.get('x_in')) if done_row else None
    m['final_y_in'] = num(done_row.get('y_in')) if done_row else None
    volts = [num(r.get('battery_v')) for r in rows if (num(r.get('battery_v')) or 0) > 1]
    m['battery_start_v'] = volts[0] if volts else None
    m['battery_min_v'] = min(volts) if volts else None
    times = [r['_t'] for r in rows]
    m['max_gap_s'] = max((b - a for a, b in zip(times, times[1:])), default=None)
    statuses = [r.get('camera_status') for r in rows if r.get('camera_status')]
    m['camera_status_last'] = statuses[-1] if statuses else ''
    m['trace_rows'] = len(rows) if trace else None
    m['trace_status'] = info.get('trace', '' if report else 'report missing')
    m['last_phase'] = rows[-1]['_base'] if rows else (base_phase(transitions[-1]['phase']) if transitions else '')

    timeline = [[t['t'], t['phase'].strip(), t['hive'], t['feed'], t['reason'], t['x'], t['y'], t['h']]
                for t in transitions]
    if not timeline:  # No report: fall back to trace transitions.
        timeline = [[r['t_s'], r.get('phase', ''), r.get('hive_state', ''), r.get('feed_requested', ''),
                     r.get('reason', ''), r.get('x_in', ''), r.get('y_in', ''), r.get('heading_deg', '')]
                    for r in rows if r.get('kind') in ('TRANSITION', 'FAULT', 'STOP')]
    slim, stop_at = [], (m['done_at'] + 1.0) if m['done_at'] is not None else None
    for r in rows if trace else []:
        if stop_at is not None and r['_t'] > stop_at:
            break
        out = []
        for col in SLIM_COLUMNS:
            if col == 'clear_out':
                out.append(fmt(r['_clear']))
            elif col == 'execution':
                v = num(r.get('execution'))
                out.append(EXECUTION.get(str(int(v)), '') if v is not None else '')
            else:
                v = r.get(col, '')
                f = num(v)
                out.append(fmt(f) if f is not None and col not in ('cell',) else ('' if v in ('NaN',) else v))
        slim.append(out)
    extra = {'faults': faults, 'stop': info['stop'], 'report_lines': len(info['lines'])}
    return m, timeline, slim, extra


# ---------------------------------------------------------------- other kinds

def camera_metrics(run_id, data):
    info = parse_report(text_of(data))
    millis = re.search(r'-(\d{12,})-', run_id)
    m = {'run_id': run_id, 'kind': 'camera_check', 'build': info.get('build', ''),
         'hub_start': iso_from_ms(millis.group(1) if millis else None),
         'opmode': 'Calibration Hive Camera', 'result': info['stop'],
         'fault': ' ; '.join(info['faults'])}
    return m, [], [], {'lines': info['lines']}


def auto_metrics(run_id, data):
    rows = csv_rows(data)
    if rows and rows[0][:1] == ['START'] and len(rows[0]) >= 3:
        return collection_metrics(run_id, rows)
    return odo_metrics(run_id, rows)


def collection_metrics(run_id, rows):
    start = rows[0]
    m = {'run_id': run_id, 'kind': 'collection_practice', 'build': start[1], 'opmode': start[2],
         'settings': start[3] if len(start) > 3 else '', 'hub_start': iso_from_stamp(run_id)}
    m['settings_id'] = hashlib.md5(m['settings'].encode()).hexdigest()[:8] if m['settings'] not in ('', '{}') else ''
    timeline, faults, previous, last_t, done_t, stop = [], [], None, None, None, None
    for r in rows[2:]:
        if len(r) >= 2 and r[1] == 'FAULT':
            faults.append(r[3] if len(r) > 3 else r[-1])
            timeline.append([r[0], 'FAULT', r[2] if len(r) > 2 else '', '', '', ''])
        elif len(r) >= 2 and r[1] == 'STOP':
            stop = r
        elif len(r) >= 6:
            t = num(r[0])
            if r[1] != previous:
                timeline.append([r[0], r[1], r[2], r[3], r[4], r[7] if len(r) > 7 else ''])
                previous = r[1]
            if r[1] == 'DONE' and done_t is None:
                done_t = t
            if r[1] != 'DONE':
                last_t = t
    m['collect_s'] = done_t if done_t is not None else last_t
    m['done_at'] = done_t
    m['last_phase'] = previous or ''
    m['fault'] = ' ; '.join(dict.fromkeys(f for f in faults if f))
    m['result'] = 'DONE' if done_t is not None else ('FAULT' if faults else (stop[2] if stop and len(stop) > 2 else ''))
    m['stopped_early'] = done_t is None and not faults and stop is not None
    m['duration_s'] = num(stop[0]) if stop else None
    return m, timeline, [], {'faults': faults, 'header': ['t_s', 'phase', 'route_x_in', 'route_y_in', 'heading_deg', 'event']}


def odo_metrics(run_id, rows):
    m = {'run_id': run_id, 'kind': 'odo_auto', 'hub_start': iso_from_stamp(run_id)}
    timeline, warnings, volts, stop_t = [], [], [], None
    for r in rows[1:]:
        if len(r) < 3:
            continue
        t_s, wall, text = r[0], r[1], r[2]
        volts += [num(v) for v in re.findall(r'battery=([\d.]+)V', text)]
        if text.startswith('START '):
            m['hub_start'] = iso_from_ms(wall) or m['hub_start']
            m['opmode'] = text[len('START '):].split(' build=')[0]
            for key, col in (('build', 'build'), ('installed', 'installed'), ('route', 'run')):
                found = re.search(r'\b' + key + r'=(\S+)', text)
                if found:
                    m[col] = found.group(1)
            settings = re.findall(r'\b([A-Z][A-Z0-9_]+=[^ ]+)', text)
            m['settings'] = ' '.join(settings)
            m['settings_id'] = hashlib.md5(m['settings'].encode()).hexdigest()[:8] if settings else ''
            timeline.append([t_s, 'START', text[:200]])
        body = re.sub(r'^t=[\d.]+ ', '', text)
        if body.startswith('PHASE '):
            timeline.append([t_s, body[len('PHASE '):].split(' | ')[0], ''])
        elif body.startswith('RESULT '):
            m['result'] = body[len('RESULT '):].split(' | ')[0]
            m['done_at'] = num(t_s)
            timeline.append([t_s, 'RESULT', m['result']])
        elif body.startswith('WARNING'):
            warnings.append(body)
            timeline.append([t_s, 'WARNING', body[:200]])
        elif body.startswith('STOP'):
            stop_t = num(t_s)
            timeline.append([t_s, 'STOP', body[:200]])
    m['warnings'] = ' ; '.join(dict.fromkeys(warnings))
    m['stopped_early'] = stop_t is not None and 'result' not in m
    m['duration_s'] = stop_t
    m['battery_start_v'] = volts[0] if volts else None
    m['battery_min_v'] = min((v for v in volts if v), default=None)
    m['last_phase'] = next((row[1] for row in reversed(timeline) if row[1] not in ('STOP', 'WARNING')), '')
    return m, timeline, [], {'header': ['t_s', 'event', 'text']}


def teleop_metrics(run_id, data):
    rows = dict_rows(data)
    m = {'run_id': run_id, 'kind': 'teleop', 'hub_start': iso_from_stamp(run_id), 'opmode': 'PollenBot'}
    start = next((r for r in rows if r.get('kind') == 'START'), None)
    if start:
        note = start.get('note', '')
        for key, col in (('build', 'build'), ('installed', 'installed'), ('opmode', 'opmode')):
            found = re.search(r'\b' + key + r'=(\S+)', note)
            if found:
                m[col] = found.group(1)
        found = re.search(r'settings=(\S+)', note)
        m['settings'] = found.group(1) if found else ''
        m['hub_start'] = iso_from_ms(start.get('wall_ms')) or m['hub_start']
    times = [num(r.get('t_s')) for r in rows if num(r.get('t_s')) is not None]
    m['duration_s'] = max(times) if times else None
    presses, previous = 0, False
    for r in rows:
        if r.get('kind') != 'SAMPLE':
            continue
        now = r.get('r1') == 'true'
        presses += now and not previous
        previous = now
    m['r1_presses'] = presses
    m['rehearsals'] = sum(1 for r in rows if r.get('kind') == 'MARK_START')
    loops = [num(r.get('slowest_loop_ms')) for r in rows if num(r.get('slowest_loop_ms')) is not None]
    m['slowest_loop_ms'] = max(loops) if loops else None
    volts = [num(r.get('battery_v')) for r in rows if (num(r.get('battery_v')) or 0) > 1]
    m['battery_start_v'] = volts[0] if volts else None
    m['battery_min_v'] = min(volts) if volts else None
    m['stopped_early'] = None
    timeline = [[r.get('t_s'), r.get('kind'), r.get('rehearsal'), r.get('note', '')[:200]]
                for r in rows if r.get('kind') not in ('SAMPLE',)]
    return m, timeline, [], {'header': ['t_s', 'kind', 'rehearsal', 'note']}


def benchmark_metrics(run_id, data):
    m = {'run_id': run_id, 'kind': 'benchmark', 'opmode': 'Calibration Hive Pipeline Benchmark'}
    first_line = text_of(data).splitlines()[:1]
    try:
        head = json.loads(first_line[0]) if first_line else {}
    except ValueError:
        head = {}
    m['build'] = head.get('build', '')
    millis = re.search(r'-(\d{12,})-', run_id)
    m['hub_start'] = iso_from_ms(head.get('wall_ms') or (millis.group(1) if millis else None))
    events = [json.loads(line) for line in text_of(data).splitlines() if line.startswith('{')]
    stop = next((e for e in events if e.get('event') == 'stop'), None)
    m['result'] = ('complete' if stop.get('complete') else 'incomplete: ' + stop.get('message', '')) if stop else ''
    return m, [], [], {}


# ---------------------------------------------------------------- writing one run

def summary_md(m, timeline, extra, source):
    lines = [f"# {m['run_id']}", '',
             'Generated by `tools/run-history.py` from the robot\'s own files; do not edit. '
             'People write in `notes.md`. Policy: [docs/RUN_HISTORY.md](../../../docs/RUN_HISTORY.md).', '',
             '| Field | Value |', '| --- | --- |']
    for key in ('kind', 'opmode', 'build', 'installed', 'hub_start', 'run', 'execution', 'secondary',
                'make_room', 'cell', 'result', 'fault', 'warnings', 'stopped_early'):
        if m.get(key) not in (None, ''):
            lines.append(f'| {key} | {md_cell(fmt(m[key]))} |')
    lines.append(f"| source | `{source['path']}` {source['bytes']} bytes, md5 `{source['md5']}`, "
                 f"first seen in `{m.get('first_seen_session', '')}` |")
    if m.get('settings'):
        lines += ['', '## Settings at START', '', '```text', m['settings'], '```']
    metrics = [(k, m.get(k)) for k in INDEX_COLUMNS[INDEX_COLUMNS.index('feed1_requested_s'):]
               if k not in ('notes_filled', 'last_phase', 'warnings') and m.get(k) not in (None, '')]
    if metrics:
        lines += ['', '## Metrics', '', 'Seconds from START (trace `t_s`). Blank means it did not happen.',
                  '', '| Metric | Value |', '| --- | --- |']
        lines += [f'| {k} | {md_cell(fmt(v))} |' for k, v in metrics]
    if timeline:
        lines += ['', '## Timeline', '']
        if m['kind'] == 'hive':
            lines += ['| t (s) | Phase | Hive | Feed | Reason | x, y (in) | Heading |',
                      '| ---: | --- | --- | --- | --- | --- | ---: |']
            lines += [f'| {r[0]} | {md_cell(r[1])} | {r[2]} | {r[3]} | {md_cell(r[4])} | {r[5]}, {r[6]} | {r[7]} |'
                      for r in timeline]
        else:
            header = extra.get('header', ['t_s', 'event'])
            lines += ['| ' + ' | '.join(header) + ' |', '|' + ' --- |' * len(header)]
            lines += ['| ' + ' | '.join(md_cell(c) for c in r) + ' |' for r in timeline]
    if m['kind'] == 'benchmark':
        lines += ['', f"The camera analysis is `analysis/summary.md` in session `{m.get('first_seen_session', '')}` "
                  '(not copied here).']
    return '\n'.join(lines) + '\n'


def notes_filled(path):
    if not path.exists():
        return ''
    filled = [line for line in path.read_text().splitlines()
              if line.startswith('- ') and ':' in line and line.split(':', 1)[1].strip()]
    return 'yes' if filled else 'no'


def notes_field(path, label):
    if not path.exists():
        return ''
    for line in path.read_text().splitlines():
        if line.startswith('- ' + label):
            return line.split(':', 1)[1].strip() if ':' in line else ''
    return ''


def run_dir(runs, m):
    day = (m.get('hub_start') or '')[:10] or 'unknown-date'
    return runs / day / m['run_id']


def derive(kind, run_id, files):
    if kind == 'hive':
        return hive_metrics(run_id, files.get('report'), files.get('trace'))
    if kind == 'camera_check':
        return camera_metrics(run_id, files['report'])
    if kind == 'auto':
        return auto_metrics(run_id, files['main'])
    if kind == 'teleop':
        return teleop_metrics(run_id, files['main'])
    return benchmark_metrics(run_id, files['main'])


# ---------------------------------------------------------------- index and summary

def load_runs(runs):
    out = []
    for path in sorted(runs.glob('*/*/metrics.json')):
        m = json.loads(path.read_text())
        m['notes_filled'] = notes_filled(path.parent / 'notes.md')
        m['_dir'] = path.parent
        out.append(m)
    out.sort(key=lambda m: (m.get('hub_start') or '', m['run_id']))
    return out


def median_worst(values, worst=max):
    values = [v for v in values if v is not None]
    if not values:
        return ''
    return f'{statistics.median(values):.1f} / {worst(values):.1f} (n={len(values)})'


def summary_report(all_runs):
    hive = [m for m in all_runs if m['kind'] == 'hive' and m.get('execution') == 'LIVE']
    lines = ['# Run history summary', '',
             'Generated by `tools/run-history.py` from `runs/*/*/metrics.json` and `notes.md`; '
             'overwritten each time. Policy: [docs/RUN_HISTORY.md](../docs/RUN_HISTORY.md).', '',
             'Numbers come from the robot\'s own recordings. Ball counts come only from `notes.md`.', '',
             '## Runs by kind and build', '', '| Kind | Build | Runs |', '| --- | --- | ---: |']
    for (kind, build), n in sorted(Counter((m['kind'], m.get('build') or '?') for m in all_runs).items()):
        lines.append(f'| {kind} | {build} | {n} |')
    lines += ['', '## Hive autos, LIVE only', '',
              'Times are seconds: median / worst (n). Worst is the longest, except spare before park, '
              'where it is the smallest.', '']
    groups = {}
    for m in hive:
        groups.setdefault((m.get('opmode') or m.get('run') or '?', m.get('build') or '?'), []).append(m)
    if not groups:
        lines.append('No LIVE hive runs yet.')
    for (opmode, build), ms in sorted(groups.items()):
        def d(a, b, rows=ms):
            return [r[b] - r[a] if r.get(a) is not None and r.get(b) is not None else None for r in rows]
        lines += [f'### {opmode}, {build}', '',
                  f"Runs {len(ms)}; both volleys {sum(1 for r in ms if r.get('both_volleys'))}; "
                  f"second volley cut by park {sum(1 for r in ms if r.get('feed2_cut_by_park'))}; "
                  f"faults {sum(1 for r in ms if r.get('fault'))}; "
                  f"stopped early by the driver {sum(1 for r in ms if r.get('stopped_early'))}.", '',
                  '| Step | Seconds |', '| --- | --- |',
                  f"| Spin-up to first feed | {median_worst([r.get('spin_up_s') for r in ms])} |",
                  f"| Wait for UP before first feed | {median_worst([r.get('up_wait_first_s') for r in ms])} |",
                  f"| Volley 1 | {median_worst(d('feed1_start', 'feed1_end'))} |",
                  f"| Outbound | {median_worst(d('outbound_start', 'collect_start'))} |",
                  f"| Collection | {median_worst(d('collect_start', 'collect_end'))} |",
                  f"| Return | {median_worst(d('collect_end', 'return_end'))} |",
                  f"| Settle / align | {median_worst([r.get('settle_align_s') for r in ms])} |",
                  f"| Wait for UP before second feed | {median_worst([r.get('up_wait_s') for r in ms])} |",
                  f"| Volley 2 | {median_worst(d('feed2_start', 'feed2_end'))} |",
                  f"| Spare before park | {median_worst([r.get('spare_before_park_s') for r in ms], min)} |",
                  f"| Return error (in) | {median_worst([r.get('return_error_in') for r in ms])} |", '']
        results = Counter((r.get('result') or '?')[:70] for r in ms)
        lines += ['Results: ' + '; '.join(f'{md_cell(k)} ({v})' for k, v in results.most_common()), '']
        parks = Counter((r.get('park_reason') or '')[:70] for r in ms if r.get('park_reason'))
        if parks:
            lines += ['Why it left for park: ' + '; '.join(f'{md_cell(k)} ({v})' for k, v in parks.most_common()), '']
    lines += ['## Launcher dips against the feed time', '',
              'Last dip is seconds after the volley started. If it is well before the requested feed time, '
              'the feed may be longer than needed. 10 rows/s can miss short dips.', '',
              '| Run | Build | Feed 1 req. | Dips 1 | Last dip 1 | Feed 2 req. | Dips 2 | Last dip 2 | Clear-out dips |',
              '| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
    for m in hive:
        if m.get('feed1_start') is None:
            continue
        lines.append(f"| {m['run_id']} | {m.get('build')} | {fmt(m.get('feed1_requested_s'), 1)} | "
                     f"{fmt(m.get('dips_feed1'))} | {fmt(m.get('last_dip_into_feed1_s'), 1)} | "
                     f"{fmt(m.get('feed2_requested_s'), 1)} | {fmt(m.get('dips_feed2'))} | "
                     f"{fmt(m.get('last_dip_into_feed2_s'), 1)} | {fmt(m.get('dips_clear_out'))} |")
    slow = [m for m in hive if (m.get('settle_align_s') or 0) > 1.0]
    lines += ['', '## Return and settle', '',
              'Runs where SETTLE or ALIGN_SHOT took over 1 s: ' +
              (', '.join(f"{m['run_id']} ({m['settle_align_s']:.1f} s)" for m in slow) or 'none') + '.', '']
    secondary = [m for m in hive if m.get('secondary') == 'true' and m.get('up_wait_first_s') is not None]
    if secondary:
        lines += ['## Secondary: first-volley wait against a second volley', '',
                  '| Run | Build | Wait for UP before first feed (s) | Second volley |', '| --- | --- | ---: | --- |']
        lines += [f"| {m['run_id']} | {m.get('build')} | {m['up_wait_first_s']:.1f} | {fmt(m.get('both_volleys'))} |"
                  for m in secondary]
        lines.append('')
    noted = [m for m in all_runs if m['kind'] in NOTES_KINDS and m.get('notes_filled') == 'yes']
    lines += ['## From notes.md', '']
    if noted:
        lines += ['| Run | Build | Balls left after volley 1 | Jam or overload |', '| --- | --- | --- | --- |']
        for m in noted:
            left = notes_field(m['_dir'] / 'notes.md', 'Balls left in the robot')
            jam = notes_field(m['_dir'] / 'notes.md', 'Jam or overload')
            lines.append(f"| {m['run_id']} | {m.get('build')} | {md_cell(left)} | {md_cell(jam)} |")
    else:
        lines.append('No notes filled in yet.')
    missing = [m for m in hive if m.get('notes_filled') != 'yes']
    lines += ['', f'LIVE hive runs without notes: {len(missing)} of {len(hive)}.', '']
    return '\n'.join(lines)


def index_rows(all_runs):
    return [[fmt(m.get(c)) for c in INDEX_COLUMNS] for m in all_runs]


# ---------------------------------------------------------------- main

def add_sessions(runs, sessions):
    sources_path = runs / 'sources.csv'
    sources = {}
    if sources_path.exists():
        with sources_path.open(newline='') as f:
            for row in csv.DictReader(f):
                sources[(row['session'], row['path'])] = row
    candidates = {}  # (kind, run_id) -> role -> list of (bytes, md5, data, session)
    for session in sessions:
        for rel, data in session.files.items():
            found = classify(rel)
            if not found:
                continue
            kind, run_id, role = found
            md5 = hashlib.md5(data).hexdigest()
            sources[(session.name, rel)] = {'session': session.name, 'session_time': session.time,
                                            'path': rel, 'bytes': str(len(data)), 'md5': md5, 'run_id': run_id}
            candidates.setdefault((kind, run_id), {}).setdefault(role, []).append((len(data), md5, data, session, rel))
    write_csv(sources_path, SOURCE_COLUMNS,
              [[r[c] for c in SOURCE_COLUMNS] for _, r in sorted(sources.items())])
    first_seen = {}
    for row in sources.values():
        key = row['run_id']
        stamp = (row['session_time'] or '9999', row['session'])
        first_seen[key] = min(first_seen.get(key, stamp), stamp)
    existing = {m['run_id']: m for m in load_runs(runs)}
    changed = 0
    for (kind, run_id), roles in sorted(candidates.items()):
        # The same run downloaded twice can differ if it was still being written: keep the largest.
        best = {role: max(items, key=lambda i: (i[0], i[1])) for role, items in roles.items()}
        primary = best.get('report') or best.get('main') or best.get('trace')
        total = sum(b[0] for b in best.values())
        old = existing.get(run_id)
        if old and old.get('tool_version') == TOOL_VERSION and old.get('source_bytes', 0) >= total:
            if old.get('first_seen_session') != first_seen[run_id][1]:
                old_dir = old.pop('_dir')
                old.pop('notes_filled', None)
                old['first_seen_session'] = first_seen[run_id][1]
                write_if_changed(old_dir / 'metrics.json', json.dumps(old, indent=1, sort_keys=True) + '\n')
            continue
        m, timeline, slim, extra = derive(kind, run_id, {role: b[2] for role, b in best.items()})
        m['first_seen_session'] = first_seen[run_id][1]
        m['tool_version'] = TOOL_VERSION
        m['source_bytes'] = total
        m['sources'] = sorted(f'{b[4]} {b[1]}' for b in best.values())
        folder = run_dir(runs, m)
        if old and old['_dir'] != folder:  # Better data moved the run to another date folder.
            folder.mkdir(parents=True, exist_ok=True)
            if (old['_dir'] / 'notes.md').exists() and not (folder / 'notes.md').exists():
                (old['_dir'] / 'notes.md').rename(folder / 'notes.md')
            for stale in old['_dir'].iterdir():
                if stale.name != 'notes.md':
                    stale.unlink()
            if not any(old['_dir'].iterdir()):
                old['_dir'].rmdir()
        source = {'path': primary[4], 'bytes': primary[0], 'md5': primary[1]}
        record = {k: v for k, v in m.items() if not k.startswith('_')}
        changed += write_if_changed(folder / 'metrics.json', json.dumps(record, indent=1, sort_keys=True) + '\n')
        changed += write_if_changed(folder / 'summary.md', summary_md(m, timeline, extra, source))
        if kind == 'hive':
            write_csv(folder / 'timeline.csv', ['t_s', 'phase', 'hive_state', 'feed', 'reason', 'x_in', 'y_in', 'heading_deg'], timeline)
            if slim:
                write_csv(folder / 'slim-trace.csv', SLIM_COLUMNS, slim)
        elif timeline:
            write_csv(folder / 'timeline.csv', extra.get('header', ['t_s', 'event']), timeline)
        notes = folder / 'notes.md'
        if m['kind'] in NOTES_KINDS and not notes.exists():
            context = (f"{m.get('opmode') or m['kind']}, build {m.get('build') or '?'}, "
                       f"Hub start {m.get('hub_start') or '?'}, execution {m.get('execution') or '-'}, "
                       f"result: {m.get('result') or '?'}")
            notes.write_text(NOTES_TEMPLATE.format(run_id=run_id, context=context))
    return changed


def is_session_zip(path):
    try:
        with zipfile.ZipFile(path) as z:
            return any('/pollen-hive/' in '/' + n for n in z.namelist())
    except (zipfile.BadZipFile, OSError):
        return False


def expand(paths):
    """A folder that is not itself a session (for example ~/Desktop) means every session in it.
    Folders come before ZIPs, so a session that was unzipped is read once, from the folder."""
    out = []
    for path in paths:
        if path.is_dir() and not (path / 'pollen-hive').is_dir():
            inner = sorted(p for p in path.iterdir() if p.is_dir() and (p / 'pollen-hive').is_dir())
            inner += sorted(p for p in path.glob('*.zip') if is_session_zip(p))
            if not inner:
                raise SystemExit(f'{path}: no downloaded sessions (folders with pollen-hive/, or their ZIPs).')
            out += inner
        else:
            out.append(path)
    return sorted(out, key=lambda p: p.suffix == '.zip')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('sessions', nargs='*', type=Path, help='downloaded session folders or their ZIPs')
    parser.add_argument('--runs', type=Path, default=ROOT / 'runs', help='history folder (default: runs/)')
    args = parser.parse_args()
    runs = args.runs
    runs.mkdir(parents=True, exist_ok=True)
    sessions = []
    names = set()
    for path in expand(args.sessions):
        session = Session(path)
        if session.name in names:  # A folder and its ZIP: read once.
            continue
        names.add(session.name)
        sessions.append(session)
    changed = add_sessions(runs, sessions) if sessions else 0
    write_if_changed(runs / 'README.md',
                     '# Run history\n\nOne folder per robot run, derived from downloaded sessions by '
                     '`tools/run-history.py`. Raw recordings are not here.\nWhat each file means and how to '
                     'add sessions: [docs/RUN_HISTORY.md](../docs/RUN_HISTORY.md).\n')
    all_runs = load_runs(runs)
    write_csv(runs / 'index.csv', INDEX_COLUMNS, index_rows(all_runs))
    write_if_changed(runs / 'SUMMARY.md', summary_report(all_runs))
    print(f'{len(sessions)} session(s) read; {len(all_runs)} runs in {runs}; {changed} run file(s) changed.')


if __name__ == '__main__':
    sys.exit(main())
