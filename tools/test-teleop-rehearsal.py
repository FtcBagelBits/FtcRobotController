#!/usr/bin/env python3
"""Rehearsal converter: synthetic TeleOp recordings in, waypoints out. No robot."""
import csv
import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True  # loading the tool below must not leave tools/__pycache__ behind
ROOT = Path(__file__).resolve().parent.parent
TOOL = ROOT / 'tools' / 'teleop-rehearsal.py'
spec = importlib.util.spec_from_file_location('teleop_rehearsal', TOOL)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)

COLUMNS = ['t_s', 'wall_ms', 'kind', 'rehearsal', 'note', 'stick_forward', 'stick_strafe', 'stick_turn',
           'trigger_in', 'trigger_out', 'r1', 'l1', 'wheel_fl', 'wheel_fr', 'wheel_bl', 'wheel_br',
           'intake_cmd', 'agitator_cmd', 'launcher_target_tps', 'launcher_measured_tps', 'launching',
           'feeding', 'reversing', 'odom_status', 'x_in', 'y_in', 'heading_deg', 'raw_x_in', 'raw_y_in',
           'raw_heading_rad', 'battery_v', 'slowest_loop_ms']


def row(t, kind, rehearsal, x, y, heading, intake=0.0, note='', status='READY', launching=False):
    values = dict.fromkeys(COLUMNS, '0.0000')
    values.update(t_s=f'{t:.4f}', wall_ms='0', kind=kind, rehearsal=str(rehearsal), note=note,
                  r1='false', l1='false', launching=str(launching).lower(), feeding='false', reversing='false',
                  odom_status=status, x_in=f'{x:.4f}', y_in=f'{y:.4f}', heading_deg=f'{heading:.4f}',
                  intake_cmd=f'{intake:.4f}', battery_v='12.5000', slowest_loop_ms='21.0000')
    return values


def recording(path, rows):
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, COLUMNS, quoting=csv.QUOTE_ALL)
        writer.writeheader()
        writer.writerows(rows)


def l_shaped(start_x, start_y, heading):
    """Start, sit 1 s, drive 40 in along the robot's route-forward, pause 1 s, then 30 in to its left
    with the intake on. The start pose and heading are arbitrary: the result must not depend on them."""
    cos, sin = math.cos(math.radians(heading)), math.sin(math.radians(heading))
    def pose(forward, left):
        return start_x + forward * cos - left * sin, start_y + forward * sin + left * cos
    rows = [row(0, 'START', 0, start_x, start_y, heading, note='build=test')]
    rows.append(row(10, 'MARK_START', 1, *pose(0, 0), heading))
    t = 10.0
    for step in range(1, 141):
        t += 0.05
        if step <= 20:
            forward, left, intake = 0, 0, 0                       # 1 s still
        elif step <= 60:
            forward, left, intake = (step - 20) * 1.0, 0, 0       # 40 in forward in 2 s
        elif step <= 80:
            forward, left, intake = 40, 0, 0                      # 1 s pause at the corner
        else:
            forward, left, intake = 40, (step - 80) * 0.5, 0.8    # 30 in left in 3 s, intake on
        wobble = 0.2 * math.sin(math.pi * (step - 20) / 10)       # a human does not drive straight
        rows.append(row(t, 'SAMPLE', 1, *pose(forward, left + (wobble if 20 < step <= 60 else 0)), heading, intake))
    rows.append(row(t + 0.05, 'MARK_END', 1, *pose(40, 30), heading, 0.8))
    rows.append(row(t + 2, 'STOP', 0, *pose(40, 30), heading))
    return rows


class Args:
    tolerance_in = 1.0
    absolute = False
    out = None


with tempfile.TemporaryDirectory() as tmp:
    folder = Path(tmp)
    (folder / 'pollen-hive' / 'teleop').mkdir(parents=True)
    for name, pose in (('teleop-20261116-100000.csv', (0, 0, 0)), ('teleop-20261116-100500.csv', (-31.5, 12.25, 90))):
        recording(folder / 'pollen-hive' / 'teleop' / name, l_shaped(*pose))
    for path in tool.find_recordings(folder):
        with path.open(newline='') as f:
            rows = [r for r in csv.DictReader(f) if r['rehearsal'] == '1']
        marks = tool.waypoints(rows, tool.relative_path(rows, False), 1.0)
        assert len(marks) == 3, (path.name, [(m['forward'], m['left']) for m in marks])
        start, corner, end = marks
        assert abs(start['hold'] - 1.0) < 0.11, start
        assert abs(corner['forward'] - 40) < 0.6 and abs(corner['left']) < 0.6, corner
        assert abs(corner['hold'] - 1.0) < 0.11 and not corner['intake_into'], corner
        assert abs(end['forward'] - 40) < 0.6 and abs(end['left'] - 30) < 0.6 and end['intake_into'], end
        assert all(abs(m['heading']) < 0.01 for m in marks), 'heading is measured from the mark'
    # The same file measured from INIT (--absolute) keeps the recorded numbers.
    absolute = tool.relative_path(rows, True)
    assert abs(absolute[0][1] + 31.5) < 1e-6 and abs(absolute[0][3] - 90) < 1e-6

    # No position, a turn, an open rehearsal at the end of a cut-off file, and a run with no marks.
    recording(folder / 'pollen-hive' / 'teleop' / 'teleop-20261116-101000.csv', [
        row(0, 'START', 0, math.nan, math.nan, math.nan, status='NO PINPOINT'),
        row(1, 'MARK_START', 1, math.nan, math.nan, math.nan, status='NO PINPOINT'),
        row(2, 'SAMPLE', 1, math.nan, math.nan, math.nan, 0.5, status='NO PINPOINT', launching=True),
        row(3, 'MARK_END', 1, math.nan, math.nan, math.nan, status='NO PINPOINT', note='ended by STOP'),
        row(3, 'STOP', 0, math.nan, math.nan, math.nan, status='NO PINPOINT')])
    recording(folder / 'pollen-hive' / 'teleop' / 'teleop-20261116-102000.csv', [
        row(0, 'START', 0, 0, 0, 0), row(1, 'MARK_START', 1, 0, 0, 0),
        row(2, 'SAMPLE', 1, 20, 0, 45), row(3, 'SAMPLE', 1, 40, 0, 90)])
    recording(folder / 'pollen-hive' / 'teleop' / 'teleop-20261116-103000.csv', [
        row(0, 'START', 0, 0, 0, 0), row(5, 'STOP', 0, 0, 0, 0)])
    out = folder / 'analysis'
    result = subprocess.run([sys.executable, str(TOOL), str(folder), '--out', str(out)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    text = result.stdout
    assert 'move("R1 P1", 101.6, 0.0, 0, false),   // then hold 1.0 s' in text, text
    assert 'move("R1 P2", 101.6, 76.2, 0, true),' in text, text
    assert '76.2 cm @ +90.0 deg' in text and '-0.0' not in text, text
    assert 'No position was recorded' in text and 'launcher on: 1.0-2.0 s' in text and 'ended by: ended by STOP' in text
    assert 'The robot turned up to 90 deg' in text and 'the file stops mid-rehearsal' in text
    assert 'No STOP row: the file is incomplete' in text
    assert 'No rehearsals were marked' in text
    names = sorted(p.name for p in out.iterdir())
    assert 'teleop-20261116-100000-rehearsal-1.svg' in names and 'teleop-20261116-101000-rehearsal-1.md' in names, names
    assert 'teleop-20261116-101000-rehearsal-1.svg' not in names, 'no drawing without positions'
    assert '<polyline' in (out / 'teleop-20261116-100000-rehearsal-1.svg').read_text()
    # Old downloads (before recordings existed) are not an error.
    empty = subprocess.run([sys.executable, str(TOOL), str(out)], capture_output=True, text=True)
    assert empty.returncode == 0 and 'No teleop-*.csv recordings' in empty.stdout
print('PASS teleop rehearsal: waypoints independent of start pose, pauses, intake legs, turns, missing positions, cut-off files')
