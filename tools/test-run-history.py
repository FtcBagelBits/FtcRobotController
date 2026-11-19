#!/usr/bin/env python3
"""Fake-data tests for tools/run-history.py. Never reads a real download or the robot."""
import csv
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile

TOOL = Path(__file__).resolve().parent / 'run-history.py'
RUN = 'match-1795054654268-811650241'
TRACE_HEADER = ['t_s', 'wall_ms', 'kind', 'phase', 'reason', 'decision_inputs', 'execution', 'odom_status',
                'x_in', 'y_in', 'heading_deg', 'requested_launcher_tps', 'measured_launcher_tps',
                'feed_requested', 'hive_state', 'hive_reason', 'cell', 'camera_status', 'battery_v', 'tags_id_xyz_m_height_in']


def report():
    lines = [
        '0.000s build=pollen-hive-v33 run=match',
        '0.010s START build=pollen-hive-v33 opmode=PollenHivePrimaryAuto wall_ms=1795054654270 run=MATCH '
        'secondary=false makeRoom=false settings=execution=2.0, firstFeed=3.0 camera={} unsaved_route=0',
        '0.015s HUB installed=2026-11-18T18:14:39 trace=' + RUN + '.txt.csv storage=[1 GB free]',
        '0.100s FIRST / UNKNOWN / feed=false reason= poseIn=0.00,0.00 headingDeg=0.00',
        '1.000s FIRST / UP / feed=true reason=Feeding primary preload poseIn=0.00,0.00 headingDeg=0.00',
        '4.000s OUTBOUND: CLEAR END WALL / UNKNOWN / feed=false reason=First feed done poseIn=0.00,0.00 headingDeg=0.00',
        '6.000s GARDEN: Inward pass 1 / UNKNOWN / feed=false reason=First feed done poseIn=13.00,50.00 headingDeg=0.00',
        '10.000s RETURN: RETURN ACROSS / UNKNOWN / feed=false reason=Collection complete poseIn=13.00,50.00 headingDeg=0.00',
        '14.000s SETTLE / UNKNOWN / feed=false reason=Collection complete poseIn=0.30,0.40 headingDeg=359.00',
        '15.500s SECOND / UP / feed=false reason=Collection complete poseIn=0.30,0.40 headingDeg=359.00',
        '16.000s SECOND / UP / feed=true reason=Feeding: hive UP latched poseIn=0.30,0.40 headingDeg=359.00',
        '19.000s PARK: PARK CLEAR / UNKNOWN / feed=false reason=Park deadline during SECOND poseIn=0.30,0.40 headingDeg=0.00',
        '22.000s DONE / UNKNOWN / feed=false reason=Finished departure poseIn=87.00,43.00 headingDeg=0.00',
        '23.000s STOP requested_by_driver previous=DONE Finished departure',
    ]
    return '\n'.join(lines) + '\n'


def trace():
    rows = []
    def add(t, phase, feed=False, meas=1180, req=1180, kind='SAMPLE', decision='requested_s=3.000', x=0, y=0, reason='', h=0.0):
        rows.append([f'{t:.4f}', str(1795054654270 + int(t * 1000)), kind, phase, reason, decision, '2.0000', 'READY',
                     f'{x:.4f}', f'{y:.4f}', f'{h:.4f}', f'{req:.4f}', f'{meas:.4f}', 'true' if feed else 'false',
                     'UP', 'Stable UP, 50.6 in', '42', 'OK', '12.5000' if t < 5 else '11.9000', ''])
    add(0.1, 'FIRST', req=0, meas=0, kind='TRANSITION')
    add(0.2, 'FIRST', meas=200)
    for i in range(30):  # First volley 1.0 .. 3.9 s with one dip at 2.0 s.
        t = 1.0 + i * 0.1
        add(t, 'FIRST', feed=True, meas=1000 if abs(t - 2.0) < 0.05 else 1180)
    add(4.0, 'OUTBOUND: CLEAR END WALL', decision='clear_out_s_left=3.0', meas=1000)
    add(4.1, 'OUTBOUND: CLEAR END WALL', decision='clear_out_s_left=2.9', meas=1180)
    add(6.0, 'GARDEN: Inward pass 1', req=0, meas=0, x=13, y=50)
    add(10.0, 'RETURN: RETURN ACROSS', req=0, meas=0, x=13, y=50)
    add(14.0, 'SETTLE', x=0.3, y=0.4)
    add(15.5, 'SECOND', x=0.3, y=0.4, h=359.0)
    for i in range(30):
        add(16.0 + i * 0.1, 'SECOND', feed=True, x=0.3, y=0.4)
    add(19.0, 'PARK: PARK CLEAR', reason='Park deadline during SECOND', req=0, meas=0)
    add(22.0, 'DONE', kind='TRANSITION', reason='Finished departure', req=0, meas=0, x=87, y=43)
    add(40.0, 'DONE', req=0, meas=0, x=87, y=43)  # After DONE + 1 s: left out of the slim trace.
    return rows


def write_session(folder, report_text, trace_rows, extra=None):
    hive = folder / 'pollen-hive'
    for sub in ('reports', 'traces', 'auto', 'teleop'):
        (hive / sub).mkdir(parents=True, exist_ok=True)
    (hive / 'reports' / (RUN + '.txt')).write_text(report_text)
    with (hive / 'traces' / (RUN + '.txt.csv')).open('w', newline='') as f:
        w = csv.writer(f, quoting=csv.QUOTE_ALL)
        w.writerow(TRACE_HEADER)
        w.writerows(trace_rows)
    (hive / 'secondary.properties').write_text('not a run\n')
    for rel, text in (extra or {}).items():
        (hive / rel).write_text(text)
    (folder / 'logcat.txt').write_text('raw SDK log: must never be copied\n')


ODO = ('"t_s","wall_ms","text"\n'
       '"0.0251","1794877463197","START Odo ROBOT Shoot + Park build=pollen-hive-v10 route=FULL_PARK FLYWHEEL=1180.0 installed=2026-11-16T12:21:57 battery=11.39V"\n'
       '"0.0780","1794877463250","t=0.03 PHASE SQUARE TO WALL | x=-3.6 y=-0.2 h=88.0 battery=11.03V"\n'
       '"18.0200","1794877481192","t=17.99 RESULT DONE - all outputs zero | x=87.7 y=42.9 h=0.0 battery=11.24V"\n'
       '"22.4903","1794877485662","t=22.49 STOP (OpMode ended; last phase: DONE - all outputs zero)"\n')
PRACTICE = ('"START","pollen-hive-v25","flower-practice","{gap=24.0}"\n'
            '"seconds","phase","route_x_in","route_y_in","heading_deg","intake","drainer_down","event"\n'
            '"0.0331","Lower drainer","93.1","207.1","278.7","1.0000","true",""\n'
            '"1.2051","FAULT","Approach flower","Approach flower no odometry progress"\n'
            '"30.1228","STOP","FAULT"\n')
TELEOP = ('"t_s","wall_ms","kind","rehearsal","note","r1","slowest_loop_ms","battery_v"\n'
          '"0.0000","1795059433154","START","0","build=pollen-hive-v29 installed=2026-11-18T19:01:21 opmode=PollenBot settings=DEFAULTS","false","0","12.3"\n'
          '"0.1000","1795059433254","SAMPLE","0","","true","12","12.2"\n'
          '"0.2000","1795059433354","SAMPLE","0","","true","30","12.1"\n'
          '"0.3000","1795059433454","SAMPLE","0","","false","8","12.0"\n'
          '"0.4000","1795059433554","MARK_START","1","","false","8","12.0"\n'
          '"0.5000","1795059433654","SAMPLE","0","","true","8","12.0"\n')


class RunHistoryTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.runs = self.tmp / 'runs'
        self.session = self.tmp / 'practice-20261118-185018'
        write_session(self.session, report(), trace(), {
            'auto/auto-20261116-170423.csv': ODO, 'auto/auto-20261118-175039.csv': PRACTICE,
            'teleop/teleop-20261118-193713.csv': TELEOP})

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def tool(self, *args, ok=True):
        result = subprocess.run([sys.executable, '-I', str(TOOL), '--runs', str(self.runs), *map(str, args)],
                                capture_output=True, text=True)
        if ok:
            self.assertEqual(result.returncode, 0, result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0)
        return result

    def index(self):
        with (self.runs / 'index.csv').open(newline='') as f:
            return {r['run_id']: r for r in csv.DictReader(f)}

    def snapshot(self):
        return {p.relative_to(self.runs).as_posix(): p.read_bytes() for p in sorted(self.runs.rglob('*')) if p.is_file()}

    def test_hive_metrics_from_report_and_trace(self):
        self.tool(self.session)
        r = self.index()[RUN]
        self.assertEqual((r['kind'], r['build'], r['opmode'], r['execution'], r['cell']),
                         ('hive', 'pollen-hive-v33', 'PollenHivePrimaryAuto', 'LIVE', '42'))
        self.assertEqual(r['hub_start'], '2026-11-18T18:17:34-08:00')
        self.assertEqual((r['feed1_start'], r['feed1_end'], r['feed2_start'], r['feed2_end']),
                         ('1.00', '3.90', '16.00', '18.90'))
        self.assertEqual((r['both_volleys'], r['feed2_cut_by_park'], r['stopped_early']), ('true', 'true', 'false'))
        self.assertEqual((r['dips_feed1'], r['last_dip_into_feed1_s'], r['dips_feed2']), ('1', '1.00', '0'))
        self.assertEqual((r['clear_out_s'], r['dips_clear_out']), ('0.10', '1'))
        self.assertEqual((r['spin_up_s'], r['up_wait_s'], r['settle_align_s']), ('0.80', '0.50', '1.50'))
        self.assertEqual((r['return_error_in'], r['return_heading_deg']), ('0.50', '-1.00'))
        self.assertEqual((r['final_x_in'], r['battery_start_v'], r['battery_min_v']), ('87.00', '12.50', '11.90'))
        self.assertEqual(r['park_reason'], 'Park deadline during SECOND')
        self.assertEqual(r['feed1_requested_s'], '3.00')
        self.assertEqual(r['fault'], '')  # Blank, not 0, when nothing happened.
        folder = self.runs / '2026-11-18' / RUN
        slim = list(csv.DictReader((folder / 'slim-trace.csv').read_text().splitlines()))
        self.assertEqual(slim[-1]['t_s'], '22.00')  # Stops 1 s after the first DONE.
        self.assertNotIn('tags_id_xyz_m_height_in', slim[0])
        self.assertEqual(slim[0]['hive_reason'], 'Stable UP, 50.6 in')  # Why the hive read UP/DOWN/UNKNOWN.
        self.assertEqual(sum(r['clear_out'] == 'true' for r in slim), 2)
        self.assertEqual(len(list(csv.reader((folder / 'timeline.csv').read_text().splitlines()))), 11)  # header + 10 transitions
        self.assertIn('Settings at START', (folder / 'summary.md').read_text())
        self.assertTrue((folder / 'notes.md').exists())

    def test_other_kinds(self):
        self.tool(self.session)
        idx = self.index()
        odo, practice, teleop = idx['auto-20261116-170423'], idx['auto-20261118-175039'], idx['teleop-20261118-193713']
        self.assertEqual((odo['kind'], odo['build'], odo['result'], odo['battery_min_v']),
                         ('odo_auto', 'pollen-hive-v10', 'DONE - all outputs zero', '11.03'))
        self.assertEqual(odo['opmode'], 'Odo ROBOT Shoot + Park')
        self.assertEqual((practice['kind'], practice['opmode'], practice['result']),
                         ('collection_practice', 'flower-practice', 'FAULT'))
        self.assertIn('no odometry progress', practice['fault'])
        self.assertEqual((teleop['kind'], teleop['r1_presses'], teleop['rehearsals'], teleop['slowest_loop_ms']),
                         ('teleop', '2', '1', '30.00'))
        self.assertFalse((self.runs / '2026-11-18' / 'teleop-20261118-193713' / 'notes.md').exists())
        self.assertEqual(len(idx), 4)  # Settings files and raw SDK logs are not runs.
        text = ''.join(p.read_text() for p in self.runs.rglob('*') if p.is_file())
        self.assertNotIn('raw SDK log', text)

    def test_twice_changes_nothing_and_notes_survive(self):
        self.tool(self.session)
        notes = self.runs / '2026-11-18' / RUN / 'notes.md'
        notes.write_text(notes.read_text().replace('- Balls left in the robot after the first volley (counted at the garden or flower):',
                                                   '- Balls left in the robot after the first volley (counted at the garden or flower): 2'))
        self.tool(self.session)
        before = self.snapshot()
        self.tool(self.session)
        self.tool()
        after = self.snapshot()
        self.assertEqual([k for k in after if before.get(k) != after[k]], [])
        self.assertIn('Balls left in the robot after the first volley (counted at the garden or flower): 2',
                      notes.read_text())  # Never overwritten.
        self.assertEqual(self.index()[RUN]['notes_filled'], 'yes')
        self.assertIn('| 2 |', (self.runs / 'SUMMARY.md').read_text())

    def test_same_run_in_two_sessions_keeps_the_longer_copy(self):
        early = self.tmp / 'early-20261118-180000'
        write_session(early, '\n'.join(report().splitlines()[:6]) + '\n', trace()[:10])
        self.tool(early)
        self.assertEqual(self.index()[RUN]['feed2_start'], '')
        zipped = self.tmp / 'zips'
        zipped.mkdir()
        with zipfile.ZipFile(zipped / (self.session.name + '.zip'), 'w') as z:
            for p in self.session.rglob('*'):
                if p.is_file():
                    z.write(p, self.session.name + '/' + p.relative_to(self.session).as_posix())
        self.tool(zipped)  # A folder of sessions; the ZIP is read without unpacking.
        r = self.index()[RUN]
        self.assertEqual((r['feed2_start'], r['first_seen_session']), ('16.00', 'early-20261118-180000'))
        with (self.runs / 'sources.csv').open() as f:
            sessions = {row['session'] for row in csv.DictReader(f) if row['run_id'] == RUN}
        self.assertEqual(sessions, {'early-20261118-180000', self.session.name})
        meta = json.loads((self.runs / '2026-11-18' / RUN / 'metrics.json').read_text())
        self.assertEqual(len(meta['sources']), 2)

    def test_report_without_trace_still_gives_phase_times(self):
        hive = self.session / 'pollen-hive'
        (hive / 'traces' / (RUN + '.txt.csv')).unlink()
        self.tool(self.session)
        r = self.index()[RUN]
        self.assertEqual((r['feed1_start'], r['second_start'], r['trace_rows'], r['min_launcher_tps_feed1']),
                         ('1.00', '15.50', '', ''))
        self.assertFalse((self.runs / '2026-11-18' / RUN / 'slim-trace.csv').exists())

    def test_session_without_hive_folder_fails(self):
        empty = self.tmp / 'broken-20261118-120000'
        empty.mkdir()
        (empty / 'logcat.txt').write_text('x')
        self.tool(empty, ok=False)


if __name__ == '__main__':
    unittest.main(verbosity=2)
