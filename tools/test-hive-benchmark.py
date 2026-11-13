#!/usr/bin/env python3
"""Metrics must expose empty/frozen/wrong-pipeline streams, false UP and partial files."""
import importlib.util
import json
from pathlib import Path
import tempfile
import sys
sys.dont_write_bytecode = True
import unittest

spec = importlib.util.spec_from_file_location('analysis', Path(__file__).with_name('hive-benchmark-analyze.py'))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class BenchmarkTest(unittest.TestCase):
    def start(self, label='UP'):
        return {'event':'record_start', 't':0, 'pipeline':0, 'trial':0, 'pass':1, 'expected':label}

    def sample(self, t, fresh=True, hit=True, accepted=True, state='UNKNOWN'):
        return {'event':'sample', 'phase':'RECORD', 't':t, 'distinct':fresh,
                'raw_cell_hit':hit, 'accepted':accepted, 'state':state, 'tags':[], 'decode_error':''}

    def summary(self, samples, label='UP'):
        return m.summarize(self.start(label), 1, samples, {}, 'test', True)

    def test_duplicate_polls_do_not_improve_hit_rate(self):
        r = self.summary([self.sample(0), self.sample(.02, False), self.sample(.04, True, False, False)])
        self.assertEqual(r['fresh_frames'], 2)
        self.assertEqual(r['raw_cell_hit_pct'], 50)
        self.assertEqual(r['usable_cell_pct'], 50)

    def test_no_frames_and_stalls_are_visible(self):
        r = self.summary([])
        self.assertEqual(r['fresh_fps'], 0)
        self.assertEqual(r['raw_cell_hit_pct'], '')
        self.assertEqual(r['max_usable_gap_s'], 1)
        self.assertEqual(r['max_poll_gap_s'], 1)
        self.assertEqual(self.summary([self.sample(0, state='UP')])['ready_time_pct_est'], 5)

    def test_raw_tags_can_all_be_rejected(self):
        r = self.summary([self.sample(0, accepted=False)])
        self.assertEqual(r['raw_cell_hit_pct'], 100)
        self.assertEqual(r['usable_cell_pct'], 0)

    def test_down_false_up_includes_duplicate_polls(self):
        r = self.summary([self.sample(0, state='UP'), self.sample(.02, False, state='UP')], 'DOWN')
        self.assertEqual(r['false_up_polls'], 2)
        self.assertGreater(r['ready_time_pct_est'], 0)

    def test_partial_file_preserves_data(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)/'hive-test.jsonl'
            records = [{'event':'session', 'cell':34}, self.start(), self.sample(.1)]
            p.write_text(''.join(json.dumps(x)+'\n' for x in records)+'{"event":')
            rows, warnings = m.read_run(p)
            self.assertEqual(len(rows), 1)
            self.assertFalse(rows[0]['complete_trial'])
            self.assertEqual(len(warnings), 2)

    def test_advanced_record_end_ids_do_not_mix_trials(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)/'hive-test.jsonl'
            records = [self.start(), self.sample(.1), {'event':'record_end','t':1,'trial':1},
                       {'event':'stop','t':1.1,'complete':True,'restore':'restored'}]
            p.write_text(''.join(json.dumps(x)+'\n' for x in records))
            rows, warnings = m.read_run(p)
            self.assertEqual(rows[0]['trial'], 0)
            self.assertTrue(rows[0]['complete_trial'])
            self.assertEqual(warnings, [])


if __name__ == '__main__':
    unittest.main()
