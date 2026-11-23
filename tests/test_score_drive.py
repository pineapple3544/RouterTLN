import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from score_drive import score


def write_csv(path, header, rows):
    with path.open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


class JudgeTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        folder = Path(temp.name)
        self.controls, self.physics, self.map = [folder/name for name in ('car.csv','truth.csv','track.json')]
        self.line = [(0,0),(1,0),(1,1),(0,1)]
        self.map.write_text(json.dumps({'global_traj_wpnts_iqp':{
            'wpnts':[{'x_m':x,'y_m':y} for x,y in self.line]}}))
        write_csv(self.controls, ['stamp','mode','reason','collision','progress_m'],
                  [[0,0,'race','',0], [.08,0,'race','',999]])

    def run_case(self, points, collision_at=None, stop_at=None):
        write_csv(self.physics, ['stamp','x','y','speed','collision'],
                  [(i*.01, x, y, 0 if i==stop_at else 1, 'wall' if i==collision_at else 'none')
                   for i,(x,y) in enumerate(points)])
        return score(self.controls,self.physics,self.map,2)

    def test_timed_window_counts_all_laps_and_ignores_later_contact(self):
        points = self.line * 13 + [self.line[0]]
        write_csv(self.controls, ['stamp','mode','reason','collision','progress_m'],
                  [[0,0,'race','',0], [.52,0,'race','',0]])
        self.run_case(points, collision_at=50)
        scored = score(self.controls,self.physics,self.map,0,.48)
        self.assertTrue(scored['accepted'])
        self.assertEqual(scored['consecutive_laps'], 12)
        self.assertAlmostEqual(scored['best_lap_s'], .04)
        late = score(self.controls,self.physics,self.map,0,.51)
        self.assertFalse(late['accepted'])
        self.assertEqual(late['consecutive_laps'], 12)
        short = score(self.controls,self.physics,self.map,0,.60)
        self.assertEqual(short['failure'], 'short_timed_run')

    def test_fault_is_scoped_to_the_timed_window(self):
        self.run_case(self.line * 2 + [self.line[0]])
        log = self.controls.with_name('car-controller.log')
        log.write_text('fault_stamp=0.07 control_fault=sensor_timeout')
        self.assertTrue(score(self.controls,self.physics,self.map,0,.06)['accepted'])
        scored = score(self.controls,self.physics,self.map,0,.08)
        self.assertFalse(scored['accepted'])
        self.assertEqual(scored['consecutive_laps'], 1)
        self.assertEqual(scored['controller_fault'], 'sensor_timeout')

    def test_stop_after_third_lap_fails_timed_run(self):
        write_csv(self.controls, ['stamp','mode','reason','collision','progress_m'],
                  [[0,0,'race','',0], [.20,0,'race','',0]])
        self.run_case(self.line * 5 + [self.line[0]], stop_at=15)
        scored = score(self.controls,self.physics,self.map,0,.20)
        self.assertFalse(scored['accepted'])
        self.assertEqual(scored['consecutive_laps'], 3)

    def test_requires_physical_crossings(self):
        result = self.run_case(self.line*2 + [self.line[0]])
        self.assertEqual(result['physical_laps'], 2)
        self.assertTrue(result['accepted'])
        short = self.run_case(self.line + [self.line[0]]*5)
        self.assertEqual(short['physical_laps'], 1)
        self.assertFalse(short['accepted'])

    def test_contact_between_command_frames(self):
        result = self.run_case(self.line*2 + [self.line[0]], collision_at=3)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['collision_samples'], 1)
        with self.controls.open() as stream:
            self.assertEqual([r['collision'] for r in csv.DictReader(stream)], ['0','1'])

    def test_stop_before_crossing_and_missing_judge_coverage(self):
        result = self.run_case(self.line*2 + [self.line[0]], stop_at=3)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['stopped_samples_before_finish'], 1)
        self.assertEqual(self.run_case(self.line)['failure'], 'incomplete_judge_coverage')


if __name__ == '__main__':
    unittest.main()
