#!/usr/bin/env python3
"""Long-run test matrix (Task 1). Run inside the sim container from /workspace.

  python3 runs/longrun/run.py --mode F --laps 10            # Mode F, 10-lap eval-format runs
  python3 runs/longrun/run.py --mode F --laps 20            # Mode F, 20-lap endurance runs
  python3 runs/longrun/run.py --mode R --laps 10 --layouts A C

Mode F: el_scenario places a fixed layout from config/longrun_layouts.json.
Mode R: the shipped race_manager obstacle_spawner + scorer (unmodified) re-draw obstacles every
lap; el_scenario only supplies the clock and start pose (EL_SCENARIO_OBSTACLES=0).

Each run lands in runs/longrun-<date>/<mode>/<layout>-obs<N>-<laps>L-s<noise seed>/ as
run.csv, run-judge.csv, run-controller.log, run-sim.log, run-scenario.log, run-score.json
(+ race_result_*.json and spawner/scorer logs in Mode R). Existing finished runs are skipped,
so the script can be restarted.
"""
import argparse
import datetime
import json
import os
import signal
import subprocess
import time
from pathlib import Path

ROOT = Path('/workspace')
LAYOUTS = json.loads((ROOT / 'config/longrun_layouts.json').read_text())['layouts']
RM_SHARE = '/home/misys/forza_ws/race_stack/install/race_manager/share/race_manager'
WAYPOINTS = '/workspace/maps/berlin/global_waypoints.json'

# One fixed controller configuration, shared with run_demo.sh. Identical for every run.
FINAL_CONFIG = dict(line.split('=', 1) for line in (ROOT / 'config/final_demo.env').read_text().splitlines()
                    if line.strip() and not line.lstrip().startswith('#'))
NOISE_SEEDS = {10: [51001, 51002], 20: [51001]}
BUDGET_S = {10: 200, 20: 480}   # wall-clock cap inside smoke.sh; eval cutoff (180 s) is applied later
R_START_INDEX = 0               # Mode R starts on the race_manager line, like the grader's sim


def run_one(out_root, mode, layout, count, laps, noise_seed, extra_env, noise_scale=1.0, variant='', rm_seed=None):
    name = f'{layout}-obs{count}-{laps}L-s{noise_seed}' + (f'-rm{rm_seed}' if rm_seed is not None else '')
    label = mode if noise_scale == 1 else f'{mode}-noise{noise_scale:g}x'
    if variant:
        label += f'-{variant}'
    rel = Path(out_root.name) / label / name
    run_dir = ROOT / 'runs' / rel
    if (run_dir / 'run-score.json').exists():
        print('skip', rel, flush=True)
        return
    run_dir.mkdir(parents=True, exist_ok=True)
    cfg = LAYOUTS[layout]['configs'][str(count)]
    env = dict(os.environ, **FINAL_CONFIG, **extra_env)
    env.update(EL_STOP_AFTER_LAPS=str(laps), EL_SCAN_NOISE_SEED=str(noise_seed),
               EL_SCAN_NOISE_SCALE=f'{noise_scale:g}')
    env.pop('EL_RACE_DURATION_S', None)
    procs = []
    spec = dict(mode=label, obstacle_mode=mode, noise_scale=noise_scale, variant=variant or None,
                extra_env=extra_env, layout=layout, obstacles=count, target_laps=laps, noise_seed=noise_seed,
                config=FINAL_CONFIG, started=datetime.datetime.now().isoformat(timespec='seconds'))
    if mode == 'F':
        env.update(EL_START_INDEX=str(cfg['start_index']), EL_OBSTACLE_INDEX=str(cfg['obstacle_index']),
                   EL_OBSTACLE_STRIDE=str(cfg['obstacle_stride']),
                   EL_OBSTACLE_OFFSETS=json.dumps(cfg['obstacle_offsets']))
        scenario_count = count
        spec['layout_config'] = cfg
    else:
        seed = rm_seed if rm_seed is not None else LAYOUTS[layout]['seed']
        env.update(EL_START_INDEX=str(R_START_INDEX), EL_SCENARIO_OBSTACLES='0', EL_OBSTACLE_OFFSETS='')
        scenario_count = 0
        spec.update(race_manager_seed=seed, start_index=R_START_INDEX)
        params = f'{RM_SHARE}/config/race_manager_params.yaml'
        ros = 'source /opt/ros/humble/setup.bash; source /home/misys/forza_ws/race_stack/install/setup.bash; exec '
        spawner = (f'{ros} ros2 run race_manager obstacle_spawner --ros-args --params-file {params} '
                   f'-p waypoints_file:={WAYPOINTS} -p seed:={seed} -p num_obstacles:={count}')
        scorer = (f'{ros} ros2 run race_manager scorer --ros-args --params-file {params} '
                  f'-p waypoints_file:={WAYPOINTS} -p map_name:=berlin -p seed:={seed} '
                  f'-p log_dir:={run_dir}')
        for label, cmd in (('spawner', spawner), ('scorer', scorer)):
            log = open(run_dir / f'{label}.log', 'w')
            procs.append(subprocess.Popen(['bash', '-c', cmd], stdout=log, stderr=subprocess.STDOUT,
                                          env=env, start_new_session=True))
        time.sleep(3)
    (run_dir / 'spec.json').write_text(json.dumps(spec, indent=2) + '\n')
    t0 = time.time()
    smoke = subprocess.run(['bash', 'scripts/smoke.sh', str(BUDGET_S[laps]), str(scenario_count), 'moe',
                            str(rel / 'run'), '', str(LAYOUTS[layout]['seed'])],
                           cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for p in procs:   # SIGINT lets the scorer write its final JSON
        try:
            os.killpg(p.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    for p in procs:
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
    (run_dir / 'smoke.out').write_text(smoke.stdout)
    if not (run_dir / 'run-score.json').exists():
        (run_dir / 'run-score.json').write_text(json.dumps({'accepted': False, 'failure': 'runner_no_score',
                                                            'smoke_returncode': smoke.returncode}) + '\n')
    print(f'{rel}  rc={smoke.returncode}  {time.time() - t0:.0f}s  '
          f'{(run_dir / "run-score.json").read_text().strip()[:160]}', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices='FR', required=True)
    ap.add_argument('--laps', type=int, choices=(10, 20), required=True)
    ap.add_argument('--layouts', nargs='+', default=list(LAYOUTS))
    ap.add_argument('--obstacles', nargs='+', type=int, default=[1, 3, 5])
    ap.add_argument('--seeds', nargs='+', type=int, default=None)
    ap.add_argument('--noise-scale', type=float, default=1.0,
                    help='scan range noise as a multiple of the Gym 1 cm (Task 2)')
    ap.add_argument('--variant', default='', help='label for a controller variant (added to the mode directory)')
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                    help='extra environment for the controller, e.g. EL_LOCALIZATION_GRACE_SCANS=2')
    ap.add_argument('--rm-seed', type=int, default=None, help='Mode R: race_manager seed instead of the layout seed')
    ap.add_argument('--date', default=datetime.date.today().strftime('%Y%m%d'))
    args = ap.parse_args()
    out_root = ROOT / 'runs' / f'longrun-{args.date}'
    extra = dict(kv.split('=', 1) for kv in args.set)
    if extra and not args.variant:
        ap.error('--set changes the controller: give it a --variant label')
    for layout in args.layouts:
        for count in args.obstacles:
            for seed in args.seeds or NOISE_SEEDS[args.laps]:
                run_one(out_root, args.mode, layout, count, args.laps, seed, extra, args.noise_scale, args.variant, args.rm_seed)


if __name__ == '__main__':
    main()
