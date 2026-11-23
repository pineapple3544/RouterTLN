"""Run both preregistered repetitions, retaining every attempt including infrastructure failures."""
import json
import os
import subprocess
import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
FROZEN = HERE.parent/'berlin-blind-300'
TAGS = json.loads((FROZEN/'preregister.json').read_text())['ros_selected']
LAYOUTS = json.loads((FROZEN/'layouts.json').read_text())


def run_group(group, destination, binary):
    out = destination/f'group{group}'
    out.mkdir(parents=True)
    sim_cpus, control_cpus = [('0,2','4,6'), ('8,10','12,14')][group]
    for rep in (1, 2):
        for ix, name in enumerate(TAGS[group::2]):
            first, stride, offsets = LAYOUTS[name]
            tag = f'{name}-{rep}'
            env = dict(os.environ, ROS_DOMAIN_ID=str(32+ix+group*54+(rep-1)*16),
                       EL_CONTROLLER_BINARY=str(binary), EL_LAP_PLANNING='true',
                       EL_SIM_CPUSET=sim_cpus, EL_CONTROL_CPUSET=control_cpus, EL_START_INDEX='37',
                       EL_OBSTACLE_INDEX=str(first), EL_OBSTACLE_STRIDE=str(stride),
                       EL_OBSTACLE_OFFSETS=json.dumps(offsets), EL_AVOID_CHECKPOINT='/workspace/models/avoid-corridor-v2.pt',
                       EL_SPEED_CAP='3.0', EL_SIM_AVOID_MIN='2.7', EL_SIM_AVOID_MAX='3.0',
                       EL_INFERENCE_DEVICE='cpu', EL_STOP_AFTER_LAPS='3', EL_MAP_DIR='/workspace/maps/berlin')
            env.pop('EL_UNIFIED_CHECKPOINT', None)
            env.pop('EL_RACE_CHECKPOINT', None)
            with (out/(tag+'-run.log')).open('w') as log:
                runner = subprocess.run(['taskset','-c',sim_cpus+','+control_cpus,'bash','scripts/smoke.sh','105','3','moe',
                                         str((out/tag).relative_to(ROOT/'runs'))], cwd=ROOT, env=env, stdout=log, stderr=log)
            result = out/(tag+'-score.json')
            data = json.loads(result.read_text()) if result.exists() else {'accepted':False,'failure':'missing_score'}
            data.update(seed=name, repetition=rep, runner_exit=runner.returncode)
            if runner.returncode:
                data['accepted'] = False
            result.write_text(json.dumps(data, indent=2)+'\n')
            print(tag, data.get('accepted'), data.get('physical_laps'), data.get('emergency'), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=HERE/'ros')
    parser.add_argument('--controller', type=Path, default=Path('/build/el_controller'))
    args = parser.parse_args()
    destination = args.output.resolve()
    destination.relative_to(ROOT/'runs')
    if destination.exists():
        parser.error('output already exists')
    binary = args.controller.resolve(strict=True)
    with ThreadPoolExecutor(max_workers=2) as workers:
        futures = [workers.submit(run_group, group, destination, binary) for group in range(2)]
        for future in futures:
            future.result()
