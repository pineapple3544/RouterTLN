"""Berlin ROS placement trials. Run in el-native-dev from /workspace."""
from pathlib import Path
import argparse
import json
import os
import subprocess

ROOT = Path('/workspace')
HERE = Path(__file__).resolve().parent
LAYOUTS = {
    'mirror': (94, 30, [-.083, .083, -.083]),
    'tight': (124, 18, [.12, -.12, .12]),
    'same-side': (304, 32, [.12, .12, .12]),
    'mid-bends': (197, 40, [-.10, .0, .10]),
    'distributed': (102, 142, [.08, -.12, .04]),
}
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    out = parser.parse_args().output.resolve()
    out.relative_to(ROOT/'runs')
    out.mkdir(parents=True, exist_ok=False)
    cores = sorted(os.sched_getaffinity(0))
    cpus = ','.join(map(str, cores[2:6] if len(cores)>=6 else cores[:4]))
    (out/'layouts.json').write_text(json.dumps(LAYOUTS,indent=2)+'\n')
    for rep in (1, 2):
        for ix,(name,(first,stride,offsets)) in enumerate(LAYOUTS.items()):
            tag = f'{name}-{rep}'
            env=dict(os.environ, ROS_DOMAIN_ID=str(140+(rep-1)*5+ix),
                EL_MAP_DIR='/workspace/maps/berlin', EL_START_INDEX='37', EL_STOP_AFTER_LAPS='3',
                EL_OBSTACLE_INDEX=str(first), EL_OBSTACLE_STRIDE=str(stride),
                EL_OBSTACLE_OFFSETS=json.dumps(offsets), EL_LAP_PLANNING='true', EL_SPEED_CAP='3.0',
                OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
            env.pop('EL_RACE_CHECKPOINT',None)
            cmd=['taskset','-c',cpus,'bash','scripts/smoke.sh','130','3','moe',
                 str((out/tag).relative_to(ROOT/'runs'))]
            print('START',tag,flush=True)
            with (out/(tag+'-poses.yaml')).open('w') as poses, (out/(tag+'-run.log')).open('w') as log:
                probe=subprocess.Popen(['bash','-c',
                    'source /opt/ros/humble/setup.bash && exec timeout 30 ros2 topic echo --once /sim/obstacles geometry_msgs/msg/PoseArray'],
                    env=env,stdout=poses,stderr=subprocess.DEVNULL)
                ret=subprocess.run(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
                probe_rc=probe.wait()
            print('END',tag,'runner',ret.returncode,'pose_probe',probe_rc,flush=True)
            if ret.returncode or probe_rc:
                raise SystemExit('Trial infrastructure failed: '+tag)
