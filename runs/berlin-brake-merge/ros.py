"""Berlin ROS placement trials. Run in el-native-dev from /workspace."""
from pathlib import Path
import argparse
import json
import os
import subprocess

ROOT = Path('/workspace')
HERE = Path(__file__).resolve().parent
LAYOUTS = {
    'original': (94,30,[.083,-.083,.083]),
    'mirror': (94, 30, [-.083, .083, -.083]),
    'tight': (124, 18, [.12, -.12, .12]),
    'same-side': (304, 32, [.12, .12, .12]),
    'mid-bends': (197, 40, [-.10, .0, .10]),
    'distributed': (102, 142, [.08, -.12, .04]),
}
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--layouts', nargs='+', choices=list(LAYOUTS), default=list(LAYOUTS))
    parser.add_argument('--spread-cores', action='store_true', help='Use one logical CPU per physical core')
    parser.add_argument('--repeats', type=int, choices=[1,2], default=2)
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    layouts = {name: LAYOUTS[name] for name in args.layouts}
    out = args.output.resolve()
    out.relative_to(ROOT/'runs')
    out.mkdir(parents=True, exist_ok=args.resume)
    cores = sorted(os.sched_getaffinity(0))
    if args.spread_cores:
        physical = {}
        for core in cores:
            topology = Path(f'/sys/devices/system/cpu/cpu{core}/topology/thread_siblings_list')
            physical.setdefault(topology.read_text().strip(), core)
        cores = list(physical.values())
    selected = cores[2:6] if len(cores)>=6 else cores[:4]
    cpus = ','.join(map(str, selected))
    print('CPU affinity', cpus, flush=True)
    spec = out/'layouts.json'
    content = json.dumps(layouts,indent=2)+'\n'
    if args.resume:
        assert spec.read_text() == content
    else:
        spec.write_text(content)
    for rep in range(1, args.repeats + 1):
        for ix,(name,(first,stride,offsets)) in enumerate(layouts.items()):
            tag = f'{name}-{rep}'
            if args.resume and (out/(tag+'.csv')).exists():
                assert '"iterations"' in (out/(tag+'-run.log')).read_text()
                continue
            env=dict(os.environ, ROS_DOMAIN_ID=str(140+(rep-1)*len(LAYOUTS)+ix),
                EL_MAP_DIR='/workspace/maps/berlin', EL_START_INDEX='37', EL_STOP_AFTER_LAPS='3',
                EL_OBSTACLE_INDEX=str(first), EL_OBSTACLE_STRIDE=str(stride),
                EL_OBSTACLE_OFFSETS=json.dumps(offsets), EL_LAP_PLANNING='true', EL_SPEED_CAP='3.0', EL_SIM_AVOID_MIN='2.7', EL_SIM_AVOID_MAX='3.0',
                EL_AVOID_CHECKPOINT='/workspace/models/avoid-corridor-v2.pt',
                OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
            env.pop('EL_RACE_CHECKPOINT',None)
            if args.spread_cores and len(selected)>=4:
                env['EL_SIM_CPUSET'] = ','.join(map(str, selected[:2]))
                env['EL_CONTROL_CPUSET'] = ','.join(map(str, selected[2:]))
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
