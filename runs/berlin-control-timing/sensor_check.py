"""Pause only the bridge spawned for a private watchdog test; retain the stop trace."""
from pathlib import Path
import csv
import json
import os
import signal
import subprocess
import time

root=Path('/workspace')
import sys
out=Path(sys.argv[1]).resolve()
out.relative_to(root/'runs')
out.mkdir(exist_ok=False)
tag=str((out/'check').relative_to(root/'runs'))
domain='204'
env=dict(os.environ, ROS_DOMAIN_ID=domain, EL_STOP_AFTER_LAPS='3', EL_LAP_PLANNING='true',
         EL_MAP_DIR='/workspace/maps/berlin',EL_START_INDEX='37',EL_SPEED_CAP='3.0',
         EL_SIM_AVOID_MIN='2.7',EL_SIM_AVOID_MAX='3.0',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
with (out/'run.log').open('w') as log:
    smoke=subprocess.Popen(['bash','scripts/smoke.sh','40','0','moe',tag],cwd=root,env=env,stdout=log,stderr=log)
    deadline=time.monotonic()+35
    while not (out/'check.csv').exists() or (out/'check.csv').stat().st_size<12000:
        if smoke.poll() is not None or time.monotonic()>deadline:raise RuntimeError('controller did not start')
        time.sleep(.05)
    bridges=[]
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            if ((proc/'comm').read_text().strip()=='gym_bridge' and
                b'ROS_DOMAIN_ID=204' in (proc/'environ').read_bytes().split(bytes([0]))):
                bridges.append(int(proc.name))
        except (OSError,ProcessLookupError):continue
    if len(bridges)!=1:raise RuntimeError('expected exactly the dedicated bridge')
    pid=bridges[0]
    start=time.monotonic()
    os.kill(pid,signal.SIGSTOP)
    try:time.sleep(.24)
    finally:os.kill(pid,signal.SIGCONT)
    held=time.monotonic()-start
    if smoke.wait(timeout=60):raise RuntimeError('watchdog harness failed')
with (out/'check.csv').open() as f:rows=list(csv.DictReader(f))
fault=next(i for i,r in enumerate(rows) if r['mode']=='3')
assert rows[fault]['reason'] in ('scan_timeout','sensor_timeout','odom_sync_timeout')
assert any(r['valid']=='1' and float(r['target_speed'])>0 for r in rows[:fault])
assert all(r['valid']=='0' and float(r['target_speed'])==0 for r in rows[fault:])
assert float(rows[-1]['speed'])<.05 and all(r['collision']=='0' for r in rows)
print(json.dumps(dict(bridge_pause_ms=held*1000,reason=rows[fault]['reason'],
                      final_speed=float(rows[-1]['speed']),first_bad_step=rows[fault]['step'])))
