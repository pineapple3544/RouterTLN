"""Observe the 150 ms scan guard on an isolated ROS domain during a bridge pause."""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

ROOT = Path('/workspace')
HERE = Path(__file__).resolve().parent


def main():
    output = Path(sys.argv[1]).resolve()
    tag = str((output/'check').relative_to(ROOT/'runs'))
    output.mkdir()
    domain = '204'
    os.environ['ROS_DOMAIN_ID'] = domain
    env = dict(os.environ, EL_STOP_AFTER_LAPS='3', EL_LAP_PLANNING='true', EL_START_INDEX='37',
               EL_SIM_AVOID_MIN='2.7', EL_SIM_AVOID_MAX='3.0', EL_SPEED_CAP='3.0',
               EL_SIM_CPUSET='3', EL_CONTROL_CPUSET='5', EL_MAP_DIR='/workspace/maps/berlin')
    rclpy.init()
    monitor = rclpy.create_node('scan_guard_observer')
    clock = Clock(clock_type=ClockType.STEADY_TIME)
    def now():
        return clock.now().nanoseconds * 1e-9
    commands, acquisitions = [], []
    monitor.create_subscription(AckermannDriveStamped, '/drive',
                                lambda m: commands.append((now(), m.drive.speed)), 20)
    monitor.create_subscription(LaserScan, '/scan', lambda m: acquisitions.append(
        m.header.stamp.sec + m.header.stamp.nanosec*1e-9), qos_profile_sensor_data)
    with (output/'smoke.log').open('w') as log:
        job = subprocess.Popen(['bash','scripts/smoke.sh','30','0','moe',tag],cwd=ROOT,
                               env=env,stdout=log,stderr=log)
        def spin_until(predicate, timeout):
            deadline = time.monotonic()+timeout
            while not predicate() and time.monotonic()<deadline:
                rclpy.spin_once(monitor,timeout_sec=.01)
            if not predicate():
                raise RuntimeError('observer timeout')
        try:
            spin_until(lambda: sum(v>0.2 for _,v in commands)>=12, 25)
            pids = []
            for proc in Path('/proc').glob('[0-9]*'):
                try:
                    args=(proc/'cmdline').read_bytes().split(bytes([0]))
                    environ=(proc/'environ').read_bytes().split(bytes([0]))
                    if b'/workspace/scripts/local_gym_bridge.py' in args and b'ROS_DOMAIN_ID=204' in environ:
                        pids.append(int(proc.name))
                except OSError:
                    continue
            if len(pids)!=1:
                raise RuntimeError('expected one private bridge')
            start=now()
            os.kill(pids[0],signal.SIGSTOP)
            try:
                spin_until(lambda: now()-start>.25,.5)
            finally:
                os.kill(pids[0],signal.SIGCONT)
            spin_until(lambda: now()-start>.55,1)
            zero=next(t for t,v in commands if t>=start and v==0)
            stamp=max(t for t in acquisitions if t<=zero)
            assert all(v==0 for t,v in commands if t>=zero)
            assert 0.140 <= zero-stamp < .200, (zero,stamp)
            job.wait(timeout=40)
            controller=(output/'check-controller.log').read_text()
            assert any('control_fault='+r in controller for r in ('sensor_timeout','scan_timeout'))
            result={'last_scan_to_observed_zero_ms':1000*(zero-stamp),
                    'commands_after_stop':sum(t>=zero for t,_ in commands),
                    'positive_after_stop':sum(v>0 for t,v in commands if t>=zero)}
            (output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result))
        finally:
            monitor.destroy_node()
            rclpy.shutdown()
            if job.poll() is None:
                job.terminate()
                job.wait(timeout=15)


if __name__=='__main__':
    main()
