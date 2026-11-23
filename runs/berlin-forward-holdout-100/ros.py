"""Run one cold-start ROS trial per selected layout on two isolated CPU groups."""
import json
import os
import runpy
import signal
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def run_one(tag, domain, folder="ros"):
    output = HERE / folder / tag
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
    probe = subprocess.Popen(["bash", "-c", 'source /opt/ros/humble/setup.bash && exec /usr/bin/python3 "$@"',
                              "odom", str(HERE / "odom.py"), str(output.with_suffix(".odom.csv"))], env=env)
    try:
        layouts = json.loads((HERE / "layouts.json").read_text())
        sys.argv = [sys.argv[0], str(output), "--spread-cores", "--repeats", "1"]
        runpy.run_path(str(ROOT / "runs/berlin-brake-merge/ros.py"))["main"]({tag:layouts[tag]}, domain_base=domain)
    finally:
        if probe.poll() is None:
            probe.send_signal(signal.SIGINT)
        probe.wait(timeout=15)


def main():
    selected = json.loads((HERE / "selection.json").read_text())
    targets = selected["lowest_margin"] + selected["failures"]
    folder = HERE / "ros"
    folder.mkdir()
    def group(index):
        cores = ("0,2,4,6","8,10,12,14")[index]
        for i in range(index, len(targets), 2):
            tag = targets[i]
            print("START", tag, flush=True)
            with (folder / f"{tag}.log").open("w") as stream:
                subprocess.run(["taskset","-c",cores,sys.executable,__file__,tag,str(115+i)],
                               cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, check=True)
            print("END", tag, flush=True)
    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(group, (0,1)))
    (folder / "complete").write_text(str(len(targets))+"\n")


if __name__ == "__main__":
    if len(sys.argv) in (3, 4):
        run_one(sys.argv[1], int(sys.argv[2]), sys.argv[3] if len(sys.argv) == 4 else "ros")
    else:
        main()
