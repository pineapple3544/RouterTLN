"""Repeat the two ROS scenarios and the exact 100 frozen offline cases."""
import argparse
import csv
import hashlib
import json
import os
import runpy
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
BASE = HERE.parent / "berlin-forward-holdout-100"
TARGETS = ["random-9006203", "random-5557202"]


def offline(out, subset=False):
    out.mkdir()
    def worker(index):
        cfg = json.loads((BASE / "offline" / f"cases-{index}.json").read_text())
        if subset:
            cfg["cases"] = [row for row in cfg["cases"] if row["name"] in TARGETS]
        if not cfg["cases"]:
            return
        path = out / f"cases-{index}.json"
        path.write_text(json.dumps(cfg))
        with (out / f"worker-{index}.log").open("w") as stream:
            subprocess.run(["taskset", "-c", str(2*index), "bash", "scripts/evaluate_avoid.sh",
                            "models/avoid-corridor-v2.pt", str(out/f"results-{index}.jsonl"), str(path),
                            "models/race-berlin-wsp.pt"], cwd=ROOT, stdout=stream, stderr=stream, check=True)
    with ThreadPoolExecutor(max_workers=8) as pool:
        list(pool.map(worker,range(8)))
    (out / "complete").touch()


def ros(out):
    layouts = json.loads((BASE/"layouts.json").read_text())
    out.mkdir()
    def run(index):
        name = TARGETS[index]
        cores = ("0,2,4,6", "8,10,12,14")[index]
        with (out / (name+".log")).open("w") as stream:
            subprocess.run(["taskset","-c",cores,sys.executable,__file__,"one",str(out/name),name],
                           stdout=stream, stderr=stream, check=True)
    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(run,range(2)))
    (out / "complete").touch()


if __name__ == "__main__":
    mode, path = sys.argv[1:3]
    out = Path(path).resolve()
    if mode == "one":
        name=sys.argv[3]
        layouts=json.loads((BASE/"layouts.json").read_text())
        sys.argv=[sys.argv[0],str(out),"--spread-cores","--repeats","2"]
        runpy.run_path(str(HERE.parent/"berlin-brake-merge/ros.py"))["main"](
            {name:layouts[name]},domain_base=75+TARGETS.index(name)*8)
    elif mode=="ros":
        ros(out)
    elif mode in ("offline","two"):
        offline(out,mode=="two")
    else:
        raise ValueError(mode)
