"""Run fixed, previously unseen Berlin layouts in the native offline evaluator."""
import hashlib
import json
import os
import random
import runpy
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SEEDS = random.Random(20261123).sample(range(1000000, 10000000), 100)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def main():
    output = HERE / "offline"
    output.mkdir()
    catalog = runpy.run_path(str(ROOT / "runs/berlin-forward-holdout/random_layouts.py"))["layouts"](SEEDS)
    write_json(HERE / "layouts.json", catalog)
    waypoints = json.loads((ROOT / "maps/berlin/global_waypoints.json").read_text())["global_traj_wpnts_iqp"]["wpnts"]
    template = json.loads((ROOT / "runs/berlin-offline-39204/layout.json").read_text())
    cases = []
    for name, (first, stride, offsets) in catalog.items():
        stations = [waypoints[first + i * stride]["s_m"] - waypoints[37]["s_m"] for i in range(3)]
        cases.append(dict(template, name=name, stations=stations, offsets=offsets))
    files = ["/build/el_controller", "/build/el_eval_avoid", "include/el/avoid_corridor.hpp",
             "models/avoid-corridor-v2.pt", "models/race-berlin-wsp.pt", "maps/berlin/router.yaml"]
    write_json(HERE / "hashes.json", {f: hashlib.sha256((ROOT / f).read_bytes()).hexdigest() for f in files})
    cores = sorted(os.sched_getaffinity(0))[::2]
    def worker(pair):
        shard, core = pair
        cfg = output / f"cases-{shard}.json"
        write_json(cfg, {"device": "cpu", "teacher": False, "cases": cases[shard::len(cores)]})
        with (output / f"worker-{shard}.log").open("w") as log:
            subprocess.run(["taskset", "-c", str(core), "bash", "scripts/evaluate_avoid.sh",
                            "models/avoid-corridor-v2.pt", str(output / f"results-{shard}.jsonl"),
                            str(cfg), "models/race-berlin-wsp.pt"], cwd=ROOT, check=True,
                           env=dict(os.environ, OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1"),
                           stdout=log, stderr=subprocess.STDOUT)
    with ThreadPoolExecutor(max_workers=len(cores)) as pool:
        list(pool.map(worker, enumerate(cores)))
    (output / "complete").write_text("100\n")


if __name__ == "__main__":
    main()
