"""Paired ROS trials over the same 180-second scoring window."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cap", type=float, choices=[3.65, 3.75])
    parser.add_argument("--candidate-only", action="store_true",
                        help="Add three runs to an existing paired reference evaluation")
    args = parser.parse_args()
    layouts = json.loads((HERE.parent / "berlin-race-speed/ros-layouts.json").read_text())
    folder = Path(os.environ.get("EL_SPEED_OUTPUT", ROOT / "runs/berlin-endurance")) / "ros"
    folder.mkdir(parents=True, exist_ok=args.candidate_only)
    if args.candidate_only:
        for case in layouts:
            if not (folder / f"obs{case['count']}-3.5-verdict.json").is_file():
                raise ValueError("candidate-only evaluation requires the matched reference results")
    affinities = [("8,9", "10,11"), ("12,13", "14,15")]
    if args.candidate_only:
        affinities.append(("0,1", "2,3"))

    def lane(index):
        sim, control = affinities[index]
        for i, case in enumerate(layouts):
            if args.candidate_only and i % len(affinities) != index:
                continue
            cap = args.cap if args.candidate_only else [3.5, args.cap][(i+index) % 2]
            tag = folder / f"obs{case['count']}-{cap:g}"
            if Path(str(tag)+"-run.log").exists():
                raise FileExistsError(tag)
            env = dict(os.environ, ROS_DOMAIN_ID=str(143+index), EL_SIM_CPUSET=sim,
                       EL_CONTROL_CPUSET=control, EL_MAP_DIR=str(ROOT/"maps/berlin"),
                       EL_START_INDEX=str(case["start"]), EL_OBSTACLE_INDEX=str(case["index"]),
                       EL_OBSTACLE_STRIDE=str(case["stride"]), EL_OBSTACLE_OFFSETS=json.dumps(case["offsets"]),
                       EL_SCAN_NOISE_SEED=str(case["seed"]), EL_STOP_AFTER_LAPS="0",
                       EL_RACE_DURATION_S="180", EL_LAP_PLANNING="true", EL_INFERENCE_DEVICE="cpu",
                       EL_SPEED_CAP=str(cap), EL_STRAIGHT_BOOST=str(cap > 3.5).lower(),
                       EL_SIM_SECTION_SPEED_PROFILE="true", EL_SIM_RACE_ONLY="true",
                       EL_SIM_AVOID_MIN="2.7", EL_SIM_AVOID_MAX="3.0", EL_RACE_BRAKE_PREVIEW_S="0.18",
                       EL_AVOID_CHECKPOINT=str(ROOT/"models/avoid-corridor-v2.pt"),
                       EL_RACE_CHECKPOINT=str(ROOT/"models/race-berlin-wsp.pt"))
            env.pop("EL_UNIFIED_CHECKPOINT", None)
            with Path(str(tag)+"-run.log").open("w") as log:
                run = subprocess.run(["taskset","-c",sim+","+control,"bash","scripts/smoke.sh",
                                      "186",str(case["count"]),"moe",str(tag.relative_to(ROOT/"runs")),
                                      "",str(case["seed"])],env=env,cwd=ROOT,stdout=log,stderr=log)
            result = json.loads(Path(str(tag)+"-score.json").read_text())
            result.update(layout=case["name"], count=case["count"], cap=cap, runner_exit=run.returncode)
            Path(str(tag)+"-verdict.json").write_text(json.dumps(result,indent=2)+"\n")
            print(tag.name,result["accepted"],result.get("consecutive_laps"),flush=True)
    with ThreadPoolExecutor(max_workers=len(affinities)) as pool:
        list(pool.map(lane, range(len(affinities))))


if __name__ == "__main__":
    main()
