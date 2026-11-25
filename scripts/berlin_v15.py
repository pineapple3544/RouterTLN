"""Run the frozen Berlin v1.5 evaluation inside the portable image."""
import argparse
import json
import os
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
import subprocess
from uuid import uuid4

CASES = Path("/opt/berlin-cases")
MODELS = Path("/workspace/models/v1.5")
VARIANTS = ("both", "local", "half", "full", "continuity", "rescue", "coupled", "graduated")


def evaluate(cases, output, stem, cpu=None):
    spec = output / (stem + ".json")
    spec.write_text(json.dumps({"device": "cpu", "teacher": False, "cases": cases}, indent=2))
    result = output / (stem + ".jsonl")
    command = ["/build/el_eval_avoid", MODELS / "avoid.pt", result, spec, MODELS / "race.pt"]
    if cpu is not None:
        command = ["taskset", "-c", str(cpu), *command]
    subprocess.run(command, check=True)
    return [json.loads(line) for line in result.read_text().splitlines()]


def matrix(output):
    for variant in VARIANTS:
        cases = json.loads((CASES / "cases.json").read_text())
        for case in cases:
            case.update(cluster_entry=variant != "local",
                        entry_candidates="both" if variant == "local" else variant,
                        phase_trace=str(output / (variant + "-" + case["name"] + ".csv")))
        evaluate(cases, output, variant + "-0")
    subprocess.run(["python3", "/workspace/scripts/compare_berlin_v15.py", output], check=True)


def replay(fixture, output):
    spec = json.loads((CASES / "replays.json").read_text())[fixture]
    case = next(c for c in json.loads((CASES / "cases.json").read_text()) if c["name"] == spec["case"])
    case["entry_candidates"] = spec["producer"]
    snapshot = str(output / "start.cbor")
    evaluate([dict(case, checkpoint={"save": snapshot, "at_m": spec["at_m"]})], output, "producer")
    branches = [dict(case, name=variant, phase_trace=str(output / (variant + ".csv")),
                     checkpoint={"restore": snapshot, "horizon_ticks": 260, "entry_branch": variant})
                for variant in VARIANTS]
    rows = evaluate(branches, output, "branches")
    if len(rows) != len(branches) or any(r["branch_entry"] != rows[0]["branch_entry"] for r in rows):
        raise RuntimeError("Replays did not begin from the same snapshot")
    for row in rows:
        print(row["case"], row["reason"], row["first_rejection"])


def offline(args, output):
    cases = json.loads((CASES / "cases.json").read_text())
    if args.case != "all":
        cases = [case for case in cases if case["name"] == args.case]
        if not cases:
            raise SystemExit(f"Unknown layout: {args.case}. Use 'cases' to list layouts.")
    for case in cases:
        case.update(cluster_entry=args.variant != "local",
                    entry_candidates="both" if args.variant == "local" else args.variant,
                    smooth_isolated_exit=args.smooth_exit)
        if args.trace:
            case["phase_trace"] = str(output / (case["name"] + ".csv"))
    rows = evaluate(cases, output, "results")
    passed = [row for row in rows if row["reason"] == "time_limit"
              and abs(row["time_s"] - 180) < 1e-5 and row["laps"]
              and row["contact"] is None and row["first_rejection"] is None
              and row["min_speed_after_first_half_m"] >= .05]
    for row in rows:
        best = min((lap["seconds"] for lap in row["laps"]), default=None)
        print(row["case"], row["reason"], f"laps={len(row['laps'])}", f"best_s={best}")
    print(f"{len(passed)}/{len(cases)} passed at 180 s")
    return len(rows) == len(passed) == len(cases)


def reliability(args, output):
    cases = json.loads((CASES / "reliability" / (args.suite + ".json")).read_text())
    if args.case:
        cases = [case for case in cases if case["name"] == args.case]
        if not cases:
            raise SystemExit(f"Unknown case in {args.suite}: {args.case}")
    cpus = [None]
    if args.cpus:
        try:
            cpus = [int(cpu) for cpu in args.cpus.split(",")]
        except ValueError:
            raise SystemExit("--cpus expects comma-separated CPU IDs")
        if len(set(cpus)) != len(cpus) or not set(cpus) <= os.sched_getaffinity(0):
            raise SystemExit("--cpus must contain distinct CPU IDs available to this container")
    (output / "traces").mkdir()
    for case in cases:
        case["phase_trace"] = str(output / "traces" / (case["name"] + ".csv"))
    workers = min(len(cpus), len(cases))

    def shard(index):
        group = cases[index::workers]
        rows = evaluate(group, output, f"rows-{index}", cpu=cpus[index])
        if [r["spec"] for r in rows] != group:
            raise RuntimeError("Evaluator results do not match the requested cases")
        return rows

    with ThreadPoolExecutor(workers) as pool:
        rows = [r for group in pool.map(shard, range(workers)) for r in group]
    passed = [r for r in rows if r["pose_source"] == "lidar_static_map" and
              r["reason"] == "time_limit" and abs(r["time_s"] - 180) < 1e-5 and
              r["laps"] and r["contact"] is None and r["first_rejection"] is None and
              r["min_speed_after_first_half_m"] >= .05]
    totals = {"suite": args.suite, "runs": len(rows), "passed": len(passed),
              "contacts": sum(r["contact"] is not None for r in rows)}
    (output / "totals.json").write_text(json.dumps(totals, indent=2))
    print(json.dumps(totals))


def ros(args, output):
    layout = json.loads((CASES / "ros-cases.json").read_text())[str(args.obstacles)]
    env = {**os.environ, **layout, "EL_MAP_DIR": "/workspace/maps/berlin",
           "EL_STOP_AFTER_LAPS": "0", "EL_RACE_DURATION_S": "180", "EL_LAP_PLANNING": "true",
           "EL_INFERENCE_DEVICE": "cpu", "EL_SPEED_CAP": "4.5",
           "EL_SIM_SECTION_SPEED_PROFILE": "true", "EL_SIM_RACE_ONLY": "true",
           "EL_SIM_AVOID_MIN": "2.7", "EL_SIM_AVOID_MAX": "4.5", "EL_STRAIGHT_BOOST": "true",
           "EL_AVOID_CORRIDOR_STEER_SEED": "true", "EL_AVOID_POLICY_FIRST": "true",
           "EL_SHARED_SPEED_BAND": "true", "EL_WHOLE_LAP_PATH": "true", "EL_AVOID_EXIT_PACE": "true",
           "EL_RACE_BRAKE_PREVIEW_S": "0.18", "EL_RACE_VERIFIED_PACE": "false",
           "EL_AVOID_CHECKPOINT": str(MODELS / "avoid.pt"), "EL_RACE_CHECKPOINT": str(MODELS / "race.pt"),
           "EL_ENTRY_CANDIDATES": "both" if args.variant == "local" else args.variant,
           "EL_CLUSTER_ENTRY": str(args.variant != "local").lower(),
           "EL_SMOOTH_ISOLATED_EXIT": str(args.smooth_exit).lower()}
    (output / "settings.json").write_text(json.dumps(
        {k: v for k, v in env.items() if k.startswith("EL_")}, indent=2))
    subprocess.run(["bash", "/workspace/scripts/smoke.sh", "186", str(args.obstacles), "moe",
                    output.name + "/drive", "", layout["EL_SCAN_NOISE_SEED"]], env=env, check=True)
    score = json.loads((output / "drive-score.json").read_text())
    print(json.dumps(score, indent=2))
    return score["accepted"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("cases", help="list the 54 frozen offline layouts")
    commands.add_parser("matrix", help="compare all eight variants on all 54 layouts")
    branch = commands.add_parser("replay", help="rebuild and replay an entry checkpoint")
    branch.add_argument("fixture", choices=("early", "late", "five"))
    off = commands.add_parser("offline", help="run the 180 s simulator evaluation")
    off.add_argument("--case", default="pace-3-023", help="a layout name or all")
    off.add_argument("--trace", action="store_true", help="save per-control telemetry")
    validation = commands.add_parser("reliability", help="reproduce the frozen 300/600 layout trials")
    validation.add_argument("suite", choices=("paired300", "fresh600"))
    validation.add_argument("--case", help="run one named layout from the chosen suite")
    validation.add_argument("--cpus", help="one worker per listed CPU ID (default: one unpinned worker)")
    drive = commands.add_parser("ros", help="run sensor, controller and independent judge")
    drive.add_argument("--obstacles", type=int, choices=(3, 5), default=3)
    for child in (off, drive):
        child.add_argument("--variant", choices=VARIANTS, default="both")
        child.add_argument("--smooth-exit", action="store_true")
    args = parser.parse_args()
    if args.command == "cases":
        for case in json.loads((CASES / "cases.json").read_text()):
            print(case["name"], case["count"], case["seed"])
        return
    parent = Path("/workspace/runs")
    parent.mkdir(exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    label = getattr(args, "variant", getattr(args, "fixture", "all"))
    output = parent / f"{args.command}-{label}-{stamp}-{uuid4().hex[:8]}"
    output.mkdir()
    print(f"Results: {output}", flush=True)
    if args.command == "reliability":
        return reliability(args, output)
    if args.command == "matrix":
        return matrix(output)
    if args.command == "replay":
        return replay(args.fixture, output)
    success = (offline if args.command == "offline" else ros)(args, output)
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
