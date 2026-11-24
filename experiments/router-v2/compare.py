"""Paired v1/v2 and WSP/preview ablation using frozen full-lap scenarios."""
import argparse
import csv
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LEGACY_FLAGS = ("sim_avoid_min", "sim_avoid_max", "sim_section_speed_profile",
                "sim_race_only", "straight_boost", "race_sight_speed",
                "race_lap_speed", "race_exit_speed", "race_brake_preview_s",
                "scout_then_cruise")

def accepted(row):
    spec = row["spec"]
    duration = spec.get("duration_s", 0)
    done = (row["reason"] == "time_limit" and abs(row["time_s"] - duration) < 1e-5
            and bool(row["laps"])) if spec["laps"] == 0 else (
                row["reason"] == "completed" and len(row["laps"]) == spec["laps"])
    return (done and row["pose_source"] == "lidar_static_map" and row["contact"] is None
            and row["first_rejection"] is None and row["min_speed_after_first_half_m"] >= .05)

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("cases", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--binary", default="/build/el_eval_avoid")
    p.add_argument("--avoid-scale", type=float, default=1.)
    p.add_argument("--approach-speed", type=float, default=3.)
    p.add_argument("--cap", type=float, default=None)
    p.add_argument("--device", choices=["cpu", "cuda"], default="cpu")
    p.add_argument("--arms", nargs="+", choices=["v1-original", "v1-wsp", "v2-original-always",
                   "v2-wsp-always", "v2-original-first", "v2-wsp-first"],
                   default=["v1-original", "v1-wsp", "v2-original-always", "v2-wsp-always",
                            "v2-original-first", "v2-wsp-first"])
    args = p.parse_args()
    batch = json.loads(args.cases.read_text())
    cases = batch["cases"]
    if not cases or len({c["name"] for c in cases}) != len(cases):
        p.error("unique nonempty scenarios required")
    if any(c.get("map") != "berlin" or c.get("layout", 0) % 2 or not c.get("full_lap")
           or not c.get("lap_planning") or not c.get("ros_finish")
           or c.get("count") not in (1, 3, 5) or c.get("checkpoint")
           or "laps" not in c or c.get("original") for c in cases):
        p.error("use frozen Berlin forward sensor-only full-lap cases with independent finish")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    summary = []
    for arm in args.arms:
        config = {"teacher": False, "device": args.device, "cases": []}
        for source in cases:
            case = dict(source)
            if args.cap is not None:
                case["speed_cap"] = args.cap
            if arm.startswith("v2"):
                for flag in LEGACY_FLAGS:
                    case.pop(flag, None)
                case.update(hierarchical_v2=True, v2_policy_first=arm.endswith("first"),
                            v2_avoid_speed_scale=args.avoid_scale,
                            v2_approach_speed=args.approach_speed)
            else:
                case["hierarchical_v2"] = False
            config["cases"].append(case)
        conf = args.output / (arm + ".json")
        result = args.output / (arm + ".jsonl")
        conf.write_text(json.dumps(config, indent=2))
        model = "avoid-corridor-v2-wsp-highspeed-lead.pt" if "wsp" in arm else "avoid-corridor-v2.pt"
        with (args.output / (arm + ".log")).open("w") as log:
            subprocess.run([args.binary, "models/" + model, str(result), str(conf),
                            "models/race-berlin-wsp.pt"], cwd=ROOT, stdout=log,
                           stderr=subprocess.STDOUT, check=True)
        rows = [json.loads(line) for line in result.read_text().splitlines()]
        if len(rows) != len(cases) or {r["case"] for r in rows} != {c["name"] for c in cases}:
            raise RuntimeError("incomplete paired results: " + arm)
        for row in rows:
            stats = row["authority"]
            laps = [lap["seconds"] for lap in row["laps"]]
            summary.append(dict(arm=arm, case=row["case"], count=row["spec"]["count"],
                accepted=accepted(row), reason=row["reason"], laps=len(laps),
                best_lap=min(laps, default=None), contact=row["contact"] is not None,
                rejection=row["first_rejection"] is not None, **{
                    k:v for k,v in stats.items() if k != "phase_steps"}))
        print(arm, sum(accepted(r) for r in rows), "/", len(cases), flush=True)
    with (args.output / "paired.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=summary[0].keys())
        writer.writeheader()
        writer.writerows(summary)

if __name__ == "__main__":
    main()
