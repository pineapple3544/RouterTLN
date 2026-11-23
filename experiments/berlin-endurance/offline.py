"""Run matched Berlin endurance trials in simulated time."""
import argparse
import importlib.util
import json
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
spec = importlib.util.spec_from_file_location("offline", HERE.parent / "berlin-race-speed/offline.py")
offline = importlib.util.module_from_spec(spec)
spec.loader.exec_module(offline)


def clean(row):
    return (row["reason"] == "time_limit" and abs(row["time_s"] - 180) < 1e-5
            and row["pose_source"] == "lidar_static_map" and bool(row["laps"])
            and row["contact"] is None and row["first_rejection"] is None
            and row["min_speed_after_first_half_m"] >= .05)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite", choices=["pilot", "matched", "holdout"])
    parser.add_argument("cap", type=float, choices=[3.5, 3.65, 3.75])
    args = parser.parse_args()
    if args.suite == "matched":
        cases = json.loads((HERE.parent / "berlin-passage/random-300.json").read_text())["cases"]
    else:
        cases = json.loads((HERE / "layouts.json").read_text())[args.suite]
    cases = [dict(case, laps=0, duration_s=180, full_lap=True, ros_finish=True,
                  lap_planning=True, sim_race_only=True, straight_boost=args.cap > 3.5,
                  race_brake_preview_s=.18) for case in cases]
    os.environ.setdefault("EL_SPEED_OUTPUT", str(ROOT / "runs/berlin-endurance"))
    rows = offline.run(f"{args.suite}-{args.cap:g}", cases, 2.7, args.cap, adaptive=True)
    print(f"{sum(map(clean, rows.values()))}/{len(rows)} clean for 180 seconds", flush=True)


if __name__ == "__main__":
    main()
