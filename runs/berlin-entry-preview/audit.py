"""Verify matching 39204 branches and the cold-start ROS campaign."""
import csv
import hashlib
import json
import runpy
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
BASE = HERE.parent / "berlin-offline-39204" / "verified"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_branches(directory, prefix):
    cases = {}
    for index in range(3):
        with (directory / f"{prefix}{index}.jsonl").open() as stream:
            for line in stream:
                row = json.loads(line)
                assert row["case"] not in cases
                cases[row["case"]] = row
    return cases


def audit():
    before = read_branches(BASE, "sweep")
    after = read_branches(HERE / "paired-v3", "result")
    assert len(before) == 75 and before.keys() == after.keys()
    for name, row in after.items():
        original = before[name]
        assert row["branch_entry"] == original["branch_entry"], name
        assert row["spec"]["checkpoint"]["perturb"] == original["spec"]["checkpoint"]["perturb"], name
        assert row["reason"] == "completed" and len(row["laps"]) == 3, name
        assert row["first_rejection"] is None and row["contact"] is None, name
        assert row["min_speed_after_first_half_m"] > 0.05, name

    summarize = runpy.run_path(str(HERE.parent / "berlin-brake-merge/analyze.py"))["summarize"]
    trials = {}
    for group in ("ros-a", "ros-b"):
        directory = HERE / group
        layouts = json.loads((directory / "layouts.json").read_text())
        results = summarize(directory)
        assert len(results) == 2 * len(layouts)
        for trial in results:
            tag = trial["tag"]
            with (directory / f"{tag}.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            trial["max_compute_ms"] = max(float(row["compute_ms"]) for row in rows)
            trial["max_scan_age_ms"] = max(float(row["scan_source_age_ms"]) for row in rows)
            trial["max_odom_age_ms"] = max(float(row["odom_source_age_ms"]) for row in rows)
            controller_log = (directory / f"{tag}-controller.log").read_text()
            trial["sensor_timeout"] = (
                any("timeout" in row["reason"].lower() for row in rows)
                or "timeout:" in controller_log.lower()
            )
            trial["accepted"] = all((
                trial["accepted"], not trial["collision"],
                not trial["stopped"], not trial["sensor_timeout"],
            ))
            trials.setdefault(tag.rsplit("-", 1)[0], []).append(trial)

    return {
        "checkpoint_sha256": digest(BASE / "approach.cbor"),
        "producer_sha256": digest(HERE / "baseline-evaluator"),
        "avoid_sha256": digest(ROOT / "models/avoid-corridor-v2.pt"),
        "controller_sha256": digest(Path("/build/el_controller")),
        "baseline_clean": sum(row["reason"] == "completed" for row in before.values()),
        "paired_clean": len(after),
        "ros": trials,
    }


if __name__ == "__main__":
    result = audit()
    (HERE / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    for layout, trials in result["ros"].items():
        print(layout, [(t["accepted"], t["lap_times"], t["failure"]) for t in trials])
