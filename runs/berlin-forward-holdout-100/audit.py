"""Audit one cold-start trial per selected layout and compare obstacle margins."""
import csv
import json
import math
import runpy
from pathlib import Path

HERE = Path(__file__).resolve().parent


def sat_gap(x, y, yaw, box):
    bx, by, byaw, length, width = box
    def axis_gap(axis):
        center = abs((bx-x)*math.cos(axis) + (by-y)*math.sin(axis))
        car = .29*abs(math.cos(yaw-axis)) + .1016*abs(math.sin(yaw-axis))
        obstacle = length*.5*abs(math.cos(byaw-axis)) + width*.5*abs(math.sin(byaw-axis))
        return center - car - obstacle
    return max(axis_gap(a) for a in (yaw, yaw+math.pi/2, byaw, byaw+math.pi/2))


def audit_one(tag, boxes, summarize, group="ros"):
    folder = HERE / group / tag
    trial, = summarize(folder)
    with (folder / f"{tag}-1.csv").open() as stream:
        metrics = list(csv.DictReader(stream))
    times = [float(row["stamp"]) for row in metrics]
    with folder.with_suffix(".odom.csv").open() as stream:
        poses = [row for row in csv.DictReader(stream) if times[0]-.000001 <= float(row["stamp"]) <= times[-1]+.000001]
    stamps = [float(row["stamp"]) for row in poses]
    if not poses or stamps[0] - times[0] > .15 or times[-1] - stamps[-1] > .15:
        raise ValueError(f"{tag}: odometry does not cover controller trial")
    gaps = (sat_gap(float(row["x"]),float(row["y"]),float(row["yaw"]),box) for row in poses for box in boxes)
    trial["odom_gap_m"] = min(gaps)
    trial["odom_samples"] = len(poses)
    trial["max_odom_sample_interval_s"] = max(b-a for a,b in zip(stamps,stamps[1:]))
    controller = (folder / f"{tag}-1-controller.log").read_text()
    timeout = any("timeout" in row["reason"] for row in metrics) or "timeout:" in controller
    trial["sensor_timeout"] = timeout
    trial["max_scan_age_ms"] = max(float(row["scan_source_age_ms"]) for row in metrics)
    trial["max_odom_age_ms"] = max(float(row["odom_source_age_ms"]) for row in metrics)
    trial["max_compute_ms"] = max(float(row["compute_ms"]) for row in metrics)
    trial["accepted"] = all((trial["accepted"], not timeout, not trial["collision"], not trial["stopped"]))
    return trial


def main():
    selected = json.loads((HERE / "selection.json").read_text())
    with (HERE / "offline.csv").open() as stream:
        offline = {row["layout"]:row for row in csv.DictReader(stream)}
    raw = {}
    for path in (HERE / "offline").glob("results-*.jsonl"):
        for line in path.read_text().splitlines():
            row = json.loads(line)
            raw[row["case"]] = row
    summarize = runpy.run_path(str(HERE.parent / "berlin-brake-merge/analyze.py"))["summarize"]
    trials = {}
    summary = []
    for tag in selected["lowest_margin"] + selected["failures"]:
        trial = audit_one(tag, raw[tag]["boxes"], summarize)
        trials[tag] = trial
        failure = trial["failure"] or []
        summary.append(dict(layout=tag,offline_reason=offline[tag]["reason"],offline_laps=offline[tag]["laps"],
                            ros_pass=trial["accepted"],ros_reason=failure[-1] if failure else "completed",
                            ros_laps=len(trial["lap_times"]),ros_progress_m=trial["end_progress"],
                            ros_stop=trial["stopped"],ros_collision=trial["collision"],sensor_timeout=trial["sensor_timeout"],
                            offline_gap_m=offline[tag]["gap_m"],ros_odom_gap_m=trial["odom_gap_m"],
                            **{f"ros_lap{i+1}_s":t for i,t in enumerate(trial["lap_times"])}))
    with (HERE / "ros.csv").open("w",newline="") as stream:
        fields = list(summary[0])
        fields = [k for k in fields if not (k.startswith("ros_lap") and k.endswith("_s"))]
        writer = csv.DictWriter(stream, fieldnames=fields+["ros_lap1_s","ros_lap2_s","ros_lap3_s"])
        writer.writeheader()
        writer.writerows(summary)
    if (HERE / "ros-confirmation/complete").is_file():
        tag, = selected["lowest_margin"]
        repeat = audit_one(tag, raw[tag]["boxes"], summarize, "ros-confirmation")
        (HERE / "confirmation.json").write_text(json.dumps(repeat, indent=2) + "\n")
    (HERE / "ros-results.json").write_text(json.dumps(trials, indent=2) + "\n")


if __name__ == "__main__":
    main()
