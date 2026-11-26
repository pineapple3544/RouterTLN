"""Summarize paired full-duration RCN2, RCN1 and no-RCN official trials."""
import argparse
import csv
import json
from pathlib import Path
from statistics import mean

from summarize import summarize


def one(folder):
    result = summarize(folder)
    obs = json.loads((folder / "observations.json").read_text())
    start, end = obs["first_drive_monotonic_s"], obs["end_monotonic_s"]
    if start is None:
        raise ValueError(f"No first drive in {folder}")
    with (folder / "gpu.csv").open() as stream:
        gpu = [row for row in csv.DictReader(stream) if start <= float(row["monotonic_s"]) < end]
    if not gpu:
        raise ValueError(f"No active-window GPU samples in {folder}")
    for key in ["stop", "elapsed_s", "consecutive", "best_s", "collisions", "teleports",
                "compute_over25", "max_compute_ms", "fault", "longest_measured_low_speed_s",
                "counts", "completed_lap_times"]:
        if key not in result:
            raise ValueError(f"Missing metric {key}")
    out = {key: result[key] for key in ("stop", "elapsed_s", "consecutive", "best_s",
           "collisions", "teleports", "compute_over25", "max_compute_ms", "fault",
           "longest_measured_low_speed_s", "counts", "completed_lap_times")}
    out["mean_clean_lap_s"] = (mean(result["completed_lap_times"][:result["consecutive"]])
                               if result["consecutive"] else None)
    out["scored_candidates"] = sum(int(row["candidates"]) for row in gpu)
    out["optional_candidates"] = sum(int(row["candidates"]) for row in gpu if row["optional"] == "1")
    out["kernel_s"] = sum(float(row["kernel_ms"]) for row in gpu) / 1000
    # Fixed-duration comparison for pairs where a localization fault ended one trial early.
    if end-start >= 60:
        prefix = [r for r in gpu if float(r["monotonic_s"]) < start+60]
        out["first_60s_kernel_ms_per_s"] = sum(float(r["kernel_ms"]) for r in prefix)/60
        out["first_60s_scored_candidates"] = sum(int(r["candidates"]) for r in prefix)
    out["gpu_solves"] = len(gpu)
    out["gpu_requests"] = len({row["generation"] for row in gpu})
    out["active_kernel_ms_per_s"] = out["kernel_s"]*1000/(end-start)
    out["scored_candidates_per_s"] = out["scored_candidates"]/(end-start)
    return out, [x["boxes"] for x in obs["layouts"]]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("report", type=Path)
    parser.add_argument("--confirm", type=Path, help="same-binary repeats and a separate evaluation seed")
    parser.add_argument("--repeat-2033", type=Path, help="reverse-order paired replication")
    args = parser.parse_args()
    result = {}
    for count in (1, 3, 5):
        rows, layouts = {}, {}
        for mode in ("original", "off", "wsp"):
            rows[mode], layouts[mode] = one(args.root / f"{mode}-{count}")
        equal = {mode: layouts[mode] == layouts["wsp"] for mode in ("original", "off")}
        delta = {mode: {key: 100*(1-rows["wsp"][key]/rows[mode][key])
                        for key in ("active_kernel_ms_per_s", "scored_candidates_per_s")}
                 for mode in ("original", "off")}
        result[count] = dict(trials=rows, identical_layout_sequences=equal, reductions_pct=delta)
    report = {"seed_2026":result}
    if args.confirm:
        for count, modes, seed in [(1, ("off","original"), 2026), (5, ("original","wsp"), 2033)]:
            controls = {}
            traces = {}
            for mode in modes:
                directory = args.confirm / (f"{mode}-{count}" if seed==2026 else f"{mode}-{seed}")
                controls[mode], traces[mode] = one(directory)
            if seed == 2026:
                controls["wsp"], traces["wsp"] = one(args.root / "wsp-1")
            report[f"confirmation_{seed}_{count}"] = {
                "trials":controls,
                "identical_layout_sequences":{mode: trace==traces["wsp"] for mode, trace in traces.items()},
                "reductions_pct":{mode:{key:100*(1-controls["wsp"][key]/controls[mode][key])
                    for key in ("active_kernel_ms_per_s","scored_candidates_per_s")} for mode in controls if mode!="wsp"}}
    if args.repeat_2033:
        repeated = {mode: one(args.repeat_2033/mode) for mode in ("original", "wsp")}
        a, b = (repeated[mode][1] for mode in ("original", "wsp"))
        report["repeat_2033_5"] = {"trials":{mode:data for mode,(data,_) in repeated.items()},
                                   "matching_common_layout_prefix":a[:min(len(a),len(b))] == b[:min(len(a),len(b))]}
    args.report.write_text(json.dumps(report, indent=2)+"\n")


if __name__ == "__main__":
    main()
