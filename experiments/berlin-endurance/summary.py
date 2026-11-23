"""Summarize paired runs without treating an early failure as a fast race."""
import argparse
import csv
from itertools import accumulate
import json
from pathlib import Path
import statistics

from offline import HERE, clean, offline


def measure(row):
    failure = (row["first_rejection"] or {}).get("reason", row["reason"])
    emergency = next((event["time_s"] for event in row["events"] if event["mode"] == 3), float("inf"))
    first_fault = min(emergency, (row["contact"] or {}).get("t", float("inf")))
    laps = [lap["seconds"] for lap in row["laps"]]
    clean_laps = [lap for lap, end in zip(laps, accumulate(laps)) if end < first_fault]
    return {
        "clean": clean(row), "reason": failure, "count": row["spec"]["count"],
        "contact": row["contact"] is not None, "first": bool(clean_laps),
        "stop": row["reason"] == "stopped" or row["min_speed_after_first_half_m"] < .05,
        "laps": len(clean_laps), "best": min(clean_laps, default=None),
        "first_s": clean_laps[0] if clean_laps else None,
        "end_s": row["time_s"], "peak_command": row["peak_command_mps"],
    }


def compare(before, after):
    assert set(before) == set(after) and before
    names = sorted(before)
    paired = [k for k in names if before[k]["clean"] and after[k]["clean"]]
    def metrics(runs):
        return {
            "clean": sum(r["clean"] for r in runs.values()),
            "first": sum(r["first"] for r in runs.values()),
            "contacts": sum(r["contact"] for r in runs.values()),
            "stops": sum(r["stop"] for r in runs.values()),
            "clean_through_three": sum(r["laps"]>=3 for r in runs.values()),
            "failed_after_three": [k for k,r in runs.items() if r["laps"] >= 3 and not r["clean"]],
            "consecutive_laps": sum(r["laps"] for r in runs.values()),
            "paired_best_s": statistics.mean(runs[k]["best"] for k in paired) if paired else None,
            "failures": {k:r["reason"] for k,r in runs.items() if not r["clean"]},
        }
    deltas = [after[k]["best"] - before[k]["best"] for k in paired]
    return {
        "n": len(names), "paired": len(paired), "before": metrics(before), "after": metrics(after),
        "max_first_lap_delta_s": max((abs(before[k]["first_s"]-after[k]["first_s"])
                                      for k in names if before[k]["first"] and after[k]["first"]), default=None),
        "gained": [k for k in names if after[k]["clean"] and not before[k]["clean"]],
        "lost": [k for k in names if before[k]["clean"] and not after[k]["clean"]],
        "faster": sum(d < -1e-6 for d in deltas), "slower": sum(d > 1e-6 for d in deltas),
        "median_delta_s": statistics.median(deltas) if paired else None,
        "fewer_laps": [k for k in names if after[k]["laps"] < before[k]["laps"]],
        "more_laps": [k for k in names if after[k]["laps"] > before[k]["laps"]],
    }


def paired(folder, suite, cap):
    groups = [offline.results(folder/f"{suite}-{speed:g}") for speed in (3.5,cap)]
    before, after = groups
    assert set(before) == set(after)
    for k in before:
        same = dict(after[k]["spec"], speed_cap=3.5, straight_boost=False)
        assert same == before[k]["spec"] and same["laps"] == 0 and same["duration_s"] == 180
    return [{k:measure(r) for k,r in g.items()} for g in groups]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("cap", type=float)
    parser.add_argument("--output", type=Path, default=HERE)
    args = parser.parse_args()
    report = {}
    csvrows = []
    for suite, size in [("pilot",15),("holdout",30),("matched",300)]:
        a,b = paired(args.folder, suite, args.cap)
        assert len(a)==size
        summary = compare(a,b)
        summary["by_count"] = {i: compare({k:v for k,v in a.items() if v["count"]==i},
                                         {k:v for k,v in b.items() if v["count"]==i}) for i in (1,3,5)}
        report[suite] = summary
        for k in sorted(a):
            csvrows.append(dict(suite=suite, layout=k, count=a[k]["count"],
                                **{f"{side}_{field}":row[field] for side,row in [("before",a[k]),("after",b[k])]
                                for field in ["clean","laps","best","first_s","end_s","reason","contact","stop","peak_command"]}))
    (args.output/"results.json").write_text(json.dumps(report, indent=2)+"\n")
    with (args.output/"paired.csv").open("w",newline="") as stream:
        writer=csv.DictWriter(stream, fieldnames=csvrows[0])
        writer.writeheader()
        writer.writerows(csvrows)


if __name__ == "__main__":
    main()
