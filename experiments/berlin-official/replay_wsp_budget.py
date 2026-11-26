"""One-request ablation on an unmodified RCN1 run with gate capture and CUDA profiling."""
import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from program_residual_gate import frozen_layers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("run", "base", "wsp", "output"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    base = args.base.read_text()
    wsp = args.wsp.read_text().split()
    if wsp[0] != "RCN2" or wsp[1:-33] != base.split()[1:]:
        raise ValueError("WSP and base control weights must match")
    gate = np.asarray(wsp[-33:], dtype=float)
    features = [json.loads(line) for line in (args.run / "features.jsonl").read_text().splitlines()]
    scores = frozen_layers(base)(np.array([r["x"] for r in features])) @ gate[:-1] + gate[-1]
    with (args.run / "gpu.csv").open() as stream:
        batches = defaultdict(list)
        for row in csv.DictReader(stream):
            batches[int(row["generation"])].append(row)
    obs = json.loads((args.run / "observations.json").read_text())
    start, end = obs["first_drive_monotonic_s"], obs["end_monotonic_s"]
    if len(features) != len(batches):
        raise ValueError("Incomplete capture")
    total_work = total_ms = omitted_work = omitted_ms = 0
    gate_requests = contributing = 0
    for i, ((generation, rows), sample, score) in enumerate(zip(batches.items(), features, scores)):
        first = float(rows[0]["monotonic_s"])
        if first + .000002 < sample["t"] or (i+1 < len(features) and first >= features[i+1]["t"]):
            raise ValueError("Request and GPU trace are not aligned")
        primary_calls = sum(row["optional"] == "0" for row in rows)
        acquired = sample["obstacle"] and (i == 0 or not features[i-1]["obstacle"]
                                          or features[i-1]["epoch"] != sample["epoch"])
        defer = (sample["valid"] and sample["x"][56] == 0 and not acquired and score < 0
                 and generation%4 != 0 and primary_calls == 1)
        in_window = [r for r in rows if start <= float(r["monotonic_s"]) < end]
        gate_requests += bool(defer and in_window)
        contributing += bool(defer and any(r["optional"] == "1" for r in in_window))
        for row in in_window:
            work, ms = int(row["candidates"]), float(row["kernel_ms"])
            total_work += work
            total_ms += ms
            if defer and row["optional"] == "1":
                omitted_work += work
                omitted_ms += ms
    report = dict(run=args.run.name, eligible_deferrals=gate_requests, deferrals_with_cuda_work=contributing,
                  scored_candidates=total_work, omitted_candidates=omitted_work,
                  measured_kernel_ms=total_ms, omitted_kernel_ms=omitted_ms,
                  omitted_candidates_pct=100*omitted_work/total_work,
                  omitted_kernel_pct=100*omitted_ms/total_ms)
    args.output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
