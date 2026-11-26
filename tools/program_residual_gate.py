"""Program one linear readout of a frozen RCN1 to defer optional GPU work.

Use distinct full-budget runs for fitting and testing. Inputs come from the
controller's live sensor features. Labels use future planner reliance on hypothetical or fallback paths
only to fit/evaluate offline. No scorer data or obstacle coordinates are read.
"""
import argparse
import csv
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np


def load_examples(path):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    with (path.parent / "gpu.csv").open() as stream:
        primary = Counter(int(r["generation"]) for r in csv.DictReader(stream) if r["optional"] == "0")
    if len(primary) != len(rows):
        raise ValueError("Capture and primary GPU request lengths differ")
    with (path.parent / "gpu.csv").open() as stream:
        first_times = {}
        for sample in csv.DictReader(stream):
            first_times.setdefault(int(sample["generation"]), float(sample["monotonic_s"]))
    fallback = []
    for i, (generation, stamp) in enumerate(first_times.items()):
        if stamp + .000002 < rows[i]["t"] or (i+1<len(rows) and stamp >= rows[i+1]["t"]):
            raise ValueError("Capture and primary GPU request clocks differ")
        fallback.append(primary[generation] > 1)
    needs_search = [f or not row["valid"] or row["x"][56] > 0 for row, f in zip(rows, fallback)]
    features, targets, epochs = [], [], []
    for i, row in enumerate(rows):
        acquired = row["obstacle"] and (i == 0 or not rows[i-1]["obstacle"] or rows[i-1]["epoch"] != row["epoch"])
        if acquired or needs_search[i]:
            continue
        end = i + 1
        while end < len(rows) and rows[end]["t"] - row["t"] <= .5:
            end += 1
        # Require a continuous, same-epoch observation through the horizon.
        if end == len(rows) or rows[end]["epoch"] != row["epoch"]:
            continue
        if any(b["t"] - a["t"] > .20 for a, b in zip(rows[i:end], rows[i+1:end+1])):
            continue
        features.append(row["x"])
        targets.append(any(needs_search[i+1:end]))
        epochs.append(row["epoch"])
    if not features:
        raise ValueError(f"No eligible sensor frames in {path}")
    return np.asarray(features), np.asarray(targets), np.asarray(epochs)


def frozen_layers(text):
    magic, payload = text.split(maxsplit=1)
    weights = np.fromstring(payload, sep=" ")
    if magic != "RCN1" or len(weights) != 64*48 + 48 + 48*32 + 32 + 32*2 + 2:
        raise ValueError("Expected a dense RCN1 with 64/48/32/2 dimensions")
    w1, b1, w2, b2, _, _ = np.split(weights, [3072, 3120, 4656, 4688, 4752])
    def encode(x):
        h1 = np.maximum(0, x @ w1.reshape(48, 64).T + b1)
        return np.maximum(0, h1 @ w2.reshape(32, 48).T + b2)
    return encode


def describe(score, hazard, threshold):
    mask = score < threshold
    return dict(frames=len(score), upcoming_hazards=int(hazard.sum()),
                requested_deferrals=int(mask.sum()), deferred_hazards=int(hazard[mask].sum()),
                expected_deferred_hazards_uniform=float(mask.mean()*hazard.sum()),
                coverage=float(mask.mean()))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    for arg in ["checkpoint", "fit_run", "test_run", "output", "report"]:
        ap.add_argument(arg, type=Path)
    args = ap.parse_args()
    if args.fit_run.resolve() == args.test_run.resolve():
        ap.error("Use an independent test run")
    original = args.checkpoint.read_text()
    encode = frozen_layers(original)
    x, hazard, epochs = load_examples(args.fit_run)
    hidden = encode(x)
    # Hold out the final third of the fit run by lap for threshold calibration.
    laps = np.unique(epochs)
    if len(laps) < 3:
        raise ValueError("At least three fit epochs are needed")
    fit = epochs < laps[2*len(laps)//3]
    h, y = hidden[fit], hazard[fit]
    if min(np.count_nonzero(y), np.count_nonzero(~y)) < 2:
        raise ValueError("Both target classes require at least two fit examples")
    scale = np.maximum(h.std(axis=0), .02)
    z = (h-h.mean(axis=0))/scale
    negative, positive = z[~y], z[y]
    # Regularized Fisher direction in the existing hidden basis, no optimizer.
    covariance = (np.cov(negative.T) + np.cov(positive.T)) / 2
    direction = np.linalg.solve(covariance + .25*np.eye(h.shape[1]),
                                positive.mean(axis=0) - negative.mean(axis=0))
    weights = direction/scale
    raw = hidden @ weights
    cal = ~fit
    if not hazard[cal].any():
        raise ValueError("Threshold calibration requires planner interventions")
    # At least three quarters of impending interventions should keep full optional work.
    # The runtime independently forces a full-budget refresh every fourth request.
    threshold = min(np.quantile(raw[cal], .60), np.quantile(raw[cal & hazard], .25))
    output = "RCN2" + original[len("RCN1"):].rstrip() + "\n"
    args.output.write_text(output + " ".join(f"{v:.10g}" for v in weights) + "\n" + f"{-threshold:.10g}\n")
    test_x, test_hazard, _ = load_examples(args.test_run)
    report = dict(horizon_s=.5, fit=describe(raw[fit], y, threshold),
                  calibration=describe(raw[cal], hazard[cal], threshold),
                  independent_test=describe(encode(test_x) @ weights, test_hazard, threshold),
                  fit_run=args.fit_run.parent.name, test_run=args.test_run.parent.name,
                  calibration_trace_sha256={p.parent.name:{name:hashlib.sha256((p.parent/name).read_bytes()).hexdigest()
                      for name in ("features.jsonl","gpu.csv")} for p in (args.fit_run,args.test_run)},
                  base_sha256=hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
                  wsp_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest())
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
