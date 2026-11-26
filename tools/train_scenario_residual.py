#!/usr/bin/env python3
"""Train the bounded CPU RCN from offline MPPI-labelled records. Group split by run."""
import argparse
import json
import random
from pathlib import Path

import torch
from torch import nn

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("labels", nargs="+")
    parser.add_argument("--out", required=True)
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args()
    rows = [json.loads(line) for p in args.labels for line in Path(p).read_text().splitlines() if line.strip()]
    runs = sorted({r["run"] for r in rows})
    if len(runs) < 3:
        raise SystemExit("collect at least three distinct runs; use --scenario_run_id per run")
    random.Random(args.seed).shuffle(runs)
    holdout = set(runs[:max(1, len(runs)//5)])
    def tensor(items):
        x = torch.tensor([r["features"] for r in items], dtype=torch.float32)
        y = torch.tensor([r["target"] for r in items], dtype=torch.float32)
        if x.ndim != 2 or x.shape[1] != 64 or y.shape != (len(items), 2):
            raise ValueError("bad dataset dimensions")
        if not torch.isfinite(x).all() or not torch.isfinite(y).all():
            raise ValueError("nonfinite training data")
        return x, y
    x, y = tensor([r for r in rows if r["run"] not in holdout])
    xv, yv = tensor([r for r in rows if r["run"] in holdout])
    torch.manual_seed(args.seed)
    torch.set_num_threads(1)
    model = nn.Sequential(nn.Linear(64,48), nn.ReLU(), nn.Linear(48,32), nn.ReLU(), nn.Linear(32,2), nn.Tanh())
    # Start near zero residual rather than random corrections.
    nn.init.zeros_(model[4].weight)
    nn.init.zeros_(model[4].bias)
    opt = torch.optim.AdamW(model.parameters(), lr=1e-3)
    loss = nn.SmoothL1Loss()
    best = float("inf")
    for epoch in range(args.epochs):
        model.train()
        for idx in torch.randperm(len(x)).split(256):
            opt.zero_grad()
            value = loss(model(x[idx]), y[idx])
            value.backward()
            opt.step()
        model.eval()
        with torch.no_grad():
            val = loss(model(xv), yv).item()
        print(f"epoch={epoch+1} holdout_loss={val:.6f}")
        if val < best:
            best = val
            values = [p.detach().cpu().reshape(-1).tolist() for layer in (model[0],model[2],model[4]) for p in (layer.weight, layer.bias)]
            dest = Path(args.out)
            dest.parent.mkdir(parents=True, exist_ok=True)
            tmp = dest.with_suffix(dest.suffix + ".tmp")
            tmp.write_text("RCN1\n" + "\n".join(" ".join(f"{v:.9g}" for v in a) for a in values) + "\n")
            tmp.replace(dest)
    print("holdout runs", sorted(holdout), "loss", best)
    print("Checkpoint is not certified for driving; run closed-loop holdout and guard tests.")
if __name__ == "__main__":
    main()
