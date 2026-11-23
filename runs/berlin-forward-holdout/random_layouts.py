"""Generate four seeded three-obstacle layouts and run the standard Berlin ROS trial."""
from pathlib import Path
import random
import runpy

ROOT = Path(__file__).resolve().parents[2]
SEEDS = (39101, 39102, 39103, 39104)


def layouts(seeds=SEEDS):
    catalog = {}
    for seed in seeds:
        rng = random.Random(seed)
        # 489 waypoints; keep the start at 37 clear and avoid index wraparound.
        stride = rng.randint(18, 140)
        first = rng.randint(75, 465 - 2 * stride)
        offsets = [round(rng.uniform(-0.12, 0.12), 4) for _ in range(3)]
        catalog[f"random-{seed}"] = (first, stride, offsets)
    return catalog


if __name__ == "__main__":
    runpy.run_path(str(ROOT / "runs/berlin-brake-merge/ros.py"))["main"](layouts())
