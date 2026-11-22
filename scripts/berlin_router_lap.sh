#!/usr/bin/env bash
# Run inside el-native-dev. The final finish issues a planned stop.
set -euo pipefail
cd /workspace
tag=${1:-berlin-lap-$(date +%Y%m%d-%H%M%S)}
if [[ -e "runs/$tag.csv" ]]; then echo "Run already exists: $tag" >&2; exit 1; fi
laps=${EL_LAPS:-1}
[[ $laps =~ ^([1-9]|1[0-9]|20)$ ]] || { echo "EL_LAPS must be 1..20" >&2; exit 2; }
export EL_MAP_DIR=/workspace/maps/berlin EL_START_INDEX=37 EL_STOP_AFTER_LAPS=$laps
export EL_OBSTACLE_INDEX=94 EL_OBSTACLE_STRIDE=30 EL_OBSTACLE_OFFSETS='[0.083,-0.083,0.083]'
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
# Bound simulation worker pools on hosts with many available logical CPUs.
cpus=$(python3 -c 'import os; a=sorted(os.sched_getaffinity(0)); print(",".join(map(str,(a[2:6] if len(a)>=6 else a[:4]))))')
taskset -c "$cpus" bash scripts/smoke.sh "$((40*laps+10))" 3 moe "$tag"
python3 - "$tag" "$laps" <<'PY'
import csv
import json
import sys

with open("runs/" + sys.argv[1] + ".csv") as f:
    rows = list(csv.DictReader(f))
with open("maps/berlin/global_waypoints.json") as f:
    length = json.load(f)["global_traj_wpnts_iqp"]["wpnts"][-1]["s_m"]
if len(rows) < 2:
    raise SystemExit("Insufficient ROS samples")
for r in rows:
    if r["collision"] != "0" or r["mode"] == "3":
        raise SystemExit(f"Failed at {r['progress_m']} m ({r['reason']}, collision={r['collision']})")
laps = int(sys.argv[2])
if float(rows[-1]["progress_m"]) < laps*length - 0.01:
    raise SystemExit("Lap incomplete")
if any(r["valid"] != "1" or float(r["target_speed"]) <= 0 or
       (float(r["progress_m"]) > 0.5 and float(r["speed"]) < 0.05) for r in rows[:-1]):
    raise SystemExit("Invalid command or stop before finish")
last_crossing = float(rows[0]["stamp"])
for lap in range(laps):
    segment = [r for r in rows if lap*length <= float(r["progress_m"]) < (lap+1)*length]
    if not segment or not any(r["mode"] == "1" for r in segment) or segment[-1]["mode"] != "0":
        raise SystemExit(f"Missing Avoid traversal or Race return on lap {lap+1}")
    end = next((r for r in rows if float(r["progress_m"]) >= (lap+1)*length), rows[-1])
    crossing = float(end["stamp"])
    print(f"Berlin ROS lap {lap+1}: {crossing-last_crossing:.3f} s")
    last_crossing = crossing
PY
