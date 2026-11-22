#!/usr/bin/env bash
set -euo pipefail
base=/home/misys/forza_ws/race_stack/stack_master/maps/hyhall12
out=/workspace/runs/planner-audit
g++-11 -std=c++17 -O2 /workspace/tools/planner_audit.cpp -o /build/el_planner_audit
g++-11 -std=c++17 -O2 /workspace/tools/planner_metrics.cpp -o /build/el_planner_metrics
/build/el_planner_audit "$base" "$out"
for route in iqp sp; do
  EL_MAP_DIR="$out/$route" bash /workspace/scripts/smoke.sh 70 0 mppi "audit-$route" \
    > "$out/$route-test.log" 2>&1
  /build/el_planner_metrics "$out/$route/global_waypoints.json" "/workspace/runs/audit-$route.csv" > "$out/$route-metrics.json"
done
