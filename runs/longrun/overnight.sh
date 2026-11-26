#!/usr/bin/env bash
# Unattended overnight queue (host side). Each step continues on failure; every step is logged.
#   nohup bash runs/longrun/overnight.sh > ../overnight.log 2>&1 &
set -u
REPO=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
DATE=20261125
C=(docker exec routertln-longrun bash -c)
RUN="cd /workspace && python3 runs/longrun/run.py --date $DATE"
LOCFIX="--variant locfix --set EL_CONTROLLER_BINARY=/opt/build-locfix/el_controller \
 --set EL_LOCALIZER_MEASURED_SPEED_PRIOR=true --set EL_LOCALIZATION_GRACE_SCANS=2 \
 --set EL_DEADLINE_GRACE_CYCLES=1 --set EL_FALLBACK_BUDGET_MS=15 \
 --set EL_STOP_HOLD_STEERING=true --set EL_RACE_FALLBACK=true"

step() { echo; echo "[$(date '+%F %T')] == $*"; }
summarize() { "${C[@]}" "cd /workspace && nice -n 5 python3 runs/longrun/summarize.py runs/longrun-$DATE > /dev/null" || echo "summarize failed"; }
status() { echo "- $(date '+%F %T') overnight: $*" >> "$REPO/docs/eval-prep-status.md"; }
bundle() { step "bundle"; bash "$REPO/tools/make_bundle.sh" "$(dirname "$REPO")" || echo "bundle failed"; }

step "wait for the Task 2 noise runs (baseline) to finish"
while "${C[@]}" "pgrep -f 'noise-scal[e]' > /dev/null"; do sleep 30; done
summarize; status "Task 2 noise runs (baseline) finished"

step "build patched controller (locfix) into /opt/build-locfix"
locfix_ok=0
if "${C[@]}" "bash /workspace/runs/longrun/build_locfix.sh"; then locfix_ok=1; else status "locfix build FAILED, locfix runs skipped"; fi

if (( locfix_ok )); then
  step "Mode F 10-lap matrix, locfix (30 runs)"
  "${C[@]}" "$RUN --mode F --laps 10 $LOCFIX" || echo "step failed"
  summarize; status "Mode F 10-lap locfix finished"

  step "noise 2x/3x, locfix (10 runs)"
  for k in 2 3; do "${C[@]}" "$RUN --mode F --laps 10 --obstacles 3 --seeds 51001 --noise-scale $k $LOCFIX" || echo "step failed"; done
  summarize; status "noise runs locfix finished"
fi

step "demo crash case: Mode R, layout slot A, 1 obstacle, race_manager seed 2026, noise seed 51001"
"${C[@]}" "$RUN --mode R --laps 10 --layouts A --obstacles 1 --seeds 51001 --rm-seed 2026" || echo "step failed"
(( locfix_ok )) && { "${C[@]}" "$RUN --mode R --laps 10 --layouts A --obstacles 1 --seeds 51001 --rm-seed 2026 $LOCFIX" || echo "step failed"; }
summarize

step "Mode R 10-lap matrix, baseline (30 runs)"
"${C[@]}" "$RUN --mode R --laps 10" || echo "step failed"
summarize; status "Mode R 10-lap baseline finished"

if (( locfix_ok )); then
  step "Mode R 10-lap matrix, locfix (30 runs)"
  "${C[@]}" "$RUN --mode R --laps 10 $LOCFIX" || echo "step failed"
  summarize; status "Mode R 10-lap locfix finished"
fi

bundle   # first bundle: everything decision-relevant is done

step "20-lap endurance, layouts A/C/E, baseline and locfix"
"${C[@]}" "$RUN --mode F --laps 20 --layouts A C E" || echo "step failed"
(( locfix_ok )) && { "${C[@]}" "$RUN --mode F --laps 20 --layouts A C E $LOCFIX" || echo "step failed"; }
summarize; status "20-lap A/C/E finished; queue complete"

bundle   # final bundle with the 20-lap runs
step "done"
