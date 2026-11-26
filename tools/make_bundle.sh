#!/usr/bin/env bash
# Package the eval-prep work as a zip to apply on another clone:
#   changes.patch (tracked files, against BASE), new-files/ (new files at their repo paths),
#   results/ (summaries and small per-run JSON, no raw logs), APPLY.md.
# The patch is verified against a fresh clone of BASE before zipping.
#   bash tools/make_bundle.sh [output dir]
set -euo pipefail
REPO=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
BASE=23fa88608d98198f4bc22172c279503e58efd153
STAMP=$(date +%Y%m%d-%H%M)
OUTDIR=${1:-$(dirname "$REPO")}
NAME=RouterTLN-eval-prep-$STAMP
WORK=$(mktemp -d)
B=$WORK/$NAME
mkdir -p "$B/new-files" "$B/results"
cd "$REPO"

# Tracked changes, minus scripts/build.sh (local CUDA-arch change from before this work).
EXCLUDE=(':(exclude)scripts/build.sh' ':(exclude)runs')
git diff --binary "$BASE" -- . "${EXCLUDE[@]}" > "$B/changes.patch"

NEW=(
  SUBMISSION_README.md install_final_demo.sh run_demo.sh
  config/final_demo.env config/longrun_layouts.json
  tools/longrun_layouts.py tools/make_bundle.sh
  runs/longrun/run.py runs/longrun/summarize.py runs/longrun/build_locfix.sh runs/longrun/overnight.sh
  docs/eval-obstacle-behavior.md docs/eval-prep-status.md docs/hardcoding-audit.md
  docs/localization-lost-analysis.md docs/demo-wall-contact-analysis.md docs/install-test.log
)
for f in "${NEW[@]}"; do
  [[ -f $f ]] || { echo "missing $f" >&2; exit 1; }
  mkdir -p "$B/new-files/$(dirname "$f")"; cp -p "$f" "$B/new-files/$f"
done
# Warn about untracked files that are not in the list (run outputs excluded).
git status --porcelain --untracked-files=all | awk '/^\?\?/{print $2}' | grep -v '^runs/' | while read -r f; do
  printf '%s\n' "${NEW[@]}" | grep -qx "$f" || echo "warning: untracked file not bundled: $f" >&2
done || true

# Results: summaries + small JSON per run (no CSV/log files).
for d in runs/longrun-*/; do
  [[ -d $d ]] || continue
  find "$d" \( -name 'summary.csv' -o -name 'summary.md' -o -name 'spec.json' -o -name 'spec.json.broken' \
            -o -name 'run-score.json' -o -name 'race_result_*.json' -o -name 'WHY.txt' \) -print0 |
    while IFS= read -r -d '' f; do mkdir -p "$B/results/$(dirname "$f")"; cp -p "$f" "$B/results/$f"; done
done
[[ -f $REPO/../overnight.log ]] && cp -p "$REPO/../overnight.log" "$B/results/overnight.log"

# Verify: fresh clone of BASE + patch + new files reproduces the working tree for every bundled path.
CHECK=$WORK/check
git clone -q --no-checkout "$REPO" "$CHECK"
git -C "$CHECK" checkout -q "$BASE"
git -C "$CHECK" apply --check "$B/changes.patch"
git -C "$CHECK" apply "$B/changes.patch"
cp -rp "$B/new-files/." "$CHECK/"
bad=0
for f in $(git diff --name-only "$BASE" -- . "${EXCLUDE[@]}") "${NEW[@]}"; do
  cmp -s "$REPO/$f" "$CHECK/$f" || { echo "verify: $f differs" >&2; bad=1; }
done
(( bad == 0 )) || { echo "bundle verification failed" >&2; exit 1; }
echo "verified: patch applies to $BASE and reproduces all bundled files"

cat > "$B/APPLY.md" <<EOF
# Applying the eval-prep work (bundle $STAMP)

Base commit: \`$BASE\` (\`23fa886\`). The patch was verified against a fresh clone of it.

\`\`\`bash
cd RouterTLN                                  # your clone, at 23fa886 (or later, if the patch still applies)
git checkout -b Juhwan_Kim-eval-prep
git apply --check /path/to/$NAME/changes.patch && git apply /path/to/$NAME/changes.patch
cp -r /path/to/$NAME/new-files/. .
\`\`\`

\`runs/\` is in \`.gitignore\`, so \`runs/longrun/*\` needs \`git add -f\`.

## Suggested commits (one topic each)

1. **CPU-only build, installer and demo**
   \`git add CMakeLists.txt install_final_demo.sh run_demo.sh config/final_demo.env SUBMISSION_README.md docs/install-test.log\`
2. **Long-run test matrix and noise injection** (test tooling, no driving change)
   \`git add scripts/smoke.sh scripts/local_gym_bridge.py tools/longrun_layouts.py config/longrun_layouts.json tools/make_bundle.sh && git add -f runs/longrun/\`
3. **Controller robustness options, all default off** (localizer rejection logging, measured-speed prior, localization/deadline grace, fallback budget, stop steering hold, Race fallback)
   \`git add src/node.cpp include/el/scan_localizer.hpp include/el/section_router.hpp include/el/map_paths.hpp\`
4. **Evaluation docs**
   \`git add README.md docs/eval-obstacle-behavior.md docs/eval-prep-status.md docs/hardcoding-audit.md docs/localization-lost-analysis.md docs/demo-wall-contact-analysis.md\`

Optionally commit the result summaries: \`results/runs/longrun-*/summary.{csv,md}\` (copy to the same paths, then \`git add -f\`).

## Not included

- \`scripts/build.sh\`: the CUDA-architecture change (120 → 75) on the test VM predates this work and was left out.
- Raw run logs (controller/physics CSVs, logs): only summaries and per-run JSON are under \`results/\`.

Before submission, set \`ROUTERTLN_REF\` in \`install_final_demo.sh\` to the final commit or tag.
EOF

mkdir -p "$OUTDIR"
(cd "$WORK" && zip -qr "$OUTDIR/$NAME.zip" "$NAME")
rm -rf "$WORK"
echo "$OUTDIR/$NAME.zip"
