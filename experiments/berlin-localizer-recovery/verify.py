"""Replay the four scan-matching regressions or either paired three-lap evaluation."""
import argparse
import importlib.util
import json
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
PREVIOUS = HERE.parent / "berlin-race-speed"
REGRESSIONS = {"random-351979622", "random-562935709", "random-317869705", "random-570222089"}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite", choices=["regressions", "evaluation", "holdout"])
    parser.add_argument("label", help="A new output directory name, e.g. before or after")
    args = parser.parse_args()
    if not args.label or args.label in (".", "..") or Path(args.label).name != args.label:
        parser.error("label must be a directory name")
    os.environ.setdefault("EL_SPEED_OUTPUT", str(HERE.parents[1] / "runs/berlin-localizer-replay"))
    spec = importlib.util.spec_from_file_location("offline", PREVIOUS / "offline.py")
    offline = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(offline)
    path = HERE / "holdout.json" if args.suite == "holdout" else PREVIOUS / "evaluation.json"
    cases = json.loads(path.read_text())
    if args.suite == "regressions":
        cases = [case for case in cases if case["name"] in REGRESSIONS]
        assert {case["name"] for case in cases} == REGRESSIONS
    cases = [dict(case, sim_race_only=True, race_brake_preview_s=.18) for case in cases]
    results = offline.run(f"{args.suite}-{args.label}", cases, 2.7, 3.5, adaptive=True)
    failures = sorted(name for name, result in results.items() if not offline.passed(result))
    print(json.dumps({"clean":len(results)-len(failures), "total":len(results), "failed_layouts":failures}, indent=2))
    if args.suite == "regressions" and failures:
        raise SystemExit(1)

if __name__ == "__main__":
    main()
