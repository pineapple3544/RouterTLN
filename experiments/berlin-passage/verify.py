"""Run the frozen Berlin layout suites using the evaluator selected by EL_EVAL_BINARY."""
import argparse
import importlib.util
import json
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
SPEED = HERE.parent / "berlin-race-speed"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite", choices=["seven", "known", "fresh"])
    parser.add_argument("label", help="New run name, for example before or after")
    args = parser.parse_args()
    if not args.label or Path(args.label).name != args.label or args.label in {".", ".."}:
        parser.error("label must be a directory name")
    os.environ.setdefault("EL_SPEED_OUTPUT", str(HERE.parents[1] / "runs/berlin-passage-replay"))
    module = importlib.util.spec_from_file_location("offline", SPEED / "offline.py")
    offline = importlib.util.module_from_spec(module)
    module.loader.exec_module(offline)
    if args.suite == "fresh":
        cases = json.loads((HERE / "random-300.json").read_text())["cases"]
    else:
        cases = json.loads((SPEED / "evaluation.json").read_text())
        if args.suite == "seven":
            names = set(json.loads((HERE / "diagnostics.json").read_text()))
            cases = [c for c in cases if c["name"] in names]
            if {c["name"] for c in cases} != names:
                raise ValueError("Missing layout in the original evaluation suite")
    cases = [dict(c, sim_race_only=True, race_brake_preview_s=.18) for c in cases]
    runs = offline.run(f"{args.suite}-{args.label}", cases, 2.7, 3.5, adaptive=True)
    failed = {
        name: (row["first_rejection"] or {}).get("reason", row["reason"])
        for name, row in runs.items() if not offline.passed(row)
    }
    summary = {
        "clean": len(runs) - len(failed),
        "total": len(runs),
        "first_lap": sum(bool(row["laps"]) for row in runs.values()),
        "contacts": sum(row["contact"] is not None for row in runs.values()),
        "failures": failed,
    }
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
