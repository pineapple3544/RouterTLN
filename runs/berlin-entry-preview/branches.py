"""Run the same 39204 branch entries against a new planner build."""
import argparse
import csv
import json
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
BASE = HERE.parent / "berlin-offline-39204" / "verified"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path, help="New directory for results")
    parser.add_argument("--single", action="store_true", help="Only run the unperturbed entry")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True)

    def run(index):
        config = json.loads((BASE / f"sweep{index}.json").read_text())
        if args.single:
            config["cases"] = [case for case in config["cases"]
                               if not any(case["checkpoint"]["perturb"].values())]
            assert len(config["cases"]) == 1
        for case in config["cases"]:
            checkpoint = case["checkpoint"]
            checkpoint.update(restore=str(BASE / "approach.cbor"),
                              producer_executable=str(HERE / "baseline-evaluator"))
            if "ticks" in checkpoint:
                checkpoint["ticks"] = str(output / "unchanged.jsonl")
        spec = output / f"cases{index}.json"
        spec.write_text(json.dumps(config))
        result = output / f"result{index}.jsonl"
        with (output / f"run{index}.log").open("w") as log:
            subprocess.run(["bash", "scripts/evaluate_avoid.sh", "models/avoid-corridor-v2.pt",
                            str(result), str(spec), "models/race-berlin-wsp.pt"],
                           cwd=ROOT, stdout=log, stderr=log, check=True)
        with result.open() as stream:
            return [json.loads(line) for line in stream]

    with ThreadPoolExecutor(max_workers=3) as pool:
        groups = pool.map(run, range(1 if args.single else 3))
        rows = [row for group in groups for row in group]
    with (output / "outcomes.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["case", "reason", "laps", "first_rejection", "progress", "min_speed", "collision"])
        for row in rows:
            entry = row["branch_entry"]
            assert entry["step"] == 2198 and entry["plan_ready"]
            rejection = row["first_rejection"] or {}
            writer.writerow([row["case"], row["reason"], len(row["laps"]), rejection.get("reason"),
                             rejection.get("progress_m"), row["min_speed_after_first_half_m"],
                             row["contact"] is not None])


if __name__ == "__main__":
    main()
