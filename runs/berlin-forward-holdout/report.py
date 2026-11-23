"""Audit Berlin holdout trials, and write per-trial data and lap comparisons."""
import csv
import hashlib
import json
from pathlib import Path
import runpy
from statistics import mean

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
summarize = runpy.run_path(str(ROOT / "runs/berlin-brake-merge/analyze.py"))["summarize"]


def document():
    layouts = {}
    conditions = {
        "sensor_limit_ms": 150, "planned_avoid_target_band_mps": [2.7, 3.0],
        "start_index": 37, "map": "Berlin forward", "obstacles": 3,
        "race_model": "race-berlin-wsp.pt", "avoid_model": "avoid-corridor-v2.pt",
        "avoid_sha256": hashlib.sha256((ROOT / "models/avoid-corridor-v2.pt").read_bytes()).hexdigest(),
        "random_seeds": [39101, 39102, 39103, 39104],
        "affinity": [4, 6, 8, 10], "repeat_count": 2,
    }
    for group in ("fixed", "random"):
        directory = HERE / group
        spec = json.loads((directory / "layouts.json").read_text())
        trials = summarize(directory)
        assert len(trials) == 2 * len(spec)
        for name, placement in spec.items():
            subset = [r for r in trials if r["tag"].rsplit("-", 1)[0] == name]
            assert len(subset) == 2
            for trial in subset:
                tag = trial["tag"]
                with (directory / (tag + ".csv")).open() as f:
                    rows = list(csv.DictReader(f))
                log = (directory / (tag + "-controller.log")).read_text()
                trial["sensor_timeout"] = "sensor_timeout" in log or any(r["reason"] == "sensor_timeout" for r in rows)
                trial["max_logged_scan_age_ms"] = max(float(r["scan_source_age_ms"]) for r in rows)
                trial["plan_empty_at_start"] = all(int(rows[0][field]) == 0 for field in ("plan_ready", "planned_entries", "plan_brake_steps"))
                assert trial["plan_empty_at_start"] and "TLN inference device: cpu" in log
                first_stop = next((row for row in rows if row["mode"] == "3"), None)
                if first_stop:
                    trial["first_stop"] = {field: float(first_stop[field]) for field in (
                        "speed", "progress_m", "scan_source_age_ms", "odom_source_age_ms", "compute_ms"
                    )}
                if trial["plan_ready_m"] is not None:
                    assert 48.62 <= trial["plan_ready_m"] <= 49.0, tag
                if trial["command_range"]:
                    assert min(trial["command_range"]) >= 2.7 - 1e-5
            # Count only laps completed cleanly, and expose the sample count with each mean.
            clean = [r["lap_times"][:r["failure"][0]-1] if r["failure"] else r["lap_times"] for r in subset]
            per_lap = [[times[i] for times in clean if len(times) > i] for i in range(3)]
            layouts[name] = dict(placement=placement, trials=subset,
                                 accepted=sum(r["accepted"] and not r["sensor_timeout"] for r in subset),
                                 lap_counts=[len(x) for x in per_lap],
                                 mean_lap_s=[round(mean(x), 3) if x else None for x in per_lap])
    return dict(conditions=conditions, layouts=layouts)


if __name__ == "__main__":
    report = document()
    (HERE / "results.json").write_text(json.dumps(report, indent=2) + "\n")
    lines = ["# Berlin 정방향 추가 배치 검증", "",
             "각 배치에서 제어기와 시뮬레이터를 새로 시작해 2회씩 주행했다. "
             "첫 랩은 계획 수집, 2·3랩은 그 주행에서 만든 계획 적용 구간이다. "
             "목표 속도 하한은 계획 적용 랩의 Avoid에 대해 2.7 m/s, 센서 보호 기준은 150 ms다.",
             "", "| 배치 | 회차 | 1랩 (초) | 2랩 (초) | 3랩 (초) | 결과 |",
             "|---|---:|---:|---:|---:|---|"]
    for name, data in report["layouts"].items():
        for trial in data["trials"]:
            failure = trial["failure"]
            times = trial["lap_times"][:failure[0]-1] if failure else trial["lap_times"]
            values = [f"{v:.2f}" for v in times] + ["—"] * (3 - len(times))
            status = f"{failure[0]}랩 {failure[1]:.2f} m (누적), {failure[2]}" if failure else ("통과" if trial["accepted"] else "평가 기준 미충족")
            lines.append("| " + " | ".join([name, trial["tag"][-1], *values, status]) + " |")
    lines += ["", "각 시드의 배치 좌표 입력값은 [랜덤 배치 명세](random/layouts.json)에, "
              "주행별 충돌·정지 판정과 누적 거리, 계획 생성 지점, 목표·실측 속도는 [결과 JSON](results.json)에 있다.",
              "", "## 재현", "", "개발 컨테이너의 /workspace에서 새 출력 디렉터리를 지정한다.", "",
              "```sh",
              "python3 runs/berlin-brake-merge/ros.py runs/fixed-recheck --layouts mirror tight same-side mid-bends --spread-cores",
              "python3 runs/berlin-forward-holdout/random_layouts.py runs/random-recheck --spread-cores",
              "```"]
    (HERE / "RESULTS.md").write_text("\n".join(lines) + "\n")
