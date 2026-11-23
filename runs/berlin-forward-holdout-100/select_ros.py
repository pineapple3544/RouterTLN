"""Classify all drives and freeze the ROS subset before running ROS."""
import csv
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def main():
    layouts = json.loads((HERE / "layouts.json").read_text())
    waypoints = json.loads((ROOT / "maps/berlin/global_waypoints.json").read_text())["global_traj_wpnts_iqp"]["wpnts"]
    rows = []
    for path in sorted((HERE / "offline").glob("results-*.jsonl")):
        for line in path.read_text().splitlines():
            row = json.loads(line)
            tag = row["case"]
            spec = row["spec"]
            assert spec["start"] == 37 and spec["laps"] == 3 and spec["lap_planning"] and "checkpoint" not in spec
            assert spec["sim_avoid_min"] == 2.7 and spec["sim_avoid_max"] == 3.0 and row["control_ms"] == 25
            first, stride, offsets = layouts[tag]
            for i, (x, y, yaw, length, width) in enumerate(row["boxes"]):
                wp = waypoints[first + i * stride]
                assert math.hypot(x - (wp["x_m"] - math.sin(yaw)*offsets[i]),
                                  y - (wp["y_m"] + math.cos(yaw)*offsets[i])) < 0.0001
                assert abs(math.remainder(yaw - wp["psi_rad"], 2*math.pi)) < 0.0001
                assert max(abs(length-.3), abs(width-.3)) < 0.00001
            laps = row["laps"]
            clean = (row["reason"] == "completed" and len(laps) == 3 and row["contact"] is None
                     and row["first_rejection"] is None and row["min_speed_after_first_half_m"] >= .05)
            if clean:
                assert all(lap["passed"] == 3 for lap in laps)
                assert laps[0]["recorded_sections"] and laps[2]["brake_steps"] > laps[1]["brake_steps"] > laps[0]["brake_steps"]
            why = (row["first_rejection"] or {}).get("reason") or row["reason"]
            rows.append(dict(layout=tag, accepted=clean, reason=why, laps=len(laps), progress_m=row["progress_m"],
                             gap_m=row["min_sat_gap_m"], collision=row["contact"] is not None,
                             min_speed=row["min_speed_after_first_half_m"],
                             **{f"lap{i+1}_s":lap["seconds"] for i,lap in enumerate(laps)}))
    assert len(rows) == 100 and set(layouts) == {r["layout"] for r in rows}
    rows.sort(key=lambda r:r["layout"])
    with (HERE / "offline.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["layout","accepted","reason","laps","progress_m","gap_m","collision","min_speed","lap1_s","lap2_s","lap3_s"])
        writer.writeheader()
        writer.writerows(rows)
    passing = [r for r in rows if r["accepted"]]
    least = min(r["gap_m"] for r in passing)
    selection = {"failures":[r["layout"] for r in rows if not r["accepted"]],
                 "lowest_margin":[r["layout"] for r in passing if r["gap_m"] == least],
                 "lowest_margin_m":least}
    (HERE / "selection.json").write_text(json.dumps(selection, indent=2) + "\n")
    print(selection)


if __name__ == "__main__":
    main()
