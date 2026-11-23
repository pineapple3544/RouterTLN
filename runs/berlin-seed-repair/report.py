import json
from pathlib import Path

root=Path(__file__).resolve().parent
results=json.loads((root/"results.json").read_text())
cases=results["cases"]
trials=[trial for case in cases.values() for trial in case["trials"]]
lines=["# Berlin 정방향 회피 검증", "",
       f"최종 판정은 총 {len(trials)}회 중 {sum(t['accepted'] for t in trials)}회 통과, "
       f"주행 중 정지 {sum(t['stopped'] for t in trials)}회, 충돌 {sum(t['collision'] for t in trials)}회, "
       f"센서 타임아웃 {sum(t['timeout'] for t in trials)}회였다.",
       "", "## 재현된 실패", "",
       "39103은 2랩 누적 70.276 m에서 경로 탐색이 성공했지만 제동 궤적이 세 번째 장애물의 관측 셀을 밟았다. "
       "제어기에서 거부한 조향과 최초 접촉 예측 자세를 따로 계측했다. 셀 중심은 시뮬레이터 장애물의 실제 경계에서 1.9 cm 떨어져 있었다. "
       "첫 랩 통과 때와 횡위치는 4 mm 이내였고 차체 방향은 약 9° 달랐다. "
       "[점유 격자와 주행 궤적](snapshots/39103/comparison.png)",
       "", "39104는 기존 실패와 같은 첫 장애물 접근 구간에서 다시 멈췄다. 실패 프레임의 전방 두 번째 탐색 단계에서 연결 가능한 후보가 0개였다. "
       "같은 진행 거리의 완주 기록은 반대쪽으로 접근했다. 동일한 점유 격자와 자세에 더 촘촘한 후보를 적용하자 경로가 끝까지 연결됐다. "
       "[점유 격자와 주행 궤적](snapshots/39104/comparison.png)",
       "", "## 제어기 변경", "",
       "원본 Avoid가 낸 조향으로 진행했을 때와 대안 조향을 사용했을 때의 차체 위치와 제동 궤적을 비교한다. "
       "첫 랩부터 여유가 있는 통로를 우선하며, 10 cm 간격 후보가 끊기면 횡방향을 2.5 cm 간격으로 다시 탐색한다. "
       "경로 복귀 중에도 Race에 제어를 넘길 때까지 Avoid의 경로와 조향을 계속 검사한다. "
       "가중치·최종 제동 안전 검사·150 ms 센서 보호·25 ms 제어 기한은 유지했다.",
       "", "## 주행 기록", "", "1랩에서 계획을 새로 만들고 다음 두 랩에 사용했다. 각 배치는 두 번 주행했다. 첫 회는 상세 계측을 켰고 두 번째 회는 껐다. "
       "39201–39204는 시작 전에 정한 시드다. 39301–39302는 최종 제어기를 고정한 후 추가했다. 각 그룹의 ROS 도메인과 물리 코어를 분리했다.",
       "", "| 시드 | 회차 | 1랩 (초) | 2랩 (초) | 3랩 (초) | 결과 |",
       "|---|---:|---:|---:|---:|---|"]
for seed,case in cases.items():
    for trial in case["trials"]:
        failure=trial["failure"]
        times=trial["lap_times"][:failure[0]-1] if failure else trial["lap_times"]
        values=[f"{s:.2f}" for s in times]+["—"]*(3-len(times))
        why=f"{failure[0]}랩 누적 {failure[1]:.3f} m, {failure[2]}" if failure else ("통과" if trial["accepted"] else "판정 기준 미충족")
        lines.append("| "+" | ".join([seed.split("-")[-1],trial["tag"][-1],*values,why])+" |")
lines += ["", "## 재현 명령", "", "ROS Humble 개발 컨테이너의 /workspace에서 실행한다. 출력 폴더는 새 이름을 사용한다.",
          "", "```sh", "source /opt/ros/humble/setup.bash", "cmake --build /build -j2",
          "ctest --test-dir /build --output-on-failure",
          "EL_TRACE_FIRST_REPEAT=1 python3 runs/berlin-seed-repair/run.py runs/berlin-seeds-recheck --spread-cores",
          "```", "", "[최종 주행별 수치](results.json) · [실패 프레임 분석](diagnosis.json) · [회귀 테스트](refined-build.log)"]
(root/"RESULTS.md").write_text("\n".join(lines)+"\n")
