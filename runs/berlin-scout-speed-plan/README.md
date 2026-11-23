# 첫 랩 속도 계획의 오프라인 재생

[실험 조건과 판정](../../experiments/berlin-endurance/scout-speed-plan.md)에 여섯 배치의 완주·정지와 ROS 기록을 정리했다. 여기에는 같은 배치를 생성한 입력과 실패 전 상태의 조향 비교 스크립트가 있다. 평가에는 원본 Avoid와 WSP Race 가중치를 사용한다.

로컬 시뮬레이터 컨테이너의 `/workspace`에서 아래 명령을 실행한다. `sweep.py`는 출력 디렉터리를 새로 만들며, 이 캠페인의 전체 실행은 논리 CPU 0–11을 사용한다. 체크포인트 파일을 보존하려면 분기 비교는 기존 스냅샷이 없는 작업 복사본에서 실행한다.

```bash
export PYTHONPATH="/home/misys/forza_ws/race_stack/base_system/f110_simulator/f1tenth_gym/gym:${PYTHONPATH:-}"
d=runs/berlin-scout-speed-plan
python3 "$d/sweep.py" replay-425 planned 4.25 six
cp /build/el_eval_avoid "$d/plan-eval"
python3 "$d/branch.py"
python3 "$d/branch_clearance.py"
```

`estimate_from_log.py`는 이전 13.706초 완주 주행의 제어·물리 로그를 읽는 사후 분석이다. 구간별 시간은 [77랩 CSV](../../experiments/berlin-endurance/flying-lap-sectors.csv), 독립 채점 결과는 [ROS 점수 JSON](../../experiments/berlin-endurance/ros-development-scores.json)에서 확인할 수 있다.
