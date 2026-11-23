# Berlin 정방향 센서 기반 3랩 평가

WSP Race와 원본 Avoid를 연결하고 첫 랩부터 계획을 새로 만들었다. 스캔을 Berlin 정적 지도에 정합해 구한 자세와 직전 구동 명령을 사용한다. 시뮬레이터의 절대 자세·장애물 배열·충돌 상태는 주행 입력에서 제외했다. [실행 중인 ROS 노드](ros-node-info.txt) · [코드와 모델 SHA-256](provenance.json)

| 시험 | 무정지·무충돌 3랩 | 충돌한 시도 | 통과한 시도의 랩별 중앙값 |
|---|---:|---:|---|
| 오프라인, 300개 배치 | 289/300 (96.3%) | 0 | 28.17 / 18.20 / 18.18초 |
| ROS, 사전 추첨 30개 × 2회 | 58/60 (96.7%) | 0 | 28.70 / 18.51 / 18.45초 |

기존 GT 자세 입력으로는 같은 300개에서 294개(98.0%)가 통과했다. 자세 추정을 적용하자 6개가 새로 실패하고 이전 실패 중 1개가 통과했다. 11개 실패 사유는 `no_corridor` 6개, `localization_lost` 3개, `braking_footprint`와 `dynamic_braking_footprint` 각 1개다. 고정한 실행 파일로 다시 돌린 300개에서도 시드별 통과 여부와 첫 실패 사유가 일치했다. [오프라인 시드별 랩타임과 판정](offline.csv)

ROS에서는 **30개 중 29개가 두 번 모두 통과**했다. 남은 `625657002`는 첫 랩의 6.60 m와 6.40 m에서 두 번 모두 `no_corridor`로 보호 정지했다. 같은 위치에서 오프라인은 Avoid 주행을 이어 갔다. 두 ROS 정지 시점의 위치 추정 오차는 3.7 cm와 5.2 cm, 제어 연산은 2.04 ms와 2.51 ms였다. 실패 지점의 근접성은 확인했고, 통로가 끊기게 한 요인은 아직 분리하지 못했다. [ROS 반복별 랩타임과 판정](ros.csv) · [같은 진행 위치의 비교](ros-offline-disagreement.json)

ROS 60회는 모두 첫 시도의 주행 기록으로 판정했다. 실행 중 시험 스크립트를 갱신해 두 번째 반복 두 건(`623640967`, `690736814`)의 주행 후 채점 단계에서 셸 오류가 발생했다. 두 주행 모두 이미 결승 명령과 물리 로그를 남겼다. 같은 로그로 채점기와 검사기를 다시 실행해 실제 3랩을 확인했다. CSV에 두 건의 원래 종료 코드와 재채점 여부를 남겼다. 이전 GT 캠페인의 ROS 기록은 이 표에 포함하지 않았다. [집계 JSON](summary.json) · [주행·심판 원본 기록 압축 파일](evaluation-records.tar.gz)

채택 기준인 모든 배치 무정지·무충돌을 충족하지 못해 속도 상한을 높이는 실험은 보류했다.

## 시험 조건과 판정

- 같은 사전 추첨 목록의 300개 배치를 오프라인에서 주행하고 그중 사전 선정한 30개를 ROS에서 두 번씩 주행했다. 각 ROS 시도는 별도 시뮬레이터와 새 제어 프로세스로 시작한다. GT 버전에서 이미 사용한 목록을 다시 썼으므로 두 방식의 대응 비교에 해당한다. [사전 추첨 원본](../berlin-blind-300/preregister.json) · [배치별 장애물 설정](../berlin-blind-300/layouts.json)
- 25 ms 제어 주기, 150 ms 스캔 보호, 첫 랩 상한 2.2 m/s, 이후 회피 목표 범위 2.7~3.0 m/s를 적용했다. 물리적 차량 속도는 목표 속도와 다를 수 있다. ROS는 CPU 추론을 사용한다.
- 오프라인에서는 물리 엔진의 전진 진행량, 접촉, 정지와 보호 개입을 판정한다. ROS에서는 물리 브리지가 매 샘플 실제 위치·속도·충돌을 기록한 뒤 종료한다. 채점기가 궤적을 지도 경로에 투영해 출발 지점에서부터 전진 3랩을 누적했는지 확인한다. 출발 후 처음 0.5 m와 결승선 통과 후에는 정지 판정을 제외하고, 주행 중 0.05 m/s 미만·보호 정지·충돌이 있으면 실패로 기록한다. ROS 잠정 통과 기록은 실제 속도가 처음 0.1 m/s를 넘은 순간부터 결승선까지 0.05 m/s 미만으로 다시 떨어졌는지도 검사했다. 두 반복의 출발 시 계획 비활성, 첫 랩 뒤 계획 활성과 적용 이력도 확인한다. [채점 코드](../../scripts/score_drive.py) · [집계 코드](summarize.py)
- ROS 랩타임은 물리 샘플에 찍힌 경과 시각, 오프라인 랩타임은 시뮬레이션 시각이다. 랩타임 중앙값에는 3랩을 모두 통과한 시도만 포함한다.

## 센서와 모델 검사

- 차량의 정답 자세 필드에 지도 밖 값을 넣고, 네 출발점에서 두 진행 방향에 걸쳐 1 cm 표준편차의 합성 LiDAR 거리 노이즈로 전역 위치 추정을 시험했다. 유효 빔이 없는 스캔과 151 ms 간격 스캔은 거절했다. 기존 GT 주행 기록 다섯 개에서는 과거 구동 명령과 스캔만 순서대로 공급한 뒤 정답과 오차를 계산했다. 각 주행의 위치 오차 95백분위는 2.86~3.69 cm, 방향 오차 95백분위는 0.49~0.65도였다. 대회 센서의 미공개 노이즈 분포에서 얻은 수치는 아니다. [재생 결과](localizer-test.log) · [회귀 시험](../../tests/scan_localizer_test.cpp)
- 별도 ROS 도메인에서 주행 중 브리지를 250 ms 멈췄다. 외부 관측자가 본 마지막 스캔 시각에서 첫 속도 0 명령까지 149.3 ms였다. 이후 관측한 명령 17개의 양의 속도 명령은 0개였고 제어기는 `sensor_timeout`을 기록했다. [보호 시험](watchdog-final/report.json)
- 단일망 `unified-race-avoid-v2.pt`와 두 전문가 모델을 기존 개발용 5개 시드에서 같은 입력과 같은 경로 계획으로 비교했다. 두 모델은 4/5회, 단일망은 0/5회 완주했다. 단일망의 5회 모두 제동 중 차체가 점유 영역과 겹칠 것으로 예측해 보호 정지했다. 이 비교의 결론은 해당 체크포인트와 배치에 한정된다. [배치별 비교](policy-comparison.csv)
- C++ 회귀 시험 13개와 독립 채점기 시험 3개가 통과했다. Race 단독 평가기도 같은 위치 추정 입력으로 빈 Berlin 트랙 1랩을 완주했다. [C++ 시험](all-tests.log) · [채점기 시험](judge-tests.log) · [Race 단독 기록](race-only/summary.json)

## 재현

프로젝트 Docker 컨테이너에서 저장소를 `/workspace`에 마운트한다. ROS 2 Humble, 기존 Forza Gym 환경, LibTorch, Eigen과 SciPy가 필요하다. 아래 출력 경로는 처음 실행할 때 비어 있어야 한다. CUDA 아키텍처 120과 ROS 재현 스크립트의 CPU 번호는 평가 호스트에 맞춘 값이다.

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
export PYTHONPATH="/home/misys/forza_ws/race_stack/base_system/f110_simulator/f1tenth_gym/gym:${PYTHONPATH:-}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
cmake -S . -B /build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120 -DCMAKE_PREFIX_PATH=/opt/libtorch
cmake --build /build -j2
ctest --test-dir /build --output-on-failure
python3 tests/test_score_drive.py
cd /workspace/runs/berlin-sensor-only
EVAL_CORES=1,9 python3 evaluate_frozen.py reproduced-offline /build/el_eval_avoid all
python3 ros_trials.py --controller /build/el_controller --output /workspace/runs/berlin-sensor-only/reproduced-ros
```

단일망 비교를 재현하려면 저장소 루트에서 같은 배치 파일을 사용한다. 두 번째 체크포인트 인자가 있으면 두 네트워크를 각각 로드한다.

```bash
cd /workspace
/build/el_eval_avoid models/unified-race-avoid-v2.pt /tmp/single-network.jsonl runs/berlin-sensor-only/single-network-cases.json
/build/el_eval_avoid models/avoid-corridor-v2.pt /tmp/two-networks.jsonl runs/berlin-sensor-only/single-network-cases.json models/race-berlin-wsp.pt
```
