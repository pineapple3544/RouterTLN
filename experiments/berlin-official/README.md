# Berlin 랩별 재배치 비교

## RCN 적용 후 180초 주행

시드 2026, 동일한 공식 배치기, 속도 상한 3.5 m/s로 측정했다. 첫 양의 속도 명령부터 180초 동안 10랩을 넘어도 주행을 계속했다. 평균은 첫 랩부터 공식 채점기가 인정한 모든 완주 랩을 포함하며 마지막 미완주 구간은 제외한다.

| 장애물 설정 | 연속 클린 랩 | 최고 랩 | 평균 랩 | 10랩 누적 랩타임 | 동작 중 25 ms 초과 |
|---|---:|---:|---:|---:|---:|
| 1개 | 11 | 15.540초 | 15.783초 | 157.938초 | 0 |
| 3개 | 11 | 15.630초 | 16.057초 | 160.998초 | 0 |
| 5개 | 11 | 15.720초 | 16.115초 | 161.130초 | 0 |

같은 바이너리에서 잔차만 끄고 재실행했을 때도 세 조건 각각 11랩, 정지·충돌·25 ms 초과 0회였다. 장애물 1·3·5개에서 최고 랩은 각각 15.500초, 15.500초, 15.619초였고 평균은 15.729초, 16.064초, 15.997초였다. 잔차를 켰을 때 실제 최종 조향에 보정이 반영된 프레임 수는 각각 2,102개, 2,972개, 3,940개다. 각 조건의 12개 배치 좌표와 회전은 on/off 간 모두 같았다. 이 시드의 최고 랩으로는 잔차 자체의 이득이 관찰되지 않았다.

세 조건에서 공식 충돌과 재배치 후 기억 초기화 누락은 0회였다. 출발 후 속도가 0.12 m/s 이하로 유지된 측정 구간은 없었다. 5개 조건의 12개 배치 중 두 배치에서 공식 스포너가 실제로 4개를 생성했다. 제어기에 전달되는 정보는 동일했다.

기존 v3에서 장애물 쌍을 통과한 기록과 실패한 기록은 코너 진입 전에 이미 횡방향 위치가 약 1 m 벌어졌다. 수정한 탐색기는 관측된 비정적 LiDAR 반사점 뒤의 미관측 영역을 계획용 점유 격자에 반영하고, 연결된 통로와 조향 속도 제한을 함께 고려한다. 긴 경로가 탈락하면 서로 다른 횡방향 위치와 방향을 유지하는 조향 시퀀스를 6 m까지 검사한다. 실패 상태는 `fixtures/chicane-entry-3.json.gz`와 `fixtures/chicane-entry-5.json.gz`로 보관했다. 같은 점유 격자를 재생하면 두 상태 모두 6 m 조향 시퀀스가 하나 이상 통과한다. 조향을 구현할 수 있는 경로를 선택한 뒤 CPU RCN이 보정하며, 최종 명령은 기존 제동 및 차체 검사를 거친다.

수정 도중 3개 주행에서 한 제어 주기가 36.6 ms 걸려 보호 정지했다. 경로를 계산할 때 중복되던 기준 좌표 변환과 한 스캔 내 동일 셀의 반복 정렬을 제거했다. 합성 스캔 160개를 재생해 기존 센서 특징값 491,520개가 정확히 일치했다. 제한된 벽 정합의 단일 거부 시에는 즉시 제동하고 다음 스캔이 정합 검사를 통과할 때만 재개한다. 이전 유효 스캔과의 150 ms 한계 및 25 ms 명령 보호는 그대로 적용한다.

중간 실험에서는 무보정 전진 궤적 추가 후 5개 조건이 첫 랩에서 멈췄고, 잔차를 처음 연결한 실험에서는 3개 조건이 25 ms를 넘었다. 연산량을 줄인 버전도 5개 조건에서 4랩 후 경로를 잃었다. 위 표의 세 주행은 이 실패를 수정한 동일 바이너리에서 연속으로 측정했다. RTX 5070 Ti에서 사용한 제어기 바이너리 SHA-256은 `7c3d161db25b109f40f2d1d65b72d41902ccf5699dc5f350ab117e755d568dc7`이다.

같은 조건의 실험은 아래처럼 실행한다. 외부 관찰 로그, 실제 장애물 수, 최고 랩, 잔차 조향이 가드 후에도 적용된 프레임 수는 [최종 측정값](seed-2026-rcn-results.json)에 있다.

~~~bash
for n in 1 3 5; do
  python3 scripts/official_berlin_trial.py v3 "$n" "/runs/on/v3-$n" --seed 2026 --full-duration --residual-checkpoint /workspace/models/v3/berlin.rcn || exit 1
  python3 scripts/official_berlin_trial.py v3 "$n" "/runs/off/v3-$n" --seed 2026 --full-duration || exit 1
done
python3 experiments/berlin-official/summarize.py /runs/on --paired-v3 /runs/off
~~~

## 2026-11-26 잔차 적용 전 기준 기록

Berlin 정방향에서 배치 seed 2026을 사용해 각 조건을 한 번씩 측정했다. 두 제어기가 모두 도달한 랩에서 장애물의 좌표와 회전이 모두 일치했다. 모든 주행에서 공식 벽·장애물 충돌과 제어 연산 25 ms 초과는 각각 0회였다. 정지 기준은 아래 실행 조건을 따른다.

| 장애물 | v1.5 연속 랩 / 최고 랩 | v1.5 종료 | v3 연속 랩 / 최고 랩 | v3 종료 |
|---|---|---|---|---|
| 1개 | 10 / 16.290초 | 172.0초에 10랩 | 9 / 17.129초 | 180초 주행 |
| 3개 | 4 / 17.370초 | 92.9초, no_corridor | 1 / 17.940초 | 34.0초, scenario_wait |
| 5개 | 2 / 18.090초 | 66.3초, localization_lost | 1 / 18.489초 | 35.0초, scenario_wait |

v3에서는 11회의 공식 재배치와 센서 기반 초기화가 모두 대응했다. 스캔 시각 기준 초기화와 외부 관찰자의 새 배치 수신 시각 차이는 −5.3~+25.4 ms였다. v3의 각 재배치 후 처음 만난 장애물의 진행좌표는 모두 통과했다. 장애물 3개와 5개에서는 두 번째 랩의 누적 진행거리 약 73 m에서 안전한 전진 후보가 소진됐다. 종료 직전 점유 격자를 재생해도 선택된 6 m·3 m 후보군에서 승인된 경로가 없었다. 이 기록은 seed 하나의 결과다. [조건별 원본 집계](seed-2026-results.json)

초기 정지 상태를 저장한 `fixtures/entrance-3.json.gz`에서는 6 m 기본 탐색의 625개 후보가 전부 탈락했고, 같은 상태에서 횡방향 간격을 좁힌 탐색으로 27개가 통과했다. `fixtures/near-wall-3.json.gz`는 이어지는 저속 코너에서 저장했다. 최종 실패 재현용 점유 격자는 `fixtures/final-3.json.gz`와 `fixtures/final-5.json.gz`다. gzip을 해제한 후 아래 재생 도구의 입력으로 사용한다. CUDA 관련 회귀 테스트 4개와 CPU 전용 Docker 빌드의 테스트 12개가 통과했다.

## 재현 방법

원본 `f1tenth_gym_ros/gym_bridge`, `race_manager/obstacle_spawner`, `race_manager/scorer`가 설치된 ROS Humble 환경을 사용한다. 이 저장소를 `/workspace`로 마운트하고 CUDA 빌드의 `el_controller`를 `/build`에 둔다. 결과 디렉터리는 쓰기 가능해야 한다. 전용 ROS 도메인에서 각 주행마다 세 노드와 제어기를 새로 시작한다.

```bash
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
cd /workspace
export ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
for n in 1 3 5; do
  for mode in v3 v15; do
    python3 scripts/official_berlin_trial.py "$mode" "$n" "/runs/evaluation/$mode-$n" --seed 2026 || exit 1
  done
done
python3 experiments/berlin-official/summarize.py /runs/evaluation
```

두 버전의 속도 상한은 3.5 m/s, 제어 주기는 25 ms, 센서 보호는 150 ms다. v15는 같은 바이너리의 기존 MoE 경로와 v1.5 가중치를 사용한다. `official_simulator`는 시스템 시각으로 스캔을 발행하는 원본 Gym에서도 기존 시뮬레이터 속도 설정을 검증하고 사용할 수 있게 한다. v3는 CPU Race TLN과 CUDA 경로 탐색을 사용하며 잔차 가중치는 비워 둔다. 생성한 장애물 위치와 GT odometry는 채점과 외부 관찰에만 사용한다.

기본 실행은 첫 양의 속도 명령부터 180초, 공식 10랩, 충돌, 또는 주행 중 6초 연속 실측 속도 0.12 m/s 이하에서 종료한다. `--full-duration`을 지정하면 10랩에 도달해도 180초까지 계속 주행한다. `--residual-checkpoint /workspace/models/v3/berlin.rcn`을 추가하면 학습 가중치를 적용한다. 비교 스크립트에서 가중치를 생략하면 무보정 주행을 수행한다. 채점기의 JSON에서 연속 클린 랩과 최고 랩을 집계한다. 첫 장애물 통과는 각 배치에서 처음 만나는 장애물의 기준선 진행좌표를 차량이 정방향으로 지났는지로 표시한다. 25 ms 초과는 제어 CSV의 유효 계산 샘플에서 집계하고 센서 타임아웃은 별도로 확인한다. epoch 오차는 제어에 사용한 스캔 시각과 외부 관찰자가 토픽을 받은 시각의 차이이며 통신 지연도 포함한다.

현재 v3는 6 m 스플라인과 차량 전진 궤적, 관측된 통로를 추종하는 전진 궤적을 검사한다. 모두 탈락하면 같은 지평에서 기준선 좌우 0.58 m를 더 촘촘히 탐색한다. 저속에서 이 후보들도 탈락하면 첫 0.70 m 구간의 차체를 구간당 네 등분하여 검사하고, 마지막으로 종단에서 정지 가능한 3 m 경로를 검사한다. 저속 검사에서는 시작점의 기하 여유를 4 cm에서 8.5 cm로 점차 늘리고 샘플 사이 이동·회전 여유를 더한다. 최종 명령은 별도의 제동 예측과 차체 검사를 통과해야 한다.

센서 기반 점유 격자와 경로 입력은 실행 옵션 `scenario_no_path_snapshot`을 지정하면 정지 시 저장할 수 있다. 외부 관찰 스크립트가 이를 자동 설정한다. 주행 로그에 `no_path.json`이 생성된 경우 아래 명령으로 재생하면 검사별·경로 지점별 탈락 수와 유효 후보 수가 출력된다.

~~~bash
g++ -std=c++17 -O2 -Iinclude tools/replay_scenario_wait.cpp -o /tmp/replay_scenario_wait
/tmp/replay_scenario_wait /runs/evaluation/v3-3/no_path.json
~~~

이 실험의 WSL 호스트에서는 시스템 시각이 약 2.4초 역행했다. 비교 재실행에는 ROS 프로세스의 실시간 시계를 공통 기준점과 단조 시계에 연결했다. 실제 시간과 같은 속도로 흘러가며 메시지가 끊기면 기존 타임아웃이 그대로 적용된다. 안정적인 시스템 시계를 쓰는 호스트에서는 아래 설정이 필요하지 않다. 재현하려면 주행 명령을 실행할 셸에서 다음을 먼저 실행한다.

```bash
gcc -O2 -shared -fPIC experiments/berlin-official/steady_realtime.c -o /runs/steady_realtime.so
export TRIAL_REALTIME_OFFSET_NS=$(python3 -c 'import time; print(time.time_ns()-time.monotonic_ns())')
export LD_PRELOAD=/runs/steady_realtime.so
```

## WSP 잔차 가중치와 GPU 연산 측정

원본 RCN1의 조향·속도 가중치 4,754개를 유지하고, 마지막 은닉층의 32개 활성값에 선형 출력 하나를 추가했다. RCN2의 새 가중치 32개와 편향 1개는 시드 2031의 센서 기록을 사용해 정규화한 Fisher 판별식으로 계산했다. 같은 시드의 마지막 3분의 1에 해당하는 랩은 임곗값 보정에만 사용했다. 표적은 이후 0.5초 안에 가상 장애물 경로를 선택하거나, 유효한 경로가 없거나, 기본 경로 탐색을 재시도한 경우다. 장애물 좌표와 GT 포즈는 이 과정이나 제어 입력에 사용하지 않았다.

실행 중에는 기본 GPU 탐색을 항상 수행한다. 새 장애물을 감지한 요청, 기본 탐색이 실패한 요청, 직전 가상 장애물 경로를 추종하던 요청, 네 번째 요청에는 전체 탐색 예산을 준다. 나머지 요청에서만 RCN2 점수가 낮으면 추가 가상 장애물 후보 연산을 건너뛴다. 다음 요청에서 다시 판단한다. RCN1 또는 빈 가중치 경로에서는 매번 기존 예산을 쓴다.

개발용 재생 시드 2032의 동일 입력 1,652개에서 C++ 조향·속도 출력이 원본과 모두 일치했다. 재생에서 생략 대상으로 분류된 CUDA 작업은 전체 후보 32,278,125개 중 2,525,625개였다. 같은 실행에 기록한 GPU 시간 중 3.968초에 해당한다. 폐루프 주행에서는 경로가 달라질 수 있으므로 아래 공식 비교를 별도로 실시했다.

커널 측정은 동일 CUDA 스트림에서 후보 점수 계산과 선택 경로 복원 커널을 각각 CUDA 이벤트로 감싼다. 측정 구간은 첫 양의 속도 명령부터 시험 종료까지다. CPU 제어·메모리 복사·GPU 전력은 측정치에 포함하지 않는다. 후보 수는 실제 GPU에 제출한 가설 수에 가설당 1,875개를 곱한다. 모든 완주 랩의 평균에는 첫 랩이 포함된다.

재현할 때 공식 러너에 `--full-duration --gate-capture`와 원본 체크포인트를 지정해 장애물 5개에서 시드 2031과 2032를 각각 수집한다. 다음 명령의 `root`는 두 시험 디렉터리가 있는 경로다.

~~~bash
root=/runs/wsp-gpu
# 보관한 수집 데이터를 사용할 때
mkdir -p "$root"
tar -xzf experiments/berlin-official/fixtures/wsp-calibration.tar.gz -C "$root"
OPENBLAS_NUM_THREADS=1 python3 tools/program_residual_gate.py models/v3/berlin.rcn \
  "$root/cal-2031/features.jsonl" "$root/cal-2032/features.jsonl" "$root/rebuilt.rcn" "$root/fit.json"
cmp "$root/rebuilt.rcn" models/v3/berlin-wsp.rcn
python3 experiments/berlin-official/replay_wsp_budget.py "$root/cal-2032" \
  models/v3/berlin.rcn "$root/rebuilt.rcn" "$root/replay.json"
~~~

가중치와 입력 파일의 해시는 [보정 기록](wsp-gate-fit.json)에 있다. 고정 입력에서 생략되는 연산은 [재생 결과](wsp-gate-replay.json)에서 확인할 수 있다.


### 공식 폐루프 비교 결과

RTX 5070 Ti, ROS Humble, 속도 상한 3.5 m/s, 센서 보호 150 ms로 시험했다. 시드 2026에서 WSP는 1·3·5개 조건 모두 180초 동안 무정지·무충돌 11랩을 마쳤고 25 ms를 넘은 제어 주기는 0회였다. 1개 조건의 표는 최종 바이너리로 다시 실행한 원본 대조군을 사용한다.

| 설정 | 원본 → WSP 커널 시간 (ms/주행초) | 원본 → WSP 평균 랩 (초) |
|---|---:|---:|
| 장애물 1개 | 293.8 → 274.8 | 15.786 → 15.785 |
| 장애물 3개 | 301.0 → 279.0 | 16.053 → 16.073 |
| 장애물 5개 | 328.0 → 288.0 | 16.004 → 16.077 |

5개 조건에서는 GPU에 제출한 후보 수가 31.6% 늘었다. 가설마다 조기 탈락 지점과 비용이 다르고 작업자가 시간 예산 안에서 추가 후보를 제출하므로 커널 시간과 후보 수는 각각 기록한다. 1개 무잔차의 첫 실행은 센서 타임아웃으로 3랩에서 멈췄고 같은 시드 재실행은 11랩을 마쳤다. 5개 WSP의 첫 실행 시도에서 제어기 시작 전 ROS 토픽 대기가 만료되어 별도 ROS 도메인에서 재시작했다.

별도 시드 2033, 장애물 5개 첫 실행에서 WSP는 9랩, 원본은 5랩 뒤 위치 추정기의 `ambiguous_motion`으로 정지했다. 두 실행 모두 충돌 0건이었다. 이 체크포인트는 선택 실행으로 제공한다. 기존 RCN1 기본값을 유지한다.

~~~bash
# v3 제어기 인자
-p residual_checkpoint:=/workspace/models/v3/berlin-wsp.rcn
~~~

[주행별 시간·후보 수·완주 기록](wsp-gpu-results.json)

같은 시드 2033에서 실행 순서를 바꿔 한 번씩 더 주행했다. 원본은 유효한 경로를 찾지 못해 `scenario_wait` 상태에서 8랩 뒤 정지했고, WSP는 무정지·무충돌 11랩을 완주했다. 두 번 합쳐 180초 완주는 원본 0/2, WSP 1/2였다. 첫 60초 동안의 초당 커널 시간은 첫 실행에서 원본 310.9 ms, WSP 294.7 ms였고, 반복에서 원본 321.6 ms, WSP 302.3 ms였다. WSP 완주 실행에서는 공식 결과 파일과 180초 관측이 저장됐고, 종료 처리 중 scorer 로그에 rclpy 예외가 남았다. [전체 수치](wsp-gpu-results.json)
