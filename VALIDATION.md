# 구현·검증 결과

검증일: 2026-10-28. 실제 차량이나 Orin Nano를 사용하지 않았으며, TLN 학습은 실행하지 않았다.

## 환경

- Windows + WSL2 Ubuntu, RTX 5070 Ti 16 GB, Ryzen 7 7800X3D.
- `el-native:cuda12.8`, CUDA 12.8.93, GCC 11, ROS 2 Humble.
- LibTorch 2.9.1 CUDA 12.8. 설치 ZIP SHA256: `b052452965093db69f537b3cf376812d5acf6dca28819b20d28d7f0b171d7699`.
- 새 TLN 모델 파라미터: 177,866개. 가중치는 학습되지 않았다.
- 기본 Python F1TENTH Gym과 브리지를 그대로 사용했다.

## 빌드와 회귀 검사

`bash /workspace/scripts/build.sh`로 전체 C++/CUDA 빌드와 CTest 2개를 통과했다.

1. `native_regression`: 관측 장애물 등록, 가림 시 기억 유지, 빈 공간 재관측에 의한 제거, reset, 제동거리 부족/NaN 명령 거부, router 유지/복귀, 데이터 roundtrip·손상 탐지, CUDA MPPI 정상/완전 차단 환경.
2. `training_environment_forward_only`: 새 네트워크의 GPU forward와 유한 출력 확인. backward·optimizer step·체크포인트 저장 없음.

`mode:=moe`에서 체크포인트를 지정하지 않으면 `trained TLN checkpoint required`로 실행을 거부하는 것도 확인했다. 무작위 초기화 정책을 실제 주행 정책으로 사용하지 않았다.

## 폐루프 시험

조건: `hyhall12`, GT odometry, 속도 상한 2.5 m/s, 정적 0.30 m 장애물 1개, 기본 MPPI 512 samples × 32 steps, 예측 dt 0.05 s, 제어 목표 주기 0.025 s.

| 실행 | 반복 teacher + fallback MPPI | 검증용 데이터 수집 | MPPI 단독 |
|---|---:|---:|---:|
| 로그 | `repeat-final-test.log` | `collection-test.log` | `mppi-test.log` |
| 시드 | 2026 | 2027 | 2026 |
| 주행 요청 시간 | 70초 | 18초 | 35초 |
| 기록된 제어 주기 | 2,381 | 633 | 1,177 |
| 누적 경로 진행 | 146.97 m | 38.21 m | 63.76 m |
| 충돌 프레임 | 0 | 0 | 0 |
| 거부/정지 전환 주기 | 0 | 0 | 1 |
| MPPI로 실행한 주기 | 510 | 126 | 1,176 |
| 계산시간 p50 | 1.83 ms | 1.81 ms | 2.86 ms |
| 계산시간 p95 | 3.06 ms | 3.01 ms | 3.82 ms |
| 계산시간 p99 | 3.58 ms | 3.46 ms | 4.17 ms |
| 계산시간 최대 | 4.24 ms | 4.01 ms | **119.92 ms** |

트랙 경로 길이는 약 42 m이므로 반복 시험은 3바퀴 이상 진행했다. 이 수치는 최근접 waypoint 기반 누적 진행량이며 강사 race_manager의 공식 랩 판정 결과가 아니다. 기본 시뮬레이터의 `/sim/collision`을 충돌 판정에 사용했다.

MPPI 단독 실행의 step 168에서 약 119.92 ms 지연이 발생했다. 설정한 25 ms 계산 마감을 초과해 정지 명령으로 전환했고 이후 주행을 재개했다. 원인은 아직 특정하지 않았다. 이 결과는 WSL/데스크톱 GPU 환경에서 하드 실시간 보장을 주장할 수 없음을 보여준다.

계산시간은 제어 callback 내부의 관측 처리·기억·판단·계획 구간이다. DDS 대기, 센서 전달, 최종 로그·메시지 발행 전체를 포함한 종단 지연은 아니다. 반복 시험 중 별도의 forward 환경 확인이 수행된 구간이 있으므로 순수 성능 벤치마크로 취급하지 않는다.

## 데이터 경로

- 파일: `data/pipeline_validation.elds`, 메타데이터: `.elds.json`.
- 633개 레코드, Race 520개, Avoid 113개.
- C++ 재읽기 검사에서 유한 입력·단조 증가 step/시각·파일 스키마 통과.
- 현재 검사 기준 유효 라벨 633개, 충돌 프레임 0개.
- 약 11 MB의 **파이프라인 검증용** 데이터다. 본 학습 데이터로 승인하거나 학습에 사용하지 않았다.
- 학습 manifest는 빈 상태다. checkpoint 파일은 생성하지 않았다.

## 센서 중단 검사

`scripts/test_watchdog.sh`에서 기본 시뮬레이터를 중단한 뒤 제어기의 `/drive`를 새로 구독했다. 목표 속도 `0.0` 발행을 확인했다. 로그: `watchdog-test.log`, `runs/watchdog-command.txt`.

## 발견하고 수정한 문제

- 직접 실행 시 기본 시뮬레이터의 YAML 노드 이름 불일치: `__node:=bridge`로 수정.
- 초기 반복 시험에서 충돌 없이 장애물 근처에 정체: MPPI 종료 후 제동 비용, 비용순 대체 후보 검사, 25 ms 제어 주기에 맞춘 warm-start 이동, 장시간 미사용 후 계획 초기화를 추가.
- 일반 전환의 반복을 줄이기 위해 clear 유지와 MPPI 복귀 안정 구간을 적용.
- 기존 ROS setup 파일의 미정의 변수와 셸 `set -u` 충돌: ROS 환경 로드 후 적용.

## 아직 검증하지 않은 사항

- 학습된 TLN의 closed-loop 주행과 실제 MoE 성능.
- 학습 후 DAgger 라벨 수집 경로의 폐루프 효과.
- 미학습 트랙, 많은 장애물 배치, 고속·낮은 마찰·센서 잡음 일반화.
- 동적 장애물 추적, localization 오차, 디스크 저장 장애물 기억의 재사용.
- 형식적인 안전 보장, Shield-MPPI/CBF 논문의 재현.
- Orin Nano 빌드와 성능.

이 단계의 완료 범위는 **기본 시뮬레이터에 연결되는 C++/CUDA 제어·데이터 파이프라인과 새 TLN 학습 환경**, 그리고 위 제한된 조건의 실행 검증이다.

## Global planner 검토 — 2026-10-28

현재 C++ 제어기는 저장된 `global_traj_wpnts_iqp` 경로를 읽는다. 실시간 global optimizer는 실행하지 않는다. 기존 ForzaETH 소스의 `global_planner_logic.py`는 `mincurv_iqp`와 `shortest_path` 결과를 생성한다. 최소 곡률은 최소 랩타임과 동일한 목적함수가 아니다.

실제 컨테이너의 hyhall12 JSON을 읽어 IQP/SP 경로를 별도 디렉터리에 복사하고, 동일한 C++/CUDA MPPI로 각각 시험했다. 원본 경로와 제어 알고리즘은 변경하지 않았다. 재현: `bash /workspace/scripts/audit_planners.sh`. 실행은 Docker 내부에서 수행되며, 새 분석 도구도 C++이다. TLN 학습은 실행하지 않았다.

조건: 동일 초기 위치, 장애물 0개, seed 2026, MPPI 512×32, dt 0.05 s, 속도 상한 2.5 m/s, 각 70초 실행 요청. 센서 timestamp가 기록된 실제 구간은 IQP 64.53초, SP 64.67초다. 각 경로 1회 시험이며 MPPI는 고정 참조 경로에서 벗어날 수 있다.

| 항목 | 기존 IQP | 저장된 SP |
|---|---:|---:|
| 경로 길이 | 41.90 m | 39.08 m |
| 최대 절대 곡률 | 0.493 /m | 1.393 /m |
| 곡률로 계산한 필요 조향각, atan(0.3302κ) | 0.161 rad | 0.431 rad |
| 현재 0.4 rad 조향 제한을 넘는 waypoint | 0 | 3 |
| 원래 저장된 속도 범위 | 4.93–10.47 m/s | 2.93–10.21 m/s |
| 추정 1/2/3번째 랩 시간 | 21.42 / 21.04 / 21.23 s | 20.26 / 20.09 / 19.77 s |
| 2·3번째 랩 평균 | 21.13 s | 19.93 s |
| 첫 바퀴 이후 참조 경로 거리 RMS | 4.26 cm | 2.32 cm |
| 첫 바퀴 이후 참조 경로 거리 최대 | 10.45 cm | 8.54 cm |
| 충돌 프레임 / 명령 거부 | 0 / 0 | 0 / 0 |
| 계산시간 p99 | 3.96 ms | 4.03 ms |
| 계산시간 최대 | 6.20 ms | 4.84 ms |

SP가 이번 실행의 2·3번째 랩 평균에서 약 5.7% 빨랐다. 따라서 현재 IQP가 현재 저속 MPPI 구성에서 최선이라고 주장할 수 없다. SP의 일부 곡률이 조향 제한을 넘지만 MPPI는 해당 선을 정확히 따르도록 강제되지 않으며, 실제 주행 성공은 저장된 모든 곡률의 실현 가능성을 증명하지 않는다. 두 경로의 속도는 모두 2.5 m/s로 잘리므로 원래 최적화된 속도 프로파일의 성능을 시험한 것도 아니다.

랩은 최근접 waypoint 누적 진행량과 ROS timestamp로 추정했다. 강사 race_manager 공식 랩 기록이나 고정 시간 간격의 simulation-clock 벤치마크가 아니다. 단일 시드·단일 트랙·무장애물 결과여서 고속, 회피, 일반적 우월성 또는 안전성을 입증하지 않는다. 첫 위치가 SP와 약 25 cm 떨어져 있어 추종 오차 비교는 첫 바퀴 이후 수치도 별도로 보고했다. 기존 설정 파일에는 IQP safety_width 0.90 m, SP 0.75 m로 서로 다른 값이 있으며 저장 경로의 생성 당시 설정까지 확인하지는 않았다. 따라서 동일 여유 폭으로 재최적화한 순수 목적함수 비교가 아니다.

검토 결론 및 다음 우선순위:

1. IQP는 재현 가능한 기준선으로 유지하되 최종 Race teacher로 확정하지 않는다. SP도 제약 보정 후 비교 후보로 유지한다.
2. 시뮬레이터·MPPI·global planner의 차량 형상, 조향, 가감속 및 횡가속 한계를 일치시키고 경로별 속도 프로파일을 다시 산출한다. 현재 C++ Track은 곡률과 좌우 경계 데이터를 읽지 않으며 원래 속도에 상한만 적용한다.
3. Race 목표는 동일 모델·안전 여유 아래 최소 곡률, 최단거리, minimum-time 경로의 폐루프 비교다. minimum-time 최적화는 아직 실행하지 않았다. 새 구현은 C++/CUDA 요구사항을 따라야 한다.
4. Avoid 목표는 장애물 기억을 반영해 좌우 우회 후보와 주행 가능한 통로를 만드는 local planner를 보완하는 것이다. 현재 MPPI는 고정 global reference와 관측 장애물 비용을 사용하며 별도의 우회 경로/통로 생성 계층은 없다. global optimizer 교체만으로 이 문제를 해결할 수 없다.

근거: [TUM 원 구현](https://github.com/TUMFTM/global_racetrajectory_optimization)은 최소 곡률과 minimum-time의 차이 및 후자의 추가 모델·연산 요구를 설명한다. [ForzaETH 논문 §6](https://arxiv.org/html/2403.11784v2#S6)은 offline 최소 곡률 global planner와 online 회피 local planner를 구분한다. 본 시험은 논문 성능 재현이 아니며 위 개선 순서는 현재 코드와 로그에 근거한 설계 판단이다.

산출물: `runs/planner-audit/geometry.json`, `iqp-metrics.json`, `sp-metrics.json`, `iqp-test.log`, `sp-test.log`, 그리고 `runs/audit-iqp.csv`, `runs/audit-sp.csv`.

## 2026-10-29 SMPPI / SVG-MPPI teacher collection

- C++/CUDA teacher implementations and synchronous C++ Python-C-API adapter to the existing simulator were built inside el-native-dev.
- CTest: teacher_regression, native_regression (including wall occlusion checks), and training_environment_forward_only passed.
- Final index: data/teachers-18m-current/audit.json. Full ELDS payload validation, placement-coordinate checks, original-attempt split checks, and SHA256 inventory completed.
- Race: 6 successful two-lap episodes / 7,184 records. Avoid: 53 completed encounter segments / 15,137 records (48 wall-occluded, 5 rear-blind placements).
- Latest placement mixture: 30% exact waypoint proposals, 70% bounded lateral/longitudinal jitter. Four exact-waypoint and seven jittered encounters passed in this additional batch.
- LiDAR max range 18 m; checkpoint schema 3. Old 10 m data is excluded and rejected by the training loader.
- Avoid attempts: 36 total; 8 finished their configured distance, 28 stopped for no feasible plan. All attempts reported zero collisions. Failed/incomplete tails are excluded; successful prefixes remain grouped by original attempt for splitting.
- Actual peak speed: Race 3.640 m/s, Avoid 2.863 m/s. No 10 m/s performance claim. No TLN training or learned checkpoint was produced.
- Details and limitations: TEACHER_DATA.md.

## 2026-10-29 teacher improvement v5

- Fixed world-frame collision grid at original map resolution, validated incumbent retention, covariance-preconditioned SVG guide updates, continuous lateral lattice and curvature speed profile, and exact angles for downsampled LiDAR rays.
- CTest 3/3 passed, including nondegrading incumbent, collision frame-invariance, CUDA world occupancy, and exact-ray-angle regressions.
- Separate fixed scenarios: Race mean lap 29.825 -> 19.883 seconds (6 laps each); Avoid completed 3/6 -> 6/6, zero collisions. Scenario JSON and obstacle geometry equality verified.
- Cyclic scenarios: same 8 seeds/starts/rules completed 3/8 -> 8/8, obstacle passes 11 -> 32, zero collisions. Adaptive later placements are not identical between controllers. Spawn visibility/distance guards verified.
- Tradeoff: mean teacher compute approximately Race 5.98 -> 18.75 ms; Avoid 17.85 -> 32.72 ms. This is offline synchronous collection, not an Orin or hard-deadline benchmark.
- Existing training files/manifests unchanged; new improvement runs used metrics_only. No TLN training.
- Reports: runs/teacher-improvement/final-comparison.json, cycling-comparison.json, TEACHER_IMPROVEMENT.md.

## 2026-10-29 — 개선 교사 v5 확장 수집

- Docker C++/CUDA 수집기: hyhall12/teras, 양방향, 시작 자세·속도 상한·장애물 형태/위치/도로 구간 변화.
- Race 28개 2바퀴 완주(22,263프레임), Avoid 248개 완료 통과(26,367프레임). 총 48,630프레임, 843.51 MiB.
- 전체 276개 채택 ELDS 및 sidecar 감사 통과. train/validation/test 원본 시도·시드·파일 분리 확인. 552개 파일 SHA256 기록.
- 회피 원본 시도 87개: 목표 달성 82, no_feasible_plan 5, 기록된 충돌 0. 의도적으로 중단한 후방 전용 시도 1개는 별도 기록하고 학습에서 제외.
- 3개 CTest 통과. 중복 파일, 변경된 split 메타데이터, 중복 shard, 미완료 수집 거부 확인. 명시적 중단 기록을 통한 완료 구간만의 통합 확인.
- 최종 감사: `runs/expanded-final-audit.log`. 결과와 한계: `TEACHER_DATA_EXPANDED.md`. 학습 목록: `data/teachers-expanded-v5-current`.
- TLN 학습, backward, optimizer step, 학습 체크포인트 생성은 수행하지 않았다.

## 2026-10-29 — Race TLN 첫 학습 및 최저 검증 모델 저장

- Docker C++ LibTorch, train 12,773 / validation 4,829프레임, 20 epochs. Test는 평가하지 않음.
- 검증 최저 epoch 19: Smooth L1 0.001000471601. 마지막 epoch 20은 0.001012768310이므로 교체하지 않음.
- 실제 저장 모델을 재로드해 최저 손실 일치 확인. 검증 조향 MAE 0.011929 rad, 속도 MAE 0.185020 m/s.
- CTest 4개 통과. 새 검사: 최저 epoch 실제 가중치 보존, 악화/동률/NaN 처리, 재로드, 기존 실행 덮어쓰기 거부.
- 모델: `models/race-teachers-expanded-v5.pt`. 실행 기록: `runs/race-tln-v1-20261029`. 상세: `TRAINING_RACE.md`.
- Avoid 학습, 시험 세트 평가, TLN 폐루프 주행 및 MoE 통합은 실행하지 않음.

## 2026-10-29 — Race TLN 폐루프 검증

- 기존 F110 Gym, C++ 평가기, 학습된 19 epoch 가중치 고정. 두 지도·양방향·10조건 × 3제어방식, 각 3바퀴 목표.
- TLN 단독: 10/10 완주, 30랩, 충돌 0, 평균 19.471초. SMPPI v5: 10/10, 평균 19.401초.
- 동역학 검사/SMPPI 보호: 9/10 완주, 충돌 0, no_feasible_plan 1. fallback 39주기 중 38주기는 유효 계획, 1주기는 실패 감속.
- 실패 조건 재현: 929행의 물리/제어 열 동일. 관측 기억 포함 시 4 cm 안전 여유 영역이 점유와 겹치고, 기본 지도만 사용하면 후보·nominal 검사가 통과. 조건 완화 없이 실패로 기록.
- 4개 CTest 통과, 진행거리 wrap·경계 보간 검사를 추가. 공유 시뮬레이터 코드 추출 전후 텍스트 일치, 30개 대응 조건, 모델 hash 불변 확인.
- 결과: `SIMULATION_RACE.md`, `runs/race-closedloop-v1-20261029`, `runs/race-closedloop-diagnostic-v1-20261029`.
- 오프라인 동기식 실행이며 ROS 실시간·Orin Nano·장애물 회피·전체 MoE 검증은 아니다. 추가 학습은 실행하지 않았다.

## 2026-10-29 — 보호 로직의 벽 기억 오류 수정 및 같은 조건 재검증

- 실패 전 929개 스캔 재생: 벽보다 0.13–2.07 cm 짧은 반환점 15개가 5 cm 빈 셀의 가짜 점유로 누적돼 4 cm 차체 여유와 겹쳤다. 해당 셀은 수정 후 계획 지도에서 비점유다.
- 같은 광선의 첫 지도 벽과 3 cm 이내 반환점을 연결하는 보정 적용. 기존 장애물 기억·실제 free-space 제거·기본 벽·차체 여유 유지.
- 첫 보정 지도를 TLN 입력에도 적용한 v2는 단독 주행 2충돌로 기각했다. 최종 v3는 계획 지도와 schema 3 학습 입력을 분리한다. 929프레임의 입력 2,853,888개 값이 기존과 정확히 같다.
- 같은 설정(출력 경로만 변경), 같은 모델 SHA256, 두 지도·10조건 × 3방식 재검증: 단독·보호·SMPPI 각각 10/10 완주, 30랩, 충돌 0. 보호 경로 비상 감속 0, 유효 fallback 24/11,702주기. 원래 실패 조건은 3랩 완료, fallback 0.
- TLN 단독 11,687행 × 14개 물리/명령 열이 원래 CSV와 동일. 최종 CTest 4/4 통과; 벽 잡음, 실제 근접 장애물, 가림 기억, schema 3 보존 검사 포함.
- 추가 SVG-MPPI 회피 회귀 4조건, 12개 장애물 통과, 충돌·유효 계획 부재·기하 겹침 0. 생성 시 가림/거리 규칙 통과. metrics-only, 기존 학습 데이터에 병합하지 않음.
- 상세: `MEMORY_PROTECTION_FIX.md`. 실행: `runs/race-closedloop-v3-memory-20261029`, 원인/회귀: `runs/memory-wall-fix-20261029`. 가중치·학습 데이터 불변, 재학습 없음.
- 이 검증은 PC 동기식 실행이다. 벽에서 3 cm 이내의 작은 돌출물 구별, ROS fallback 통합, 실시간 deadline, Orin Nano, Avoid TLN·전체 MoE는 별도 검증이 필요하다.
