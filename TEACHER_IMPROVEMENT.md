# SMPPI / SVG-MPPI 교사 개선 기록

2026-10-29. Docker `el-native-dev`, RTX 5070 Ti, 기존 hyhall12 Gym에서 검증한다. 신규 제어·수집·분석 코드는 C++/CUDA이며 TLN 학습은 실행하지 않는다. 기존 ELDS 학습 데이터와 통합 목록은 변경하지 않았다. 이번 개선 시험은 `runs/teacher-improvement`에 분리하며 새 교사의 대규모 학습 데이터셋으로 재분류하지 않는다.

## 구현

1. **유효 incumbent 보존.** SMPPI/SVG 가중 평균이 더 나쁜 경로가 되면 검증된 warm start 또는 nominal 제안을 유지한다. 매 후보를 동일 비용과 차체 충돌·종단 제동 검사로 비교한다. SMPPI 변화율 상태는 다른 제안을 채택할 때 초기화해 실제 명령과 맞춘다. `source` 열은 weighted update=0, shifted warm=1, nominal=2, SVG guide=3, sampled fallback=4다. Nominal 보존을 순수 MPPI 갱신으로 숨기지 않는다.
2. **SVG guide의 갱신 안정화.** 이전 갱신은 작은 조향 분산의 역수로 인해 과도하게 커져 이동 제한에 자주 걸릴 수 있었다. Monte Carlo 로그 밀도 기울기에 공분산을 곱하는 preconditioning을 적용하고, guide와 가중 표본 평균 사이의 절반만 이동한다. Guide 이동 기록 중 비용이 가장 낮은 것을 prior로 사용하며 기존 bounded covariance fitting을 유지한다.
3. **회피 참조 경로의 연속성.** Lateral lattice 시작점을 차량의 실제 횡방향 위치·방향에 맞춘다. 위치와 이전 위치를 상태로 하는 2차 차분 비용으로 급격한 좌우 변경을 억제한다. 실제로 참조 경로가 점유됐을 때만 수정하며, 수정 경로의 곡률에 맞춰 참조 속도와 제동 profile을 제한한다.
4. **고정된 world collision grid.** 차량을 따라 이동·회전하는 0.15 m max-pooled 지도 대신 원본 지도 해상도인 0.05 m의 world occupancy를 충돌 검사에 사용한다. 기존 0.04 m 차체 여유와 종단 제동 검사를 유지한다. 점유 정보는 기존 base map과 LiDAR 장애물 기억뿐이며, 시나리오 생성 좌표를 교사에 전달하지 않는다. 최대 262,144개 지도 셀을 지원하며 초과하면 명시적으로 실패한다.
5. **LiDAR 각도 보존.** 1,080개 중 선택된 각 빔의 실제 각도를 기억 지도 ray tracing과 시간 이력 재투영에 사용한다. 181개 표본이 원본 각도와 정확히 같은 간격이라는 이전 가정을 제거했다. C++ 수집기와 ROS 입력 처리 모두 수정했다. LiDAR 18 m, 관측 배열 형상과 정규화 단위는 유지한다.

신규 알고리즘은 논문을 그대로 재현한 구현이 아니라 incumbent 보존, 제약 검사, 국소 경로 제안 및 공분산 preconditioning을 추가한 실용적 변형이다. 고정 시간 전체 학습을 수행하거나 학습된 TLN으로 교사를 대체하지 않는다.

## 검증 구성

최종 고정 조건 결과는 `runs/teacher-improvement/final-comparison.json`에 저장했다. 동일 조건 확인은 도구에서 통과했다.

최종 순환 회귀 결과는 `runs/teacher-improvement/cycling-comparison.json`에 저장했다. **동일한 8개 시드·시작점·배치 규칙에서 기존 완료 3/8 → 최종 완료 8/8**, 완료된 장애물 통과는 **11개 → 32개**, 최종 시뮬레이터/독립 기하 충돌은 0이었다. 생성 시 가시 빔 0개 및 최소 거리 조건도 다시 검사했다. 모든 어려운 배치나 기존 전체 36개 시도에서 성공한다고 주장하지 않는다. 이번 순환 회귀에는 일반 벽 뒤/사각지대 혼합 6개 시드와 후방 직선 전용 2개 시드가 포함된다.

v4에서는 이 순환 회귀의 7/8이 완료됐고, 한 시드에서 base map에는 없지만 관측 기억에 추가된 점유가 차량 footprint와 겹쳤다. 실제 빔 각도를 보존한 v5에서는 해당 시드가 3바퀴와 장애물 5개 통과를 완료했고, 전체 8개 시드의 회귀도 통과했다. v5 고정 조건 및 순환 시험은 CTest 통과 후 실행했다. 최종 소스와 해시는 `runs/teacher-improvement/final-source`, `improved-source-sha256.json`에 보존한다.

| 별도 회귀군 지표 | 기존 | 최종 v5 |
|---|---:|---:|
| Race 평균 랩타임 (6랩) | 29.825초 | 19.883초 |
| Race 중앙값 / 최소 / 최대 | 29.75 / 29.55 / 30.20초 | 19.875 / 19.45 / 20.50초 |
| Race 2바퀴 주행 완료 | 3/3 | 3/3 |
| Avoid 고정 장애물 주행 완료 | 3/6 | 6/6 |
| 고정 조건 충돌 | 0 | 0 |
| Race 교사 계산시간, 조건별 평균의 평균 | 5.98 ms | 18.75 ms |
| Avoid 교사 계산시간, 조건별 평균의 평균 | 17.85 ms | 32.72 ms |

Race 평균 랩타임 감소는 약 33.3%다. Avoid 완료 랩 평균은 기존 23.05초(완료 3조건), 개선 21.154초(완료 6조건)지만 표본 구성이 다르므로 이 두 평균만으로 속도 개선률을 계산하지 않는다. 최종 고정 시험의 최고 실제 속도는 Race 3.850 m/s, Avoid 2.499 m/s다. 종단 안전 검사와 원본 지도 해상도 검사로 계산량이 늘었으며, 최소 랩타임의 전역 최적성을 주장하지 않는다.

고정 회귀군 Race의 2,389개 명령 중 1,880개는 최적화 갱신, 509개는 검증된 warm 제안 유지였다. Avoid 2,540개 중 401개는 weighted update, 355개는 warm 유지, 14개는 nominal 제안, 1,770개는 갱신된 SVG guide였다. Warm 제안은 첫 주기에 nominal로 초기화된다. 단순 추종기로 전환해 결과를 SMPPI/SVG로 표시한 것이 아니라, 후보 출처를 구분해 기록하는 incumbent 보존 방식이다.

- 개선 전 실행 파일: `/build/el_collect_baseline_20261029`. 소스 보존 위치: `runs/teacher-improvement/baseline-source`.
- 개발 비교: Race 2조건 × 2랩, Avoid 고정 장애물 6조건 × 1랩. `development-comparison.json`.
- 별도 회귀 비교: Race 3조건 × 2랩, Avoid 고정 장애물 6조건 × 1랩. 동일한 시작점, 난수 시드, 속도 상한, 장애물 좌표·크기를 사용한다. `el_compare_teachers`가 두 JSON의 조건·장애물 좌표가 정확히 같은지 검사한다.
- 순환 회귀: 이전 jitter 수집의 8개 시드와 시작점·속도 상한·배치 규칙을 재사용한다. 이후 배치 위치는 차량의 진행 상태와 가시성에 의존하므로 고정 장애물 비교와 달리 전후 모든 장애물 좌표가 일치하지 않는다. 순환 결과는 별도 end-to-end 회귀로 해석한다.
- 랩타임: 0.05초 주기의 simulator-time progress 로그에서 한 바퀴 경계 통과 시각을 보간한다. 정지 출발 첫 랩을 포함하고 미완료 랩을 제외한다. 실제 길이 약 41.9037 m, unique waypoints 420개다.
- 실제 충돌은 시뮬레이터 판정과 독립적인 차량/장애물 oriented-box 겹침 검사로 확인한다. 기록된 무충돌은 모든 가능한 배치의 안전 보장을 뜻하지 않는다.

`metrics_only=true`인 개선 시험에서는 ELDS 파일을 생성하지 않는다. summary의 예상 `path`를 학습 목록에 추가하면 안 된다. 개선 전 바이너리는 해당 옵션이 없으므로 baseline 폴더에 ELDS가 있을 수 있지만 기존 학습 목록에는 포함하지 않는다.

## 회귀 검사

CTest는 기존 CUDA MPPI·기억 지도·데이터 무결성 검사에 더해 다음을 검사한다.

- 유효 nominal보다 나쁜 비용의 weighted update를 채택하지 않는지.
- effective sample size가 1부터 sample count 사이인지.
- world-grid 충돌 판정이 같은 물리 위치를 다른 ego 좌표계로 표현해도 동일한지.
- CUDA가 점유된 world grid에서 경로를 거부하는지.
- 축소된 LiDAR 표본의 실제 각도로 관측 셀을 기록하는지.
- TLN은 forward-only 검사만 수행하고 backward·optimizer·학습 체크포인트를 만들지 않는지.

## 재실행

```bash
docker exec el-native-dev bash /workspace/scripts/build.sh
docker exec el-native-dev bash /workspace/scripts/collect_teachers.sh /workspace/config/teacher_holdout_v5.json
docker exec el-native-dev bash /workspace/scripts/collect_teachers.sh /workspace/config/teacher_cycling_v5.json
```

완료된 output은 덮어쓰지 않으므로 재실행할 때 JSON의 output을 새 경로로 바꾼다. 비교 도구는 `el_compare_teachers BASELINE_DIR IMPROVED_DIR OUTPUT.json`이다. 이전 단계 v3/v4 로그는 원인 분석 및 비교 근거로 보존한다.

교사 계산시간은 simulator, 지도 작성, 특징 생성 시간을 제외한다. 더 정밀한 충돌 검사와 여러 후보 검증으로 기존보다 계산량이 늘었다. 현재 PC에서 동기식 오프라인 수집에 사용하며, ROS 실시간 deadline이나 Orin Nano 지연시간을 검증한 것은 아니다. 최대 10 m/s 명령 설정이 10 m/s 실주행 검증을 뜻하지 않는다.
