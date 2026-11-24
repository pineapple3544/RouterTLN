# RouterTLN v2 / 첫 시뮬레이터 시험판

기준 main의 모델과 지도, 위치 추정, 점유 기억, 랩 계획, 회피 격자 탐색, 물리 채점기를 유지한다. 기본 실행은 v1이며 `hierarchical_v2=true`가 선택형 v2다. 실제 차량에서는 이 옵션을 거절한다. 기존 3랩 및 180초 결과는 v1의 성적이며 v2의 실측 성적이 아니다.

## 구조

- Router / Planner: 기존 Hard Routing, 기억한 장애물과 현재 스캔 기반 전환, 공통 `DrivingPlan`(전문가, Race/Prepare/Avoid/Merge 단계, 로컬 경로 특징, 목표 및 안전 속도 상한)을 만든다. Prepare 단계에서는 아직 Race가 기준 경로를 따라가며 장애물 접근 상한을 적용한다. 회피 경로가 만들어지면 실제 회피 경로의 곡률 제약으로 교체한다. 두 전문가는 첫 랩 기록과 정적 지도에서 계산한 동일 순환 속도 프로파일(곡률 / 가속 / 제동 제약)을 상한으로 공유한다. 이 프로파일에는 기존 회피 구간 전체에 대한 고정 속도 상한을 넣지 않는다. 관측한 장애물에 대한 진입 거리는 기존 거리와 제동 여유 중 큰 값을 사용한다. 기억만으로 Avoid 전환을 강제하지 않는다. Merge는 Avoid를 유지하며 기존 복귀 조건에 추정 속도/조향/요레이트와 다음 Race 곡률 조건을 추가한다.
- TLN: 기존 schema 3 Race와 schema 4 Avoid 입력을 그대로 사용한다. 새로운 계획의 경로 특징은 기존 context에 들어가며 추가 속도/단계 특징은 아직 학습 모델의 텐서에 추가하지 않았다. 새 스키마의 학습은 후속 작업이다. Race의 기존 가중치와 nominal speed scale 유지. Avoid는 예측 속도를 선택형 scale로 보정할 수 있지만 강제 양의 하한은 없다. 공통 상한과 가속 상승 제한은 모델 전환에도 유지한다.
- Safety Guard: 기존 운동학/동역학 명령 및 완전 제동 궤적 검사를 분리해 그대로 재사용한다. Avoid의 정상 명령은 단일 후보의 짧은 경로 추종 예측과 기존 guard를 통과하면 유지한다. 실패하면 기존 preview_steer와 저속 대체 조향을 검사한다. 모든 후보가 거부되면 기존 방식으로 정지한다. 25ms 제어 기한 및 150ms 센서 보호는 그대로다.
- 첫 성공 정찰 랩까지 기존 제어를 사용한다. 새 계획은 이후 랩에서만 작동한다. v1의 2.7~3.0m/s 스트레스 밴드 및 Race 고속 실험 옵션과 v2를 혼용하지 않는다. v2 상한은 최초 시험에서 5m/s 이하로 제한한다.

## 빌드 및 단위 계약 시험

기존 Forza/ROS Humble 개발 컨테이너에서 `bash scripts/build.sh`를 실행한다. 새 CTest `hierarchical_control_contract`는 단계 분리, 접근 상한, 인계 상태, 출력 제약, 안전한 정책 명령 유지, 위험시 탐색 및 전 후보 거부를 확인한다. 순수 C++ 테스트만 실행하려면 저장소 루트에서:

```bash
g++ -std=c++17 -Iinclude tests/v2_control_test.cpp -o /tmp/el_v2_control_test
/tmp/el_v2_control_test
```

## 오프라인 대응 실험

```bash
python3 experiments/router-v2/compare.py \
  experiments/berlin-endurance/scout-speed-cases.json \
  runs/router-v2-pilot --arms v1-original v2-original-always v2-original-first
```

원본이 빌드되고 같은 6개 배치에 대해 실행되는지 확인한 다음 WSP 후보를 포함한 6개 arm을 비교한다. WSP 체크포인트의 실제 C++ 로딩/추론과 주행 성능은 아직 확인하지 않았다. `--avoid-scale 1.0`이 기본이며, 다른 scale은 별도 실험으로 명시한다. 기존 v1 설정과 v2 전체 비교는 여러 요인이 바뀐다. v2 내부 always/first와 원본/WSP의 2x2 비교는 속도 계획을 고정하고 조향 재탐색의 기여를 분리한다. 평가 입력과 모든 결과를 보존하며 실패를 분모에서 빼지 않는다.

## ROS 첫 시험

기존 시뮬레이터 환경에서, legacy 속도 밴드 및 옵션을 해제한 별도 셸을 사용한다.

```bash
unset EL_SIM_AVOID_MIN EL_SIM_AVOID_MAX EL_SIM_SECTION_SPEED_PROFILE
unset EL_SIM_RACE_ONLY EL_RACE_LAP_SPEED EL_RACE_SIGHT_SPEED EL_RACE_EXIT_SPEED EL_STRAIGHT_BOOST
unset EL_UNIFIED_CHECKPOINT
export EL_MAP_DIR=/workspace/maps/berlin EL_START_INDEX=37
export EL_OBSTACLE_INDEX=289 EL_OBSTACLE_STRIDE=1 EL_OBSTACLE_OFFSETS='[0.1739]'
export EL_SCAN_NOISE_SEED=934488799 EL_LAP_PLANNING=true EL_SPEED_CAP=4.25
export EL_HIERARCHICAL_V2=true EL_V2_POLICY_FIRST=true EL_V2_AVOID_SPEED_SCALE=1.0
export EL_INFERENCE_DEVICE=cpu EL_RACE_DURATION_S=180 EL_STOP_AFTER_LAPS=0
export EL_AVOID_CHECKPOINT=/workspace/models/avoid-corridor-v2.pt
export EL_RACE_CHECKPOINT=/workspace/models/race-berlin-wsp.pt
bash scripts/smoke.sh 186 1 moe v2-pilot-original '' 934488799
```

`EL_V2_POLICY_FIRST=false`는 매 회피 주기 탐색하는 대조군이다. `EL_HIERARCHICAL_V2=false`는 기존 경로이며, v1 고속 기록과 정확히 비교할 때는 해당 기록의 legacy 옵션을 복원한다. WSP 후보는 Avoid 체크포인트 경로만 변경해 새 실행 태그로 시험한다.

## 판정과 남은 검증

ROS CSV에는 raw policy steer/speed, 후단 탐색 여부, 대체 명령 여부, 최종 명령, v2 단계/속도 상한이 추가된다. 오프라인 결과에는 Avoid 개입 횟수(전체 / 첫 랩 이후 별도), 고속 WSP 활성 가능 구간 횟수, 조향 수정량 합계, 고정 트랙 8분할 통과 시간이 추가된다. 공통 속도 프로파일은 레이싱 라인과 정찰 편차를 사용하는 보수적 첫 구현이며, 회피 궤적 자체의 전역 시간 최적화는 아직 수행하지 않는다. 단계만 빨리 바꿔서 Avoid 시간만 짧아진 경우와 전체 구간이 빨라진 경우를 구분한다. `v2_phase=-1`은 v1 또는 정찰 랩이다.

첫 시험의 통과 기준은 물리 충돌/정지/보호 거부 없이 180초를 완료하는 것. 이후 동일 설정의 기존 배치 대응 실험과 완전히 미사용인 장애물 1/3/5개 300배치, ROS 반복을 실시한다. 전체 제어 시간 P99/최대, 25ms 초과, 최소 여유 거리, 랩타임과 개입률을 비교한다. 고속 Avoid 재학습, 속도/상태 조건부 새 모델 스키마, Race에 계획 특징을 직접 입력하는 학습, 진입·복귀 공동 최적화는 이 시험 결과를 보고 진행한다. 현재 코드에 포함된 새 옵션은 아직 폐루프 주행 검증 전이다.
