# 보호 로직 실패 수정과 동일 조건 재검증

2026-10-29. Docker `el-native-dev`, RTX 5070 Ti, 기존 F110 Gym. **보호 주행은 기존 9/10에서 10/10조건 완주로 개선됐고, 30랩의 충돌·유효 계획 부재·비상 감속은 모두 0회다.** 같은 모델·시드·시작 자세·속도 제한·제어 설정으로 다시 실행했다. 안전 여유 4 cm를 유지했고 재학습하지 않았다.

## 원인

기존 실패는 `new-start-0`: hyhall12 정방향, seed 41000, start index 23, 횡오차 +0.18 m, yaw 오차 -0.12 rad, 상한 4 m/s다. 46.40초에 TLN 명령과 SMPPI가 모두 거부됐고 감속 한 주기 후 46.45초에 평가가 중단됐다.

실패 전 929개 LiDAR 프레임을 저장하고 기억 갱신을 재생했다. 문제는 기본 지도에서 비어 있는 **5 cm 셀 78017**, 좌하단 좌표 약 `(4.27, 4.82)`였다. 인접한 벽은 y=4.87에서 시작한다. 이 셀에 들어온 15개 반환점은 지도 벽에서 약 0.01–1.39 cm 떨어져 있었고, 해당 광선의 지도 벽 도달 거리보다 약 0.13–2.07 cm 짧았다. 시뮬레이터의 거리 잡음 표준편차는 1 cm다. 광선 각도 이산화도 있으므로 잔차 전체를 순수한 Gaussian 잡음이라고 단정하지 않는다.

기존 inverse sensor model은 이런 벽 반환점을 인접 빈 셀 전체의 장애물 증거로 누적했다. 46.40초에 점유 log-odds가 `0.5999999 > 0.5`가 되면서 차체의 4 cm 안전 여유와 겹쳤다. 실제 크기 차체와 기본 지도만의 검사는 통과했다. 따라서 이 실패의 직접 원인은 **벽 반환점의 잡음과 격자화가 만든 가짜 점유**였다.

근거: [원본 셀 추적](runs/memory-wall-fix-20261029/cell-trace.json), [수정 후 동일 스캔 추적](runs/memory-wall-fix-20261029/cell-trace-final.json). 추적 JSON의 `odds`는 진단을 위해 재구성한 **기존** 갱신값이고, `occupied`는 해당 실행의 실제 Memory 구현 결과다. 수정 후에도 진단용 기존 odds는 0.6이지만 계획 지도 점유는 0이다.

## 수정

`Memory::update`는 광선이 기본 지도 벽에 처음 진입하는 거리를 DDA로 계산한다. 기본 지도상 빈 셀에 끝난 반환점이 그 거리에서 **3 cm 이내**라면 알려진 벽의 관측으로 연결하고, 인접 빈 셀에 새 장애물 증거를 추가하지 않는다. 이 허용치는 현재 시뮬레이터 거리 잡음의 3σ에 해당한다.

- 가까운 벽까지의 단순 공간 거리로 장애물을 지우지 않는다. **같은 광선 방향의 첫 벽**과 비교한다.
- 벽 자체의 점유와 차체 4 cm 여유는 유지한다. 벽보다 충분히 앞선 반환점은 즉시 장애물로 등록한다.
- 애매한 벽 반환점을 이유로 이미 기억한 장애물을 직접 지우지 않는다. 가려진 장애물의 기억은 유지하며, 실제로 통과한 free-space 관측으로만 감소시킨다.
- 계획·보호 검사 지도와 **schema 3 TLN 입력 지도**를 분리한다. 계획 지도에는 위 보정을 적용하고, TLN의 점유·관측 여부·관측 나이 입력은 학습 당시 갱신 규칙 그대로 유지한다. 앞으로 정리된 지도를 TLN 입력으로 쓰려면 입력 버전 변경과 재학습이 필요하다.

마지막 분리는 실제 회귀 실패를 보고 추가했다. 첫 수정(v2)은 보정 지도를 TLN에도 넣었고, 보호 주행은 10/10 통과했으나 단독 주행 2개가 충돌했다(`heldout-balanced-race-03` 5.65초, `new-start-2` 8.39초). **이 버전은 채택하지 않았다.** 동일 가중치에서 입력 처리만 바뀐 영향이며, 최종 v3에서는 학습 입력을 복원해 기존 단독 궤적까지 일치하는 것을 확인했다. [v2 실패 기록](runs/race-closedloop-v2-memory-20261029/summary.json)도 보존한다.

## 동일 조건 전체 결과

[v3 실행 설정](config/race_closedloop_v3_memory.json)은 [v1 설정](config/race_closedloop_v1.json)과 출력 폴더를 제외하고 동일하다. 두 지도·양방향, test 초기조건 6개와 추가 시작조건 4개, 방식별 3바퀴다. LiDAR 18 m, 물리 0.01초/제어 0.05초, SMPPI 2,048 samples·horizon 40·2 iterations, 복귀 대기 0.3초를 유지했다. 추가 장애물은 없는 Race 시험이다.

| 방식 | 기존 → 수정 후 완주 | 수정 후 충돌 | 랩 수 | 평균 / 중앙값 / 최소 / 최대 랩타임 s |
|---|---:|---:|---:|---|
| Race TLN 단독 | 10/10 → **10/10** | 0 | 30 | 19.471 / 20.373 / 16.320 / 21.681 |
| TLN + SMPPI 보호 | 9/10 → **10/10** | 0 | 30 | 19.496 / 20.373 / 16.320 / 21.962 |
| SMPPI 단독 | 10/10 → **10/10** | 0 | 30 | 19.402 / 20.303 / 15.821 / 21.555 |

원래 실패했던 보호 조건은 **3랩: 20.496 / 20.539 / 20.357초**, fallback 0주기, 비상 감속 0주기로 완료했다. 전체 보호 주행에서는 후보 거부 4주기, 복귀 대기를 포함한 유효 SMPPI 실행 24/11,702주기(0.205%)가 있었다. 안전 검사를 비활성화해서 얻은 결과가 아니다. 기존 39 fallback 주기에는 실패 감속 1주기가 포함돼 있었으며, 최종 실행에는 없다.

검증은 출력 설정의 `completed` 플래그만 확인하지 않고 30개 시도 각각의 3랩·완주 사유·충돌·비상 감속을 검사했다. TLN 단독 10조건, **11,687행 × 14개 물리/명령 열**이 기존 CSV와 정확히 같았다. 검사 진단값(`candidate_safe`)과 계산시간은 비교에서 제외했다. 저장 CSV의 정밀도에서 일치한다는 뜻이다. 별도로 원본 929개 스캔의 지도 입력 **2,853,888개 float 값**은 기존 구현과 정확히 같았다.

| 제어 계산 구간 | 평균 ms | 중앙값 ms | p95 ms | 최대 ms |
|---|---:|---:|---:|---:|
| TLN 단독 | 2.935 | 2.874 | 3.631 | 7.988 |
| 보호 주행 | 3.019 | 2.908 | 3.752 | 22.198 |
| SMPPI 단독 | 21.820 | 21.517 | 23.925 | 94.562 |

기억 갱신·입력 생성·추론·안전 검사·필요 시 optimizer를 포함하며, simulator 계산과 ROS 전달은 제외한다. 동기식 PC 평가이며 SMPPI에는 50 ms 제어 주기보다 긴 표본도 있다. 실시간 deadline 만족이나 Orin 성능을 검증한 결과가 아니다. 단독 TLN 추론 평균은 0.711 ms다.

## 회귀 검사와 범위

CTest **4/4 통과**. 추가한 기억 검사는 반복된 벽 잡음, 같은 셀 안에서도 벽보다 4 cm 앞선 실제 장애물, 기존 장애물 보존, 가림 상태 기억 유지, 실제 free-space 관측으로 제거, 반대 방향 광선, 벽과 평행한 광선의 장애물 검출, schema 3 입력 보존, 초기화를 포함한다.

실제 장애물에 대한 추가 회귀로 기존 확장 수집의 `avoid-001/003/022/024`를 **원래 시드·조건 그대로** metrics-only 실행했다. 표준·긴 형태·넓은 형태·회전 형태, 두 지도·양방향을 포함한다. **4/4 완료, 12/12 장애물 통과, 충돌·유효 계획 부재·차체/장애물 겹침 0회**다. 모든 생성 이벤트의 보이는 LiDAR 빔 수 0과 최소 생성 거리 조건도 확인했다. 후속 생성 위치는 주행 진행에 따라 정해지므로 이전 실행과 모든 장애물 좌표까지 동일하다는 주장은 아니다.

| 조건 | 장애물 형태 | 통과 | 기록된 최소 separating-axis gap m |
|---|---|---:|---:|
| avoid-001 | standard | 3/3 | 0.167 |
| avoid-003 | long | 3/3 | 0.079 |
| avoid-022 | wide | 3/3 | 0.086 |
| avoid-024 | rotated | 3/3 | 0.085 |

이 수치는 평가기의 차체·장애물 분리 축 gap이며 일반적인 유클리드 최소 거리와 동일한 값은 아니다. 제어기는 장애물 좌표를 직접 받지 않고 LiDAR 기억을 사용한다. **SVG-MPPI의 기억 처리 회귀 검사**이고 Avoid TLN 검증이나 새 학습 데이터 수집은 아니다. 종료 직후 남는 1프레임 구간은 수집기 원래 규칙에 따라 `unfinished_encounter`로 분리되며, 전체 시도의 `attempts/*.json`은 모두 목표 통과 완료다. 기존 데이터 manifest에 추가하지 않았다. [검사 요약](runs/memory-wall-fix-20261029/obstacle-regression-checks.json), [실행 설정](config/memory_wall_obstacle_regression.json), [로그](runs/memory-wall-fix-20261029/obstacle-regression.log).

현재 3 cm 연관 규칙은 이 시뮬레이터의 지도·센서 정합성에 맞춘 것이다. **벽보다 3 cm 이내로 돌출된 새 장애물**은 벽 잡음과 구별하지 못할 수 있다. 4 cm 차체 여유를 유지하지만 이것이 모든 기하·모델 오차에서 안전을 보장하지는 않는다. 다른 센서 잡음, localization 오차, 신규 지도에는 재검증이 필요하다. ROS fallback 통합, Avoid TLN, 전체 MoE, 미관측 지도 일반화는 이번 검증에 포함하지 않는다.

## 재현 및 산출물

```bash
docker exec el-native-dev bash /workspace/scripts/build.sh
docker exec el-native-dev /build/el_memory_replay_test \
  /workspace/runs/memory-wall-fix-20261029/baseline/new-start-0-tln_smppi_guarded.scans \
  /home/misys/forza_ws/race_stack/stack_master/maps/hyhall12/hyhall12_orig.yaml
# 재실행 시 결과 보존을 위해 JSON의 output을 새 경로로 복사해 지정한다.
docker exec el-native-dev bash /workspace/scripts/evaluate_race.sh /workspace/config/race_closedloop_v3_memory.json
```

- [전체 결과](runs/race-closedloop-v3-memory-20261029/report.json), [조건별 결과](runs/race-closedloop-v3-memory-20261029/summary.json), [계산시간·완주 검사](runs/race-closedloop-v3-memory-20261029/checks-and-timing.json)
- [설정·모델 불변 및 궤적 비교](runs/race-closedloop-v3-memory-20261029/provenance.json), [소스 SHA256](runs/race-closedloop-v3-memory-20261029/source-sha256.json), 같은 폴더의 `source/` 스냅샷
- [입력 일치 검사](runs/memory-wall-fix-20261029/feature-parity.json), [최종 빌드·CTest 로그](runs/memory-wall-fix-20261029/final-tests.log)
- 변경 코드: [기억 처리](src/core.cpp), [인터페이스](include/el/core.hpp), [단위 회귀 검사](tests/tests.cpp), [스캔 재생 입력 검사](tests/memory_replay.cpp), [원인 추적기](src/memory_trace.cpp)

선택된 epoch 19 모델 SHA256: `F2D88F6F29589E636F7AB1B59F13632085998447D3FA4F7FBC6A0E7AEA38D925`. 원본 학습 데이터·분할·가중치를 변경하지 않았다.
