# Race TLN 첫 학습 결과

후속 검증: [원래 폐루프 결과](SIMULATION_RACE.md), [보호 로직 수정 및 재검증](MEMORY_PROTECTION_FIX.md). Race 단독과 수정된 보호 주행 모두 두 지도 10조건·30랩을 무충돌 완주했다. 가중치는 변경하지 않았다.

2026-10-29 (Asia/Seoul). 사용자 재개 지시에 따라 Docker `el-native-dev`의 RTX 5070 Ti에서 C++ LibTorch 학습을 실행했다. **Race만 학습했으며 Avoid 학습과 MoE 통합 주행은 실행하지 않았다.**

## 결과

| 항목 | 결과 |
|---|---:|
| 학습 epoch | 20 |
| 최저 검증 손실 epoch | **19** |
| 최저 검증 Smooth L1 | **0.001000471601** |
| 마지막 20 epoch 검증 손실 | 0.001012768310 |
| 초기 무학습 모델 검증 손실 | 0.093901322253 |
| 최저 모델 검증 조향 MAE | 0.011929 rad (약 0.684°) |
| 최저 모델 검증 속도 MAE | 0.185020 m/s |
| 학습 루프와 종료 후 재검증 시간 | 약 21.00초 |

시간은 데이터 로딩·초기 평가·빌드를 제외한다. 손실은 조향 `steer/0.4`, 속도 `speed/5-1`의 정규화된 두 출력에 대한 평균 Smooth L1이다. 물리 단위 MAE는 검증 데이터에서 교사 명령과 예측 명령을 비교한 값이다. 차량의 실제 추종 오차나 폐루프 주행 성능이 아니다.

검증 손실이 엄격히 감소했을 때만 체크포인트를 저장한다. 동률이면 먼저 저장한 모델을 유지하고 NaN/Inf는 거부한다. 마지막 epoch가 더 나쁘면 덮어쓰지 않는다. 이번 실행에서는 19 epoch 모델을 실제로 다시 불러와 검증 손실이 기록된 최저값과 일치함을 확인했다.

## 데이터와 설정

- 데이터: 개선 교사 v5의 Race SMPPI 시연. hyhall12/teras 양방향.
- Train: 16개 주행, 12,773프레임.
- Validation: 6개 주행, 4,829프레임. 모델 선택에 사용.
- Test: 6개 주행, 4,661프레임. **관측·label을 로드하거나 평가하지 않았다.** 파일 경로와 메타데이터만 분할 중복 검사에 사용했다.
- Adam, learning rate 1e-4, batch 64, seed 2026, gradient norm clipping 1.0.
- 20 epochs를 고정 실행했으며 early stopping이나 hyperparameter 탐색은 수행하지 않았다.
- 기존 가중치 없는 새 TLN 계열 확장 CNN을 학습했다. LiDAR 18 m, 4시점 이력, 로컬 기억 지도, context 16개, schema 3을 유지한다.

## 파일

- [선택된 19 epoch 모델](models/race-teachers-expanded-v5.pt)
- [전체 epoch 손실 및 저장/재검증 기록](models/race-teachers-expanded-v5.pt.training.json)
- [실행 설정](runs/race-tln-v1-20261029/config.yaml)
- [학습 로그](runs/race-tln-v1-20261029/train.log)
- [빌드·CTest 로그](runs/race-tln-v1-20261029/build.log)
- [모델/기록 SHA256](runs/race-tln-v1-20261029/checkpoint-sha256.json)
- [학습 소스·설정 SHA256](runs/race-tln-v1-20261029/source-sha256.json)

`models/race-teachers-expanded-v5.pt`는 최저 검증 손실 모델이다. 마지막 epoch 모델은 별도 저장하지 않았다. 학습 재개에 필요한 optimizer 상태는 아직 저장하지 않으므로 이 파일을 완전한 학습 재개 체크포인트로 취급하지 않는다. 새 실행은 기존 모델/학습 기록을 덮어쓰지 않으며 새 출력 경로가 필요하다.

학습기 수정: `include/el/checkpoint.hpp`가 현재 GPU 모델과 optimizer를 이동시키지 않고 CPU 스냅샷을 저장한다. 임시 파일을 쓴 뒤 rename으로 교체한다. `src/train.cpp`는 train/validation/test 파일과 원본 시도의 분할 중복, 메타데이터 모드·상태·split을 검사하고, epoch별 손실과 물리 단위 검증 MAE를 JSON에 남긴다. test는 모델 선택에서 제외한다.

검사 4개가 통과했다. 새 체크포인트 회귀 검사는 검증 손실이 좋아졌다가 나빠지는 경우, 동률, NaN, 재로드한 실제 가중치와 기존 실행 덮어쓰기 거부를 검사한다. 나머지는 교사·native·forward-only 회귀 검사다.

## 현재 한계와 다음 단계

이 결과는 교사 행동의 오프라인 모방 정확도다. 학습 당시에는 시험/폐루프 평가를 하지 않았다. 이후 시험 초기조건 6개와 새 시작조건 4개의 폐루프 주행을 평가했으며, 결과는 `SIMULATION_RACE.md`에 기록했다. 시험 파일의 오프라인 모방 오차와 새로운 지도 일반화는 아직 측정하지 않았다. 장애물 없는 Race 시연으로 학습했으므로 이 모델 자체의 장애물 회피 능력을 가정하지 않는다.

후속 폐루프 검증에서 단독 정책과 fallback이 개입한 결과를 분리했다. 보호 경로의 한 조건에서 발생했던 벽 반환점의 가짜 점유를 수정해 동일 조건 10/10 완주·충돌 0을 확인했다. schema 3 학습 입력은 그대로 유지하며, 모델을 재학습하거나 교체하지 않았다. 수정과 입력 일치 검사는 [MEMORY_PROTECTION_FIX.md](MEMORY_PROTECTION_FIX.md)에 기록했다. Avoid 학습 및 전체 MoE 검증은 별도 단계다. 기존 데이터 수집 문서와 `audit.json`/`READY.json`의 `training_executed: false`는 수집 완료 당시의 기록이고, 현재 학습 상태는 이 문서와 모델의 `.training.json`이 기준이다.

