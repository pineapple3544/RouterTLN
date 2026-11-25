# Berlin 정방향 v1.5 신뢰성 평가 재현

기존 300개와 사전 추첨한 신규 600개를 각각 180초씩 평가했다. 기존 300개에는 수정 과정에서 확인한 배치가 포함된다. 신규 600개는 수정본을 결정한 뒤 한 번씩 평가했다. 각 배치는 출발 시 장애물 기억과 랩 계획을 초기화한다. 아래 두 매니페스트에는 주행 순서와 출발점, 장애물 좌표를 정하는 종방향 거리와 횡방향 오프셋, 센서 노이즈 시드, 속도·라우터 설정이 모두 들어 있다.

| 평가군 | 장애물 1·3·5개 시도 수 | 무정지·무충돌 | 첫 랩 완주 | 접촉 | 입력 및 결과 |
|---|---|---:|---:|---:|---|
| 기존 배치 재검증 | 각각 100회 | 299/300 | 300/300 | 0 | [입력](paired300/manifest.json) · [배치별 결과](paired300/cases.csv) · [원시 주행 결과](paired300/runs.jsonl.gz) |
| 신규 무작위 배치 | 각각 200회 | 582/600 | 595/600 | 0 | [입력](fresh600/manifest.json) · [배치별 결과](fresh600/cases.csv) · [원시 주행 결과](fresh600/runs.jsonl.gz) |

[장애물 수별 랩타임·회귀·실패 원인 보고서](report.md) · [기존 실패 목록](paired300/failures.csv) · [신규 실패 목록](fresh600/failures.csv)

## 시드와 배치

기존 300개의 마스터 시드는 `126131944439733618`, 신규 600개의 마스터 시드는 `1975547728446134052`다. 매니페스트의 `seed`는 개별 주행의 센서 시드이고, 물리 배치는 같은 항목의 `stations`와 `offsets`가 결정한다. 장애물은 0.30 × 0.30m이며 출발점에서 5.5~44.9m 사이에 놓인다. 같은 배치의 장애물 사이 종방향 거리는 최소 1.5m, 횡방향 오프셋은 ±0.18m다. 재현 명령은 저장된 배치를 그대로 읽는다. [기존 추첨 자료](paired300/draw-protocol.json) · [신규 추첨 자료](draw.json) · [과거 배치 중복 검사](novelty-audit.json)

## 다른 PC에서 실행

Linux x86-64 또는 Linux 컨테이너 모드의 Docker Desktop에서 저장소 루트를 기준으로 실행한다. 최초 빌드에는 인터넷이 필요하다. Compose는 고정된 ROS 2 Humble 베이스, CPU용 LibTorch, 저장소의 Gym과 모델을 사용한다. [휴대용 Dockerfile](../../../../docker/v1.5.Dockerfile) · [Compose 설정](../../../../compose.yaml)

```bash
git clone --branch v1.5 --single-branch https://github.com/pineapple3544/RouterTLN.git
cd RouterTLN
docker compose build berlin
docker compose run --rm berlin reliability paired300 --case holdout-5-099
docker compose run --rm berlin reliability fresh600 --case fresh-1-188
```

게시용 Dockerfile로 빌드한 이미지에서 위 두 배치를 다시 주행했다. 첫 예시는 180초에 9랩을 완주했고, 둘째는 첫 랩에 동적 제동 보호로 정지했다. 두 배치 모두 원본 기록과 완주 랩별 시간, 종료 시각, 최초 보호 판정, 접촉 여부가 일치했다. 전체 평가에는 다음 명령을 사용한다.

```bash
docker compose run --rm berlin reliability paired300
docker compose run --rm berlin reliability fresh600
```

기본값은 작업자 1개다. 기록된 시험은 CPU 작업자 8개에 매니페스트 순서대로 배치를 순환 배분하고 작업자마다 코어 하나를 지정했다. 컨테이너에서 CPU 0, 2, 4, 6, 8, 10, 12, 14를 사용할 수 있으면 두 명령 끝에 `--cpus 0,2,4,6,8,10,12,14`를 붙여 같은 배분으로 실행할 수 있다. 사용 가능한 CPU 목록은 `docker compose run --rm --entrypoint python3 berlin -c "import os; print(sorted(os.sched_getaffinity(0)))"`으로 확인한다.

매번 호스트의 `results/reliability-*/` 아래 새 폴더에 실행 명세, 작업자별 `rows-*.jsonl`, 제어 주기별 `traces/*.csv`, 통과·접촉 집계 `totals.json`이 생긴다. 명령은 요청한 평가가 정상적으로 끝나면 종료 코드 0을 반환한다. 주행 통과 여부는 집계에서 확인한다. [평가 진입점](../../../../scripts/berlin_v15.py) · [C++ 평가기](../../../../src/eval_avoid.cpp)

## 판정과 실행 환경

180초 종료, 한 바퀴 이상 완주, 접촉 0건, 보호 거부 0건, 출발 후 0.5m부터 최저 속도 0.05m/s 이상을 모두 충족해야 통과다. LiDAR와 Berlin 정적 지도로 위치를 추정하며, 주행 정책에 시뮬레이터의 장애물 좌표나 실제 자세를 전달하지 않는다. 가중치와 정상 주행 속도 범위 2.7~4.5m/s, ROS 센서 타임아웃 150ms를 유지했다. 이 표는 오프라인 결과이며, 제어 시간은 CPU와 동시 작업에 따라 달라진다.

기록을 낸 이미지는 기존 로컬 이미지에 수정 헤더를 복사해 재빌드했다. 당시 사용한 [증분 Dockerfile](Dockerfile.incremental), [빌드·CTest 로그](build-and-tests.log), [이미지·평가기·가중치·매니페스트 SHA-256](evaluation-protocol.json)을 보존했다. 다른 PC에서는 위의 Compose 빌드로 동일 소스와 가중치를 설치한다. 공개용 이미지에는 재현 명령과 매니페스트도 들어가므로 이미지 ID가 달라진다.
