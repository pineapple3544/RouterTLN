# Berlin 정방향 추가 배치 검증

각 배치에서 제어기와 시뮬레이터를 새로 시작해 2회씩 주행했다. 첫 랩은 계획 수집, 2·3랩은 그 주행에서 만든 계획 적용 구간이다. 목표 속도 하한은 계획 적용 랩의 Avoid에 대해 2.7 m/s, 센서 보호 기준은 150 ms다.

| 배치 | 회차 | 1랩 (초) | 2랩 (초) | 3랩 (초) | 결과 |
|---|---:|---:|---:|---:|---|
| mirror | 1 | 25.83 | 17.10 | 17.07 | 통과 |
| mirror | 2 | 25.98 | 17.12 | 17.10 | 통과 |
| tight | 1 | 24.95 | 16.65 | 16.58 | 통과 |
| tight | 2 | 24.88 | 16.65 | 16.60 | 통과 |
| same-side | 1 | 24.10 | 16.57 | 16.55 | 통과 |
| same-side | 2 | 24.08 | 16.57 | 16.53 | 통과 |
| mid-bends | 1 | 27.63 | 18.10 | 18.00 | 통과 |
| mid-bends | 2 | 27.95 | 18.25 | 18.19 | 통과 |
| random-39101 | 1 | 26.72 | 17.83 | 17.87 | 통과 |
| random-39101 | 2 | 26.97 | 17.93 | 17.90 | 통과 |
| random-39102 | 1 | 25.40 | 17.25 | 17.20 | 통과 |
| random-39102 | 2 | 25.55 | 17.25 | 17.20 | 통과 |
| random-39103 | 1 | 25.27 | — | — | 2랩 70.18 m (누적), braking_footprint |
| random-39103 | 2 | 25.40 | — | — | 2랩 70.28 m (누적), braking_footprint |
| random-39104 | 1 | 27.48 | 17.72 | 17.63 | 통과 |
| random-39104 | 2 | 24.05 | 16.52 | — | 3랩 122.56 m (누적), no_corridor |

각 시드의 배치 좌표 입력값은 [랜덤 배치 명세](random/layouts.json)에, 주행별 충돌·정지 판정과 누적 거리, 계획 생성 지점, 목표·실측 속도는 [결과 JSON](results.json)에 있다.

## 재현

개발 컨테이너의 /workspace에서 새 출력 디렉터리를 지정한다.

```sh
python3 runs/berlin-brake-merge/ros.py runs/fixed-recheck --layouts mirror tight same-side mid-bends --spread-cores
python3 runs/berlin-forward-holdout/random_layouts.py runs/random-recheck --spread-cores
```
