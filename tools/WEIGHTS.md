# 체크포인트 가중치 확인

저장소 루트에서 Python용 PyTorch를 설치한 환경을 사용한다. 주행 프로그램의 LibTorch와 별개다. 기존 Windows 시험에서는 Python 3.14.3과 PyTorch 2.12.0+cpu를 사용했다.

```bash
python tools/read_weights.py
python tools/read_weights.py models/race-teachers-expanded-v5.pt --tensor head.4.weight --row 0
python tools/read_weights.py models/avoid-corridor-v2.pt --tensor head.4.bias
```

기본 체크포인트는 원본 Avoid다. 텐서를 지정하지 않으면 스키마와 크기를, 지정하면 값을 JSON으로 출력한다. 마지막 층의 0행은 조향, 1행은 속도 출력에 해당한다. [가중치 편집 방법과 주행 검증](../README.md)

