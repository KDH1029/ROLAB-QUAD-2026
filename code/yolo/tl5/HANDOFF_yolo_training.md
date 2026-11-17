# 신호등 YOLOv8n 재학습 — 인수인계 (2026-11-01)

> 다른 PC 에서 이 파일만 읽고 이어갈 수 있게 썼다. 대회(2026 로보월드)는 **11-04 연습 / 11-05 예선 / 11-06 본선**.

## 1. 무엇을 하나
Go2 사족보행 로봇의 **신호등 인식 모델(YOLOv8n, 5클래스)을 다시 학습**한다.
9월(강남구 대회)에 학습·증강·INT8 경량화까지 했던 원래 가중치(`tl5_aug.pt`, `tl5_1280.pt`)를 지금 찾을 수 없어서,
같은 데이터셋으로 다시 만든다. 결과 모델은 나중에 **Go2 내부 PC(Jetson Orin NX)에서 TensorRT INT8 로 변환**해 돌린다(이 PC 에서 할 일 아님).

## 2. 가져갈 파일
| 파일 | 크기 | 위치 | 비고 |
|---|---|---|---|
| `ds_aug.zip` | 362,721,189 B | NUC `~/Downloads/` 또는 Colab 계정 구글 드라이브 | **학습에 필요한 것은 이것 하나** |
| `yolov8n.pt` | 6,549,796 B (sha256 앞 16자리 `f59b3d833e2ff32e`) | NUC `~/ws/datasets/tl5/weights/` | 공식 COCO 사전학습(github.com/ultralytics/assets). 새로 받아도 됨 |
| `tl_dataset.zip` | 378,451,637 B | NUC `~/Downloads/` | 09-11 원본 프레임 1,076장, **라벨 없음**. 학습엔 불필요(나중에 자동 라벨링용) |

## 3. 데이터셋 (`ds_aug/`, 11-01 NUC 에서 무결성 확인)
```
클래스      0 tl_red   1 tl_green   2 pedestrian   3 temi   4 unknown
학습        1,242장 = 원본 414 + 오프라인 증강 828 (흔들림 426 · 밝게 146 · 어둡게 132 · 대비낮춤 124)
            박스 4,122개 (pedestrian 1575, unknown 816, tl_red 699, temi 654, tl_green 378 ← 가장 적음)
검증        103장 (전부 원본), 박스 348개  ← 9월 성능을 잰 검증셋과 정확히 같다
해상도      전부 1280×720
라벨        누락 0 · 빈 라벨 0 · 형식 오류 0
```
**주의: `data.yaml` 의 `path: ds_aug` 가 상대경로**라 그대로 쓰면 "images not found" 로 실패한다. 절대경로로 고칠 것(아래 명령에 포함).
증강(흔들림·밝기·대비)은 이미 파일로 들어가 있다 — ultralytics 기본 온라인 증강만 쓰면 된다. 추가 블러 설정 불필요.

## 4. 학습 명령 (GPU PC 또는 Colab 권장)
```bash
unzip ds_aug.zip
sed -i "s|^path: .*|path: $PWD/ds_aug|" ds_aug/data.yaml
yolo detect train data=ds_aug/data.yaml model=yolov8n.pt imgsz=1280 epochs=100 patience=20 batch=16
```
- **해상도 1280 을 쓸 것.** 9월 실측: 640 은 mAP50 0.921, 1280 은 0.939 — 신호등이 작아서 해상도 덕을 본다.
- GPU 메모리가 모자라면 `batch=8`. CPU 만 있으면 1280 은 비현실적(NUC 실측 100에폭 27시간) → GPU/Colab 을 쓸 것.
- 9월 Colab 학습의 에폭 수는 기록이 없다. 100 + 조기종료(patience 20)면 충분.

## 5. 넘어야 할 기준 (같은 검증셋 103장·348박스, 9월 실측)
| 모델 | P | R | mAP50 | mAP50-95 | 초록불 P |
|---|---|---|---|---|---|
| 640, 19에폭 (노트북 CPU) | 0.871 | 0.901 | 0.921 | 0.656 | 0.805 |
| 1280 (Colab) | 0.911 | 0.908 | 0.939 | 0.688 | 0.850 |
| **1280 + 증강 (목표)** | **0.951** | 0.906 | **0.945** | 0.681 | **0.981** (R 0.895, mAP50 0.949) |

**가장 중요한 숫자는 초록불 Precision.** 초록불 오판 = 빨간불에 출발이다(규칙: 빨강·알수없음이면 정지, 초록일 때만 주행).
학습 후 반드시 클래스별 표를 뽑을 것: `yolo detect val model=runs/detect/train/weights/best.pt data=ds_aug/data.yaml imgsz=1280`

## 6. 결과물 — NUC 로 돌려보낼 것
```
best.pt          → 이름을 tl5_1280_aug_retrain.pt 로 바꿔 NUC ~/Downloads/ (또는 같은 구글 드라이브)
results.csv, args.yaml, confusion_matrix.png   → 같이
클래스별 검증표   → 위 5번 표와 나란히
```
가능하면 ONNX 도 같이 만들어 오면 좋다(내부 PC TensorRT 8.5 호환):
`yolo export model=best.pt format=onnx imgsz=1280 opset=12 simplify=True dynamic=False`

## 7. 그다음 (이 PC 에서 하지 않음 — 참고)
- **내부 PC(Go2 안 Jetson Orin NX 16GB, JetPack 5, ROS 2 Foxy, Python 3.8)** 에서 TensorRT 엔진을 만든다. 엔진은 장치마다 다시 만들어야 한다.
  9월 Orin Nano 실측: INT8 1280 추론 19.0 ms(FP32 31.5), FP32 와 검출 일치 98.6%, 5 Hz 에서 CPU 0.17코어·GPU 13%. 보정 사진 200장.
- 내부 PC 는 로봇 펌웨어가 도는 PC 다. **거기서 학습하지 말 것.** 추론도 최하 우선순위·코어 제한·5 Hz, NUC 가 `/lowstate`(500 Hz) 를 감시하다 흔들리면 끈다.
- 결과는 Foxy↔Jazzy 호환을 위해 `std_msgs/String`(JSON) 으로 NUC 에 보낸다.
- 신호등 판단 로직(시간 필터, 30초 주기 규칙)은 NUC `~/ws/lab_visual/src/traffic_light_detector.py`, 팀원 쪽 `~/ws/Rgosu/src/traffic_light_detector_node.py`.
- 카메라: D435i 는 아래로 32° 숙어 있어 0.42 m 위가 안 보인다 → **신호등은 Go2 앞머리 카메라(`/frontvideostream`, 1280×720 H.264)** 로 볼 것. 학습 사진도 그 시야와 비슷해야 한다.

## 8. 지금 NUC 상태 (예비)
- NUC 에서도 예비로 학습 중(11-01 20:0x 시작): 640 해상도, 60에폭, 최하 우선순위, 터미널 탭 c11.
  예상 종료 11-02 00:30 전후, 결과 `~/ws/datasets/tl5/runs/tl5_640_aug/weights/best.pt`. 다른 PC 결과가 더 좋으면 그쪽을 쓴다.
- 스크립트: `~/ws/v_gosu/tools/train_tl5.sh [해상도] [에폭]`

## 9. 하지 말 것
- NUC `~/venv/go2` 에 `pip install opencv-python` 금지 (시스템 cv2 만 GStreamer 지원 → Go2 앞카메라 수신이 깨진다).
- NUC `~/ws/lab`(팀 공용) 수정 금지.
- 검증셋(103장)을 학습에 섞지 말 것 — 9월 수치와 비교가 안 된다.

## 10. (11-02 추가) 라바콘까지 합친 6클래스 데이터셋
- `ds_tl5_cone.zip` (462 MB, NUC `~/Downloads/` · `~/ws/datasets/tl5/`), 노트북 `colab_train_tl5_cone.ipynb`
- 클래스: 0 tl_red · 1 tl_green · 2 pedestrian · 3 temi · 4 unknown · **5 cone** (기존 번호 그대로, 콘만 추가)
- 학습 2,142장(신호등 1,242 + 콘 900) / 검증 221장(신호등 103 + 콘 118) / 시험 93장(콘)
- 콘 원본: Roboflow `cone_1` v1 (CC BY 4.0), 6색 클래스를 `cone` 하나로 묶음. 야외 잔디·흙 운동장, 영상 약 10개에서 뽑은 사진
- 콘 사진 속 사람은 현재 NUC 모델로 자동 라벨(확신도 ≥0.5, 56박스) — 안 하면 보행자를 배경으로 배운다
- 검증은 두 가지로 볼 것: 전체 221장 + **신호등 103장만**(9월 수치와 같은 조건, 노트북 7번 셀이 둘 다 출력)
- 한계: 실내 경기장(회색 카펫·합판·형광등)의 콘 사진이 0장. 로봇 충전 후 실제 대회 콘을 놓고 앞머리 카메라로 2~3분 찍어 200장쯤 보충할 것
