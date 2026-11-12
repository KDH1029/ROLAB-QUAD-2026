# yolo — 신호등·콘 인식 (기반 기술)

> 구분: 기반 기술 · 시점: NUC16 `~/ws/datasets` 의 2026-11-01 ~ 11-06 상태 · git 이력 없음(소스 사본)
> 이 폴더에는 **학습·데이터 쪽**만 있다. 이미지·라벨·가중치(`.pt`)·zip 은 빠져 있다(용량, 데이터 출처).
> 로봇에서 도는 추론 코드는 각 미션 플래너의 `onboard_vision/` 에 있다(아래 4절).

## 1. 모델
| 모델 | 클래스 | 학습 데이터 | 비고 |
|---|---|---|---|
| `tl5` (YOLOv8n) | 0 tl_red · 1 tl_green · 2 pedestrian · 3 temi · 4 unknown | 학습 1,242장 = 원본 414 + 오프라인 증강 828(흔들림·밝게·어둡게·대비 낮춤), 검증 103장(박스 348) | 9월 데이터셋. 전부 1280×720 |
| `tl5cone` | 위 5개 + cone | tl5 + Roboflow `cone_1`(CC BY 4.0, 6색 → cone 1클래스) | 실내 콘 사진이 없는 게 약점 |
| `tl_cone_1104` | 0 빨강 · 1 초록 · 2 unknown · 3 콘 | 11-04 경기장 사진 1,500장 포함 | 대회 기본 모델. OpenVINO IR 로만 남아 ONNX 로 복원해 씀 |

**클래스 번호는 모델마다 다르다.** 추론 노드는 출력 모양(클래스 수)으로 콘 번호를 자동 판별한다(6클래스면 5, 4클래스면 3).
예전에 노드가 9/0/1 을 기대하고 모델은 0/1/2 를 내서 신호등 인식이 0% 였던 적이 있다.

## 2. 폴더 구조
| 경로 | 내용 |
|---|---|
| `tl5/HANDOFF_yolo_training.md` | 재학습 인수인계: 데이터셋 무결성, 학습 명령, 넘어야 할 기준표, 결과물 형식 |
| `tl5/colab_train_tl5.ipynb`, `colab_train_tl5_cone.ipynb` | Colab 학습 노트북(신호등 5클래스 / +콘 6클래스) |
| `tl5/ds_aug/data.yaml`, `data_nuc.yaml`, `classes.txt` | 데이터셋 정의. **`path:` 가 상대경로라 그대로 쓰면 "images not found"** — 절대경로로 바꿀 것 |
| `tl5/ds_tl5_cone/data.yaml`, `CONE_SOURCE_README.roboflow.txt` | 콘 합친 데이터셋 정의, 콘 데이터 출처·라이선스 |
| `tl5/runs/tl5_640_aug/`, `tl5cone_640/` | NUC CPU 학습의 설정(`args.yaml`)과 에폭별 지표(`results.csv`) |
| `arena1104/tools/extract_frames.py`, `select_frames.py`, `make_dataset.sh` | 경기장 녹화(bag)에서 학습용 프레임 뽑기 → 고르기 → 데이터셋 만들기 |

## 3. 학습
```bash
unzip ds_aug.zip
sed -i "s|^path: .*|path: $PWD/ds_aug|" ds_aug/data.yaml
yolo detect train data=ds_aug/data.yaml model=yolov8n.pt imgsz=1280 epochs=100 patience=20 batch=16
yolo detect val   model=runs/detect/train/weights/best.pt data=ds_aug/data.yaml imgsz=1280   # 클래스별 표 확인
yolo export model=best.pt format=onnx imgsz=1280 opset=12 simplify=True dynamic=False       # 내부 PC TensorRT 8.5 호환
```
- **해상도 1280 을 쓸 것**: 신호등이 작다. 9월 실측 mAP50 은 640 이 0.921, 1280 이 0.939 였다.
- **CPU 학습은 비현실적**(NUC 에서 1280·100에폭 약 27시간). GPU 나 Colab 을 쓴다.
- **가장 중요한 숫자는 초록불 정밀도(P)**: 초록 오판 = 빨간불에 출발이다. 9월 목표(1280 + 증강)는 초록 P 0.981 이었다.
- NUC 재학습 결과(640, CPU): `tl5_640_aug` 최고 mAP50 0.941(17에폭), `tl5cone_640` 0.936(9에폭).
  초록 P 0.876 으로 9월보다 낮아, 그대로 쓰기는 위험하다고 판단했다.

## 4. 로봇에 올리기 (내부 PC = Jetson Orin NX, Foxy, JetPack 5)
추론 노드와 배포 스크립트: `code/juninsu/onboard_vision/`(본선 판), `code/junsu/onboard_vision/`(+ 모델 복원·카메라 콘 도구)
```bash
onboard_vision/deploy_onboard_vision.sh fp16     # 올리기 → TensorRT FP16 엔진 → 채점
onboard_vision/deploy_onboard_vision.sh start    # 카메라 + 신호등 노드 켜기
onboard_vision/deploy_onboard_vision.sh bench    # 실제 카메라로 15초 추론(러너와 안 섞이는 토픽)
TL_MODEL=tl5cone onboard_vision/deploy_onboard_vision.sh start    # 모델 바꾸기(기본 tl_cone_1104)
```
- **FP16 을 기본으로**: 314장 채점에서 FP16 은 원본과 100% 일치(9.5 ms/장), INT8 은 초록 재현율 71.1%(원본 84.2%)로 떨어졌다.
- **엔진은 장치마다 새로 만든다.** 다른 젯슨에서 만든 엔진은 못 쓴다.
- **엔진 만들기는 로봇이 엎드려 있을 때만** 한다. 내부 PC 는 로봇 펌웨어도 도는 PC 라, 스크립트가 `/lowstate` 가 300 Hz 아래로 두 번 떨어지면 멈춘다.
- **결과 전달**: Foxy ↔ Jazzy 사이에서 `vision_msgs` 가 깨질 수 있어 `std_msgs/String`(JSON)을 쓴다.
- **TensorRT 8.5 + numpy 1.24**: `trt.nptype` 이 `np.bool` 을 써서 엔진 로드가 죽는다 → `trt_engine.py` 의 `np.bool = bool` 보강을 유지할 것.
- **전처리는 letterbox 로 맞출 것**: 맞추자 ultralytics 결과와의 일치가 43 → 100% 가 됐다.
- 시스템 OpenCV 4.6 은 YOLOv8 ONNX 를 못 읽는다. 로더는 ONNX Runtime → OpenVINO → OpenCV 순으로 시도한다.

### OpenVINO IR 로만 남은 모델 되살리기 (`code/junsu/onboard_vision/models_1104/`)
1. `ov_dump.py`: IR 상수에서 합성곱 가중치를 꺼낸다(BN 합쳐짐, 이름 = 모듈 경로)
2. `rebuild.py`: ultralytics yolov8n(nc=80 으로 만들고 cv3 끝만 4로, 머리 폭 c3=80)에 넣는다
3. `export_onnx.py`: ONNX 로 내보낸다

원본 IR 과 상자 707/707 이 일치했고, 점수 차는 0이었다.

## 5. 재사용할 때
- 데이터셋 `path:` 를 절대경로로 바꾼다. 원본 zip 위치는 `tl5/HANDOFF_yolo_training.md` 2절에 있다.
- 새 경기장이면 `arena1104/tools/` 로 그 경기장 녹화에서 프레임을 뽑아 섞는다. 실제 신호등 사진이 없으면 합성 그림으로 학습한 모델은 색 정답이 16/53 수준이었다.
- 신호 판정 규칙(초록 몇 초 연속, 빨강 대기)은 모델이 아니라 미션 플래너 러너에 있다 → [`../juninsu/README.md`](../juninsu/README.md) 3절
