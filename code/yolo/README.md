# yolo

신호등·콘 인식 모델(YOLOv8n) 학습 자료. 이미지·라벨·가중치는 미포함.
로봇 내부 PC 추론 코드는 [`juninsu/onboard_vision`](../juninsu/onboard_vision), [`junsu/onboard_vision`](../junsu/onboard_vision).

## Models

| 모델 | 클래스 | 데이터 |
|---|---|---|
| `tl5` | tl_red, tl_green, pedestrian, temi, unknown | 학습 1,242장(원본 414 + 증강 828), 검증 103장 |
| `tl5cone` | tl5 + cone | tl5 + Roboflow cone_1 (CC BY 4.0) |
| `tl_cone_1104` | red, green, unknown, cone | 경기장 사진 1,500장 포함 (대회 사용) |

## Structure

| 경로 | 내용 |
|---|---|
| `tl5/HANDOFF_yolo_training.md` | 데이터셋, 학습 명령, 기준 성능 |
| `tl5/colab_train_*.ipynb` | Colab 학습 노트북 |
| `tl5/ds_aug/`, `tl5/ds_tl5_cone/` | 데이터셋 정의 |
| `tl5/runs/*/` | 학습 설정, 에폭별 지표 |
| `arena1104/tools/` | 녹화에서 학습 프레임 추출 |

## Train

```bash
sed -i "s|^path: .*|path: $PWD/ds_aug|" ds_aug/data.yaml
yolo detect train data=ds_aug/data.yaml model=yolov8n.pt imgsz=1280 epochs=100 patience=20 batch=16
yolo export model=best.pt format=onnx imgsz=1280 opset=12 simplify=True dynamic=False
```

- `data.yaml`의 `path`는 절대 경로로 변경 필요
- 640보다 1280이 유리 (mAP50 0.921 → 0.939)

## Deploy (Jetson Orin NX)

```bash
onboard_vision/deploy_onboard_vision.sh fp16
onboard_vision/deploy_onboard_vision.sh start
TL_MODEL=tl5cone onboard_vision/deploy_onboard_vision.sh start
```

- TensorRT FP16 9.5 ms/장. INT8은 초록 재현율 저하(84% → 71%)로 미사용
- 엔진은 장치별로 생성, 로봇이 앉은 상태에서 빌드
- 결과는 `std_msgs/String`(JSON)으로 전송
- TensorRT 8.5 + numpy 1.24: `np.bool = bool` 보강 필요
- OpenVINO IR 모델 복원: `junsu/onboard_vision/models_1104/` (`ov_dump.py` → `rebuild.py` → `export_onnx.py`)
