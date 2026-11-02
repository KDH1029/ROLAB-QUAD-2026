# YOLO Weights Directory (`models/`)

이 디렉토리는 대회 미션 비전 인식을 위한 YOLO 모델 가중치 파일(`.onnx` 또는 `.pt`)을 배치하는 곳입니다.

## 지원되는 모델 형식
1. **ONNX 모델 (`.onnx`) [적극 권장]**:
   - `yolo_mission.onnx` (YOLOv5, YOLOv8, YOLOv11 등을 ONNX로 익스포트한 파일)
   - **장점**: PyTorch나 무거운 라이브러리 없이 우분투 기본 **OpenCV DNN(`cv2.dnn`)**으로 100% 네이티브 구동되어 가장 가볍고 호환성이 우수합니다.
2. **PyTorch 가중치 (`.pt`)**:
   - `yolo_mission.pt` (Ultralytics 기반 파일)
   - 파이썬 환경에 `ultralytics`가 설치되어 있을 때 자동으로 감지하여 실행됩니다.
3. **가중치 파일이 없을 때 (Mock 모드)**:
   - 모델 파일이 없거나 로드할 수 없을 경우, 프레임워크가 자동으로 **Mock(모의) 모드**로 동작하여 전체 자율주행 파이프라인(`mission_runner`)의 끊김 없는 테스트를 보장합니다.

## 클래스 정의 (3-Class)
`yolo_mission.onnx` 메타데이터 기준 (`config/yolo_params.yaml` 의 `class_*` 와 일치해야 함):
- `Class 0`: `traffic_light` (지상 1.3m 신호등)
- `Class 1`: `traffic_cone` (23cm 주황색 라바콘)
- `Class 2`: `parking_sign` (10cm 'P' 문자 및 QR 코드)

모델이 없거나 로드에 실패하면 노드는 에러로 종료합니다. Mock 은 `engine_type:=mock` 일 때만.
