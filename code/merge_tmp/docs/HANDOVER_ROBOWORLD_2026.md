# 2026 로보월드 사족보행 프로젝트 인수인계서 (간략본)

> **현재 상태**: 코드 뼈대 및 기본 로직 작성 완료, **실물 로봇/트랙 검증 전 단계**

---

## 1. 프로젝트 기본 정보
- **로봇**: Unitree Go2 Edu (RealSense D435, Lidar)
- **환경**: Ubuntu 24.04, ROS 2 Jazzy
- **작업 공간**: `/home/kdh/go2_workspaces_backup_20260922/go2_nav_ws`
- **깃허브 저장소**: `https://github.com/KDH1029/ROLAB-QUAD-2026.git` (`origin/main`)

---

## 2. 지금까지 작업/수정한 파일 목록

### 1) 맵 & 트랙 좌표
- `maps/roboworld_2026.pgm, .yaml`: CAD 기준 6m x 8m 직교 맵
- `src/missions/mission_runner/config/waypoints_track.yaml`: 12개 미션 웨이포인트 좌표

### 2) 로봇 제어 C++ (`src/core/go2_bridge/`)
- `odom_tf_node.cpp`: 로봇 오도메트리 TF 발행
- `imu_bridge_node.cpp`: 흔들림 보정 수평 좌표계(`base_stab`) 생성
- `cloud_bridge_node.cpp`: 경사로(18도)/콘(0.1m) 지형 분리 필터
- `cmd_vel_to_sport_node.cpp`: 속도 명령을 Unitree Sport API로 변환
- `odom_nav_node.cpp`: 웨이포인트 닫힌루프 주행 제어

### 3) 미션 상태 머신 (`src/missions/mission_runner/`)
- `mission_fsm_node.cpp`: 출발 → 램프 → 정지선1(신호등) → 슬라럼 → 정지선2 → 주차(Sit) → 도착 7단계 시나리오

### 4) 비전 인식 (`src/missions/mission_vision/`)
- `models/yolo_mission.onnx`: YOLOv8-Nano 3클래스(0:신호등, 1:콘, 2:주차표지판) 임시 가중치
- `scripts/yolo_detector_node.py`: OpenCV DNN 기반 추론 노드 (pip 설치 없이 동작)
- `config/yolo_params.yaml`: 클래스 매핑 및 토픽 설정

### 5) 도구 스크립트 (`scripts/`)
- `go2_mission_run.sh`: 통합 실행 런처 (`--sim` 가상 모드 지원)
- `train_mission_yolo.py`: 데이터셋 생성 및 YOLOv8 재학습 스크립트
- `test_yolo_inference.py`: ONNX 추론 단독 테스트 스크립트

---

## 3. 현재 상태
- 모든 패키지 빌드 정상 (`colcon build` 0 에러)
- 합성 이미지 기반 비전 추론 테스트 정상
- **실제 로봇 구동 및 현장 센서 연동 테스트는 아직 진행되지 않음**

---

## 4. 다음에 할 일 (디버깅 순서)
1. **시뮬레이션 확인**: `./scripts/go2_mission_run.sh --sim` 실행하여 FSM 순서대로 도는지 확인
2. **실물 센서 확인**: 로봇 켜고 카메라 토픽(`/camera/color/image_raw`), 오도메트리(`/odom`) 잘 들어오는지 확인
3. **비전 실물 확인**: 실제 콘/신호등/표지판을 카메라 앞에 비춰보고 `/mission/yolo_debug_image` 확인
   - 인식이 약하면 실물 사진 20~30장 찍어서 `./scripts/train_mission_yolo.py` 실행
4. **트랙 주행**: 바닥에 로봇 내려놓고 12개 웨이포인트 1바퀴 완주 테스트
