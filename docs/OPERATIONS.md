# 운용 가이드 (이전 README 내용)

> 포트폴리오용으로 메인 README 를 바꾸면서 옮긴 운용·튜닝 내용. 최신 실행 순서는 [PROGRESS_20261103.md](PROGRESS_20261103.md) 2절.


2026학년도 4족보행로봇 경진대회(`ROLAB-QUAD-2026`) 공식 ROS 2 워크스페이스입니다.  
Unitree Go2 Edu와 Hesai XT16 3D LiDAR 환경에서 보행 시 발생하는 착지 충격 지터(gait jitter)와 경사 지형(언덕형 교차로 등)에서의 바닥 오검출 문제를 기구학-관성 융합 필터와 반경 방향 경사면 분리 알고리즘으로 보완하도록 설계되었습니다.

- **진행 상황 및 구현 방법 (2026-11-03)**: [`docs/PROGRESS_20261103.md`](PROGRESS_20261103.md)
- **협업 및 형상 관리 규칙**: [`docs/GITHUB_RULES.md`](GITHUB_RULES.md)
- **시스템 아키텍처 블루프린트**: [`docs/Blueprint_Complete.md`](Blueprint_Complete.md) / [`docs/Blueprint_Complete.pdf`](Blueprint_Complete.pdf)
- **프로젝트 인수인계 요약서**: [`docs/HANDOVER_ROBOWORLD_2026.md`](HANDOVER_ROBOWORLD_2026.md)

---

## 📌 현재 개발 진행상황 (Current Status)

> **현재 단계**: 뼈대 로직 및 사전 모델 구축 완료, **실제 로봇 하드웨어 및 현장 트랙 검증 전 단계**

### ✅ 구현 및 반영 완료 항목
- [x] **1:1 CAD 트랙 맵 & 웨이포인트**: $6\text{m} \times 8\text{m}$ 도면 맵(`maps/roboworld_2026.pgm`) 및 12개 공식 웨이포인트([`waypoints_track.yaml`](../src/missions/mission_runner/config/waypoints_track.yaml)) 정합
- [x] **C++ Core 스택 (`go2_bridge`)**: 151Hz 오도메트리 TF, 12-DoF FK 접촉 평면 보정(`base_stab`), $18^\circ$ 경사로/$0.1\text{m}$ 장애물 분리 필터, Sport API 제어기 구축
- [x] **7단계 미션 FSM (`mission_runner`)**: 출발선(IN) → 경사로 → 정지선1(신호등 2초 정지/초록불 출발) → 슬라럼 → 정지선2 → 주차(5초 착석) → 도착선(OUT) 상태 머신 작성
- [x] **YOLOv8-Nano 비전 (`mission_vision`)**: 3개 클래스(0: 신호등, 1: 23cm 콘, 2: 10cm 주차 표지판) 사전 학습 및 `yolo_mission.onnx` 모델 배치 (OpenCV DNN 0-dependency 추론)
- [x] **통합 런처 및 유틸리티**: 원터치 실행기([`go2_mission_run.sh`](../scripts/go2_mission_run.sh)), 재학습 스크립트([`train_mission_yolo.py`](../scripts/train_mission_yolo.py)), 추론 테스트기([`test_yolo_inference.py`](../scripts/test_yolo_inference.py))

### 🔄 다음 작업 및 디버깅 예정 항목
- [ ] `./scripts/go2_mission_run.sh --sim` 기반 가상 시뮬레이션 스모크 테스트 (FSM 상태 천이 검증)
- [ ] 실물 로봇 센서 토픽 수신 확인 (`/camera/color/image_raw`, `/odom`, `/scan`)
- [ ] 실제 소품(신호등, 콘, 주차판) 비전 검출 실측 (필요 시 현장 사진 20~30장 촬영 후 5분 재학습)
- [ ] 12개 웨이포인트 트랙 1바퀴 실주행 디버깅

---

## 1. 시스템 아키텍처 및 핵심 기술

### 좌표계(TF) 구조
```text
map  ──(AMCL / Map Server)──>  odom  ──(151Hz Leg Odometry)──>  base_link  ──(FK + Gyro Filter)──>  base_stab
```

1. **다리 기구학 오도메트리 브릿지 (`odom_tf_node`)**
   - 사족보행 착지 충격($>2.5\text{g}$)이나 특징점이 부족한 복도에서 발산하기 쉬운 LIO(LiDAR-Inertial Odometry) 대신, Go2 내부의 151Hz 다리 기구학 오도메트리(`/utlidar/robot_odom`)를 `odom -> base_link` 변환에 사용합니다.
   - `ClockAligner`와 `MonotonicStamp`를 통해 로봇 내부 시계와 호스트 PC 간 타임스탬프 오프셋을 실시간 추정하고 역전 현상을 방지합니다.

2. **12관절 순기구학(FK) + 자이로 상보 필터 기반 수평 안정화 (`imu_bridge_node`)**
   - `/lowstate` 토픽의 12개 관절 각도($q_1, q_2, q_3$)와 4개 발바닥 접지 압력(`foot_force`)을 이용해 동체 기준 발끝 3차원 좌표를 계산합니다.
   - 접지된 발이 3개 이상일 때는 최소자승 평면 피팅(Least-Squares Plane Fitting), 2개(대각선 트롯 보행)일 때는 접지선 투영과 직전 각도 감쇠를 결합해 기구학적 지면 경사($\phi_{\text{kin}}, \theta_{\text{kin}}$)와 차고($h_{\text{kin}}$)를 추정합니다.
   - 자이로 각속도 적분치와 기구학/IMU 경사를 상보 필터($\alpha = 0.96$)로 결합하여 보행 진동이 제거된 수평 좌표계 `base_stab`을 생성합니다.
   - **11-03 기본값 꺼짐** (`use_kinematics_filter: false`): 10-30 녹화에서 트로트 중 대각선 발 2개만 닿아 기구학 기울기가
     출렁이고(base_stab 흔들림 IMU 대비 2.5배, 중력 대비 평균 2.6° 치우침), AMCL 재생 비교는 켜짐/꺼짐 동등. 코드는 유지.
   - 경사로(언덕) 진입 시 지면 경사와 중력 수평면 간 차이가 `max_kin_imu_diff_deg`를 초과하면 중력 기준으로 클램프하여 원거리 라이다 스캔의 수평을 유지합니다.

3. **3D 경사 지형(언덕 교차로) 대응 라이다 슬라이서 (`cloud_bridge_node`)**
   - Hesai XT16 포인트클라우드(`/lidar_points`)를 `/velodyne_points`로 변환함과 동시에 `base_stab` 기준으로 2D `/scan`을 생성합니다.
   - 중앙 교차로 등이 완만한 언덕 형태로 솟아 있는 지형에서 단순 높이 임계값($z > z_{\min}$)으로 자를 경우 오르막 바닥이 장애물로 오인되는 문제를 해결하기 위해, 방위각 섹터별 반경 빈(`radial_bin_m`)의 최하단 점들로 국소 지면 프로파일을 추정합니다.
   - 인접 빈 간 경사각이 `max_walkable_slope_deg`(기본 $18^\circ$) 이하인 완만한 경사면은 바닥으로 통과시키고, 국소 지면 대비 `obstacle_min_dz`(기본 $0.18\text{m}$) 이상 수직으로 솟은 벽/기둥만 `/scan`에 반영합니다.

4. (삭제 11-05) 저장소 안 매퍼(`mapper_node`) — 지도는 저장소 밖에서 만든다.

5. **C++ 20Hz 오도메트리 직결 주행기 (`odom_nav_node`) 및 모터 제어 브릿지 (`cmd_vel_to_sport_node`)**
   - `odom_nav_node`: Nav2 없이도 20Hz C++ 제어 루프(`ALIGN` $\rightarrow$ `CRUISE` $\rightarrow$ `YAW_LOCK`)와 전방 `/scan` 장애물 비상 정지 로직으로 목표 3D 좌표를 추종합니다.
   - `cmd_vel_to_sport_node`: `/cmd_vel` 명령을 Unitree Sport API(`/api/sport/request`, API ID `1008`)로 변환하며, 최소 유효 속도 데드밴드(`min_vx`, `min_vy`, `min_wz`), 가속도 제한, 데드맨 타임아웃(`0.5s`)을 적용합니다.
   - `scripts/go2_odom_nav.py`는 YAML/JSON 설정 파싱 후 `os.execvp`를 통해 프로세스를 C++ 바이너리(`odom_nav_node`)로 즉시 교체하므로 주행 중 Python 런타임 메모리가 상주하지 않습니다.

---

## 2. 디렉토리 구조

```text
ROLAB-QUAD-2026/
├── config/
│   ├── system_params.yaml           # 매핑/오도메트리 주행/센서 외부 파라미터 통합 설정
│   ├── waypoints_track.yaml         # 2026 로보월드 대회 트랙(8m x 6m) 웨이포인트 및 대기 시간 설정
│   └── targets.json                 # 저장된 웨이포인트 좌표 데이터
├── docs/
│   ├── GITHUB_RULES.md              # 팀 GitHub 브랜치·커밋·디렉토리 분리 규칙
│   ├── Blueprint_Complete.md        # 4족보행 자율주행 시스템 총괄 아키텍처 블루프린트
│   └── Blueprint_Complete.pdf
├── maps/
│   ├── roboworld_2026.yaml          # 대회 규격 치수(8m x 6m) 기반 정밀 2D 도면 맵
│   ├── roboworld_2026.pgm
│   ├── map_default.yaml             # 기본 2D 점유격자 메타데이터
│   ├── map_default.pgm              # 기본 2D 점유격자 이미지
│   └── map_default_elevation.npz    # 2.5D 지면 고도 및 경사도 데이터
├── scripts/
│   ├── go2_odom_run.sh              # [2단계] 오도메트리 및 기구학 브릿지 단독 실행 스크립트
│   ├── go2_nav_run.sh               # [통합] 브릿지 + AMCL + Nav2 자율주행 실행 스크립트
│   ├── go2_mission_run.sh           # [대회 미션] 비전 인식 + 7단계 미션 FSM 통합 실행 스크립트
│   ├── go2_odom_nav.py              # C++ odom_nav_node 실행용 경량 CLI 런처 (os.execvp)
│   ├── go2_goto.py                  # Nav2 목표 좌표 전송 및 웨이포인트 관리 CLI
│   └── sim_nav_test.py              # 오프라인 검증용 사족보행 기구학/라이다 시뮬레이터
└── src/
    ├── core/                        # [기반 기술 도메인]
    │   ├── go2_bridge/              # C++ 핵심 노드 패키지 (odom_tf, imu_bridge, cloud_bridge, cmd_vel_to_sport, odom_nav_node)
    │   ├── go2_localization/        # Map Server, AMCL 로컬라이제이션 패키지
    │   └── go2_navigation/          # Nav2 RPP 컨트롤러 및 플래너 설정 패키지
    ├── missions/                    # [대회 미션 제어 도메인 (C++)]
    │   ├── mission_vision/          # 신호등(traffic_light_node), 주차 표식(parking_marker_node), 23cm 라바콘(cone_detector_node)
    │   └── mission_runner/          # 7단계 대회 시퀀스 상태 머신 (mission_fsm_node)
    └── interfaces/                  # [공통 메시지 인터페이스]
        ├── unitree_api/             # Unitree Sport API 메시지 패키지
        └── unitree_go/              # Unitree LowState 메시지 패키지
```

---

## 3. 빌드 및 실행 방법

### 워크스페이스 빌드
```bash
git clone https://github.com/KDH1029/ROLAB-QUAD-2026.git
cd ROLAB-QUAD-2026
colcon build --symlink-install
source install/setup.bash
```

### A. 지도
저장소 안의 매핑 도구(go2_map_run.sh, go2_mapper.py, mapper_node, build_hybrid_map.py)는 11-05 삭제했다.
지도(.pgm/.yaml)는 저장소 밖에서 FAST-LIO 녹화 재생으로 만들어 `maps/` 에 둔다.
```bash
# 매핑 중 찍힌 라바콘 등 작은 물체 제거 (미리보기 → --write)
python3 scripts/go2_map_clean.py maps/arena.yaml
python3 scripts/go2_map_clean.py maps/arena.yaml --write
```

### B. 오도메트리 단독 주행 (Odometry-Direct Mode)
Nav2 전체 스택을 띄우지 않고 151Hz 기구학 오도메트리와 전방 `/scan` 장애물 감지만으로 목표 좌표를 주행할 때 사용합니다.
```bash
# 터미널 1: 오도메트리 & 기구학 브릿지 기동 (시뮬레이션은 --sim 추가)
./scripts/go2_odom_run.sh

# 터미널 2: 오도메트리 상태 확인 및 목표 좌표 이동
python3 scripts/go2_odom_nav.py status
python3 scripts/go2_odom_nav.py --x 2.0 --y 0.5 --yaw 0.0
python3 scripts/go2_odom_nav.py save wp_center
python3 scripts/go2_odom_nav.py go wp_center
```

### C. 사전 지도 기반 전체 자율주행 (AMCL + Nav2 Stack) 및 대회 미션 주행
```bash
# 1) Nav2 기반 자유 목표 주행
./scripts/go2_nav_run.sh --map ./maps/roboworld_2026.yaml
python3 scripts/go2_goto.py --x 3.6 --y 3.0 --yaw -140

# 2) 2026 로보월드 7단계 전체 미션(신호등 2초 정지, 슬라럼, 주차 5초 SIT, 완주) 자동 수행
./scripts/go2_mission_run.sh          # 실물 로봇 모드
./scripts/go2_mission_run.sh --sim    # 시뮬레이션 검증 모드
```

### D. SLAM (FAST-LIO) — 외부 빌드본 참조
FAST-LIO 는 이 저장소에 포함하지 않고, 팀이 원래 쓰던 빌드본을 underlay 로 참조한다 (원본 수정 금지).
- 실행 파일: `~/ws/lab/install/fast_lio` (소스 `~/ws/lab/go2_ws/src/fast_lio`), 경로는 `SLAM_UNDERLAY` 로 변경
- 설정: [`config/fastlio/fastlio_xt16_odometry.yaml`](../config/fastlio/fastlio_xt16_odometry.yaml) — 원본 `odometry.yaml` 포크, `FORK:` 3곳만 변경
- 라이다 입력은 반드시 XT16 `/lidar_points`. `/utlidar/cloud`(L1) 를 넣으면 2분 안에 km 단위로 발산한다 (bag 재생 실측)
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch go2_bridge bridge.launch.py &
ros2 run fast_lio fastlio_mapping --ros-args --params-file "$GO2_FASTLIO_CONFIG"
```

### E. 한 줄 실행 (lifecycle 스택)
```bash
# 0) 지도: maps/arena.yaml (저장소 밖에서 제작, 콘은 go2_map_clean.py 로 제거)
# 1) 주행 스택: 브리지 + FAST-LIO + AMCL(Omni, 수직성 스캔) + EGO-Planner + 모터 브리지
./scripts/go2_run.sh --map maps/arena.yaml --start "0 0 0"            # --no-motor 로 모터 없이
python3 scripts/go2_goto.py --x 2.0 --y 1.0 --yaw 0                    # 목표는 map 좌표
# 2) 보험: RANSAC 벽 추종 (지도·SLAM 불필요)
./scripts/go2_run.sh --mode patrol
ros2 topic pub --once /patrol/cmd std_msgs/msg/String "{data: START}"
```
- 모든 런타임 노드는 lifecycle 노드이고 `lifecycle_manager_go2` 가 순서대로 올린다(모터 브리지는 마지막).
- 소프트웨어 정지: `ros2 lifecycle set /go2_cmd_vel_to_sport deactivate` (StopMove 전송). Ctrl+C 도 StopMove.
- 파라미터: `go2_bridge/config/bridge_params.yaml`, `go2_localization/config/amcl_params.yaml`,
  `ego_planner/config/ego_planner_params.yaml`, `go2_wall_patrol/config/patrol_params.yaml`

### F. 라바콘 위치 인식 (카메라 + 라이다 퓨전)
```bash
# (선택) 외부파라미터 점검: 로봇을 세워 둔 구간이 있는 bag 으로 depth 와 LiDAR 를 맞대어 본다.
#   두 절반(장소별)의 결과가 0.5°/2 cm 안으로 일치할 때만 저장된다. 10-30 bag 은 불일치 → 기본값 유지.
python3 src/missions/mission_vision/tools/depth_lidar_align.py ~/bags/<bag> \
    [--write src/missions/mission_vision/config/camera_extrinsics.yaml]
# 주행 스택에 콘 추적 추가 (YOLO 는 로봇 PC 에서 /mission/cone_detections 발행)
./scripts/go2_run.sh --map maps/arena.yaml --cones
```
- 센서 싱크: 카메라 stamp 는 로봇 PC 시계(1970, 미동기). `cone_tracker_node` 가 스트림별로
  (수신시각 − stamp) 하위 5 % 로 시계 차이를 추정해 NUC 시계로 바꾸고, YOLO 영상 시각에 가장 가까운
  depth·라이다(±50 ms)를 골라 odom TF 로 영상 시각까지 이동 보정한다. 10초마다 `sync ...` 로그:
  시계 차이, 맞춘 시간차, YOLO 지연, 보정량, "최신 스캔을 썼다면 어긋났을 양", 스캔당 라이다 콘 후보 수.
- 좌표는 라이다가 정한다: 매 스캔에서 콘 모양 덩어리(폭 ≤0.30 m, 콘 높이 띠, 위는 비어 있고 주변 바닥보다
  3 cm 이상 뜬 것)를 찾고, 카메라는 그중 어느 것이 콘인지만 고른다(깊이 추정 반경 0.30 m + 0.05 m/m,
  깊이 없으면 방향 ±6°). 카메라 각도가 바뀌거나 흔들려도 결과 좌표는 라이다 값. 확정된 콘은 카메라가
  못 보는 거리(약 2.2 m 너머)에서도 라이다만으로 계속 갱신. 설정: `cone_tracker_params.yaml` refine/lidar_cone.
- 경로 계획: `ego_planner` 가 `/mission/cones`(콘 중심)를 받아 벽과 별개로 다룬다 —
  `ego_planner_params.yaml` 의 `cones:` inflate_radius(절대 접근 금지 반경), safe_dist(밀어내는 범위),
  weight(밀어내는 세기), max_range. 네 값 모두 `ros2 param set` 으로 실행 중 변경 가능.
- 카메라 각도를 바꾼 뒤: 로봇을 평지에 세우고 (앞 2 m 바닥이 보이게)
  `ros2 run mission_vision camera_floor_calib.py --write src/missions/mission_vision/config/camera_extrinsics.yaml`
  — 10초 동안 뎁스 바닥 + 발 4개(다리 기구학)로 숙인 각도·옆 기울기·높이를 재서 저장(백업 남김). 좌우 방향과
  x/y 는 유지. 움직였거나 바닥이 덜 보이면 저장 거부. `--write-params .../cone_tracker_params.yaml` 을 붙이면
  발 아래 바닥 높이로 `fusion.floor_z` 를 바꾸고 콘 높이 띠(`refine.z_min/z_max`)도 같이 옮긴다 — 카메라 각도가
  아니라 서는 높이에 따라 바뀌는 값(10-30 bag: −0.336 m, 이전 가정 −0.30). 10-30 bag: 30.7°/31.0°(두 정지 구간), 일부러 20°로 바꾼
  설정에서도 30.5° 복원.
- 콘 탐지 합성 시험: `ros2 run mission_vision test_cone_lidar` (55 cm 간격 콘 2개·기둥·벽·바닥 잡음).
- `cone_tracker_node`: YOLO 박스 + depth 로 대략 위치 → 그 근처 LiDAR 콘 덩어리 선택 → `map` 좌표 → 추적
  (0.3 m 묶기, 3회·LiDAR 2회 확정, 확정 후엔 LiDAR 만으로 갱신) → `~/go2_cones.yaml` 저장.
  출력 `/mission/cones`(PoseArray, EGO-Planner 입력), `/mission/cone_obstacles`(점 고리, 시각화용), `/mission/cone_markers`.
- 설정: `mission_vision/config/cone_tracker_params.yaml`, `camera_extrinsics.yaml`(`camera_floor_calib.py` 측정값).

---

## 4. 주요 튜닝 파라미터 요약

환경 및 지형 특성에 따라 [`config/system_params.yaml`](../config/system_params.yaml) 과 [`src/core/go2_bridge/config/bridge_params.yaml`](../src/core/go2_bridge/config/bridge_params.yaml) 에서 아래 파라미터를 조정합니다.

| 구분 | 파라미터 | 기본값 | 설명 |
| :--- | :--- | :---: | :--- |
| **경사 지형 스캔** | `max_walkable_slope_deg` | `18.0` | 오르막/내리막 바닥으로 허용할 최대 경사각 (도) |
| **경사 지형 스캔** | `obstacle_min_dz` | `0.18` | 국소 지면 프로파일 위로 장애물로 판정할 최소 돌출 높이 (m) |
| **경사 지형 스캔** | `radial_bin_m` | `0.35` | 반경 방향 지면 높이 추정 간격 (m) |
| **자세 안정화** | `stab_mode` | `"gravity"` | `"gravity"`(중력 수평 유지) 또는 `"terrain"`(경사면 평행 정렬) |
| **자세 안정화** | `filter_alpha` | `0.96` | 자이로 적분 가중치 ($0.94 \sim 0.98$) |
| **자세 안정화** | `max_kin_imu_diff_deg` | `6.0` | 언덕 주행 시 기구학 경사와 IMU 중력 경사 간 최대 허용 편차 (도) |
| **3D $\rightarrow$ 2D 매핑** | `local_floor_cell_m` | `0.50` | 국소 바닥 고도를 추정할 셀 크기 (m) |
| **3D $\rightarrow$ 2D 매핑** | `wall_z_min` | `0.22` | 국소 바닥 대비 벽으로 기록할 최소 높이 (m) |
| **주행 제어** | `max_vx`, `max_vy`, `max_wz` | `0.65`, `0.35`, `1.20` | 최대 전진/횡이동 속도(m/s) 및 회전 각속도(rad/s) |
| **주행 제어** | `min_vx`, `min_vy`, `min_wz` | `0.08`, `0.08`, `0.15` | 사족보행 모터 불감대 극복을 위한 최소 속도 클램프 |
| **주행 제어** | `obstacle_stop_dist_m` | `0.35` | 오도메트리 직결 주행 시 전방 비상 정지 거리 (m) |


### G. 경로 순차 주행 (`route_runner_node`)
- 지점은 현장에서 조종기로 세운 뒤 저장한다. 좌표는 AMCL map (FAST-LIO 아님 — 발산 시 수 km 로 튐).
  `python3 scripts/go2_route.py save <이름> --route maps/route_arena.yaml` 은 1.5초 정지(3 cm·2°), AMCL 표준편차
  (x·y 0.15 m, 방향 8°), 지도의 빈 칸·벽 0.10 m 이격을 모두 통과해야 저장하고, 아니면 이유를 출력한다.
- 순서는 파일의 `sequence` 를 직접 편집: `"<지점> [stopline|parking|finish] [tol=m] [vmax=m/s]"`. 같은 지점을 여러 번 써도 됨.
  예: `maps/route_example.yaml`. `go2_route.py list` 로 빠진 지점 확인.
- 실행: `go2_run.sh --map ... --route maps/route_arena.yaml` → 노드는 대기, `go2_route.py start [단계]` 로 출발.
  `stop` = 모터 브리지 비활성(StopMove), `resume` = 현재 단계를 처음부터 다시(정지선은 다시 정지, 주차는 다시 앉음).
  정비 시간 후에는 `start <단계>` 로 그 단계부터 (규정 2.5: 요청한 위치에서 재개).
- 단계마다 EGO 의 `goal_tol`·`max_vel` 을 먼저 바꾸고 목표를 보낸다. 통과 지점은 0.5 m 안에 들면 다음 목표(멈추지 않음).
- 정지선: 도착 → 정지 2.3초 → 1초 이내의 GREEN 에만 출발, 15초 동안 GREEN 이 없으면 진행(50점 포기, 완주 200점 유지).
- 주차: 도착·정지 → Sit(1009) 5.5초 → RiseSit(1010) 3초 → 다음 단계.
- FAST-LIO 감시: `/Odometry` 가 한 번에 0.5 m 넘게 튀거나 3 m/s 를 0.3초 넘게 보이면 정지(`lio.mode: stop|log|off`).
  계단 충격에도 튈 수 있으니 실기에서 기준을 정할 것.
- 상태: `/route/status` (2 Hz) 또는 `go2_route.py status`.
- 시험: `ros2 run mission_runner test_route_logic` (판단 로직, 모의 로봇). ROS 통합 시험(가짜 로봇)은 저장소 밖 시험 도구.
