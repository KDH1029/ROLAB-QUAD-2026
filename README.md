# ROLAB-QUAD-2026

Unitree Go2 EDU 사족보행 로봇이 경사로·계단·좁은 통로가 섞인 코스를 자율주행하기 위한 ROS 2 워크스페이스.
2026 로보월드 4족보행로봇 경진대회(출발 → 경사/계단 교차로 → 신호등 정지선 → 라바콘 슬라럼 → 주차 → 도착)용.

## 주요 기능

- **경사에 강한 2D 스캔**: 인접 LiDAR 링이 같은 수평 거리에 맞는 수직면만 장애물로 사용 (경사로·계단을 벽으로 오인하지 않음)
- **위치추정**: FAST-LIO(LiDAR-관성 오도메트리) + AMCL(2D 지도, Omni 이동 모델, 다리 오도메트리 입력으로 SLAM 과 독립)
- **매핑**: FAST-LIO 위치와 보정 점구름으로 2D 점유격자 생성, 궤적 기반 자동 수평
- **경로 생성**: EGO-Planner (A* + B-spline 최적화), 모든 파라미터 YAML 분리, 충돌 시 정지
- **라바콘 위치 인식**: YOLO + depth 로 대략 위치 → LiDAR 로 정밀 좌표 → 지도 좌표로 추적·저장
- **카메라-LiDAR 캘리브레이션**: 노트북 화면에 체커보드를 띄워 수행
- **안전**: 모든 런타임 노드가 lifecycle 노드 (비활성화 = 정지), 모터 브리지는 마지막에 기동, 모터 없는 시험 모드
- **보험 모드**: 지도·SLAM 없이 RANSAC 벽 직선 추종

## 시스템 구성

```
 XT16 LiDAR ──► cloud_bridge ──► 수직성 스캔 /scan ──┬──► AMCL ──► map 좌표 ──┐ 목표·장애물 변환
     │                                              └──► EGO-Planner ◄────────┘
     └────────► FAST-LIO ──► /Odometry ────────────────► EGO-Planner (국소 제어)
 Go2 /lowstate ──► imu_bridge (IMU + 다리 기구학 → 중력 기준 base_stab)
 Go2 다리 odom ──► odom_tf ──► AMCL 이동 입력
 D435i ──► YOLO ──► cone_tracker (depth → LiDAR) ──► /mission/cones, EGO 장애물
 EGO-Planner ──► cmd_vel_to_sport ──► Go2 Sport API
```

| 하드웨어 | |
|---|---|
| 로봇 | Unitree Go2 EDU |
| LiDAR | Hesai XT16 (16채널) |
| 카메라 | Intel RealSense D435i (로봇 내부 PC) |
| 연산 | Intel NUC 16 (Ubuntu 24.04, ROS 2 Jazzy) |

## 요구 사항

- ROS 2 Jazzy (Humble 에서도 빌드되도록 작성), CycloneDDS (`rmw_cyclonedds_cpp`)
- Nav2 (`nav2_amcl`, `nav2_map_server`, `nav2_lifecycle_manager`), `vision_msgs`, PCL (`pcl_ros`), Eigen3, Boost, yaml-cpp, OpenCV (Python)
- FAST-LIO, Livox 메시지, Hesai XT16 드라이버, Unitree 메시지는 저장소에 포함 (`src/external`, `src/interfaces`)

## 빌드

```bash
git clone https://github.com/kdh1029/rolab-quad-2026.git
cd rolab-quad-2026
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 사용법

로봇에 영향을 주는 명령은 직접 터미널에서 실행한다. Ctrl+C 로 전체 정지(StopMove 전송).

```bash
# 1. (선택) 카메라-LiDAR 외부파라미터 점검: 녹화 bag 에서 depth 와 LiDAR 를 맞대어 확인
python3 src/missions/mission_vision/tools/depth_lidar_align.py <bag>   # 일치할 때만 --write <yaml>

# 2. 매핑 (조종기로 주행, 모터 명령 없음)
./scripts/go2_map_run.sh --lio --name arena --no-default

# 3. 주행 스택
./scripts/go2_run.sh --map maps/arena.yaml --start "0 0 0"         # --cones: 라바콘 추적, --no-motor: 모터 없음
python3 scripts/go2_goto.py --x 2.0 --y 1.0 --yaw 0                 # 목표 (map 좌표)

# 4. 보험 모드
./scripts/go2_run.sh --mode patrol
ros2 topic pub --once /patrol/cmd std_msgs/msg/String "{data: START}"
```

- 소프트웨어 정지: `ros2 lifecycle set /go2_cmd_vel_to_sport deactivate`
- 녹화본 시험: `GO2_RUN_TEST_LO=<도메인>` / `GO2_MAP_TEST_LO=<도메인>` → 루프백 전용, 로봇과 섞이지 않음

## 설정 파일

| 파일 | 내용 |
|---|---|
| `src/core/go2_bridge/config/bridge_params.yaml` | 브리지, 스캔 방식(`scan_mode: vertical`), 모터 제한 |
| `src/core/go2_localization/config/amcl_params.yaml` | AMCL (Omni 이동 모델) |
| `src/core/ego_planner/config/ego_planner_params.yaml` | 경로 생성 (팽창·반발 거리, 최적화, 제어 이득) |
| `src/missions/mission_vision/config/cone_tracker_params.yaml` | 라바콘 융합·추적·저장 |
| `src/missions/mission_vision/config/camera_extrinsics.yaml` | 카메라·LiDAR 외부파라미터 (캘리브레이션 결과) |
| `src/fallback/go2_wall_patrol/config/patrol_params.yaml` | 보험 모드 |
| `config/system_params.yaml` | 매핑 (2D 변환, 자동 수평, 위치 소스) |
| `config/fastlio/fastlio_xt16_odometry.yaml` | FAST-LIO (XT16) |

## 저장소 구조

```
src/core/go2_bridge           LiDAR·IMU·odom·모터 브리지, 수직성 스캔, 매퍼
src/core/go2_localization     AMCL 설정
src/core/go2_navigation       통합 런치 (go2_stack.launch.py)
src/core/ego_planner          EGO-Planner, 오프라인 시험 도구 (test_plan_on_map)
src/missions/mission_vision   YOLO, cone_tracker, 캘리브레이션 도구
src/missions/mission_runner   미션 상태기
src/fallback/go2_wall_patrol  RANSAC 벽 추종 (보험)
src/interfaces                Unitree 메시지
src/external                  FAST-LIO, Livox 메시지, Hesai 드라이버 (포크, 출처: src/external/README.md)
scripts/                      go2_run.sh, go2_map_run.sh, go2_amcl_init.py, go2_goto.py
config/ · maps/ · docs/
```

## 검증 결과

녹화본(누리관 경사·계단 구간, 542초) 재생과 합성 데이터 기준. 실기·대회장 검증 전.

| 항목 | 이전 | 현재 |
|---|---|---|
| AMCL 위치 상실 (오차 > 0.5 m) | 306 s / 528 s | 0 s (2회 재현, 오차 중앙 약 8 cm) |
| AMCL 출발 위치 0.49 m·20° 오차 | 출발 후 30초 동안 틀림 | 정지 상태 10초 만에 6 cm |
| 지도 벽 두께 | 24.6 cm | 13.2 cm |
| 걸은 경로가 장애물로 찍힌 비율 | 99 % | 0 % |
| 경로 계획 성공률 / 벽 최소 이격 | 71~75 % / 0.00 m | 88 % / 0.23 m |
| 경사 10~25°·계단 가짜 벽 (합성) | 경사·계단에서 발생 | 0 |
| 라바콘 위치 오차 (합성) | — | 1.7 cm (카메라 3° 오차에도 동일) |
| 캘리브레이션 풀이 (합성, 잡음 포함) | — | 회전 0.36°, 이동 1.2 cm |

자세한 측정 방법과 구현: [docs/PROGRESS_20261103.md](docs/PROGRESS_20261103.md)

## 알려진 제한

- FAST-LIO 는 XT16 `/lidar_points` 와 CycloneDDS 에서만 안정적으로 확인됨. 높이(z) 드리프트 있음(9분 +0.83 m, 2D 주행에는 영향 없음)
- YOLO 모델은 합성 데이터 학습본 (실제 콘·신호등 데이터로 재학습 필요)
- `camera_extrinsics.yaml` 기본값은 미보정
- 보험 모드는 직사각형 경기장에서 검증 필요

## 문서

- [docs/PROGRESS_20261103.md](docs/PROGRESS_20261103.md) — 진행 상황, 구현 방법, 측정 결과
- [docs/OPERATIONS.md](docs/OPERATIONS.md) — 운용·튜닝 (이전 README)
- [docs/GITHUB_RULES.md](docs/GITHUB_RULES.md) — 협업 규칙
- [docs/Blueprint_Complete.md](docs/Blueprint_Complete.md) — 설계 블루프린트
- [docs/HANDOVER_ROBOWORLD_2026.md](docs/HANDOVER_ROBOWORLD_2026.md) — 인수인계

## 라이선스

Apache-2.0. 단, `src/interfaces`(Unitree, BSD 3-Clause)와 `src/external`(FAST-LIO GPL-2.0, Hesai BSD 등)은
각 패키지의 라이선스를 따른다 ([src/external/README.md](src/external/README.md)).
