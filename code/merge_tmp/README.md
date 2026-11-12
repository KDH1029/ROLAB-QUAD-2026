# ROLAB-QUAD-2026

Unitree Go2 EDU 사족보행 로봇이 경사로·계단·좁은 통로가 섞인 코스를 자율주행하기 위한 ROS 2 워크스페이스.
2026 로보월드 4족보행로봇 경진대회(출발 → 경사/계단 교차로 → 신호등 정지선 → 라바콘 슬라럼 → 주차 → 도착)용.
대회 규정·QnA·경기장 배치도: [docs/rulebook/](docs/rulebook/)

## 주요 기능

- **경사에 강한 2D 스캔**: 인접 LiDAR 링이 같은 수평 거리에 맞는 수직면만 장애물로 사용 (경사로·계단을 벽으로 오인하지 않음)
- **위치추정**: FAST-LIO(LiDAR-관성 오도메트리) + AMCL(2D 지도, Omni 이동 모델, FAST-LIO 를 이동 입력으로 — 대회장 녹화 중앙 3 cm). 출발 정렬(`go2_start_align.py`)
- **지도**: 저장소 밖에서 만든 FAST-LIO 지도(.pgm/.yaml)를 쓰고, 매핑 중 찍힌 콘은 `go2_map_clean.py` 로 지운다
- **경로 생성**: EGO-Planner (A* + B-spline 최적화), 모든 파라미터 YAML 분리, 충돌 시 정지
- **라바콘 위치 인식 (LiDAR 만)**: 스캔마다 벽·다리 같은 키 큰 기둥을 먼저 빼고 콘 모양 덩어리를 찾아, 같은 지도 자리에서
  계속 보이면 콘으로 확정·저장. 확정된 콘은 가까이 가서 안 보여도 경로 계획에 남는다 (대회장 녹화 10/10, 가짜 0)
- **경로 계획의 콘 전용 여유**: 콘 중심 좌표를 벽과 별도로 — 절대 접근 금지 반경, 밀어내는 범위·세기를 YAML 로 관리
- **범위**: 이 저장소는 오도메트리·위치추정(AMCL)·경로 생성까지. 미션 제어(경로 순서, 정지선·신호등, 주차)와 비전은 팀원 담당
  — 팀원 쪽은 `/goal_pose`(map 좌표)로 목표를 주고, 정지는 모터 브리지 비활성(`go2_cmd_vel_to_sport`)으로
- **안전**: 모든 런타임 노드가 lifecycle 노드 (비활성화 = 정지), 모터 브리지는 마지막에 기동, 모터 없는 시험 모드
- **보험 모드**: 지도·SLAM 없이 RANSAC 벽 직선 추종

## 시스템 구성

```
 XT16 LiDAR ──► cloud_bridge ──► 수직성 스캔 /scan ──┬──► AMCL ──► map 좌표 ──┐ 목표·장애물 변환
     │                                              └──► EGO-Planner ◄────────┘
     └────────► FAST-LIO ──► /Odometry ────────────────► EGO-Planner (국소 제어)
 Go2 /lowstate ──► imu_bridge (IMU → 중력 기준 base_stab)
 FAST-LIO /Odometry ──► odom_tf ──► AMCL 이동 입력 (--leg-odom: Go2 다리 odom)
 XT16 ──► cone_tracker (콘 덩어리 → 지도 좌표 확정) ──► /mission/cones ──► EGO-Planner (콘 전용 여유)
 EGO-Planner ──► cmd_vel_to_sport ──► Go2 Sport API
```

| 하드웨어 | |
|---|---|
| 로봇 | Unitree Go2 EDU |
| LiDAR | Hesai XT16 (16채널) |
| 카메라 | Intel RealSense D435i (로봇 내부 PC, 팀원 비전용 — 이 저장소는 안 씀) |
| 연산 | Intel NUC 16 (Ubuntu 24.04, ROS 2 Jazzy) |

## 요구 사항

- ROS 2 Jazzy (Humble 에서도 빌드되도록 작성), CycloneDDS (`rmw_cyclonedds_cpp`)
- Nav2 (`nav2_amcl`, `nav2_map_server`, `nav2_lifecycle_manager`), PCL (`pcl_ros`), Eigen3, Boost, yaml-cpp, Python numpy·scipy·PyYAML (그림은 matplotlib)
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
# 1. 녹화 (조종기로 주행): ./scripts/go2_record.sh <이름> "<메모>"
# 지도는 이 저장소 밖에서 만든다 (FAST-LIO 녹화 재생 → .pgm/.yaml). 결과를 maps/ 에 두고, 콘이 찍혔으면
#   python3 scripts/go2_map_clean.py maps/arena.yaml --write   → maps/arena_clean.yaml

# 2. 주행 스택
./scripts/go2_run.sh --map maps/arena.yaml --start "0 0 0"         # --cones: 라바콘 추적, --no-motor: 모터 없음
python3 scripts/go2_goto.py --x 2.0 --y 1.0 --yaw 0                 # 목표 (map 좌표)

# 3. 보험 모드
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
| `src/core/ego_planner/config/ego_planner_params.yaml` | 경로 생성 (팽창·반발 거리, 최적화, 제어 이득, `cones:` 콘 전용 여유) |
| `src/missions/mission_vision/config/cone_tracker_params.yaml` | 라바콘 (LiDAR): 콘 판정, 콘 구역(`regions`), 확정·저장, LiDAR 장착 위치 |
| `src/fallback/go2_wall_patrol/config/patrol_params.yaml` | 보험 모드 |
| `config/system_params.yaml` | 매핑 (2D 변환, 자동 수평, 위치 소스) |
| `config/fastlio/fastlio_xt16_odometry.yaml` | FAST-LIO (XT16) |

## 저장소 구조

```
src/core/go2_bridge           LiDAR·IMU·odom·모터 브리지, 수직성 스캔, 매퍼
src/core/go2_localization     AMCL 설정
src/core/go2_navigation       통합 런치 (go2_stack.launch.py)
src/core/ego_planner          EGO-Planner, 오프라인 시험 도구 (test_plan_on_map)
src/missions/mission_vision   cone_tracker (LiDAR 콘), 콘 판정 합성 시험
src/fallback/go2_wall_patrol  RANSAC 벽 추종 (보험)
src/interfaces                Unitree 메시지
src/external                  FAST-LIO, Livox 메시지, Hesai 드라이버 (포크, 출처: src/external/README.md)
scripts/                      go2_run.sh, go2_map_clean.py, go2_amcl_init.py, go2_start_align.py, go2_goto.py
config/ · maps/ · docs/ (docs/rulebook: 대회 규정)
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
| 라바콘 LiDAR 판정 (합성, 55 cm 간격 2개 포함 4개) | — | 4/4 검출, 오차 0.2~3 cm, 기둥·벽 오검출 0 |
| 라바콘 LiDAR 확정 (대회장 녹화 145915, 콘 10개) | 6/10 (벽에 붙은 콘 놓침) | 10/10, 오차 6~13 cm, 가짜 0 |
| 콘 2.5 m 간격 경로 계획 (누리관 지도) | 반발 없음 28 % | 70 %, 콘 절대 반경 침범 0 |
| AMCL 이동 입력 다리 odom → FAST-LIO (대회장 녹화 3개) | 중앙 9~14 cm | 3~3.6 cm, 스캔 일치 97~99 % |
| 슬라럼 1.0 m 통로 경로 계획 (규정 치수 지도) | — | 콘 지그재그(±0.25 m) 100 %, 콘이 가운데면 기본값 0 % (아래 제한) |
| base_stab 기구학 보정 켜짐 → 꺼짐 | 흔들림 2.5배, 중력 대비 2.6° | AMCL 동등 (중앙 8.3 → 7.0 cm) → 꺼짐 |

자세한 측정 방법과 구현: [docs/PROGRESS_20261103.md](docs/PROGRESS_20261103.md)

## 알려진 제한

- FAST-LIO 는 XT16 `/lidar_points` 와 CycloneDDS 에서만 안정적으로 확인됨. 높이(z) 드리프트 있음(9분 +0.83 m, 2D 주행에는 영향 없음)
- **슬라럼 통로 폭은 약 1.0 m** (지도 0.99 m). 콘이 통로 가운데에 있으면 양쪽 틈 0.425 m 에 로봇 0.31 m — 각 5.75 cm 여유.
  기본 설정으로는 경로가 안 나오고, `grid.resolution 0.05`·`inflate_radius 0.19`·`cones.inflate_radius 0.25` 로도 25~40 %
- 라바콘은 LiDAR 만: 콘 구역(`regions`, 지도 좌표) 밖에서는 계단·경사로 모서리, 낮고 좁은 물체가 콘이 된다(콘 없는 10-30 건물
  녹화 41개) — 지도가 바뀌면 구역을 다시 정할 것. 왼쪽 콘 2개는 보는 방향에 따라 2~3개로 겹쳐 확정되기도 함
- FAST-LIO 가 튀어도 이 스택은 스스로 멈추지 않는다(예전 경로 실행기의 튐 감지는 미션 제어와 함께 삭제) — 팀원 미션 쪽에서 감시할 것
- 보험 모드는 직사각형 경기장에서 검증 필요

## 문서

- [docs/rulebook/](docs/rulebook/) — 2026 로보월드 대회 규정, QnA(라바콘 23 cm·14×14 cm, 신호등 130 cm), 경기장 배치도
- [docs/PROGRESS_20261103.md](docs/PROGRESS_20261103.md) — 진행 상황, 구현 방법, 측정 결과
- [docs/OPERATIONS.md](docs/OPERATIONS.md) — 운용·튜닝 (이전 README)
- [docs/GITHUB_RULES.md](docs/GITHUB_RULES.md) — 협업 규칙
- [docs/Blueprint_Complete.md](docs/Blueprint_Complete.md) — 설계 블루프린트
- [docs/HANDOVER_ROBOWORLD_2026.md](docs/HANDOVER_ROBOWORLD_2026.md) — 인수인계

## 참고 논문 (References)

본 프로젝트의 궤적 계획, 3D 매핑, 위치추정은 다음 연구들을 바탕으로 사족보행 환경에 맞춰 구현·적용되었습니다:

1. **ABPolicy (Asynchronous B-Spline Flow Policy & $C^2$ Handover)**
   - Yang, F., Jing, P., Qu, K., Zhao, N., & Su, Y. (2026). *ABPolicy: Asynchronous B-Spline Flow Policy for Real-Time and Smooth Robotic Manipulation*. arXiv preprint [arXiv:2602.23901](https://arxiv.org/abs/2602.23901).
   - *적용*: `src/core/ego_planner`의 비동기 재계획 시 저크(Jerk) 없는 $C^2$ 연속 궤적 스티칭(`optimizeWithContinuity`) 구현

2. **EGO-Planner (ESDF-free Gradient-based Local Planner)**
   - Zhou, X., Wang, Z., Ye, H., Xu, C., & Gao, F. (2021). *EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors*. IEEE Robotics and Automation Letters (RA-L), 6(2), 478-485. [DOI: 10.1109/LRA.2020.3047728](https://doi.org/10.1109/LRA.2020.3047728).
   - *적용*: B-스플라인 제어점 수치 최적화 및 2D 라이다 점군 장애물 반발력 모델

3. **FAST-LIO2 (Fast Direct LiDAR-Inertial Odometry)**
   - Xu, W., Cai, Y., He, D., Lin, J., & Zhang, F. (2022). *FAST-LIO2: Fast Direct LiDAR-Inertial Odometry*. IEEE Transactions on Robotics (T-RO), 38(4), 2053-2073. [DOI: 10.1109/TRO.2022.3141876](https://doi.org/10.1109/TRO.2022.3141876).
   - *적용*: `src/external/fast_lio` 기반 Hesai XT16 + Go2 IMU(500Hz) 고주파 주행 오도메트리 및 3D 점군 지도(`.pcd`) 구축

## 라이선스

Apache-2.0. 단, `src/interfaces`(Unitree, BSD 3-Clause)와 `src/external`(FAST-LIO GPL-2.0, Hesai BSD 등)은
각 패키지의 라이선스를 따른다 ([src/external/README.md](src/external/README.md)).


1. 신호등 4초 기다리기 문제.
가끔 4초 넘게 안 바뀌는 경우 있으니 대기 시간 늘리기
2. 왼쪽 쏠림 보정 필수.
3. 위치추정 발산
4. 주차장에서 진입후 앉았다 회전하기. 회전후 바로 앉으려면 불안정함. 잠깐 기다리고 안정화한 후 도는 방밥더 있지만 앉았다 일어난 다음 도는게 훨씬 깔끔
5. 경로를 주차장 안까지 한번에 가려고 하니 꼬깔을 쳐바림 꼬깔 앞뒤로 핀을 찍어 그 부분만 regression 기반 알고리즘으로 주행
6. 슬라럼 주행중 길이 막혔다고 판단될 때 매번 4초 기다릴 필요 있을까? 그리고 경로 생성 알고리즘 조금 손봐야 할듯. 너무 쉽게 막힘.
7. 벽에 페널티 강화하기. 자꾸 부딛힘
---

## 재사용 가이드

### 떼어 쓰기 좋은 부품
| 부품 | 위치 | 입력 → 출력 | 다른 곳에서 쓰려면 |
|---|---|---|---|
| IMU 브리지 | `src/core/go2_bridge/src/imu_bridge_node.cpp` | `/lowstate` → `/imu/body`(500 Hz, `imu_link`) | Unitree 메시지(`src/interfaces`)만 있으면 됨 |
| 점구름 브리지 + 수직성 스캔 | `src/core/go2_bridge/src/cloud_bridge_node.cpp` | `/lidar_points` → `/velodyne_points`(왜곡보정), `/scan`(`base_stab` 기준) | `scan_mode: vertical` 은 링 번호가 있는 회전형 라이다 전제. 장착 위치는 TF 로 |
| 다리 odom TF | `src/core/go2_bridge/src/odom_tf_node.cpp` | `/utlidar/robot_odom` → TF `odom→base_link` | — |
| 모터 브리지 | `src/core/go2_bridge/src/cmd_vel_to_sport_node.cpp` | `/cmd_vel` → `/api/sport/request` | `max_vx/vy`, 최소 유효 명령, `yaw_trim_*`(직진 쏠림 보정)은 로봇·걸음새마다 다시 잴 것 |
| AMCL 설정 | `src/core/go2_localization/config/amcl_params.yaml` | `/scan` + 2D 지도 → `map→odom` | Omni 이동모델은 옆걸음하는 로봇이면 유지 |
| 출발 정렬 | `scripts/start_align_core.py`(ROS 무관 핵심) + `go2_start_align.py` | 기립 직후 3초 스캔 + 2D 지도 → 경로점 파일을 오늘 좌표로 변환 | 거부 범위(±0.3 m / ±10°)는 경기장 대칭성에 맞춰 조정 |
| AMCL 출발 초기화 | `scripts/go2_amcl_init.py` | 공분산 넣은 `/initialpose` + 정지 갱신 반복 | 그대로 |
| LiDAR 콘 추적기 | `src/missions/mission_vision/src/cone_tracker_node.cpp` | 점구름 → `/mission/cones`, `/mission/cone_obstacles` | 콘 크기 띠(z·폭·위 빈칸)와 `regions`(지도 좌표)를 대상 물체·지도에 맞춰 다시 정할 것 |
| 직진 쏠림 측정 | `scripts/go2_yaw_drift.py` | 실주행 → `yaw_trim_per_vx`, `yaw_trim_bias` 제안 | **명령** 속도 기준으로 맞출 것(실제 속도 기준이면 과보정) |
| EGO-Planner 오프라인 시험 | `src/core/ego_planner` 의 `test_plan_on_map` | 저장 지도 위 계획 성공률·벽 간격 | 로봇 없이 파라미터 비교 |

### 기동 순서 (lifecycle)
런타임 노드는 모두 lifecycle 노드다. `configure → activate` 를 하지 않으면 라이다·IMU·모터 명령이 나오지 않는다.
`scripts/go2_run.sh` 가 순서대로 처리하며, 모터 브리지는 마지막에 켠다. 소프트웨어 정지는 모터 브리지 `deactivate` 다.

### 다른 로봇·경기장으로 옮길 때 바꿀 것
1. **라이다 장착 TF**(브리지 설정·런치)와 FAST-LIO `extrinsic_T/R`(`config/fastlio/`). 자가추정(`extrinsic_est_en`)은 켜지 말 것 — 정지 중 흔들림이 생기고, 공분산이 무너지면 틀린 값에서 멈춘다
2. **FAST-LIO `det_range`(방 크기 + 여유)와 `preprocess.max_range`**(넓은 실내라면 15 m 정도). 크게 잡으면 궤적이 경보 없이 미끄러진다
3. **지도**: 로봇이 완전히 선 뒤 매핑 시작, 사람 없이. 2D 변환은 바닥 평면 기준으로 자를 것
4. **콘 추적기 `regions`**(지도 좌표) — 지도가 바뀌면 다시 정한다
5. **모터 제한·최소 명령·쏠림 보정** — 로봇과 걸음새마다 다시 잰다
6. **DDS**: CycloneDDS, 커널 `rmem_max/wmem_max` 128 MB, 멀티캐스트는 디스커버리에만

### 경로가 박혀 있는 곳
- `src/external/fast_lio/config/go2_xt16_mapping.yaml` 의 `map_file_path` 가 `/home/ilovehan9283/go2_ws/maps/...` 다. 자기 경로로 바꿀 것
- `config/system_params.yaml` 의 주석에 개발 PC 경로가 있다(동작과 무관)

### 로봇 없이 시험
- `GO2_RUN_TEST_LO=<도메인>` 으로 루프백 전용 재생(로봇과 섞이지 않음)
- `scripts/sim_nav_test.py`, EGO `test_plan_on_map`, 콘 판정 합성 시험(`mission_vision`)
- 재생은 반드시 CycloneDDS 로 한다. FastDDS 로는 FAST-LIO 가 비결정적으로 발산했다
