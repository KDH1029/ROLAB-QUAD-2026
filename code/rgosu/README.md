# rgosu — 자율주행 미션 플래너의 틀

> 구분: 미션 플래너(자율주행) · 시점: 2026-11-05 새벽(junsu 로 분리되기 직전) · git 이력 없음(소스 사본)
> 이 폴더를 복사해 만든 [`../junsu`](../junsu) 가 이후 대회에 쓴 판이다. **새로 가져다 쓸 거라면 junsu 를 먼저 보라.**
> 여기는 원형(구조·설계 원칙·시뮬레이터)을 볼 때 쓴다.

## 1. 무엇을 하는가
FAST-LIO 로 위치를 잡고, 벽·장애물 점구름을 EGO-Planner 에 넣어 지역 경로를 만든다.
그 위에서 **코스 러너**가 미리 정한 목표점을 순서대로 보내며 정지선·신호등·주차·출구를 처리한다.

```
XT16 ─ hesai_ros_driver ─▶ cloud_bridge ─▶ FAST-LIO(다리 결합) ─▶ wall_filter(수직 벽만) ─▶ obstacle_fusion(벽+L1+카메라 콘)
                                                     │                                              │
                                              /Odometry ───────────────────────────────▶ EGO-Planner ─▶ /cmd_vel
                                                     │                                              ▲
                                        real_course_runner(목표 순서·정지선·주차·출구) ──── /goal ───┘
                                                     │
                              내부 PC: traffic_light_trt_node(YOLO) ─▶ 신호 / 카메라 콘
```

## 2. 설계 원칙 (2026-11-01 확정, `WORKLOG.md` 맨 앞)
- **센서 역할 고정**
  - 라이다(XT16 + FAST-LIO)는 위치·주행·회피를 맡는다.
  - 카메라(D435i)는 신호등 색과 QR 인식만 하고, 위치추정 루프에는 넣지 않는다. 비전 SLAM(FAST-LIVO2)은 이 로봇에서 발산으로 실패했다.
- **카메라는 정지선 근처에서만 켠다**(`segments_config.json` 의 `camera_active`). 열린 대회장에서 주변 물체를 빨간불로 오인하지 않게 하고, 나머지 구간의 연산은 라이다에 몰아준다.
- **정지선·주차 위치는 좌표(지오펜스)로 판정**한다. 비전은 "신호가 무엇인가"만 본다.
- **콘과 벽은 구분하지 않는다.** "피할 것 / 지나갈 수 있는 것"만 나눈다. 계단·경사 구간은 지오펜스로 장애물 높이 필터를 완화한다.

## 3. 폴더 구조
| 경로 | 내용 |
|---|---|
| `src/real_course_runner.py` | **코스 러너**(핵심). 경로 파일 순서대로 목표 발행, 정지선·신호 대기, 주차, 출구, 막힘 복구 |
| `src/track_sequencer_node.py`, `segments_config.json` | 구간 상태기: 구간별 카메라 on/off·장애물 필터 strict/relaxed |
| `src/obstacle_fusion_node.py` | 벽(wall_cloud) + L1 낮은 점 + 카메라 콘을 합쳐 `/mission/obstacle_cloud`(다리 odom 으로 2초 기억) |
| `src/stopline_mission_node.py`, `parking_*_node.py`, `traffic_light_detector_node.py` | 정지선·주차·신호등 단위 노드(초기 구조) |
| `src/test_*.py` | 로봇 없이 도는 단위 시험(러너·정지선 디바운스·주차·막힘 복구·신호 판정 등) |
| `tools/go2_full_stack.sh` | **전체 스택 기동**(9단계, 순서·주의가 머리말에 자세함) |
| `tools/go2_bridge_lc.sh` | go2_bridge lifecycle 노드 configure → activate |
| `tools/route_editor.py` | 지도 위 경로점 편집기(한글 자판 키 대응) |
| `tools/record_all_with_camera.sh`, `start/stop_recording_tmux.sh` | 녹화 |
| `tools/replay_*_check.sh`, `run_compare_rgosu.sh` | 녹화 재생으로 스택·벽 필터·장애물 점검 |
| `sim/run_sim.sh`, `sim_world_node.py` | **폐루프 시뮬레이터**: 가상 로봇 + 실제 러너·EGO·모터 브리지·신호등 노드 |
| `ws_fastlio_tunable/`, `ws_legfuse_gated/` | FAST-LIO 포크(다리 속도 결합·모션 게이팅·점프 상한) |
| `ws_ego_planner_tunable/` | EGO-Planner 포크(실행 중 `inflate_radius` 변경, `grid_res`, `min_range_xy`) |
| `ws_wall_filter/` | 링 정보로 수직 벽만 골라 `/mission/wall_cloud`(C++) |
| `onboard_vision/` | 내부 PC 신호등·콘 YOLO(TensorRT) |
| `depth_terrain/` | 깊이 카메라 위험도 격자 PoC(설계 원칙 확정 뒤 보류) |
| `config/` | FAST-LIO 설정 변형(`fastlio_leg_nurigwan_derange30.yaml` = 대회 기준), LIO-SAM 파라미터 스윕 |
| `maps/*.json` | 경로점 파일(`route_map.json` 등). 지도 바이너리는 빠져 있음 |
| `WORKLOG.md` | 09-30 ~ 11-05 작업 일지(실측·실패·함정 전부, 약 200 KB) |

## 4. 의존성
- Ubuntu 24.04 + ROS 2 Jazzy, CycloneDDS(`rmw_cyclonedds_cpp`)
- 다른 작업공간의 빌드 결과를 `source` 한다. 이 폴더만으로는 돌지 않는다.
  - `~/ws/merge_tmp_20261102/review/install` → **go2_bridge**(lifecycle), Hesai 드라이버, Unitree 메시지 = [`../merge_tmp`](../merge_tmp)
  - `~/unitree_ros2/cyclonedds_ws/install` → Unitree 공식 메시지
  - `~/ws/v_gosu/ws_legfuse/install` → 다리 결합 FAST-LIO 빌드(같은 계열 소스가 `ws_legfuse_gated/`)
- 파이썬: numpy, scipy, PyYAML, matplotlib(그림), ultralytics·onnxruntime(신호등 시험, `venv_yolo`)

## 5. 빌드
```bash
source /opt/ros/jazzy/setup.bash
source <merge_tmp>/install/setup.bash          # go2_bridge 등
for ws in ws_fastlio_tunable ws_ego_planner_tunable ws_wall_filter; do
  (cd $ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
done
```
- FAST-LIO 는 Release 빌드를 권장한다. aarch64 에서는 CMake 가 OpenMP 를 꺼 지연이 1.5배 늘어난다.
- `src/` 를 고친 뒤 `install/` 에 반영(빌드)하지 않으면 런치가 옛 설정을 읽는다.

## 6. 실행
```bash
tools/go2_full_stack.sh                 # 라이다 → 브리지 → FAST-LIO → 벽 → 합치기 → EGO → 모터 → 카메라 → 신호등
python3 src/real_course_runner.py ...   # 러너(경로 파일·작전 값은 RUNNER_* 환경변수)
tools/go2_stop_all.sh                   # 전부 끄기
```
- **로봇은 엎드린 채로 켜고, 기립 후 30초 안에 출발한다**(서 있기만 해도 뒷다리 hip 이 과열됨).
- 경기 중에는 시각화 브리지(Foxglove)를 끈다(센서 데이터 외부 전송 금지 규정).

## 7. 로봇 없이 시험
```bash
cd src && python3 test_real_course_runner.py       # 러너 상태기
sim/run_sim.sh 420 5 0.25                           # 폐루프 시뮬(제한 초, 신호 주기, 벽 여유) → sim/runs/sim_<시각>/report.txt
```
- 시뮬레이터는 DDS 도메인 79 + 루프백 전용이고, 실제 로봇이 응답하면 실행을 거부한다.
- 한계: 시뮬 로봇은 늘 잘 돌고 미끄러지지 않는다. 제자리 회전 실패나 콘 밀기는 재현되지 않는다.

## 8. 재사용할 때 고칠 곳
- **절대 경로**: 파일 16개(스크립트·설정)에 `~/ws/Rgosu`, `~/ws/merge_tmp_20261102/review`, `~/ws/v_gosu` 가 박혀 있다. 먼저 바꿀 것:
  ```bash
  grep -rn '~/ws/\|/home/' tools sim src --include=*.sh --include=*.py
  ```
- **좌표**: `segments_config.json` 의 구간 좌표와 `maps/*.json` 경로점은 대회장 지도 기준이다. 새 지도에서는 다시 찍는다.
- **FAST-LIO**: `det_range`(방 크기 + 여유)와 `preprocess.max_range`(넓은 실내 15 m). [`../../docs/03_트러블슈팅.md`](../../docs/03_트러블슈팅.md) B5·B7 참고
- **EGO 여유**: `inflate_radius` 는 0.25 m 밑으로 내리지 말 것. 계획기가 로봇 중심 25 cm 안 점을 버려서, 더 내리면 사각지대가 생긴다(시뮬: 0.30 이면 접촉 54초·7/7 도착, 0.25 면 247초·5/7).

## 9. 알려진 문제 (이 시점 기준)
- wall_filter 가 원뿔형 콘을 "수직 벽 아님"으로 걸러 낼 수 있다 → junsu 에서 obstacle_fusion 으로 보완
- 주차 동작 순서는 이 시점 이후에 바뀌었다. 일어나는 도중 걸음새 명령을 보내면 넘어진다(예선 사고) → junk/juninsu 의 순서(바닥 확인 → 시간 → 일어남 안정 확인 → BalanceStand)를 쓸 것
- 자세한 경과는 `WORKLOG.md`
