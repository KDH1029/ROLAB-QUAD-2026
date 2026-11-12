# juninsu — 기록 경로 반복 미션 플래너 (발전판)

> 구분: 미션 플래너(기록 경로 반복) · 시점: 원본 `~/ws/juninsu` `main` = 2f40691 (2026-11-06 15:32, 본선 회전 실패 수정)
> 뿌리: [`../junk`](../junk) 의 검증본(`nail-1105`)에서 갈라져 11-06 에 다듬은 판. 본선 시간대 주행에 쓰였다.
> **실시간 경로 계획을 하는 일반적인 자율주행 알고리즘이 아니다.** 사람이 한 번 몰아 기록한 길을 FAST-LIO 위치로 따라간다.

## 1. 왜 이 방식인가
대회 도중(11-05) 규정이 바뀌었다: 콘이 고정되고 대형(높이 0.7~0.8 m)으로 바뀌어, 학습해 둔 YOLO 로는 인식할 수 없게 됐다.
콘 배치가 고정이면 **지도가 곧 코스**다. 그래서 실시간 인식·회피 대신 기록한 경로를 정확히 따라가는 쪽을 택했다.
위치추정(XT16 + FAST-LIO)과 하드웨어는 그대로 두었고, 신호등만 YOLO 로 본다.

## 2. 전체 흐름 (지도 → 경로 → 주행)
```bash
# ① 리모컨으로 코스를 한 바퀴 돌며 녹화(/lidar_points, /lowstate 필수)
~/go2_record/go2_record_auto.sh final1
# ② 녹화 → 지도: FAST-LIO 재생 → 3D 복셀(2 cm)·궤적 map.npz → 2D map.pgm/yaml (세로 25 cm 넘는 면 = 벽)
tools/make_map.sh ~/bags/<녹화 폴더> final1          # 시간 ≈ 녹화 길이 + 1분. 로봇이 연결돼 있으면 멈추고 묻는다(ROBOT_OK=1)
# ③ 지도 위에 경로 찍기 → routes/final1/route.json
~/venv/go2/bin/python3 tools/route_editor.py final1   # --new 로 처음부터
~/venv/go2/bin/python3 tools/show_route.py final1     # 확인 그림 route.png (벽·콘 0.25 m 안 = 빨강)
# ④ 주행
tools/junk_ctl.sh                                     # Q → a → 출발 자리 → (옮겼으면 3) → 리모컨 기립 → m → r
```
녹화 궤적에서 경로 초안을 만들려면 `tools/route_from_path.py`, 경로 일부만 잘라 시험하려면 `tools/sub_route.py`.

### 경로 편집기 키 (`tools/route_editor.py`)
- 점 종류: `1` start · `2` line(지나가는 점) · `4` stopline · `7` parking · `8` exit
- 빈 곳 클릭 = 고른 점 다음에 새 점 · 끌기 = 옮김 · `d`/오른쪽 클릭 = 지움 · `n`/`b` 다음·앞 점 · `u` 되돌리기 · `s` 저장
- **각도**: 점을 고르고 `a` → 바라볼 쪽 클릭 · `←`/`→` 5° · `z` 지움
  - 각도 있는 점 = 거기서 멈춰 그 방향으로 돈 뒤 동작
  - 각도 없는 점 = 지나감
  - 정지선은 신호등 쪽을 보게 각도를 찍는다(카메라가 앞만 본다)

## 3. 러너가 하는 일 (`src/real_course_runner.py`, 경로 전용 모드)
| 단계 | 동작 | 주요 값 |
|---|---|---|
| 출발 | 첫 3초 스캔을 지도에 정합(`tools/start_align/`) → 경로를 오늘 좌표로 변환. ±0.3 m / ±10° 밖이면 거부 | FAST-LIO 뜬 뒤 5초 정지 필요 |
| 경로 추종 | 점을 순서대로(점마다 25 cm 안) 따라감. 몸 방향 = 경로 접선(S자) | `PIN_V 0.30`, 최소 `PATH_V_MIN 0.18` m/s |
| 앞 막힘 | 몸 앞 띠(중심 앞 0.33~0.48 m, 좌우 ±0.18 m)에 점이 있으면 전진 보류, 3초 넘으면 그냥 감. 지도에 이미 있는 벽(10 cm 안)은 무시 | `FRONT_*`, `RUNNER_FRONT_KNOWN_TOL` |
| 막힘 건너뛰기 | 5초 넘게 제자리(이동 5 cm·회전 5° 미만)면 그 점 건너뜀. 정지선·주차·출구 0.5 m 안이면 도착으로 봄 | `BLOCK_SKIP_S 5.0` |
| 회전 | 각도 있는 점에서 제자리 회전. 1.5초에 3° 못 돌면 0.12 m/s 로 걸으면서 돈다(본선 뒤 추가) | |
| 정지선 | 2초 정지 → YOLO 켬 → 초록 1초 연속이면 출발. 10초 지나면 빨강이 2초 안 보일 때 출발. 빨강이 보이는 동안은 계속 대기 | `STOPLINE_HOLD_S 2.0`, `GREEN_CONFIRM_SECONDS 1.0`, `RED_RECENT_S 2.0` |
| 주차 | StandDown → 바닥 높이(0.09 m 아래) 확인 → 6초 → StandUp → 높이 0.5초 안정(최소 2초) → BalanceStand → 1초 → ClassicWalk 재전송 → 돌기 → 출발 | `PARK_HOLD_S 6.0` |
| 출구 | 몸 전체가 Out Line 밖이면 완주(`out_line_x`, 경로 파일) | |
| 멈춘 곳 위치 확인 | 출발·정지선·주차 뒤 멈췄을 때만 AMCL 로 찾고 3D ICP 로 확인해 맞으면 좌표 보정. 로봇을 세우지 않음 | `juninsu/`, 관리자 `s → J` 로 끔 |

작전 값 바꾸기는 관리자 `s` → `1` 경로 고르기, `H` 몸 방향 방식, `J` 위치 확인 on/off.
비상은 `x`/스페이스, 이어가기는 `m` → `R` → 엔터(FAST-LIO 를 다시 켜지 않았을 때).

## 4. 폴더 구조
| 경로 | 내용 |
|---|---|
| `src/real_course_runner.py` | 러너(경로 전용 모드 포함). `test_path_mode.py`, `test_park_profile.py` = 로봇 없이 도는 시험 |
| `tools/junk_ctl.sh` → `go2_ctl.sh/.py` | 관리자(라이다·브리지·FAST-LIO·비전·위치 확인·모터·러너) |
| `tools/make_map.sh`, `map_replay.sh` | 녹화 → 지도 |
| `tools/route_editor.py`, `show_route.py`, `show_map.py`, `route_from_path.py`, `sub_route.py` | 경로 만들기·확인 |
| `tools/start_align/` | 출발 정렬(`start_align_core.py` = ROS 무관 핵심) |
| `juninsu/` | 멈춘 곳 위치 확인 노드(`stop_loc_node.py`, `icp_core.py`, `loc_params.yaml`, `run_loc.sh`) — 설명 `juninsu/README.md` |
| `routes/<이름>/` | `map.npz`(3D 복셀·궤적), `map.pgm/yaml`(2D), `route.json`(경로), `route.png`, `SOURCE.txt`(어느 녹화에서 왔나) |
| `routes/rec1118x/` | **기본 경로·지도**(11:14 와 11:18 두 녹화의 교집합). `rec1650/`, `rec1645/` = 11-05 녹화 기반 |
| `config/` | FAST-LIO 대회 설정, CycloneDDS(멀티캐스트는 디스커버리만), `yaw_trim.json`(직진 쏠림 보정 0.0156 / +0.0397) |
| `onboard_vision/` | 내부 PC 신호등 YOLO(TensorRT FP16) 노드·배포 스크립트 |
| `ws_fastlio_tunable/` | FAST-LIO 포크(다리 속도 결합·모션 게이팅) |
| `debug/` | 검증 기록: `fullsim/`(코스 랜덤 시뮬 40판), `slalom/`(슬라럼 접촉 판정·S자), `loc/`, `loc_v2/`(위치추정 비교), `stopline/`(신호 규칙), `gait/`, `cones1645/`(콘 위치 대조) |
| `MANUAL.md`, `HANDOFF_juninsu_1105.md` | 실행 매뉴얼, 11-05~06 인수인계 |

## 5. 의존성·빌드
- Ubuntu 24.04 + ROS 2 Jazzy, CycloneDDS, `nav2_amcl`·`nav2_map_server`(위치 확인)
- 다른 작업공간의 빌드 결과를 쓴다:
  - `~/ws/merge_tmp_20261102/review/install` → go2_bridge(lifecycle)·Hesai 드라이버·Unitree 메시지 = [`../merge_tmp`](../merge_tmp)
  - `~/unitree_ros2/cyclonedds_ws/install` → Unitree 공식 메시지
- 파이썬: `~/venv/go2`(numpy·scipy·matplotlib — 편집기는 matplotlib 창). opencv 는 시스템 판을 쓴다(pip opencv 로 덮으면 GStreamer 카메라 수신이 깨짐)
```bash
source /opt/ros/jazzy/setup.bash && source <merge_tmp>/install/setup.bash
(cd ws_fastlio_tunable && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
```
- 관리자 스크립트는 `ws_wall_filter`·`ws_ego_planner_tunable`·`ws_mission_vision` 도 source 하려 하지만 이 폴더에는 없다. 경로 전용 모드에서는 쓰지 않으므로 없어도 된다(junsu 관리자에서 물려받은 부분).
- 내부 PC: `onboard_vision/deploy_onboard_vision.sh fp16` → `start`

## 6. 로봇 없이 시험
```bash
cd src && python3 test_path_mode.py && python3 test_park_profile.py     # 경로 모드·주차 순서(11-04 녹화 실측 높이 곡선)
debug/slalom/sim_env.sh                                                 # 격리(루프백·도메인 93) 시뮬 환경
python3 debug/fullsim/sim_full_route.py                                 # 코스 랜덤 시뮬(SIM_ROUTE=<경로>)
```
- 11-06 결과(rec1118x, 40판): 40/40 완주, 142.0 ± 13.5초, 슬라럼 접촉 9.0 → 5.2초/판(경로 수정 후). 신호 규칙은 기록대로 동작했다.
- 시뮬은 로봇이 늘 잘 돈다. 본선의 회전 실패(39° 틀어진 채 제자리 회전 안 됨)는 '제자리 회전 없음' 모델을 따로 넣어 재현했다.

## 7. 재사용할 때 고칠 곳
- **경로는 자기 위치 기준**(`J=$(dirname …)/..`)이라 폴더를 옮겨도 대부분 동작한다. 남은 절대 경로는 `~/ws/merge_tmp_20261102/review`(브리지 설정), `~/unitree_ros2`, `~/go2_record`, `~/venv/go2` 다:
  ```bash
  grep -rn '~/ws/\|/home/\|~/go2_record\|~/venv' tools juninsu src --include=*.sh --include=*.py
  ```
- **관리자 기본 경로**는 `routes/rec1118x`. 저장된 상태 파일(`logs/go2_ctl_state.json`, 실행 중 생김)은 경로 말고 다른 작전 값만 덮는다.
- **새 코스**: ①~③을 다시 한다. 매핑은 로봇이 완전히 선 뒤 시작하고, 사람 없이 돈다. 2D 지도의 벽 기준(세로 25 cm)은 장애물 높이에 맞춘다.
- **신호 규칙·대기 시간**은 대회 규정(신호 5초 주기, 적색이면 녹색까지 대기)에 맞춘 값이다.
- **로봇 특성 값**: 최소 속도 0.18 m/s(이보다 작으면 발을 떼지 않음), 쏠림 보정 `config/yaw_trim.json` — 로봇·걸음새마다 다시 잰다.

## 8. 알려진 문제
- 경로에서 벗어나는 크기(발 미끄러짐·걸음 흔들림)가 모델에 없다. 슬라럼 콘 접촉은 실측 궤적 없이 장담할 수 없다.
- 위치 확인 노드 실행 스크립트의 주석 버그(`SIM=false` 가 삼켜짐)는 f329568 에서 고쳤다. 본선 때는 이 버그로 노드가 죽어 있었다.
- 자세한 실주행 결과: [`../../docs/05_실주행_기록.md`](../../docs/05_실주행_기록.md)
