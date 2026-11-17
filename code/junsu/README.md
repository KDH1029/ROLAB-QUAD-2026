# junsu — 자율주행 미션 플래너 (통합 스택)

> 구분: 미션 플래너(자율주행) · 시점: 원본 `~/ws/junsu` 브랜치 `v2` = 12aff3e (2026-11-06 13:38 최종)
> 뿌리: [`../rgosu`](../rgosu) 폴더를 복사(러너·전체 기동 스크립트·시뮬레이터)하고, [`../merge_tmp`](../merge_tmp) 의 기반 기술을 얹었다.
> 11-05 16:54 완주(134초) 때의 코드는 원본 저장소 태그 `run-1105-1654`(= `main` a438045)에 있다. 이 폴더는 그 뒤 11-06 개선판(v2)이다.

## 1. 무엇을 하는가
FAST-LIO 위치 + EGO-Planner 지역 경로 + 코스 러너로, 경로 파일의 목표를 순서대로 따라가며 코스 과제를 수행한다.
- 정지선 → 신호등 판정 → 출발
- 슬라럼: EGO 먼저, 막히면 핀(미리 찍은 점) 회귀 곡선 추종
- 주차 → 출구 밀고 나가기 → Out Line 통과 판정

운영은 글자 화면 관리자 하나(`tools/go2_ctl.sh`)로 한다.

## 2. 한눈에 실행
```bash
tools/go2_ctl.sh                       # 실제 로봇
GO2_CTL_REPLAY=80 tools/go2_ctl.sh     # 재생 시험(격리 도메인 78~81, 로봇이 응답하면 거부)
```
```
Q(남은 부품 정리) → a(전체 켜기) → 상태판 전부 초록 → 로봇을 출발 자리에 → 리모컨 기립 → m(모터) → r(러너)   ★ 기립 후 30초 안에 r
비상: x 또는 스페이스 · 이어가기: m → R → 목표 번호 · 정비 정지: p · 작전 값: s · 전부 끄기: Q
```
자세한 순서·키·문제 해결: [`MANUAL.md`](MANUAL.md)(11-06 판), [`MANUAL_1105.md`](MANUAL_1105.md)(11-05 판)

## 3. 구성 (관리자 `a` 가 켜는 순서)
| # | 부품 | 위치 | 하는 일 |
|---|---|---|---|
| 1 | XT16 드라이버 | merge_tmp 빌드(`hesai_ros_driver`) | `/lidar_points` 20 Hz. **L1(`/utlidar/cloud`)로 바꾸지 말 것** — FAST-LIO 가 수 분 안에 발산 |
| 2 | 브리지(lifecycle) | `tools/go2_bridge_lc.sh` → merge_tmp `go2_bridge` | IMU 500 Hz, 점구름 왜곡보정, 다리 odom TF. configure → activate 를 안 하면 아무것도 안 나옴 |
| 3 | FAST-LIO | `ws_fastlio_tunable` + `config/fastlio_leg_nurigwan_derange30.yaml` | 다리 속도 결합·모션 게이팅, `det_range 30`, `max_range 15` |
| 4 | 벽 필터 | `ws_wall_filter` | 링 정보로 수직 벽만 → `/mission/wall_cloud` |
| 5 | 장애물 합치기 | `src/obstacle_fusion_node.py` | 벽 + L1 낮은 점(경사로 마스크 `maps/ramp_mask.npz` 밖) + 카메라 콘 → `/mission/obstacle_cloud` |
| 6 | EGO-Planner | `ws_ego_review`(기본) / `ws_ego_planner_tunable` | 격자 0.05, 벽 여유 0.30(콘 구간 0.20), 실행 중 여유 변경 |
| 7 | 카메라·YOLO | 내부 PC `~/junsu_vision`(= `onboard_vision/`) | 정지선에서 신호등, 상시 카메라 콘(`/mission/cone_cam`) |
| 8 | 위치 감시 | `tools/amcl_watch_core.py` 등 | FAST-LIO 와 AMCL 비교, 크게 어긋나면 ICP 재정렬 |
| — | 몸 모양 안전 필터 | `src/footprint_guard_node.py`(+ `_core.py`) | 직사각형 몸으로 모든 명령을 1초 앞까지 굴려 봄. 관리자 `.` 로 켜고 끔(11-06 기본 off) |
| — | 모터 | merge_tmp `cmd_vel_to_sport` | 관리자 `m`. 직진 쏠림 보정값을 넘긴다 |
| — | 러너 | `src/real_course_runner.py` | 관리자 `r` / `R` |

## 4. 코스 러너가 하는 일 (`src/real_course_runner.py`)
- **출발**: BalanceStand → ClassicWalk 응답 확인(실패면 1초 간격 재전송) → 첫 목표. 자가진단으로 출발점에서 0.5 m 넘게 떨어져 있으면 출발하지 않는다.
- **지나가는 점**: 1.2 m 앞에서 다음 점으로 넘겨 멈칫거림을 줄인다. 경로에서 벗어나면 되짚는다.
- **정지선**: 15 cm 안(또는 0.4 m 안에서 1.5초)이면 도착. EGO 를 세우고 카메라를 켠다 → 초록 1초 연속이면 출발. 빨강은 최대 40초, 신호 없음은 15초.
- **슬라럼**(`RUNNER_SLALOM_MODE`): EGO 먼저 → 첫 실패(1.5초 길 못 냄·2초 제자리·곡선 0.5 m 이탈·25초)에서 지금 자리부터 핀 경로. 핀이 2.5초 막히면 빠져나오기, 3번 넘으면 다음 점.
- **주차**: 엎드림 → 바닥 확인 → 시간 → 일어남 → 넓은 재정렬(±0.6 m / ±45°) → 회전
- **출구**: 몸 전체가 Out Line 밖(중심 x ≤ 선 − 0.45, 기본 선 −0.54)이면 완주. 모자라면 0.18 m/s 로 밀고 나간다(앞 0.22 m 막히면 정지, 6초 한도).
- **막힘 복구**: 4초 제자리 + 장애물 가까움 → 반대쪽으로 0.20 m(0.25 m/s) 물러남. EGO 가 1.5초 침묵하면 2초 만에 막힘 처리.
- **넘어짐**: 기울기 50° → 멈춤 → 다시 서면 넓은 재정렬 후 이어감. 물러나기 12번이면 그 자리 대기.

작전 값은 환경변수(관리자 `s` 에서도 바꿈):
`RUNNER_SLALOM_MODE` `RUNNER_PIN_EGO_FIRST` `RUNNER_PIN_EGO_MAX_S` `RUNNER_EXIT_LINE_X` `RUNNER_STOPLINE_ARRIVE` `RUNNER_STOPLINE_NEAR_WAIT`
`RUNNER_EGO_SILENT_S` `RUNNER_EGO_SILENT_STUCK_S` `RUNNER_OFF_ROUTE_M` `RUNNER_RETRACE_MAX` `RUNNER_ROTATE_SKIP_DEG` `RUNNER_TRACK_MIN_V`
`RUNNER_GAIT_MODE`(legacy = 옛 걸음새 전환) `RUNNER_PARK_MODE`(sit = 옛 주차) `RUNNER_CARROT_M` `RUNNER_SHORTCUT_CLEAR` `RUNNER_SNAP_*` `RUNNER_SLALOM_JOIN_MAX` `RUNNER_CONE_ZONE_EARLY`

## 5. 폴더 구조
| 경로 | 내용 |
|---|---|
| `src/` | 러너, 장애물 합치기, 몸 모양 필터, 구간 상태기(`track_sequencer_node.py`), `test_*.py` 단위 시험 |
| `tools/` | 관리자(`go2_ctl.sh/.py`), 전체 기동(`go2_full_stack.sh`), 경로 편집기(`route_editor.py`), 쏠림 측정(`yaw_trim_measure.sh`), 위치 감시, 재생 점검 도구 |
| `maps/` | 경로 파일 `route_*.json`(11-06 13:30 기본 = `route_map_1106_user.json`), 지도 `rec1114/ rec1118/ rec1118x/ rec1650/`(2D npz·pgm), 경사로 마스크, 좌표 변환 `T_1459_from_1650.json` |
| `sim/` | 폐루프 시뮬레이터 v2(`run_sim_v2.sh`, `sim_world_v2.py`, 벽·콘 배치 npy, 시험용 경로 json) |
| `config/` | FAST-LIO 대회 설정, CycloneDDS(`cyclonedds_junsu.xml`: 멀티캐스트는 디스커버리만), 콘 추적기 설정 |
| `onboard_vision/` | 내부 PC 신호등·콘 YOLO: `traffic_light_trt_node.py`, TensorRT 빌드·평가, 배포 `deploy_onboard_vision.sh`, OpenVINO → ONNX 복원 `models_1104/` |
| `ws_*` | colcon 작업공간 6개(아래 빌드) |
| `debug/` | 실주행·재생 분석(예: `run1118/` 11:19 주행, `yolo_cone_1106/` 카메라 장착값, `agents/` 검토 보고서) |
| `README_junsu.md` | Rgosu 대비 바뀐 것 목록(11-05) |
| `HANDOFF_*.md` | 날짜별 인수인계(11-05 위치 감시, 11-06 v2 작업 전체) |
| `WORKLOG.md` | Rgosu 시절부터의 작업 일지(복사해 온 판) |
| `ARTIFACTS.md5` | git 에 안 넣은 큰 파일(모델·엔진·pcd)의 md5 목록 |

## 6. 빌드
```bash
source /opt/ros/jazzy/setup.bash
source <merge_tmp>/install/setup.bash                 # go2_bridge·Hesai 드라이버·Unitree 메시지
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
for ws in ws_fastlio_tunable ws_legfuse_gated ws_ego_review ws_ego_planner_tunable ws_wall_filter ws_mission_vision; do
  (cd $ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
done
```
- `ws_mission_vision` 은 merge_tmp 의 콘 추적기를 따로 빌드한 것이다.
- 내부 PC(Foxy, JetPack 5): `onboard_vision/deploy_onboard_vision.sh fp16` → `start`. TensorRT 엔진은 장치마다 새로 만든다(INT8 은 초록 재현율이 떨어져 기본 FP16).
  - 모델 선택 `TL_MODEL=tl_cone_1104|tl5cone`, 카메라 장착값 `GO2_CAM_FIX=1650`(11-06: 거의 수평. 관리자가 넘긴다)
- 파이썬은 `~/venv/go2`(numpy·scipy·matplotlib, opencv 는 시스템 판 사용)

## 7. 로봇 없이 시험
```bash
cd src && python3 test_real_course_runner.py                # 러너 상태기 — 전부 PASS 여야 함
python3 tools/test_amcl_watch.py                            # 위치 감시
tools/loc_park_replay/run.sh B_gate true 81                 # 엎드림 구간 위치 보호(녹화 재생, 도메인 81)
cd sim && TMAX=420 EXIT_X=-0.14 CONE=det GUARD=off ./run_sim_v2.sh <이름> 80 $PWD/route_v2_1650_hybrid_sim.json
```
- 시뮬 조건: `CONE=det|walls`(콘 검출 됨/안 됨), `GUARD=on|off`, `SMODE=auto|path|ego`, `RUNNER_*`. 결과는 `sim/runs_v2/<이름>/report.txt`, `run.png`
- 11-06 결과: 3/3 완주 128~147초. 보정 세계 슬라럼 1판은 갇힘(미해결)
- 시뮬 로봇은 늘 잘 돈다. 제자리 회전 실패·콘 밀기는 재현되지 않는다(본선 회전 실패를 못 잡은 이유).

## 8. 재사용할 때 고칠 곳
- **절대 경로**: `~/ws/junsu`, `~/ws/junsu_v2`, `~/ws/merge_tmp_20261102/review`, `~/ws/v_gosu/ws_legfuse`, 내부 PC `~/junsu_vision` 이 스크립트에 박혀 있다(`debug/` 포함 약 100개 파일).
  운영에 필요한 것은 `tools/*.sh`, `tools/go2_ctl.py`, `sim/run_sim_v2.sh` 다:
  ```bash
  grep -rn '~/ws/\|/home/\|junsu_vision' tools/*.sh tools/go2_ctl.py sim/*.sh onboard_vision/*.sh
  ```
- **경로 파일 좌표계**: 경로 파일은 매핑한 날의 지도 좌표다. 새 지도면 다시 찍거나 `T_*.json` 변환을 새로 구한다. 출발 정렬 거부 범위 ±0.3 m / ±10° 밖이면 원본 좌표를 그대로 쓴다.
- **관리자 상태 파일**: `maps/segments/logs/go2_ctl_state.json` 에 저장된 작전 값이 기본값을 덮는다. 기본 경로를 바꿨는데 예전 경로로 돈다면 이 파일을 지운다.
- **직진 쏠림 보정**: 로봇·걸음새마다 `tools/yaw_trim_measure.sh` 로 다시 잰다(명령 속도 기준).
- **콘 크기**: 콘 추적기 설정(`config/cone_tracker_junsu.yaml`)은 작은 콘(23 cm) 기준이다. 대형 콘(0.7~0.8 m)은 z −0.25~0.25, 위 빈칸 0.30, 최대 폭 0.5, 최소 점 5 로 6/6 이었다.

## 9. 알려진 문제·미검증 (11-06 기준, `MANUAL.md` 12절)
- 새 대형 콘을 카메라·라이다가 콘으로 알아보는지 실기 미확인
- 슬라럼 대체 길·경로 이탈 되짚기·회전 완료 규칙은 시험·재생으로만 확인했다
- 출구 선 x(지도 −0.54 가정), 발을 떼는 최소 속도(0.08~0.15 추정)
- 11-06 실주행 2번(11:19, 14:15) 모두 출구를 밀고 나가다 앞 장애물 판정으로 통과 확인 전에 멈췄다 — [`../../docs/05_실주행_기록.md`](../../docs/05_실주행_기록.md)
