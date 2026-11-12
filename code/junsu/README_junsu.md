# junsu — Rgosu 에서 분리한 주행 스택 (2026-11-05 새벽)

Rgosu 를 여러 세션이 동시에 고치며 서로 덮어써서(03:59 go2_full_stack.sh, 04:00 run_sim.sh) 사용자 지시로 복사·분리했다.
이 폴더(~/ws/junsu)는 이 세션만 고친다. Rgosu 는 건드리지 않는다. 녹화(~/bags)·venv_yolo(~/ws/Rgosu/venv_yolo)·v_gosu 는 원본을 읽기만 한다.
내부 PC(젯슨) 배포 폴더도 따로: `~/junsu_vision` (Rgosu 는 ~/onboard_vision).

## 쓰는 법 — 올인원 관리자
```
~/ws/junsu/tools/go2_ctl.sh                      # 실제 로봇(setup_go2_ros2.sh)
GO2_CTL_REPLAY=80 ~/ws/junsu/tools/go2_ctl.sh    # 재생 시험(격리 도메인, 로봇 응답하면 거부)
```
키: `a` 전체 켜기(라이다→브리지→FAST-LIO→벽→합치기→ego→카메라) · `1~7` 그 부품 다시 켜기 · `m`/`M` 모터 켜기/끄기 ·
`r` 러너 처음부터 · `R` 이어가기(목표 번호) · `p` 정비 멈춤(러너 끔+StopMove+모터 끔, 마지막 목표 기억) · `x`/스페이스 비상 정지 ·
`s` 작전(경로 파일·벽 여유·콘 구간 여유·최고 속도·녹색 확인 초·신호 대기 최대·콘 감지 on/off·ego 입력) · `l` 로그 · `q` 관리자만 끝 · `Q` 전부 끄기.
상태판: 부품별 켜짐/토픽 Hz/lifecycle, 러너 단계·목표·남은 거리·콘 구간·물러난 횟수, 로봇 배터리·모터 최고 온도(75°C 넘으면 빨강)·속도·몸 높이, 신호.
★ 규정: 모두 NUC 에서 돌고 화면은 글자뿐(센서 데이터 밖으로 안 보냄). 경기 중 조작은 시작·비상·정비 시간만. 기립 후 30초 안에 출발.

## Rgosu 대비 바뀐 것(전부 이 세션 작업)
- **go2_bridge lifecycle**: review 브리지는 11-03 부터 lifecycle 노드 — `tools/go2_bridge_lc.sh`(bridge_params.yaml + configure→activate). 안 하면 라이다·IMU·모터 명령이 안 나옴. 재생 확인: active · /imu/body 500 Hz · FAST-LIO 20 Hz · 출렁임 0.98/2.7 cm.
- **FAST-LIO `preprocess.max_range: 15.0`**(먼 점이 자세를 흔들던 것) — config/fastlio_leg_nurigwan_derange30.yaml.
- **콘 감지 항상 켬** `src/obstacle_fusion_node.py`: 벽(wall_cloud) + L1(바닥 위 8~35 cm, X자 경사로 마스크 `maps/ramp_mask.npz` 밖, 3장 이상, 벽 20 cm 밖) +
  카메라 콘(`/mission/cone_cam`) → **다리 기구학 odom** 기준 2초 기억 → `/mission/obstacle_cloud`. 17:56 재생: 정답 콘 6개 모두 2~5 cm 안, CPU 4%.
- **카메라 콘**(내부 PC `onboard_vision/traffic_light_trt_node.py`): 같은 YOLO 추론(5 Hz)으로 콘 상자 + 뎁스(가까운 면 25% 백분위) · 주황+뎁스(`cone_cam_core.py`).
  신호등은 정지선에서만 판정(+ 뎁스 6 m 필터). 카메라는 컬러+뎁스로 켬(`jetson_vision.sh start`).
- **러너** `src/real_course_runner.py`: 경로 파일 순서(route_v1) · stopline 접근 STATICWALK · GREEN 1.0초 · 벽·콘 붙음 복구(BACKOFF) ·
  **콘 구간(rabain~rabaout) ego 여유 0.20**(그 밖 0.30) · `--start-at N` 이어가기 · `/mission/runner_status` · `RUNNER_*` 환경변수로 작전 값.
  (다른 세션의 신호 타임아웃 10초·도착 후 제자리 회전 ROTATE 는 그대로 둠)
- **ego_planner(junsu 빌드)**: 실행 중 `inflate_radius` 변경, `grid_res` 파라미터(기본 0.1 m).
- **주행 스택** `tools/go2_full_stack.sh`: 위 lifecycle + 5단계 obstacle_fusion_node + ego 입력 obstacle_cloud.
- **시뮬레이션** `sim/run_sim.sh`: `SIM_FUSE=1` 이면 합치기 노드 경유. 벽 `sim/sim_walls_fix1105_clean.npy`(경기장 안 작은 덩어리 8개 지움 — 14:59 녹화 때 통로 한가운데 있던 사람·물건),
  콘 시험 `sim_walls_fix1105_clean_cones4.npy`.
- 경로 편집기 `tools/route_editor.py`(한글 자판 키·저장 버튼·자동 저장), 지도 `maps/route_map.json`.

## 시뮬레이션 결과(경로 route_map.json, 고친 벽에서 작은 덩어리 지운 판, 합치기 노드, 콘 구간 여유 0.20)
| 설정 | 콘 없음 | 콘 4개(콘 통로 엇갈림) |
|---|---|---|
| ego 격자 0.1 · 가까운 점 0.25 | 완주 · 접촉 4.3초 | 완주 · 접촉 10.4초 · 바깥 벽 넘음 8번 |
| 0.1 · 0.12 | 완주 · 3.7초 | 완주 · 11.3초 · 콘 중심 0.07 m |
| **0.05 · 0.12 (junsu 기본)** | **완주 · 4.1초 · 110초** | **완주 · 1.8초 · 103초 · 콘 중심 0.20 m** |
정지선 2곳 규정 ✔ · 주차 6.4초 ✔(모든 판). ★ 콘 중심 0.20 m 는 로봇 몸(반폭 0.155)+콘(반지름 0.13) 기준으로 스칠 수 있는 거리.
ego 새 파라미터: `min_range_xy`(원래 0.25 고정) · `grid_res`(원래 0.1 고정) · 실행 중 `inflate_radius`.

## 아직(로봇이 있어야)
1. `ssh-copy-id unitree@192.168.123.18` → `onboard_vision/deploy_onboard_vision.sh all`(→ ~/junsu_vision, INT8 엔진 약 10분, 로봇 엎드린 상태)
2. 정지선에서 `python3 onboard_vision/check_tl_live.py 20`(신호 5초 주기 확인)
3. 라이다 뒤 10~15 cm 가림 물체 확인(뒤쪽 62° 안 보임)
4. 격자 0.05 에서 ego CPU 확인(상태판 옆 `top`)
5. 고친 FAST-LIO 로 좌표 다시 확인 · exit 는 Out Line(x −0.51~−0.57) 밖 몸 전체 — route_map.json 의 exit(−0.83, 4.06)는 뒤끝이 선 안에 남음

## 11-05 오전 추가
- **신호등·콘 모델 교체 `tl_cone_1104`**(팀원 11-04 학습, 경기장 사진 1500장 포함, 4클래스 0 빨강·1 초록·2 unknown·3 콘). 받은 파일은 OpenVINO(인텔용)라 젯슨 GPU 에서 못 씀
  → `onboard_vision/models_1104/` 에서 가중치를 꺼내 ONNX 로 되살림(원본과 상자 707/707 일치, 점수 차 0) → 젯슨에서 TensorRT INT8 로 다시 양자화.
  모델 고르기 `TL_MODEL=tl_cone_1104`(기본) | `tl5cone`(예비). 엔진 없으면 다른 모델 엔진으로 자동 대체. 콘 번호는 출력 모양으로 자동(6클래스 5 · 4클래스 3).
  짧은 시험: `onboard_vision/deploy_onboard_vision.sh quick`(INT8 만, 약 7분 + 채점).
- **주차**: 도착·정지 → 회전 → ego 를 제자리 목표로 세움(0.8초) → Sit → 높이로 착석 확인(문턱 0.15 또는 선 높이보다 6 cm 낮음, 재전송 3번, 그래도 안 되면 시간 기준) → 5.5초 → RiseSit → 일어남 확인(재전송 3번, 안 되면 BalanceStand) → 걸음새 다시 → 다음 점.
- **출구**: 몸 전체가 Out Line(x −0.54, 경로 파일 `out_line_x` 로 바꿀 수 있음) 밖 = 중심 x ≤ −0.99 이면 완주·정지. 도착했는데 모자라면 0.18 m/s 로 직접 밀고 나감(앞 0.22 m 막히면 정지, 6초 한도).
- 시험: `src/test_real_course_runner.py` 4h(착석 미확인)·4i(출구) 추가, 전체 PASS. (시뮬레이션은 로봇이 연결돼 있으면 일부러 안 돈다)

## 경기 전 체크리스트
1. 내부 PC: `deploy_onboard_vision.sh fp16`(기본 엔진) → 채점표가 NUC 기준(초록 재현 84 / 정밀 91)과 비슷한지 → `start` → `bench`(실제 카메라 속도)
2. 정지선에서 `python3 onboard_vision/check_tl_live.py 20` — 5초마다 RED↔GREEN 바뀌는지, 전시장 조명 가짜 초록 없는지
3. 주차 칸에서 Sit/RiseSit 한 번 — 러너 로그의 body_height(선 높이·앉은 높이) 확인. 앉은 높이가 0.15 넘으면 `RUNNER_SIT_HEIGHT_THRESH` 조정(상대 판정이 있어 대개 그대로 됨)
4. 출구 선 x 확인(지도 x −0.51~−0.57). 다르면 route_map.json 에 `"out_line_x": 값`
5. 출발 자리·방향을 지도 start 와 맞춤(0.5 m 넘게 다르면 러너가 출발 거부) · 기립 후 30초 안에 출발(뒷다리 과열)
6. 올인원 `tools/go2_ctl.sh` → `a` 전체 켜기 → 상태판 전부 초록 → `m` 모터 → `r` 출발. 비상은 `x`/스페이스
7. 경기 중 Foxglove 끔(센서 데이터 외부 전송 금지)

## 모듈 한눈에 + 실행 명령 (11-05)
| # | 모듈 | 하는 일 | 따로 실행 |
|---|---|---|---|
| 0 | **올인원 관리자** `tools/go2_ctl.sh` | 아래 1~8 켜기/끄기/다시 켜기, 모터, 러너, 작전 변경, 상태판 | `~/ws/junsu/tools/go2_ctl.sh` |
| 1 | XT16 라이다 | `/lidar_points` | 관리자 `1` |
| 2 | go2_bridge(lifecycle) | 라이다·IMU 몸체 좌표, configure→activate | `source tools/go2_bridge_lc.sh` 후 `lc_up go2_cloud_bridge` |
| 3 | FAST-LIO | 위치(`/Odometry`), max_range 15 | 관리자 `3` · 위치만: `tools/go2_lio_only.sh` |
| 4 | wall_filter | 벽 점(`/mission/wall_cloud`) | 관리자 `4` |
| 5 | obstacle_fusion | 벽 + 콘(카메라 기본, L1 선택) → `/mission/obstacle_cloud`, 다리 odom 2초 기억 | 관리자 `5` · 작전 `s` 에서 cam / l1+cam / off |
| 6 | ego_planner(junsu) | 경로·속도(`/cmd_vel`), 실행 중 벽 여유 변경 | 관리자 `6` |
| 7 | 내부 PC 비전 | 카메라(컬러+뎁스) + YOLO(tl_cone_1104, FP16): 신호등(정지선에서만, 6 m 필터) + 콘(5 Hz) | `onboard_vision/deploy_onboard_vision.sh start/stop/status/bench` |
| 8 | cmd_vel_to_sport(모터) | `/cmd_vel` → Move, 직진 쏠림 보정 | 관리자 `m` 켬 / `M` 끔 |
| 9 | 러너 | 경로 파일 순서대로: 정지선(2.5초+초록 1초, 10초 타임아웃) · 콘 구간 · 주차(앉기 5.5초→일어나기) · 출구(선 통과 확인) · 막히면 물러남 | 관리자 `r` 처음 / `R` 이어가기 · 직접: `python3 src/real_course_runner.py maps/route_map.json [--start-at N]` |
| - | 경로 편집기 | 지도 위에서 점·순서 정하기 → `maps/route_map.json` | `~/venv/go2/bin/python3 tools/route_editor.py` |
| - | 정지선 신호 실시간 확인 | 내부 PC 노드 판정을 20초 보기 | `source ~/gosu/env_drive.sh; python3 onboard_vision/check_tl_live.py 20` |
| - | 비전 엔진 만들기·채점 | FP16(기본)/INT8, `/lowstate` 감시하며 | `deploy_onboard_vision.sh fp16` · `quick`(INT8) · `eval8` |
| - | 녹화 | 전 토픽 + 내부 PC 카메라 | `~/go2_record/go2_record_auto.sh 이름` |
| - | 시뮬레이션(로봇 끊었을 때만) | 가짜 로봇+벽으로 전 코스 | `SIM_FUSE=1 ~/ws/junsu/sim/run_sim.sh` |
| - | 녹화로 비전 확인(NUC) | 신호등 거리·콘 | `TL_MODEL=tl_cone_1104 ~/venv/go2/bin/python3 onboard_vision/tl_depth_check.py <카메라폴더> <출력.json>` · `cone_cam_check.py` |

## 11-05 10:50 — 라이다 7.6 Hz 원인 · 경로 생성 교체
- **라이다 7.6 Hz(정상 20 Hz) 원인 = DDS 멀티캐스트 넘침.** `~/.config/go2/cyclonedds.xml`(AllowMulticast default)로 띄우면 점구름(1.4 MB×20 Hz)이 멀티캐스트로
  robot-br 양쪽 포트(enp86s0 은 100 Mb/s)로 나가 링크가 꽉 참(각 포트 12 MB/s 송신 실측) → 발행이 막혀 Hesai 드라이버가 밀림(소켓 45 MB 적체, 패킷 63% 유실).
  로봇 제어 신호·카메라도 같은 선. → junsu 전용 `config/cyclonedds_junsu.xml`(AllowMulticast **spdp** — 발견만 멀티캐스트, 데이터는 유니캐스트)을
  go2_ctl.sh · go2_full_stack.sh · go2_lio_only.sh 가 씀. 공용 설정 파일은 안 건드림. ★ 적용하려면 스택 전부 다시 켜야 함(Q → q → go2_ctl.sh → a).
- **경로 생성 = merge_tmp review ego**(사용자 지시). `ws_ego_review/`(review 소스 복사·빌드, SOURCE.txt 에 커밋). lifecycle(관리자가 configure→activate).
  벽 = `/scan`(브리지 수직 스캔, 경사로·계단 안 막음), 콘 = 합치기 노드가 내는 `/mission/cones`(PoseArray, 콘 여유 0.28 · 밀어내기 0.38), 벽 여유 0.20, 격자 0.05.
  작전 `s` → `9` 로 review/junsu 바꿈, `2` 는 review 벽 여유(다시 켬). 러너는 `RUNNER_EGO_REVIEW=1` 이면 inflate_radius 안 보냄.
