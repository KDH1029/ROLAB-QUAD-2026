# research — 연구·검증 코드

> 대회 미션 플래너에 직접 들어가지 않았지만, 결론의 근거가 된 실험 코드다.
> git 저장소가 아니던 작업공간에서 소스·설정·문서만 복사했다(300 KB 이하, 로그·결과 csv·지도·녹화·가상환경·외부 FAST-LIO 포크 사본 제외). 이력은 없다.
> 각 폴더 안의 `README.md`·`RESULT(S).md` 가 원래 실험 기록이다.

## 한눈에
| 폴더 | 시기 | 핵심 결과 | 다시 쓸 만한 것 |
|---|---|---|---|
| `night_20261104/` | 11-04 | 백업 위치추정 v2 p95 0.050 m, 경사로 헛장애물 13.9 → 1.0% | 경사로 거부권 C++ 노드, 다리 roll·pitch 로 세운 스캔의 3D 정합, D435i 외부파라미터 보정 |
| `v_gosu/` | 10월 | 전 토픽 녹화 + 재현 조건 기록, 백업 위치추정 v1, XT16 대 L1 지도 비교 | `tools/record_all.sh`, `backup/` |
| `lab_visual/` | 09월 | 3D ICP 다단 해상도로 수렴 범위 확대(0.5 m / 5° 100%), 사전 장애물 층 자유공간 9.9 → 46.1 m² | `tools/make_obstacle_layer.py`, `tools/align_obstacle_layer.py`, `src/icp3d.py` |
| `gosu_slam/` | 09-30 | 사전 지도 없이 SLAM 만으로 달리는 구성 | 런치·설정 예시 |
| `archbench/` | 10-08 | FAST-LIO OpenMP 끄면 스캔당 지연 1.5배 | 지연 측정 도구 |
| `go2_3d/` | 08~09월 | 초기 지도 메타데이터, SC-PGO 최적화 전·후 궤적 | SC-PGO 효과 감사 자료 |
| `go2_record/` | 11-04~05 | 대회장 녹화 자동화 최종판: 묻는 것 없이 점검 → 시계 맞춤 → 카메라 → 녹화, 실패 재시도·문제/해결 기록 | `go2_record_auto.sh`, 시각 보정 `go2_clock_probe.py`·`go2_camera_bag_fix.py` |

## night_20261104 — 백업 위치추정 v2 · 경사로 · 카메라 보정
- **`backup_v2/`** — FAST-LIO 가 이상할 때 넘겨받는 백업 위치추정
  - 다리 odom(150 Hz) 증분으로 예측한다.
  - XT16 원본 스캔을 다리 roll·pitch 로 수평 세운 뒤 3D 지도에 x·y·yaw 를 정합한다(small_gicp, 5 Hz).
  - 주요 파일: `backup_loc_node_v2.py`, `backup_core_v2.py`(ROS 무관), `gicp_matcher.py`
  - `robust_ground.py`: 바닥이 안 찍힌 칸에서 천장을 바닥으로 오인하던 문제를 막는다.
  - `frame_align.py`: 지도 좌표 ↔ 오늘 FAST-LIO 좌표 정렬
  - `odom_mux_node_v2.py`: 전환 규칙 — 0.5초 무소식, 2 m/s, 백업과 0.5 m·1초 차이
  - 런치 `launch/full_nav_slam_backup_v2.launch.py`, 재생 확인 `replay_check_v2.sh`, 시험 `tests*/`
  - 결과: [`backup_eval/RESULTS.md`](night_20261104/backup_eval/RESULTS.md)
  - **실기 전환은 하지 않았다.** 출발 직후 튐이 정렬값에 흡수되는 문제가 남아 그림자 모드만 썼다.
- **`ws/src/terrain_veto/`** — 경사로·계단 헛장애물 거부권(C++ 노드 `terrain_veto_node.cpp`)
  - 발밑 평면에서 시작해 이어진 매끈한 면의 점만 계획기 입력에서 지운다. 격자에 L1·깊이를 더한다.
  - 평가: `ramp/eval_veto*.py`
  - 낭떠러지 보완: `ramp/cliff/`(`cliff_block.hpp`, `RESULTS.md`)
- **`vision/`** — D435i 외부파라미터 보정(`calib_depth.py`, 결과 `CALIB.md`: 몸체 x 0.466 · z 0.113 m, 아래 34°)
  - 이 값은 11-04 기준이다. 11-06 에는 카메라가 거의 수평으로 바뀌어 있었다 → 쓰기 전에 정지 20초 녹화로 다시 확인할 것.
  - `yolo_depth/`: YOLO 상자 → 깊이 → 3D 콘 위치

## v_gosu — 녹화와 백업 v1
- **`tools/record_all.sh <이름> "<메모>"`** — 전 토픽 녹화(zstd, 5분 분할)
  - 다시 만들 수 있는 무거운 파생 토픽만 뺀다.
  - 끝나면 `meta.txt`(메모·시각 동기·소켓 버퍼·설정 md5·토픽 목록)를 남긴다.
  - 시작 전 라이다 드라이버 수와 발행자 수를 점검한다(드라이버 중복 사고 방지).
- `tools/map_replay.sh`, `slam_replay_check.sh`, `env_replay.sh`: 격리된 재생(도메인 분리 + 루프백)
- `backup/`: 백업 위치추정 v1(평평한 바닥 가정, 경사로에서 최대 0.46 m → v2 에서 개선)
- `eval/`: XT16 대 L1 지도 비교(`maps/render_v2.py` — 같은 2 cm 복셀·복셀당 2회 이상으로 그려야 공정하다)

## go2_record — 대회장 녹화 자동화 (최종판)
- `go2_record_auto.sh <이름> "<메모>"`: 어디서든 이름만으로 실행한다. 묻는 것 없이 진행하고, 실패는 기록만 남기고 계속한다.
  - 순서: 점검(로봇 응답·로봇 PC 1 Gbps·디스크 100 GB 이상) → 시계 차이 측정·맞춤 → 카메라(848×480×30) → XT16(18 Hz 미만이면 재시작) → 녹화
  - 녹화 중 15초마다 감시하고 자동 복구한다. 끝나면 `problems.txt`(원인·해결 명령)를 남긴다.
- 카메라는 내부 PC 에 로컬로 녹화한다. 나중에 `go2_record_fetch.sh all` 로 옮기면서 시각을 보정한다(잔차 0.01 ms, 내부 PC 시계 표류 −35 ppm).
- `go2_ssh.sh`: 로봇 PC 접속 재사용(비밀번호는 처음 한 번 사람이 입력), `go2_record_check.py`: 녹화 점검
- 경로: 원래 `~/go2_record` 에 두고 `~/bin` 에 연결해 썼다. 다른 곳에 두면 스크립트 머리의 경로를 바꾼다.

## lab_visual — 9월 재생 실험실 도구
- `src/icp_localization.py`, `src/icp3d.py`: 3D 점군을 3D 지도에 직접 정합. 4단 해상도(0.80/0.40/0.20/0.10 m)로 수렴 범위를 넓혔다.
- `tools/make_obstacle_layer.py`: 기울어진 지도에서 사전 장애물 층 다시 만들기
  - 바닥 평면 맞춤 + 평면 위 0.6~1.6 m + 칸당 30회 이상 찍힌 붙박이만 남긴다.
- `tools/align_obstacle_layer.py --from-tf`: 계획기 좌표(FAST-LIO 부팅 자리)에 층을 맞춘다. 계획기 로더는 좌표 변환을 하지 않는다.
- `tools/why_stopped.py`, `diag_tf*.py`, `find_start_pose.py`: 멈춘 원인·TF·출발 자세 진단

## 재사용할 때
- 대부분 `~/ws/<원래 폴더>` 절대 경로를 쓴다:
  ```bash
  grep -rn '~/ws/\|/home/' <폴더>
  ```
- 재생 스크립트는 격리(별도 DDS 도메인, 로봇이 응답하면 거부)를 전제로 한다. 이 장치를 빼고 쓰지 말 것.
- 결과 수치는 같은 녹화로 지도를 만들고 시험한 경우가 있어 낙관적일 수 있다(각 RESULTS 문서에 조건이 적혀 있다).
