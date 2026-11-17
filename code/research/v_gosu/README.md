# v_gosu — 비전 결합 오도메트리 실험 프로젝트

```
생성      2026-10-29
목적      계단·경사·좁은 통로에서 FAST-LIO 단독보다 튼튼한 오도메트리를 찾는다
원칙      기존 코드(~/ws/gosu_slam, ~/ws/Rgosu, ~/ws/lab, ~/ws/fastlio_ws)는 읽기만 한다
          같은 bag 으로 FAST-LIO 와 맞붙여 수치로 이긴 것만 채택한다
```

## 왜 시작하나 — 지금 FAST-LIO 의 약점 (실측)

| 위험 | 근거 | 상태 |
|---|---|---|
| 계단 충격 | arena_0921 충격 상위 10% 구간에서 속도 튐 6.66배, 고도 튐 4.95배. 재방문 시 약 10 cm 어긋나 평평한 바닥 높이 표준편차 104 mm | 실측 |
| 좁은 통로 | 대회 트랙은 폭 약 1 m, 벽 높이 약 0.6 m 통로. 평행한 벽 두 장과 바닥만 보이면 진행 방향 구속이 약해진다 | 미측정 (누리관 경기장은 더 넓음) |
| IMU 시각 | 팀 브리지가 IMU 를 호스트 now() 로 찍음. dt 표준편차 1.78 ms, 최대 47.6 ms | 실측 |
| 실기 | NUC 통째 SLAM 은 무모터 재생만 했다 | 미검증 |

경사·계단의 높이 변화 자체는 3D LIO 에 문제가 아니다. 문제는 **발이 디딜 때의 충격**과 **특징 없는 통로**다.

## 후보

| | 방식 | 센서 | 장점 | 걱정 |
|---|---|---|---|---|
| A. FAST-LIVO2 | 라이다·카메라·IMU 한 필터(ESIKF) | XT16 + D435i 컬러 + /lowstate | FAST-LIO 계열이라 설정 재사용. 퇴화 환경용 | 카메라-라이다 외부파라미터·시각 동기 필요. 흰 벽은 시각 특징이 적다 |
| B. cuVSLAM (Isaac ROS) | 스테레오 시각-관성, GPU | D435i 적외선 스테레오, 젯슨 | 젯슨 GPU 를 처음으로 제대로 씀. 라이다와 독립된 교차 검증 | 젯슨(Humble)에서 돌고 결과를 NUC 로 보내 결합해야 함 |
| C. 다리 기구학 결합 LIO | LIO + 관절각·접지 구속 | XT16 + /lowstate | 새 센서 없음. 충격 구간을 직접 겨냥 | 기성 ROS2 구현 확인 필요 (Leg-KILO 계열) |

## 먼저 할 일 — 카메라 든 녹화본 확보 (10-30, 1개)

지금 있는 bag 13개 전부 카메라 토픽이 없다. 비교하려면 **새로 녹화**해야 한다.

> **10-30 녹화 `~/bags/gosu_20261030_001126`** (37.1 GiB, 542초) — ★ 카메라 든 첫 녹화본. XT16 20 Hz 무손실, 뎁스·적외선 30 Hz, 컬러 1280×720 17 Hz.
> 카메라 stamp 는 로봇 PC 1970 시계라 `meta.txt` 의 오프셋 식으로 NUC 시각에 맞출 것.
>
> **10-29 녹화 `~/bags/gosu_20261029_234318`** (2.2 GB, 445초, 전 토픽) — 그래도 **카메라 없음**.
> 로봇 전원이 한 번 끊긴 뒤 로봇 PC 의 D435i 노드가 다시 안 떠서 `/camera/...` 가 0개였다.
> XT16 도 9.4 Hz 로 들어갔다(09-21 은 20 Hz, 프레임은 완전. 드라이버 2개 동시 실행 추정). 그래서 VLIO 비교에도, 오도메트리 기준선에도 쓰지 않는다.
> 자세한 내용은 `~/go2_handover_20260930/go2_인수인계_20260930/README_인수인계.md` 맨 위 ★.
> D435i 는 NUC 가 아니라 **로봇 내부 PC** 에 꽂혀 있고 토픽 이름은 `/camera/color/image_raw` 처럼 `/camera/camera/` 가 아니다.

```
장소     누리관 X자 구간 (계단·경사, 대회 트랙과 같은 구조)
토픽     /lidar_points  /lowstate  /sportmodestate  /utlidar/robot_odom  /utlidar/cloud_base
         /camera/camera/color/image_raw  /camera/camera/infra1/image_rect_raw  /camera/camera/infra2/image_rect_raw
         /camera/camera/color/camera_info  /camera/camera/infra1/camera_info
경로     같은 계단·경사를 최소 3번 왕복 (재방문 일치도를 재야 한다)
도구     tools/record_vlio.sh  (로봇이 움직이므로 사용자 터미널에서 실행)
```

D435i 는 IMU 를 켜면 USB 가 끊긴다(Rgosu/depth_terrain 기록). IMU 는 끄고 로봇 /lowstate 를 쓴다.

## 평가 — 같은 bag, 같은 기준

| 지표 | 뜻 | 현재 FAST-LIO |
|---|---|---|
| 재방문 바닥 높이 표준편차 | 같은 자리를 다시 지날 때 어긋남 | 104 mm |
| 계단 구간 속도 튐 | 충격 순간 자세가 얼마나 튀나 | 6.66배 |
| 왕복 종료점 오차 | 출발점으로 돌아왔을 때 위치 차 | 측정 예정 |
| 처리율·지연 | 20 Hz 유지, 스캔당 지연 | 20.0 Hz, 13.5~14.4 ms |

재생은 DDS 도메인 78 에서 한다(77 은 기존 lab 재생용).

## 채택 규칙

1. 위 지표를 **전부** FAST-LIO 이상으로 낼 것
2. 같은 bag 을 3회 재생해 결과가 흔들리지 않을 것
3. 실기에서 무모터 → 저속 순서로 검증할 것

대회(11-05 예선) 전까지 이기지 못하면 대회는 기존 gosu_slam 으로 간다.

## 진행 상황 (10-29)

| 단계 | 상태 |
|---|---|
| FAST-LIVO2 ROS 2 포팅본 Jazzy 빌드 (`ws_livo`) | 완료. 고친 곳: Sophus 1.22.4 + `SOPHUS_USE_BASIC_LOGGING`, vikit 컴파일 플래그 덮어쓰기, cv_bridge `.hpp`, Velodyne 점 `time` 필드(초)를 팀 브리지 형식에 맞춤. 원본은 각 `.orig` |
| Go2·XT16 설정 `config/livo_go2.yaml` | 완료. 라이다·IMU 값은 실주행 FAST-LIO 와 동일, 카메라 외부파라미터는 자리표시 |
| 평가 도구 `eval/` (run_compare.sh, live_eval.py, analyze.py) | 완료. 도메인 78, 로봇 연결 시 거부 |
| FAST-LIO 기준선 (arena_0921 전체) | 완료 — `docs/results.md` |
| acc_cov 실험 (0.5/1/2/4) | 완료 — 채택 안 함 (지도↑ 궤적↓ 맞바꿈) |
| 후보 C: 다리 속도 결합 FAST-LIO (`ws_legfuse`) | 구현·빌드 완료. 팀 fast_lio 사본에 `leg.*` 매개변수 추가: /utlidar/robot_odom 수평 속도를 라이다 갱신 직후 측정값으로 넣고 χ² 게이트로 미끄러짐 거부. 원본 `laserMapping.cpp.orig` |
| FAST-LIVO2 실행 | **대기** — 자동 모드 안전 검사가 내려받은 외부 프로그램 실행을 막음. 사용자 허용 필요 |
| 카메라 녹화 | **완료 (10-30)** — `~/bags/gosu_20261030_001126`. D435i 는 로봇 내부 PC 에서 켬. 카메라 stamp 오프셋 식은 bag 의 `meta.txt` |

빌드 명령(재현용):
```bash
cd ~/ws/v_gosu/ws_livo && source /opt/ros/jazzy/setup.bash && source ~/ws/livox/ws_livox/install/setup.bash
S=$HOME/ws/v_gosu/deps/sophus_tpl
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$S -DSophus_INCLUDE_DIRS=$S/include -DCMAKE_CXX_FLAGS=-DSOPHUS_USE_BASIC_LOGGING
```

## 폴더

```
src/      후보 소스 (각자 별도 colcon 작업공간으로 빌드)
config/   후보별 설정
launch/   후보별 런치
tools/    녹화·재생·평가 스크립트
bags/     새 녹화본
eval/     평가 결과
docs/     실험 기록
```
