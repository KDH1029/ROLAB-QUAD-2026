# Unitree Go2 CW 외곽 순환 주행 디버깅 및 파라미터 튜닝 가이드

본 문서는 실제 경기장(Arena)에서 로봇을 주행시키며 **센서 점검 ➡️ RANSAC 벽면 확인 ➡️ 실시간 모니터링 ➡️ 무중단 파라미터 튜닝 ➡️ 문제 해결**까지 순서대로 진행할 수 있는 실전 가이드입니다.

---

## 📌 목차
1. [1단계: 센서 및 데이터 정상 확인 (LiDAR & IMU)](#1단계-센서-및-데이터-정상-확인)
2. [2단계: 실시간 TUI 모니터 대시보드 실행](#2단계-실시간-tui-모니터-대시보드-실행)
3. [3단계: RViz2 시각화로 RANSAC 벽면 직접 보기](#3단계-rviz2-시각화로-ransac-벽면-직접-보기)
4. [4단계: 노드 재시작 없는 실시간 파라미터 튜닝 (Dynamic Parameter Set)](#4단계-노드-재시작-없는-실시간-파라미터-튜닝)
5. [5단계: 코너 3축 정렬 & 2·4번 길 장애물 세부 튜닝](#5단계-코너-3축-정렬--24번-길-장애물-세부-튜닝)
6. [증상별 문제 해결 가이드 (Troubleshooting)](#증상별-문제-해결-가이드)

---

## 1단계: 센서 및 데이터 정상 확인

로봇 주행 노드를 실행하기 전에 센서 데이터가 정상적으로 들어오는지 확인합니다.

```bash
# 1. Hesai XT16 라이다 점군 토픽 수신 확인 (20Hz)
ros2 topic hz /velodyne_points

# 2. Go2 내장 IMU 토픽 수신 확인 (500Hz)
ros2 topic hz /imu/body

# 3. IMU 쿼터니언 데이터 정상 출력 확인
ros2 topic echo --once /imu/body
```

---

## 2단계: 실시간 TUI 모니터 대시보드 실행

패키지에 내장된 **전용 컬러 TUI 디버그 대시보드**를 실행하면, 현재 주행 상태와 RANSAC 인라이어 개수, 오차를 한눈에 확인할 수 있습니다.

```bash
# 터미널 창을 하나 더 열고 실행:
cd ~/go2_patrol_ws
source install/setup.bash
ros2 run go2_wall_patrol patrol_monitor.py
# (또는 python3 ~/go2_patrol_ws/src/go2_wall_patrol/scripts/patrol_monitor.py)
```

### 🖥️ 화면 출력 예시 및 판독법
```text
======================================================================
  Unitree Go2 CW Patrol Live Debug Dashboard  (Loop # 1)
======================================================================
  FSM State      : [DRIVING]
  Current Course : path_2  ➔  Target Corner: corner_c
  Traffic Light  : [GREEN]
----------------------------------------------------------------------
  [LiDAR 2D RANSAC Wall Tracking]
    Left Wall  :   0.50 m  (Inliers: 115) ➔ 50개 이상이면 매우 양호
    Front Wall :   3.12 m  (Inliers:  98)
----------------------------------------------------------------------
  [IMU Orientation Lock]
    Current Yaw:  -89.8°  |  Target Yaw:  -90.0°  |  Error:   +0.2° (헤딩 오차)
----------------------------------------------------------------------
  [Velocity Command (/cmd_vel)]
    Vx (Forward): +0.45 m/s | Vy (Lateral): +0.01 m/s | Wz (Yaw): -0.01 rad/s
======================================================================
```

---

## 3단계: RViz2 시각화로 RANSAC 벽면 직접 보기

노드 내부에서 피팅된 RANSAC 직선이 **RViz2 마커(`visualization_msgs/Marker`)**로 실시간 브로드캐스트됩니다.

```bash
# RViz2 실행
rviz2
```
* **Fixed Frame**: `laser` 또는 `velodyne` 또는 `base_link`
* **추가할 디스플레이**:
  1. `PointCloud2`: 토픽 `/velodyne_points` (원본 라이다 점군)
  2. `Marker`: 토픽 `/patrol/viz/left_wall` (초록색 선 ➔ 추출된 왼쪽 외곽벽)
  3. `Marker`: 토픽 `/patrol/viz/front_wall` (빨간색 선 ➔ 추출된 앞쪽 벽)

> **💡 확인 팁**:  
> 2번/4번 길 주행 시 벽과 로봇 사이에 장애물이 있을 때, **초록색 선이 장애물 표면에 달라붙지 않고 뒤쪽의 진짜 외곽 벽면에 정확히 그어지는지** RViz에서 바로 확인할 수 있습니다.

---

## 4단계: 노드 재시작 없는 실시간 파라미터 튜닝

`wall_runner_node`는 **동적 파라미터 콜백(`OnSetParametersCallback`)**이 탑재되어 있어, **로봇이 달리는 중에도 노드를 끄지 않고 즉시 파라미터를 수정**할 수 있습니다.

### 실시간 명령어 예시:
```bash
# 1. 1번 길의 왼쪽 벽 유지 거리를 0.50m -> 0.45m로 즉시 변경
ros2 param set /wall_runner_node path_1.target_left_dist 0.45

# 2. 직진 순항 속도를 0.45m/s -> 0.55m/s로 고속 증속
ros2 param set /wall_runner_node cruise_speed 0.55

# 3. 코너 90도 회전 속도를 0.50 -> 0.60 rad/s로 빠르게 변경
ros2 param set /wall_runner_node turn_speed 0.60

# 4. 왼쪽 벽면 거리 유지 반응성(P게인) 조정 (0.8 -> 1.0)
ros2 param set /wall_runner_node kp_dist 1.0

# 5. 꼭짓점 앉아있는 시간을 3.0초 -> 2.0초로 단축
ros2 param set /wall_runner_node sit_duration 2.0

# 6. 신호등 주기를 30초 -> 20초로 변경
ros2 param set /wall_runner_node traffic_cycle_sec 20.0
```

> **영구 저장**:  
> 실시간 튜닝으로 가장 이상적인 값을 찾았다면, [`config/patrol_params.yaml`](file:///home/kdh/full_passive/go2_patrol_ws/src/go2_wall_patrol/config/patrol_params.yaml) 파일을 열어 해당 수치를 적어두시면 다음 부팅 때도 유지됩니다.

---

## 5단계: 코너 3축 정렬 & 2·4번 길 장애물 세부 튜닝

### 1) 꼭짓점 3축 정렬이 너무 오래 걸리거나 덜컹거릴 때
꼭짓점에서는 앞벽($x$), 왼쪽벽($y$), 각도($yaw$) 3축이 모두 오차 범위 안에 들어와야 앉기(`SIT`)로 넘어갑니다.

* **증상**: 로봇이 꼭짓점에서 멈추지 않고 앞뒤/좌우로 미세하게 계속 비비적거림
* **원인**: 허용 오차가 너무 빡빡함 (`4cm` 이내)
* **해결법**: 허용 오차를 `5~6cm`로 살짝 완화
  ```yaml
  align_tol_x: 0.05       # 4cm -> 5cm
  align_tol_y: 0.05       # 4cm -> 5cm
  align_tol_yaw_deg: 3.0  # 2도 -> 3도
  align_settle_time: 0.30 # 0.4초 -> 0.3초 안정화 후 즉시 앉기
  ```

### 2) 2번·4번 길 장애물(벽과 로봇 사이) 튜닝
* **증상**: 로봇이 장애물 옆을 지날 때 오른쪽으로 살짝 밀려남 (장애물을 벽으로 오인)
* **원인**: RANSAC이 장애물 표면 포인트를 벽으로 피팅함
* **해결법**:
  1. `prefer_outer_wall: true` 확인
  2. `ransac_iterations: 100`으로 증가 (더 많은 샘플링으로 뒤쪽 외곽벽 검출률 극대화)
  3. 장애물 표면 배제: `wall_runner_node.cpp`의 외곽벽 필터링 임계치 유지

---

## 6단계: 증상별 문제 해결 가이드 (Troubleshooting)

| 현상 | 의심 원인 | 해결 조치 |
| :--- | :--- | :--- |
| **바닥을 기어가는 다른 팀 로봇을 벽으로 인식함** | 라이다 Z 높이 필터 하한이 낮음 | `z_min`을 `0.50` ➔ `0.55` 또는 `0.60`으로 올려 다른 로봇(40~50cm) 완전 배제 |
| **직진할 때 뱀처럼 좌우로 흔들림 (Fishtailing)** | IMU 헤딩 게인이 약하거나 거리 게인이 과도함 | `kp_yaw`를 `1.5` ➔ `2.0`으로 상향 / `kp_dist`를 `0.8` ➔ `0.6`으로 하향 |
| **왼쪽 벽과의 거리를 너무 늦게 맞춤** | 거리 게인이 낮음 | `kp_dist`를 `0.8` ➔ `1.1`로 상향 |
| **코너에서 90도 회전할 때 발이 미끄러져 덜 돎** | 모터 회전 최저 각속도 부족 | `min_turn_speed`를 `0.15` ➔ `0.22` rad/s로 상향 |
| **신호등이 없는 3번 길인데 멈춰 서서 대기함** | 파라미터 설정 오류 | `path_3.has_traffic_light: false` 설정 확인 |
| **센서 타임아웃 경고와 함께 로봇이 멈춤** | 라이다/IMU 케이블 접촉 불량 또는 토픽 딜레이 | `ros2 topic hz /velodyne_points` 및 `sensor_timeout` (기본 0.6초) 점검 |
| **주행 중 즉시 강제 비상 정지하고 싶을 때** | 긴급 정지 명령 | `ros2 topic pub --once /patrol/cmd std_msgs/msg/String "{data: 'STOP'}"` |

