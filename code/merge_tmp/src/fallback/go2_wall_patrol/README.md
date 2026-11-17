# go2_wall_patrol (Unitree Go2 CW RANSAC 외곽 벽면 추종 및 꼭짓점 정밀 정렬 노드)

## 1. 개요
경기장 내 타 로봇 장애물(높이 40~50cm)과 SLAM 드리프트 문제를 완전히 극복하기 위해 개발된 **무지도(Mapless) CW 외곽 벽면 추종 및 꼭짓점 정밀 정렬 순환 주행 ROS 2 Humble 패키지**입니다.

### 🌟 핵심 메커니즘
1. **CW 전용 (Clockwise Only)**: 경기장 외곽(왼쪽 벽)을 따라 시계 방향으로만 주행하도록 특화.
2. **LiDAR Z-Filter (0.50m ~ 1.20m)**: 바닥 및 지상 40~50cm 높이의 타 사족보행 로봇 장애물을 100% 배제.
3. **2D RANSAC Line Fitting (앞벽 & 왼쪽 벽)**:
   - 단순 거리 최솟값이 아닌 2D RANSAC 직선 피팅 및 인라이어 최소제곱법(Least Squares)으로 이상치를 제거하여 mm 단위의 정밀 수직 벽면 거리를 산출.
4. **각 길(Leg 0~3) & 각 꼭짓점(Corner 0~3) 개별 파라미터 분리**:
   - 길 주행 시: 복도별 수평 각도(`leg_drive_yaw_deg`), 왼쪽 거리(`leg_target_left_dist`), 코너 트리거 거리(`leg_front_trigger_dist`)
   - 꼭짓점 정렬 시: 목표 전방 거리(`corner_target_front_dist`), 목표 왼쪽 거리(`corner_target_left_dist`), 목표 각도(`corner_target_yaw_deg`)
5. **꼭짓점 3축($v_x, v_y, \omega_z$) 동시 닫힌루프 정밀 정렬**:
   - 앞 RANSAC + 왼쪽 RANSAC + IMU Yaw를 동시에 맞추어 허용 오차 이내로 수렴.
6. **앉았다 일어나기(Sit-to-Stand) 자세 제어**:
   - 꼭짓점 위치 정렬 완료 시 `SIT` 명령 ➡️ 설정 시간(예: 3초) 대기 ➡️ `STAND` 명령 ➡️ 안정화.
7. **직사각형 맵 30초 주기 신호등(Traffic Light) 대응**:
   - 시작 구간(Leg 0) 및 건너편/맞은편(Leg 2): 신호등 없이 즉시 진행.
   - 바로 앞 구간(Leg 1) 및 나머지 구간(Leg 3): 30초 주기 점멸 타이머 또는 외부 토픽(`/traffic_light`) 확인 후 출발.
8. **90도 제자리 회전 후 다음 길 전진 무한 반복**.

---

## 2. FSM 상태 흐름도 (State Machine)

```text
[ALIGN_INIT] (0° 또는 가장 가까운 90° 단위 기준 각도로 IMU 스냅)
      │
      ▼
┌──► [DRIVE] ────────────────(d_front <= leg_front_trigger_dist[leg])
│      │                        │
│      │ (왼쪽 RANSAC 거리 유지)  ▼
│      │ (IMU 수평 헤딩 고정)    [CORNER_ALIGN] (앞벽 vx + 왼쪽벽 vy + IMU wz 3축 동시 정렬)
│      │                        │
│      │                        │ (허용 오차 충족 후 안정화)
│      │                        ▼
│      │                     [CORNER_SIT] (Go2 앉기 명령 발행 후 sit_duration 대기)
│      │                        │
│      │                        ▼
│      │                     [CORNER_STAND] (Go2 일어서기 명령 발행 후 안정화 대기)
│      │                        │
│      │                        ▼
│      │                     [TRAFFIC_CHECK] ─── (건너편: 즉시 통과 / 신호등 구간: 녹색 대기)
│      │                        │
│      │                        ▼
│      │                     [CORNER_TURN] (다음 길 목표 각도로 제자리 90도 회전)
│      │                        │
│      │                        ▼
│      └───────────────────── [POST_TURN_DWELL] (0.5s 안정화 후 leg = (leg+1)%4로 DRIVE 복귀)
```

---

## 3. 입출력 토픽 규격

### 📥 구독 토픽
| 토픽명 | 타입 | 설명 |
| :--- | :--- | :--- |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | Hesai XT16 라이다 원본 점군 (20Hz) |
| `/imu/body` | `sensor_msgs/msg/Imu` | Go2 내장 IMU 쿼터니언 (500Hz) |
| `/traffic_light` | `std_msgs/msg/String` | (옵션) 외부 신호등 토픽 (`"GREEN"`, `"RED"`) |
| `/traffic_light_state`| `std_msgs/msg/Bool` | (옵션) 외부 신호등 불리언 토픽 (`true`/`false`) |
| `/patrol/cmd` | `std_msgs/msg/String` | 운전 제어 명령 (`"START"`, `"STOP"`, `"RESET"`) |

### 📤 발행 토픽
| 토픽명 | 타입 | 설명 |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Go2 전방향 속도 명령 ($v_x, v_y, \omega_z$) |
| `/patrol/status` | `std_msgs/msg/String` | 실시간 10Hz 디버그 텔레메트리 (JSON) |
| `/patrol/posture` | `std_msgs/msg/String` | 자세 명령 (`"SIT"`, `"STAND"`) |
| `/robot/motion_cmd`| `std_msgs/msg/String` | 로봇 모션 명령 (`"STAND_DOWN"`, `"STAND_UP"`) |

---

## 4. 빌드 및 실행 가이드

### 1) 빌드
```bash
cd ~/go2_patrol_ws
# (또는 현재 워크스페이스: /home/kdh/full_passive/go2_patrol_ws)
colcon build --symlink-install --packages-select go2_wall_patrol
source install/setup.bash
```

### 2) 실행 (단 한 줄)
```bash
ros2 launch go2_wall_patrol patrol.launch.py
```

### 3) 실시간 상태 모니터링
```bash
ros2 topic echo /patrol/status
```
출력 예시:
```json
{
  "state": "CORNER_ALIGN",
  "leg": 1,
  "d_left": 0.50,
  "d_front": 0.65,
  "yaw_deg": -89.92,
  "target_yaw_deg": -90.00,
  "vx": 0.01,
  "vy": 0.00,
  "wz": -0.01,
  "traffic_light": "GREEN",
  "left_inliers": 94,
  "front_inliers": 112,
  "laps": 1
}
```

---

## 5. 파라미터 상세 설정 (`config/patrol_params.yaml`)

| 파라미터 그룹 | 파라미터명 | 기본값 | 설명 |
| :--- | :--- | :---: | :--- |
| **길별 주행 요구치** | `leg_drive_yaw_deg` | `[0, -90, 180, 90]` | 4개 복도별 수평 직진 헤딩 각도 (도) |
| | `leg_target_left_dist` | `[0.50, 0.50, 0.50, 0.50]` | 4개 복도별 왼쪽 RANSAC 벽 유지 거리 (m) |
| | `leg_front_trigger_dist`| `[0.85, 0.85, 0.85, 0.85]` | 코너 정렬 모드로 진입하는 전방 벽 거리 (m) |
| | `leg_slowdown_dist` | `[1.50, 1.50, 1.50, 1.50]` | 감속 시작 전방 거리 (m) |
| **꼭짓점별 정밀 요구치**| `corner_target_front_dist`| `[0.65, 0.65, 0.65, 0.65]`| 각 꼭짓점 정렬 시 목표 전방 거리 (m) |
| | `corner_target_left_dist` | `[0.50, 0.50, 0.50, 0.50]`| 각 꼭짓점 정렬 시 목표 왼쪽 거리 (m) |
| | `corner_target_yaw_deg` | `[0, -90, 180, 90]` | 각 꼭짓점 정렬 시 목표 헤딩 각도 (도) |
| **정밀 정렬 허용 오차** | `align_tol_x` / `y` | `0.04` | 꼭짓점 전방/왼쪽 위치 허용 오차 (4cm) |
| | `align_tol_yaw_deg` | `2.0` | 꼭짓점 각도 허용 오차 (2도) |
| | `align_settle_time` | `0.40` | 오차 조건을 만족해야 하는 지속 시간 (초) |
| **자세 제어** | `sit_duration` | `3.0` | 앉아있는 대기 시간 (초) |
| | `stand_duration` | `2.0` | 일어선 후 안정화 대기 시간 (초) |
| **신호등** | `has_traffic_light` | `[false, true, false, true]`| Leg별 신호등 유무 (건너편 Leg 2는 false) |
| | `traffic_cycle_sec` | `30.0` | 신호등 켜짐/꺼짐 주기 (초) |
| **RANSAC** | `z_min` / `z_max` | `0.50` / `1.20` | 높이 필터링 (지면/다른 로봇 배제) |
| | `ransac_iterations` | `60` | RANSAC 최대 반복 샘플링 횟수 |
| | `ransac_inlier_thresh`| `0.035` | 인라이어 거리 임계값 (3.5cm) |
