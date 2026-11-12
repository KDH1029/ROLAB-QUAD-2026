# Jetson(로봇 본체) 실행 & PC 디버깅 연동 가이드

Unitree Go2 온보드 **NVIDIA Jetson Orin**에서 실제 제어 노드를 실행하고, 무선(WiFi)으로 연결된 **개발 PC**에서 실시간 모니터링, RViz2 시각화, 파라미터 튜닝을 수행하는 완벽한 연동 가이드입니다.

---

## 🏗️ 시스템 아키텍처 개요

```text
  ┌──────────────────────────────────────────────┐
  │         NVIDIA Jetson Orin (로봇 온보드)      │
  │  - Hesai XT16 라이다 드라이버 (/velodyne_points)│
  │  - Go2 IMU 및 보행 제어기 (/imu/body, /cmd_vel)│
  │  - wall_runner_node (20Hz 고속 실시간 제어)   │
  └──────────────────────┬───────────────────────┘
                         │ 동일 WiFi 네트워크 (ROS 2 DDS)
                         │ (ROS_DOMAIN_ID=42 일치)
  ┌──────────────────────▼───────────────────────┐
  │               개발 PC (노트북/데스크톱)        │
  │  - RViz2 시각화 (GPU 부하를 PC가 전담)         │
  │  - patrol_monitor.py (컬러 TUI 대시보드)      │
  │  - ros2 param set (원격 무중단 파라미터 튜닝)  │
  │  - 비상 정지 / 재시작 명령 (/patrol/cmd)       │
  └──────────────────────────────────────────────┘
```

> **💡 핵심 장점**:  
> 무거운 RViz 렌더링을 개발 PC의 GPU가 전담하므로, Jetson Orin의 CPU/GPU 자원은 로봇 보행 제어와 20Hz RANSAC 연산에만 100% 집중되어 랙(Lag)이 발생하지 않습니다.

---

## 1단계: 네트워크 및 ROS 2 DDS 환경 설정

Jetson과 PC가 **동일한 공유기(WiFi 또는 무선 라우터)**에 연결되어 있어야 합니다.

### 1) Jetson과 PC 양쪽 모두에 `ROS_DOMAIN_ID` 설정
두 기기의 `~/.bashrc` 맨 아래에 동일한 ID를 추가합니다:
```bash
# Jetson과 PC 터미널에서 각각 실행:
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

### 2) 통신 확인 (PC 터미널에서)
Jetson에서 라이다나 노드가 켜져 있을 때, PC에서 다음 명령어를 쳐서 토픽 목록이 보이면 연결 성공입니다:
```bash
ros2 topic list
```

> **⚠️ 필수 주의사항 (시간 동기화)**:  
> ROS 2는 메시지 타임스탬프가 1초 이상 차이 나면 메시지를 버립니다. Jetson과 PC의 시간이 맞지 않다면 시간을 맞춰주세요:
> ```bash
> # Jetson에서 (인터넷이 연결된 경우):
> sudo systemctl restart systemd-timesyncd
> # 또는 현재 시간 수동 동기화:
> sudo date -s "$(ssh user@pc_ip 'date -u -Iseconds')"
> ```

---

## 2단계: PC에서 Jetson으로 최신 코드 전송

PC에서 작성/수정한 패키지를 Jetson으로 단 한 줄로 동기화합니다.

```bash
# PC 터미널에서 실행 (Jetson의 IP와 계정명을 입력):
rsync -avz --exclude 'build' --exclude 'install' --exclude 'log' \
  ~/full_passive/go2_patrol_ws/src/go2_wall_patrol \
  unitree@<JETSON_IP>:~/go2_patrol_ws/src/
```

---

## 3단계: [Jetson] 노드 빌드 및 실행

Jetson에 SSH로 접속하여 노드를 실행합니다.

```bash
# 1. Jetson SSH 접속
ssh unitree@<JETSON_IP>

# 2. 워크스페이스 빌드
cd ~/go2_patrol_ws
colcon build --symlink-install --packages-select go2_wall_patrol
source install/setup.bash

# 3. 벽면 추종 자율 주행 노드 실행
ros2 launch go2_wall_patrol patrol.launch.py
```

---

## 4단계: [PC] 실시간 디버깅 및 시각화

이제 Jetson은 손대지 않고, **PC에서 편안하게 모니터링과 파라미터 수정**을 수행합니다.

### ① 실시간 컬러 TUI 대시보드 모니터 (PC 터미널 1)
```bash
# PC 터미널에서:
cd ~/full_passive/go2_patrol_ws
source install/setup.bash
ros2 run go2_wall_patrol patrol_monitor.py
```
* FSM 상태, RANSAC 인라이어 개수, 벽면 거리, IMU 각도 오차, 속도 명령, 신호등 상태가 실시간 10Hz로 컬러 갱신됩니다.

### ② RViz2 원클릭 벽면 3D 시각화 (PC 터미널 2)
패키지에 미리 설정된 RViz 설정 파일을 바로 엽니다:
```bash
# PC 터미널에서:
rviz2 -d ~/full_passive/go2_patrol_ws/src/go2_wall_patrol/rviz/patrol.rviz
```
* **초록색 선**: RANSAC이 추출한 왼쪽 외곽 벽
* **빨간색 선**: RANSAC이 추출한 앞벽
* **포인트클라우드**: Hesai XT16 3D 점군 (바닥 타 로봇 필터링 상태 확인)

---

## 5단계: [PC] 주행 중 무중단 파라미터 튜닝

로봇이 Jetson에서 주행 중일 때, **PC 터미널에서 명령어를 치는 즉시 Jetson 노드의 파라미터가 실시간 변경**됩니다!

```bash
# [PC 터미널 3]

# 1. 1번 길의 왼쪽 벽 유지 거리를 0.50m -> 0.45m로 변경
ros2 param set /wall_runner_node path_1.target_left_dist 0.45

# 2. 2번 길(장애물 구간)의 왼쪽 벽 유지 거리를 0.50m -> 0.55m로 변경
ros2 param set /wall_runner_node path_2.target_left_dist 0.55

# 3. 직진 순항 속도를 0.45m/s -> 0.50m/s로 즉시 변경
ros2 param set /wall_runner_node cruise_speed 0.50

# 4. 꼭짓점 앉아있는 시간을 3.0초 -> 2.0초로 단축
ros2 param set /wall_runner_node sit_duration 2.0

# 5. 코너 90도 회전 속도를 0.50 -> 0.60 rad/s로 빠르게 변경
ros2 param set /wall_runner_node turn_speed 0.60
```

---

## 6단계: [PC] 긴급 제어 및 원격 명령

로봇이 이상하게 주행하거나 긴급 정지가 필요할 때 PC에서 명령을 보냅니다:

```bash
# [긴급 정지] 모터 멈추고 IDLE 상태로 전환
ros2 topic pub --once /patrol/cmd std_msgs/msg/String "{data: 'STOP'}"

# [주행 재개] IDLE 상태에서 다시 주행 시작
ros2 topic pub --once /patrol/cmd std_msgs/msg/String "{data: 'START'}"

# [초기화] 시작점(corner_a / path_1)으로 리셋
ros2 topic pub --once /patrol/cmd std_msgs/msg/String "{data: 'RESET'}"
```

