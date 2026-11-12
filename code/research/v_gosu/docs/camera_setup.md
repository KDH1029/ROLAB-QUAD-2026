# D435i 장착 · 보정 · 녹화 절차 (VLIO 비교용)

VLIO 는 카메라-라이다 외부파라미터(Rcl, Pcl)와 시각 동기가 맞아야 효과가 난다.
틀린 외부파라미터는 라이다만 쓸 때보다 나쁜 결과를 낸다. 그래서 녹화를 두 종류 한다.

## 0. 준비 (NUC)

```bash
sudo apt install ros-jazzy-realsense2-camera
```
- D435i 는 NUC 의 USB-A(3.x) 포트에 꽂는다. USB-C 로 IMU 까지 켜면 2초마다 끊겼다(10-03 기록).
- 카메라 IMU 는 끈다. IMU 는 로봇 /lowstate(500 Hz)를 쓴다.
- 시각: realsense 기본값 `global_time_enabled` 가 카메라 시각을 호스트 시계로 맞춘다.
  라이다(cloud_bridge)와 IMU(imu_bridge)도 호스트 시계라 같은 기준이 된다.

## 1. 장착

- 위치: 몸통 앞쪽 위, 렌즈가 **아래로 15~25° 숙이게**. 수평(0°)이면 바닥이 0.87 m 앞부터 보인다(09-21 계산).
  숙이면 바닥·계단 모서리·점선이 화면에 많이 들어와 시각 특징이 는다(트랙 벽은 무늬 없는 흰색).
- 단단히 고정한다. 녹화 후 한 번이라도 움직이면 보정을 다시 해야 한다.
- 줄자로 대략값을 적어 둔다(라이다 중심 → 카메라 렌즈 중심, x·y·z, 숙인 각도). 보정의 초기값이 된다.

## 2. 보정용 녹화 (정지 상태, 5분)

```
로봇     서서 정지 (모터 켠 채 움직이지 않음) 또는 엎드린 채
장소     벽 모서리·계단·물건이 많은 곳 (특징이 많을수록 좋다)
동작     로봇을 손으로 천천히 좌우 30°, 상하 10° 돌려 준다 (보정 도구가 여러 각도를 원함)
명령     ~/ws/v_gosu/tools/record_all.sh calib "D435i 보정용, 정지+천천히 회전"
```
보정 도구 후보: `direct_visual_lidar_calibration`(koide3, ROS 2, 회전식 라이다 지원, 표적 없이 가능).
결과 Rcl·Pcl 을 `~/ws/v_gosu/config/livo_go2.yaml` 의 `extrin_calib` 에 넣는다.

## 3. 비교용 녹화 (주행, 10~15분)

```
장소     누리관 X자 구간 (계단·경사, 대회 트랙과 같은 구조)
경로     같은 계단·경사를 최소 3번 왕복. 시작·끝은 같은 자리에서 10초 정지
속도     평소 주행 속도 (충격 조건을 그대로 재현해야 비교가 의미 있다)
명령     ~/ws/v_gosu/tools/record_all.sh stairs "X자 계단·경사 3회 왕복, D435i 숙임 20°"
```
카메라 실행(별도 터미널)은 `record_all.sh` 맨 위 주석의 명령을 그대로 쓴다.

## 4. 녹화 후 비교 (내가 한다)

```
~/ws/v_gosu/eval/run_compare.sh fastlio <태그>      # 기준선
~/ws/v_gosu/eval/run_compare.sh livo    <태그>      # FAST-LIVO2 (img_en 0, 1 둘 다)
python3 ~/ws/v_gosu/eval/analyze.py out/*.npz
```
지표: 경로/다리오도 비, 경기장 범위, 충격 구간 속도·고도 튐 배율, 재방문 바닥 높이 σ.
