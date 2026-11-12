# 11-04 밤 기반기술 작업 보고 (night_20261104)

> 규칙: 이 폴더에만 썼다. 팀원 코드·설정(gosu_slam, lab, Rgosu, fastlio_ws)과 v_gosu 원본은 읽기만. 재생 시험은 도메인 80·81, lo 전용, robot-br DOWN 확인.

## 결론 먼저
1. **백업 위치추정 v2 완성·검증** — GICP + 견고 지면표. 공정 시험(앞 270초 지도 → 뒤 270초) xy p95 0.099→**0.050 m**, 최대 0.282→**0.086 m**, 경사 위 p95 0.23→**0.054 m**, 거부 9.5%→**0%**.
   무모터 재생(270초에 FAST-LIO 강제 종료): 0.5초 만에 전환, 위치 변화 0.019 m, cmd_vel 20 Hz 유지, 끝까지 정합 거부 0.
   → 경사로 오차의 진짜 원인은 정합기가 아니라 **지면표가 천장(2.23 m)을 바닥으로 낸 것**(경기장 지도에서도 똑같이 터질 수 있던 버그).
2. **경사로·계단 헛장애물**: ego_planner 규칙은 X자 구간 표본의 **16.4%** 에서 로봇이 실제로 걸어간 길 위에 장애물을 찍는다(평지 4.9%).
   지형 모양만으로 장애물을 고르면 헛장애물은 0.6% 로 줄지만 **벽을 절반 놓친다** → 채택 안 함. ★(결과 대기) 거부권 방식…
3. **D435i 외부파라미터** 첫 추정: 몸체 기준 x 0.466 / y 0.010 / z 0.113 m, 아래 34.0°, roll −2.2°. 홀·짝 교차검증 1.2 cm·0.25° 일치.
   깊이 점의 90% 가 지도 3 cm 안. 카메라 시계 오프셋은 재부팅마다 바뀌므로 노드가 온라인으로 잡게 했다.
4. ★(결과 대기) KISS-ICP …

## 자료조사 — 버전 맞는 것 고르기 (NUC Jazzy·x86·NVIDIA 없음 / 로봇 PC Foxy)
| 후보 | 버전·상태 | 판단 |
|---|---|---|
| small_gicp (Koide) | 1.0.1, pip, MIT | **채택** — 백업 정합기. 2.4 ms/회 |
| KISS-ICP | 1.3.0, pip, MIT | 평가 — 라이다 단독 오도(IMU 없음) |
| CMU terrain_analysis (autonomous_exploration_development_environment) | humble-jazzy 브랜치, Ubuntu 24.04 Jazzy 시험됨 | 참고 — '지면 대비 높이' 개념. 스택 통째(상태추정 토픽 규약)라 이식 부담. 우리 구조엔 거부권 노드가 더 가벼움 |
| elevation_mapping_cupy (ETH) | CUDA(CuPy) 필수, ROS1 noetic 중심 | **제외** — NUC 에 NVIDIA 없음 |
| spatio_temporal_voxel_layer | Jazzy 패키지 있음(미설치, apt=sudo) | **제외** — Nav2 costmap 층. 우리 플래너는 ego_planner |
| grid_map 2.2.2 (Jazzy) | 설치됨 | 필요 시 시각화용. 판정 핵심은 자체 구현(5 cm 격자) |
| Unitree 공식 /utlidar/height_map_array | 로봇이 계산, 6 cm 128×128 | 평가 — 단독 판정은 헛장애물 19%로 나쁨 |

출처: CMU 탐사 환경(https://www.ri.cmu.edu/publications/autonomous-exploration-development-environment-and-the-planning-algorithms , https://arxiv.org/abs/2110.14573),
elevation_mapping_cupy(https://github.com/leggedrobotics/elevation_mapping_cupy), STVL(https://docs.ros.org/en/jazzy/p/spatio_temporal_voxel_layer)
