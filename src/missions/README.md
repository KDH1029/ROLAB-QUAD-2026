# Mission Packages (`src/missions/`)

2026 로보월드 4족보행 로봇경진대회(450점 만점 규정) 수행을 위한 온보드 C++ 비전 인식 및 미션 시퀀스 제어 도메인입니다.

## 포함된 패키지

1. **[`mission_vision`](mission_vision/)** (C++ 비전 및 RGB-D/LiDAR 센서 융합)
   - `traffic_light_node`: 지상 1.3m 높이(26×30cm 본체) 신호등의 독립 비동기 Red/Green 점등 상태 판별 (`/mission/traffic_light`)
   - `parking_marker_node`: 주차구역 벽면 10×10cm QR 코드 및 문자 'P' 표식 + 전방 벽 거리 정합 판별 (`/mission/parking_ready`)
   - `cone_detector_node`: 높이 23cm, 밑면 14×14cm 소형 주황색 라바콘의 RGB-D 역투영 + `/scan` 클러스터 3D 위치 추정 (`/mission/cone_poses`)

2. **[`mission_runner`](mission_runner/)** (C++ 20Hz 대회 통합 상태 머신)
   - `mission_fsm_node`: 출발(IN) $\rightarrow$ X자 교차로 경사로 $\rightarrow$ 정지선 1(2초 정지 & 초록불 대기, 50점) $\rightarrow$ 라바콘 슬라럼(측면 반발력 회피, 최대 100점) $\rightarrow$ 계단 및 정지선 2(2초 정지 & 초록불 대기, 50점) $\rightarrow$ 주차구역 진입 및 5초 앉기(API `1005` $\rightarrow$ `1004`, 50점) $\rightarrow$ 결승선(OUT) 통과(완주 200점)
