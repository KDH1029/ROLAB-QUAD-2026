# Mission Packages (`src/missions/`)

2026 로보월드 4족보행 로봇경진대회(450점 만점 규정) 수행을 위한 온보드 C++ 라바콘 위치 확정과 미션 시퀀스 제어 도메인입니다.

## 포함된 패키지

1. **[`mission_vision`](mission_vision/)** (LiDAR 라바콘)
   - `cone_tracker_node`: XT16 스캔에서 콘 모양 덩어리 → 지도 좌표 확정·저장 → `/mission/cones`. 카메라·YOLO·신호등 인식은
     팀원 담당이라 이 저장소에 없다 (11-05 삭제)

2. **[`mission_runner`](mission_runner/)** (C++ 20Hz 대회 통합 상태 머신)
   - `mission_fsm_node`: 출발(IN) $\rightarrow$ X자 교차로 경사로 $\rightarrow$ 정지선 1(2초 정지 & 초록불 대기, 50점) $\rightarrow$ 라바콘 슬라럼(측면 반발력 회피, 최대 100점) $\rightarrow$ 계단 및 정지선 2(2초 정지 & 초록불 대기, 50점) $\rightarrow$ 주차구역 진입 및 5초 앉기(API `1005` $\rightarrow$ `1004`, 50점) $\rightarrow$ 결승선(OUT) 통과(완주 200점)
