# Mission Packages (`src/missions/`)

미션 제어(경로 순서, 정지선·신호등, 주차)와 비전은 팀원 담당이라 이 저장소에 없다 (11-05 삭제).
여기 남은 것은 경로 생성에 콘 위치를 넣는 부분뿐이다.

## 포함된 패키지

1. **[`mission_vision`](mission_vision/)** (LiDAR 라바콘)
   - `cone_tracker_node`: XT16 스캔에서 콘 모양 덩어리 → 지도 좌표 확정·저장 → `/mission/cones` (EGO-Planner 콘 여유)
