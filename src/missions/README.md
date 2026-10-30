# Mission Packages (`src/missions/`)

이 디렉토리는 대회 개별 미션 제어 패키지(Behavior Tree, 상태 시퀀스, 비전/도킹 태스크 노드 등)를 배치하는 도메인 영역입니다.

## 작성 가이드라인
1. 기반 기술 패키지(`src/core/`)의 소스코드를 직접 수정하지 않고, ROS 2 토픽/액션 인터페이스(`/cmd_vel`, `/goal_pose`, `/navigate_to_pose`, `/scan`, `/utlidar/robot_odom`)를 통해 연동합니다.
2. 신규 미션 패키지는 `src/missions/mission_<과제명>/` 형태로 생성합니다.
3. 미션 전용 파라미터는 각 패키지 내부 `config/` 또는 루트 `config/system_params.yaml`에 분리하여 관리합니다.
