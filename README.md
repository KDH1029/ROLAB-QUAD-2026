# Go2 RoboWorld 2026

Unitree Go2 EDU 사족보행 로봇 자율주행 스택 (ROS 2).
2026 로보월드 4족보행 로봇 경진대회 자율주행 부문 **우수상** (350 / 450).

## Features

- FAST-LIO 기반 위치추정 (다리 속도 결합, 모션 게이팅)
- AMCL 위치 보정 (Omni 이동모델, FAST-LIO odom 입력) + 출발 정렬
- EGO-Planner 지역 경로 계획 (1.0 m 슬라럼 통로 대응)
- LiDAR 라바콘 추적
- 온보드 YOLOv8 신호등 인식 (Jetson Orin NX, TensorRT FP16)
- 코스 러너: 정지선 신호 대기, 주차, Out Line 판정, 막힘 복구
- 기록 경로 반복 모드: 녹화 → 지도 생성 → 경로 편집 → 경로 추종
- 운영 도구: 터미널 관리자, 녹화 자동화, 격리 재생, 시뮬레이터

## Structure

```
code/
├── merge_tmp/   위치추정, 출발 정렬, 경로 계획, LiDAR 콘 추적, 카메라 보정
├── yolo/        신호등·콘 모델 학습
├── rgosu/       자율주행 스택 (초기)
├── junsu/       자율주행 스택 (대회용)
├── junk/        기록 경로 반복 (초기)
├── juninsu/     기록 경로 반복 (본선용)
└── research/    백업 위치추정, 경사로 오인식 제거, 녹화 도구
docs/            설계·구현·트러블슈팅 문서
tools/           실주행 로그 정리
```

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy, CycloneDDS
- Nav2 (`nav2_amcl`, `nav2_map_server`), PCL, Eigen3
- Unitree Go2 EDU, Hesai XT16
- 로봇 내부 PC: JetPack 5, TensorRT 8.5 (YOLO)

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd code/merge_tmp && colcon build --symlink-install && source install/setup.bash
cd ../junsu
for ws in ws_fastlio_tunable ws_legfuse_gated ws_ego_review ws_ego_planner_tunable ws_wall_filter ws_mission_vision; do
  (cd $ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
done
```

## Usage

```bash
code/junsu/tools/go2_ctl.sh        # 자율주행 스택
code/juninsu/tools/junk_ctl.sh     # 기록 경로 반복
```

자세한 실행 방법은 각 폴더의 README 참고. 스크립트 경로는 개발 환경(`~/ws/...`) 기준.

## Docs

- [시스템 구성](docs/01_시스템_구성.md)
- [기술 구현](docs/02_기술_구현.md)
- [트러블슈팅](docs/03_트러블슈팅.md)
- [설계 결정](docs/04_설계_결정.md)
- [실주행 기록](docs/05_실주행_기록.md)
- [타임라인](docs/06_타임라인.md)

## Team

| 이름 | 담당 |
|---|---|
| 김도현 | 위치추정·경로 계획 (merge_tmp), 기록 경로 반복 (junk), 직진 보정 |
| 송인애 | 신호등·콘 인식 (yolo), 기록 경로 반복 (juninsu), 직진 보정 |
| 고수영 | 자율주행 스택 (rgosu) |
| 한준희 | 자율주행 스택 (junsu) |

## License

외부 코드 포함: FAST-LIO (GPL-2.0), Hesai ROS Driver. 각 폴더의 LICENSE 참고.
