# 외부 패키지 (저장소 안으로 포크)

빌드에 필요한 외부 코드를 저장소 하나로 빌드되도록 복사했다. **소스는 수정하지 않았다**
(문서 이미지·로그·백업 설정만 제외). 수정이 필요하면 이 폴더에서 고치고 아래 표에 기록한다.

| 패키지 | 출처 | 포함 범위 | 라이선스 |
|---|---|---|---|
| `fast_lio` | 팀 FAST-LIO 포크 (`~/ws/lab/go2_ws/src/fast_lio`, 원본 hku-mars/FAST_LIO ROS 2 판 + motion_gating) | src, include, config, launch, msg, rviz (doc/ 이미지 128 MB, Log/, PCD/ 제외) | GPL-2.0 (LICENSE 파일. package.xml 의 BSD 표기와 다름) |
| `livox_ros_driver2` | 메시지 전용 패키지 (`~/ws/lab/deps/livox_ros_driver2`) — FAST-LIO 빌드용 CustomMsg 만 | 전체 | MIT (package.xml) |
| `hesai_ros_driver` | Hesai HesaiLidar_ROS_2.0 (https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0, e7e112f 2026-04-27) + HesaiLidar_SDK_2.0 서브모듈을 실제 파일로 포함 | 전체 (.git 제외) | Hesai Modified BSD (LICENSE) |

- XT16 설정: `hesai_ros_driver/config/config.yaml` (장치 IP 192.168.123.20, 출력 `/lidar_points`).
  드라이버는 빌드 시점의 소스 경로(PROJECT_PATH)에서 이 파일을 읽는다.
- FAST-LIO 실행 설정은 이 폴더가 아니라 `config/fastlio/fastlio_xt16_odometry.yaml` (det_range 15, 외부파라미터).
- GPL-2.0 코드(FAST-LIO)를 포함하므로, 저장소를 배포할 때 라이선스 조건을 확인할 것.
