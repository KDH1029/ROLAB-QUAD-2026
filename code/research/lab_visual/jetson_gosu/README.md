# gosu — 젯슨 위의 격리 작업공간

팀원 스택(`~/go2_ws`, `~/ws/go2_vision`)과 **섞이지 않게** 내 것만 따로 돌려 보는 자리다.
여기 있는 파일은 팀원 것을 하나도 건드리지 않는다. 읽기만 한다.

## 격리를 어떻게 보장하나

`env.sh` 가 두 겹을 건다.

| 겹 | 무엇 | 왜 |
|---|---|---|
| `ROS_DOMAIN_ID=77` | 팀원 스택(도메인 0)과 다른 도메인 | 도메인이 다르면 DDS 가 서로를 아예 못 본다 |
| `CYCLONEDDS_URI` + `ROS_LOCALHOST_ONLY=1` | 루프백(lo)에만 묶고 멀티캐스트 끔 | 이 젯슨 밖으로 한 바이트도 안 나간다. 로봇 내부PC 도 못 본다 |

하나만 있어도 격리되지만 둘 다 건다. 실수 한 번으로 로봇이 움직이면 안 되기 때문이다.

`check_isolation.sh` 가 **켜기 전에** 확인한다. 실패하면 아무것도 실행되지 않는다.
정리(`down`)도 `/proc/<pid>/environ` 에 `ROS_DOMAIN_ID=77` 이 있는 프로세스만 죽인다 —
팀원 프로세스는 애초에 대상이 아니다.

## 쓰는 법

```bash
source ~/gosu/lab_visual/env.sh
./check_isolation.sh            # 격리 확인. ✔ 가 나와야 한다
./run_replay.sh ~/bags/data3 120
```

## 들어 있는 것

```
env.sh                  격리 환경
check_isolation.sh      켜기 전 점검
run_replay.sh           녹화본으로 전체 돌려 보기
src/icp_localization.py 3D 정합 위치추정 (TF map->odom, localization/status)
src/odometry_monitor.py 오도메트리 발산 감시 (구독만, odometry/health 만 발행)
src/traffic_light_detector.py  신호등 판정 (30초 초록불 규칙)
src/read_pcd.py         PCD 판독
tools/                  지도·장애물층·시작자세 도구
maps/go2_prior_map_v05.pcd     정합용 지도 (175,480점)
maps/go2_prior_map_clean.pcd   정리본 (804,982점)
maps/room_obstacles_map.bin    다시 구운 장애물 층 (지도 좌표)
```

## 팀원 것과 겹치는 것 — 알고 쓸 것

2026-09-12 기준 젯슨에는 이미 다음이 있다. **여기 것보다 완성도가 높다.**

| 팀원 것 | 여기 것 | 판단 |
|---|---|---|
| `go2_3d_loc_node.py` v4.0 (`/Odometry_map`, 50 Hz) | `icp_localization.py` | 팀원 것이 낫다. 여기 것은 비교용 |
| `go2_auto_cw_node.py` + `/traffic_light/status` | `traffic_light_detector.py` | 팀원 것이 낫다 (30/60초 위상, WEST 프리패스) |
| `go2_mission_manager.py` (38 KB) | 없음 | 팀원 것만 있다 |
| **없음** | **`odometry_monitor.py`** | **여기 것만 있다** |

`odometry_monitor.py` 가 유일하게 중복이 아니다. 젯슨 스크립트 전체에서
`divergen|발산|watchdog|health|imu_state.rpy` 를 찾으면 `mock_robot_sim.py` 하나만 걸린다.

## 왜 감시가 필요한가 (2026-09-12 조사)

FAST-LIO 는 서서히 흐르는 게 아니라 **한 번에 터지고 영영 못 돌아온다.**
매핑본에서 한 스캔 만에 6.5 m → 985 m, 배증시간 14스캔, 약 4초 만에 24 km 까지 갔다.
깨지는 것은 위치가 아니라 **자세**다 — 로봇 자체 IMU 는 발산 중에도 롤·피치 ±5~10° 안에
있었는데 FAST-LIO 만 ±40° 로 튀었다.

**공분산으로는 못 잡는다.** 고착 순간 23%밖에 안 움직였고 정상 변동폭이 이미 3.5배다.
필터는 17° 틀린 채로 완전히 확신하고 있었다.

그래서 감시는 **로봇 자체 IMU 와 자세를 대조한다.** 정상 1.4°, 발산 후 9.7~17.1° 로 갈린다.
방아쇠가 무엇인지는 아직 못 밝혔다 — 같은 녹화본을 두 번 돌렸는데 한 번만 터졌다.
그래서 예방이 아니라 **감지**로 간다.
