# juninsu

기록 경로 반복 방식 (본선용). 리모컨 주행으로 기록한 경로를 FAST-LIO 위치 기준으로 추종한다.
[`junk`](../junk)의 검증본 `nail-1105`에서 분기. 기준 커밋 `main` 2f40691 (11-06).

## Workflow

```bash
~/go2_record/go2_record_auto.sh <name>                    # 1. 녹화
tools/make_map.sh ~/bags/<bag> <name>                     # 2. 지도 생성 → routes/<name>/
~/venv/go2/bin/python3 tools/route_editor.py <name>       # 3. 경로 편집 → routes/<name>/route.json
tools/junk_ctl.sh                                         # 4. 주행: Q → a → 기립 → m → r
```

## Route editor

| 키 | 동작 |
|---|---|
| `1` `2` `4` `7` `8` | start, line, stopline, parking, exit |
| 클릭 / 드래그 / `d` | 추가 / 이동 / 삭제 |
| `a`, `←` `→`, `z` | 방향 지정, 5° 조정, 삭제 |
| `u`, `s` | 되돌리기, 저장 |

방향이 있는 점에서는 정지 후 회전. 정지선은 신호등 방향으로 지정.

## Runner

| 단계 | 동작 | 값 |
|---|---|---|
| 출발 | 첫 3초 스캔을 지도에 정합, 경로 좌표 변환 | ±0.3 m / ±10° 초과 시 거부 |
| 추종 | 경로점 순서대로, 몸 방향 = 경로 접선 | 0.30 m/s, 최소 0.18 m/s |
| 전방 정지 | 몸 앞 0.15 m 내 물체 (지도 벽 제외) | 최대 3 s |
| 건너뛰기 | 5 s 정체 시 다음 경로점 | |
| 회전 | 1.5 s에 3° 미만이면 전진하며 회전 | |
| 정지선 | 2 s 정지 → 초록 1 s 연속 시 출발 | 10 s 후 빨강 2 s 미검출 시 출발 |
| 주차 | StandDown → 바닥 확인 → 6 s → StandUp → BalanceStand | |
| 위치 확인 | 정지 시 AMCL + 3D ICP | `juninsu/stop_loc_node.py` |

## Structure

| 경로 | 내용 |
|---|---|
| `src/real_course_runner.py` | 러너 |
| `tools/` | 관리자, 지도 생성, 경로 편집, 출발 정렬 |
| `juninsu/` | 정지 시 위치 확인 노드 |
| `routes/rec1118x/` | 기본 경로·지도 |
| `config/` | FAST-LIO, CycloneDDS, `yaw_trim.json` |
| `onboard_vision/` | 내부 PC 신호등 YOLO |
| `ws_fastlio_tunable/` | FAST-LIO 포크 |
| `debug/` | 시뮬레이션·재생 분석 |

## Build

```bash
source /opt/ros/jazzy/setup.bash
source ../merge_tmp/install/setup.bash
(cd ws_fastlio_tunable && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
```

## Test

```bash
cd src && python3 test_path_mode.py && python3 test_park_profile.py
python3 debug/fullsim/sim_full_route.py
```

`rec1118x` 시뮬레이션 40회 모두 완주 (142.0 ± 13.5 s).

## Notes

- 경로는 폴더 기준 상대 경로. 외부 경로: merge_tmp 설치 경로, `~/unitree_ros2`, `~/go2_record`, `~/venv/go2`
- `logs/go2_ctl_state.json` (실행 시 생성)은 경로 외 설정만 덮어씀
- 시뮬레이터는 제자리 회전 실패를 재현하지 않음
