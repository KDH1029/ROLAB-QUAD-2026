# rgosu

자율주행 스택 초기판. FAST-LIO + EGO-Planner + 코스 러너.
대회에는 이 폴더를 바탕으로 만든 [`junsu`](../junsu)를 사용했다.

## Structure

| 경로 | 내용 |
|---|---|
| `src/real_course_runner.py` | 코스 러너 (목표 순서, 정지선, 주차, 출구, 막힘 복구) |
| `src/track_sequencer_node.py`, `segments_config.json` | 구간별 카메라 on/off, 장애물 필터 설정 |
| `src/obstacle_fusion_node.py` | 벽 + L1 + 카메라 콘 병합 |
| `src/test_*.py` | 단위 시험 |
| `tools/go2_full_stack.sh` | 전체 스택 기동 |
| `tools/route_editor.py` | 경로점 편집기 |
| `sim/run_sim.sh` | 폐루프 시뮬레이터 |
| `ws_fastlio_tunable/`, `ws_legfuse_gated/` | FAST-LIO 포크 |
| `ws_ego_planner_tunable/` | EGO-Planner 포크 |
| `ws_wall_filter/` | 수직 벽 추출 |
| `onboard_vision/` | 내부 PC 신호등 YOLO |
| `WORKLOG.md` | 작업 기록 (09-30 ~ 11-05) |

## Build

```bash
source /opt/ros/jazzy/setup.bash
source ../merge_tmp/install/setup.bash
for ws in ws_fastlio_tunable ws_ego_planner_tunable ws_wall_filter; do
  (cd $ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
done
```

## Usage

```bash
tools/go2_full_stack.sh
tools/go2_stop_all.sh
```

## Test

```bash
cd src && python3 test_real_course_runner.py
sim/run_sim.sh 420 5 0.25    # 제한 시간, 신호 주기, inflate_radius
```

## Notes

- 스크립트 경로가 `~/ws/Rgosu`, `~/ws/merge_tmp_20261102/review`, `~/ws/v_gosu` 기준
- `segments_config.json`, `maps/*.json` 좌표는 대회장 지도 기준
- EGO-Planner `inflate_radius`는 0.25 m 미만 비권장 (로봇 중심 25 cm 내 점 무시)
