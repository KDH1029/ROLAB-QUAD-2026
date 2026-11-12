# junsu

대회용 자율주행 스택. [`rgosu`](../rgosu) 기반에 [`merge_tmp`](../merge_tmp)의 위치추정·계획 기술을 결합했다.
기준 커밋 `v2` 12aff3e (11-06). 11-05 16:54 완주 시점은 원본 태그 `run-1105-1654`.

## Components

| 순서 | 부품 | 위치 |
|---|---|---|
| 1 | XT16 드라이버 | merge_tmp `hesai_ros_driver` |
| 2 | go2_bridge (lifecycle) | `tools/go2_bridge_lc.sh` |
| 3 | FAST-LIO | `ws_fastlio_tunable`, `config/fastlio_leg_nurigwan_derange30.yaml` |
| 4 | wall_filter | `ws_wall_filter` |
| 5 | obstacle_fusion | `src/obstacle_fusion_node.py` |
| 6 | EGO-Planner | `ws_ego_review` |
| 7 | 신호등·카메라 콘 | `onboard_vision/` (내부 PC) |
| 8 | 위치 감시 | `tools/amcl_watch_core.py` |
| - | footprint guard | `src/footprint_guard_node.py` (기본 off) |
| - | 코스 러너 | `src/real_course_runner.py` |

## Build

```bash
source /opt/ros/jazzy/setup.bash
source ../merge_tmp/install/setup.bash
for ws in ws_fastlio_tunable ws_legfuse_gated ws_ego_review ws_ego_planner_tunable ws_wall_filter ws_mission_vision; do
  (cd $ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
done
```

내부 PC:

```bash
onboard_vision/deploy_onboard_vision.sh fp16
onboard_vision/deploy_onboard_vision.sh start
```

## Usage

```bash
tools/go2_ctl.sh                      # 실기
GO2_CTL_REPLAY=80 tools/go2_ctl.sh    # 재생 (격리 도메인)
```

| 키 | 동작 |
|---|---|
| `Q` | 전체 종료 |
| `a` | 전체 기동 |
| `m` | 모터 켜기 |
| `r` / `R` | 러너 시작 / 이어가기 |
| `p` | 정비 정지 |
| `s` | 설정 |
| `x`, Space | 비상 정지 |

순서: `Q` → `a` → 출발 위치 → 기립 → `m` → `r` (기립 후 30초 이내). 상세: [`MANUAL.md`](MANUAL.md)

## Runner options

`RUNNER_SLALOM_MODE`, `RUNNER_PIN_EGO_FIRST`, `RUNNER_EXIT_LINE_X`, `RUNNER_STOPLINE_ARRIVE`, `RUNNER_EGO_SILENT_S`, `RUNNER_OFF_ROUTE_M`, `RUNNER_RETRACE_MAX`, `RUNNER_GAIT_MODE`, `RUNNER_PARK_MODE` 등

## Test

```bash
cd src && python3 test_real_course_runner.py
python3 tools/test_amcl_watch.py
cd sim && TMAX=420 CONE=det GUARD=off ./run_sim_v2.sh <name> 80 $PWD/route_v2_1650_hybrid_sim.json
```

## Notes

- 스크립트 경로가 `~/ws/junsu`, `~/ws/junsu_v2`, `~/ws/merge_tmp_20261102/review`, 내부 PC `~/junsu_vision` 기준
- 경로 파일(`maps/route_*.json`)은 매핑 당일 지도 좌표
- 저장된 관리자 설정 `maps/segments/logs/go2_ctl_state.json`이 기본값보다 우선
- 콘 추적 설정 `config/cone_tracker_junsu.yaml`은 소형 콘(23 cm) 기준
- 카메라 장착값 `GO2_CAM_FIX=1650` (11-06, 수평 장착)
