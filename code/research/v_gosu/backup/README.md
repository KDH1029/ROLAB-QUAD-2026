# 백업 위치추정 + 전환 장치 (v_gosu/backup)

```
만든 날   2026-11-03~04
목적      FAST-LIO 가 튀거나 죽어도 주행을 이어가는 두 번째 위치추정 — FAST-LIO 와 독립
원칙      gosu_slam·lab·Rgosu 원본은 건드리지 않는다. 런치는 사본(launch/full_nav_slam_backup.launch.py)
```

## 구조
```
다리 오도메트리 /utlidar/robot_odom (Unitree 공식, 150 Hz) ──┐  증분으로 예측 + roll·pitch 로 스캔 수평 세우기
XT16 /lidar_points (드라이버 원본, 원시 바이트, 5 Hz 만 해석) ─┼─▶ backup_loc_node ─▶ /backup/odometry (50 Hz)
사전 3D 지도 (출발 자세 = 원점)                              ──┘   x·y·yaw 3자유도 ICP      /backup/cloud (5 Hz) /backup/status

/Odometry (FAST-LIO) ─┐                       평소: FAST-LIO 를 그대로 중계
/backup/odometry ─────┼─▶ odom_mux_node ─▶ /Odometry_mux → odom_filter → /Odometry_filtered → ego_planner, TrackSequencer
/cloud_registered ────┤                    /cloud_active → ego_planner
/backup/cloud ────────┘   전환: FAST-LIO 0.5초 무소식 / 순간속도 2 m/s 초과(메시지 시각 기준) / 백업과 0.5 m·1초
                          시작 5초 판단 안 함. 한 번 전환하면 유지, 복귀는 ros2 service call /mux/reset std_srvs/srv/Trigger
```
- 핵심 계산은 `backup_core.py` 하나 — 노드와 녹화본 시험(`test_offline.py`)이 같은 코드를 쓴다.
- 플래너는 "점 높이 − 로봇 높이"로 장애물을 판단하므로, 백업 모드에선 점구름과 위치가 같은 백업 z 를 써서 절대 z 오차가 상쇄된다.

## 녹화본 시험 결과 (10-30 녹화본, 기준 = FAST-LIO 궤적)
| | 다리 오도 단독 | 정합(수평 안 세움, 9월 lab_visual 방식) | **정합 + 수평 세우기 (이것)** |
|---|---|---|---|
| 위치오차 중앙 / p95 / 최대 | 0.39 / 0.67 / 0.73 m | 0.05 / 0.11 / 0.46 m | **0.05 / 0.08 / 0.23 m** |
| X자 가운데(벽이 먼 곳) p95 | 0.66 m | 0.20 m | **0.08 m** |
- 정합 주기: 5 Hz ≈ 10 Hz, 20 Hz 는 오히려 악화(170~257초 구간 최대 0.69 m, 원인 미확정) → 5 Hz.
- 다리 오도만으로는 걸은 거리의 10~14% 틀어짐(10초 0.15 m, 30초 0.22 m) → 정합이 10초 끊기면 health=degraded, 30초 lost.
- z 를 정합으로 같이 풀면 악화(대응점 대부분이 벽) → 끔.
- 전환 장치(4 상황): 정상 주행 전환 없음 · 끊김 0.5초 · 3 m 점프 즉시 · 0.05 m/s 미끄러짐 0.58 m 에서. 정상 주행 여유: FAST-LIO 순간속도 최대 0.52 m/s, 백업 차이 최대 0.25 m.
- **주의**: 지도와 시험 녹화가 같은 주행이라 낙관적이다. 경기장 지도 + 다른 주행 녹화로 다시 볼 것.

## 사용
```bash
# ① 경기장 사전 지도 (출발 자세에서 시작한 매핑 녹화로, 로봇 분리 후)
V_DOMAIN=79 V_VOX=0.02 ~/ws/v_gosu/tools/map_replay.sh <매핑bag> arena1104      # → ~/ws/v_gosu/eval/out/arena1104_map.npz
# ② 녹화본 시험 (로봇 분리 후, 200초에 FAST-LIO 를 끈다)
~/ws/v_gosu/backup/backup_replay_check.sh <bag> <태그> ~/ws/v_gosu/eval/out/arena1104_map.npz 200
python3 ~/ws/v_gosu/backup/analyze_replay.py <태그>
# ③ 실기 그림자 모드 (주행엔 안 씀 — /backup/* 만 냄. 주 스택과 같이 띄워 비교)
source ~/gosu/env_drive.sh && python3 ~/ws/v_gosu/backup/backup_loc_node.py --map ~/ws/v_gosu/eval/out/arena1104_map.npz
# ④ 실주행 (백업 포함 런치 — 원본 gosu_slam 런치 대신)
ros2 launch ~/ws/v_gosu/backup/launch/full_nav_slam_backup.launch.py backup_map:=<지도> motor:=false   # 무모터 먼저
```

## 남은 일
- [ ] 경기장 지도로 재검증 (오늘 매핑 녹화 필요)
- [x] 녹화본 무모터 재생 시험 ② (11-04, `out/bk1030.*`) — 200초에 FAST-LIO 강제 종료 → 0.5초 만에 전환, 위치 출력 19.7→50.5 Hz(최대 공백 0.06 s), 경로 7.6→7.2 Hz, cmd_vel 20 Hz 유지, 전환 순간 위치 변화 0.031 m, 끝까지 백업 health ok(99.8%), 끝 위치 출발점 0.16 m
- [ ] 실기 그림자 모드 ③
- [ ] 20 Hz 악화 원인
