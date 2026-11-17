# 백업 위치추정 v2 (night_20261104/backup_v2)

```
만든 날   2026-11-04 새벽
무엇      v_gosu/backup 의 백업 위치추정에서 부품 두 개만 교체 — 정합기(ICP→GICP), 지면표(GroundGrid→RobustGroundGrid)
원칙      v_gosu/backup, gosu_slam, lab, Rgosu 는 읽기만. 전환 장치(odom_mux_node)·플래너 설정은 v_gosu 것을 그대로 씀
파이썬    야간 venv (~/ws/night_20261104/venv, --system-site-packages, small_gicp 1.0.1 MIT)
```

## 결과 (10-30 녹화본, 공정 시험: 앞 270초로 만든 지도 → 뒤 270초)
| | v1 (v_gosu/backup) | **v2** |
|---|---|---|
| 오프라인 xy 오차 중앙 / p95 / 최대 | 0.046 / 0.099 / 0.282 m | **0.027 / 0.050 / 0.086 m** |
| 경사 위 p95 | 0.23 m | **0.054 m** |
| 정합 거부 | 9.5% | **0%** |
| 정합 시간 | — | 2.4 ms 중앙 / 3.2 ms p95 |

**무모터 재생 시험** (`replay_check_v2.sh`, 도메인 80, lo 전용, 270초에 FAST-LIO 강제 종료, 앞부분 지도만 사용) — `out/v2half.*`
- 전환 270.3초 (FAST-LIO 0.5초 무소식), 전환 순간 위치 변화 0.019 m
- /Odometry_mux 19.7 → 50.5 Hz(최대 공백 0.04 s), 경로 6.1 → 5.7 Hz, cmd_vel 20 Hz 유지
- 백업 정합 2711 수락 / 0 거부, health ok 99.8%
- 전환 뒤 백업 궤적 → 원 FAST-LIO 경로까지 거리 중앙 0.011 / p95 0.029 / 최대 0.045 m (시각 대응 없는 공간 거리 — 하한)
- 끝에 보이는 odom_mux Traceback(ExternalShutdownException)은 Ctrl-C 종료 때만 나는 것 — v1 재생에서도 같음, 동작 무관

## 왜 바꿨나 (자세히는 ../backup_eval/RESULTS.md)
경사로 오차 0.25 m 의 원인은 정합기가 아니라 **지면표가 천장(2.23 m)을 바닥으로 낸 것**. 바닥이 안 찍힌 칸에서 z 씨앗이 2.5 m 튀어 정합이 11초 거부됐다.
경기장 지도도 바닥이 덜 찍힌 칸이 있으면 같은 일이 난다 → v2 의 견고 지면표로 막음.

## 사용
```bash
# 재생 시험 (로봇 분리 후)
~/ws/night_20261104/backup_v2/replay_check_v2.sh <bag> <태그> <지도.npz> [FAST-LIO 끌 시각]
python3 ~/ws/night_20261104/backup_v2/analyze_replay_v2.py <태그>
# 실기 그림자 모드 (/backup/* 만 냄 — 주행에 안 씀)
source ~/gosu/env_drive.sh && ~/ws/night_20261104/venv/bin/python ~/ws/night_20261104/backup_v2/backup_loc_node_v2.py --map <지도.npz>
# 실주행 (원본 gosu_slam 런치 대신) — 무모터 먼저
ros2 launch ~/ws/night_20261104/backup_v2/launch/full_nav_slam_backup_v2.launch.py backup_map:=<지도.npz> motor:=false
```
