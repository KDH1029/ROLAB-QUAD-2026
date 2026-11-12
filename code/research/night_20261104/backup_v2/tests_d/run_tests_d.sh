#!/usr/bin/env bash
# [D] 부품 고침(①지면 z_ref ②정합 비율 판정 ③전환 가드) 시험 전부 — ROS 노드 없이 bag 파일만 읽는다(DDS 참여 없음).
# 사용: ~/ws/night_20261104/backup_v2/tests_d/run_tests_d.sh        (약 4분, 병렬 없음)
set -e
source /opt/ros/jazzy/setup.bash
PY=/home/ilovehan9283/ws/night_20261104/venv/bin/python
cd "$(dirname "$0")"
echo '── ① 지면표 z_ref (고치기 전 vs 후, 지도 훼손 시험)';  nice -n 5 $PY test_robust_ground.py
echo '── ② 정합 비율 문턱 실험 (맞는 자세 비틀기, 120장)';   nice -n 5 $PY gicp_inlier_experiment.py 120 | tail -3
echo '── 회귀: 공정 시험 (앞 270초 지도 → 뒤 270~542초), 고치기 전 vs 후'
VARIANTS='orig5:0.08:0.2:0:60:1:0:orig,new5:0.08:0.2:0:60:1:0.70:new,newg5:0.08:0.2:0:60:1:0:new,new10:0.08:0.1:0:60:1:0.70:new' TAG=d_regress \
  nice -n 5 $PY split_validate_gicp_var_d.py 270 /home/ilovehan9283/ws/night_20261104/backup_eval/map_first270s_fastlio.npz | grep '②'
echo '── ③ 전환 장치 가드 (v1 MuxLogic vs v2 GuardedMux)';   nice -n 5 $PY test_mux_guard.py --recompute
echo '── ③ 전환 장치 노드 껍데기 (가짜 rclpy — DDS 없음): 보류·전환·reset·종료 예외'; python3 test_mux_node_mock.py
