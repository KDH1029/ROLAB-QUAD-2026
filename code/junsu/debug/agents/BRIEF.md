# 멀티에이전트 점검 공통 안내 (11-06 08시) — 모든 에이전트가 먼저 읽을 것

## 상황
- Go2 EDU 로봇 대회(로보월드 2026) **본선 당일**. 주행 스택 `~/ws/junsu_v2`(git 브랜치 v2, `~/ws/junsu` 의 worktree).
- 마지막 실주행 = 11-05 16:54 `~/ws/junsu`(main a438045). 2분 14초에 완주했고 회전 시간초과 4번(40초)이 있었다.
  - 그대로 복사본: `~/ws/junsu_v2_bak`(태그 run-1105-1654).
- v2 는 그 뒤 11-05 오후 + 11-06 에 많이 바뀌었지만 **실기에서 한 번도 안 돌렸다**. 변경 목록: `~/ws/junsu_v2/HANDOFF_junsu_v2_1106.md`(L~R절), `MANUAL.md`.
- 코스 = 1650 지도(`~/ws/junk/routes/rec1650/map.npz`, 콘 위치 고정·바뀌지 않음).
  - v2 의 출발 정렬·AMCL 은 1459 지도 좌표다. 1650↔1459 변환은 `maps/T_1459_from_1650.json`.
- 사용자는 한국어. 보고도 한국어, 결론부터.

## 절대 규칙(어기면 안 됨)
1. **로봇 격리**: ROS 를 쓰는 시뮬·재생 전에 `ip link show robot-br` 가 `state UP` 이거나 192.168.123.161/.18 이 ping 되면 **아무것도 띄우지 말고 멈춘 뒤 보고**.
   - 시뮬은 도메인 78~81, lo 전용(`~/ws/v_gosu/env_replay.sh`)만 쓴다. `run_sim_v2.sh` 가 알아서 확인한다.
   - **각 에이전트는 배정된 도메인 하나만** 쓴다(동시에 여러 개 띄우지 말 것).
2. **건드리지 말 것(읽기만)**: `~/ws/junk`, `~/ws/juninsu`, `~/ws/junsu`(main), `~/ws/junsu_v2_bak`, `~/ws/junsu_merge`, `~/ws/merge_tmp_*`, `~/ws/v_gosu`, `~/ws/lab`, `~/ws/gosu_slam`, 로봇 내부 PC.
3. **junsu_v2 의 git 추적 파일(src/·tools/·config/·maps/·sim/ 기존 파일)은 고치지 말 것.** 고칠 내용은 **패치(diff)** 로만 낸다.
   - 내 결과는 전부 `~/ws/junsu_v2/debug/agents/<내 이름>/` 아래에 둔다(새 스크립트·로그·그림·patch).
   - git commit 금지. 통합은 메인 세션이 한다.
4. 로봇을 움직이는 명령·실제 관리자(`tools/go2_ctl.sh` 를 실제 모드로) 실행 금지. sudo·네트워크 설정 변경 금지. 비밀번호 입력 금지.
5. `pkill`·`kill` 은 자기가 띄운 프로세스(PID)만. 패턴 kill 금지(다른 에이전트 시뮬이 죽는다).
6. 백그라운드 `ros2 bag record` 는 SIGINT 를 무시한다 → SIGTERM. `ros2 bag play` 는 `< /dev/null`.

## 환경·명령
- ROS: `source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash; source ~/ws/merge_tmp_20261102/review/install/setup.bash`
- 파이썬(numpy/scipy/matplotlib): `~/venv/go2/bin/python3`. 한글 그림 글꼴: `plt.rcParams['font.family']='NanumGothic'`.
- 러너 단위 테스트(로봇 불필요, 도메인 하나):
  `cd ~/ws/junsu_v2/src && ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=<내 도메인> python3 test_real_course_runner.py` → "전체 PASS"
- v2 폐루프 시뮬(새 코스, `sim/run_sim_v2.sh` — 파일 앞 주석 참고):
  ```
  cd ~/ws/junsu_v2/sim && TMAX=420 EXIT_X=-0.14 CONE=det GUARD=off ./run_sim_v2.sh <결과이름> <도메인> <경로파일>
  ```
  - 경로(1650 좌표, 시뮬용): `route_v2_1650_hybrid_sim.json`(권장), `route_v2_1650_suggest_sim.json`, `route_v2_1650_sim.json`(사용자 원본), `route_main1654_in1650.json`(16:54 경로 — STACK=main 과 같이)
  - 조건: `STACK=main`(16:54 코드) · `CONE=det|walls` · `GUARD=on|off` · `SMODE=auto|path|ego` · `WALLS=$PWD/sim_walls_rec1650_thin.npy`(벽 4 cm 깎음) · `PRIOR=<bin>` · `RUNNER_<상수>=값`(러너 환경변수, 예 RUNNER_CARROT_M=1.0) · `GUARD_ARGS="-p half_w:=0.20 ..."`
  - 결과: `sim/runs_v2/<이름>/report.txt`(완주·시간·접촉 δ0/δ10·머뭇·회전·복구 횟수), `world.jsonl`(0.1초 자세·gap·러너 상태 rs), `runner.log`·`ego.log`·`guard.log`·`fuse.log`, `plot_run_v2.py <폴더> [x0 x1 y0 y1]` → run.png
  - 한 판 약 7분(TMAX 420). **결과 이름은 `<내 이름>_` 으로 시작**(겹침 방지).
- 시뮬 세계: `sim/sim_world_v2.py`(가상 라이다 /scan, 벽 = 1650 지도 칸, 콘 검출 흉내, 걸음새·엎드리기·발 떼기 문턱 0.10 m/s 모델).
  - 시뮬 시간은 실제의 약 2배다(16:54 실제 134초 vs 시뮬 266초).

## 지금까지 알아낸 것(11-06)
- 16:54 junsu(STACK=main)는 시뮬에서도 완주했다(266~408초).
- v2 는 **첫 X 통로 입구(1650 좌표 line1 ≈ (1.45~1.84, −0.1) → 정지선1 (3.93, 1.62))에서 자주 갇힌다**.
  - ego 가 북쪽 막다른 통로(x≈1.0, y 0.2~1.4)로 들어간다. 보이지 않는 곳을 빈 곳으로 보거나, 입구가 팽창(벽 0.20 m)으로 막혀 보이는 것으로 추정한다.
  - 같은 조건에서도 판마다 결과가 엇갈린다.
- 몸 모양 안전 필터(guard)를 켜면 매번 갇힌다 → 기본 끔.
- 경로 이탈 되짚기(복구)가 18~46번 걸리며 ego 자체 복구와 싸웠다 → 뒤로 걸어 나오기 + 1.0 m·3초로 바꿈. 아직 갇히는 판이 있다.
- 지도 사전 장애물 층(ego use_prior_map)과 미끄러지는 목표(RUNNER_CARROT_M)로도 해결 안 됨.
- 실제로는 16:54 junsu 가 이 입구를 문제없이 지났다. 시뮬 벽이 실제보다 약 4 cm/면 두껍다(사람 주행 보정).
