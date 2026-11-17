# runtime 에이전트 보고 — 관리자·실행 준비 점검 (11-06 08:50) — 메인 세션이 받아 적음

## 결론
1. **★ 07:54~55 에 띄운 실제 스택이 NUC 에 그대로 돈다(러너 PID 180734, 옛 route_map.json·정렬 실패 T=0 포함).**
   - 랜선이 꽂히면 러너가 출발점 점검을 통과하고, 목표와 ClassicWalk(/api/sport/request 직접)를 보낸다.
   - 사용자 관리자(PID 190215)는 113d307 이전 코드다 → **열린 관리자에서 `Q`→`y`→`q` 후 새로 켤 것.**
2. **UI 튕김 0건**: 실제 ui() 를 pty 에서 60·80·120·184·40 폭으로, 81키씩 눌렀다.
   - 단 ask 한 번 뒤 화면 루프가 바쁘게 돈다(nodelay 가 timeout 을 지움; 한 코어 100%) → 패치 A.
3. **패치 `go2_ctl_runtime.patch`**(113d307 기준, 모두 재현 → 패치본에서 해결 확인):
   - A: 바쁜 루프
   - B: `a` 가 남은 러너를 먼저 끔·시작 시 경고
   - C: `x` 가 lifecycle 시간초과 예외로 튕기며 러너를 못 끄던 것
   - D: 정렬 도구가 죽으면 지난 route_today 를 쓰던 것
   - E: 관리자 다시 켠 뒤 `R` 이 정렬 안 된 경로로 가던 것
   - F: 사소한 것
4. 실행 경로·설정 파일·패키지는 전부 있다. 기본 route_map_1106.json·hybrid 둘 다 러너 load_waypoints 를 통과한다. 정렬 도구는 slalom_path 를 보존한다.
5. 켰을 때 경로 = route_map_1106.json(상태 파일에 ver 가 없어 버려짐). hybrid 는 `s`→`1`→`3`. 나머지 작전 값은 기본과 같다(guard off).
6. 시뮬 에이전트가 같은 NUC 에서 돈다. 관리자 pgrep/pkill 패턴이 시뮬 프로세스와 겹치므로 실기 전에 끝낼 것.
7. bak 은 md5 3개 모두 통과했다. 단 bak 관리자는 `s` 를 누르면 튕긴다(옛 코드) → bak 에서는 `s` 금지.

## 실행 전 체크리스트
1. 시뮬 에이전트 모두 끝냄.
2. 옛 관리자 `Q`→`y`→`q`. `pgrep -af 'real_course_runner|ego_planner_node|obstacle_fusion'` 가 비었는지 확인.
3. 로봇 엎드린 채 전원·랜선. `ip -br link show robot-br` UP, `ping 192.168.123.161` 응답 확인.
4. `~/ws/junsu_v2/tools/go2_ctl.sh` 실행. 첫 메시지 "옛 작전 저장값 … 버림" 은 정상.
5. 경로 확인(hybrid 는 `s`→`1`→`3`).
6. `Q`→`a` → 모두 초록(xt16 약 20 · imu 약 500 · Odometry 약 20 Hz · ego active · cone_cam 약 5 · loc). guard 꺼짐이 정상.
7. 출발 자리 → 기립 → `m`(y) → `r`. 정렬 OK 확인. 기립 후 30 s 안에 `r`.
8. 비상 `x`/스페이스. 이어가기 `m`→`R`→엔터.

파일: go2_ctl_runtime.patch · go2_ctl.patched.py · ui_harness.py · drive_*.py · test_ctl_patch.py(.out) · check_routes.py · check_paths.py · pgrep_cost.py · ui_runs*/
