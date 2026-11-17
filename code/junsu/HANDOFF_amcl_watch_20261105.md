# 인계: 출발 ICP + AMCL 감시자 (11-05 11시, 다른 세션 → junsu 담당 세션)

사용자 결정: 출발 ICP 로 좌표를 맞추고, AMCL 은 **감시만** 한다(주행 보정 안 함). 차이가 크면 러너가 선다.
이 세션은 기존 junsu 파일을 하나도 고치지 않았다. 새 파일 둘만 있음:
- `tools/amcl_watch_core.py` — 판정 로직(ROS 없음). `Watch(T, t_init).update(t, lio_pose, amcl_pose, sigma_xy)` → WARMUP/NO_AMCL/UNSURE/OK/WARN/STOP
- `tools/test_amcl_watch.py` — `python3 tools/test_amcl_watch.py` → ALL PASS (review amcl3 o_*.npz 재생 기록 사용)

## 측정 결과 (오프라인, review amcl3 o_arena4 / o_113612 / o_145915, AMCL = FAST-LIO odom, ~470 s)
| | 결과 |
|---|---|
| 출발 정렬 | 3개 모두 OK (T: −3.9/+10.5 cm −4.02° · +15.1/+1.0 cm +1.82° · +1.3/−0.3 cm +0.38°) |
| 정렬 후 AMCL 과 차이(정상) | 최대 0.149 m / 4.78°, p95 0.116 m / 2.78° |
| 문턱 STOP 0.30 m 또는 8°, 1초 지속 | 정상 주행 STOP 0번 |
| 0.5 m 튐 / 10° 튐 주입 | 1.1~1.5초 안에 STOP |
| 0.1 m/s 흐름 주입 | 3.8~5초 안에 STOP |
| 대조: 정렬 없이 T=0 | arena4·113612 가짜 STOP → 정렬이 반드시 필요 |
| AMCL σxy | 중앙 0.09, p95 0.19, 초기화 직후만 0.47 → σ>0.25 면 판정 안 함(UNSURE) |

## 실기에 붙일 때 꼭 지킬 것
1. **AMCL odom 은 FAST-LIO.** 다리 odom AMCL 은 정상 주행에서도 0.315 m / 7.3° → 0.3 m 문턱이면 오탐.
2. **라이브 /tf 에 AMCL 붙이지 말 것.** 지금 `body` 부모가 둘(FAST-LIO camera_init→body, 브리지 정적 base_link→body).
   → AMCL 은 `-r /tf:=/tf_amcl -r /tf_static:=/tf_amcl_static`, `tf_broadcast:false`, 네임스페이스(/junsu_amcl).
   /Odometry → /tf_amcl 로 `lio_odom → base_stab`(x, y, yaw 만) 중계. `/scan` frame 이 `base_stab`, base_link→body 이동량 0 이라 그대로 맞음.
   AMCL 파라미터: odom_frame_id lio_odom, base_frame_id base_stab, 나머지는 review amcl_params(Omni, likelihood_field).
3. **서 있으면 AMCL 이 갱신을 안 낸다**(113612: 53초 공백). amcl_pose 가 0.5초 넘게 없으면 `request_nomotion_update` 호출.
4. 초기자세 = `compose(T, 지금 FAST-LIO 위치)`, 공분산 작게(0.05 m, 3°). 기립 후에(엎드린 스캔 금지).
5. 시각은 메시지 시각으로(최근 /Odometry stamp 를 '지금'으로) — 재생에서도 같게 동작.
6. **정렬 거부(exit 2)면 감시는 기록만**(T=0 이면 오탐 — 위 대조).
7. 러너: STOP 이 오면 hold_here + 멈춤 상태. STOP 은 한 번 나면 유지되므로, 이어가기(R) 때 이전 STOP 을 무시하는 장치 필요(예: stop 번호).
8. **출구 선 판정**: 정렬하면 경로점은 오늘 좌표인데 `out_line_x` 는 지도 좌표 → `exit_passed`/`exit_dir`/EXIT_PUSH 를 T 로 지도 좌표로 바꿔 판정할 것.
   방향 3° 차이면 y≈4 m 에서 x 가 ~0.2 m 틀어진다.

## 확인 못 한 것
- AMCL 이 FAST-LIO 튐을 따라가는 정도(AMCL 이 같은 odom 을 쓰므로). 오프라인 시험은 'AMCL 이 맞다' 가정 → 로봇 끊고 ROS 재생으로 튐 주입 시험 필요.
- 경로 파일과 정렬 지도의 좌표계: 145915(14:59 녹화) 정렬 T≈0 이라 같은 좌표계로 판단.
