# 인계: Go2 위치추정 (AMCL + 3D ICP) 고치기 — 2026-11-05 저녁 기준

> 이 문서를 그대로 담당자에게 주면 된다. 아래 "작업 지시"가 프롬프트이고, 나머지는 맥락·현재 상황이다.

---

## 작업 지시 (프롬프트)

너는 Unitree Go2 EDU 대회 로봇의 **위치추정 담당**이다. 내일(11-06) 본선 전에 아래를 해 줘.

**목표**: 대회 주행 코드(`~/ws/junk`, 태그 `nail-1105`)가 쓰는 FAST-LIO 좌표가 **오늘 지도(rec1650)와 계속 맞도록** 바로잡는 위치추정 노드를 만든다.
- 핵심은 **3D ICP 재정렬**: 오늘 라이다 점구름을 rec1650 3D 지도에 맞춰 `T`(지도 = T · 오늘 FAST-LIO)를 구한다.
- **AMCL**은 감시용이다. FAST-LIO와 지도가 어긋났는지 판단하는 데 쓰고, 고칠 수 있으면 고친다.
- 우선순위:
  1. **주차 뒤 재정렬**: 러너가 이미 요청을 보낸다. 지금은 아무도 응답하지 않는다.
  2. **출발 정렬 보강**: 지금은 2D 스캔만 쓴다.
  3. **주행 중 감시**: 대회 코드에서는 지금 효과가 없다. 아래 "주의" 참고.

**반드시 지킬 것**
1. **대회 코드는 수정 금지.**
   - 대상: `~/ws/junk/src/`, `~/ws/junk/tools/`, `~/ws/junk/config/` 기존 파일. 태그 `nail-1105`로 고정돼 있다.
   - 새 파일은 `~/ws/junk/debug/loc/` 아래에만 만든다.
   - 러너를 바꿔야만 되는 부분은 바꾸지 말고 **필요한 변경을 문서로** 남긴다. 반영은 팀장이 판단한다.
2. **이미 있는 토픽 약속을 그대로 쓴다.** 러너는 이미 이 토픽들을 듣고 있다. 아래 "인터페이스" 참고.
3. **재생 시험은 격리해서 한다.**
   - ROS 도메인 78~81, lo 전용으로만 돌린다.
   - 로봇이 연결된 상태(robot-br UP)에서 재생하지 않는다. 09-10에 재생 데이터가 라이브 로봇에 섞여 주행이 흔들린 사고가 있었다.
4. **로봇을 움직이는 시험은 팀장이 직접 한다.** 너는 재생 결과와 실행 명령까지만 준비한다.
5. **위치추정 때문에 로봇이 계속 멈춰 있으면 안 된다.** 판단이 안 서면 아무것도 하지 않는다(fail-open). 예선에서 ICP 실패가 STOP을 붙잡아 로봇이 섰다.
6. **팀원 저장소는 읽기만 한다.** 대상: `~/ws/merge_tmp_20261102`, `~/ws/lab`, `~/ws/gosu_slam`.

**완료 기준 (재생으로 확인)**
- **주차 재정렬**: bag `gosu_20261105_165023_rec` 재생 중 주차 구간에서 `/mission/realign`을 보내면, 5초 안에 `/mission/align_T`(ok=true)가 오고 T 오차가 녹화 궤적 대비 5 cm·2° 이내다.
- **엉뚱한 답 거부**: 일부러 FAST-LIO를 0.3 m·10° 틀어 놓아도 맞는 T로 돌아온다. 틀린 답(먼 곳으로 튀는 T)을 내느니 `ok=false`를 낸다.
- **CPU**: NUC 6코어 기준 한 코어 50% 이하. 점구름 매 프레임 처리는 파이썬으로 하지 말 것. 파이썬 역직렬화만으로 CPU 45~70%를 먹은 실측이 있다.
- **결과물**:
  - 실행 명령 한 줄
  - 재생 결과 숫자 표
  - 실패 사례
  - 러너에 필요한 변경 목록 (있으면)

---

## 1. 지금 대회 전략 (11-05 저녁 규정 변경 뒤)

- 경기장 지도는 고정됐다. 콘이 크고 모양이 다른 것으로 바뀌어 YOLO로는 콘을 인식하지 못한다.
- 그래서 **미리 녹화한 지도 위에 손으로 찍은 경로를 그대로 따라가는** 방식으로 바꿨다.
  - 실시간 장애물 회피와 동적 경로 생성은 없다.
  - YOLO는 신호등에서만 쓴다.
- 경로: `~/ws/junk/routes/rec1650/route.json` (48점)

| 점 | 의미 |
|---|---|
| #1 | 출발 |
| #5 | 신호등 1 (#30과 같은 좌표) |
| #7~#25 | 슬라럼 (콘 고정, 지도에 들어 있음) |
| #30 | 신호등 2 |
| #40 | 주차 (StandDown 5.5초 → StandUp → BalanceStand) |
| #41~#46 | #39 → #34로 되돌아감 |
| #48 | 출구 |

- 실행: `~/ws/junk/tools/junk_ctl.sh` (관리자). 키는 `a`(부품) → `m`(모터) → `r`(출발).
- 부품 구성: xt16 라이다 드라이버 → go2_bridge(cloud/imu) → FAST-LIO → 비전(내부 PC Jetson, 신호등) → cmd_vel_to_sport(모터) → 러너. **AMCL·ICP 노드는 지금 없다.**

## 2. 지금 위치추정 구조 (junk)

1. **주행 좌표는 FAST-LIO 하나**다.
   - `/Odometry`, camera_init 프레임. 원점은 FAST-LIO를 켠 자리다.
   - 설정은 `~/ws/junk/config/fastlio_leg_nurigwan_derange30.yaml`. 다리 속도를 융합한 포크이고 max_range 15 m다.
   - 다리 게이트가 있다: 기울기 12° 또는 회전 40°/s를 넘으면 그 프레임은 다리 융합을 건너뛴다.
2. **출발 정렬은 2D 한 번뿐**이다.
   - `r`을 누르면 `tools/start_align/go2_start_align.py`가 서 있는 동안 3초 `/scan`을 모아 `routes/rec1650/map.pgm`에 맞춘다.
   - 탐색 범위 ±0.3 m·±10°. 결과 T를 `logs/route_today.json.align.json`에 쓰고, 경로를 오늘 좌표로 바꾼다.
   - 오늘 실측: 일치율 1.0, T = (0.033, −0.005, −4.5°).
3. **주행 중에는 아무도 위치를 고치지 않는다.**
   - 러너는 주차 후 `/mission/realign`을 보내고 5초 기다린다.
   - 응답이 없어서 오늘 로그에 "주차 뒤 재정렬 응답 없음(5초)"이 찍혔다.

## 3. 예전 위치추정 (junsu) — 참고용, 실패 기록

- 코드: `~/ws/junsu_v2/src/junsu_loc_node.py`, `tools/amcl_watch_core.py`, `config/amcl_junsu.yaml`
- 구성: nav2 AMCL(감시 전용, `/tf_junsu` 별도 트리) + **2D ICP** 재정렬. 2D ICP는 `start_align_core.align`이고 /scan 대 2D 지도, ±0.3 m·±10°다.
- 실패 기록 (`~/ws/junsu/maps/segments/logs/ctl_loc.log`):
  - **예선 12:21**
    - 증상: "재정렬 ICP 실패: best fit outside box (+0.21, −0.32 m, −10.0°)"가 3초마다 반복됐고, STOP이 유지돼 러너가 섰다.
    - 원인 추정: 주차(앉기·일어나기) 충격으로 FAST-LIO가 흐른 양이 탐색 범위를 넘었다. 충격 구간에서 자세가 6.66배 튀는 것을 실측했다.
  - **16:53**
    - 증상: ICP 최적해가 (−9.07, +3.53 m), (−1.9, +3.2 m)처럼 수 m씩 튀었다.
    - 원인 추정: 지도(11-04 14:59, `arena1459_*`)에 **새 콘이 없다**. 경기장이 바뀌어 2D 스캔이 지도와 안 맞았다.
  - **AMCL 초기자세가 안 들어간 세션**: "AMCL cannot publish a pose… set the initial pose"가 계속 찍혔다. 정렬 T가 없으면 AMCL이 영영 시작하지 않는다.
- 결론: 2D 스캔 ICP와 옛 지도로는 안 된다. **오늘 지도(rec1650, 새 콘 포함) + 3D 정합**으로 다시 만든다.

## 4. 인터페이스 (러너가 이미 듣는 것 — 이 형식 그대로)

| 토픽 | 방향 | 형식 | 러너 동작 |
|---|---|---|---|
| `/mission/realign` | 러너 → 너 | `std_msgs/String` JSON `{"max_xy":0.5,"max_yaw":20,"why":"park"}` | 주차에서 일어난 뒤 보냄. 지금 T 둘레 ±max_xy·±max_yaw로 찾아 달라는 요청 |
| `/mission/align_T` | 너 → 러너 | `std_msgs/String` JSON `{"T":[x, y, yaw_rad], "ok":true, "reason":"realign …"}` | T: 지도 = T·오늘 FAST-LIO. `reason`이 `realign`으로 시작해야 주차 대기가 바로 풀림. ok=false거나 T가 같으면 무시. 받으면 **남은 목표를 다시 계산** |
| `/mission/loc_watch` | 너 → 러너 | `std_msgs/String` JSON `{"state":"OK"/"WARN"/"STOP", …}` | STOP이면 그 자리에 서서 재정렬을 기다림(최대 4초, 이후 그냥 진행) |

- QoS: `align_T`는 러너가 RELIABLE + TRANSIENT_LOCAL(depth 1)로 구독한다. 너도 같은 QoS로 발행해야 늦게 켠 러너도 마지막 T를 받는다.
- 출발 T 초기값: `~/ws/junk/logs/route_today.json.align.json`의 `T`. 러너 `r`마다 새로 쓴다.

**★ 주의 — 대회 러너(nail-1105)의 한계** (고치려면 러너를 바꿔야 함 → 문서로만 제안)
- 러너는 경로 전용 모드라 대부분 시간 `PIN_TRACK` 상태다. `loc_watch` STOP은 `TO_GOAL`·`EXIT_PUSH`·`BACKOFF` 상태에서만 먹는다. **주행 중 STOP은 사실상 무시된다.**
- `align_T`가 새로 와도 **지금 따라가는 곡선은 안 바뀐다.** 다음 구간(다음 정지선·주차·출구까지)부터 반영된다.
- 그래서 **실제로 효과가 나는 지점**은 다음 셋이다.
  1. 주차 재정렬(PARK_REALIGN, 5초 대기)
  2. 각 정지선에서 멈춘 동안
  3. 출발 정렬

## 5. 데이터·지도

| 것 | 위치 | 설명 |
|---|---|---|
| 오늘 녹화 (지도 원본) | `~/bags/gosu_20261105_165023_rec/` (974 MB mcap) | 16:50 경기장 한 바퀴. `/lidar_points`(XT16 20 Hz), `/lowstate`, `/sportmodestate`, `/utlidar/*`, `/tf_static` 포함. `meta.txt` 참고 |
| 첫 녹화 (비교용) | `~/bags/gosu_20261105_164532_rec/` | 더 느리게 오래 돈 것. 지도는 rec1650이 더 깨끗함(벽 두께 중앙 10.7 cm 대 12.4 cm) |
| 3D 지도 | `~/ws/junk/routes/rec1650/map.npz` | `vox`(N×3 int, 2 cm 복셀 번호) · `cnt`(복셀당 점 수, ≥2만 쓰기) · `V`=0.02 · `odom`(녹화 궤적: t, t, x, y, z, qx, qy, qz, qw). 좌표 = 그 녹화의 camera_init(녹화 시작 자리가 원점) |
| 2D 지도 | `~/ws/junk/routes/rec1650/map.pgm` / `map.yaml` | 3 cm, origin [−12.66, −12.98]. 벽 = 세로 0.25 m 넘는 면(콘 포함). 녹화 길 0.18 m 안의 점(로봇 몸·사람)은 뺐음 |
| 편집기 캐시 | `~/ws/junk/routes/rec1650/editor_cache.npz` | `walls`, `floor`, `path`(녹화 궤적 xy) |
| 재생 → FAST-LIO | `~/ws/junk/tools/map_replay.sh <bag> <이름>` | 도메인 78, lo 전용. bag을 재생해 FAST-LIO 궤적과 복셀을 만든다(로봇 연결 시 거부). `~/ws/v_gosu/env_replay.sh` 사용 |
| 오늘 주행 로그 | `~/ws/junk/logs/ctl_runner.log` | 18:00, 18:06 주행. 주차 재정렬 응답 없음, 주차 뒤 회전 타임아웃(남은 −64°) |

FAST-LIO 출력 토픽: `/Odometry`, `/cloud_registered`(월드), `/cloud_registered_body`(몸 좌표), `/Laser_map`. 입력은 `/velodyne_points`(cloud_bridge가 /lidar_points를 바꾼 것), `/imu/body`. 2D `/scan`은 cloud_bridge가 낸다.

## 6. 하드웨어·환경 메모

- **기기**:
  - NUC16: Ubuntu 24.04, ROS 2 Jazzy. 주행 스택 전부가 여기서 돈다.
  - 내부 PC: Jetson Orin NX, Foxy, 192.168.123.18. 신호등 YOLO만 돈다.
  - 로봇: 192.168.123.161
- **DDS**: `CYCLONEDDS_URI=file://~/ws/junk/config/cyclonedds_junsu.xml`. 점구름 멀티캐스트가 100 Mb/s 포트를 넘치게 해서 XT16이 7.6 Hz로 떨어진 적이 있어 spdp만 멀티캐스트로 바꿨다. 새 노드도 이 xml로 띄울 것.
- **IMU는 6축**이라 9축 전용인 LIO-SAM은 안 된다(확인 끝, 다시 시도하지 말 것).
- **FAST-LIO가 흔들리는 조건** (실측):
  - 15 m 넘는 먼 점: max_range 15로 해결
  - 계단·주차 충격: 다리 게이트로 일부 해결
  - 출발 전 엎드린 자세로 잡은 초기자세
- **규정**:
  - 접촉 시 1초에 −10점
  - **경기 중 센서 데이터 외부 전송 금지**: Foxglove 같은 스트리밍은 끈다
  - 기립한 채 서 있기만 해도 뒷다리 hip이 과열된다(79°C). 기립 후 30초 안에 출발

## 7. 추천 접근 (참고 — 더 나은 방법이 있으면 그걸로)

1. **3D 지도 준비**: rec1650 `map.npz`(cnt≥2)를 5 cm 정도로 줄이고, 바닥 밴드를 빼고(벽·콘 위주), KD-tree나 voxel hash로 만든다.
2. **3D ICP 재정렬**
   - 입력: `/cloud_registered_body` 최근 1~2초를 FAST-LIO 자세로 묶은 것.
   - 초기값: 지금 T.
   - 방법: point-to-plane이 좋다. 처음엔 2D(x, y, yaw)만 풀고 z·roll·pitch는 고정한다.
   - 거부 조건: 일치율이 낮거나, 결과가 탐색 범위 밖이거나, 둘째 해와 차이가 작을 때(모호함).
   - 처리량: 요청이 올 때만 돌린다. 상시 20 Hz로 돌리지 않는다.
3. **AMCL**: nav2 AMCL을 rec1650 2D 지도로 돌린다. 초기자세는 T·FAST-LIO로 자동 주입한다. FAST-LIO 대비 어긋남이 30 cm 또는 8°를 1초 넘게 지속하면 `loc_watch` STOP.
   - 기존 문턱은 `amcl_watch_core.py`에 있다.
   - 오늘 경기장은 콘이 지도에 들어 있으므로, 예전 "슬라럼 구간에서는 감시 안 함" 예외는 다시 따져 볼 것.
4. **검증**: 재생에서 녹화 궤적을 기준으로, 일부러 넣은 오차(0.1/0.3/0.5 m, 5/10/20°)를 얼마나 되찾는지 표로 낸다.
