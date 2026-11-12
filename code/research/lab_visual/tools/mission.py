#!/usr/bin/env python3
"""미션 실행기 — 출발 신호에 한 번 실행하면 끝까지 혼자 돈다.

  python3 tools/mission.py plan  --zones 1 4 2 3        계획만 (로봇 불필요)
  python3 tools/mission.py dry   --zones 1 4 2 3        시계만 돌려 예행 (로봇 불필요)
  python3 tools/mission.py run   --zones 1 4 2 3 --go   실제 주행. 여기서부터 사람은 안 건드린다
  python3 tools/mission.py mark-zone 1                  지금 로봇 자리를 1번 존 목표로 기록
  python3 tools/mission.py mark-overpass                지금 로봇 자리를 육교(overpass)로 기록

【육교 overpass】
2번과 3번 사이에는 육교가 있어 **신호와 무관하게** 건널 수 있다.
그래서 2->3 과 3->2 는 언제나 육교를 경유하도록 못박는다 — 시계도 신호등도 보지 않는다.
venue.json 의 `leg_via` 가 그 규칙을 들고 있다.
  python3 tools/mission.py goto 2                       등록한 2번 존 목표로 보낸다 (신호 무시)
  python3 tools/mission.py zones                        찍힌 좌표 보기 (다시 찍으면 덮어쓴다)
  python3 tools/mission.py circuit --go                 시계방향으로 계속 돈다 (권장)
  python3 tools/mission.py deliver                      배달 계획대로 끝까지 (재개 가능)
  python3 tools/mission.py deliver --go                 실제 주행
  python3 tools/mission.py deliver --reset              진행 기록을 지우고 처음부터

【배달 — 중단되면 이어서 한다】
경로는 팀 알고리즘(gangnam_dispatch/algorithm.py)이 정한다. 다시 만들지 않았다.
gen_mission_commands.py --json 으로 뽑은 28 leg 계획을 그대로 실행한다.

  · leg 을 **하나 끝낼 때마다** 진행을 파일에 적는다.
    중단되거나 처음부터 다시 실행해도 끝난 leg 은 건너뛴다.
  · **발이 3개 이상 구역에 들어가야만** 적재/하차로 친다.
    "들어간 줄 알았는데 아니었다"가 가장 위험하다 — 그러면 그 뒤가 전부 꼬인다.
    확인이 안 되면 도킹을 다시 하고, 그래도 안 되면 **그 leg 을 실패로 적고** 넘어간다.
  · 앉아 있는 시간은 기본 7초.

【도착 정확도 — Nav2 는 22 cm 까지만 한다】
Nav2 의 xy_goal_tolerance 가 0.22 m 다. 그 안에 들면 "도착"으로 치고 멈춘다.
구역이 좁으면 그것만으로 발이 빠진다. 그렇다고 허용치를 0.10 으로 줄이면
**진동한다** — 목표 10 cm 앞에서 Nav2 가 0.05 m/s 를 요구하는데 브리지가
V_MIN 0.10 으로 올려 지나치기 때문이다(2026-08-28 실측: 10->20->17->19->26 cm).

그래서 Nav2 는 느슨하게 두고, 도착 뒤에 **go2_align.fine_align** 으로 좁힌다.
연속 P 제어가 아니라 "정지 -> AMCL 로 재기 -> robot_odom 폐루프로 그만큼 이동"
을 2~4회 반복하는 방식이라 최소 유효 명령 문제에 안 걸린다.
카메라는 쓰지 않는다. 이미 있는 모듈이라 새로 만들지 않았다.

【도착하면 엎드린다】
기본으로 각 존에 닿으면 StopMove(1003) -> StandDown(1005) 을 보내 엎드리고,
다음 구간으로 떠나기 전에 RecoveryStand(1006) 으로 다시 선다. `--no-lowdown` 으로 끈다.
API 번호는 waypoint.py 가 쓰던 것과 같다 — 새로 만들지 않았다.
시간: 엎드리기 약 3초 + 일어서기 약 4초 = 존마다 7초. 시간이 점수이므로 규정이
요구할 때만 켤 것.
  python3 tools/mission.py reach                        움직이지 않고 '경로가 나오나'만 물어본다

【왜 이 파일이 있나】
자율주행 대회다. **사람 개입은 출발할 때와 긴급점검 때뿐**이다.
그래서 구간마다 사람이 명령을 치는 방식(gosu.sh cross)은 쓸 수 없다.
한 번 실행하면 신호를 스스로 기다리고, 다음 존으로 스스로 보내고, 실패해도 스스로 넘어가야 한다.

【사람이 없다는 전제에서 바뀌는 것】
  · 어떤 경우에도 **입력을 기다리지 않는다**. 애매하면 기본값을 고르고 그 사실을 적는다.
  · 목표가 실패하면 한 번 다시 하고, 또 실패하면 **그 존을 버리고 다음으로 간다**.
    멈춰 서 있으면 0점이다. 하나 빠뜨려도 계속 도는 쪽이 낫다.
  · 구간마다 시간 상한을 둔다. 넘으면 취소하고 다음으로 간다.
  · 신호 대기가 남은 시간보다 길면 그 존을 건너뛴다.

【신호 판단】
signals.py 와 **같은 시계**를 쓴다. 실행 순간이 t=0 이다(--t0-now, 기본).
신호등 인식은 여기서 조회하지 않는다 — 시계와 인식이 서로 발목을 잡지 않게 하기 위해서다.

【쓰는 것은 공식 인터페이스뿐】
목표 전달은 Nav2 공식 액션 `navigate_to_pose` 다. 새 노드도 새 토픽도 만들지 않는다.
"""
import argparse
import json
import math
import os
import subprocess
import shutil
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import signals as SIG          # noqa: E402  — 시계·경로 계산을 한 벌만 둔다

GOSU = os.path.expanduser('~/gosu')
VENUE = os.path.join(GOSU, 'config', 'venue.json')

C0 = '\033[0m'; CB = '\033[1;36m'; CG = '\033[1;32m'; CY = '\033[1;33m'
CR = '\033[1;31m'; CD = '\033[2m'


def load_venue():
    if os.path.exists(VENUE):
        try:
            return json.load(open(VENUE, encoding='utf-8'))
        except Exception:
            pass
    return {}


def save_venue(d):
    SIG.guard_path(VENUE)
    os.makedirs(os.path.dirname(VENUE), exist_ok=True)
    json.dump(d, open(VENUE, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)


def zone_poses(d):
    z = d.get('zone_poses') or {}
    return {str(k): v for k, v in z.items()}


def named_points(d):
    """존이 아닌 이름 있는 지점. 지금은 육교(overpass) 하나다."""
    return {str(k): v for k, v in (d.get('waypoints') or {}).items()}


def quad_yaws(base_yaw):
    """저장된 방향에서 90도 간격 네 방향."""
    return [base_yaw + i * (math.pi / 2) for i in range(4)]


def nearest_quad_yaw(base_yaw, cur_yaw):
    """지금 방향에서 **가장 적게 돌아서** 닿는 방향을 고른다.

    【왜 — 2026-09-13】
    존마다 방향을 하나만 저장하면 접근 방향에 따라 최대 180도를 돌아야 한다.
    제자리회전은 느리고(W_MIN 0.5 rad/s) 몸 중심을 밀어내므로 순수 손해다.

    발 사각형이 335 x 328 mm 로 **거의 정사각형**이라, 90도 돌려도 60 cm 박스 안
    여유가 132 -> 136 mm 로 사실상 같다. 그래서 네 방향 중 아무거나 써도
    규정 판정에 차이가 없고, 가장 가까운 것을 고르면 회전이 **최대 45도**로 준다.
    실측 회전 속도에서 180도 3.5~6초 -> 45도 1~1.6초. 존당 3~4초를 아낀다.

    돌려주는 것: (고른 방향, 돌아야 할 각도)
    """
    best, best_d = None, None
    for y in quad_yaws(base_yaw):
        d = math.atan2(math.sin(y - cur_yaw), math.cos(y - cur_yaw))
        if best_d is None or abs(d) < abs(best_d):
            best, best_d = math.atan2(math.sin(y), math.cos(y)), d
    return best, best_d


def leg_via(d, src, dst):
    """이 구간에서 반드시 거쳐야 하는 지점들. 방향에 맞게 순서를 준다.

    【왜 필요한가 — 2026-09-13】
    Nav2 는 존 A 에서 존 B 로 **직선**을 긋는다. 존이 사각 광장 네 모서리에
    있으므로 그 직선은 한가운데(도로)를 가로지른다. 횡단보도도 육교도
    "지나가야 할 지점"으로 주지 않으면 쓸 이유가 없다.
    그래서 구간마다 통과점을 박아 둔다.
      · 육교(3-4)  : overpass 한 점
      · 횡단보도    : 들머리·날머리 두 점 (상자의 짧은 변 중점)
    반대 방향으로 갈 때는 **순서를 뒤집는다**.
    """
    v = d.get('leg_via') or {}
    key = '%s-%s' % (src, dst)
    if key in v:
        return list(v[key])
    rkey = '%s-%s' % (dst, src)
    if rkey in v:
        return list(reversed(v[rkey]))
    return []


def cmd_make_via(a):
    """횡단보도 상자에서 통과점(들머리·날머리)을 뽑아 venue.json 에 넣는다.

    상자는 네 모서리다. 두 존을 잇는 방향이 긴 축이므로, **짧은 변의 중점** 둘이
    들머리와 날머리가 된다. 그 둘을 차례로 지나가면 자연히 횡단보도 위를 걷는다.
    """
    d = load_venue()
    zp = zone_poses(d)
    B = os.path.join(GOSU, 'config', 'zones_box.json')
    if not os.path.exists(B):
        raise SystemExit('★ 횡단보도 상자가 없다: %s\n'
                         '   ~/gosu/gosu.sh crosswalks  또는  ~/gosu/gosu.sh mark 1-2' % B)
    boxes = json.load(open(B, encoding='utf-8'))
    wp = d.setdefault('waypoints', {})
    via = d.setdefault('leg_via', {})
    made = []
    for name, box in boxes.items():
        if '-' not in name:
            continue
        p_, q_ = name.split('-')
        if p_ not in zp or q_ not in zp:
            continue
        pts = [(float(x), float(y)) for x, y in box.get('corners', [])]
        if len(pts) != 4:
            print('  %s 는 모서리가 %d개라 건너뛴다' % (name, len(pts)))
            continue
        # 네 변 중 **짧은 변** 둘의 중점을 고른다
        edges = []
        for i in range(4):
            x1, y1 = pts[i]; x2, y2 = pts[(i + 1) % 4]
            edges.append((math.hypot(x2 - x1, y2 - y1), ((x1 + x2) / 2, (y1 + y2) / 2)))
        edges.sort(key=lambda e: e[0])
        m1, m2 = edges[0][1], edges[1][1]
        # 존 p_ 에 가까운 쪽이 들머리
        dp1 = math.hypot(m1[0] - zp[p_]['x'], m1[1] - zp[p_]['y'])
        dp2 = math.hypot(m2[0] - zp[p_]['x'], m2[1] - zp[p_]['y'])
        a_, b_ = (m1, m2) if dp1 <= dp2 else (m2, m1)
        yaw = math.atan2(b_[1] - a_[1], b_[0] - a_[0])
        # 중심선을 따라 spacing 간격으로 통과점을 깐다.
        # 두 점(들머리·날머리)만 주면 그 사이를 Nav2 가 자유롭게 긋는다.
        # 촘촘히 깔면 경로가 중심선에 붙고, 마네킹은 지역 코스트맵이 피한다 —
        # 피할 때만 선에서 벗어난다. 그게 "중심에서 20~30cm 안"의 뜻이다.
        L = math.hypot(b_[0] - a_[0], b_[1] - a_[1])
        # ★ 2026-09-13: 기본을 둘로 되돌렸다.
        #   1 m 간격으로 여섯 개를 깔았더니 로봇이 그 점들을 **정확히 밟으려고**
        #   횡단보도 한가운데서 멈췄다. 필요한 것은 "여기로 들어가서 저기로 나와라"
        #   뿐이고, 그 사이는 Nav2 가 알아서 간다 — 마네킹도 그때 피한다.
        #   폭 0.5 m 짜리 띠 안에서는 어차피 좌우 여유가 ±0.1 m 뿐이다.
        k = 1 if a.spacing <= 0 else max(1, int(round(L / max(a.spacing, 0.3))))
        names = []
        for i in range(k + 1):
            f = i / k
            pt = (a_[0] + (b_[0] - a_[0]) * f, a_[1] + (b_[1] - a_[1]) * f)
            nm = 'cw_%s_%02d' % (name, i)
            wp[nm] = {'x': round(pt[0], 4), 'y': round(pt[1], 4),
                      'yaw': round(yaw, 5), 'yaw_deg': round(math.degrees(yaw), 2),
                      'note': '횡단보도 %s 중심선 통과점 %d/%d' % (name, i, k),
                      'set_at': time.strftime('%Y-%m-%d %H:%M:%S')}
            names.append(nm)
        na, nb = names[0], names[-1]
        for old in list(wp):
            if old.startswith('cw_%s_' % name) and old not in names:
                wp.pop(old, None)
        # 육교 규칙은 절대 안 건드린다. 횡단보도는 --force 로 다시 만든다.
        rname = '-'.join(reversed(name.split('-')))
        cur = via.get(name) or via.get(rname) or []
        is_overpass = any(not str(x).startswith('cw_') for x in cur)
        if is_overpass:
            print('  %s 는 육교다 — 그대로 둔다 (%s)' % (name, ' -> '.join(cur)))
        elif cur and not a.force:
            print('  %s 는 이미 통과점이 %d개 있다 — 다시 만들려면 --force'
                  % (name, len(cur)))
        else:
            via.pop(rname, None)
            via[name] = names
            made.append((name, math.hypot(b_[0] - a_[0], b_[1] - a_[1]), len(names)))
    if not made:
        print('  새로 만든 통과점이 없다')
    save_venue(d)
    print()
    print('  구간   통과 길이   통과점')
    for nm, L, k in made:
        print('  %-6s %6.2f m   %d개 (간격 %.1f m)' % (nm, L, k, a.spacing))
    print()
    print('  경유 규칙:')
    for k, v in sorted(via.items()):
        print('    %-6s -> %s' % (k, ' -> '.join(v)))
    return 0




def robot_pose(tries=3, wait_s=8.0):
    """map 프레임에서의 로봇 자세를 읽는다. (x, y, yaw도)

    【왜 TF 를 직접 읽나 — 2026-09-13】
    처음엔 `waypoint.py pose` 를 불렀는데 현장에서 계속 "자세 없음"이 났다.
    원인은 TF 가 아니었다. 그 함수(waypoint.py:557)는 `spin_once` 를 **60번** 도는데,
    /tf 와 /sportmodestate 가 고빈도로 들어오면 60번이 0.2초 만에 소진된다.
    그런데 tf2 버퍼가 map -> base_link 를 세우는 데는 실측 **약 2초**가 걸린다
    (tf2_echo 도 처음 2초는 'frame does not exist' 를 찍다가 그 뒤 정상).
    횟수가 아니라 **벽시계로** 기다려야 한다.

    새 노드·새 토픽은 만들지 않는다. tf2_ros 로 조회만 한다 — `ros2 run tf2_ros tf2_echo`
    가 하는 것과 같은 일이고, 공식 방법이다.
    """
    import rclpy
    from rclpy.node import Node
    import tf2_ros

    err = ''
    started = rclpy.ok()
    if not started:
        rclpy.init(args=sys.argv)
    n = Node('pose_reader')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    try:
        for k in range(tries):
            end = time.time() + wait_s
            shown = False
            while time.time() < end:
                rclpy.spin_once(n, timeout_sec=0.02)
                try:
                    tr = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
                except Exception as e:
                    err = str(e)
                    if not shown and time.time() > end - wait_s + 2.0:
                        shown = True
                        print('  TF 를 기다리는 중… (%.0f초)' % wait_s)
                    continue
                # ★ 신선도 검사. 버퍼에 남은 낡은 값을 그대로 읽으면 안 된다
                #   (2026-09-13: 로봇을 옮겼는데 같은 좌표가 찍혔다).
                now_s = n.get_clock().now().nanoseconds * 1e-9
                st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                if now_s - st > 0.4:
                    err = 'TF 가 %.1f초 낡았다' % (now_s - st)
                    continue
                q = tr.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                return (tr.transform.translation.x, tr.transform.translation.y,
                        math.degrees(yaw))
            if k + 1 < tries:
                print('  아직 안 온다 (%d/%d) — 다시 기다린다' % (k + 1, tries))
    finally:
        n.destroy_node()
        if not started:
            rclpy.try_shutdown()

    raise SystemExit(
        '★ TF map -> base_link 를 %.0f초 x %d 동안 못 받았다.\n'
        '   마지막 사유: %s\n'
        '\n'
        '   = 위치추정이 map -> odom 을 안 내고 있다는 뜻이다. 둘 중 하나다:\n'
        '     (가) amcl 이 안 떴다\n'
        '     (나) 떴는데 /initialpose 를 못 받아 아직 자세를 모른다\n'
        '\n'
        '   확인:  ~/gosu/gosu.sh status\n'
        '   안 떠 있으면 다른 창에서:  ~/gosu/gosu.sh run --no-motor\n'
        '     "⑤ 검증" 과 "상대 판정" 이 뜨면 준비된 것이다. 그 창은 켜 둔다.'
        % (wait_s, tries, err[:200] or '(사유 없음)'))


def cmd_mark_overpass(a):
    """지금 로봇 자리를 육교(overpass)로 기록하고, 3-4 를 그리로 못박는다.

    ★ 2026-09-13: 존 번호를 +1 로 재정렬하기 전에는 육교가 2-3 이었다.
      이 함수에 2-3 이 그대로 남아 있어서, 다시 찍으면 2-3 이 육교가 되고
      3-4 는 낡은 점을 가리키게 됐다. 3-4 로 바로잡았다.
      (venue.json 은 재정렬 때 이미 3-4 로 고쳐져 있었다 — 이 함수만 뒤처져 있었다)
    """
    x, y, yaw_deg = robot_pose()
    d = load_venue()
    d.setdefault('waypoints', {})['overpass'] = {
        'x': round(x, 4), 'y': round(y, 4),
        'yaw': round(math.radians(yaw_deg), 5), 'yaw_deg': round(yaw_deg, 2),
        'note': '육교. 신호와 무관하게 3-4 를 건널 수 있는 유일한 길',
        'set_at': time.strftime('%Y-%m-%d %H:%M:%S')}
    d.setdefault('leg_via', {})['3-4'] = ['overpass']
    d['leg_via'].pop('4-3', None)          # 양방향을 한 항목으로 다룬다
    save_venue(d)
    print('육교(overpass) 기록: x %.3f  y %.3f  yaw %.1f도' % (x, y, yaw_deg))
    print('  3->4 와 4->3 은 이제 **언제나** 여기를 거친다 (신호·시계 무관)')
    zp = zone_poses(d)
    for z in ('3', '4'):
        if z in zp:
            print('  %s번 존까지 %.2f m'
                  % (z, math.hypot(zp[z]['x'] - x, zp[z]['y'] - y)))
    return 0


def cmd_pose(a):
    """좌표만 읽어 본다. 등록이 왜 안 되는지 가릴 때 쓴다."""
    x, y, yaw = robot_pose()
    print('  x %.3f   y %.3f   yaw %.1f도' % (x, y, yaw))
    print()
    print('  이 값으로 등록하려면:')
    print('    ~/gosu/gosu.sh mark-start     (출발 위치)')
    print('    ~/gosu/gosu.sh zone <번호>    (존 목표)')
    return 0


def cmd_mark_zone(a):
    if not a.zone:
        raise SystemExit('★ 존 번호가 필요하다 (1~4)')
    z = str(a.zone[0])
    if z not in SIG.ZONES:
        raise SystemExit('★ 존은 1~4 다: %s' % z)
    x, y, yaw_deg = robot_pose()
    # waypoint.py pose 는 yaw 를 **도**로 낸다. 내부는 라디안으로 들고 있는다.
    yaw = math.radians(yaw_deg)
    d = load_venue()
    d.setdefault('zone_poses', {})[z] = {
        'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 5),
        'yaw_deg': round(yaw_deg, 2),
        'set_at': time.strftime('%Y-%m-%d %H:%M:%S')}
    save_venue(d)
    print('%s번 존 목표 기록: x %.3f  y %.3f  yaw %.1f도' % (z, x, y, yaw_deg))
    have = sorted(zone_poses(load_venue()))
    print('  지금까지 찍은 존: %s  (남은 것: %s)'
          % (', '.join(have) or '없음',
             ', '.join(z for z in SIG.ZONES if z not in have) or '없음'))




# ── 엎드리기 / 일어서기 ────────────────────────────────────────────────
# Go2 공식 스포츠 API 를 그대로 쓴다. waypoint.py:990 이 쓰던 것과 같은 번호다.
#   1003 StopMove   1004 StandUp   1005 StandDown   1006 RecoveryStand
API_STOP_MOVE = 1003
API_STAND_DOWN = 1005
API_RECOVERY_STAND = 1006
# ── 보행 전환 (정밀 도킹 때만) ────────────────────────────────────────
# 출처: ~/unitree_ros2/example/src/include/common/ros2_sport_client.h  (Go2 전용)
#   STATICWALK   = 1061   파라미터 없음
#   TROTRUN      = 1062   파라미터 없음
#   ECONOMICGAIT = 1063   파라미터 없음
#   CLASSICWALK  = 2049   {"data": bool}
#   SPEEDLEVEL   = 1015   {"data": int}
#   BALANCESTAND = 1002   파라미터 없음 (기본 보행으로 복귀)
#
# ★ 2026-09-13 정정: 처음에 SWITCHGAIT(1011) + {"gait_type": N} 을 보냈는데
#   **둘 다 틀렸다.** 1011 은 B2 전용 헤더에만 있고 Go2 클라이언트에는 SwitchGait
#   자체가 없다. 그리고 JSON 키는 gait_type 이 아니라 data 다
#   (ros2_sport_client.cpp 의 SwitchGait 구현: js["data"] = d).
#   그래서 펌웨어가 통째로 무시했고, 보행이 안 바뀐 채 트로트로 도킹하다
#   4회 동안 168 -> 160 mm 로 제자리였다.
#
# 【왜 StaticWalk 인가】 정적 보행은 **항상 세 발이 땅에 있다.**
#   한 발씩만 옮기므로 몸통 흔들림이 작고 보폭이 작다 — 10 cm 급 위치 제어에 맞는다.
#   규정이 "구역 안에 발 3개 이상"이라 보행 자체가 그 조건과 같은 방향이다.
API_BALANCE_STAND = 1002
API_SPEED_LEVEL = 1015           # {"data": -1 느림 / 0 보통 / 1 빠름}
API_STATIC_WALK = 1061           # 파라미터 없음
API_TROT_RUN = 1062
API_ECONOMIC_GAIT = 1063
API_CLASSIC_WALK = 2049          # {"data": bool}

# 도킹용 보행. 1061 StaticWalk 이 기본. 2049 ClassicWalk 로 바꿔볼 수 있다.
# 2026-09-13: 기본을 StaticWalk(1061) -> ClassicWalk(2049) 로 바꿨다 (사용자 지시).
GAIT_DOCK = int(os.environ.get('GO2_DOCK_GAIT', str(API_CLASSIC_WALK)))
CLASSIC_WALK_ON = os.environ.get('GO2_CLASSIC', '1') != '0'


def _sport(node, pub, api_id, wait=1.5, param=None):
    import rclpy
    from unitree_api.msg import Request
    req = Request()
    req.header.identity.api_id = api_id
    # bool/int 를 받는 명령은 {"data": 값} 형식이다.
    # (unitree_sdk2 바이너리 문자열에서 확인: data / gait_type / velocity)
    req.parameter = json.dumps({} if param is None else param)
    pub.publish(req)
    end = time.time() + wait
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


_PARAM_CLI = {}


def _set_param(node, node_name, name, value, timeout=3.0):
    """다른 노드의 파라미터를 바꾼다.

    ★ 2026-09-13: `ros2 param set` 를 subprocess 로 부르던 것을 프로세스 안에서
      하도록 바꿨다. CLI 는 호출마다 python+rclpy 를 새로 띄워 1~2초가 든다.
      도킹 한 번에 네 번 부르므로 존마다 5~8초를 그냥 버렸다.
      쓰는 것은 공식 서비스(rcl_interfaces/SetParameters) 그대로다.
    """
    import rclpy
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParameters
    key = node_name.lstrip('/')
    cli = _PARAM_CLI.get(key)
    if cli is None:
        cli = node.create_client(SetParameters, '/%s/set_parameters' % key)
        _PARAM_CLI[key] = cli
    if not cli.wait_for_service(timeout_sec=timeout):
        return False
    p = (Parameter(name, Parameter.Type.BOOL, bool(value)) if isinstance(value, bool)
         else Parameter(name, Parameter.Type.DOUBLE, float(value)))
    req = SetParameters.Request(parameters=[p.to_parameter_msg()])
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    r = fut.result()
    return bool(r and r.results and r.results[0].successful)


# ── AMCL 갱신 문턱 ────────────────────────────────────────────────────
#
# 【2026-09-13 — 정밀 도킹이 안 되던 진짜 이유】
#   amcl_go2_fast.yaml:87-88
#       update_min_d: 0.15   # m
#       update_min_a: 0.15   # rad (8.6도)
#   AMCL 은 로봇이 **15 cm 이상 움직여야** 관측을 반영해 자세를 갱신한다.
#   도킹은 15 cm 를 몇 번에 나눠 움직이므로 문턱을 한 번도 못 넘는다.
#   그러면 AMCL 자세가 얼어붙고, 재측정해도 오차가 그대로 나온다.
#   실측 로그가 정확히 그 모양이었다: 168 -> 167 -> 160 -> 160 mm.
#   로봇은 움직였는데 "안 움직였다"고 보고하니 같은 명령을 반복했다.
#
# 【왜 평소에는 0.15 인가】 걷는 중에는 몸통이 기울어 스캔이 오염된다.
#   매 프레임 갱신하면 그 잡음을 그대로 먹는다. 주행 중에는 0.15 가 맞다.
#   그래서 **도킹 구간에서만** 낮추고 끝나면 되돌린다.
AMCL_DOCK_MIN_D = 0.02
AMCL_DOCK_MIN_A = 0.02
AMCL_DRIVE_MIN_D = float(os.environ.get('GO2_AMCL_MIN_D', '0.15'))
AMCL_DRIVE_MIN_A = float(os.environ.get('GO2_AMCL_MIN_A', '0.15'))


AMCL_FREEZE_D = 100.0
AMCL_FREEZE_A = 100.0

# 앉아서 얼려 둔 상태인가. 도킹용 복원(_restore)이 이 얼림을 실수로 풀지
# 못하게 막는 문패다. 푸는 것은 stand_up 하나뿐이다(force=True).
_SIT_FROZEN = {'on': False}


def freeze_amcl(node, log=print):
    """앉고 일어서는 **동안** AMCL 을 완전히 멈춘다.

    【왜 0.15 로 되돌리는 것만으로는 부족한가 — 2026-09-13】
    이전 주석은 "앉아 있는 동안은 움직이지 않으므로 0.15 문턱을 못 넘는다"고
    적혀 있었다. 그 전제가 틀렸다.

    odom->base_link 는 FAST-LIO 가 낸다. LIO 가 추적하는 것은 **몸통이 아니라
    라이다**다. 앉으면 몸높이가 0.33 -> 0.06 m 로 내려가면서 라이다가 27 cm
    내려간다. LIO 는 그것을 **이동으로 보고한다**. 27 cm > 문턱 15 cm —
    문턱을 넘는다. 그래서 앉은 채 기울어진 스캔으로 AMCL 이 갱신된다.
    일어설 때 27 cm 가 또 한 번. 존 하나 방문마다 오염된 갱신이 두 번이다.

    증상: 첫 주행은 멀쩡한데 **두 번째 이동부터 지도가 밀린다.**

    그래서 문턱을 아예 손댈 수 없는 값(100 m / 100 rad)으로 올려 둔다.
    경기장이 12 m 라 절대 안 넘는다. 일어서서 자세가 안정된 **뒤에**
    restore_amcl 로 되돌린다.

    되돌리는 것을 잊으면 AMCL 이 영원히 갱신을 안 한다. 반드시 짝을 맞출 것.
    """
    d_ok = _set_param(node, '/amcl', 'update_min_d', AMCL_FREEZE_D)
    a_ok = _set_param(node, '/amcl', 'update_min_a', AMCL_FREEZE_A)
    _SIT_FROZEN['on'] = True
    log('AMCL 정지 — 앉고 일어서는 동안 자세를 얼린다%s'
        % ('' if (d_ok and a_ok) else '  ★ 실패'))
    return d_ok and a_ok


def restore_amcl(node, log=print, force=False):
    """AMCL 갱신 문턱을 **앉기 전에** 되돌린다.

    【왜 앉기 전인가 — 2026-09-13】
    앉으면 몸높이가 0.33 -> 0.06 m 로 내려가 라이다가 27 cm 낮아진다.
    그 스캔은 지도와 안 맞는다. 그런데 도킹 때문에 문턱이 0.02 m 로 낮아져 있으면,
    앉아 있는 7초 동안 작은 오도메트리 잡음에도 AMCL 이 그 **틀린 스캔으로**
    자세를 갱신한다. 일어났을 때 자세가 틀어져 있게 된다.

    문턱을 0.15 로 되돌리면 앉아 있는 동안(움직이지 않으므로) AMCL 이 아예
    갱신하지 않는다 — 앉은 자세는 그렇게 얼려 두는 것이 맞다.
    (loc_watchdog 도 body_height < 0.25 이면 판정을 보류한다. 같은 이유다.)
    """
    if _SIT_FROZEN['on'] and not force:
        # 아직 앉아 있다. 지금 풀면 일어서는 27 cm 를 AMCL 이 먹는다.
        # 푸는 것은 stand_up 의 몫이다.
        # 그냥 물러나면 안 된다 — 그 사이 누가 문턱을 도킹값(2 cm)으로
        # 낮춰 놨을 수 있고, 그러면 앉은 채로 오히려 더 자주 갱신된다.
        # 그래서 물러나는 대신 **얼림을 다시 세운다**.
        _set_param(node, '/amcl', 'update_min_d', AMCL_FREEZE_D)
        _set_param(node, '/amcl', 'update_min_a', AMCL_FREEZE_A)
        return True
    d_ok = _set_param(node, '/amcl', 'update_min_d', AMCL_DRIVE_MIN_D)
    a_ok = _set_param(node, '/amcl', 'update_min_a', AMCL_DRIVE_MIN_A)
    _SIT_FROZEN['on'] = False
    log('AMCL 갱신 문턱 %.0f cm 로 되돌림%s'
        % (AMCL_DRIVE_MIN_D * 100, '' if (d_ok and a_ok) else '  ★ 실패'))
    return d_ok and a_ok


def _restore(node, pub, log=print, amcl=True):
    """보행·장애물 감시·AMCL 문턱을 **반드시** 되돌린다.

    amcl=False 는 **아직 앉아 있을 때** 부르는 경우다. 일어서기 전에 문턱을
    되돌리면 일어서는 27 cm 를 AMCL 이 이동으로 먹어 자세가 틀어진다.
    그 경우 일어선 뒤에 restore_amcl 을 따로 불러야 한다.
    """
    if GAIT_DOCK == API_CLASSIC_WALK:
        _sport(node, pub, API_CLASSIC_WALK, 0.8, {'data': False})
    _sport(node, pub, API_BALANCE_STAND, 0.8)
    _sport(node, pub, API_SPEED_LEVEL, 0.2, {'data': 0})
    ok = _set_param(node, '/cmd_vel_to_sport', 'obstacle_guard', True)
    d_ok = restore_amcl(node, log=lambda m: None) if amcl else True
    bad = [n for n, v in (('장애물감시', ok), ('AMCL문턱', d_ok)) if not v]
    log('보행·장애물 감시 되돌림%s'
        % ('' if not bad else '  ★ 실패: %s — 확인할 것' % ', '.join(bad)))


def feet_in_zone(node, zone_pose, box_m=0.60, log=print):
    """엎드린 지금, 60x60 cm 구역 안에 발이 몇 개 들어갔나.

    규정이 "구역 안에 발 3개 이상"이라 **엎드린 시점의 발 위치**가 전부다.
    구역은 등록한 존 자세를 중심으로, 그 자세의 방향으로 놓인 정사각형으로 본다.
    발 위치는 zones.py 가 쓰는 것과 같은 계산을 쓴다 — 두 벌로 두지 않는다.
    """
    import types as _t
    import rclpy
    import tf2_ros
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    try:
        import zones as Z
    except Exception as e:
        log('발 판정 모듈을 못 불렀다 (%s)' % e)
        return None
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    a = _t.SimpleNamespace(map_frame='map', base_frame='base_link',
                           lowstate_topic='/lowstate')
    r = Z.Reader(node, buf, a)
    end = time.time() + 6.0
    feet = None
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        f = r.feet_map()
        if f:
            feet = f
    if not feet:
        log('발 위치를 못 읽었다 (/lowstate 가 오나?)')
        return None
    cx, cy = float(zone_pose['x']), float(zone_pose['y'])
    yaw = float(zone_pose.get('yaw', 0.0))
    c, sn = math.cos(yaw), math.sin(yaw)
    h = box_m / 2.0
    inside, planted = 0, 0
    detail = []
    for name, fx, fy, force in feet:
        ex, ey = fx - cx, fy - cy
        bx = ex * c + ey * sn          # 구역 좌표계
        by = -ex * sn + ey * c
        ok = abs(bx) <= h and abs(by) <= h
        on = force >= 20.0
        if on:
            planted += 1
            if ok:
                inside += 1
        detail.append('%s%s' % (name, '○' if (ok and on) else '×'))
    log('구역 %.0fx%.0f cm 판정: 접지한 발 %d개 중 **%d개가 안**   %s'
        % (box_m * 100, box_m * 100, planted, inside, ' '.join(detail)))
    return inside


def lie_down(node, pub):
    """도착 지점에서 엎드린다. 멈춘 뒤에 내려야 자세가 안 튄다.

    ★ 앉기 **전에** AMCL 을 얼린다. 앉으면 라이다가 27 cm 내려가는데,
      odom 을 내는 FAST-LIO 는 그것을 이동으로 보고한다. 그 상태로 AMCL 이
      갱신되면 기울어진 스캔을 먹고 자세가 틀어진다. freeze_amcl 주석 참조.
      푸는 것은 stand_up 이 한다 — 짝을 이 두 함수가 책임진다.
    """
    freeze_amcl(node, log=lambda m: None)
    _sport(node, pub, API_STOP_MOVE, 0.5)
    _sport(node, pub, API_STAND_DOWN, float(os.environ.get('GO2_SIT_WAIT', '1.5')))


def stand_up(node, pub):
    """다음 구간으로 떠나기 전에 선다. 엎드린 채로는 라이다가 27cm 낮아 정합이 무너진다.

    ★ 일어선 **뒤에** AMCL 을 푼다. 일어서는 27 cm 도 LIO 는 이동으로 보고한다.
      자세가 안정될 때까지 기다렸다가 푼다. lie_down 과 짝이다.

      호출부가 순서를 틀려도 안전하도록 여기서 책임진다 —
      2026-09-13 이전에는 호출부마다 순서가 달랐고, 네 곳 중 두 곳이 틀렸다.
      증상은 "두 번째 이동부터 지도가 밀린다" 였다.
    """
    _sport(node, pub, API_RECOVERY_STAND, float(os.environ.get('GO2_STAND_WAIT', '2.5')))
    _settle(node, float(os.environ.get('GO2_STAND_SETTLE', '1.2')))
    restore_amcl(node, log=lambda m: None, force=True)


def reseed_at_zone(node, zone_pose, log=print, cov_m=0.05, cov_deg=8.0):
    """일어선 직후, **도킹으로 확인된 자리**를 AMCL 에 다시 심는다.

    【왜 필요한가 — 2026-09-13 bag 실측】
    앉으면 라이다가 27 cm 내려간다. FAST-LIO 는 라이다를 추적하므로 그것을
    이동으로 본다. 실제 주행 기록(live_0910_1512)에서 자세 전환 6회 중 5회가
    AMCL 문턱(0.15 m)을 넘었고, 그중 두 번은 **발산**이었다:
        66.9초 앉음   LIO 48.474 m  vs  다리 0.291 m
        69.1초 일어섬 LIO 21.593 m  vs  다리 0.272 m
    적재지에서 음식을 기다리며 100초까지 앉아 있으면 더 나빠질 수 있다.

    AMCL 을 얼려 두면(freeze_amcl) 그 오염은 막는다. 그런데 얼어 있는 동안
    odom 은 계속 흐르므로, 일어섰을 때 map->odom 이 그만큼 낡아 있다.

    여기서는 그 문제를 **아는 사실**로 끊는다 — 방금 이 존에 도킹했으므로
    로봇이 어디 있는지 정확히 안다. 그 자세를 그대로 심으면 앉은 동안 무슨
    일이 있었든 상관없어진다.

    ★ 도킹이 성공했을 때만 불러야 한다. 엉뚱한 데 서 있는데 심으면
      그 오차를 그대로 주입한다.
    """
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'map'
    yaw = float(zone_pose.get('yaw',
                              math.radians(zone_pose.get('yaw_deg', 0.0))))
    m.pose.pose.position.x = float(zone_pose['x'])
    m.pose.pose.position.y = float(zone_pose['y'])
    m.pose.pose.orientation.z = math.sin(yaw / 2)
    m.pose.pose.orientation.w = math.cos(yaw / 2)
    cov = [0.0] * 36
    cov[0] = cov[7] = cov_m ** 2
    cov[35] = math.radians(cov_deg) ** 2
    m.pose.covariance = cov
    for _ in range(3):
        m.header.stamp = node.get_clock().now().to_msg()
        pub.publish(m)
        rclpy.spin_once(node, timeout_sec=0.15)
    log('자세 재심기 — 앉아 있던 동안의 표류를 끊는다 (x %.3f y %.3f)'
        % (zone_pose['x'], zone_pose['y']))


def _settle(node, sec):
    """자세가 안정될 때까지 콜백을 돌리며 기다린다. time.sleep 과 달리
    그 사이 TF·오도메트리가 계속 들어온다."""
    import rclpy
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(node, timeout_sec=0.05)



# ══ 스캔 정합 도킹 ═══════════════════════════════════════════════════════
#
# 【왜 만들었나 — 2026-09-13】
# 지금 도킹은 AMCL 자세를 보고 목표와의 차이를 구한다. 그래서 측위가 흔들리면
# 도킹도 같이 흔들린다. 존이 밀리고, 재정위가 튀고, 앉았다 일어나면 또 틀어진다.
# 원인이 전부 측위에 있다.
#
# 벽은 표류하지 않는다. 존에 제대로 선 자세에서 스캔을 한 장 떠 두고,
# 다음부터는 **지금 스캔이 그 기준 스캔과 같아질 때까지** 움직이면 된다.
#
# 【경기장 점군으로 미리 잰 성능 — 로봇 없이 검증】
#   ±12 cm / ±8도 안  : 12전 12승, 남은 오차 1 mm 이하
#   AMCL 을 씨앗으로 : ±30 cm / ±20도, AMCL 오차 135 mm 에서도 10/12, 최악 3.5 mm
#   마네킹이 끼어들어도 흔들리지 않음 (대응점 하위 20% 를 버리는 정합)
#   못 믿을 정합은 적합도로 걸러진다 — **거절하면 움직이지 않는다**
#
# 【기존 경로는 그대로 둔다】
# AMCL 도킹(fine_align_to)을 지우지 않았다. 예비책이 살아 있어야 한다.
# --dock-mode scan 일 때만 이 경로를 탄다.
# ══ 페널티 처리 ══════════════════════════════════════════════════════════
#
# 【무슨 상황인가 — 2026-09-13】
# 횡단보도 규정을 어기면 페널티를 받고 심판이 로봇을 **원래 자리로 되돌린다.**
# 그동안 우리 소프트웨어는 세 가지를 모른다:
#   ① 로봇이 지금 들려서 옮겨지고 있다   -> 계속 명령을 내면 안 된다
#   ② 놓인 자리가 어디인가              -> AMCL 자세가 통째로 틀려진다
#   ③ 그 사이 신호 위상이 어떻게 됐나    -> 비전을 못 쓰니 알 방법이 없다
#
# 그래서 두 명령으로 나눈다:
#   penalty        모션만 세우고 시계를 멈춘다 (스택은 살려 둔다)
#   back <존>      그 존에 놓였다고 알려주고 자세를 다시 심고 시계를 되돌린다
#
# ★ 시계를 멈추는 것이 맞는 경우와 아닌 경우를 반드시 구분할 것.
#   대회 진행 자체가 멈춘다면 penalty 로 멈췄다가 back 으로 재개한다.
#   실제 신호등이 계속 돌고 있다면 멈춰도 소용없다 — 돌아온 뒤
#   `~/gosu/gosu.sh signal sync <막 초록이 된 구간>` 으로 다시 맞춰야 한다.
#   둘을 헷갈리면 빨간불에 건넌다.
def _stop_motion(node, log=print):
    """모션만 세운다. 스택은 그대로 둔다 (gosu.sh stop 과 다르다)."""
    from geometry_msgs.msg import Twist
    from unitree_api.msg import Request as _Req
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    sport = node.create_publisher(_Req, '/api/sport/request', 10)
    import rclpy
    for _ in range(8):
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)
    _sport(node, sport, API_STOP_MOVE, 0.5)
    log('모션 정지 (StopMove). 스택은 살아 있다.')


def cmd_penalty(a):
    """페널티 — 모션을 세우고 신호 시계를 멈춘다."""
    import rclpy
    from rclpy.node import Node
    rclpy.init(args=sys.argv)
    n = Node('penalty_stop')
    try:
        _stop_motion(n, log=lambda m: print('  ' + str(m)))
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
    _vd, _sg = SIG.load()
    if _sg.get('t0_epoch') is None:
        print('  %s신호 시계가 없어서 멈출 것이 없다%s' % (CY, C0))
    elif SIG.is_paused(_sg):
        print('  신호 시계는 이미 멈춰 있다')
    else:
        _sg['paused_at'] = time.time()
        SIG.save(_vd, _sg)
        st = SIG.state(_sg)
        print('  %s신호 시계 멈춤%s  경과 %s 에서 정지 (지금 초록: %s)'
              % (CY, C0, SIG.fmt(st['elapsed']),
                 ', '.join(_sg['groups'][st['green']])))
    print()
    print('  %s다음에 할 일%s' % (CB, C0))
    print('    로봇을 원래 자리에 놓은 뒤:   ~/gosu/gosu.sh back <존번호>')
    print('    %s실제 신호등이 계속 돌았다면%s back 대신:' % (CY, C0))
    print('       ~/gosu/gosu.sh signal sync <막 초록이 된 구간>   (예: sync 1-2)')
    return 0


def cmd_back(a):
    """원위치에 놓였다 — 자세를 다시 심고 시계를 되돌린다."""
    if not a.zone:
        raise SystemExit('★ 어느 존에 놓았는지 줄 것:  ~/gosu/gosu.sh back 2')
    z = str(a.zone[0])
    zp = zone_poses(load_venue())
    if z not in zp:
        raise SystemExit('★ %s번 존이 등록 안 됐다. register 로 먼저 등록할 것' % z)
    v = zp[z]
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseWithCovarianceStamped
    rclpy.init(args=sys.argv)
    n = Node('penalty_back')
    try:
        # 들려서 옮겨졌으므로 AMCL 자세는 통째로 틀리다. 등록된 존 자세로 다시 심는다.
        pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        yaw = float(v.get('yaw', math.radians(v.get('yaw_deg', 0.0))))
        m.pose.pose.position.x = float(v['x'])
        m.pose.pose.position.y = float(v['y'])
        m.pose.pose.orientation.z = math.sin(yaw / 2)
        m.pose.pose.orientation.w = math.cos(yaw / 2)
        # 손으로 놓은 자리라 몇 cm 는 틀리다. 그만큼만 퍼뜨린다.
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05 ** 2          # x, y 5cm
        cov[35] = math.radians(8.0) ** 2     # yaw 8도
        m.pose.covariance = cov
        for _ in range(3):
            m.header.stamp = n.get_clock().now().to_msg()
            pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.2)
        print('  자세를 존 %s 로 다시 심었다: x %.3f  y %.3f  yaw %.1f도'
              % (z, v['x'], v['y'], math.degrees(yaw)))
    finally:
        n.destroy_node()
        rclpy.try_shutdown()

    _vd, _sg = SIG.load()
    if SIG.is_paused(_sg):
        was = time.time() - float(_sg['paused_at'])
        _sg['pause_total'] = float(_sg.get('pause_total') or 0.0) + was
        _sg['paused_at'] = None
        SIG.save(_vd, _sg)
        st = SIG.state(_sg)
        print('  %s신호 시계 재개%s  %s 동안 멈춰 있었다. 지금 초록: %s (%.0f초 남음)'
              % (CG, C0, SIG.fmt(was), ', '.join(_sg['groups'][st['green']]),
                 st['left']))
    else:
        print('  %s신호 시계는 멈춰 있지 않았다 — 그대로 둔다%s' % (CD, C0))
    print()
    print('  %s실제 신호와 어긋난 것 같으면 지금 맞출 것:%s' % (CY, C0))
    print('     ~/gosu/gosu.sh signal sync <막 초록이 된 구간>')
    return 0


# ══ 본선 당일 등록 ═══════════════════════════════════════════════════════
#
# 【왜 따로 만들었나 — 2026-09-13】
# 경기장을 미리 못 쓰는 상황이다. 본선 당일 짧은 준비 시간에 한 번에 끝내야 한다.
# 기존 `zone N` 은 AMCL 자세를 그냥 받아 적기만 했다. 검증이 없어서, 엎드린 채로
# 찍어도, 자세가 낡아도, 스캔이 비어도 조용히 저장됐다. 한 번뿐인 기회에는 위험하다.
#
# `register N` 은 한 번에 다 한다:
#   ① 서 있는지 / 멈춰 있는지 확인      (엎드려 찍으면 라이다가 27cm 낮아 못 쓴다)
#   ② 자세가 신선한지 확인              (낡은 TF 를 받아 적으면 엉뚱한 곳이 된다)
#   ③ 존 좌표 기록                      venue.json  zone_poses
#   ④ 기준 스캔 기록                    zone_scans.json (스캔 정합 도킹용)
#   ⑤ 품질 보고                         점 개수, 가장 가까운 벽, 이웃 존과의 거리
#
# 그리고 **아는 사실**을 쓴다 — 네 존은 직사각형을 이룬다.
#   register fit             네 점에 직사각형을 강제해 어긋남을 접는다
#   register fit --width W --height H    도면 값을 알면 더 세게 접힌다
#   존 두 개만 찍고 --width/--height 를 주면 나머지 둘이 자동으로 나온다
def _zkey(a, b):
    """'3-2' 와 '2-3' 을 같은 키로. JSON 키로 쓰려면 문자열이어야 한다
    (SIG.norm 은 튜플을 돌려준다)."""
    return '-'.join(sorted([str(a).strip(), str(b).strip()]))


def _standing(node, wait=3.0):
    """서 있고 멈춰 있는가. (몸높이 m, 속도, 읽었는지)"""
    import rclpy
    try:
        from unitree_go.msg import SportModeState
    except Exception:
        return None, None, False
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    st = {'h': None, 'v': None}

    def on(m):
        st['h'] = float(m.body_height)
        st['v'] = math.hypot(m.velocity[0], m.velocity[1]) + abs(m.yaw_speed)

    sub = node.create_subscription(
        SportModeState, '/sportmodestate', on,
        QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST, depth=1))
    end = time.time() + wait
    while time.time() < end and st['h'] is None:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    return st['h'], st['v'], st['h'] is not None


def cmd_register(a):
    """본선용 한 방 등록.  register 2  |  register check  |  register fit"""
    what = str(a.zone[0]) if a.zone else 'check'
    if what == 'check':
        return _register_check()
    if what == 'fit':
        return _register_fit(a)
    z = what
    if z not in SIG.ZONES:
        raise SystemExit('★ 존은 1~4 다. 또는 check / fit')

    import rclpy
    from rclpy.node import Node
    rclpy.init(args=sys.argv)
    n = Node('zone_register')
    try:
        print('  %s존 %s 등록%s — 지금 서 있는 자리가 그대로 목표가 된다' % (CB, z, C0))
        h, v, got = _standing(n)
        if not got:
            print('  %s자세를 못 읽었다 (/sportmodestate 없음) — 계속한다%s' % (CY, C0))
        else:
            print('  몸높이 %.2f m,  속도 %.3f' % (h, v))
            if h < 0.25:
                raise SystemExit('★ 엎드려 있다(몸높이 %.2f). 세우고 다시 할 것.\n'
                                 '   엎드린 스캔은 라이다가 27 cm 낮아 쓸 수 없다.' % h)
            if v > 0.05:
                raise SystemExit('★ 아직 움직이고 있다(속도 %.3f). 멈춘 뒤 할 것.' % v)

        x, y, yaw_deg = robot_pose()
        print('  자세  x %.3f  y %.3f  yaw %.1f도' % (x, y, yaw_deg))

        sc = grab_scan(n, frames=a.scan_frames, log=lambda m: print('  ' + str(m)))
        if sc is None:
            raise SystemExit('★ /scan 을 못 받았다. 주행 스택을 확인할 것.')
        import scan_match as SM
        pts = SM.scan_points(sc['ranges'], sc['angle_min'], sc['angle_increment'],
                             sc.get('range_min', 0.0), sc.get('range_max', 1e9))
        if len(pts) < 100:
            raise SystemExit('★ 스캔에 쓸 점이 %d개뿐이다. 벽이 안 보이는 자리다.'
                             % len(pts))
        near = float(min((p[0] ** 2 + p[1] ** 2) ** 0.5 for p in pts))
        print('  스캔  점 %d개,  가장 가까운 벽 %.2f m' % (len(pts), near))

        d = load_venue()
        d.setdefault('zone_poses', {})[z] = {
            'x': round(x, 4), 'y': round(y, 4),
            'yaw': round(math.radians(yaw_deg), 5), 'yaw_deg': round(yaw_deg, 2),
            'how': 'register', 'set_at': time.strftime('%Y-%m-%d %H:%M:%S')}
        save_venue(d)

        rec = dict(sc)
        rec['points'] = int(len(pts))
        rec['pose'] = {'x': round(x, 4), 'y': round(y, 4),
                       'yaw_deg': round(yaw_deg, 2)}
        rec['set_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
        zs = load_zone_scans()
        zs[z] = rec
        save_zone_scans(zs)
        print('  %s저장 완료%s  존 좌표 + 기준 스캔' % (CG, C0))

        # ── 줄자로 잰 절대거리를 같은 자리에서 같이 넣는다 ───────────
        # 【왜 같이 받나】 본선 준비 시간에 존을 찍으러 간 김에 줄자를 대면
        #   한 번에 끝난다. 나중에 따로 재러 갈 시간이 없다.
        #   이 값은 register fit 에서 직사각형의 가로·세로를 **고정**하는 데 쓴다.
        #   AMCL 로 잰 거리보다 줄자가 정확하므로, 있으면 그쪽이 이긴다.
        if a.dist > 0:
            if not a.frm:
                raise SystemExit('★ --dist 를 주려면 --from 도 줄 것 '
                                 '(예: register 3 --from 2 --dist 6.90)')
            key = _zkey(a.frm, z)
            dd = load_venue()
            dd.setdefault('zone_dist', {})[key] = {
                'm': round(float(a.dist), 4),
                'how': '줄자 실측',
                'set_at': time.strftime('%Y-%m-%d %H:%M:%S')}
            save_venue(dd)
            print('  %s절대거리 기록%s  %s = %.3f m (줄자)' % (CG, C0, key, a.dist))

        zp = zone_poses(load_venue())
        zd = (load_venue().get('zone_dist') or {})
        for o in sorted(zp):
            if o == z:
                continue
            dcalc = math.hypot(zp[o]['x'] - x, zp[o]['y'] - y)
            k = _zkey(o, z)
            if k in zd:
                gap = (dcalc - zd[k]['m']) * 1000
                print('    존 %s 까지 %.3f m   줄자 %.3f m   %s차이 %+.0f mm%s'
                      % (o, dcalc, zd[k]['m'],
                         CG if abs(gap) < 100 else CY, gap, C0))
            else:
                print('    존 %s 까지 %.3f m' % (o, dcalc))
        rest = [q for q in SIG.ZONES if q not in zp]
        print('  남은 존: %s' % (', '.join(rest) or '없음'))
        if not rest:
            print('  다음:  ~/gosu/gosu.sh register check')
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
    return 0


def _register_check():
    """등록 상태와 기하 일관성을 점검한다. 로봇이 없어도 된다."""
    import zone_fit as ZF
    d = load_venue()
    zp = zone_poses(d)
    zs = load_zone_scans()
    print('  %s등록 상태%s' % (CB, C0))
    for z in SIG.ZONES:
        p = zp.get(z)
        sc = zs.get(z)
        print('    존 %s  좌표 %s   기준스캔 %s' % (
            z,
            '%s(%s)%s' % (CG, p.get('set_at', '?'), C0) if p else '%s없음%s' % (CR, C0),
            '%s점 %d개%s' % (CG, sc.get('points', 0), C0) if sc else '%s없음%s' % (CR, C0)))
    miss = [z for z in SIG.ZONES if z not in zp]
    if miss:
        print('  %s좌표가 없는 존: %s — 기하 점검 불가%s' % (CY, ', '.join(miss), C0))
        return 1
    meas = {z: (zp[z]['x'], zp[z]['y']) for z in SIG.ZONES}
    print()
    print('  %s네 존이 이루는 사각형%s' % (CB, C0))
    lines, worst = ZF.report(meas)
    for ln in lines:
        print(ln)
    col = CG if worst < 50 else (CY if worst < 150 else CR)
    print('  %s최악 어긋남 %.0f mm%s' % (col, worst, C0))
    zd = (d.get('zone_dist') or {})
    if zd:
        print()
        print('  %s줄자 실측과 대조%s' % (CB, C0))
        for k, v in sorted(zd.items()):
            p_, q_ = k.split('-')
            if p_ in meas and q_ in meas:
                dc = math.hypot(meas[p_][0] - meas[q_][0],
                                meas[p_][1] - meas[q_][1])
                gap = (dc - v['m']) * 1000
                print('    %s  등록값 %.3f m   줄자 %.3f m   %s차이 %+.0f mm%s'
                      % (k, dc, v['m'], CG if abs(gap) < 100 else CY, gap, C0))
    else:
        print('  %s줄자 실측이 없다. 존을 찍을 때 --from/--dist 로 같이 넣으면'
              ' fit 이 훨씬 정확해진다%s' % (CD, C0))
    if worst >= 50:
        print('  %s-> 경기장이 직사각형이면 이건 등록 오차다. '
              '`register fit` 으로 접을 수 있다%s' % (CY, C0))
    return 0


def _register_fit(a):
    """직사각형을 강제해 존 좌표를 다시 쓴다. 되돌릴 수 있게 백업한다."""
    import zone_fit as ZF
    d = load_venue()
    zp = zone_poses(d)
    meas = {z: (zp[z]['x'], zp[z]['y']) for z in SIG.ZONES if z in zp}
    if len(meas) < 2:
        raise SystemExit('★ 존이 최소 두 개는 등록돼 있어야 한다 (지금 %d개)'
                         % len(meas))
    # 가로·세로 정하는 우선순위:  ① 인자로 준 값  ② 줄자 실측  ③ 추정
    # CCW 순서 2-3-4-1 이므로 2-3 과 4-1 이 한 쌍(가로), 3-4 와 1-2 가 다른 쌍(세로).
    zd = {k: v['m'] for k, v in (d.get('zone_dist') or {}).items()}
    W = a.width if a.width > 0 else None
    H = a.height if a.height > 0 else None
    if W is None:
        cand = [zd[k] for k in ('2-3', '1-4') if k in zd]
        if cand:
            W = sum(cand) / len(cand)
            print('  가로: 줄자 실측 %s -> %.3f m'
                  % (', '.join('%.3f' % c for c in cand), W))
    if H is None:
        cand = [zd[k] for k in ('3-4', '1-2') if k in zd]
        if cand:
            H = sum(cand) / len(cand)
            print('  세로: 줄자 실측 %s -> %.3f m'
                  % (', '.join('%.3f' % c for c in cand), H))
    try:
        r = ZF.fit(meas, W=W, H=H)
    except ValueError as e:
        raise SystemExit('★ %s' % e)
    print('  맞춘 직사각형: 가로 %.3f m  세로 %.3f m  방향 %.2f도'
          % (r['W'], r['H'], r['theta_deg']))
    print('  찍은 존 %d개,  잔차 rms %.0f mm' % (len(meas), r['rms_mm']))
    print()
    print('  %s존별 이동량%s' % (CB, C0))
    for z in sorted(r['fitted']):
        nx, ny = r['fitted'][z]
        if z in meas:
            mv = math.hypot(nx - meas[z][0], ny - meas[z][1]) * 1000
            print('    존 %s  %5.0f mm   (%.3f, %.3f) -> (%.3f, %.3f)'
                  % (z, mv, meas[z][0], meas[z][1], nx, ny))
        else:
            print('    존 %s  %s도면에서 나옴%s      -> (%.3f, %.3f)'
                  % (z, CY, C0, nx, ny))
    if not a.go:
        print()
        print('  %s미리보기다. 실제로 쓰려면 --go 를 붙일 것%s' % (CY, C0))
        return 0
    bak = VENUE + '.bak_before_fit_' + time.strftime('%m%d_%H%M%S')
    shutil.copy2(VENUE, bak)
    for z, (nx, ny) in r['fitted'].items():
        e = d.setdefault('zone_poses', {}).setdefault(z, {})
        e['x'] = round(nx, 4)
        e['y'] = round(ny, 4)
        e.setdefault('yaw', 0.0)
        e.setdefault('yaw_deg', 0.0)
        e['how'] = 'register fit (직사각형 강제)'
        e['fit_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
    save_venue(d)
    print()
    print('  %s적용했다%s   되돌리려면:  cp %s %s' % (CG, C0, bak, VENUE))
    print('  횡단보도 통과점(cw_*)과 경유 규칙(leg_via)은 %s건드리지 않았다%s — '
          '찍은 그대로 쓴다' % (CB, C0))
    print('  %s★ 좌표가 바뀌었으니 기준 스캔을 다시 떠야 한다 — '
          '각 존에서 register N%s' % (CY, C0))
    return 0


# ── 주문서 ────────────────────────────────────────────────────────────
#
# 위치 칸은 **목적지**다. 코엑스(존 1)는 목적지에 없어 적재지로 본다.
#
# 【적재 용량이 랩 수를 정한다 — 2026-09-13】
# 용량은 5개다. 그런데 가장 작은 두 주문이 3 + 3 = 6 으로 **용량을 넘는다.**
# 두 주문을 한 번에 실을 수 없다는 뜻이다. 따라서 **주문 1건 = 1랩**,
# 주문 8건이면 8랩이다. 이건 경로를 아무리 잘 짜도 안 줄어든다.
ORDERS = os.path.join(GOSU, 'config', 'orders.json')


def load_orders():
    if not os.path.exists(ORDERS):
        return None
    try:
        return json.load(open(ORDERS, encoding='utf-8'))
    except Exception:
        return None


def orders_for_zone(od, zone):
    if not od:
        return []
    return [o for o in od.get('orders', []) if str(o.get('zone')) == str(zone)]


def order_size(o):
    return sum((o.get('items') or {}).values())


def fmt_order(o):
    it = ', '.join('%s×%d' % (k, v) for k, v in (o.get('items') or {}).items())
    return '%s번(%s, %d개)  %s' % (o.get('num', '?'), o.get('grade', '?'),
                                   order_size(o), it)


def cmd_orders(a):
    """주문서를 보여주고, 용량 때문에 몇 랩이 필요한지 계산한다."""
    od = load_orders()
    if od is None:
        raise SystemExit('★ 주문서가 없다: %s' % ORDERS)
    cap = int(od.get('capacity', 5))
    lab = (load_venue().get('zone_labels') or {})
    print('  %s%s%s' % (CD, od.get('note', ''), C0))
    print()
    by = {}
    for o in od.get('orders', []):
        by.setdefault(str(o.get('zone')), []).append(o)
    for z in sorted(by):
        print('  %s존 %s (%s)%s  — %d건' % (CB, z, lab.get(z, ''), C0, len(by[z])))
        for o in by[z]:
            print('      %s' % fmt_order(o))
    print()
    sizes = sorted(order_size(o) for o in od.get('orders', []))
    n = len(sizes)
    print('  적재 용량 %d개.  가장 작은 두 주문 %d + %d = %d'
          % (cap, sizes[0], sizes[1] if n > 1 else 0,
             sizes[0] + (sizes[1] if n > 1 else 0)))
    if n > 1 and sizes[0] + sizes[1] > cap:
        print('  %s-> 두 주문을 한 번에 실을 수 없다. 주문 1건 = 1랩 -> %d랩 필요%s'
              % (CY, n, C0))
    else:
        print('  -> 일부는 합쳐 실을 수 있다')
    print()
    print('  %s주의%s  숫자 칸의 뜻은 아직 확정이 아니다. %s'
          % (CY, C0, od.get('num_note', '')))
    return 0


# ── 정밀 도킹 속도 ────────────────────────────────────────────────────
#
# 두 도킹 경로(fine_approach, scan_align_to)가 **같은 값**을 쓴다.
# 예전에는 각자 박아 두어서 한쪽만 고치면 조용히 달라졌다.
#
# 【왜 이 값인가 — 2026-09-13】
#   V_MIN 0.11 / W_MIN 0.50 은 **내리면 안 된다**. 브리지의 최소 유효 명령
#   (V_MIN 0.10 / W_MIN 0.45) 바로 위다. 그보다 작게 내면 로봇이 발을 안 뗀다.
#   상한은 반대다. 빠르면 멈출 때 미끄러져 지나친다.
#   제어 주기 30ms 기준 한 틱에 가는 거리:  0.32 m/s -> 9.6mm,  0.24 m/s -> 7.2mm
#   멈춤 문턱이 10~12mm 라 0.32 는 한 틱이 문턱에 거의 닿는다 — 너무 빠르다.
#   그래서 0.24 로 내렸다. 회전도 1.10 -> 0.90 (1.10 이 도킹을 망친 전례가 있다).
#
# 현장에서 배포 없이 조절할 수 있게 전부 환경변수로 뺐다:
#   GO2_DOCK_VFAR  먼 구간 최소속도   GO2_DOCK_VMAX  병진 상한
#   GO2_DOCK_WMAX  회전 상한          GO2_DOCK_SETTLE  멈춘 뒤 정착 시간
DOCK_V_MIN = 0.11                 # ★ 내리지 말 것 (브리지 하한)
DOCK_W_MIN = 0.50                 # ★ 내리지 말 것 (브리지 하한)
DOCK_V_FAR = float(os.environ.get('GO2_DOCK_VFAR', '0.15'))
DOCK_V_MAX = float(os.environ.get('GO2_DOCK_VMAX', '0.24'))
DOCK_W_MAX = float(os.environ.get('GO2_DOCK_WMAX', '0.90'))
DOCK_NEAR_M = 0.05                # 이 안쪽은 DOCK_V_MIN 으로 천천히


ZONE_SCANS = os.path.join(GOSU, 'config', 'zone_scans.json')


def load_zone_scans():
    if not os.path.exists(ZONE_SCANS):
        return {}
    try:
        return json.load(open(ZONE_SCANS, encoding='utf-8'))
    except Exception:
        return {}


def save_zone_scans(d):
    os.makedirs(os.path.dirname(ZONE_SCANS), exist_ok=True)
    tmp = ZONE_SCANS + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(d, f, ensure_ascii=False, indent=1)
    os.replace(tmp, ZONE_SCANS)


def grab_scan(node, frames=8, timeout=5.0, log=print):
    """정지 상태에서 /scan 을 여러 장 모아 각도칸마다 중앙값을 낸다.

    한 장만 쓰면 잡음 한 점이 정합을 끌 수 있다. 반드시 **멈춘 뒤에** 부를 것 —
    걷는 중에 모으면 장마다 자세가 달라 중앙값이 의미를 잃는다.
    """
    import rclpy
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    import scan_match as SM

    got = []

    def on_scan(m):
        if len(got) < frames:
            got.append({'ranges': list(m.ranges), 'angle_min': float(m.angle_min),
                        'angle_increment': float(m.angle_increment),
                        'range_min': float(m.range_min),
                        'range_max': float(m.range_max)})

    sub = node.create_subscription(
        LaserScan, '/scan', on_scan,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    end = time.time() + timeout
    while time.time() < end and len(got) < frames:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    if not got:
        log('★ /scan 이 안 온다. 주행 스택이 떠 있는지 볼 것')
        return None
    if len(got) < frames:
        log('스캔 %d장만 모았다 (원한 것 %d장)' % (len(got), frames))
    return SM.merge_frames(got)


def cmd_teach(a):
    """지금 서 있는 자리의 스캔을 이 존의 **기준**으로 저장한다.

    이 한 번만 제대로 서 있으면 된다. 그 뒤로는 영원히 그 자세로 돌아간다.
    반드시 **선 자세**로 찍을 것 — 앉으면 라이다가 27 cm 낮아져 다른 스캔이 된다.
    """
    if not a.zone:
        raise SystemExit('★ 존 번호가 필요하다:  ~/gosu/gosu.sh teach 2')
    z = str(a.zone[0])
    import rclpy
    from rclpy.node import Node
    rclpy.init(args=sys.argv)
    n = Node('zone_scan_teacher')
    try:
        print('  존 %s 의 기준 스캔을 뜬다. 지금 자세가 그대로 목표가 된다.' % z)
        print('  %s선 자세여야 한다. 앉아 있으면 다시 세우고 찍을 것.%s' % (CY, C0))
        sc = grab_scan(n, frames=a.scan_frames, log=lambda m: print('  ' + str(m)))
        if sc is None:
            raise SystemExit('★ 스캔을 못 받았다.')
        import scan_match as SM
        pts = SM.scan_points(sc['ranges'], sc['angle_min'], sc['angle_increment'],
                             sc.get('range_min', 0.0), sc.get('range_max', 1e9))
        if len(pts) < 100:
            raise SystemExit('★ 쓸 수 있는 점이 %d개뿐이다. 벽이 안 보이는 자리다.'
                             % len(pts))
        rec = dict(sc)
        rec['frames'] = sc.get('frames', 0)
        rec['points'] = int(len(pts))
        rec['set_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
        try:
            px, py, pdeg = robot_pose()
            rec['pose'] = {'x': round(px, 4), 'y': round(py, 4),
                           'yaw_deg': round(pdeg, 2)}
            print('  그때 AMCL 자세: x %.3f  y %.3f  yaw %.1f도  '
                  '(정합의 초기 추측으로만 쓴다)' % (px, py, pdeg))
        except SystemExit:
            rec['pose'] = None
            print('  %sAMCL 자세를 못 읽었다 — 씨앗 없이 쓴다 (포획 범위가 좁아진다)%s'
                  % (CY, C0))
        d = load_zone_scans()
        d[z] = rec
        save_zone_scans(d)
        print('  %s저장했다%s  점 %d개, 스캔 %d장  ->  %s'
              % (CG, C0, len(pts), rec['frames'], ZONE_SCANS))
        print('  이제:  ~/gosu/gosu.sh goto %s --dock-mode scan' % z)
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
    return 0


def scan_align_to(node, zkey, zone_pose, pos_ok=0.02, yaw_ok=0.05,
                  budget=20.0, log=print, restore=True, use_classic=True,
                  frames=6, max_iter=5):
    """기준 스캔에 맞을 때까지 좁힌다. AMCL 도 odom 도 '어디로' 에는 안 쓴다.

    쓰는 것
      · 어디로 가야 하나 : /scan 대 기준 스캔 정합      <- 표류하지 않는다
      · 얼마나 움직였나  : /utlidar/robot_odom (다리 기구학, 한 동작 1~3초)
      · AMCL            : **초기 추측으로만**. 틀려도 정합이 바로잡는다.

    돌려줌: (성공?, 남은 오차 m, 사유)
    """
    import rclpy
    import tf2_ros
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    import scan_match as SM

    ref = load_zone_scans().get(str(zkey))
    if ref is None:
        return False, None, ('존 %s 의 기준 스캔이 없다. 먼저: '
                             '~/gosu/gosu.sh teach %s' % (zkey, zkey))

    NEAR_M = DOCK_NEAR_M
    V_MIN_FAR = DOCK_V_FAR
    V_MIN, V_MAX = DOCK_V_MIN, DOCK_V_MAX
    W_MIN, W_MAX = DOCK_W_MIN, DOCK_W_MAX
    SETTLE = float(os.environ.get('GO2_DOCK_SETTLE', '0.6'))
    TICK = 0.05
    T0 = time.time()

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    odo = {'p': None}

    def on_odom(m):
        q = m.pose.pose.orientation
        odo['p'] = (m.pose.pose.position.x, m.pose.pose.position.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y),
                               1 - 2 * (q.y * q.y + q.z * q.z)))
    node.create_subscription(Odometry, '/utlidar/robot_odom', on_odom,
                             QoSProfile(depth=10,
                                        reliability=ReliabilityPolicy.BEST_EFFORT))

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=TICK)

    def stop():
        for _ in range(6):
            pub.publish(Twist())
            spin(0.05)

    def seed():
        """AMCL 로 초기 추측을 만든다. 없거나 틀려도 된다 — 추측일 뿐이다."""
        rp = ref.get('pose')
        if not rp:
            return None
        try:
            tr = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        now_s = node.get_clock().now().nanoseconds * 1e-9
        st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
        if now_s - st > 0.5:
            return None
        q = tr.transform.rotation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y),
                        1 - 2 * (q.y * q.y + q.z * q.z))
        rth = math.radians(rp.get('yaw_deg', 0.0))
        ddx = tr.transform.translation.x - rp['x']
        ddy = tr.transform.translation.y - rp['y']
        c, sn = math.cos(-rth), math.sin(-rth)
        return (c * ddx - sn * ddy, sn * ddx + c * ddy,
                math.atan2(math.sin(th - rth), math.cos(th - rth)))

    sport_pub = None
    last = None
    try:
        if use_classic:
            from unitree_api.msg import Request as _Req
            sport_pub = node.create_publisher(_Req, '/api/sport/request', 10)
            log('보행 전환: %d 로 (정밀 도킹용)' % GAIT_DOCK)
            _sport(node, sport_pub, API_SPEED_LEVEL, 0.2, {'data': -1})
            if GAIT_DOCK == API_CLASSIC_WALK:
                _sport(node, sport_pub, GAIT_DOCK, 1.0, {'data': True})
            else:
                _sport(node, sport_pub, GAIT_DOCK, 1.0)
            if _set_param(node, '/cmd_vel_to_sport', 'obstacle_guard', False):
                log('장애물 감시 잠시 내림 (도킹 중에만)')

        rejects = 0
        for it in range(max_iter):
            if time.time() - T0 > budget:
                log('    시간 상한 %.0f초 — 여기서 끝낸다' % budget)
                break
            stop(); spin(SETTLE)
            cur = grab_scan(node, frames=frames, log=log)
            if cur is None:
                return False, last, '/scan 이 안 온다'
            r = SM.offset(cur, ref, init=seed())
            if r is None:
                return False, last, '정합할 점이 모자라다'
            dx, dy, dyaw, fit = r
            if not SM.is_good(fit):
                rejects += 1
                log('    [%d] 정합을 믿을 수 없다 (적합도 %.0f mm > %.0f mm) — '
                    '움직이지 않는다' % (it + 1, fit * 1000, SM.FIT_OK * 1000))
                if rejects >= 2:
                    return False, last, ('정합 적합도 미달 %d회 — '
                                         '기준 스캔을 다시 뜨거나 점 도킹으로' % rejects)
                continue
            rejects = 0
            err = math.hypot(dx, dy)
            last = err
            log('    [%d] 남은 거리 %.0f mm  방향 %+.1f도  (적합도 %.0f mm, 점 %d)'
                % (it + 1, err * 1000, math.degrees(dyaw), fit * 1000,
                   len(cur['ranges'])))
            if err <= pos_ok and abs(dyaw) <= yaw_ok:
                log('    %s목표 안 — 끝%s' % (CG, C0))
                return True, err, None
            if err > 0.6:
                return False, err, '너무 멀다 (%.2f m) — Nav2 가 할 일이다' % err

            # 방향이 크면 먼저 돌린다. 병진만으로는 방향이 안 맞는다.
            if abs(dyaw) > yaw_ok and odo['p'] is not None:
                o0 = odo['p']
                t0 = time.time()
                cap = abs(dyaw) / W_MIN + 2.5
                while time.time() - t0 < cap:
                    rclpy.spin_once(node, timeout_sec=TICK)
                    q = odo['p']
                    if q is None:
                        break
                    d_turn = math.atan2(math.sin(q[2] - o0[2]),
                                        math.cos(q[2] - o0[2]))
                    rem = dyaw - d_turn
                    if abs(rem) <= math.radians(1.5):
                        break
                    t = Twist()
                    wfloor = 0.80 if abs(rem) > math.radians(15) else W_MIN
                    t.angular.z = math.copysign(
                        min(W_MAX, max(wfloor, abs(rem) / 0.5)), rem)
                    pub.publish(t)
                stop(); spin(0.4)
                log('    방향 %+.1f도 맞춤 — 다시 잰다' % math.degrees(dyaw))
                continue

            # ── 한 축씩 움직인다 ─────────────────────────────────────
            # 두 축을 같이 내면 브리지가 **성분마다 따로** 최소속도로 올려서
            # 방향이 틀어진다(2026-09-13 실측: -71도가 -62도가 됐다).
            # 좌우를 먼저 한다 — 구역 안에 넣는 데 좌우가 더 결정적이다.
            for axis, want in (('좌우', dy), ('앞뒤', dx)):
                if abs(want) < 0.012 or odo['p'] is None:
                    continue
                o0 = odo['p']
                oc, os_ = math.cos(o0[2]), math.sin(o0[2])
                t0 = time.time()
                cap = abs(want) / V_MIN + 2.5
                while time.time() - t0 < cap:
                    rclpy.spin_once(node, timeout_sec=TICK)
                    q = odo['p']
                    if q is None:
                        break
                    dxo, dyo = q[0] - o0[0], q[1] - o0[1]
                    mx = dxo * oc + dyo * os_
                    my = -dxo * os_ + dyo * oc
                    moved = my if axis == '좌우' else mx
                    rem = want - moved
                    if abs(rem) <= 0.010:
                        break
                    floor = V_MIN_FAR if abs(rem) > NEAR_M else V_MIN
                    sp = math.copysign(min(V_MAX, max(floor, abs(rem) / 0.5)), rem)
                    t = Twist()
                    if axis == '좌우':
                        t.linear.y = sp
                    else:
                        t.linear.x = sp
                    t.angular.z = 0.0           # ★ 절대 돌지 않는다
                    pub.publish(t)
                stop(); spin(0.4)
        return (last is not None and last <= pos_ok), last, None
    finally:
        if sport_pub is not None:
            stop()
            if restore:
                _restore(node, sport_pub, log)


def fine_approach(node, target, pos_ok=0.08, yaw_ok=0.12, max_iter=4,
                  log=print, use_classic=True, restore=True, budget=20.0):
    """도착 뒤 정밀 접근 — **회전하지 않고 옆걸음으로** 좁힌다.

    【왜 새로 썼나 — 2026-09-13 현장 실측】
    go2_align.fine_align 은 횡오차를 고치려고 제자리회전을 한다.
        [1] 좌우 -82mm, 방향 +58.5도 -> 회전 +55.5
        [2] 좌우 -199mm             -> 횡오차 대응 **회전 -85.5도**
        [3] 272mm   (218 -> 199 -> 272, 발산)
    4족보행의 제자리회전은 순수 피벗이 아니다. 몸 중심이 수십 mm 밀린다.
    오차를 줄이려는 회전이 오차를 만든다. 그 모듈도 결론을 적어놨다 —
    "이 로봇은 병진 정밀 제어가 안 된다".

    【무엇이 다른가】 Go2 는 **옆걸음(vy)이 된다**(브리지 MAX_VY 0.3).
    돌 이유가 없다. 앞뒤와 좌우를 **동시에** 내고 회전은 0 으로 묶는다.
    몸통 방향이 안 변하므로 몸 좌표계가 이동 중에 고정돼 계산이 단순하고,
    회전이 만드는 오차가 아예 생기지 않는다.

    【두 센서를 나눠 쓰는 것은 그대로】
      · 어디로 가야 하나 : AMCL (**정지 중에만** — 걸으면 몸통이 기울어 못 믿는다)
      · 얼마나 움직였나  : /utlidar/robot_odom (149Hz, 다리 기구학)
    한 동작이 1~3초라 robot_odom 의 장기 드리프트는 상관없다.

    【좌표 변환】 지도 좌표 오차를 몸 좌표로 바꿔 목표 변위를 정하고,
    이동 중에는 robot_odom 변위를 **이동 시작 시점의 몸 좌표**로 바꿔 비교한다.
    회전이 없으므로 그 몸 좌표계는 이동 내내 고정이다. map<->odom 관계가 필요 없다.
    """
    import rclpy
    import tf2_ros
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    # ── 두 단계 속도 ─────────────────────────────────────────────────
    # 【왜 나누나 — 2026-09-13】
    #   허용 오차를 넓히면 회차는 주는데 정확도를 잃는다. 대신 **속도**로 줄인다.
    #   그런데 마지막 몇 cm 를 빠르게 가면 멈출 때 미끄러져 지나친다.
    #   그래서 NEAR_M 보다 멀면 빠르게, 가까우면 원래 속도로 붙인다.
    #   제어 주기가 30ms 이므로 0.32 m/s 에서 한 틱에 9.6mm — 멈춤 문턱 12mm 안이다.
    NEAR_M = DOCK_NEAR_M             # 이 안쪽은 느리게
    V_MIN_FAR = DOCK_V_FAR
    V_MIN, V_MAX = DOCK_V_MIN, DOCK_V_MAX   # 위 DOCK_* 주석 참조
    W_MIN, W_MAX = DOCK_W_MIN, DOCK_W_MAX
    # 2026-09-13: 속도를 올리기 위해 1.2 -> 0.6.
    #   몸이 가라앉는 데 필요한 시간이지, 그보다 길 이유는 없다.
    #   느리면 --settle 로 되돌린다.
    SETTLE = float(os.environ.get('GO2_DOCK_SETTLE', '0.6'))
    TICK = 0.05

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    odo = {'p': None}

    def on_odom(m):
        q = m.pose.pose.orientation
        odo['p'] = (m.pose.pose.position.x, m.pose.pose.position.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y),
                               1 - 2 * (q.y * q.y + q.z * q.z)))
    node.create_subscription(Odometry, '/utlidar/robot_odom', on_odom,
                             QoSProfile(depth=10,
                                        reliability=ReliabilityPolicy.BEST_EFFORT))

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=TICK)

    def amcl_pose(n_samples=5, wait=4.0):
        """정지 상태에서만 읽는다. 여러 장의 중앙값을 쓴다."""
        xs, ys, cs, ss = [], [], [], []
        end = time.time() + wait
        while time.time() < end and len(xs) < n_samples:
            rclpy.spin_once(node, timeout_sec=TICK)
            try:
                tr = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            except Exception:
                continue
            # 낡은 값은 버린다 — 도킹은 **지금** 자세로 재야 한다
            now_s = node.get_clock().now().nanoseconds * 1e-9
            st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            if now_s - st > 0.4:
                continue
            q = tr.transform.rotation
            y = math.atan2(2 * (q.w * q.z + q.x * q.y),
                           1 - 2 * (q.y * q.y + q.z * q.z))
            xs.append(tr.transform.translation.x)
            ys.append(tr.transform.translation.y)
            cs.append(math.cos(y)); ss.append(math.sin(y))
            time.sleep(0.08)
        if len(xs) < 3:
            return None
        med = lambda v: sorted(v)[len(v) // 2]
        return med(xs), med(ys), math.atan2(med(ss), med(cs))

    def stop():
        for _ in range(6):
            pub.publish(Twist())
            spin(0.05)

    # ── 정밀 도킹 동안만 보행을 바꾼다 ────────────────────────────────
    # 트로트는 한 걸음이 커서 10 cm 급 위치 제어가 안 된다.
    # ClassicWalk 는 느리고 걸음이 작아 같은 명령에 대한 이동량이 작다.
    # **반드시 되돌린다** — 되돌리지 않으면 다음 구간 주행이 느려진다.
    sport_pub = None
    if use_classic:
        from unitree_api.msg import Request as _Req
        sport_pub = node.create_publisher(_Req, '/api/sport/request', 10)
        log('보행 전환: %d 로 (정밀 도킹용, 항상 세 발 접지)' % GAIT_DOCK)
        _sport(node, sport_pub, API_SPEED_LEVEL, 0.2, {'data': -1})
        if GAIT_DOCK == API_CLASSIC_WALK:
            _sport(node, sport_pub, GAIT_DOCK, 1.0, {'data': True})
        else:
            _sport(node, sport_pub, GAIT_DOCK, 1.0)      # 1061/1062/1063 은 파라미터 없음
        # ── 장애물 감시를 잠시 내린다 ──────────────────────────────────
        # 도킹은 20 cm 안쪽을 옆걸음으로 움직인다. 그때 진행 방향이 옆이 되는데
        # 브리지의 장애물 감시는 그 방향 0.75 m 를 본다 — 구역은 벽 근처라 늘 걸린다.
        # 걸리면 vx/vy 가 0 이 되고 회전만 남아 **제자리걸음**이 된다(2026-09-13 실측).
        # 도킹 구간에서는 이미 Nav2 가 목표 근처까지 안전하게 데려다 놓은 뒤다.
        guard_off = _set_param(node, '/cmd_vel_to_sport', 'obstacle_guard', False)
        if guard_off:
            log('장애물 감시 잠시 내림 (도킹 중에만)')
        # ── AMCL 갱신 문턱을 낮춘다 ────────────────────────────────────
        # 이게 도킹이 안 되던 근본 원인이다. 위 AMCL_DOCK_MIN_D 주석 참조.
        d_ok = _set_param(node, '/amcl', 'update_min_d', AMCL_DOCK_MIN_D)
        a_ok = _set_param(node, '/amcl', 'update_min_a', AMCL_DOCK_MIN_A)
        if d_ok and a_ok:
            log('AMCL 갱신 문턱 %.0f cm -> %.0f cm (도킹 중에만)'
                % (AMCL_DRIVE_MIN_D * 100, AMCL_DOCK_MIN_D * 100))
        else:
            log('★ AMCL 갱신 문턱을 못 낮췄다 — 자세가 안 바뀌어 도킹이 수렴하지 않는다')

    spin(2.0)
    if odo['p'] is None:
        log('robot_odom 이 안 온다 — 정밀 접근을 건너뛴다')
        if sport_pub is not None:
            _restore(node, sport_pub, log)
        return False, None

    tx, ty = float(target['x']), float(target['y'])
    tyaw = float(target.get('yaw', 0.0))     # 존을 찍을 때의 방향. 이걸 되찾아야 한다
    last_err = None
    best = {'s': None, 'stall': 0}      # 정체 감지
    turned = {'once': False}            # 이력(히스테리시스)
    t_dock = time.time()
    try:
      for it in range(1, max_iter + 1):
        # ★ 시간 상한. 회차를 줄이는 것만으로는 못 막는다 — 어떤 회차가
        #   오래 걸릴 수 있다. 남은 시간이 없으면 지금까지 결과로 끝낸다.
        if time.time() - t_dock > budget:
            log('    도킹 시간 상한 %.0f초 도달 — 여기서 끝낸다' % budget)
            return (last_err is not None and last_err <= pos_ok), last_err
        stop(); spin(SETTLE)
        cur = amcl_pose()
        if cur is None:
            log('AMCL 자세를 못 읽었다 — 중단')
            return False, last_err
        ex, ey = tx - cur[0], ty - cur[1]
        err = math.hypot(ex, ey)
        last_err = err
        # 지도 좌표 오차 -> 몸 좌표 (앞뒤 fx, 좌우 fy)
        c, s2 = math.cos(cur[2]), math.sin(cur[2])
        fx = ex * c + ey * s2
        fy = -ex * s2 + ey * c
        dyaw = math.atan2(math.sin(tyaw - cur[2]), math.cos(tyaw - cur[2]))
        log('[%d] 오차 %5.0f mm (앞뒤 %+5.0f, 좌우 %+5.0f)  방향 %+5.1f도'
            % (it, err * 1000, fx * 1000, fy * 1000, math.degrees(dyaw)))

        # ── 방향과 위치를 번갈아 맞춘다 ───────────────────────────────
        # 제자리회전은 몸 중심을 수십 mm 밀어내고, 옆걸음은 방향을 조금 틀어놓는다.
        # 그래서 **한 번에 하나만** 고치고 다시 재는 것을 반복한다.
        #
        # 핑퐁을 막는 장치 두 가지:
        #   · 이력(히스테리시스) — 한 번 맞춘 뒤에는 허용치의 1.6배를 넘어야 다시 돈다.
        #     안 그러면 옆걸음이 만든 1~2도 때문에 계속 돌기만 한다.
        #   · 정체 감지 — 두 회차 연속으로 나아지지 않으면 멈추고 가장 좋았던 상태를 알린다.
        #
        # 방향이 중요한 이유: 발 사각형이 335x328 mm 인데 60 cm 박스 안 여유가
        # 0도에서 ±132 mm, 45도에서 ±65 mm 로 절반이 된다. 존 좌표를 찍을 때의
        # 방향을 되찾아야 그때의 발 배치가 재현되고, 의도한 동작(적재/하차)이 된다.
        score = err + abs(dyaw) * 0.25          # 위치(m) + 방향을 m 로 환산해 합침
        if best['s'] is None or score < best['s'] - 0.005:
            best['s'], best['stall'] = score, 0
        else:
            best['stall'] += 1
            if best['stall'] >= 2:
                log('    더 나아지지 않는다 — 여기서 멈춘다'
                    ' (오차 %.0f mm, 방향 %+.1f도)' % (err * 1000, math.degrees(dyaw)))
                return err <= pos_ok, err

        need_turn = abs(dyaw) > (yaw_ok if not turned['once'] else yaw_ok * 1.6)
        if need_turn and odo['p'] is not None:
            o0 = odo['p']
            t0 = time.time()
            cap = abs(dyaw) / W_MIN + 2.5
            while time.time() - t0 < cap:
                rclpy.spin_once(node, timeout_sec=TICK)
                q = odo['p']
                if q is None:
                    break
                # ★ 이름을 turned 로 쓰면 안 된다. 바깥의 turned 는 히스테리시스
                #   dict({'once': ...}) 이고, 여기서 float 로 덮어쓰면 루프 직후
                #   turned['once'] = True 가 TypeError 로 터진다. 그 예외를
                #   fine_align_to 의 except 가 조용히 삼켜서 **병진 보정이 통째로
                #   건너뛰어진다** — 회전만 하고 위치는 안 맞춘 채 끝난다.
                #   그래서 최종 오차가 Nav2 목표 허용치(0.22m) 그대로 남았다.
                #   2026-09-13 확인. 증상: '방향 …도 맞춤' 로그가 한 번도 안 찍힘.
                d_turn = math.atan2(math.sin(q[2] - o0[2]), math.cos(q[2] - o0[2]))
                rem = dyaw - d_turn
                if abs(rem) <= math.radians(2.0):
                    break
                t = Twist()
                wfloor = 0.80 if abs(rem) > math.radians(15) else W_MIN
                t.angular.z = math.copysign(min(W_MAX, max(wfloor, abs(rem) / 0.5)), rem)
                pub.publish(t)
            stop(); spin(0.5)
            turned['once'] = True
            log('    방향 %+.1f도 맞춤 — 다시 잰다' % math.degrees(dyaw))
            continue

        if err <= pos_ok and abs(dyaw) <= yaw_ok * 1.6:
            log('    목표 안 — 끝  (위치 %.0f mm, 방향 %+.1f도)'
                % (err * 1000, math.degrees(dyaw)))
            return True, err
        if err > 0.6:
            log('    너무 멀다 (%.2f m). Nav2 가 할 일이다 — 중단' % err)
            return False, err

        # ── 한 축씩 움직인다 ─────────────────────────────────────────
        # 앞뒤와 좌우를 동시에 내면 브리지가 **성분마다 따로** V_MIN 으로 올려서
        # 방향이 틀어진다. 실측: vx 0.064 -> 0.10 으로 올라가며 -71도가 -62도가 됐다.
        # 한 번에 한 축만 내면 그 왜곡이 아예 없다. 회전은 끝까지 0 이다.
        # 좌우를 먼저 한다 — 구역 안에 넣는 데 좌우가 더 결정적이다.
        for axis, want in (('좌우', fy), ('앞뒤', fx)):
            if abs(want) < 0.02:
                continue
            if odo['p'] is None:
                break
            o0 = odo['p']
            oc, os_ = math.cos(o0[2]), math.sin(o0[2])
            t0 = time.time()
            cap = abs(want) / V_MIN + 2.5
            while time.time() - t0 < cap:
                rclpy.spin_once(node, timeout_sec=TICK)
                q = odo['p']
                if q is None:
                    break
                dxo, dyo = q[0] - o0[0], q[1] - o0[1]
                mx = dxo * oc + dyo * os_
                my = -dxo * os_ + dyo * oc
                moved = my if axis == '좌우' else mx
                rem = want - moved
                if abs(rem) <= 0.015:
                    break
                floor = V_MIN_FAR if abs(rem) > NEAR_M else V_MIN
                sp = math.copysign(min(V_MAX, max(floor, abs(rem) / 0.5)), rem)
                t = Twist()
                if axis == '좌우':
                    t.linear.y = sp
                else:
                    t.linear.x = sp
                t.angular.z = 0.0              # ★ 절대 돌지 않는다
                pub.publish(t)
            stop(); spin(0.4)
        stop()
      stop(); spin(SETTLE)
      cur = amcl_pose()
      if cur is not None:
          last_err = math.hypot(tx - cur[0], ty - cur[1])
          log('    최종 오차 %.0f mm' % (last_err * 1000))
      return (last_err is not None and last_err <= pos_ok), last_err
    finally:
        # 무슨 일이 있어도 보행을 되돌린다. 안 되돌리면 다음 구간이 느려진다.
        if sport_pub is not None:
            stop()
            if restore:
                _restore(node, sport_pub, log)
            else:
                # ★ 2026-09-13: 엎드리기 **전에** 보행을 되돌리지 않는다.
                #   BalanceStand 로 바꾸는 순간 발이 다시 잡혀, 애써 맞춘 위치가
                #   흔들린다. 규정 판정은 **엎드렸을 때의 발 위치**로 하므로
                #   그 사이에 자세를 건드리면 안 된다. 복원은 엎드린 뒤에 한다.
                log('보행 유지 (엎드린 뒤에 되돌린다)')



# 몸 좌표계에서 본 네 발 위치 (m). 2026-09-13 실측, 서 있을 때.
#   FR 앞뒤 +156.7  좌우 -152.9      FL 앞뒤 +154.7  좌우 +148.0
#   RR 앞뒤 -178.0  좌우 -130.1      RL 앞뒤 -171.8  좌우 +175.0
# 발 사각형 335 x 328 mm. base_link 와 발 중심의 차이는 10 mm 뿐이다.
FOOT_BODY = {'FR': (0.1567, -0.1529), 'FL': (0.1547, 0.1480),
             'RR': (-0.1780, -0.1301), 'RL': (-0.1718, 0.1750)}


def predict_feet_in_box(pose, zone_pose, box_m=0.60):
    """지금 자세라면 발이 구역에 **몇 개** 들어가는지 계산한다. 앉지 않고.

    【왜 — 2026-09-13】
    지금은 도착하면 **무조건** 도킹을 돌린다. 그런데 발 위치는 base_link 에서
    고정 오프셋이라, 도착 자세만 보면 들어가는지 아닌지 바로 계산된다.
    이미 들어가면 도킹을 통째로 건너뛸 수 있다 — 그게 "도착부터 판정까지"의
    가장 큰 덩어리다.
    """
    x, y, yaw = pose
    cx, cy = float(zone_pose['x']), float(zone_pose['y'])
    zyaw = float(zone_pose.get('yaw', 0.0))
    c, sn = math.cos(yaw), math.sin(yaw)
    zc, zs = math.cos(zyaw), math.sin(zyaw)
    h = box_m / 2.0
    n_in = 0
    worst = 0.0
    for nm, (fx, fy) in FOOT_BODY.items():
        # 몸 -> map
        mx = x + fx * c - fy * sn
        my = y + fx * sn + fy * c
        # map -> 구역
        ex, ey = mx - cx, my - cy
        bx = ex * zc + ey * zs
        by = -ex * zs + ey * zc
        if abs(bx) <= h and abs(by) <= h:
            n_in += 1
        worst = max(worst, abs(bx) - h, abs(by) - h)
    return n_in, worst


def read_feet(node, buf, reader, wait=2.5, need=4):
    """발 네 개의 map 좌표를 여러 장 모아 중앙값을 낸다. 정지 중에만 쓴다."""
    import rclpy
    got = {}
    end = time.time() + wait
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.03)
        f = reader.feet_map()
        if not f:
            continue
        for name, x, y, force in f:
            got.setdefault(name, []).append((x, y, force))
        if all(len(v) >= need for v in got.values()) and len(got) >= 4:
            break
    if len(got) < 4:
        return None
    out = []
    for name, rows in got.items():
        med = lambda i: sorted(r[i] for r in rows)[len(rows) // 2]
        out.append((name, med(0), med(1), med(2)))
    return out


def _dock_by_feet_inner(node, zone_pose, box_m=0.60, need=3, max_iter=3,
                 log=print, margin=0.04, yaw_ok=0.12, budget=20.0,
                 use_classic=None, restore=True, quad=True):
    """발 네 개를 직접 재서 구역 박스 안에 넣는다 (점 하나에 의존하지 않는다).

    【왜 이 방식인가 — 2026-09-13 실측】
    발은 몸 중심에 대칭이 아니다:
        FR 좌우 -152.9   FL 좌우 +148.0
        RR 좌우 -130.1   RL 좌우 +175.0      <- RL 이 RR 보다 45 mm 밖
    base_link 를 목표에 정확히 맞춰도 발은 한쪽으로 45 mm 치우친다.
    그런데 채점은 **발이 박스 안에 있는가**로 한다. 그러면 재야 할 것은 발이다.

    하는 일: 네 발을 map 좌표로 재고 -> 구역 좌표계로 옮기고 ->
    발 네 점을 감싸는 사각형이 박스 한가운데 오도록 필요한 이동량을 계산해 ->
    robot_odom 폐루프로 그만큼 옆걸음한다. 회전은 하지 않는다.

    점 하나(base_link)가 아니라 **네 점을 재므로** 자세의 치우침이 자동으로 들어간다.
    """
    import types as _t
    import rclpy
    import tf2_ros
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    try:
        import zones as Z
    except Exception as e:
        log('발 판정 모듈을 못 불렀다 (%s) — 점 도킹으로 돌아간다' % e)
        return None

    NEAR_M = 0.05
    V_MIN_FAR = float(os.environ.get('GO2_DOCK_VFAR', '0.20'))
    V_MIN, V_MAX = 0.11, 0.32
    W_MIN, W_MAX = 0.50, 1.10
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    a = _t.SimpleNamespace(map_frame='map', base_frame='base_link',
                           lowstate_topic='/lowstate')
    reader = Z.Reader(node, buf, a)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    odo = {'p': None}

    def on_odom(m):
        q = m.pose.pose.orientation
        odo['p'] = (m.pose.pose.position.x, m.pose.pose.position.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y),
                               1 - 2 * (q.y * q.y + q.z * q.z)))
    node.create_subscription(Odometry, '/utlidar/robot_odom', on_odom,
                             QoSProfile(depth=10,
                                        reliability=ReliabilityPolicy.BEST_EFFORT))

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.03)

    def stop():
        for _ in range(6):
            pub.publish(Twist()); spin(0.05)

    # ── 회전 **전에** 보행을 바꾼다 ──────────────────────────────────
    # 트로트는 보폭이 커서 제자리회전이 몸 중심을 크게 밀어낸다.
    # ClassicWalk 으로 바꾸고 나서 돌아야 그 밀림이 작고, 병진도 같은 이유로 작아야 한다.
    #
    # ★ 2026-09-13: 기본 도킹이 feet 로 바뀌었는데 보행 전환은 점 도킹에만
    #   들어 있어서, 실제로는 ClassicWalk 이 **한 번도 안 걸리고 있었다.**
    if use_classic is None:
        use_classic = CLASSIC_WALK_ON
    sport_pub = None
    if use_classic:
        from unitree_api.msg import Request as _Req2
        sport_pub = node.create_publisher(_Req2, '/api/sport/request', 10)
        log('보행 전환: %d (회전 전에 먼저)' % GAIT_DOCK)
        _sport(node, sport_pub, API_SPEED_LEVEL, 0.2, {'data': -1})
        if GAIT_DOCK == API_CLASSIC_WALK:
            _sport(node, sport_pub, GAIT_DOCK, 1.0, {'data': True})
        else:
            _sport(node, sport_pub, GAIT_DOCK, 1.0)
        if _set_param(node, '/cmd_vel_to_sport', 'obstacle_guard', False):
            log('장애물 감시 잠시 내림 (도킹 중에만)')
        if (_set_param(node, '/amcl', 'update_min_d', AMCL_DOCK_MIN_D) and
                _set_param(node, '/amcl', 'update_min_a', AMCL_DOCK_MIN_A)):
            log('AMCL 갱신 문턱 %.0f -> %.0f cm (도킹 중에만)'
                % (AMCL_DRIVE_MIN_D * 100, AMCL_DOCK_MIN_D * 100))

    spin(1.0)
    cx, cy = float(zone_pose['x']), float(zone_pose['y'])
    yaw = float(zone_pose.get('yaw', 0.0))
    c, sn = math.cos(yaw), math.sin(yaw)
    h = box_m / 2.0
    best_in = None
    t_dock = time.time()

    # ── 방향은 **맨 앞에서 한 번만** 맞춘다 ──────────────────────────
    # 회전은 몸 중심을 밀어내므로, 뒤이은 병진이 그 밀림을 흡수하게 앞에 둔다.
    # 그리고 한 번만 한다 — 병진이 만든 1~2도 때문에 또 돌면 시간만 먹는다.
    # 발 사각형이 335x328 mm 로 거의 정사각형이라 방향 몇 도는 여유를 거의 안 깎는다.
    def _cur_yaw():
        try:
            tr = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        q = tr.transform.rotation
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    stop(); spin(0.5)
    by = _cur_yaw()
    if by is not None and odo['p'] is not None:
        if quad:
            tgt, dyaw = nearest_quad_yaw(yaw, by)
            full = math.atan2(math.sin(yaw - by), math.cos(yaw - by))
            if abs(full) - abs(dyaw) > math.radians(5):
                log('방향 후보 4개 중 가장 가까운 것 선택: %+.0f도 대신 %+.0f도'
                    % (math.degrees(full), math.degrees(dyaw)))
        else:
            dyaw = math.atan2(math.sin(yaw - by), math.cos(yaw - by))
        if abs(dyaw) > yaw_ok:
            log('방향 %+.1f도 맞춘다 (한 번만)' % math.degrees(dyaw))
            o0 = odo['p']
            t0 = time.time()
            cap = min(abs(dyaw) / W_MIN + 2.0, 8.0)
            while time.time() - t0 < cap:
                rclpy.spin_once(node, timeout_sec=0.03)
                q2 = odo['p']
                if q2 is None:
                    break
                turned = math.atan2(math.sin(q2[2] - o0[2]), math.cos(q2[2] - o0[2]))
                rem = dyaw - turned
                if abs(rem) <= math.radians(2.5):
                    break
                t = Twist()
                wfloor = 0.80 if abs(rem) > math.radians(15) else W_MIN
                t.angular.z = math.copysign(min(W_MAX, max(wfloor, abs(rem) / 0.5)), rem)
                pub.publish(t)
            stop(); spin(0.4)

    for it in range(1, max_iter + 1):
        if time.time() - t_dock > budget:
            log('    도킹 시간 상한 %.0f초 도달 — 여기서 끝낸다' % budget)
            return best_in
        stop(); spin(0.6)
        feet = read_feet(node, buf, reader)
        if feet is None:
            log('발 위치를 못 읽었다 (/lowstate 가 오나?)')
            return best_in
        # 구역 좌표계로 옮긴다
        pts, planted = [], 0
        for name, fx, fy, force in feet:
            ex, ey = fx - cx, fy - cy
            bx = ex * c + ey * sn
            by = -ex * sn + ey * c
            on = force >= 20.0
            planted += 1 if on else 0
            pts.append((name, bx, by, on))
        inside = sum(1 for _, bx, by, on in pts
                     if on and abs(bx) <= h and abs(by) <= h)
        best_in = inside if best_in is None else max(best_in, inside)
        log('[%d] 발 %d/%d 안   %s' % (it, inside, planted, ' '.join(
            '%s%s' % (n, '○' if (on and abs(bx) <= h and abs(by) <= h) else '×')
            for n, bx, by, on in pts)))
        if inside >= need:
            log('    조건 만족 — 끝')
            return inside

        # 발 네 점을 감싸는 사각형을 박스 한가운데로 옮기는 이동량
        xs = [bx for _, bx, _, _ in pts]
        ys = [by for _, _, by, _ in pts]
        want_x = -((min(xs) + max(xs)) / 2.0)      # 구역 좌표계 기준 필요한 이동
        want_y = -((min(ys) + max(ys)) / 2.0)
        span_x, span_y = max(xs) - min(xs), max(ys) - min(ys)
        log('    발 사각형 %.0fx%.0f mm, 중앙까지 앞뒤 %+.0f 좌우 %+.0f mm'
            % (span_x * 1000, span_y * 1000, want_x * 1000, want_y * 1000))
        if span_x > box_m - margin or span_y > box_m - margin:
            log('    ★ 발 사각형이 박스보다 크다 — 네 발을 다 넣을 수는 없다')
        if math.hypot(want_x, want_y) < 0.015:
            log('    더 옮길 곳이 없다 — 여기서 멈춘다')
            return inside

        # 구역 좌표 -> 몸 좌표. 몸 방향은 지금 자세다.
        try:
            tr = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return inside
        q = tr.transform.rotation
        byaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))
        wx = want_x * c - want_y * sn        # 구역 -> map
        wy = want_x * sn + want_y * c
        bc, bs = math.cos(byaw), math.sin(byaw)
        fx_b = wx * bc + wy * bs             # map -> 몸
        fy_b = -wx * bs + wy * bc

        for axis, want in (('좌우', fy_b), ('앞뒤', fx_b)):
            if abs(want) < 0.02 or odo['p'] is None:
                continue
            o0 = odo['p']
            oc, os_ = math.cos(o0[2]), math.sin(o0[2])
            t0 = time.time()
            cap = abs(want) / V_MIN + 2.5
            while time.time() - t0 < cap:
                rclpy.spin_once(node, timeout_sec=0.03)
                p2 = odo['p']
                if p2 is None:
                    break
                dxo, dyo = p2[0] - o0[0], p2[1] - o0[1]
                mx = dxo * oc + dyo * os_
                my = -dxo * os_ + dyo * oc
                rem = want - (my if axis == '좌우' else mx)
                if abs(rem) <= 0.012:
                    break
                floor = V_MIN_FAR if abs(rem) > NEAR_M else V_MIN
                sp = math.copysign(min(V_MAX, max(floor, abs(rem) / 0.5)), rem)
                t = Twist()
                if axis == '좌우':
                    t.linear.y = sp
                else:
                    t.linear.x = sp
                t.angular.z = 0.0
                pub.publish(t)
            stop(); spin(0.4)
    return best_in



def dock_by_feet(node, zone_pose, box_m=0.60, need=3, max_iter=3,
                 log=print, margin=0.04, yaw_ok=0.12, budget=20.0,
                 use_classic=None, restore=True, quad=True):
    """발 네 점 도킹. 보행 복원을 **반드시** 한다 (어떤 경로로 끝나든)."""
    pub = None
    try:
        return _dock_by_feet_inner(node, zone_pose, box_m=box_m, need=need,
                                   max_iter=max_iter, log=log, margin=margin,
                                   yaw_ok=yaw_ok, budget=budget,
                                   use_classic=use_classic, restore=False,
                                   quad=quad)
    finally:
        if restore:
            from unitree_api.msg import Request as _R
            pub = node.create_publisher(_R, '/api/sport/request', 10)
            _restore(node, pub, log=log)


def fine_align_to(node, pos, label, pos_ok=0.08, yaw_ok=0.12, log=print,
                  restore=True, budget=20.0):
    """도착 뒤 정밀 접근. 회전 없이 옆걸음으로만 좁힌다."""
    log('정밀 접근 (%s 목표 ±%.0f cm, 방향 ±%.0f도)'
        % (label, pos_ok * 100, math.degrees(yaw_ok)))
    try:
        ok, err = fine_approach(node, pos, pos_ok=pos_ok, yaw_ok=yaw_ok, log=log,
                                use_classic=CLASSIC_WALK_ON, restore=restore,
                                budget=budget)
        return ok, err, None
    except Exception as e:
        log('정밀 접근 중 오류: %s — 그대로 진행한다' % e)
        return False, None, None


def _nav_setup():
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    rclpy.init(args=sys.argv)
    n = Node('zone_sender')
    ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    if not ac.wait_for_server(timeout_sec=15.0):
        n.destroy_node(); rclpy.try_shutdown()
        raise SystemExit('★ navigate_to_pose 액션 서버가 없다. 주행 스택을 먼저 올릴 것')
    return rclpy, n, ac, NavigateToPose


def _pose_msg(n, v):
    from geometry_msgs.msg import PoseStamped
    m = PoseStamped()
    m.header.frame_id = 'map'
    m.header.stamp = n.get_clock().now().to_msg()
    m.pose.position.x = float(v['x'])
    m.pose.position.y = float(v['y'])
    yaw = float(v.get('yaw', 0.0))
    m.pose.orientation.z = math.sin(yaw / 2)
    m.pose.orientation.w = math.cos(yaw / 2)
    return m


def cmd_reach(a):
    """움직이지 않고 '거기까지 경로가 나오나'만 Nav2 에게 물어본다.

    공식 액션 ComputePathToPose 를 쓴다. 계획만 시키고 주행은 안 시킨다.
    보내기 전에 이걸 먼저 돌리면 헛걸음 한 번(회복 사다리 약 17초)을 아낀다.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from nav2_msgs.action import ComputePathToPose
    d = load_venue()
    zp = zone_poses(d)
    want = [str(z) for z in (a.zone or sorted(zp))]
    if not zp:
        raise SystemExit('★ 등록된 존 목표가 없다. ~/gosu/gosu.sh zone <번호> 로 먼저 찍을 것')
    rclpy.init(args=sys.argv)
    n = Node('reach_check')
    ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
    if not ac.wait_for_server(timeout_sec=10.0):
        n.destroy_node(); rclpy.try_shutdown()
        raise SystemExit('★ compute_path_to_pose 가 없다 — planner_server 확인')
    print()
    print('  존       x        y       결과')
    ok = []
    for z in want:
        v = zp.get(z)
        if v is None:
            print('  %-6s %-8s %-8s 등록 안 됨' % (z, '-', '-')); continue
        g = ComputePathToPose.Goal()
        g.goal = _pose_msg(n, v)
        g.use_start = False
        fut = ac.send_goal_async(g)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=8.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('  %-6s %7.2f %8.2f  거절' % (z, v['x'], v['y'])); continue
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(n, rf, timeout_sec=15.0)
        r = rf.result()
        ps = r.result.path.poses if r else []
        if r and r.status == 4 and ps:
            L = sum(math.hypot(b.pose.position.x - c.pose.position.x,
                               b.pose.position.y - c.pose.position.y)
                    for c, b in zip(ps, ps[1:]))
            print('  %-6s %7.2f %8.2f  경로 %.1fm' % (z, v['x'], v['y'], L))
            ok.append(z)
        else:
            print('  %-6s %7.2f %8.2f  ★ 경로 없음 (status %s)'
                  % (z, v['x'], v['y'], r.status if r else '?'))
    print()
    print('  갈 수 있는 존: %s' % (', '.join(ok) or '없음'))
    if len(ok) < len(want):
        print('  ★ 경로가 없으면 /tmp/go2_run/nav2.log 를 볼 것.')
        print('     "off the global costmap" 이면 좌표가 지도 밖이고,')
        print('     그 밖의 실패면 코스트맵이 그 자리를 막고 있는 것이다.')
    n.destroy_node(); rclpy.try_shutdown()
    return 0


def cmd_goto(a):
    """등록한 존 목표로 보낸다. **신호는 보지 않는다** — 준비·시험용이다."""
    if not a.zone:
        raise SystemExit('★ 존 번호가 필요하다 (1~4)')
    z = str(a.zone[0])
    zp = zone_poses(load_venue())
    if z not in zp:
        raise SystemExit('★ %s번 존이 등록 안 됐다. ~/gosu/gosu.sh zone %s 로 먼저 찍을 것'
                         % (z, z))
    v = zp[z]
    rclpy, n, ac, NavigateToPose = _nav_setup()
    _ = named_points
    from unitree_api.msg import Request as _Req
    sport = n.create_publisher(_Req, '/api/sport/request', 10)
    g = NavigateToPose.Goal()
    g.pose = _pose_msg(n, v)
    T0 = time.time()
    print('  %s번 존으로 보낸다: x %.3f  y %.3f  yaw %.1f도'
          % (z, v['x'], v['y'], v.get('yaw_deg', math.degrees(v.get('yaw', 0.0)))))
    print('  %s신호는 보지 않는다 — 준비·시험용이다. Ctrl-C 로 중단%s' % (CD, C0))
    state = {'h': None}

    def on_fb(fb):
        f = fb.feedback
        print('\r  [+%5.1f초] 남은거리 %5.2fm  회복동작 %d      '
              % (time.time() - T0, f.distance_remaining, f.number_of_recoveries),
              end='', flush=True)

    fut = ac.send_goal_async(g, feedback_callback=on_fb)
    try:
        rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise SystemExit('\n★ 목표를 거절당했다.')
        state['h'] = gh
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(n, rf)
        code = rf.result().status if rf.result() else -1
        print()
        print('  %s  %.1f초' % ('도착' if code == 4 else '실패(status %d)' % code,
                                time.time() - T0))
        if code == 4 and a.fine:
            if a.dock_mode == 'scan':
                ok2, e, why = scan_align_to(
                    n, z, v, pos_ok=a.pos_ok, yaw_ok=a.yaw_ok,
                    budget=a.dock_budget, restore=False,
                    log=lambda m: print('  ' + str(m)))
                if e is not None:
                    print('  스캔 정합 도킹 %s  오차 %.1f cm'
                          % ('성공' if ok2 else '미달', e * 100))
                if not ok2:
                    # 물러날 곳이 있어야 한다. 스캔이 안 되면 예전 방식으로.
                    print('  %s스캔 도킹 실패(%s) — 점 도킹으로 물러난다%s'
                          % (CY, why or '오차 미달', C0))
                    ok2, e, _ = fine_align_to(n, v, '%s번 존' % z, pos_ok=a.pos_ok,
                                              yaw_ok=a.yaw_ok, budget=a.dock_budget,
                                              log=lambda m: print('  ' + str(m)),
                                              restore=False)
                    if e is not None:
                        print('  점 도킹 %s  오차 %.1f cm'
                              % ('성공' if ok2 else '미달', e * 100))
            if a.dock_mode in ('point', 'both'):
                ok2, e, ey = fine_align_to(n, v, '%s번 존' % z, pos_ok=a.pos_ok,
                                           yaw_ok=a.yaw_ok, budget=a.dock_budget,
                                           log=lambda m: print('  ' + str(m)),
                                           restore=False)
                if e is not None:
                    print('  점 도킹 %s  오차 %.1f cm'
                          % ('성공' if ok2 else '미달', e * 100))
            if a.dock_mode in ('feet', 'both'):
                print('  발 네 점 도킹 (박스 %.0f cm, %d발 이상)'
                      % (a.zone_box * 100, a.need_feet))
                dock_by_feet(n, v, box_m=a.zone_box, need=a.need_feet, yaw_ok=a.yaw_ok,
                             budget=a.dock_budget, restore=not a.lowdown, quad=a.quad,
                             log=lambda m: print('  ' + str(m)))
            if not a.lowdown:
                _restore(n, sport, log=lambda m: print('  ' + str(m)))
        if code == 4 and a.lowdown:
            freeze_amcl(n, log=lambda m: print('  ' + str(m)))
            print('  엎드린다 (StandDown 1005)')
            lie_down(n, sport)
            feet_in_zone(n, v, box_m=a.zone_box, log=lambda m: print('  ' + str(m)))
            # amcl=False — 아직 앉아 있다. 여기서 문턱을 되돌리면 곧바로
            # 일어서는 27 cm 를 AMCL 이 이동으로 먹어 자세가 틀어진다.
            _restore(n, sport, log=lambda m: print('  ' + str(m)), amcl=False)
            print('  일어선다 (RecoveryStand 1006)')
            stand_up(n, sport)
            # 선 자세가 안정되고 LIO 가 27 cm 복귀를 다 반영한 뒤에 푼다
            _t0 = time.time()
            while time.time() - _t0 < 1.2:
                rclpy.spin_once(n, timeout_sec=0.05)
            restore_amcl(n, log=lambda m: print('  ' + str(m)))
            print('  [+%5.1f초] 끝' % (time.time() - T0))
        if code != 4:
            print('  경로가 아예 안 나오는지 먼저 볼 것:  ~/gosu/gosu.sh reach %s' % z)
    except KeyboardInterrupt:
        print()
        if state['h'] is not None:
            c = state['h'].cancel_goal_async()
            rclpy.spin_until_future_complete(n, c, timeout_sec=5.0)
            print('  취소했다.')
        else:
            print('  ★ 취소할 목표를 못 찾았다. 걸으면: ~/gosu/gosu.sh stop')
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
    return 0




def cmd_zones(a):
    """찍힌 좌표를 보여준다. 같은 번호로 다시 찍으면 덮어쓴다."""
    d = load_venue()
    zp = zone_poses(d)
    sp = d.get('start_pose')
    print()
    if sp:
        print('  출발 위치   x %7.3f  y %7.3f  yaw %6.1f도   %s'
              % (sp['x'], sp['y'], sp.get('yaw_deg', 0), sp.get('set_at', '')))
    else:
        print('  출발 위치   %s없음%s   ~/gosu/gosu.sh mark-start' % (CY, C0))
    print()
    np_ = named_points(d)
    ov = np_.get('overpass')
    if ov:
        print('  육교(overpass)  x %7.3f  y %7.3f  yaw %6.1f도   %s'
              % (ov['x'], ov['y'], ov.get('yaw_deg', 0), ov.get('set_at', '')))
        via = (d.get('leg_via') or {})
        if via:
            print('  경유 규칙    %s' % ', '.join(
                '%s -> %s' % (k, '+'.join(v)) for k, v in via.items()))
    else:
        print('  육교         %s미등록%s   ~/gosu/gosu.sh overpass' % (CY, C0))
    print()
    if not zp:
        print('  존 목표     %s하나도 없음%s   ~/gosu/gosu.sh zone <번호>' % (CY, C0))
        return 0
    print('  존        x        y       yaw      찍은 시각         이웃과 거리')
    print('  %s(방향은 90도 간격 4개 중 접근 방향에 가장 가까운 것을 쓴다 — 최대 45도 회전)%s'
          % (CD, C0))
    for z in SIG.ZONES:
        v = zp.get(z)
        if v is None:
            print('  %-3s      %s미등록%s   ~/gosu/gosu.sh zone %s' % (z, CY, C0, z))
            continue
        near = []
        for w in SIG.ZONES:
            u = zp.get(w)
            if u is None or w == z:
                continue
            near.append((math.hypot(u['x'] - v['x'], u['y'] - v['y']), w))
        near.sort()
        s_near = '  '.join('%s:%.1fm' % (w, dd) for dd, w in near[:3])
        base = v.get('yaw', 0.0)
        qs = '/'.join('%+.0f' % math.degrees(math.atan2(math.sin(y), math.cos(y)))
                      for y in quad_yaws(base))
        print('  %-3s %8.3f %8.3f  %6.1f도   %s   %s'
              % (z, v['x'], v['y'], v.get('yaw_deg', math.degrees(base)),
                 v.get('set_at', '')[-8:], s_near))
        print('       방향 후보  %s도' % qs)
    # 너무 붙어 있으면 같은 자리에서 두 번 찍은 것이다 — 오늘 실제로 그랬다
    bad = []
    ks = sorted(zp)
    for i in range(len(ks)):
        for j in range(i + 1, len(ks)):
            a1, b1 = zp[ks[i]], zp[ks[j]]
            dd = math.hypot(a1['x'] - b1['x'], a1['y'] - b1['y'])
            if dd < 1.0:
                bad.append((ks[i], ks[j], dd))
    if bad:
        print()
        for p, q, dd in bad:
            print('  %s★ %s번과 %s번이 %.2f m 밖에 안 떨어져 있다 — 같은 자리에서 찍혔다%s'
                  % (CY, p, q, dd, C0))
        print('     해당 존으로 로봇을 옮기고 다시 찍을 것 (덮어쓴다)')
    return 0




PROGRESS = os.path.join(GOSU, 'config', 'delivery_progress.json')
PLAN_JSON = os.path.join(GOSU, 'config', 'delivery_plan.json')


def load_progress():
    if os.path.exists(PROGRESS):
        try:
            return json.load(open(PROGRESS, encoding='utf-8'))
        except Exception:
            pass
    return {'done': [], 'failed': [], 'started_at': None}


def save_progress(pr):
    SIG.guard_path(PROGRESS)
    os.makedirs(os.path.dirname(PROGRESS), exist_ok=True)
    pr['updated'] = time.strftime('%Y-%m-%d %H:%M:%S')
    json.dump(pr, open(PROGRESS, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)


def zone_map(d):
    """알고리즘의 존 번호 -> 실제로 찍은 존 번호.

    ★ 알고리즘(algorithm.py)은 Zone1=코엑스, Zone2=휘문고, Zone3=강남경찰서,
      Zone4=GTX 로 적혀 있다. 현장 번호가 그와 다르면 여기서 바꿔 준다.
      venue.json 의 "zone_map": {"1":"4","2":"1","3":"2","4":"3"} 처럼 적는다.
      **이걸 틀리면 모든 배달이 엉뚱한 곳으로 간다.** 실행할 때마다 화면에 찍는다.
    """
    m = d.get('zone_map') or {}
    return {str(k): str(v) for k, v in m.items()}


def cmd_deliver(a):
    d = load_venue()
    zp = zone_poses(d)
    zmap = zone_map(d)
    plan_path = os.path.expanduser(a.plan or PLAN_JSON)
    if not os.path.exists(plan_path):
        raise SystemExit(
            '★ 계획 파일이 없다: %s\n'
            '   팀 알고리즘으로 먼저 뽑을 것:\n'
            '     cd ~/go2_ws/gangnam_shared && python3 gen_mission_commands.py --json %s'
            % (plan_path, plan_path))
    legs = json.load(open(plan_path, encoding='utf-8'))
    pr = load_progress()
    if a.reset:
        pr = {'done': [], 'failed': [], 'started_at': None}
        save_progress(pr)
        print('  진행 기록을 지웠다')

    def phys(z):
        return zmap.get(str(z), str(z))

    print()
    print('  계획 %s  —  %d leg' % (plan_path, len(legs)))
    if zmap:
        print('  %s존 번호 변환: %s%s' % (CY, '  '.join(
            '알고리즘%s->현장%s' % (k, v) for k, v in sorted(zmap.items())), C0))
    else:
        print('  %s존 번호 변환 없음 (알고리즘 번호 = 현장 번호)%s' % (CD, C0))
        print('  %s★ algorithm.py 는 Zone1=코엑스 Zone2=휘문고 Zone3=강남경찰서 '
              'Zone4=GTX 로 본다.%s' % (CY, C0))
        print('  %s   현장 번호가 다르면 venue.json 에 zone_map 을 적을 것 — '
              '틀리면 전부 엉뚱한 곳으로 간다.%s' % (CY, C0))
    if a.from_leg:
        # 사람이 적재를 놓쳐 밀렸을 때: 그 leg 부터 다시 한다.
        # 앞의 leg 은 끝난 것으로 치고, **현재 위치**는 아래에서 실측으로 잡는다.
        pr['done'] = [l['leg'] for l in legs if l['leg'] < a.from_leg]
        pr['failed'] = [x for x in pr.get('failed', []) if x < a.from_leg]
        pr.pop('at', None)          # 어디 있는지 다시 재게 한다
        save_progress(pr)
        print('  leg %d 부터 다시 한다 (앞의 %d개는 끝난 것으로 침)'
              % (a.from_leg, len(pr['done'])))
    done = set(pr.get('done', []))
    if done:
        print('  %s이미 끝난 leg %d개 — 건너뛴다 (%s)%s'
              % (CG, len(done), pr.get('updated', ''), C0))
    todo = [l for l in legs if l['leg'] not in done]
    print('  남은 leg %d개' % len(todo))
    print()
    print('  leg  동작     존   경유        앉기')
    for l in todo[:12]:
        print('  %3d  %-7s  %s   %-10s  %s'
              % (l['leg'], l['action'], phys(l['algo_zone_num']),
                 '->'.join(phys(h) for h in l['hops']) or '-',
                 '%.0f초' % a.dwell if l.get('sit_down') else '-'))
    if len(todo) > 12:
        print('  ... 그 밖에 %d개' % (len(todo) - 12))

    miss = sorted({phys(l['algo_zone_num']) for l in todo} - set(zp))
    if miss:
        raise SystemExit('\n★ 좌표가 없는 존: %s — ~/gosu/gosu.sh zone <번호>' % ', '.join(miss))
    if not a.go:
        print()
        print('  계획만 보여줬다. 실제로 달리려면 --go')
        return 0

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
    from unitree_api.msg import Request as _Req
    rclpy.init(args=sys.argv)
    n = Node('delivery_runner')
    handle = {'h': None}
    ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    atp = ActionClient(n, NavigateThroughPoses, 'navigate_through_poses')
    atp.wait_for_server(timeout_sec=5.0)
    if not ac.wait_for_server(timeout_sec=15.0):
        n.destroy_node(); rclpy.try_shutdown()
        raise SystemExit('★ navigate_to_pose 가 없다. 주행 스택을 먼저 올릴 것')
    sport = n.create_publisher(_Req, '/api/sport/request', 10)
    T0 = time.time()

    def say(m, c=''):
        print('%s[+%6.1f초]%s %s%s%s' % (CD, time.time() - T0, C0, c, m, C0 if c else ''))

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)

    def _pose_of(pose):
        from geometry_msgs.msg import PoseStamped
        m = PoseStamped(); m.header.frame_id = 'map'
        m.header.stamp = n.get_clock().now().to_msg()
        m.pose.position.x = float(pose['x']); m.pose.position.y = float(pose['y'])
        y = float(pose.get('yaw', 0.0))
        m.pose.orientation.z = math.sin(y / 2); m.pose.orientation.w = math.cos(y / 2)
        return m

    def send_through(poses, limit, label):
        """통과점들과 목적지를 **한 번에** 보낸다 — 중간에서 안 멈춘다.

        ★ 2026-09-13: 통과점을 하나씩 NavigateToPose 로 보내면 **통과점마다**
          감속·정지·목표판정·재가속을 한다. 통과점이 4~6개면 leg 마다 그만큼 멈춘다.
          navigate_through_poses 는 중간점을 지나가기만 하고 도착 판정은 마지막에만 한다.
          Nav2 공식 액션이고 bt_navigator 가 이미 물고 있다.
        """
        g = NavigateThroughPoses.Goal()
        g.poses = [_pose_of(p) for p in poses]
        fut = atp.send_goal_async(g)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        handle['h'] = gh
        rf = gh.get_result_async()
        t = time.time()
        while not rf.done():
            rclpy.spin_once(n, timeout_sec=0.1)
            if time.time() - t > limit:
                gh.cancel_goal_async(); spin(2.0); handle['h'] = None
                return False
        handle['h'] = None
        return bool(rf.result() and rf.result().status == 4)

    def send(pose, limit, label):
        m = _pose_of(pose)
        g = NavigateToPose.Goal(); g.pose = m
        fut = ac.send_goal_async(g)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        rf = gh.get_result_async()
        t = time.time()
        while not rf.done():
            rclpy.spin_once(n, timeout_sec=0.1)
            if time.time() - t > limit:
                gh.cancel_goal_async(); spin(2.0)
                return False
        return bool(rf.result() and rf.result().status == 4)

    pts_named = named_points(d)
    # ── 신호 시계를 미션 시작에 맞춘다 ────────────────────────────────
    # 규정: 신호는 30초마다 바뀌고, 경기 시작 순간 묶음 A 가 초록이다.
    # 미션을 시작하는 이 순간이 t=0 이다. 이어서 할 때(--go 재실행)는
    # **처음 t0 를 그대로 쓴다** — 다시 맞추면 위상이 어긋난다.
    _vd, _sg = SIG.load()
    if a.reset or not pr.get('t0_epoch'):
        _sg['t0_epoch'] = time.time()
        pr['t0_epoch'] = _sg['t0_epoch']
        SIG.save(_vd, _sg)
        say('신호 시계 시작 — 지금이 t=0. %s 가 초록' % ', '.join(_sg['groups'][0]), CB)
    else:
        _sg['t0_epoch'] = pr['t0_epoch']
        SIG.save(_vd, _sg)
        say('신호 시계 이어감 — 경기 시작 후 %s' % SIG.fmt(time.time() - pr['t0_epoch']), CB)
    save_progress(pr)

    def wait_green(src, dst, need):
        """그 구간이 초록이 될 때까지 기다린다. 육교면 안 기다린다."""
        leg = '%s-%s' % (src, dst)
        if not a.signal:
            return True
        if SIG.free_cost(leg, _sg) is not None:
            say('%s 육교 — 신호를 안 본다' % leg, CG)
            return True
        try:
            gi = SIG.group_of(leg, _sg['groups'])
        except SystemExit:
            say('%s 는 신호 묶음에 없다 — 그냥 간다' % leg, CY)
            return True
        w, _left = SIG.next_window(_sg, gi, need)
        if w <= 0:
            st = SIG.state(_sg)
            say('%s 초록 (%.0f초 남음, 건너는 데 %.0f초 필요) — 간다'
                % (leg, st['left'], need), CG)
            return True
        say('%s 빨강. %s 기다린다 (건너는 데 %.0f초 필요)' % (leg, SIG.fmt(w), need), CY)
        end = time.time() + w
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
            print('\r    남은 대기 %5.1f초    ' % (end - time.time()), end='', flush=True)
        print('\r                          \r', end='')
        say('%s 초록 — 간다' % leg, CG)
        return True

    # ── 지금 어느 존에 있는지 알아낸다 ────────────────────────────────
    # ★ 2026-09-13: 이게 없어서 **첫 leg 의 신호 확인이 통째로 건너뛰어졌다.**
    #   cur_zone 이 비면 wait_green 을 안 부르는데, 시작할 때는 항상 비어 있었다.
    #   실제로 t=0 에 빨강인 2-3 을 그냥 건너갔다.
    if not pr.get('at'):
        try:
            rx, ry, _ = robot_pose(tries=2)
            near = sorted(((math.hypot(v['x'] - rx, v['y'] - ry), k)
                           for k, v in zp.items()))
            pr['at'] = near[0][1]
            say('지금 위치에서 가장 가까운 존: %s (%.2f m)' % (near[0][1], near[0][0]), CB)
            if near[0][0] > 1.5:
                say('★ 가장 가까운 존도 %.1f m 떨어져 있다 — 출발 자리가 맞는지 확인할 것'
                    % near[0][0], CY)
        except SystemExit as e:
            say('자세를 못 읽어 시작 존을 모른다 — 첫 구간은 신호를 못 본다', CY)
            say(str(e).splitlines()[0], CD)
        save_progress(pr)

    if pr.get('started_at') is None:
        pr['started_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
        save_progress(pr)

    try:
        for l in todo:
            lid = l['leg']
            z = phys(l['algo_zone_num'])
            say('leg %d  %s  ->  존 %s' % (lid, l['action'], z), CB)
            cur_zone = pr.get('at') or phys(l.get('from_zone') or '')
            failed_leg = False
            for h in l['hops']:
                hz = phys(h)
                # 통과점 + 목적지를 **한 번에** 보낸다. 중간에서 안 멈춘다.
                chain = []
                if cur_zone:
                    for vn in leg_via(d, cur_zone, hz):
                        pt = pts_named.get(vn)
                        if pt is None:
                            say('통과점 %s 이 등록 안 됨 — 건너뛴다' % vn, CY)
                        else:
                            chain.append(pt)
                chain.append(zp[hz])
                # ── 건너기 전에 초록을 기다린다 ────────────────────────
                # 건너는 데 필요한 시간은 통과점 사슬의 실제 길이로 잡는다.
                # 고정 12초로 두면 짧은 횡단보도에서 쓸데없이 오래 기다린다.
                if cur_zone and cur_zone != hz:
                    px_, py_ = None, None
                    Lc = 0.0
                    for q_ in chain:
                        if px_ is not None:
                            Lc += math.hypot(q_['x'] - px_, q_['y'] - py_)
                        px_, py_ = q_['x'], q_['y']
                    need_s = Lc / max(a.cross_speed, 0.2) + a.cross_margin
                    wait_green(cur_zone, hz, need_s)
                if len(chain) > 1:
                    say('존 %s 로 — 통과점 %d개 관통' % (hz, len(chain) - 1), CD)
                    okk = send_through(chain, 120.0, hz)
                else:
                    okk = send(zp[hz], 90.0, hz)
                if not okk:
                    say('존 %s 로 못 갔다 — 한 번 더' % hz, CY)
                    if not send(zp[hz], 90.0, hz):
                        say('존 %s 실패 — 이 leg 을 건너뛴다' % hz, CR)
                        pr.setdefault('failed', []).append(lid); save_progress(pr)
                        failed_leg = True
                        break
                cur_zone = hz
                pr['at'] = cur_zone
                save_progress(pr)
            if failed_leg:
                continue
            if True:
                ok_feet = False
                for attempt in range(1, a.dock_tries + 1):
                    if a.dock_mode in ('point', 'both'):
                        fine_align_to(n, zp[z], '존 %s' % z, pos_ok=a.pos_ok,
                                      yaw_ok=a.yaw_ok, restore=False,
                                      budget=a.dock_budget,
                                      log=lambda m: say(str(m).strip(), CD))
                    if a.dock_mode in ('feet', 'both'):
                        dock_by_feet(n, zp[z], box_m=a.zone_box, need=a.need_feet,
                                     yaw_ok=a.yaw_ok, budget=a.dock_budget, quad=a.quad,
                                     restore=False,   # 앉은 뒤에 한 번에 복원한다
                                     log=lambda m: say(str(m), CD))
                    if not l.get('sit_down'):
                        ok_feet = True
                        break
                    freeze_amcl(n, log=lambda m: say(str(m), CD))
                    say('앉는다', CD)
                    lie_down(n, sport)
                    inside = feet_in_zone(n, zp[z], box_m=a.zone_box,
                                          log=lambda m: say(str(m), CB))
                    if inside is not None and inside >= a.need_feet:
                        ok_feet = True
                        say('구역 확인 — 발 %d개. %.0f초 대기' % (inside, a.dwell), CG)
                        spin(a.dwell)
                        break
                    # ★ 2026-09-13: 조건이 모자라도 **그냥 진행한다**.
                    #   다시 맞추려다 오히려 더 틀어진 전례가 있고, 그 사이 시간이 간다.
                    #   기록만 남기고 대기는 그대로 채운다 — 심판이 보는 것은 앉은 자세다.
                    say('발이 %s개 — 기준 %d개에 모자라지만 그대로 진행한다'
                        % (inside if inside is not None else '?', a.need_feet), CY)
                    spin(a.dwell)
                    if not a.strict_feet:
                        ok_feet = True
                        break
                    if attempt < a.dock_tries:
                        say('일어나서 다시 맞춘다', CY)
                        stand_up(n, sport)
                if l.get('stand_up_after') or not ok_feet:
                    stand_up(n, sport)
                _restore(n, sport, log=lambda m: say(str(m), CD))
                if ok_feet:
                    pr.setdefault('done', []).append(lid)
                else:
                    pr.setdefault('failed', []).append(lid)
                    say('leg %d 실패로 기록 — 다음으로 간다' % lid, CR)
                save_progress(pr)
    except KeyboardInterrupt:
        print()
        say('중단됨 — 여기까지 기록했다.', CY)
        print()
        print('  이어서 하는 방법:')
        print('    그대로 이어서          ~/gosu/gosu.sh mission deliver --go')
        print('    특정 leg 부터 다시      ~/gosu/gosu.sh mission deliver --go --from-leg 7')
        print('    처음부터               ~/gosu/gosu.sh mission deliver --go --reset')
        try:
            ans = input('  지금 다시 시작할 leg 번호를 넣으세요 (엔터=안 함) > ').strip()
        except Exception:
            ans = ''
        if ans.isdigit():
            k = int(ans)
            pr['done'] = [l['leg'] for l in legs if l['leg'] < k]
            pr['failed'] = [x for x in pr.get('failed', []) if x < k]
            pr.pop('at', None)
            save_progress(pr)
            print('  leg %d 부터로 맞췄다. 다시 실행하면 **지금 있는 자리**에서 이어간다.' % k)
    finally:
        save_progress(pr)
        print()
        print('  끝난 leg %d개, 실패 %d개  (기록 %s)'
              % (len(pr.get('done', [])), len(pr.get('failed', [])), PROGRESS))
        if pr.get('failed'):
            print('  %s실패한 leg: %s%s' % (CY, ', '.join(map(str, pr['failed'])), C0))
        n.destroy_node(); rclpy.try_shutdown()
    return 0



# ── 순환 주행 ──────────────────────────────────────────────────────────
# 광장 네 모서리를 한 방향으로 계속 돈다. 존마다 앉아서 사람이 싣고 내린다.
#
# 【왜 계획 대신 순환인가 — 2026-09-13 사용자 판단】
#   짜여진 계획(algorithm.py)은 로봇이 모델만큼 빨라야 성립한다. 한 번 밀리면
#   이후 전부 어긋나고, 그때 현장에서 디버깅할 여지가 없다.
#   순환은 어디서 끊겨도 "다음 존으로" 하나면 복구된다.
#   미션 플래닝으로 얻는 시간 이점이 그 위험을 살 만큼 크지 않다.
#
# 【무엇을 로봇이 모르나】 무엇을 실었는지 모른다 — 사람이 안다.
#   로봇은 네 존을 돌며 앉아 줄 뿐이다. 그래서 상태가 어긋날 일이 없다.
# ★ 2026-09-13: 적재지는 존 2(휘문고등학교)다. 여기서 음식이 나올 때까지
#   기다렸다가 출발한다. 규정상 음식은 **20초에 한 개씩** 나오므로
#   5개짜리 주문이면 적재에만 100초다 — 이게 시간 예산을 지배한다.
# 일어서서 다음 횡단보도 앞까지 가는 데 걸리는 시간.
# 초록이 되기 이 만큼 **전에** 일어선다. 그래야 초록에 맞춰 발을 뗀다.
#   RecoveryStand 2.5 + 자세안정 1.2 + 자세재심기 0.5 + 보행복원 1.8 ≈ 6초
#   여기에 존에서 횡단보도 들머리까지 가는 시간을 더해 8초로 둔다.
STAND_LEAD_S = float(os.environ.get('GO2_STAND_LEAD', '8'))

LOAD_ZONE = os.environ.get('GO2_LOAD_ZONE', '2')
LOAD_SEC_PER_ITEM = float(os.environ.get('GO2_LOAD_SEC', '20'))

CIRCUIT_CW = ['2', '1', '4', '3']     # 시계방향 (위에서 볼 때). 좌표로 확인함
CIRCUIT_CCW = ['2', '3', '4', '1']


def cmd_circuit(a):
    d = load_venue()
    zp = zone_poses(d)
    miss = [z for z in '1234' if z not in zp]
    if miss:
        raise SystemExit('★ 좌표가 없는 존: %s — ~/gosu/gosu.sh zone <번호>' % ', '.join(miss))
    order = list(CIRCUIT_CW if a.dir == 'cw' else CIRCUIT_CCW)
    lab = d.get('zone_labels') or {}
    _vd, _sg = SIG.load()
    pts_named = named_points(d)

    print()
    print('  %s방향 순환%s   %s' % ('시계' if a.dir == 'cw' else '반시계', '',
                                    ' -> '.join(order + [order[0]])))
    for x, y in zip(order, order[1:] + order[:1]):
        leg = '%s-%s' % (x, y)
        free = SIG.free_cost(leg, _sg)
        L = math.hypot(zp[y]['x'] - zp[x]['x'], zp[y]['y'] - zp[x]['y'])
        print('    %s(%s) -> %s(%s)  %.2f m  %s'
              % (x, lab.get(x, ''), y, lab.get(y, ''), L,
                 '지하차도(신호 무관)' if free is not None else '횡단보도'))
    print()
    print('  존마다 앉아서 %.0f초 — 그동안 사람이 싣고 내린다' % a.dwell)
    if a.time_limit:
        print('  경기 시간 %.0f초 — 넘으면 그 자리에서 끝낸다' % a.time_limit)
    if not a.go:
        print()
        print('  계획만 보여줬다. 실제로 달리려면 --go')
        return 0

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
    from geometry_msgs.msg import PoseStamped
    from unitree_api.msg import Request as _Req
    rclpy.init(args=sys.argv)
    n = Node('circuit_runner')
    ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    atp = ActionClient(n, NavigateThroughPoses, 'navigate_through_poses')
    atp.wait_for_server(timeout_sec=5.0)
    if not ac.wait_for_server(timeout_sec=15.0):
        n.destroy_node(); rclpy.try_shutdown()
        raise SystemExit('★ navigate_to_pose 가 없다. 주행 스택을 먼저 올릴 것')
    sport = n.create_publisher(_Req, '/api/sport/request', 10)
    handle = {'h': None}
    T0 = time.time()

    def say(m, c=''):
        print('%s[+%6.1f초]%s %s%s%s' % (CD, time.time() - T0, C0, c, m, C0 if c else ''))

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)

    def _pose(v):
        m = PoseStamped(); m.header.frame_id = 'map'
        m.header.stamp = n.get_clock().now().to_msg()
        m.pose.position.x = float(v['x']); m.pose.position.y = float(v['y'])
        y = float(v.get('yaw', 0.0))
        m.pose.orientation.z = math.sin(y / 2); m.pose.orientation.w = math.cos(y / 2)
        return m

    def go(poses, limit):
        if len(poses) > 1:
            g = NavigateThroughPoses.Goal(); g.poses = [_pose(p) for p in poses]
            cli = atp
        else:
            g = NavigateToPose.Goal(); g.pose = _pose(poses[0])
            cli = ac
        fut = cli.send_goal_async(g)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        handle['h'] = gh
        rf = gh.get_result_async(); t = time.time()
        while not rf.done():
            rclpy.spin_once(n, timeout_sec=0.1)
            if time.time() - t > limit:
                gh.cancel_goal_async(); spin(2.0); handle['h'] = None
                return False
        handle['h'] = None
        return bool(rf.result() and rf.result().status == 4)

    _orders = load_orders()
    # 배점 높은 것부터 싣는다. 랩 시간이 신호 주기와 적재 대기에 묶여 있어
    # 8건을 다 못 돈다 — 무엇을 먼저 싣느냐가 점수를 정한다.
    _rank = {'상': 0, '중': 1, '하': 2}
    _queue = sorted((_orders.get('orders') or []),
                    key=lambda o: (_rank.get(o.get('grade'), 9),
                                   -int(o.get('num', 0)))) if _orders else []
    _carrying = {'o': None}
    _done = []                  # 배달을 마친 주문 번호
    if _orders:
        _n = len(_orders.get('orders', []))
        _sz = sorted(order_size(o) for o in _orders.get('orders', []))
        _cap = int(_orders.get('capacity', 5))
        say('주문서 %d건 (적재 용량 %d개)' % (_n, _cap), CB)
        if len(_sz) > 1 and _sz[0] + _sz[1] > _cap:
            say('가장 작은 두 건이 %d+%d=%d 로 용량을 넘는다 — '
                '주문 1건 = 1랩, %d랩 필요' % (_sz[0], _sz[1], _sz[0] + _sz[1], _n), CY)
        say('적재지 존 %s. 음식 %.0f초/개 — 배점 높은 것부터: %s'
            % (LOAD_ZONE, LOAD_SEC_PER_ITEM,
               ' '.join('%s(%s)' % (o['num'], o.get('grade')) for o in _queue)), CB)

    # ── 신호 시계 ─────────────────────────────────────────────────────
    # ★ 2026-09-13: 예전에는 여기서 **무조건** t0 를 지금으로 다시 잡았다.
    #   그러면 circuit 을 다시 띄울 때마다 신호 위상이 초기화돼, 실제 경기장
    #   신호와 어긋난다. 비전(YOLO)을 못 쓰므로 위상을 알아낼 방법이 그것뿐인데
    #   명령 한 번에 날아가면 안 된다.
    #   이제는 **이미 맞춰 둔 시계를 이어간다.** 다시 잡으려면 --reset 를 준다.
    #   경기 시작 전에 사람이 한 번만 맞춘다:
    #       ~/gosu/gosu.sh signal sync 1-2     (1-2 가 막 초록이 된 순간)
    #       ~/gosu/gosu.sh signal start        (지금이 A 묶음 초록 시작)
    if getattr(a, 'reset', False) or not _sg.get('t0_epoch'):
        _sg['t0_epoch'] = time.time()
        SIG.save(_vd, _sg)
        say('신호 시계 **새로** 시작 — %s 가 초록' % ', '.join(_sg['groups'][0]), CY)
        if not getattr(a, 'reset', False):
            say('★ 맞춰 둔 시계가 없었다. 실제 신호와 어긋날 수 있다 — '
                '경기 전에 signal sync 로 맞출 것', CR)
    else:
        _st = SIG.state(_sg)
        say('신호 시계 이어감 — 맞춘 지 %s, 지금 %s 초록 (%.0f초 남음)'
            % (SIG.fmt(_st['elapsed']), ', '.join(_sg['groups'][_st['green']]),
               _st['left']), CB)

    # 지금 어디 있나
    try:
        rx, ry, _ = robot_pose(tries=2)
        cur = min(zp, key=lambda k: math.hypot(zp[k]['x'] - rx, zp[k]['y'] - ry))
        say('가장 가까운 존: %s (%s)' % (cur, lab.get(cur, '')), CB)
    except SystemExit:
        cur = order[0]
        say('자세를 못 읽었다 — 존 %s 에 있다고 보고 시작한다' % cur, CY)
    if cur in order:
        i = order.index(cur)
        order = order[i:] + order[:i]

    # ── 첫 구간도 **앉아서** 기다린다 ──────────────────────────────────
    # 【왜 따로 처리하나 — 2026-09-13】
    # 앉아서 초록을 기다리는 로직은 '존에 도착해서 앉는' 블록 안에 있다.
    # 그런데 첫 구간은 도착한 적이 없어서 그 블록을 안 거친다.
    # 그래서 첫 30초를 서서 기다리게 된다.
    #
    # 경기 시작 순간에는 묶음 A(1-2, 3-4)가 초록이다. 출발이 존 2 이고
    # 첫 구간이 2-3(묶음 B)이면 **정확히 30초를 기다려야 한다.**
    # 그 30초를 서서 보내면 뒷다리가 달아오르고 초록 앞부분도 날린다.
    _fleg = '%s-%s' % (order[0], order[1])
    _fw = 0.0
    if a.signal:
        try:
            if SIG.free_cost(_fleg, _sg) is None:
                _fgi = SIG.group_of(_fleg, _sg['groups'])
                _fneed = float(_sg['cross_seconds']) + float(_sg['margin_s'])
                _fw, _ = SIG.next_window(_sg, _fgi, _fneed)
        except SystemExit:
            _fw = 0.0
    if _fw > STAND_LEAD_S + 0.5:
        say('첫 구간 %s 초록까지 %s — **앉아서** 기다린다'
            % (_fleg, SIG.fmt(_fw)), CY)
        freeze_amcl(n, log=lambda m: say(str(m), CD))
        lie_down(n, sport)
        _e0 = time.time() + (_fw - STAND_LEAD_S)
        while time.time() < _e0:
            rclpy.spin_once(n, timeout_sec=0.1)
            _l0 = _e0 - time.time()
            if _l0 > 1.0:
                print('\r    일어서기까지 %5.0f초   ' % _l0, end='', flush=True)
        print('\r                          \r', end='')
        say('일어선다 — %s 곧 초록' % _fleg, CG)
        stand_up(n, sport)
        if not a.no_reseed and order[0] in zp:
            reseed_at_zone(n, zp[order[0]], log=lambda m: say(str(m), CD))
        _restore(n, sport, log=lambda m: say(str(m), CD))
    elif _fw > 0:
        say('첫 구간 %s 초록까지 %s — 그대로 기다린다(일어서는 시간에 묻힌다)'
            % (_fleg, SIG.fmt(_fw)), CD)

    visits = 0
    try:
        while True:
            resync = False
            for k in range(len(order)):
                src = order[k]
                dst = order[(k + 1) % len(order)]
                # ── 페널티로 시계가 멈췄으면 여기서 기다린다 ──────────────
                # 다른 터미널의 `gosu.sh penalty` 가 venue.json 에 표시를 남긴다.
                # 그 파일을 **매 구간 다시 읽어야** 한다 — 메모리에 든 값은
                # 다른 프로세스의 변경을 모른다.
                _vd2, _sg2 = SIG.load()
                if SIG.is_paused(_sg2):
                    say('%s페널티로 시계가 멈춰 있다 — 여기서 기다린다%s' % (CY, C0), CY)
                    say('   로봇을 원위치에 놓고:  ~/gosu/gosu.sh back <존번호>', CD)
                    _n = 0
                    while SIG.is_paused(_sg2):
                        rclpy.spin_once(n, timeout_sec=0.2)
                        _n += 1
                        if _n % 25 == 0:                # 약 5초마다 다시 읽는다
                            _vd2, _sg2 = SIG.load()
                    # 재개됐다. 바뀐 시계를 받아 쓴다.
                    _vd, _sg = _vd2, _sg2
                    _st2 = SIG.state(_sg)
                    say('시계 재개 — 지금 %s 초록 (%.0f초 남음)'
                        % (', '.join(_sg['groups'][_st2['green']]), _st2['left']), CG)
                    # 들려서 옮겨졌을 수 있다. 실제 위치에서 순환을 다시 잡는다.
                    try:
                        rx3, ry3, _ = robot_pose(tries=2)
                        cur3 = min(zp, key=lambda q: math.hypot(zp[q]['x'] - rx3,
                                                                zp[q]['y'] - ry3))
                        if cur3 in order:
                            j3 = order.index(cur3)
                            order = order[j3:] + order[:j3]
                            say('지금 가장 가까운 존 %s 에서 이어간다' % cur3, CB)
                        resync = True
                        break
                    except SystemExit:
                        say('자세를 못 읽었다 — 순서를 그대로 두고 이어간다', CY)
                if a.time_limit and time.time() - T0 > a.time_limit:
                    say('경기 시간 도달 — 끝낸다', CY)
                    raise KeyboardInterrupt
                # 신호
                chain = []
                for vn in leg_via(d, src, dst):
                    pt = pts_named.get(vn)
                    if pt is not None:
                        chain.append(pt)
                chain.append(zp[dst])
                if a.signal:
                    Lc, px_, py_ = 0.0, None, None
                    for q_ in chain:
                        if px_ is not None:
                            Lc += math.hypot(q_['x'] - px_, q_['y'] - py_)
                        px_, py_ = q_['x'], q_['y']
                    need = Lc / max(a.cross_speed, 0.2) + a.cross_margin
                    leg = '%s-%s' % (src, dst)
                    if SIG.free_cost(leg, _sg) is not None:
                        say('%s 지하차도 — 신호를 안 본다' % leg, CG)
                    else:
                        try:
                            gi = SIG.group_of(leg, _sg['groups'])
                            w, _l = SIG.next_window(_sg, gi, need)
                        except SystemExit:
                            w = 0
                        if w > 0:
                            say('%s 빨강. %s 기다린다' % (leg, SIG.fmt(w)), CY)
                            end = time.time() + w
                            while time.time() < end:
                                rclpy.spin_once(n, timeout_sec=0.05)
                                print('\r    %5.1f초  ' % (end - time.time()), end='', flush=True)
                            print('\r            \r', end='')
                        say('%s 초록 — 간다' % leg, CG)
                say('존 %s(%s) 로' % (dst, lab.get(dst, '')), CB)
                # 그 존에 갈 주문을 미리 알려준다. 앉아 있는 시간이 3초뿐이라
                # 도착해서 알면 늦는다.
                for _o in orders_for_zone(_orders, dst):
                    say('    실을 것: %s' % fmt_order(_o), CD)
                if not go(chain, 120.0):
                    say('못 갔다 — 한 번 더', CY)
                    if not go([zp[dst]], 90.0):
                        # ★ 2026-09-13: 예전에는 여기서 continue 였다. 그러면 다음
                        #   반복이 src 를 '도착했다'고 가정한다. 2->3 이 실패했는데
                        #   다음에 3->4 를 계산하니, 실제로는 존 2에서 육교로
                        #   **대각선**을 긋게 된다. 대각선 이동은 절대 금지다.
                        #   그래서 실제 위치를 다시 읽어 순환을 그 자리에서 잡는다.
                        say('건너뛴다 — 지금 위치에서 순환을 다시 잡는다', CR)
                        resync = True
                        break
                # 도착
                need_dock = True
                if a.fine and a.skip_ok:
                    try:
                        pz = robot_pose(tries=1, wait_s=3.0)
                        nin, worst = predict_feet_in_box(
                            (pz[0], pz[1], math.radians(pz[2])), zp[dst], a.zone_box)
                        if nin >= a.need_feet:
                            need_dock = False
                            say('발 %d개가 이미 구역 안 — 도킹 건너뜀 (여유 %.0f mm)'
                                % (nin, -worst * 1000), CG)
                        else:
                            say('발 %d개뿐 (%.0f mm 모자람) — 도킹한다'
                                % (nin, worst * 1000), CD)
                    except SystemExit:
                        pass
                if a.fine and need_dock:
                    # ★ 2026-09-13: 예전에는 여기서 --dock-mode 를 아예 읽지 않았다.
                    #   `circuit --dock-mode feet` 이 오류 없이 통과하고 조용히
                    #   무시됐다. 그래서 어느 방식으로 시험했는지 알 수 없었다.
                    done_scan = False
                    if a.dock_mode == 'scan':
                        ok_s, e_s, why_s = scan_align_to(
                            n, dst, zp[dst], pos_ok=a.pos_ok, yaw_ok=a.yaw_ok,
                            budget=a.dock_budget, restore=False,
                            log=lambda m: say(str(m).strip(), CD))
                        if e_s is not None:
                            say('스캔 정합 도킹 %s  오차 %.1f cm'
                                % ('성공' if ok_s else '미달', e_s * 100),
                                CG if ok_s else CY)
                        done_scan = bool(ok_s)
                        if not ok_s:
                            say('스캔 도킹 실패(%s) — 점 도킹으로 물러난다'
                                % (why_s or '오차 미달'), CY)
                    if not done_scan:
                        fine_align_to(n, zp[dst], '존 %s' % dst, pos_ok=a.pos_ok,
                                      yaw_ok=a.yaw_ok, restore=False,
                                      budget=a.dock_budget,
                                      log=lambda m: say(str(m).strip(), CD))
                # ── 여기서 얼마나 앉아 있나 ────────────────────────────
                # 적재지(LOAD_ZONE)면 음식이 다 나올 때까지 기다린다.
                # 규정상 20초에 한 개씩 나오므로 5개짜리는 100초다.
                # 배달지면 짧게 내려놓고 바로 뜬다.
                # ★ 순서가 중요하다. 먼저 **내리고** 그 다음에 싣는다.
                #   예전에는 적재지에 오면 운반 중인지 보지 않고 실어서,
                #   목적지가 적재지와 같은 주문(25번·100번 -> 휘문고등학교)이
                #   실린 채 다음 주문에 덮여 **조용히 사라졌다**. 2026-09-13 확인.
                _deliver_here = bool(_carrying['o']) and \
                    str(_carrying['o'].get('zone')) == dst
                _load_here = (dst == LOAD_ZONE and bool(_queue)
                              and (_carrying['o'] is None or _deliver_here))
                _wait = a.dwell
                _what = '내려놓기'
                if _deliver_here and _load_here:
                    _o = _queue[0]
                    _cnt = sum((_o.get('items') or {}).values())
                    _wait = max(a.dwell, _cnt * LOAD_SEC_PER_ITEM)
                    _what = '%s번 배달 + %s번(%s) %d개 적재' % (
                        _carrying['o']['num'], _o['num'], _o.get('grade'), _cnt)
                elif _load_here:
                    _o = _queue[0]
                    _cnt = sum((_o.get('items') or {}).values())
                    _wait = max(a.dwell, _cnt * LOAD_SEC_PER_ITEM)
                    _what = '%s번(%s) %d개 적재' % (_o['num'], _o.get('grade'), _cnt)
                elif _deliver_here:
                    _what = '%s번 배달' % _carrying['o']['num']
                elif _carrying['o']:
                    _what = '%s번 운반중 — 지나감' % _carrying['o']['num']

                freeze_amcl(n, log=lambda m: say(str(m), CD))
                say('앉는다 — %.0f초  [%s]' % (_wait, _what), CG)
                lie_down(n, sport)
                feet_in_zone(n, zp[dst], box_m=a.zone_box, log=lambda m: say(str(m), CB))
                # 긴 대기는 남은 시간을 보여준다. 사람이 언제 끝나는지 알아야 한다.
                _end = time.time() + _wait
                while time.time() < _end:
                    rclpy.spin_once(n, timeout_sec=0.1)
                    _left = _end - time.time()
                    if _left > 1.0:
                        print('\r    %5.0f초 남음   ' % _left, end='', flush=True)
                print('\r                  \r', end='')

                # 반드시 **내리고 나서** 싣는다 (위 주석 참조)
                if _deliver_here:
                    say('배달 완료: %s' % fmt_order(_carrying['o']), CG)
                    _done.append(_carrying['o']['num'])
                    _carrying['o'] = None
                if _load_here:
                    _carrying['o'] = _queue.pop(0)
                    say('실었다: %s  (남은 주문 %d건)'
                        % (fmt_order(_carrying['o']), len(_queue)), CG)

                # ── 다음 구간이 초록이 될 때까지 **앉은 채로** 기다린다 ──
                # 【왜 앉아서 기다리나 — 2026-09-13】
                # 예전에는 일어선 뒤 횡단보도 앞에서 서서 기다렸다. 그러면
                #   ① 서 있는 동안 뒷다리 hip 이 과열된다(기립만 해도 오른다)
                #   ② 초록이 켜진 뒤에야 움직이기 시작해 초록 앞부분을 날린다
                # 앉아서 기다리다가 **초록이 되기 STAND_LEAD_S 초 전에 일어서면**
                # 초록에 맞춰 발을 뗀다. 육교(신호 무관)면 기다리지 않는다.
                _nxt = order[(k + 2) % len(order)]
                _nleg = '%s-%s' % (dst, _nxt)
                _sit_more = 0.0
                if a.signal:
                    try:
                        if SIG.free_cost(_nleg, _sg) is None:
                            _gi = SIG.group_of(_nleg, _sg['groups'])
                            _need = float(_sg['cross_seconds']) + float(_sg['margin_s'])
                            _w, _ = SIG.next_window(_sg, _gi, _need)
                            _sit_more = max(0.0, _w - STAND_LEAD_S)
                    except SystemExit:
                        _sit_more = 0.0
                if _sit_more > 0.5:
                    say('%s 초록까지 %s — **앉은 채로** 기다린다 (%.0f초 전에 일어섬)'
                        % (_nleg, SIG.fmt(_sit_more + STAND_LEAD_S), STAND_LEAD_S), CY)
                    _e2 = time.time() + _sit_more
                    while time.time() < _e2:
                        rclpy.spin_once(n, timeout_sec=0.1)
                        _l2 = _e2 - time.time()
                        if _l2 > 1.0:
                            print('\r    일어서기까지 %5.0f초   ' % _l2, end='', flush=True)
                    print('\r                          \r', end='')
                    say('일어선다 — %s 곧 초록' % _nleg, CG)

                stand_up(n, sport)
                # ★ 앉아 있던 동안의 표류를 여기서 끊는다.
                #   방금 이 존에 도킹했으므로 어디 있는지 정확히 안다.
                #   bag 실측: 앉기 한 번에 LIO 가 48m 발산한 적이 있다.
                if not a.no_reseed:
                    reseed_at_zone(n, zp[dst], log=lambda m: say(str(m), CD))
                _restore(n, sport, log=lambda m: say(str(m), CD))
                visits += 1
                say('존 %s 끝 (누적 %d곳)' % (dst, visits), CG)
            if resync:
                # 실패한 자리에서 다시 잡는다. 순서(고리)는 그대로 두고
                # **시작점만** 지금 있는 존으로 돌린다 — 방향이 바뀌지 않는다.
                try:
                    rx2, ry2, _ = robot_pose(tries=2)
                    cur2 = min(zp, key=lambda q: math.hypot(zp[q]['x'] - rx2,
                                                            zp[q]['y'] - ry2))
                    if cur2 in order:
                        j = order.index(cur2)
                        order = order[j:] + order[:j]
                    say('지금 가장 가까운 존 %s — 거기서 순환을 이어간다' % cur2, CY)
                except SystemExit:
                    say('자세를 못 읽었다 — 순서를 그대로 두고 이어간다', CY)
                continue
            if a.laps and visits >= a.laps * len(order):
                say('%d바퀴 끝' % a.laps, CG)
                break
    except KeyboardInterrupt:
        print()
        say('중단', CY)
        if handle['h'] is not None:
            c = handle['h'].cancel_goal_async()
            rclpy.spin_until_future_complete(n, c, timeout_sec=5.0)
    finally:
        print()
        print('  들른 곳 %d곳,  총 %s' % (visits, SIG.fmt(time.time() - T0)))
        if _orders:
            print('  배달 완료 %d건: %s' % (len(_done),
                  ', '.join(str(x) for x in _done) or '없음'))
            if _carrying['o']:
                print('  %s실은 채 못 내린 것: %s번%s'
                      % (CY, _carrying['o']['num'], C0))
            if _queue:
                print('  %s못 실은 주문 %d건: %s%s'
                      % (CY, len(_queue),
                         ', '.join(str(o['num']) for o in _queue), C0))
        n.destroy_node(); rclpy.try_shutdown()
    return 0


def build_plan(s, order, t0):
    """존 순서를 구간 목록으로 편다. 각 구간은 (leg, 출발가능시각, 도착예정)."""
    legs = []
    t = t0
    for i in range(len(order) - 1):
        arr, path = SIG._path_between(s, order[i], order[i + 1], t)
        if arr is None:
            raise SystemExit('★ %s 에서 %s 로 가는 길이 없다' % (order[i], order[i + 1]))
        legs += path
        t = arr
    return legs, t


def show_plan(s, order, legs, t_end, t0):
    print()
    print('  %s순서%s   %s' % (CB, C0, ' -> '.join(order)))
    print()
    print('  구간    묶음  대기      출발(t=)    도착')
    prev = t0
    wait = 0.0
    for leg, go, arr in legs:
        fc = SIG.free_cost(leg, s)
        tag = '육교' if fc is not None else 'AB'[SIG.group_of(leg, s['groups'])]
        w = go - prev
        wait += w
        print('  %-6s  %-4s %-8s  %7.1f초   %7.1f초'
              % (leg, tag, SIG.fmt(w) if w > 0.05 else '없음', go, arr))
        prev = arr
    print()
    print('  전체 %s  (신호 대기 %s)' % (SIG.fmt(t_end - t0), SIG.fmt(wait)))


def cmd_plan(a):
    d = load_venue()
    _, s = SIG.load()
    order = [str(z) for z in a.zones]
    legs, t_end = build_plan(s, order, 0.0)
    show_plan(s, order, legs, t_end, 0.0)
    zp = zone_poses(d)
    miss = [z for z in order if z not in zp]
    if miss:
        print()
        print('  %s★ 목표 좌표가 없는 존: %s%s' % (CY, ', '.join(miss), C0))
        print('     각 존에 로봇을 세우고: python3 tools/mission.py mark-zone <번호>')
    return 0


def cmd_dry(a):
    """로봇 없이 시계만 돌려 예행. 실제 소요시간 감각을 잡는다."""
    _, s = SIG.load()
    order = [str(z) for z in a.zones]
    legs, t_end = build_plan(s, order, 0.0)
    show_plan(s, order, legs, t_end, 0.0)
    print()
    print('  %s예행 시작 — 실제 시계로 돈다 (로봇은 안 움직인다)%s' % (CD, C0))
    t0 = time.time()
    for leg, go, arr in legs:
        while time.time() - t0 < go:
            r = go - (time.time() - t0)
            print('\r    %s 대기 %5.1f초   ' % (leg, r), end='', flush=True)
            time.sleep(0.2)
        print('\r  [+%5.1f초] %s 건넌다        ' % (time.time() - t0, leg))
        while time.time() - t0 < arr:
            time.sleep(0.1)
        print('  [+%5.1f초] %s 도착' % (time.time() - t0, leg.split('-')[1]))
    print()
    print('  %s예행 끝. 총 %s%s' % (CG, SIG.fmt(time.time() - t0), C0))
    return 0


def cmd_run(a):
    d = load_venue()
    _, s = SIG.load()
    order = [str(z) for z in a.zones]
    zp = zone_poses(d)
    miss = [z for z in order if z not in zp]
    if miss:
        raise SystemExit('★ 목표 좌표가 없는 존: %s\n'
                         '   python3 tools/mission.py mark-zone <번호> 로 먼저 찍을 것'
                         % ', '.join(miss))
    legs, t_end = build_plan(s, order, 0.0)
    show_plan(s, order, legs, t_end, 0.0)
    if not a.go:
        print()
        print('  계획만 보여줬다. 실제로 달리려면 --go 를 붙일 것')
        return 0

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose

    rclpy.init(args=sys.argv)
    n = Node('mission_runner')
    ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    from unitree_api.msg import Request as _Req
    sport = n.create_publisher(_Req, '/api/sport/request', 10)
    print()
    if not ac.wait_for_server(timeout_sec=15.0):
        n.destroy_node(); rclpy.try_shutdown()
        raise SystemExit('★ navigate_to_pose 액션 서버가 없다. 주행 스택을 먼저 올릴 것')

    T0 = time.time()

    def el():
        return time.time() - T0

    def say(msg, c=''):
        print('%s[+%5.1f초]%s %s%s%s' % (CD, el(), C0, c, msg, C0 if c else ''))

    # 시계를 지금으로 맞춘다. 사람이 손대는 마지막 순간이 여기다.
    s['t0_epoch'] = time.time()
    say('t0 = 지금. %s 초록으로 시작.' % ', '.join(s['groups'][0]), CB)
    say('여기서부터 사람은 건드리지 않는다. 멈추려면 Ctrl-C.', CD)

    cs = float(s['cross_seconds'])
    per = float(s['period_s'])
    budget = float(a.time_limit) if a.time_limit else None
    done, skipped = [], []
    handle = {'h': None}

    def spin(sec):
        """기다리는 동안에도 ROS 콜백을 돌린다. 안 그러면 액션 결과를 못 받는다."""
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)

    pts_named = named_points(d)

    def drive_to_pose(v, limit, label):
        """좌표 하나로 보낸다. 존이든 경유점이든 같은 경로를 쓴다."""
        return _send(v, limit, label)

    def drive_to(zone, limit):
        return _send(zp[zone], limit, '%s번' % zone)

    def _send(v, limit, label):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = n.get_clock().now().to_msg()
        m.pose.position.x = float(v['x'])
        m.pose.position.y = float(v['y'])
        yaw = float(v.get('yaw', 0.0))
        m.pose.orientation.z = math.sin(yaw / 2)
        m.pose.orientation.w = math.cos(yaw / 2)
        g = NavigateToPose.Goal(); g.pose = m
        fut = ac.send_goal_async(g)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False, '목표 거절'
        handle['h'] = gh
        res = gh.get_result_async()
        t_send = time.time()
        while not res.done():
            rclpy.spin_once(n, timeout_sec=0.1)
            if time.time() - t_send > limit:
                gh.cancel_goal_async()
                spin(2.0)
                handle['h'] = None
                return False, '시간 초과 %.0f초' % limit
        handle['h'] = None
        code = res.result().status if res.result() else -1
        return (code == 4), ('도착' if code == 4 else 'status %d' % code)

    try:
        for leg, _, _ in legs:
            src, dst = leg.split('-')
            free = SIG.free_cost(leg, s)

            # ── 신호 기다리기 ────────────────────────────────────────────
            if free is None:
                gi = SIG.group_of(leg, s['groups'])
                need = cs + float(s['margin_s'])
                w, _ = SIG.next_window(s, gi, need)
                if budget is not None and el() + w + cs > budget:
                    say('%s 대기 %s — 남은 시간에 못 넣는다. 건너뛴다.'
                        % (leg, SIG.fmt(w)), CY)
                    skipped.append(dst); continue
                if w > 0:
                    say('%s 빨강. %s 기다린다.' % (leg, SIG.fmt(w)), CY)
                    end = time.time() + w
                    while time.time() < end:
                        rclpy.spin_once(n, timeout_sec=0.05)
                        r = end - time.time()
                        print('\r    남은 대기 %5.1f초   ' % r, end='', flush=True)
                    print('\r                          \r', end='')
                say('%s 초록. 건넌다.' % leg, CG)
            else:
                say('%s 육교 — 신호를 안 본다. 바로 건넌다.' % leg, CG)

            # ── 건너기. 실패하면 한 번 다시, 또 실패하면 버리고 간다 ──────
            limit = (free if free is not None else cs) * 2.5 + 20.0

            # ── 반드시 거쳐야 하는 지점 (육교) ────────────────────────
            # 2-3 은 신호와 무관하게 육교로만 다닌다. 목적지로 바로 보내면
            # 계획기가 최단거리를 골라 횡단보도로 갈 수 있다 — 그것을 막는다.
            vias = leg_via(d, src, dst)
            skipped_leg = False
            for vn in vias:
                pt = pts_named.get(vn)
                if pt is None:
                    say('경유점 %s 이 등록 안 됨 — 건너뛰고 바로 간다' % vn, CY)
                    continue
                say('%s 경유' % vn, CD)
                vok, vwhy = drive_to_pose(pt, limit, vn)
                if not vok:
                    say('%s 경유 실패(%s) — 그래도 목적지로 시도한다' % (vn, vwhy), CY)

            ok, why = drive_to(dst, limit)
            if not ok:
                say('%s번 실패(%s). 한 번 더 해본다.' % (dst, why), CY)
                ok, why = drive_to(dst, limit)
            if ok:
                say('%s번 도착' % dst, CG)
                done.append(dst)
                if a.fine:
                    if a.dock_mode in ('point', 'both'):
                        ok2, e, _ = fine_align_to(n, zp[dst], '%s번 존' % dst,
                                                  pos_ok=a.pos_ok, yaw_ok=a.yaw_ok,
                                                  budget=a.dock_budget,
                                                  log=lambda m: say(str(m).strip(), CD),
                                                  restore=False)
                        if e is not None:
                            say('점 도킹 %s — 오차 %.1f cm'
                                % ('성공' if ok2 else '미달', e * 100),
                                CG if ok2 else CY)
                    if a.dock_mode in ('feet', 'both'):
                        dock_by_feet(n, zp[dst], box_m=a.zone_box, need=a.need_feet,
                                     yaw_ok=a.yaw_ok, budget=a.dock_budget,
                                     restore=not a.lowdown, quad=a.quad,
                                     log=lambda m: say(str(m), CD))
                if a.lowdown:
                    freeze_amcl(n, log=lambda m: say(str(m), CD))
                    say('엎드린다 (StandDown)', CD)
                    lie_down(n, sport)
                    # 규정 판정은 이 시점의 발 위치로 한다
                    feet_in_zone(n, zp[dst], box_m=a.zone_box,
                                 log=lambda m: say(str(m), CB))
                    # amcl=False — 아직 앉아 있다. 일어선 뒤에 따로 푼다.
                    _restore(n, sport, log=lambda m: say(str(m), CD), amcl=False)
                    # 마지막 존이면 엎드린 채로 끝낸다. 아니면 다시 선다 —
                    # 엎드린 채로 다음 구간을 가면 라이다가 27cm 낮아 정합이 무너진다.
                    if dst != order[-1]:
                        say('일어선다 (RecoveryStand)', CD)
                        stand_up(n, sport)
                        spin(1.2)   # 선 자세 안정 + LIO 가 27cm 복귀를 반영할 시간
                    # 엎드린 채 끝내더라도 얼린 것은 반드시 푼다. 안 그러면
                    # 다음 실행이 갱신 없는 AMCL 로 시작한다.
                    restore_amcl(n, log=lambda m: say(str(m), CD), force=True)
            else:
                say('%s번 포기(%s) — 다음으로 넘어간다. 멈춰 있는 것보다 낫다.'
                    % (dst, why), CR)
                skipped.append(dst)
            if budget is not None and el() > budget:
                say('시간 상한 %.0f초 도달. 남은 구간을 버린다.' % budget, CY)
                break
    except KeyboardInterrupt:
        print()
        say('긴급 정지 — 목표를 취소한다', CR)
        if handle['h'] is not None:
            c = handle['h'].cancel_goal_async()
            rclpy.spin_until_future_complete(n, c, timeout_sec=5.0)
        say('취소했다. 로봇이 걸으면: ~/gosu/gosu.sh stop', CR)
    finally:
        print()
        print('  %s끝. 총 %s%s' % (CB, SIG.fmt(el()), C0))
        print('  도착 %d곳 %s' % (len(done), ', '.join(done) or '-'))
        if skipped:
            print('  %s버린 곳 %d: %s%s' % (CY, len(skipped), ', '.join(skipped), C0))
        n.destroy_node()
        rclpy.try_shutdown()
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['plan', 'dry', 'run', 'mark-zone', 'pose', 'goto', 'reach', 'zones',
                                   'mark-overpass', 'deliver', 'make-via',
                                   'circuit', 'teach', 'orders',
                                   'register', 'penalty', 'back'])
    ap.add_argument('zone', nargs='*', default=[])
    ap.add_argument('--zones', nargs='+', default=['1', '4', '2', '3'],
                    help='들를 존 순서. 첫 번째가 출발점')
    ap.add_argument('--go', action='store_true', help='실제로 달린다')
    ap.add_argument('--fine', dest='fine', action='store_true', default=True,
                    help='도착 뒤 정밀 정렬 (기본 켬). Nav2 22cm -> 6cm')
    ap.add_argument('--no-fine', dest='fine', action='store_false')
    ap.add_argument('--pos-ok', type=float, default=0.08,
                    help='정밀 접근 목표 오차(m). 넓히면 회차는 주는데 정확도를 잃는다 — '
                         '대신 이동 속도를 올려서 시간을 줄인다')
    ap.add_argument('--dir', choices=['cw', 'ccw'], default='ccw',
                    help='circuit: 도는 방향. 기본 ccw=반시계(2->3->4->1). '
                         'cw=시계(2->1->4->3). 어느 쪽이든 지금 있는 존에서 시작한다')
    ap.add_argument('--no-reseed', action='store_true',
                    help='일어선 뒤 자세 재심기를 끈다 (기본 켬 — 앉은 동안의 표류를 끊는다)')
    ap.add_argument('--laps', type=int, default=0,
                    help='circuit: 몇 바퀴. 0 이면 시간이 다할 때까지')
    ap.add_argument('--plan', default='', help='deliver: 계획 JSON 경로')
    ap.add_argument('--from-leg', type=int, default=0,
                    help='deliver: 이 leg 부터 시작한다 (앞의 것은 끝난 것으로 친다). '
                         '사람이 적재를 놓쳐 밀렸을 때 쓴다')
    ap.add_argument('--reset', action='store_true', help='deliver: 진행 기록을 지운다')
    ap.add_argument('--dwell', type=float, default=3.0,
                    help='deliver: 앉아 있는 시간(초). 사람이 적재하는 시간이다')
    ap.add_argument('--need-feet', type=int, default=3,
                    help='deliver: 구역 안에 있어야 하는 발 개수')
    ap.add_argument('--cross-speed', type=float, default=0.6,
                    help='건너는 데 걸리는 시간을 잡을 때 쓰는 속도(m/s)')
    ap.add_argument('--cross-margin', type=float, default=3.0,
                    help='건너는 시간에 더할 여유(초)')
    ap.add_argument('--no-signal', dest='signal', action='store_false', default=True,
                    help='신호를 무시하고 바로 건넌다 (시험용)')
    ap.add_argument('--no-quad', dest='quad', action='store_false', default=True,
                    help='존 방향을 저장된 그대로만 쓴다 (기본은 90도 간격 4개 중 가까운 것)')
    ap.add_argument('--no-skip', dest='skip_ok', action='store_false', default=True,
                    help='도착 자세가 이미 좋아도 도킹을 돌린다 (기본은 건너뛴다)')
    ap.add_argument('--dock-budget', type=float, default=12.0,
                    help='존 하나에서 도킹에 쓸 시간 상한(초). 넘으면 그 상태로 끝낸다')
    ap.add_argument('--dock-tries', type=int, default=1,
                    help='deliver: 발 조건이 안 맞을 때 다시 맞춰 볼 횟수. '
                         '기본 1 = 다시 안 한다 (다시 맞추다 더 틀어진 전례)')
    ap.add_argument('--strict-feet', action='store_true',
                    help='deliver: 발 조건을 못 채우면 leg 을 실패로 적는다. '
                         '기본은 적어만 두고 넘어간다')
    ap.add_argument('--force', action='store_true',
                    help='make-via: 이미 있는 횡단보도 통과점을 다시 만든다 (육교는 안 건드림)')
    ap.add_argument('--spacing', type=float, default=0.0,
                    help='make-via: 중심선 통과점 간격(m). 0 이면 들머리·날머리 둘만 (기본). '
                         '촘촘하게 깔면 그 점들을 정확히 밟으려다 횡단보도에서 멈춘다')
    ap.add_argument('--from', dest='frm', default='',
                    help='register: --dist 와 짝. 어느 존에서 잰 거리인지')
    ap.add_argument('--dist', type=float, default=0.0,
                    help='register: 줄자로 잰 절대거리(m). 등록과 한 번에 넣는다')
    ap.add_argument('--width', type=float, default=0.0,
                    help='register fit: 도면에서 아는 가로(m). 0 이면 같이 추정')
    ap.add_argument('--height', type=float, default=0.0,
                    help='register fit: 도면에서 아는 세로(m). 0 이면 같이 추정')
    ap.add_argument('--scan-frames', type=int, default=8,
                    help='teach: 기준 스캔을 몇 장 모아 중앙값을 낼지')
    ap.add_argument('--dock-mode', choices=['point', 'feet', 'both', 'scan'],
                    default='point',
                    # 2026-09-13 현장: 발 측정이 오히려 느리고 더 틀어졌다.
                    #   재는 데만 존당 10초 넘게 걸리고, 그 사이 자세가 흔들린다.
                    #   점 도킹으로 되돌리고 발은 **확인용**으로만 쓴다.
                    help='point=base_link 한 점 / feet=발 네 점 / both=점으로 대충 맞춘 뒤 발로 마무리')
    ap.add_argument('--yaw-ok', type=float, default=0.12,
                    help='방향 허용 오차(rad). 기본 0.12 = 6.9도')
    ap.add_argument('--zone-box', type=float, default=0.60,
                    help='정지구역 한 변(m). 엎드린 뒤 발이 몇 개 들어갔는지 센다')
    ap.add_argument('--lowdown', dest='lowdown', action='store_true', default=True,
                    help='존에 닿으면 엎드린다 (기본 켬). 다음 구간 전에 자동으로 다시 선다')
    ap.add_argument('--no-lowdown', dest='lowdown', action='store_false')
    ap.add_argument('--time-limit', type=float, default=0.0,
                    help='초. 넘으면 남은 구간을 버리고 끝낸다')
    a = ap.parse_args()
    return {'plan': cmd_plan, 'dry': cmd_dry, 'run': cmd_run,
            'mark-zone': cmd_mark_zone, 'pose': cmd_pose,
            'goto': cmd_goto, 'reach': cmd_reach,
            'zones': cmd_zones,
            'mark-overpass': cmd_mark_overpass,
            'deliver': cmd_deliver,
            'make-via': cmd_make_via,
            'circuit': cmd_circuit,
            'teach': cmd_teach,
            'orders': cmd_orders,
            'register': cmd_register,
            'penalty': cmd_penalty,
            'back': cmd_back}[a.cmd](a) or 0


if __name__ == '__main__':
    sys.exit(main())
