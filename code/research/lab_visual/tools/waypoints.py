#!/usr/bin/env python3
"""waypoint.py 로 저장한 좌표를 gosu 스택에서 쓴다.

  python3 tools/waypoints.py list                      전부 보기 + 장애물 여유 검사
  python3 tools/waypoints.py list --today              오늘 찍은 것만
  python3 tools/waypoints.py go Z1_1                   그 자리로 /goal_pose 발행
  python3 tools/waypoints.py routes --out routes.json  미션 경로 형식으로 내보내기
  python3 tools/waypoints.py through Z1_1 Z1_2 Z2_1     경유점을 **안 멈추고** 관통 (먼저 계획만 보여줌)
  python3 tools/waypoints.py through Z1_1 Z1_2 --go     실제로 보낸다

【왜 관통인가 — 시간이 점수다】
목표를 하나씩 보내면(`go`) 경유점마다 감속·정지·목표판정·재가속에 2~4초,
다음 명령을 사람이 치는 데 5~15초가 더 든다. 경유점 7개면 50~130초를 서서 버린다.
`navigate_through_poses` 는 Nav2 **공식** 액션이고, 지금 스택의 bt_navigator 가
이미 물고 있다(`navigators` 키가 없어 기본값 사용). 스택은 한 줄도 안 고친다.
중간점은 지나가기만 하고, 도착 판정은 마지막 점에서만 한다.

【신호등에서는 끊는다】
한 번에 다 보내면 신호등 앞에서 못 선다. 신호등 사이 구간마다 한 번씩 보낼 것.

【어디서 읽나】
  ~/gosu/config/waypoints.json  (없으면 ~/ws/go2_slam/config/waypoints.json)
  waypoint.py 가 쓰는 그 파일이다. 형식:
    {"이름": {"x":…, "y":…, "yaw":…, "qz":…, "qw":…, "frame":"map",
              "saved_at":"…", "match_ratio":0.82}, …}

【좌표계】
저장할 때 AMCL 의 2D `map` 프레임을 썼는데, 그 2D 지도는 같은 3D 사전지도에서 나왔다.
오늘 찍은 17개가 전부 3D 지도의 경기장 안(x -0.9~8.8, y -6.0~2.0)에 들어오는 것으로
**같은 좌표계임을 확인했다**(2026-09-12). 그래서 그대로 쓸 수 있다.

【반드시 보는 두 가지】
  · 장애물 여유 — 계획기의 safe_dist 보다 좁으면 그 목표는 **도달 불가**다.
    횡단보도 진입점은 벽에 붙어 있어서 특히 위험하다.
  · match_ratio — 저장 시점의 스캔-지도 일치율. 낮으면 그 좌표 자체를 못 믿는다.
"""
import argparse
import json
import math
import os
import struct
import sys

import numpy as np

DEFAULT_WP = ['~/gosu/config/waypoints.json', '~/ws/go2_slam/config/waypoints.json']
DEFAULT_LAYER = ['~/gosu/lab_visual/maps/room_obstacles_map.bin',
                 '~/ws/lab_visual/maps/room_obstacles_map.bin']
RES = 0.10


def first_existing(paths):
    for p in paths:
        q = os.path.expanduser(p)
        if os.path.exists(q):
            return q
    return None


def load_layer(path):
    raw = open(path, 'rb').read()
    n = struct.unpack('<I', raw[:4])[0]
    P = np.frombuffer(raw, '<f4', 2 * n, 4).reshape(-1, 2).astype(np.float64)
    return set(map(tuple, np.floor(P / RES).astype(np.int64)))


def clearance(occ, x, y, rmax=1.5):
    """가장 가까운 장애물까지 거리. 격자 해상도라 RES 단위로 나온다."""
    ix, iy = int(math.floor(x / RES)), int(math.floor(y / RES))
    for k in range(0, int(rmax / RES) + 1):
        for dx in range(-k, k + 1):
            for dy in range(-k, k + 1):
                if max(abs(dx), abs(dy)) != k:
                    continue
                if (ix + dx, iy + dy) in occ:
                    return k * RES
    return rmax


def load_wp(path, today_only=False):
    d = json.load(open(path))
    rows = [(k, v) for k, v in d.items() if isinstance(v, dict) and 'x' in v]
    if today_only:
        import time
        t = time.strftime('%Y-%m-%d')
        rows = [(k, v) for k, v in rows if v.get('saved_at', '').startswith(t)]
    rows.sort(key=lambda kv: kv[1].get('saved_at', ''))
    return rows


def cmd_list(a, rows, occ):
    print('%-10s %8s %8s %8s %9s %8s  %s' %
          ('이름', 'x', 'y', 'yaw°', '장애물까지', '일치율', '판정'))
    nbad = 0
    for k, v in rows:
        c = clearance(occ, v['x'], v['y']) if occ else float('nan')
        mr = v.get('match_ratio', float('nan'))
        why = []
        if occ and c < a.safe_dist:
            why.append('★ 도달불가 (safe_dist %.2f 미만)' % a.safe_dist)
            nbad += 1
        elif occ and c < a.safe_dist + 0.15:
            why.append('빠듯함')
        if mr == mr and mr < a.min_match:
            why.append('★ 저장 시 일치율 낮음')
        print('%-10s %8.3f %8.3f %8.1f %8.2fm %8.3f  %s'
              % (k, v['x'], v['y'], math.degrees(v.get('yaw', 0.0)), c, mr,
                 ' / '.join(why) if why else '여유'))
    if occ:
        print()
        print('  도달 불가 %d개 / %d개.' % (nbad, len(rows)))
        if nbad:
            print('  ★ 계획기가 그 자리를 막힌 곳으로 본다. 셋 중 하나를 해야 한다:')
            print('     · 웨이포인트를 벽에서 조금 떼어 다시 찍는다  (가장 안전)')
            print('     · safe_dist 를 더 낮춘다  (몸통 반폭 0.155 m 가 하한. 회전 여유가 사라진다)')
            print('     · 그 목표만 사전 장애물 층 없이 간다 (use_prior_map:=false)')


def cmd_go(a, rows):
    sel = [v for k, v in rows if k == a.name]
    if not sel:
        raise SystemExit('★ 그런 이름이 없다: %s\n   있는 것: %s'
                         % (a.name, ', '.join(k for k, _ in rows)))
    v = sel[0]
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    rclpy.init(args=sys.argv)
    n = Node('waypoint_sender')
    pub = n.create_publisher(PoseStamped, a.goal_topic, 10)
    m = PoseStamped()
    m.header.frame_id = a.frame
    m.pose.position.x = float(v['x'])
    m.pose.position.y = float(v['y'])
    if 'qz' in v and 'qw' in v:
        m.pose.orientation.z = float(v['qz'])
        m.pose.orientation.w = float(v['qw'])
    else:
        y = float(v.get('yaw', 0.0))
        m.pose.orientation.z = math.sin(y / 2)
        m.pose.orientation.w = math.cos(y / 2)
    import time
    for _ in range(3):
        m.header.stamp = n.get_clock().now().to_msg()
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.2)
        time.sleep(0.3)
    print('  %s -> %s  (%.3f, %.3f, %.1f°)  3번 발행'
          % (a.name, a.goal_topic, v['x'], v['y'], math.degrees(v.get('yaw', 0.0))))
    n.destroy_node()
    rclpy.try_shutdown()


def cmd_through(a, rows):
    """경유점을 한 번에 보내 **중간에 멈추지 않고** 지나간다.

    Nav2 공식 액션 `navigate_through_poses` 를 그대로 쓴다. 새 노드도 새 토픽도 없다.
    액션 서버는 bt_navigator 가 띄운다 — 주행 스택이 올라와 있으면 이미 있다.
    """
    import time
    want = list(a.name)
    have = dict(rows)
    miss = [w for w in want if w not in have]
    if miss:
        raise SystemExit('★ 그런 이름이 없다: %s\n   있는 것: %s'
                         % (', '.join(miss), ', '.join(k for k, _ in rows)))
    pts = [(w, have[w]) for w in want]

    # 계획을 먼저 보여준다. 직선 거리라 실제 경로보다 짧게 나온다(하한).
    total = 0.0
    print()
    print('  순서   이름          x       y      yaw     구간거리  일치율')
    px = py = None
    for i, (nm, v) in enumerate(pts):
        d = 0.0 if px is None else math.hypot(v['x'] - px, v['y'] - py)
        total += d
        mr = v.get('match_ratio')
        print('  %4d   %-12s %6.2f  %6.2f  %6.1f°  %7.2fm  %s'
              % (i + 1, nm, v['x'], v['y'], math.degrees(v.get('yaw', 0.0)), d,
                 ('%.2f' % mr) if mr is not None else '-'))
        px, py = v['x'], v['y']
    v_nom = a.speed
    print()
    print('  직선 합계 %.1f m — %.2f m/s 로 하한 %.0f초 (실제 경로는 이보다 길다)'
          % (total, v_nom, total / v_nom if v_nom > 0 else 0))
    low = [nm for nm, v in pts
           if v.get('match_ratio') is not None and v['match_ratio'] < a.min_match]
    if low:
        print('  ★ 저장 시 일치율이 낮은 점: %s — 좌표 자체를 의심할 것' % ', '.join(low))
    if not a.go:
        print()
        print('  계획만 보여줬다. 실제로 보내려면 뒤에 --go 를 붙일 것')
        return

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateThroughPoses

    rclpy.init(args=sys.argv)
    n = Node('waypoint_sender')
    ac = ActionClient(n, NavigateThroughPoses, 'navigate_through_poses')
    print()
    if not ac.wait_for_server(timeout_sec=10.0):
        n.destroy_node()
        rclpy.try_shutdown()
        raise SystemExit('★ navigate_through_poses 액션 서버가 없다.\n'
                         '   주행 스택(bt_navigator)이 떠 있는지 볼 것: ./gosu.sh status')

    g = NavigateThroughPoses.Goal()
    for nm, v in pts:
        m = PoseStamped()
        m.header.frame_id = a.frame
        m.header.stamp = n.get_clock().now().to_msg()
        m.pose.position.x = float(v['x'])
        m.pose.position.y = float(v['y'])
        if 'qz' in v and 'qw' in v:
            m.pose.orientation.z = float(v['qz'])
            m.pose.orientation.w = float(v['qw'])
        else:
            y = float(v.get('yaw', 0.0))
            m.pose.orientation.z = math.sin(y / 2)
            m.pose.orientation.w = math.cos(y / 2)
        g.poses.append(m)

    t0 = time.time()
    state = {'h': None}

    def on_fb(fb):
        f = fb.feedback
        print('\r  [+%5.1f초] 남은거리 %5.2fm  남은점 %d  회복동작 %d      '
              % (time.time() - t0, f.distance_remaining,
                 f.number_of_poses_remaining, f.number_of_recoveries),
              end='', flush=True)

    fut = ac.send_goal_async(g, feedback_callback=on_fb)

    def cancel_now():
        # Ctrl-C 가 **수락 직후·핸들 수신 전**에 오면 로봇은 이미 걷는데
        # 취소할 대상이 없다. 그때를 위해 핸들을 잠깐 더 기다렸다가 취소한다.
        h = state['h']
        if h is None and fut.done():
            h = fut.result()
        if h is None:
            rclpy.spin_until_future_complete(n, fut, timeout_sec=3.0)
            h = fut.result()
        if h is not None and getattr(h, 'accepted', False):
            print('  중단 요청 — 목표를 취소한다')
            c = h.cancel_goal_async()
            rclpy.spin_until_future_complete(n, c, timeout_sec=5.0)
            return True
        return False

    try:
        rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise SystemExit('\n★ 목표를 거절당했다. Nav2 상태를 볼 것')
        state['h'] = gh
        print('  %d개 경유점 전송 — 중간에서 멈추지 않는다. Ctrl-C 로 중단' % len(pts))
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(n, res)
        code = res.result().status if res.result() else -1
        print()
        # 4 = SUCCEEDED (action_msgs/GoalStatus)
        print('  %s  총 %.1f초' % ('도착' if code == 4 else '실패(status %d)' % code,
                                   time.time() - t0))
    except KeyboardInterrupt:
        print()
        try:
            ok = cancel_now()
        except Exception as e:
            ok = False
            print('  ★ 취소 중 오류: %s' % e)
        if ok:
            print('  취소했다.')
        else:
            print('  ★ 취소할 목표를 못 찾았다. 로봇이 걷고 있으면 바로:')
            print('       ~/gosu/gosu.sh stop')
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


def cmd_routes(a, rows):
    out = {'frame': 'map',
           'note': 'waypoints.json 에서 변환. 순서는 저장 시각 순.',
           'points': [{'name': k, 'x': v['x'], 'y': v['y'], 'yaw': v.get('yaw', 0.0),
                       'match_ratio': v.get('match_ratio')} for k, v in rows]}
    p = os.path.expanduser(a.out)
    os.makedirs(os.path.dirname(os.path.abspath(p)), exist_ok=True)
    json.dump(out, open(p, 'w'), ensure_ascii=False, indent=1)
    print('  %d개 -> %s' % (len(rows), p))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['list', 'go', 'routes', 'through'])
    ap.add_argument('name', nargs='*', default=[])
    ap.add_argument('--file', default='')
    ap.add_argument('--layer', default='')
    ap.add_argument('--today', action='store_true')
    ap.add_argument('--safe-dist', type=float, default=0.25,
                    help='계획기의 safe_dist. 이보다 여유가 적으면 도달 불가로 본다')
    ap.add_argument('--min-match', type=float, default=0.70,
                    help='저장 시 일치율이 이보다 낮으면 좌표를 의심한다')
    ap.add_argument('--goal-topic', default='/goal_pose')
    ap.add_argument('--frame', default='map')
    ap.add_argument('--out', default='~/gosu/config/routes.json')
    ap.add_argument('--go', action='store_true',
                    help='through: 계획만 보지 않고 실제로 보낸다')
    ap.add_argument('--speed', type=float, default=0.90,
                    help='through: 시간 하한을 계산할 때 쓰는 속도 (nav2 desired_linear_vel)')
    a = ap.parse_args()

    wp = os.path.expanduser(a.file) if a.file else first_existing(DEFAULT_WP)
    if not wp:
        raise SystemExit('★ waypoints.json 을 못 찾았다. --file 로 줄 것')
    rows = load_wp(wp, a.today)
    if not rows:
        raise SystemExit('★ 웨이포인트가 없다: %s' % wp)
    print('%s  %d개%s' % (wp, len(rows), ' (오늘 것만)' if a.today else ''))

    if a.cmd == 'list':
        lp = os.path.expanduser(a.layer) if a.layer else first_existing(DEFAULT_LAYER)
        occ = load_layer(lp) if lp else None
        if lp:
            print('장애물 층 %s' % lp)
        else:
            print('★ 장애물 층이 없다 — 여유 검사를 건너뛴다')
        print()
        cmd_list(a, rows, occ)
    elif a.cmd == 'go':
        if not a.name:
            raise SystemExit('★ 웨이포인트 이름이 필요하다')
        a.name = a.name[0]
        cmd_go(a, rows)
    elif a.cmd == 'through':
        if len(a.name) < 2:
            raise SystemExit('★ 경유점을 두 개 이상 줄 것 (하나면 go 를 쓴다)')
        cmd_through(a, rows)
    else:
        cmd_routes(a, rows)
    return 0


if __name__ == '__main__':
    sys.exit(main())
