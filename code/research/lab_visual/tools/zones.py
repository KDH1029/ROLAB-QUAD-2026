#!/usr/bin/env python3
"""횡단보도 같은 **구역을 네 모서리로** 저장하고, 발이 그 안에 있는지 본다.

  # 모서리 찍기 — 로봇을 모서리에 세우고 엔터. 네 번 반복한다
  python3 zones.py mark 횡단보도1

  # 저장된 구역 보기
  python3 zones.py list

  # 주행 중 감시 — 발 몇 개가 구역 안인지 계속 본다
  python3 zones.py watch 횡단보도1 --need 1

  # 한 번만 판정
  python3 zones.py check 횡단보도1

  # Nav2 keepout 마스크 만들기 — 이 구역 **밖으로는 경로가 안 나가게** 한다
  python3 zones.py mask 횡단보도1 --out ~/gosu/config/keepout

【왜 웨이포인트가 아니라 네 모서리인가】
웨이포인트는 점 하나라 "안에 있는가" 를 못 판정한다. 규정은 영역을 말한다:
  · 횡단보도 영역을 **네 발이 다 벗어나면** 페널티  -> 최소 한 발은 안에 있어야 한다
  · 진입 판정은 "로봇이 횡단보도 위에 100% 위치" 일 때다
그래서 꼭짓점 네 개로 다각형을 만든다. 횡단보도는 바닥에 그려진 것이라
로봇을 모서리마다 세워 찍는 게 가장 정확하다.

【왜 몸통이 아니라 발인가】
발 간격이 앞뒤 0.39 m, 좌우 0.28 m 다. 몸통 중심이 구역 안이어도 발은 밖일 수 있고,
그 반대도 된다. 규정이 발을 말하므로 발로 판정해야 한다.
관절각 12개로 네 발 위치를 풀고(foot_pose 와 같은 식), 접지한 발만 센다 —
공중에 뜬 발은 "딛고 있다" 고 볼 수 없다.

【좌표계】
`map` 프레임. waypoint.py 가 쓰는 것과 같다. 저장 시점의 위치추정이 흔들리면
좌표가 틀어지므로, 모서리를 찍을 때마다 그 순간의 자세를 같이 남긴다.
"""
import argparse
import json
import math
import os
import sys
import time

# Go2 링크 치수 (공개 사양, m) — tools/foot_pose.py 와 같은 값
L_HIP, L_THIGH, L_CALF = 0.0955, 0.213, 0.213
BX, BY = 0.1934, 0.0465
LEGS = ('FR', 'FL', 'RR', 'RL')
SGN = ((+1, -1), (+1, +1), (-1, -1), (-1, +1))

STORE = os.path.expanduser('~/gosu/config/zones_box.json')


def feet_in_body(q):
    """관절각 12개 -> 네 발의 몸통 기준 (x, y, z). FR,FL,RR,RL 순서."""
    out = []
    for i, (sx, sy) in enumerate(SGN):
        q0, q1, q2 = q[i * 3], q[i * 3 + 1], q[i * 3 + 2]
        lx = -L_THIGH * math.sin(q1) - L_CALF * math.sin(q1 + q2)
        lz = -(L_THIGH * math.cos(q1) + L_CALF * math.cos(q1 + q2))
        y = sy * L_HIP * math.cos(q0) - lz * math.sin(q0) * sy
        z = sy * L_HIP * math.sin(q0) * sy + lz * math.cos(q0)
        out.append((BX * sx + lx, BY * sy + y, z))
    return out


def inside(poly, x, y):
    """다각형 안인가 (ray casting). poly 는 [(x,y), …] 순서대로."""
    n = len(poly)
    c = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and \
           (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            c = not c
        j = i
    return c


def load():
    p = os.path.expanduser(STORE)
    return json.load(open(p)) if os.path.exists(p) else {}


def save(d):
    p = os.path.expanduser(STORE)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    json.dump(d, open(p, 'w'), ensure_ascii=False, indent=1)


def parse_args(ap):
    """ROS 인자를 걷어내고 읽는다. list 는 ROS 없이도 돌아야 하므로 rclpy 가 없으면 그냥 읽는다."""
    try:
        from rclpy.utilities import remove_ros_args
        return ap.parse_args(remove_ros_args(sys.argv)[1:])
    except ImportError:
        return ap.parse_args()


class Reader:
    """map 프레임의 로봇 자세와 관절각·접지력을 읽는다."""

    def __init__(self, node, tf_buf, a):
        self.n = node
        self.buf = tf_buf
        self.a = a
        self.q = None
        self.force = None
        from unitree_go.msg import LowState
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        node.create_subscription(LowState, a.lowstate_topic, self.on_low, qos)

    def on_low(self, m):
        self.q = [float(m.motor_state[i].q) for i in range(12)]
        self.force = [float(v) for v in m.foot_force]

    def pose_wait(self, node, wait_s=8.0, max_age=0.4):
        """TF 가 **새로 들어올 때까지** 기다렸다가 읽는다.

        【왜 신선도까지 보나 — 2026-09-13 현장】
        모서리를 찍을 때 엔터를 기다리는 동안(input) 노드가 안 돈다. 그동안 TF 버퍼는
        **걷기 전 값**에 멈춰 있다. 엔터를 치자마자 읽으면 그 낡은 값이 그대로 나온다 —
        실제로 로봇을 옮겼는데 두 모서리가 (1.296, -0.370) 으로 **똑같이** 찍혔다.
        `lookup_transform(..., Time())` 은 "버퍼에 있는 가장 최근"이지 "지금"이 아니다.

        그래서 변환의 **타임스탬프**를 보고, 지금보다 max_age 초 이상 낡았으면
        새 것이 올 때까지 계속 돈다. 그래야 정말로 지금 자리를 찍는다.
        """
        import rclpy
        end = time.time() + wait_s
        said = False
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            try:
                tr = self.buf.lookup_transform(self.a.map_frame, self.a.base_frame,
                                               rclpy.time.Time())
            except Exception:
                tr = None
            if tr is not None:
                now = node.get_clock().now().nanoseconds * 1e-9
                st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                age = now - st
                if age <= max_age:
                    q = tr.transform.rotation
                    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z))
                    return (tr.transform.translation.x, tr.transform.translation.y, yaw)
                if not said and time.time() > end - wait_s + 1.0:
                    said = True
                    print('     새 TF 를 기다리는 중… (지금 것은 %.1f초 전 값)' % age)
        return None

    def pose(self):
        """map -> base_link. 못 읽으면 None."""
        import rclpy
        try:
            t = self.buf.lookup_transform(self.a.map_frame, self.a.base_frame,
                                          rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def feet_map(self):
        """접지한 발의 map 좌표 목록 [(이름, x, y, 접지력)]. 관절각이 없으면 None."""
        p = self.pose()
        if p is None or self.q is None:
            return None
        x, y, yaw = p
        c, s = math.cos(yaw), math.sin(yaw)
        out = []
        for i, (fx, fy, _) in enumerate(feet_in_body(self.q)):
            f = self.force[i] if self.force else 0.0
            out.append((LEGS[i], x + c * fx - s * fy, y + s * fx + c * fy, f))
        return out


def cmd_mark(a):
    import rclpy
    import tf2_ros
    from rclpy.node import Node
    rclpy.init(args=sys.argv)
    n = Node('zone_mark')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    r = Reader(n, buf, a)
    for _ in range(40):
        rclpy.spin_once(n, timeout_sec=0.2)

    print('구역 "%s" 의 모서리 %d개를 찍는다.' % (a.name, a.corners))
    print('로봇을 모서리에 세우고 엔터. 취소는 Ctrl-C.')
    print('★ 순서대로 돌아가며 찍어야 한다 (시계 또는 반시계). 대각선으로 찍으면 안 된다.')
    pts = []
    i = 0
    while i < a.corners:
        input('  %d/%d 번째 모서리에 세우고 엔터> ' % (i + 1, a.corners))
        p = r.pose_wait(n)
        if p is None:
            # 넘어가지 않는다 — 넘어가면 찍은 것까지 버려진다(2026-09-13 에 그랬다).
            print('  ★ map -> base_link 를 8초 동안 못 받았다.')
            print('     위치추정이 돌고 있는지 보고(~/gosu/gosu.sh status),')
            print('     같은 모서리에서 엔터를 다시 치면 재시도한다. 그만두려면 Ctrl-C.')
            continue
        # 직전 모서리와 얼마나 떨어졌는지 **바로** 보여준다.
        # 네 개 다 찍고 나서야 망한 걸 아는 일이 없게 한다(2026-09-13 에 그랬다).
        if pts:
            dprev = math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1])
            if dprev < 0.05:
                print('     ★ 직전 모서리와 %.0f mm 밖에 안 떨어졌다.' % (dprev * 1000))
                print('        로봇이 실제로 움직였는지, 좌표가 갱신되는지 확인할 것.')
                print('        (~/gosu/gosu.sh pose 로 지금 좌표를 볼 수 있다)')
                print('        이 점을 버리고 다시 찍으려면 그냥 엔터를 다시 치면 된다.')
                ans = input('     그래도 이 점을 쓰겠습니까? (y 를 치면 씀) > ').strip().lower()
                if ans != 'y':
                    continue
            print('     찍음: (%.3f, %.3f)  방향 %.1f도   직전에서 %.2f m'
                  % (p[0], p[1], math.degrees(p[2]), dprev))
        else:
            print('     찍음: (%.3f, %.3f)  방향 %.1f도' % (p[0], p[1], math.degrees(p[2])))
        pts.append([round(p[0], 4), round(p[1], 4)])
        i += 1

    if len(pts) < 3:
        print('★ 모서리가 %d개뿐이라 저장하지 않는다' % len(pts))
    else:
        # 저장 전 마지막 검사 — 망가진 상자를 파일에 남기지 않는다
        n_ = len(pts)
        edges = [math.hypot(pts[(k + 1) % n_][0] - pts[k][0],
                            pts[(k + 1) % n_][1] - pts[k][1]) for k in range(n_)]
        print()
        print('  변 길이: %s' % '  '.join('%.2f m' % e for e in edges))
        if min(edges) < 0.05:
            print('★ 붙어 있는 변이 있다 (%.0f mm) — 저장하지 않는다. 다시 찍을 것.'
                  % (min(edges) * 1000))
            n.destroy_node(); rclpy.try_shutdown(); return
        # 넓이로 순서가 꼬였는지 본다. 자기교차한 다각형은 넓이가 비정상적으로 작다.
        area = abs(sum(pts[i][0] * pts[(i + 1) % len(pts)][1] -
                       pts[(i + 1) % len(pts)][0] * pts[i][1]
                       for i in range(len(pts)))) / 2.0
        d = load()
        d[a.name] = {'frame': a.map_frame, 'corners': pts, 'area': round(area, 3),
                     'saved_at': time.strftime('%Y-%m-%d %H:%M:%S')}
        save(d)
        print()
        print('저장: %s  넓이 %.2f m²' % (STORE, area))
        if area < 0.2:
            print('★ 넓이가 너무 작다 — 모서리 순서가 꼬였을 수 있다. 다시 찍을 것')
        else:
            print('  통과점을 다시 만들 것:')
            print('    python3 ~/gosu/lab_visual/tools/mission.py '
                  'make-via --spacing 1.0 --force')
    n.destroy_node()
    rclpy.try_shutdown()



def cmd_set(a):
    """모서리 좌표를 **직접** 넣어 저장한다. 유도도 추정도 없다.

      python3 zones.py set 2-3 1.03,0.13 6.11,-0.47 6.05,-0.97 0.97,-0.36

    순서대로 돌아가야 한다(시계 또는 반시계). 넓이로 검사한다 —
    대각선으로 찍으면 자기교차 다각형이 되어 넓이가 비정상적으로 작게 나온다.
    """
    if not a.name:
        raise SystemExit('★ 구역 이름이 필요하다 (예: 2-3)')
    if len(a.corner) < 3:
        raise SystemExit('★ 모서리를 3개 이상 줄 것.  예: set 2-3 1.03,0.13 6.11,-0.47 ...')
    pts = []
    for t in a.corner:
        try:
            x, y = (float(v) for v in t.replace('(', '').replace(')', '').split(','))
        except Exception:
            raise SystemExit('★ 모서리는 "x,y" 형식이다: %s' % t)
        pts.append([round(x, 4), round(y, 4)])
    n = len(pts)
    area = abs(sum(pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1]
                   for i in range(n))) / 2.0
    print()
    print('  구역 %s — 모서리 %d개' % (a.name, n))
    for i, (x, y) in enumerate(pts):
        nx, ny = pts[(i + 1) % n]
        print('    %d  x %7.3f  y %7.3f     다음까지 %.2f m'
              % (i + 1, x, y, math.hypot(nx - x, ny - y)))
    print('    넓이 %.2f m²' % area)
    if area < 0.2:
        raise SystemExit('★ 넓이가 %.2f m² 뿐이다 — 모서리 순서가 꼬였거나 같은 점을 줬다.\n'
                         '   시계/반시계로 돌아가며 줄 것. 저장하지 않았다.' % area)
    ok = [i for i in range(n) if math.hypot(pts[(i + 1) % n][0] - pts[i][0],
                                            pts[(i + 1) % n][1] - pts[i][1]) < 0.05]
    if ok:
        raise SystemExit('★ 붙어 있는 모서리가 있다 (%s번째) — 같은 점을 두 번 준 것 같다.\n'
                         '   저장하지 않았다.' % ', '.join(str(i + 1) for i in ok))
    d = load()
    prev = d.get(a.name)
    d[a.name] = {'frame': a.map_frame, 'corners': pts, 'area': round(area, 3),
                 'from': '좌표 직접 입력',
                 'saved_at': time.strftime('%Y-%m-%d %H:%M:%S')}
    save(d)
    print()
    print('  저장 %s%s' % (STORE, '  (이전 값 덮어씀, 넓이 %.2f m²)'
                           % prev.get('area', 0) if prev else ''))
    print('  통과점을 다시 만들 것:')
    print('    python3 ~/gosu/lab_visual/tools/mission.py make-via --spacing 1.0 --force')
    return 0


def cmd_from_zones(a):
    """존 네 곳 좌표에서 횡단보도 사각형 넷을 만들어 낸다.

    【왜】 모서리를 직접 찍으면 횡단보도 4곳 x 모서리 4개 = **16번** 로봇을 옮겨야 한다.
    경기장이 사각형이고 존이 네 모서리에 있으므로, 존을 잇는 변이 곧 횡단보도다.
    폭만 주면 사각형이 나온다. 값이 안 맞으면 그 구역만 mark 로 다시 찍으면 된다.

    만드는 것: 1-2, 2-3, 3-4, 4-1 네 개. 이름은 signals.py 와 같은 표기를 쓴다.
    """
    import json as _j
    V = os.path.expanduser('~/gosu/config/venue.json')
    if not os.path.exists(V):
        raise SystemExit('★ venue.json 이 없다')
    zp = (_j.load(open(V, encoding='utf-8')).get('zone_poses') or {})
    miss = [z for z in '1234' if z not in zp]
    if miss:
        raise SystemExit('★ 존 %s 이 아직 안 찍혔다. ~/gosu/gosu.sh zone <번호>'
                         % ', '.join(miss))
    half = a.width / 2.0
    trim = a.trim          # 존 쪽 끝을 이만큼 잘라낸다(존 자체는 횡단보도가 아니다)
    # 육교로 다니는 구간만 건너뛴다.
    # ★ 2026-09-13 정정: 예전엔 leg_via 에 **있기만 하면** 건너뛰었다. 그런데
    #   횡단보도도 통과점이 생기면 leg_via 에 들어가므로, 한 번 만든 뒤에는
    #   전부 건너뛰어 다시 만들 수 없게 됐다. 통과점 이름이 cw_ 로 시작하면
    #   그건 횡단보도이므로 다시 만들어도 된다. 육교만 지킨다.
    via = _j.load(open(V, encoding='utf-8')).get('leg_via') or {}
    skip = set()
    for k, v in via.items():
        if all(str(x).startswith('cw_') for x in (v or [])):
            continue                      # 횡단보도 — 다시 만들어도 된다
        x_, y_ = k.split('-')
        skip.add((x_, y_)); skip.add((y_, x_))
    d = load()
    made = []
    for p_, q_ in (('1', '2'), ('2', '3'), ('3', '4'), ('4', '1')):
        if (p_, q_) in skip:
            print('  %s-%s 는 육교로 다닌다 — 횡단보도로 만들지 않는다' % (p_, q_))
            d.pop('%s-%s' % (p_, q_), None)
            continue
        ax, ay = zp[p_]['x'], zp[p_]['y']
        bx, by = zp[q_]['x'], zp[q_]['y']
        dx, dy = bx - ax, by - ay
        L = math.hypot(dx, dy)
        if L < 0.5:
            print('★ %s-%s 가 %.2f m 밖에 안 떨어져 있다 — 건너뛴다' % (p_, q_, L))
            continue
        ux, uy = dx / L, dy / L          # 진행 방향
        nx, ny = -uy, ux                 # 직각 방향
        t = min(trim, L / 2 - 0.2)
        a0x, a0y = ax + ux * t, ay + uy * t
        b0x, b0y = bx - ux * t, by - uy * t
        pts = [[round(a0x + nx * half, 4), round(a0y + ny * half, 4)],
               [round(b0x + nx * half, 4), round(b0y + ny * half, 4)],
               [round(b0x - nx * half, 4), round(b0y - ny * half, 4)],
               [round(a0x - nx * half, 4), round(a0y - ny * half, 4)]]
        area = abs(sum(pts[i][0] * pts[(i + 1) % 4][1] -
                       pts[(i + 1) % 4][0] * pts[i][1] for i in range(4))) / 2.0
        name = '%s-%s' % (p_, q_)
        d[name] = {'frame': a.map_frame, 'corners': pts, 'area': round(area, 3),
                   'from': '존 %s,%s 에서 유도 (폭 %.2fm, 양끝 %.2fm 잘라냄)'
                           % (p_, q_, a.width, t),
                   'saved_at': time.strftime('%Y-%m-%d %H:%M:%S')}
        made.append((name, L - 2 * t, area))
    if not made:
        raise SystemExit('★ 만들어진 구역이 없다')
    save(d)
    print()
    print('  구역     길이     넓이')
    for nm, L, ar in made:
        print('  %-6s %6.2f m %7.2f m²' % (nm, L, ar))
    print()
    print('  저장 %s' % STORE)
    print('  확인:  ~/gosu/gosu.sh zones')
    print('  값이 안 맞으면 그 구역만 다시:  ~/gosu/gosu.sh mark 1-2')
    return 0


def judge(r, poly, need):
    """돌려주는 것: (안에 있는 접지발 수, 접지발 수, 줄글)"""
    feet = r.feet_map()
    if feet is None:
        return None, None, 'TF 나 관절각을 못 읽었다'
    on = [f for f in feet if f[3] >= 20.0]          # 접지력 20 이상이면 딛고 있다고 본다
    ins = [f for f in on if inside(poly, f[1], f[2])]
    txt = ' '.join('%s%s' % (f[0], '○' if inside(poly, f[1], f[2]) else '×') for f in on)
    return len(ins), len(on), txt


def cmd_watch(a, once=False):
    import rclpy
    import tf2_ros
    from rclpy.node import Node
    d = load()
    if a.name not in d:
        raise SystemExit('★ 그런 구역이 없다: %s\n   있는 것: %s' % (a.name, ', '.join(d) or '(없음)'))
    poly = [tuple(p) for p in d[a.name]['corners']]
    rclpy.init(args=sys.argv)
    n = Node('zone_watch')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    r = Reader(n, buf, a)
    pub = None
    if not once:
        from std_msgs.msg import String
        pub = n.create_publisher(String, 'zone/status', 10)
    for _ in range(40):
        rclpy.spin_once(n, timeout_sec=0.2)

    print('구역 "%s" (모서리 %d개, 넓이 %.2f m²)  최소 %d발이 안에 있어야 한다'
          % (a.name, len(poly), d[a.name].get('area', 0), a.need))
    bad = 0
    try:
        while rclpy.ok():
            for _ in range(5):
                rclpy.spin_once(n, timeout_sec=0.1)
            ins, on, txt = judge(r, poly, a.need)
            if ins is None:
                print('  %s' % txt)
            else:
                ok = ins >= a.need
                bad = 0 if ok else bad + 1
                mark = '   ' if ok else '★★ '
                print('  %s안에 있는 발 %d / 접지 %d   %s%s'
                      % (mark, ins, on, txt, '' if ok else '  ← 이탈'))
                if pub:
                    from std_msgs.msg import String
                    pub.publish(String(data='ok=%d in=%d on=%d %s'
                                       % (1 if ok else 0, ins, on, txt)))
            if once:
                break
            time.sleep(max(0.0, 1.0 / a.hz - 0.5))
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


def cmd_list(a):
    d = load()
    if not d:
        print('저장된 구역이 없다: %s' % STORE)
        return
    print('%s  %d개' % (STORE, len(d)))
    for k, v in d.items():
        c = v.get('corners', [])
        xs = [p[0] for p in c]
        ys = [p[1] for p in c]
        print('  %-12s 모서리 %d  넓이 %5.2f m²  x %6.2f~%6.2f  y %6.2f~%6.2f  %s'
              % (k, len(c), v.get('area', 0), min(xs), max(xs), min(ys), max(ys),
                 v.get('saved_at', '')))


def cmd_mask(a):
    """Nav2 KeepoutFilter 용 마스크를 만든다 — 구역 **밖**을 금지구역으로 칠한다.

    규정: 마네킹을 피하더라도 횡단보도를 벗어나면 안 된다 (최소 한 발은 안).
    그걸 계획 단계에서 보장하는 공식 방법이 KeepoutFilter 다.
    마스크에서 흰색(자유)은 구역 안, 검정(금지)은 구역 밖이다.
    Nav2 는 금지구역으로 경로를 내지 않으므로, 비켜 가더라도 횡단보도 안에서만 비킨다.

    【왜 로봇 중심 기준으로 충분한가】
    발은 중심에서 앞뒤 0.193 m, 좌우 0.142 m 에 있다. 중심이 구역 안이면
    구역이 발 간격(0.39 x 0.28 m)보다 넓은 한 최소 한 발은 안에 있다.
    횡단보도가 그보다 좁으면 애초에 규정을 만족하며 지나갈 수 없다 — 그건 마스크가 아니라
    경로 설계 문제다.
    """
    import numpy as np
    d = load()
    names = [a.name] if a.name else list(d)
    if not names or any(n not in d for n in names):
        raise SystemExit('★ 구역이 없다. 있는 것: %s' % (', '.join(d) or '(없음)'))
    polys = [[tuple(p) for p in d[n]['corners']] for n in names]
    allp = [p for poly in polys for p in poly]
    xmin = min(p[0] for p in allp) - a.pad
    xmax = max(p[0] for p in allp) + a.pad
    ymin = min(p[1] for p in allp) - a.pad
    ymax = max(p[1] for p in allp) + a.pad
    W = int(math.ceil((xmax - xmin) / a.res))
    H = int(math.ceil((ymax - ymin) / a.res))
    if W * H > 40_000_000:
        raise SystemExit('★ 마스크가 너무 크다 (%dx%d). --res 를 키울 것' % (W, H))

    img = np.zeros((H, W), np.uint8)          # 0 = 검정 = 금지
    ys, xs = np.mgrid[0:H, 0:W]
    px = xmin + (xs + 0.5) * a.res
    py = ymin + (ys + 0.5) * a.res
    for poly in polys:                        # 구역 안만 흰색(자유)으로
        m = np.zeros((H, W), bool)
        n = len(poly)
        j = n - 1
        for i in range(n):
            xi, yi = poly[i]
            xj, yj = poly[j]
            cond = ((yi > py) != (yj > py)) & \
                   (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi)
            m ^= cond
            j = i
        img[m] = 254                          # 254 = 흰색 = 자유
    free = int((img > 0).sum())
    out = os.path.expanduser(a.out)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    # PGM 은 위에서 아래로 쓰므로 y 를 뒤집는다
    with open(out + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (W, H))
        f.write(np.flipud(img).tobytes())
    yaml = ('image: %s.pgm\n'
            'mode: scale\n'
            'resolution: %.4f\n'
            'origin: [%.4f, %.4f, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.196\n'
            % (os.path.basename(out), a.res, xmin, ymin))
    open(out + '.yaml', 'w').write(yaml)
    print('구역 %s -> keepout 마스크' % ', '.join(names))
    print('  %s.pgm  %dx%d  격자 %.3f m  원점 (%.3f, %.3f)' % (out, W, H, a.res, xmin, ymin))
    print('  자유(구역 안) %.2f m² / 전체 %.2f m²' % (free * a.res ** 2, W * H * a.res ** 2))
    print()
    print('쓰는 법 — Nav2 에 공식 필터로 붙인다 (커스텀 노드 없음):')
    print('  1) 마스크 서버와 필터정보 서버를 띄운다')
    print('     ros2 run nav2_map_server map_server --ros-args \\')
    print('       -p yaml_filename:=%s.yaml -r __node:=filter_mask_server \\' % out)
    print('       -r map:=/keepout_filter_mask -p topic_name:=/keepout_filter_mask')
    print('     ros2 run nav2_map_server costmap_filter_info_server --ros-args \\')
    print('       -p type:=0 -p filter_info_topic:=/costmap_filter_info \\')
    print('       -p mask_topic:=/keepout_filter_mask -p base:=0.0 -p multiplier:=1.0')
    print('     (둘 다 lifecycle 노드라 configure/activate 가 필요하다)')
    print('  2) nav2 파라미터의 코스트맵에 필터를 더한다')
    print('     global_costmap: filters: ["keepout_filter"]')
    print('     keepout_filter: {plugin: "nav2_costmap_2d::KeepoutFilter",')
    print('                      enabled: true, filter_info_topic: "/costmap_filter_info"}')
    print()
    print('  ★ 이걸 켜면 Nav2 가 구역 **밖으로는 경로를 내지 않는다.**')
    print('    마네킹을 피하더라도 횡단보도 안에서만 비켜 간다.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['mark', 'list', 'watch', 'check', 'mask',
                                   'from-zones', 'set'])
    ap.add_argument('name', nargs='?', default='')
    ap.add_argument('corner', nargs='*', default=[],
                    help='set: 모서리 좌표 "x,y" 를 순서대로')
    ap.add_argument('--corners', type=int, default=4)
    ap.add_argument('--width', type=float, default=2.0,
                    help='from-zones: 횡단보도 폭(m)')
    ap.add_argument('--trim', type=float, default=1.0,
                    help='from-zones: 존 쪽 끝을 이만큼 잘라낸다(m)')
    ap.add_argument('--need', type=int, default=1,
                    help='구역 안에 있어야 하는 최소 발 개수 (횡단보도는 1)')
    ap.add_argument('--hz', type=float, default=2.0)
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='base_link')
    ap.add_argument('--lowstate-topic', default='/lowstate')
    ap.add_argument('--out', default='~/gosu/config/keepout', help='mask: 만들 파일 이름(확장자 뺀 것)')
    ap.add_argument('--res', type=float, default=0.05, help='mask: 격자 크기 (m)')
    ap.add_argument('--pad', type=float, default=1.5, help='mask: 구역 바깥으로 이만큼 더 그린다 (m)')
    a = parse_args(ap)
    if a.cmd in ('mark', 'watch', 'check') and not a.name:
        raise SystemExit('★ 구역 이름이 필요하다')
    if a.cmd == 'mark':
        cmd_mark(a)
    elif a.cmd == 'list':
        cmd_list(a)
    elif a.cmd == 'mask':
        cmd_mask(a)
    elif a.cmd == 'from-zones':
        cmd_from_zones(a)
    elif a.cmd == 'set':
        cmd_set(a)
    else:
        cmd_watch(a, once=(a.cmd == 'check'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
