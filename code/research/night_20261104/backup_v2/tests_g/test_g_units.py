#!/usr/bin/env python3
"""[G] 단위 시험 (F 의 test_f_units.py + G 바뀐 기대값 3곳 + G 새 항목) — 합성 입력, 몇 초.
  F 기대값에서 설계상 바뀐 것(메인 결정): (1) init+1초 '지속' 점프·쌍 0개 → 항등 고정이 아니라 계속 모음 → 1.2 m T 가 상한 밖 → align_failed
  (2) init 전 신호는 mode=backup(in_use) 만 기억  (3) 노드: pending=true 는 쌍<3 이면 고정 안 함, mode=backup 은 즉시 고정
원래 설명: [F] 단위 시험 — 합성 입력, 몇 초. ROS 실행 없음(노드 콜백은 가짜 시계로 직접 부름, rclpy 는 메시지 타입만).
고친 것 a·b·c·d 를 하나씩 확인한다. 실행: source /opt/ros/jazzy/setup.bash && ~/ws/night_20261104/venv/bin/python test_f_units.py
"""
import sys, os, math, json
HERE = os.path.dirname(os.path.abspath(__file__)); V2 = os.path.dirname(HERE)
sys.path.insert(0, V2); sys.path.insert(1, os.path.expanduser('~/ws/v_gosu/backup'))
from frame_align import FrameAligner
from odom_mux_node_v2 import GuardedMux, HealthGuard
from backup_core import MuxLogic

ok_all = True
def chk(name, cond, info=''):
    global ok_all
    ok_all &= bool(cond); print(('  통과 ' if cond else '  ★실패 ') + name, info)


def mux_run(health_fn, fl_fn, T=40.0, bk_fn=lambda t: (0.0, 0.0), odom_alive=lambda t: True):
    """10 ms 감시, FAST-LIO 20 Hz, 백업 상태 1 Hz. fl_fn(t) -> (x,y)|None, health_fn(t) -> str. 반환 (gm, 전환 시각, 사건)."""
    gm = GuardedMux(mux=MuxLogic(), guard=HealthGuard()); t_sw = None; ev = []
    for k in range(int(T * 100)):
        t = k * 0.01
        if k % 100 == 0: gm.guard.on_status(t, health_fn(t))
        if odom_alive(t): gm.guard.on_odom(t)
        if k % 5 == 0:
            p = fl_fn(t)
            if p is not None: gm.on_fastlio(t, p[0], p[1], 0.0, ts=t)
        m = gm.tick(t, bk_fn(t)); ev += gm.pop_events()
        if m == 'backup' and t_sw is None: t_sw = t
    return gm, t_sw, ev


print('── a) 전환 가드')
gm, ts, _ = mux_run(lambda t: 'aligning', lambda t: (0.0, 0.0) if t < 8 else None)
chk("aligning + FAST-LIO 끊김(8초) → 바로 전환", ts is not None and 8.4 < ts < 8.7, f'{ts}')
gm, ts, _ = mux_run(lambda t: 'aligning', lambda t: (0.2 * t, 0.0))
chk("aligning + 차이 커짐(표류 0.2 m/s) → 차이 조건은 무시(전환 없음)", ts is None, f'{ts}')
gm, ts, _ = mux_run(lambda t: 'aligning', lambda t: (0.0, 0.0) if t < 10 else (1.0, 0.0))
chk("aligning + 점프 1 m(10초) → 속도 조건으로 바로 전환", ts is not None and abs(ts - 10.0) < 0.05, f'{ts} {gm.mux.reason}')
gm, ts, _ = mux_run(lambda t: 'degraded', lambda t: (0.0, 0.0) if t < 8 else None)
chk("degraded + FAST-LIO 사망 → 전환(정전 방지)", ts is not None and 8.4 < ts < 8.7, f'{ts}')
gm, ts, _ = mux_run(lambda t: 'degraded', lambda t: (0.2 * max(0, t - 8), 0.0))
chk("degraded + 차이 → 차이 조건은 무시(원래 MuxLogic 그대로)", ts is None, f'{ts}')
for h in ('lost', 'init'):
    gm, ts, _ = mux_run(lambda t, h=h: h, lambda t: (0.0, 0.0) if t < 8 else None)
    chk(f"{h} + FAST-LIO 사망 → 보류(전환 안 함)", ts is None and gm.pending, f'{ts} {gm.pending}')
gm, ts, _ = mux_run(lambda t: 'degraded', lambda t: (0.0, 0.0) if t < 8 else None, odom_alive=lambda t: t < 7.9)
chk("degraded 인데 /backup/odometry 끊김 → 보류(위치끊김)", ts is None and gm.pending and gm.guard.state(30.0).startswith('위치끊김'), f'{gm.guard.state(30.0)}')
gm, ts, _ = mux_run(lambda t: 'ok', lambda t: (0.0, 0.0) if t < 8 else None, odom_alive=lambda t: t < 7.9)
chk("ok 인데 /backup/odometry 끊김 → 보류(예전과 같음)", ts is None and gm.pending)

print('── c) 보류 해제')
gm, ts, ev = mux_run(lambda t: 'lost' if t < 30 else 'ok', lambda t: (0.0, 0.0) if (t < 8 or t > 11) else None)
chk("끊김 8~11초 회복, 백업 lost→ok(30초) → 보류 해제, 전환 없음", ts is None and gm.pending is None and gm.n_clear == 1 and gm.mux.mode == 'fastlio',
    f"{ts} {[m for k, m in ev if k == 'clear'][:1]}")
gm2 = gm
gm, ts, ev = mux_run(lambda t: 'lost' if t < 30 else 'ok', lambda t: (0.0, 0.0) if t < 8 else None)
chk("끊김 8초~끝, 백업 lost→ok(30초) → 30초에 전환(해제 아님)", ts is not None and 30.0 <= ts < 30.1 and gm.n_clear == 0, f'{ts}')
gm, ts, ev = mux_run(lambda t: 'lost' if t < 30 else 'ok', lambda t: (0.0, 0.0) if t < 10 else (1.0, 0.0))
chk("점프 1 m(10초) 뒤 계속 틀림, 백업 lost→ok → 전환(차이 1 m 라 해제 안 됨)", ts is not None and 30.0 <= ts < 30.1 and gm.n_clear == 0, f'{ts}')
gm, ts, ev = mux_run(lambda t: 'lost' if t < 30 else 'ok', lambda t: (0.0, 0.0) if not (10 <= t < 10.05) else (1.0, 0.0))
chk("점프 한 번 튀고 제자리, 백업 lost→ok → 보류 해제", ts is None and gm.n_clear == 1, f'{ts}')
# 해제 뒤 다시 감시: 해제 뒤 FAST-LIO 가 또 끊기면 백업 ok 이므로 바로 전환
gm, ts, ev = mux_run(lambda t: 'lost' if t < 30 else 'ok', lambda t: (0.0, 0.0) if (t < 8 or 11 < t < 35) else None)
chk("해제 뒤 다시 끊김(35초) → 35.5초 전환 (감시 계속)", ts is not None and 35.4 < ts < 35.7 and gm.n_clear == 1, f'{ts}')
chk("영구 전환: 전환 뒤 FAST-LIO 가 돌아와도 backup 유지", mux_run(lambda t: 'ok', lambda t: (0.0, 0.0) if (t < 8 or t > 12) else None)[0].mode == 'backup')

print('── b) 정렬기')
def feed_aligner(al, t0, t1, fl_fn, bk=(5.0, 2.0, 0.7), stamp=True):
    """정지 로봇. 백업 150 Hz, 정합 5 Hz(수락), FAST-LIO 20 Hz. fl_fn(t)->(x,y,yaw)|None"""
    out = []
    for k in range(int((t1 - t0) * 300)):
        t = t0 + k / 300.0
        if k % 2 == 0: al.on_backup_pose(t, *bk)
        if k % 60 == 0: al.on_match(t, True, 0.01, 0.0); m = al.tick(t); out.append((t, m)) if m else None
        if k % 15 == 0:
            p = fl_fn(t)
            if p is not None:
                m = al.on_fastlio(t + 0.008, *p, stamp=t if stamp else None)
                if m: out.append((t, m))
    return out
th = math.radians(3.0); R = lambda x, y: (math.cos(th) * x - math.sin(th) * y + 0.2, math.sin(th) * x + math.cos(th) * y + 0.2)
qx, qy = R(5.0, 2.0)
al = FrameAligner(); al.on_init(0.0)
ev = feed_aligner(al, 0.0, 12.0, lambda t: (qx, qy, 0.7 + th) if t < 5.5 else (qx + 1.0, qy, 0.7 + th))
chk("정렬 중 점프 1 m(5.5초) → 정렬기가 스스로 보고 점프 전 쌍으로 고정", al.state == 'frozen' and abs(al.T[0] - 0.2 + 0.0) < 0.3 and al.n_jump == 1,
    f"{al.state} T={tuple(round(v, 3) for v in al.T)} {al.why}")
# 쌍이 거의 없을 때(init+1초) 점프 → 항등
al = FrameAligner(); al.on_init(0.0)
feed_aligner(al, 0.0, 8.0, lambda t: (qx, qy, 0.7 + th) if t < 1.0 else (qx + 1.0, qy, 0.7 + th))
chk("[G] init+1초 지속 점프(쌍 0개) → 항등 고정 안 하고 계속 모음 → T 1.2 m 가 상한 밖 → align_failed(항등·가림 풂)",
    al.state == 'align_failed' and al.T == (0.0, 0.0, 0.0) and al.n_anomaly_wait >= 1 and not al.mask_health(8.0), al.why)
# 정상: 점프 감지 오탐 없음 (정지 + 정상 0.6 m/s 이동)
al = FrameAligner(); al.on_init(0.0)
feed_aligner(al, 0.0, 10.0, lambda t: (qx, qy, 0.7 + th))
chk("정상(정지) → 보통 고정, 점프 0", al.state == 'frozen' and al.n_jump == 0, al.why)
# 표류 0.2 m/s → align_failed (쌍 ≥40 · 8초) → 가림 풂
al = FrameAligner(); al.on_init(0.0)
ev = feed_aligner(al, 0.0, 14.0, lambda t: (qx + 0.2 * t, qy, 0.7 + th))
chk("고정 전 표류 0.2 m/s → align_failed(항등) + 가림 풂", al.state == 'align_failed' and not al.mask_health(14.0) and al.T == (0.0, 0.0, 0.0),
    f"{al.why} 고정 {al.t_frozen:.1f}s")
al = FrameAligner(); al.on_init(0.0)
chk("가림 최대 20초", al.mask_health(19.9) and not al.mask_health(20.1))
# 이상 감지 즉시 고정: 쌍 ≥3 → 그 값 / init 전 신호 → init 순간 항등
al = FrameAligner(); al.on_init(0.0)
feed_aligner(al, 0.0, 3.0, lambda t: (qx, qy, 0.7 + th))
n = len(al.pairs); m = al.on_anomaly(3.0, '전환 장치 이상 감지·보류(x)')
chk("on_anomaly: 쌍 ≥3 → 그 추정으로 고정", al.state == 'frozen' and n >= 3 and abs(math.degrees(al.T[2]) - 3.0) < 0.2, f'쌍 {n} {m}')
al = FrameAligner(); al.on_anomaly(0.5, '전환 장치가 백업 사용 시작(x)', in_use=True); s0 = al.state; m = al.on_init(1.0)
chk("init 전 mode=backup(유지형, in_use) → 기억했다가 init 순간 항등 고정", s0 == 'identity' and al.state == 'identity_frozen' and m, f'{m}')
al = FrameAligner(); al.on_anomaly(0.5, '전환 장치 이상 감지·보류(x)'); m = al.on_init(1.0)
chk("[G] init 전 pending(보류)은 기억 안 함 → init 뒤 계속 추정", al.state == 'identity' and m is None and al.pre_init_anomaly is None, f'{m}')

print('── b·d) 노드 콜백 (Node 생성 없음)')
try:
    import threading, numpy as np
    from std_msgs.msg import String
    import backup_loc_node_v2 as N
    class _Log:
        def __init__(s): s.lines = []
        def info(s, m): s.lines.append(('I', m))
        def warn(s, m): s.lines.append(('W', m))
        def error(s, m): s.lines.append(('E', m))
    class _Pub:
        def __init__(s): s.msgs = []
        def publish(s, m): s.msgs.append(m)
    class _Clk:
        def now(s):
            class _N: nanoseconds = int(CLK[0] * 1e9)
            return _N()
    CLK = [10.0]
    class _Est:
        inited = True; last_rms = 0.03; n_ok = 5; n_bad = 0; t_good = 9.9
        class matcher: n_low_inlier = 4; last_inlier = 0.913
        def pose(s): return 1.0, 2.0, 0.0, 0.0, 0.0, 0.1
        def health(s, t): return 'ok'
    class FN(N.BackupLoc):
        def __init__(s):
            s.n_exc, s.t_exc_log = {}, {}; s.est = _Est(); s.align_on, s.align_mask = True, True
            s.al = FrameAligner(); s.al.on_init(9.0); s.lock = threading.Lock(); s.pub_stat = _Pub(); s._l = _Log()
        def get_logger(s): return s._l
        def get_clock(s): return _Clk()
    n = FN(); n.on_mux_status(String(data=json.dumps(dict(mode='fastlio', pending=False))))
    s_a = n.al.state
    n.on_mux_status(String(data=json.dumps(dict(mode='fastlio', pending=True, hold_reason='FAST-LIO 0.5초 무소식'))))
    chk("[G] /mux/status pending=true · 쌍 0개 → 고정 안 함(계속 모음), 발행 없음", s_a == 'identity' and n.al.state == 'identity' and not n.pub_stat.msgs
        and n.al.n_anomaly_wait == 1 and not n.n_exc, n.al.anomaly)
    n.on_mux_status(String(data=json.dumps(dict(mode='backup', reason='FAST-LIO 0.5초 무소식', pending=False))))
    st = json.loads(n.pub_stat.msgs[-1].data)
    chk("[G] /mux/status mode=backup · 쌍 0개 → 즉시 항등 고정 + 상태 발행", n.al.state == 'identity_frozen' and st['align']['state'] == 'identity_frozen' and not n.n_exc,
        n.al.why)
    chk("/backup/status 에 gicp {n_low_inlier, last_inlier}", st.get('gicp') == dict(n_low_inlier=4, last_inlier=0.913), f"{st.get('gicp')}")
    n2 = FN(); n2.est.matcher = type('M', (), {})(); n2.publish_status(10.0)
    chk("gicp 속성 없으면 생략", 'gicp' not in json.loads(n2.pub_stat.msgs[-1].data))
    n3 = FN(); n3.al = FrameAligner(); n3.on_mux_status(String(data=json.dumps(dict(mode='backup', reason='r'))))
    chk("init 전 mode=backup → 기억(pre_init_anomaly)", n3.al.pre_init_anomaly is not None and not n3.n_exc, f'{n3.al.pre_init_anomaly}')
except ImportError as e:
    print('  (rclpy 환경 아님 — 노드 확인 건너뜀)', e)

exec(open(os.path.join(HERE, 'g_units_new.py')).read())
print('G 단위 시험', '전부 통과' if ok_all else '★실패 있음')
sys.exit(0 if ok_all else 1)
