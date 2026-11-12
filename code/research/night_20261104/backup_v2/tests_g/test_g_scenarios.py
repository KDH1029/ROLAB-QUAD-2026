#!/usr/bin/env python3
"""[G] 시나리오 — 세 구성을 같은 입력으로 나란히: v1 / F(*.orig_0900) / G(지금).
ROS 실행 없음(rosbag2_py 로 bag 파일만 읽음). 쓰기: 이 폴더(tests_g/)만. 틀은 tests_f/test_f_scenarios.py 의 run3 그대로 + 검증자 고리(hov·gap·씨앗·on_match 덮어쓰기)
  v1 : backup_core.MuxLogic 그대로, 정렬 없음, health 그대로
  F  : frame_align.py.orig_0900 + odom_mux_node_v2.py.orig_0900, 노드 on_mux_status = pending 새로 걸림 또는 mode=backup → on_anomaly
  G  : 지금 frame_align.py + odom_mux_node_v2.py, 노드 on_mux_status = pending → on_anomaly(in_use=False) / mode=backup → on_anomaly(in_use=True)
부분  1 = 시작부(녹화 0~88초)  3 = 270~542초(주행 중 init)  2 = 전체 녹화 정렬 고정 뒤(D 틀)  4 = 전환 장치 단독(합성)
실행: source /opt/ros/jazzy/setup.bash && PYTHONDONTWRITEBYTECODE=1 ~/ws/night_20261104/venv/bin/python test_g_scenarios.py --part 1
"""
import sys, os, math, bisect, json, time, argparse, importlib.util, importlib.machinery
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); V2 = os.path.dirname(HERE)
sys.path.insert(0, V2); sys.path.insert(1, os.path.expanduser('~/ws/v_gosu/backup')); sys.path.insert(2, HERE)
from backup_core import MuxLogic, load_map
from frame_align import wrap
import frame_align as FA_G
import odom_mux_node_v2 as MX_G
import backup_core_v2 as BC


def load_src(name, path):
    ld = importlib.machinery.SourceFileLoader(name, path)
    sp = importlib.util.spec_from_file_location(name, path, loader=ld); m = importlib.util.module_from_spec(sp); ld.exec_module(m); return m


FA_F = load_src('frame_align_f', f'{V2}/frame_align.py.orig_0900')
MX_F = load_src('odom_mux_f', f'{V2}/odom_mux_node_v2.py.orig_0900')
T3 = 0.3 / math.sqrt(2)
HALF_NPZ = '/home/ilovehan9283/ws/night_20261104/backup_eval/map_first270s_fastlio.npz'
VERS = ('v1', 'F', 'G')


def rot_map(yaw, tx, ty):
    from test_frame_align import make_rotated_npz
    p = os.path.join(HERE, f'_map_y{yaw:g}.npz')
    if not os.path.exists(p):
        make_rotated_npz(HALF_NPZ, yaw, tx, ty, p)
    return load_map(p)


def stats(e):
    e = np.asarray(e)
    return None if not len(e) else dict(p95=round(float(np.percentile(e, 95)), 3), 최대=round(float(e.max()), 3))


def run3(name, A, ev, tL, GT, gyaw, t0, mp, inj=None, kill_after=None, dur=88.0, start_rel=0.0, latency=0.0085,
         hov=None, gap=None, seed=None, al_wrap=None, al_kw=None, fl_none=False):
    """inj(rel, x, y)->(x, y) FAST-LIO 고장 주입(init 기준 상대초). kill_after: init 뒤 이 초부터 FAST-LIO 없음. hov(rel)->health 덮어쓰기.
    gap(rel)->bool FAST-LIO 메시지 버림. seed=(dx, dyaw°) 백업 씨앗 오차. al_wrap(Base)->Sub 정렬기 덮어쓰기(F·G 둘 다). al_kw 정렬기 인자(F·G 둘 다).
    fl_none: FAST-LIO 가 처음부터 없음 — 백업은 첫 스캔에서 원점 init(노드의 5초 대기 뒤 원점과 같은 경로)."""
    ya, ax, ay = math.radians(A[0]), A[1], A[2]; ca, sa = math.cos(ya), math.sin(ya)
    T_true = (-(ca * ax + sa * ay), -(-sa * ax + ca * ay), -ya)
    def gt_i(ts): return min(max(bisect.bisect_left(tL, ts), 0), len(tL) - 1)
    est = BC.make_estimator(mp)
    if seed is not None:
        ini = est.init; dx, dyw = seed
        est.init = lambda x, y, z, yaw, leg, t: ini(x + dx, y, z, yaw + math.radians(dyw), leg, t)
    kw = al_kw or {}
    def mk(mod):
        C = mod.FrameAligner if al_wrap is None else al_wrap(mod.FrameAligner)
        return C(**kw)
    V = {}
    V['v1'] = dict(mux=MuxLogic(), al=None)
    V['F'] = dict(mux=MX_F.GuardedMux(mux=MuxLogic(), guard=MX_F.HealthGuard()), al=mk(FA_F), pol='f')
    V['G'] = dict(mux=MX_G.GuardedMux(mux=MuxLogic(), guard=MX_G.HealthGuard()), al=mk(FA_G), pol='g')
    for v in V.values():
        v.update(h='init', sw=None, reason='', last_pending=None, holds=0, clears=0, inj_at_sw=None)
    t_init_ = [None]
    def pub_status(v, tt):
        hr = est.health(tt)
        if hov is not None and t_init_[0] is not None:
            o = hov(tt - t_init_[0])
            if o: hr = o
        if v['al'] is None:
            v['h'] = hr; return
        v['h'] = 'aligning' if (hr == 'ok' and v['al'].mask_health(tt)) else hr
        v['mux'].guard.on_status(tt, v['h'])
    def bk_xy(v):
        return (est.x, est.y) if v['al'] is None else v['al'].map_to_cam(est.x, est.y, est.yaw)[:2]
    cur_inj = [0.0]
    def after_mux(v, tt):
        g = v['mux']
        mode = g.mode
        if v['al'] is not None:
            for k, _ in g.pop_events():
                if k == 'hold': v['holds'] += 1
                if k == 'clear': v['clears'] += 1
            newly_pending = g.pending is not None and v['last_pending'] is None
            v['last_pending'] = g.pending
            if v['pol'] == 'f' and (newly_pending or mode == 'backup'):
                if v['al'].on_anomaly(tt, f"mux {'backup' if mode == 'backup' else 'pending'}"): pub_status(v, tt)
            if v['pol'] == 'g':
                if mode == 'backup':
                    if v['al'].on_anomaly(tt, 'mux backup', in_use=True): pub_status(v, tt)
                elif newly_pending:
                    if v['al'].on_anomaly(tt, 'mux pending', in_use=False): pub_status(v, tt)
        if mode == 'backup' and v['sw'] is None:
            v['sw'] = tt; v['reason'] = (g.reason if v['al'] is None else g.mux.reason); v['inj_at_sw'] = cur_inj[0]
    def mux_tick(tt):
        for v in V.values():
            if v['al'] is None:
                v['mux'].tick(tt, bk_xy(v), v['h'])
            else:
                v['mux'].guard.on_odom(tt); v['mux'].tick(tt, bk_xy(v))
            after_mux(v, tt)
    leg = None; t_init = None; t_tick = None; fl_last = None; n_leg = 0; rows = []; t_kill = None
    for t, kind, val in ev:
        if dur is not None and t_init is not None and t > t_init + dur: break
        if kind == 0:
            leg = val
            if est.inited:
                est.on_leg(val)
                for v in V.values():
                    if v['al'] is not None: v['al'].on_backup_pose(t, est.x, est.y, est.yaw)
                n_leg += 1
                if n_leg % 7 == 0: mux_tick(t)
            continue
        if kind == 2:
            if fl_none: continue
            x, y, z, yw = val
            if t_init is not None:
                rel = t - t_init
                if kill_after is not None and rel > kill_after:
                    t_kill = t_kill or t; continue
                if gap is not None and gap(rel):
                    continue
                if inj is not None:
                    x2, y2 = inj(rel, x, y); cur_inj[0] = math.hypot(x2 - x, y2 - y); x, y = x2, y2
            fl_last = (t, (x, y, z, yw))
            if est.inited:
                for v in V.values():
                    if v['al'] is not None:
                        m = v['al'].on_fastlio(t + latency, x, y, yw, stamp=t)
                        if m and v['pol'] in ('f', 'g'): pub_status(v, t + latency)
                    v['mux'].on_fastlio(t + latency, x, y, z, ts=t)
                    after_mux(v, t + latency)
            continue
        if leg is None or t < t0 + start_rel: continue
        if not est.inited:
            if fl_none:
                est.init(0.0, 0.0, 0.0, 0.0, leg, t); t_init = t; t_tick = t; t_kill = t
            else:
                if fl_last is None: continue
                fx, fy, fz, fyaw = fl_last[1]
                est.init(fx, fy, fz, fyaw, leg, t); t_init = t; t_tick = t
            t_init_[0] = t_init
            for v in V.values():
                if v['al'] is not None and v['al'].on_init(t): pub_status(v, t)
        prep = est.prepare(val.astype(np.float64))
        if prep is None: continue
        T, rms = est.matcher.align(prep['P'], prep['T0']); cm, cr = BC.match_correction(prep, T)
        res = est.apply(prep, T, rms, t)
        for v in V.values():
            if v['al'] is None: continue
            v['al'].on_match(t, res['accepted'], cm, cr); v['al'].on_backup_pose(t, est.x, est.y, est.yaw)
            if v['al'].tick(t): pub_status(v, t)
        while t >= t_tick + 1.0:
            t_tick += 1.0
            for v in V.values():
                if v['al'] is not None: v['al'].tick(t_tick)
                pub_status(v, t_tick)
        i = gt_i(t); gx, gy = GT[i, 2], GT[i, 3]
        row = [t]
        dead = t_kill is not None and t >= t_kill
        if gap is not None and t_init is not None and gap(t - t_init): dead = True
        for v in V.values():
            bx, by = bk_xy(v); e_bk = math.hypot(bx - gx, by - gy)
            on_bk = v['sw'] is not None
            row += [e_bk if on_bk else (np.nan if dead else cur_inj[0]), e_bk]
        rows.append(row)
    R = np.array(rows); out = {}
    for j, (k, v) in enumerate(V.items()):
        e_in, e_bk = R[:, 1 + 2 * j], R[:, 2 + 2 * j]
        o = dict(전환=None if v['sw'] is None else round(v['sw'] - t_init, 1), 사유=v['reason'])
        if v['sw'] is not None:
            o['전환때_FL오차m'] = round(v['inj_at_sw'], 2)
            o['전환뒤_백업오차'] = stats(e_bk[R[:, 0] >= v['sw']])
        o['플래너입력_최대오차m'] = round(float(np.nanmax(e_in)), 2) if np.isfinite(e_in).any() else None
        o['정전s'] = round(float(np.isnan(e_in).sum() * 0.2), 1)
        al = v['al']
        if al is not None:
            o['정렬'] = al.state; o['정렬사유'] = al.why[:110]
            o['고정s'] = None if al.t_frozen is None else round(al.t_frozen - t_init, 1)
            if al.state == 'frozen':
                tx, ty, th = al.T
                o['T오차'] = dict(xy_m=round(math.hypot(tx - T_true[0], ty - T_true[1]), 3), yaw_deg=round(math.degrees(wrap(th - T_true[2])), 2))
            elif al.frozen():
                o['T오차'] = dict(xy_m=round(math.hypot(T_true[0], T_true[1]), 3), yaw_deg=round(math.degrees(-T_true[2]), 2), 항등=True)
            o['보류경고'] = v['holds']
            if v['clears']: o['보류해제'] = v['clears']
        out[k] = o
    return out


# ── 정렬기 덮어쓰기 (검증자 K·P 틀) ──
def streak_break(a, b):
    def w(Base):
        class W(Base):
            def on_match(self, t, accepted, corr_m=0.0, corr_rad=0.0):
                if self.t_start is not None and a <= t - self.t_start <= b:
                    return super().on_match(t, accepted, 1.0, corr_rad)
                return super().on_match(t, accepted, corr_m, corr_rad)
        return W
    return w


def starve(Base):
    class W(Base):
        def on_match(self, t, accepted, corr_m=0.0, corr_rad=0.0):
            return super().on_match(t, accepted, 1.0, corr_rad)
    return W


ramp = lambda d, a, b: (lambda rel, x, y: (x + d * min(1.0, max(0.0, (rel - a) / (b - a))), y))
drift = lambda a_, ton: (lambda rel, x, y: (x + a_ * max(0.0, rel - ton), y))
jump = lambda d, ton: (lambda rel, x, y: (x + (d if rel > ton else 0.0), y))
spike = lambda d, ton, w=0.05: (lambda rel, x, y: (x + (d if ton <= rel < ton + w else 0.0), y))
win = lambda a, b, v: (lambda rel: v if a <= rel < b else None)
gapf = lambda a, b: (lambda rel: a <= rel < b)


def cases_part1(mp3, mp5, mp0):
    A3, A5, A0 = (3.0, T3, T3), (5.0, T3, T3), (0.0, 0.0, 0.0)
    C = [
        # ── F 의 tests_f ① 10건
        ('F① 고정 전 표류 0.05 m/s (init+0~), 3°', A3, mp3, dict(inj=drift(0.05, 0.0))),
        ('F① 고정 전 표류 0.2 m/s (init+3~), 3°', A3, mp3, dict(inj=drift(0.2, 3.0))),
        ('F① 고정 전 표류 0.2 m/s (init+3~), 0°', A0, mp0, dict(inj=drift(0.2, 3.0))),
        ('F① 고정 전 점프 1.0 m (init+2), 3°', A3, mp3, dict(inj=jump(1.0, 2.0))),
        ('F① 고정 전 점프 1.0 m (init+5.5), 3°', A3, mp3, dict(inj=jump(1.0, 5.5))),
        ('F① 고정 전 점프 1.0 m (init+5.5), 0°', A0, mp0, dict(inj=jump(1.0, 5.5))),
        ('F① 고정 뒤 표류 0.2 m/s (init+20~), 3°', A3, mp3, dict(inj=drift(0.2, 20.0))),
        ('F① 고정 전 FAST-LIO 사망 (init+4), 3°', A3, mp3, dict(kill_after=4.0)),
        ('F① 정상 (0.3 m·3° 출발 오차), 3°', A3, mp3, {}),
        ('F① 정상 (0.3 m·5° 출발 오차), 5°', A5, mp5, {}),
        # ── 검증자 A: N·S·Y·K·P
        ('N0 대조 정상, 3°', A3, mp3, {}),
        ('N1 매끈 이동 0.8 m (0.5~2.5초) 후 정지, 3°', A3, mp3, dict(inj=ramp(0.8, 0.5, 2.5))),
        ('N1 같은 경우, 0°', A0, mp0, dict(inj=ramp(0.8, 0.5, 2.5))),
        ('N2 매끈 이동 0.8 m (0.5~4.5초), 3°', A3, mp3, dict(inj=ramp(0.8, 0.5, 4.5))),
        ('N3 매끈 1.0 m 를 1초에 (3~4초), 3°', A3, mp3, dict(inj=ramp(1.0, 3.0, 4.0))),
        ('N4 매끈 1.0 m 를 0.6초에 (4~4.6초), 0°', A0, mp0, dict(inj=ramp(1.0, 4.0, 4.6))),
        ('N5 매끈 0.6 m (0~1.5초), 0°', A0, mp0, dict(inj=ramp(0.6, 0.0, 1.5))),
        ('N6 1.0 m 를 0.5초에 (1.5~2.0초), 3°', A3, mp3, dict(inj=ramp(1.0, 1.5, 2.0))),
        ('N7 0.6 m 를 두 메시지로 (1.0~1.2초), 0°', A0, mp0, dict(inj=ramp(0.6, 1.0, 1.2))),
        ('N8 계단 0.6 m (1.0초), 0°', A0, mp0, dict(inj=jump(0.6, 1.0))),
        ('S1 씨앗 1.5 m + 표류 0.2 m/s (3~), 0°', A0, mp0, dict(seed=(1.5, 0.0), inj=drift(0.2, 3.0))),
        ('S2 씨앗 1.5 m, 고장 없음, 0°', A0, mp0, dict(seed=(1.5, 0.0))),
        ('S3 씨앗 1.5 m + N1, 0°', A0, mp0, dict(seed=(1.5, 0.0), inj=ramp(0.8, 0.5, 2.5))),
        ('Y1 씨앗 45°+0.3 m, 고장 없음, 3°', A3, mp3, dict(seed=(0.3, 45.0))),
        ('Y2 씨앗 45°+0.3 m + 0.8 m (3~5초), 3°', A3, mp3, dict(seed=(0.3, 45.0), inj=ramp(0.8, 3.0, 5.0))),
        ('Y3 씨앗 20°+0.3 m + 0.8 m (3~5초), 3°', A3, mp3, dict(seed=(0.3, 20.0), inj=ramp(0.8, 3.0, 5.0))),
        ('K1 수락 끊김 3~5초 + 0.8 m (3~4.5초), 0°', A0, mp0, dict(al_wrap=streak_break(3.0, 5.0), inj=ramp(0.8, 3.0, 4.5))),
        ('K2 같은 경우, 3°', A3, mp3, dict(al_wrap=streak_break(3.0, 5.0), inj=ramp(0.8, 3.0, 4.5))),
        ('K3 수락 끊김 3~5초만 (대조), 3°', A3, mp3, dict(al_wrap=streak_break(3.0, 5.0))),
        ('K4 수락 끊김 2.5~8초 + 0.5 m (4~6초), 0°', A0, mp0, dict(al_wrap=streak_break(2.5, 8.0), inj=ramp(0.5, 4.0, 6.0))),
        ('K5 수락 끊김 2.6~5초 + 0.8 m (3~4.5초), 0°', A0, mp0, dict(al_wrap=streak_break(2.6, 5.0), inj=ramp(0.8, 3.0, 4.5))),
        ('K6 수락 끊김 2.8~5초 + 0.8 m (3~4.5초), 0°', A0, mp0, dict(al_wrap=streak_break(2.8, 5.0), inj=ramp(0.8, 3.0, 4.5))),
        ('K7 수락 끊김 3.2~6초 + 0.8 m (3.5~5초), 0°', A0, mp0, dict(al_wrap=streak_break(3.2, 6.0), inj=ramp(0.8, 3.5, 5.0))),
        ('P1 쌍 끝내 없음 + 표류 0.2 m/s (3~), 0° [가림 20초]', A0, mp0, dict(al_wrap=starve, inj=drift(0.2, 3.0))),
        # ── 검증자 B: ① 시작부 튐·lost·느린 표류
        ('B① lost(0~9) 중 FAST-LIO 0.7초 끊김(5.2), 3°', A3, mp3, dict(hov=win(0, 9, 'lost'), gap=gapf(5.2, 5.9))),
        ('B① 느린 표류 0.02 m/s (0~), 3°', A3, mp3, dict(inj=drift(0.02, 0.0))),
        ('B① 느린 표류 0.03 m/s (0~), 3°', A3, mp3, dict(inj=drift(0.03, 0.0))),
        # ── G 새: FAST-LIO 처음부터 없음 / 0.3초만 내고 죽음
        ('G FAST-LIO 처음부터 없음 (백업 원점 init), 3°', A3, mp3, dict(fl_none=True)),
        ('G FAST-LIO 0.3초만 내고 사망 (init+0.3), 3°', A3, mp3, dict(kill_after=0.3)),
        ('G FAST-LIO 0.3초만 내고 사망 (init+0.3), 0°', A0, mp0, dict(kill_after=0.3)),
    ]
    # ── G 새: 출발 직후 한 메시지 튐 (① 시작부)
    for d in (0.4, 0.7, 1.0):
        for ton in (0.5, 1.0, 2.0, 3.0, 4.0):
            C.append((f'G① 한 메시지 튐 {d} m (init+{ton}), 3°', A3, mp3, dict(inj=spike(d, ton))))
        for ton in (1.0, 2.0):
            C.append((f'G① 한 메시지 튐 {d} m (init+{ton}), 0°', A0, mp0, dict(inj=spike(d, ton))))
    C.append(('G① 두 메시지 튐 1.0 m (init+2.0), 3°', A3, mp3, dict(inj=spike(1.0, 2.0, 0.1))))
    C.append(('G① 튐 0.25초 1.0 m (init+2.0), 3°', A3, mp3, dict(inj=spike(1.0, 2.0, 0.27))))
    return C


def cases_part3(mp3, mp5, mp0):
    A3, A5, A0 = (3.0, T3, T3), (5.0, T3, T3), (0.0, 0.0, 0.0)
    C = [('F③ 정상 주행 270~542초, 3°', A3, mp3, {}), ('F③ 정상 주행 270~542초, 5°', A5, mp5, {}),
         ('B③ lost(4~9) 중 FAST-LIO 0.7초 끊김(5.2), 3°', A3, mp3, dict(hov=win(4, 9, 'lost'), gap=gapf(5.2, 5.9))),
         ('B③ lost(0~9) 중 FAST-LIO 0.7초 끊김(5.2), 3°', A3, mp3, dict(hov=win(0, 9, 'lost'), gap=gapf(5.2, 5.9))),
         ('B③ lost(20~30) 중 FAST-LIO 0.7초 끊김(25), 3°', A3, mp3, dict(hov=win(20, 30, 'lost'), gap=gapf(25, 25.7))),
         ('B③ 느린 표류 0.03 m/s (0~), 3°', A3, mp3, dict(inj=drift(0.03, 0.0)))]
    for d in (0.4, 0.7, 1.0):
        for ton in (0.5, 1.0, 1.5, 2.0, 3.0, 4.0):
            C.append((f'G③ 한 메시지 튐 {d} m (init+{ton}), 3°', A3, mp3, dict(inj=spike(d, ton))))
        for ton in (1.0, 2.0, 3.0):
            C.append((f'G③ 한 메시지 튐 {d} m (init+{ton}), 0°', A0, mp0, dict(inj=spike(d, ton))))
    for ton in (5.0, 6.0):
        C.append((f'B③ 한 메시지 튐 1.0 m (init+{ton}, 예열 뒤), 3°', A3, mp3, dict(inj=spike(1.0, ton))))
    C.append(('G③ 두 메시지 튐 1.0 m (init+2.0), 3°', A3, mp3, dict(inj=spike(1.0, 2.0, 0.1))))
    C.append(('G③ 지속 점프 1.0 m (init+2.0), 3°', A3, mp3, dict(inj=jump(1.0, 2.0))))
    return C


# ═════════════════════ ② 전체 녹화, 정렬 고정 뒤 (D 틀, F 의 part2 그대로 + G) ═════════════════════
def part2():
    import rosbag2_py
    BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
    NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
    def reader(topics):
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
        r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r
    d = np.load(NPZ); odom, lid = d['odom'], d['lid']
    hdr2bag = {}
    r = reader(['/lidar_points'])
    while r.has_next():
        tp, raw, t = r.read_next(); s, ns = np.frombuffer(raw[4:8], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]
        hdr2bag[round(float(s) + float(ns) * 1e-9, 4)] = t * 1e-9
    pairs = np.array([(w, hdr2bag[round(h, 4)]) for w, h in lid if round(h, 4) in hdr2bag])
    Kp = float(np.median(pairs[:, 0] - pairs[:, 1])); wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
    dmed = float(np.median(h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]))
    tL = h_o - dmed - Kp; o = np.argsort(tL); tL = tL[o]; GT = odom[o]
    T = np.load('/home/ilovehan9283/ws/night_20261104/backup_v2/tests_d/bk_traj_full.npy')     # D 가 만든 캐시 (읽기만)
    bk_t = T[:, 0]; t0 = tL[0]
    def bk_at(ts):
        i = min(max(bisect.bisect_right(bk_t, ts) - 1, 0), len(T) - 1); return (T[i, 1], T[i, 2]), ('ok' if T[i, 10] else 'degraded')

    def run(name, fault, health=None, status_alive=None, odom_alive=None, dead=None):
        out = {}
        for ver in VERS:
            if ver == 'v1': g = MuxLogic()
            elif ver == 'F': g = MX_F.GuardedMux(mux=MuxLogic(), guard=MX_F.HealthGuard())
            else: g = MX_G.GuardedMux(mux=MuxLogic(), guard=MX_G.HealthGuard())
            sw = None; last_stat = -1e9; holds = 0; clears = 0; blackout = 0.0; mode = 'fastlio'; clear_t = None
            for i, ts in enumerate(tL):
                xyz = fault(ts, GT[i, 2:5].copy())
                for k in range(4):
                    tt = ts + k * 0.0125; rel = tt - t0
                    bxy, h = bk_at(tt)
                    if health is not None and health(rel) is not None: h = health(rel)
                    if ver == 'v1':
                        if k == 0 and xyz is not None: g.on_fastlio(ts, *xyz)
                        mode = g.tick(tt, bxy, h)
                    else:
                        if (status_alive is None or status_alive(rel)) and tt - last_stat >= 1.0:
                            g.guard.on_status(tt, h); last_stat = tt
                        if odom_alive is None or odom_alive(rel): g.guard.on_odom(tt)
                        if k == 0 and xyz is not None: g.on_fastlio(ts, *xyz)
                        mode = g.tick(tt, bxy)
                        for kind, msg in g.pop_events():
                            if kind == 'hold': holds += 1
                            if kind == 'clear': clears += 1; clear_t = clear_t or rel
                    if mode == 'backup' and sw is None:
                        sw = (rel, g.reason if ver == 'v1' else g.mux.reason)
                    if mode == 'fastlio' and dead is not None and dead(rel):
                        blackout += 0.0125
            r = dict(전환=None if sw is None else round(sw[0], 1), 사유='' if sw is None else sw[1], 정전s=round(blackout, 1))
            if ver != 'v1':
                r['보류경고'] = holds
                if clears: r['보류해제'] = f'{clears}회 ({clear_t:.1f}초)'
            out[ver] = r
        print('②', name, json.dumps(out, ensure_ascii=False), flush=True)
        return name, out

    cut = lambda a, b=1e9: (lambda ts, p: None if a < ts - t0 < b else p)
    dead = lambda a, b=1e9: (lambda rel: a < rel < b)
    jump300 = lambda ts, p: p + np.array([3.0, 0, 0]) if ts - t0 > 300 else p
    slide = lambda ts, p: p + np.array([0.05 * max(0, ts - t0 - 150), 0, 0])
    S = [
        run('★ FAST-LIO 사망(200초~) + 백업 degraded(195초~)', cut(200), health=lambda rel: 'degraded' if rel > 195 else None, dead=dead(200)),
        run('★ 끊김 200~203초 후 회복 + 백업 degraded(195~232초)', cut(200, 203), health=lambda rel: 'degraded' if 195 < rel < 232 else None, dead=dead(200, 203)),
        run('★ 끊김 200~203초 후 회복 + 백업 lost(195~232초) → ok', cut(200, 203), health=lambda rel: 'lost' if 195 < rel < 232 else None, dead=dead(200, 203)),
        run('★ 끊김 200~203초 후 회복 + /backup/status 끊김(195~232초)', cut(200, 203), status_alive=lambda rel: not (195 < rel < 232), dead=dead(200, 203)),
        run('정상 주행', lambda ts, p: p),
        run('FAST-LIO 끊김 (200초부터)', cut(200), dead=dead(200)),
        run('FAST-LIO 3 m 점프 (300초)', jump300),
        run('서서히 미끄러짐 0.05 m/s (150초부터)', slide),
        run('끊김 200초 + 백업 lost (190초부터 끝까지)', cut(200), health=lambda rel: 'lost' if rel > 190 else None, dead=dead(200)),
        run('끊김 200초 + 백업 lost 190~230초 → 회복', cut(200), health=lambda rel: 'lost' if 190 < rel < 230 else None, dead=dead(200)),
        run('점프 300초 + 백업 degraded (295초부터)', jump300, health=lambda rel: 'degraded' if rel > 295 else None),
        run('끊김 200초 + /backup/status 소식 없음 (195초부터)', cut(200), status_alive=lambda rel: rel < 195, odom_alive=lambda rel: rel < 195, dead=dead(200)),
        run('끊김 200초 + 백업 ok 인데 /backup/odometry 끊김 (198초부터)', cut(200), odom_alive=lambda rel: rel < 198, dead=dead(200)),
        run('끊김 6초 + 백업 아직 init (12초까지)', cut(6), health=lambda rel: 'init' if rel < 12 else None, dead=dead(6)),
        run('미끄러짐 0.05 m/s + 백업 degraded (145초부터)', slide, health=lambda rel: 'degraded' if rel > 145 else None),
        # G 새: /backup/status 끊김 뒤 degraded 로 재개 (U8b 의 녹화본판)
        run('G 끊김 200~200.7초 + /backup/status 끊김(195~205초) → degraded 로 재개(205~232초)', cut(200, 200.7),
            health=lambda rel: 'degraded' if 205 <= rel < 232 else None, status_alive=lambda rel: not (195 < rel < 205), dead=dead(200, 200.7)),
        run('G 같은데 aligning 으로 재개', cut(200, 200.7),
            health=lambda rel: 'aligning' if 205 <= rel < 232 else None, status_alive=lambda rel: not (195 < rel < 205), dead=dead(200, 200.7)),
        run('G 끊김 200초 + 백업 health JSON null (195초~)', cut(200), health=lambda rel: 'None' if rel > 195 else None, dead=dead(200)),
    ]
    return dict(S)


# ═════════════════════ ④ 전환 장치 단독 (합성, 검증자 A verify_mux_nofl + B U8b) ═════════════════════
def part4():
    R = {}
    def run(name, fl_until=None, fl_from=0.0, health=lambda t: 'ok', T=60.0, st_gap=None, fl_gap=None):
        out = {}
        for ver in VERS:
            g = MuxLogic() if ver == 'v1' else (MX_F if ver == 'F' else MX_G).GuardedMux(mux=MuxLogic(), guard=(MX_F if ver == 'F' else MX_G).HealthGuard())
            sw = None; blackout = 0.0; last_stat = -9; t = 0.0; dt = 0.01; k = 0; clears = 0
            while t < T:
                fl_alive = (fl_until is None or t < fl_until) and t >= fl_from and not (fl_gap and fl_gap[0] <= t < fl_gap[1])
                if fl_until == 0: fl_alive = False
                h = health(t)
                if ver != 'v1':
                    if t - last_stat >= 1.0 and not (st_gap and st_gap[0] <= t < st_gap[1]): g.guard.on_status(t, h); last_stat = t
                    g.guard.on_odom(t)
                if fl_alive and k % 5 == 0:
                    g.on_fastlio(t, 0.0, 0.0, 0.0, ts=t)
                mode = g.tick(t, (0.0, 0.0)) if ver != 'v1' else g.tick(t, (0.0, 0.0), h)
                if ver != 'v1':
                    clears += sum(1 for kk, _ in g.pop_events() if kk == 'clear')
                if mode == 'backup' and sw is None: sw = round(t, 2)
                if mode == 'fastlio' and not fl_alive: blackout += dt
                t += dt; k += 1
            out[ver] = dict(전환=sw, 사유=(g.reason if ver == 'v1' else g.mux.reason) if sw is not None else '', 정전s=round(blackout, 1))
            if ver != 'v1' and clears: out[ver]['보류해제'] = clears
        print('④', name, json.dumps(out, ensure_ascii=False), flush=True); R[name] = out
    run('FAST-LIO 처음부터 없음, 백업 ok', fl_until=0)
    run('FAST-LIO 처음부터 없음, 백업 degraded', fl_until=0, health=lambda t: 'degraded')
    run('FAST-LIO 처음부터 없음, 백업 aligning', fl_until=0, health=lambda t: 'aligning')
    run('FAST-LIO 처음부터 없음, 백업 init→aligning(12초)', fl_until=0, health=lambda t: 'init' if t < 12 else 'aligning')
    run('FAST-LIO 0.3초만 내고 사망, 백업 ok', fl_until=0.3)
    run('FAST-LIO 3초에 사망, 백업 ok', fl_until=3.0)
    run('FAST-LIO 9초에 처음 옴, 백업 init→aligning(12초)', fl_from=9.0, health=lambda t: 'init' if t < 12 else 'aligning')
    run('FAST-LIO 3초에 처음 옴(예열 안), 백업 aligning', fl_from=3.0, health=lambda t: 'aligning')
    for rh in ('ok', 'degraded', 'aligning'):
        run(f'U8b /backup/status 끊김 8~16초 + FAST-LIO 끊김 12.0~12.7초 → 16초 {rh} 로 재개', T=25.0, st_gap=(8.0, 16.0), fl_gap=(12.0, 12.7),
            health=lambda t, rh=rh: rh if 16.0 <= t < 19.0 else 'ok')
    run('health 모르는 값 "stale" + FAST-LIO 8초 사망', fl_until=8.0, health=lambda t: 'stale', T=30.0)
    run('health JSON null + FAST-LIO 8초 사망', fl_until=8.0, health=lambda t: None, T=30.0)
    return R


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--part', default='1'); ap.add_argument('--only', default='')
    a = ap.parse_args(); parts = set(a.part.split(',')); res = {}
    if '1' in parts or '3' in parts:
        from test_frame_align import load_events, SPLIT
        t_ = time.time()
        wins = ([(0.0, 90.0)] if '1' in parts else []) + ([(SPLIT - 2, None)] if '3' in parts else [])
        evs, tL, GT, gyaw, t0 = load_events(tuple(wins)); print(f'bag 읽기 {time.time() - t_:.0f}s', flush=True)
        mp3, mp5, mp0 = rot_map(3.0, T3, T3), rot_map(5.0, T3, T3), load_map(HALF_NPZ)
        if '1' in parts:
            R1 = {}
            for nm, A, mp, kw in cases_part1(mp3, mp5, mp0):
                if a.only and a.only not in nm: continue
                t_ = time.time(); R1[nm] = run3(nm, A, evs[0], tL, GT, gyaw, t0, mp, **kw)
                print(f'① [{nm}] ({time.time() - t_:.0f}s)', json.dumps(R1[nm], ensure_ascii=False), flush=True)
            res['①시작부'] = R1
        if '3' in parts:
            R3 = {}
            for nm, A, mp, kw in cases_part3(mp3, mp5, mp0):
                if a.only and a.only not in nm: continue
                t_ = time.time(); R3[nm] = run3(nm, A, evs[-1], tL, GT, gyaw, t0, mp, dur=None, start_rel=SPLIT, **kw)
                print(f'③ [{nm}] ({time.time() - t_:.0f}s)', json.dumps(R3[nm], ensure_ascii=False), flush=True)
            res['③주행중init'] = R3
    if '2' in parts:
        res['②전체녹화'] = part2()
    if '4' in parts:
        res['④전환장치단독'] = part4()
    out = os.path.join(HERE, f"result_g_{a.part.replace(',', '')}.json")
    json.dump(res, open(out, 'w'), ensure_ascii=False, indent=1); print('저장', out)


if __name__ == '__main__':
    main()
