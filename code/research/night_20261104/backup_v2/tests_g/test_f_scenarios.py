#!/usr/bin/env python3
"""[F] 검증 에이전트 시나리오 — 세 구성을 같은 입력으로 나란히: 예전 v1 / C+D(지금, *.orig_0825) / F(고친 것).
ROS 실행 없음(rosbag2_py 로 bag 파일만 읽음). 쓰기: 이 폴더(tests_f/)만.

  v1   : backup_core.MuxLogic 그대로, 정렬 없음(백업 = 지도 좌표 그대로), health 그대로 (v_gosu 원본 구성)
  C+D  : frame_align.py.orig_0825 + odom_mux_node_v2.py.orig_0825 (GuardedMux) + 'aligning' 가림, 노드 on_mux_status = mode=backup 일 때만 고정
  F    : 지금 frame_align.py + odom_mux_node_v2.py, 노드 on_mux_status = pending 새로 걸림 또는 mode=backup → on_anomaly, on_fastlio 반환 고정

① 시작부(녹화 0~88초: 50초 정지 후 출발) — 정렬기 포함. 지도 = 앞 270초 지도를 A(3°/5°, 0.3 m) 로 돌린 것(정답 T = A⁻¹). FAST-LIO 에 고장 주입
② 전체 녹화(D 의 test_mux_guard 틀, 백업 궤적 캐시 tests_d/bk_traj_full.npy 읽기만) — 정렬 고정 뒤(경기 중) 백업 health 덮어쓰기 시나리오
③ 오전환: 3°·5° 지도로 270~542초 구간(정상 주행) 세 구성 전환 0 확인
실행: source /opt/ros/jazzy/setup.bash && ~/ws/night_20261104/venv/bin/python test_f_scenarios.py [--part 1,2,3]
"""
import sys, os, math, bisect, json, time, argparse, importlib.util, importlib.machinery
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); V2 = os.path.dirname(HERE)
sys.path.insert(0, V2); sys.path.insert(1, os.path.expanduser('~/ws/v_gosu/backup')); sys.path.insert(2, HERE)
from backup_core import MuxLogic, load_map
from frame_align import wrap
import frame_align as FA_F
import odom_mux_node_v2 as MX_F


def load_src(name, path):
    ld = importlib.machinery.SourceFileLoader(name, path)
    sp = importlib.util.spec_from_file_location(name, path, loader=ld); m = importlib.util.module_from_spec(sp); ld.exec_module(m); return m


FA_CD = load_src('frame_align_cd', f'{V2}/frame_align.py.orig_0825')
MX_CD = load_src('odom_mux_cd', f'{V2}/odom_mux_node_v2.py.orig_0825')
T3 = 0.3 / math.sqrt(2)
HALF_NPZ = '/home/ilovehan9283/ws/night_20261104/backup_eval/map_first270s_fastlio.npz'


def rot_map(yaw, tx, ty):
    from test_frame_align import make_rotated_npz
    p = os.path.join(HERE, f'_map_y{yaw:g}.npz')
    if not os.path.exists(p):
        make_rotated_npz(HALF_NPZ, yaw, tx, ty, p)
    return load_map(p)


def stats(e):
    e = np.asarray(e)
    return None if not len(e) else dict(p95=round(float(np.percentile(e, 95)), 3), 최대=round(float(e.max()), 3))


# ═════════════════════ ① 정렬기 포함, 세 구성 나란히 ═════════════════════
def run3(name, A, ev, tL, GT, gyaw, t0, mp, inj=None, kill_after=None, dur=88.0, start_rel=0.0, latency=0.0085):
    """inj(rel_init, x, y) -> (x, y) : FAST-LIO 고장 주입 (init 기준 상대초). kill_after: init 뒤 이 초부터 FAST-LIO 없음."""
    from backup_core_v2 import make_estimator, match_correction
    ya, ax, ay = math.radians(A[0]), A[1], A[2]; ca, sa = math.cos(ya), math.sin(ya)
    T_true = (-(ca * ax + sa * ay), -(-sa * ax + ca * ay), -ya)
    def gt_i(ts): return min(max(bisect.bisect_left(tL, ts), 0), len(tL) - 1)
    est = make_estimator(mp)
    V = {}
    V['v1'] = dict(mux=MuxLogic(), al=None)
    V['C+D'] = dict(mux=MX_CD.GuardedMux(mux=MuxLogic(), guard=MX_CD.HealthGuard()), al=FA_CD.FrameAligner(), pol='cd')
    V['F'] = dict(mux=MX_F.GuardedMux(mux=MuxLogic(), guard=MX_F.HealthGuard()), al=FA_F.FrameAligner(), pol='f')
    for v in V.values():
        v.update(h='init', sw=None, reason='', last_pending=None, holds=0, clears=0, inj_at_sw=None)
    def pub_status(v, tt):
        hr = est.health(tt)
        if v['al'] is None:
            v['h'] = hr; return
        v['h'] = 'aligning' if (hr == 'ok' and v['al'].mask_health(tt)) else hr
        v['mux'].guard.on_status(tt, v['h'])
    def bk_xy(v):
        return (est.x, est.y) if v['al'] is None else v['al'].map_to_cam(est.x, est.y, est.yaw)[:2]
    cur_inj = [0.0]
    def after_mux(v, tt):
        g = v['mux']
        if v['al'] is None:
            mode = g.mode
        else:
            mode = g.mode
            for k, _ in g.pop_events():
                if k == 'hold': v['holds'] += 1
                if k == 'clear': v['clears'] += 1
            newly_pending = g.pending is not None and v['last_pending'] is None
            v['last_pending'] = g.pending
            if v['pol'] == 'f' and (newly_pending or mode == 'backup'):
                if v['al'].on_anomaly(tt, f"mux {'backup' if mode == 'backup' else 'pending'}"): pub_status(v, tt)
            if v['pol'] == 'cd' and mode == 'backup':
                if v['al'].on_backup_in_use(tt): pub_status(v, tt)
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
            x, y, z, yw = val
            if t_init is not None:
                rel = t - t_init
                if kill_after is not None and rel > kill_after:
                    t_kill = t_kill or t; continue
                if inj is not None:
                    x2, y2 = inj(rel, x, y); cur_inj[0] = math.hypot(x2 - x, y2 - y); x, y = x2, y2
            fl_last = (t, (x, y, z, yw))
            if est.inited:
                for v in V.values():
                    if v['al'] is not None:
                        m = v['al'].on_fastlio(t + latency, x, y, yw, stamp=t)
                        if m and v['pol'] == 'f': pub_status(v, t + latency)
                    v['mux'].on_fastlio(t + latency, x, y, z, ts=t)
                    after_mux(v, t + latency)
            continue
        if leg is None or t < t0 + start_rel: continue
        if not est.inited:
            if fl_last is None: continue
            fx, fy, fz, fyaw = fl_last[1]
            est.init(fx, fy, fz, fyaw, leg, t); t_init = t; t_tick = t
            for v in V.values():
                if v['al'] is not None and v['al'].on_init(t) and v['pol'] == 'f': pub_status(v, t)
        prep = est.prepare(val.astype(np.float64))
        if prep is None: continue
        T, rms = est.matcher.align(prep['P'], prep['T0']); cm, cr = match_correction(prep, T)
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
        for v in V.values():
            bx, by = bk_xy(v); e_bk = math.hypot(bx - gx, by - gy)
            on_bk = v['sw'] is not None
            dead = t_kill is not None and t >= t_kill
            # 플래너가 받는 /Odometry_mux 오차: 전환 뒤 = 백업, 전 = FAST-LIO(주입 오차). FAST-LIO 사망 + 미전환 = 정전(nan)
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
            o['정렬'] = al.state; o['정렬사유'] = al.why[:90]
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


# ═════════════════════ ② 전체 녹화, 정렬 고정 뒤 (D 틀) ═════════════════════
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
        """dead(rel)->bool : FAST-LIO 가 죽은 구간(정전 계산용)."""
        out = {}
        for ver in ('v1', 'C+D', 'F'):
            if ver == 'v1': g = MuxLogic()
            elif ver == 'C+D': g = MX_CD.GuardedMux(mux=MuxLogic(), guard=MX_CD.HealthGuard())
            else: g = MX_F.GuardedMux(mux=MuxLogic(), guard=MX_F.HealthGuard())
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
        # ── 검증 에이전트 (3)(4)
        run('★ FAST-LIO 사망(200초~) + 백업 degraded(195초~)', cut(200), health=lambda rel: 'degraded' if rel > 195 else None, dead=dead(200)),
        run('★ 끊김 200~203초 후 회복 + 백업 degraded(195~232초)', cut(200, 203), health=lambda rel: 'degraded' if 195 < rel < 232 else None, dead=dead(200, 203)),
        run('★ 끊김 200~203초 후 회복 + 백업 lost(195~232초) → ok', cut(200, 203), health=lambda rel: 'lost' if 195 < rel < 232 else None, dead=dead(200, 203)),
        run('★ 끊김 200~203초 후 회복 + /backup/status 끊김(195~232초)', cut(200, 203), status_alive=lambda rel: not (195 < rel < 232), dead=dead(200, 203)),
        # ── D 기존 시나리오 (회귀)
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
    ]
    return dict(S)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--part', default='1,2,3'); a = ap.parse_args()
    parts = set(a.part.split(',')); res = {}
    if '1' in parts or '3' in parts:
        from test_frame_align import load_events, SPLIT
        t_ = time.time()
        wins = [(0.0, 90.0)] + ([(SPLIT - 2, None)] if '3' in parts else [])
        evs, tL, GT, gyaw, t0 = load_events(tuple(wins)); print(f'bag 읽기 {time.time() - t_:.0f}s', flush=True)
        mp3, mp5, mp0 = rot_map(3.0, T3, T3), rot_map(5.0, T3, T3), load_map(HALF_NPZ)
    if '1' in parts:
        ev0 = evs[0]; R1 = {}
        drift = lambda a_, ton: (lambda rel, x, y: (x + a_ * max(0.0, rel - ton), y))
        jump = lambda d, ton: (lambda rel, x, y: (x + (d if rel > ton else 0.0), y))
        cases = [
            ('고정 전 표류 0.05 m/s (init+0초~), 지도 3°', (3.0, T3, T3), mp3, drift(0.05, 0.0), None),
            ('고정 전 표류 0.2 m/s (init+3초~), 지도 3°', (3.0, T3, T3), mp3, drift(0.2, 3.0), None),
            ('고정 전 표류 0.2 m/s (init+3초~), 지도 0°', (0.0, 0.0, 0.0), mp0, drift(0.2, 3.0), None),
            ('고정 전 점프 1.0 m (init+2초), 지도 3°', (3.0, T3, T3), mp3, jump(1.0, 2.0), None),
            ('고정 전 점프 1.0 m (init+5.5초), 지도 3°', (3.0, T3, T3), mp3, jump(1.0, 5.5), None),
            ('고정 전 점프 1.0 m (init+5.5초), 지도 0°', (0.0, 0.0, 0.0), mp0, jump(1.0, 5.5), None),
            ('고정 뒤 표류 0.2 m/s (init+20초~), 지도 3°', (3.0, T3, T3), mp3, drift(0.2, 20.0), None),
            ('고정 전 FAST-LIO 사망 (init+4초), 지도 3°', (3.0, T3, T3), mp3, None, 4.0),
            ('정상 (시작부 88초), 지도 3°', (3.0, T3, T3), mp3, None, None),
            ('정상 (시작부 88초), 지도 5°', (5.0, T3, T3), mp5, None, None),
        ]
        for nm, A, mp, inj, kill in cases:
            t_ = time.time(); R1[nm] = run3(nm, A, ev0, tL, GT, gyaw, t0, mp, inj=inj, kill_after=kill)
            print(f'① [{nm}] ({time.time() - t_:.0f}s)', json.dumps(R1[nm], ensure_ascii=False), flush=True)
        res['①시작부'] = R1
    if '3' in parts:
        ev1 = evs[-1]; R3 = {}
        for nm, A, mp in (('정상 주행 270~542초, 지도 3°', (3.0, T3, T3), mp3), ('정상 주행 270~542초, 지도 5°', (5.0, T3, T3), mp5)):
            t_ = time.time(); R3[nm] = run3(nm, A, ev1, tL, GT, gyaw, t0, mp, dur=None, start_rel=SPLIT)
            print(f'③ [{nm}] ({time.time() - t_:.0f}s)', json.dumps(R3[nm], ensure_ascii=False), flush=True)
        res['③오전환'] = R3
    if '2' in parts:
        res['②전체녹화'] = part2()
    out = os.path.join(HERE, f"result_f_{a.part.replace(',', '')}.json")
    json.dump(res, open(out, 'w'), ensure_ascii=False, indent=1); print('저장', out)


if __name__ == '__main__':
    main()
