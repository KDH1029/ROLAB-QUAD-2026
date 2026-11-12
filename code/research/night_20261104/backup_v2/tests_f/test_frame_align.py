#!/usr/bin/env python3
"""frame_align(T_cam_map) 시험 — 오프라인, ROS 실행 없음 (rosbag2_py 로 bag 파일만 읽음).

① 단위 시험 (--unit, 몇 초): Umeyama 복원, 서 있기만 해도 yaw 결정, FAST-LIO 없음 → 항등 고정, 추정 중 끊김 → 조기 고정
② 녹화본 시험 (기본): 앞 270초 FAST-LIO 지도(backup_eval/map_first270s_fastlio.npz)를 강체 변환 A 로 돌려 '다른 날 지도'를 흉내
   (지도 = A · 오늘 camera_init) → 뒤 270~542초를 backup_core_v2.make_estimator 로, 노드와 같은 씨앗(FAST-LIO 첫 자세)·같은 정렬 논리로 돌린다.
   정답 T_cam_map = A⁻¹. 잰다: T 오차, 정렬 후/안 했을 때 |백업(camera_init) − FAST-LIO| p95·최대(0.5 m 문턱 대비), 고정까지 시간,
   전환 장치(backup_core.MuxLogic) 오전환 모의(정렬 없음 / 정렬+추정 중 health 가림 / 정렬만), 씨앗 한계(큰 오프셋에서 수렴하나).
기준 궤적·시각 대응(tL/GT)은 backup_eval/split_validate_gicp_var.py 그대로.
실행:  source /opt/ros/jazzy/setup.bash && ~/ws/night_20261104/venv/bin/python ~/ws/night_20261104/backup_v2/tests/test_frame_align.py [--unit] [--quick]
쓰기:  이 폴더(tests/)만 — result_frame_align.json, map_first270s_rot3deg.npz
"""
import sys, os, math, bisect, json, time, argparse
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); V2 = os.path.dirname(HERE)
sys.path.insert(0, V2)
from frame_align import FrameAligner, umeyama2d, wrap

BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
HALF_NPZ = '/home/ilovehan9283/ws/night_20261104/backup_eval/map_first270s_fastlio.npz'
SPLIT = 270.0


# ═════════════════════════════════ ① 단위 시험 ═════════════════════════════════
def unit():
    rng = np.random.default_rng(1); ok = True
    def chk(name, cond, info=''):
        nonlocal ok; ok &= bool(cond); print(('  통과 ' if cond else '  ★실패 ') + name, info)
    # Umeyama: 정답 (0.4, −0.2, 3°), 위치 잡음 3 cm, 방향 잡음 0.3°
    th = math.radians(3.0); R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]]); t = np.array([0.4, -0.2])
    P = rng.uniform(-3, 3, (80, 2)); Q = P @ R.T + t + rng.normal(0, 0.03, P.shape)
    yp = rng.uniform(-3, 3, 80); yq = yp + th + rng.normal(0, math.radians(0.3), 80)
    e = umeyama2d(P, Q, yp, yq)
    chk('Umeyama 복원', abs(e[0] - 0.4) < 0.02 and abs(e[1] + 0.2) < 0.02 and abs(math.degrees(e[2]) - 3) < 0.1, f'{np.round(e[:2], 3)} {math.degrees(e[2]):.2f}°')
    # 서 있기만 함(점 한 곳): 방향 쌍으로 yaw 결정
    P1 = np.tile([[5.0, 2.0]], (40, 1)) + rng.normal(0, 0.01, (40, 2)); Q1 = P1 @ R.T + t
    e1 = umeyama2d(P1, Q1, np.full(40, 0.7), np.full(40, 0.7 + th))
    chk('정지 상태 yaw', abs(math.degrees(e1[2]) - 3) < 0.05, f'{math.degrees(e1[2]):.2f}°')
    # 정렬기: 정지 + 연속 수락 → estimating → frozen
    al = FrameAligner(); al.on_init(0.0)
    for k in range(100):
        tt = k * 0.01; al.on_backup_pose(tt, 5.0, 2.0, 0.7)
    for k in range(10): al.on_match(0.1 * k, True, 0.01, 0.001)
    for k in range(100):                                             # FAST-LIO 20 Hz × 5 s, 같은 시계 stamp
        tt = 1.0 + k * 0.05; qx, qy = R @ [5.0, 2.0] + t
        al.on_fastlio(tt + 0.008, qx, qy, 0.7 + th, stamp=tt); al.on_match(tt, True, 0.01, 0.0)
        al.on_backup_pose(tt + 0.01, 5.0, 2.0, 0.7)
    msg = al.tick(6.0)
    chk('정지 5초 → 고정', al.state == 'frozen' and abs(math.degrees(al.T[2]) - 3) < 0.05, f'{al.state} {al.T} {msg}')
    x, y, yy = al.map_to_cam(5.0, 2.0, 0.7)
    chk('map_to_cam', abs(x - (R @ [5, 2] + t)[0]) < 1e-3 and abs(y - (R @ [5, 2] + t)[1]) < 1e-3)
    Pc = al.points_to_cam(np.array([[5.0, 2.0, 1.0]]))
    chk('points_to_cam(z 보존)', abs(Pc[0, 2] - 1.0) < 1e-9 and abs(Pc[0, 0] - x) < 1e-6)
    # FAST-LIO 처음부터 없음 → 항등 고정 + 경고
    a2 = FrameAligner(no_fastlio_s=8.0); a2.on_init(0.0); a2.tick(5.0); s5 = a2.state; m = a2.tick(8.5)
    chk('FAST-LIO 없음 → identity_frozen', s5 == 'identity' and a2.state == 'identity_frozen' and a2.T == (0, 0, 0) and a2.warn, f'{m}')
    # 추정 중 끊김: 쌍 충분·잔차 OK → 조기 고정 / 쌍 없음 → 항등
    a3 = FrameAligner(); a3.on_init(0.0)
    for k in range(300): a3.on_backup_pose(k * 0.01, 1.0 + 0.003 * k, 0.0, 0.0)
    for k in range(12): a3.on_match(k * 0.1, True, 0.0, 0.0)
    for k in range(20):
        tt = 1.2 + k * 0.05; bx = 1.0 + 0.3 * tt
        q = R @ [bx, 0.0] + t; a3.on_fastlio(tt + 0.008, q[0], q[1], th, stamp=tt); a3.on_match(tt, True, 0, 0)
    a3.tick(2.5); m3 = a3.tick(3.5)
    chk('추정 중 끊김 → 조기 고정', a3.state == 'frozen' and a3.warn is not None and abs(math.degrees(a3.T[2]) - 3) < 0.2, f'{m3} T={np.round(a3.T, 3)}')
    a4 = FrameAligner(); a4.on_init(0.0); a4.on_fastlio(0.5, 0, 0, 0, stamp=0.49); s2 = (a4.tick(2.0), a4.state); m4 = a4.tick(3.6)
    chk('쌍 없이 끊김 → 3초 뒤 항등 고정(그 전엔 대기)', s2 == (None, 'identity') and a4.state == 'identity_frozen', f'{m4}')
    # 전환 장치가 백업 사용 시작 → 즉시 고정 / FAST-LIO 끊기면 가림 풂
    a6 = FrameAligner(); a6.on_init(0.0); a6.on_fastlio(1.0, 0, 0, 0, stamp=0.99)
    g1 = a6.mask_health(1.2); g2 = a6.mask_health(1.6); m6 = a6.on_backup_in_use(1.7)
    chk('가림: FAST-LIO 살아 있을 때만 / 백업 사용 → 즉시 고정', g1 and not g2 and a6.state == 'identity_frozen', f'{m6}')
    # 수렴 중(보정량 큼) 수락은 연속으로 안 침
    a5 = FrameAligner(); a5.on_init(0.0)
    for k in range(20): a5.on_match(k * 0.2, True, 0.10, 0.0)
    chk('보정량 큰 수락은 streak 0', a5.streak == 0)
    # 노드 콜백 예외 잡기 (rclpy import 만 — DDS 참여 없음)
    try:
        from backup_loc_node_v2 import safe_cb
        class _L:
            def __init__(s): s.msgs = []
            def error(s, m): s.msgs.append(m)
        class _N:
            def __init__(s): s.n_exc, s.t_exc_log, s._l = {}, {}, _L()
            def get_logger(s): return s._l
            @safe_cb
            def boom(s, x): return 1 / x
        n = _N(); r1 = n.boom(2); r0 = n.boom(0); n.boom(0)
        chk('콜백 예외 → 로그만, 계속', r1 == 0.5 and r0 is None and n.n_exc.get('boom') == 2 and len(n._l.msgs) == 1 and 'Traceback' in n._l.msgs[0])
    except ImportError as e:
        print('  (노드 import 불가 — rclpy 환경 아님, 건너뜀)', e)
    print('단위 시험', '전부 통과' if ok else '★실패 있음', flush=True); return ok


# ═════════════════════════════════ ② 녹화본 시험 ═════════════════════════════════
def make_rotated_npz(src, yaw_deg, tx, ty, out):
    """지도(npz: vox,cnt,V) 를 p' = R(yaw)·p + (tx,ty) 로 돌려 같은 형식으로 저장 — load_map 이 그대로 읽는다. cnt 는 합침."""
    d = np.load(src); V = float(d['V'])
    P = (d['vox'].astype(np.float64) + 0.5) * V
    th = math.radians(yaw_deg); c, s = math.cos(th), math.sin(th)
    Q = np.empty_like(P); Q[:, 0] = c * P[:, 0] - s * P[:, 1] + tx; Q[:, 1] = s * P[:, 0] + c * P[:, 1] + ty; Q[:, 2] = P[:, 2]
    k = np.floor(Q / V).astype(np.int32)
    uk, inv = np.unique(k, axis=0, return_inverse=True)
    cnt = np.bincount(inv.ravel(), weights=d['cnt'].astype(np.float64)).astype(d['cnt'].dtype)
    np.savez_compressed(out, vox=uk, cnt=cnt, V=d['V'], A_yaw_deg=yaw_deg, A_t=np.array([tx, ty]),
                        note='p_map = R(A_yaw)·p_camera_init(10-30) + A_t ; 정답 T_cam_map = A^-1')
    return out


def load_events(windows=((SPLIT - 2, None),)):
    """windows: (시작 상대초, 끝 상대초|None) 목록 → 창마다 사건 목록. hdr→bag 시각 대응은 한 번만."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    import sensor_msgs_py.point_cloud2 as pc2
    sys.path.insert(1, os.path.expanduser('~/ws/v_gosu/backup'))
    from backup_core import q2R, rpy_of

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
    t0 = tL[0]
    gyaw = np.array([rpy_of(q2R(*g[5:9]))[2] for g in GT])
    evs = []
    for w0, w1 in windows:
        t_lo = t0 + w0; t_hi = tL[-1] if w1 is None else min(tL[-1], t0 + w1)
        ev = []; last = -1e9
        r = reader(['/utlidar/robot_odom', '/lidar_points'])
        while r.has_next():
            tp, raw, t = r.read_next(); t *= 1e-9
            if t < t_lo: continue
            if t > t_hi: break
            if tp == '/utlidar/robot_odom':
                m = deserialize_message(raw, Odometry); p, q = m.pose.pose.position, m.pose.pose.orientation
                rr_, pp, yy = rpy_of(q2R(q.x, q.y, q.z, q.w)); ev.append((t, 0, (p.x, p.y, p.z, yy, rr_, pp)))
            elif t - last >= 0.2 - 1e-3:                                   # 노드와 같은 5 Hz
                last = t; m = deserialize_message(raw, PointCloud2)
                ev.append((t, 1, pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float32)))
        for i in np.where((tL >= t_lo) & (tL <= t_hi))[0]:    # FAST-LIO 20 Hz (stamp = tL, 받은 시각 = tL + 8.5 ms 실측 지연)
            ev.append((tL[i], 2, (GT[i, 2], GT[i, 3], GT[i, 4], gyaw[i])))
        ev.sort(key=lambda e: (e[0], e[1])); evs.append(ev)
    return evs, tL, GT, gyaw, t0


def run_case(name, A, ev, tL, GT, gyaw, t0, map_pts, dur=None, use_stamp=True, latency=0.0085, start_rel=SPLIT, ground_start=False, kill_after=None):
    """kill_after: init 뒤 이 초에 FAST-LIO 를 끊는다(정렬기·전환 장치에 더 안 줌. 정답 비교용 GT 는 계속 씀)."""
    from backup_core_v2 import make_estimator, match_correction
    from backup_core import MuxLogic
    ya, ax, ay = math.radians(A[0]), A[1], A[2]; ca, sa = math.cos(ya), math.sin(ya)
    to_map = lambda x, y, yaw: (ca * x - sa * y + ax, sa * x + ca * y + ay, wrap(yaw + ya))     # 정답 지도 좌표
    T_true = (-(ca * ax + sa * ay), -(-sa * ax + ca * ay), -ya)                                # A⁻¹
    def gt_i(ts): return min(max(bisect.bisect_left(tL, ts), 0), len(tL) - 1)
    t_build = time.perf_counter(); est = make_estimator(map_pts); t_build = time.perf_counter() - t_build
    if ground_start:   # 지면표 바닥 기준을 '실제로 로봇이 서 있던 곳'(오늘 출발점 = 지도 좌표 A(0))으로 — 큰 이동 모의에서 지도 원점이 바닥이 아닌 문제 제거
        import inspect
        from robust_ground import RobustGroundGrid
        from backup_core import voxel
        if 'start_xy' in inspect.signature(RobustGroundGrid).parameters:
            est.ground = RobustGroundGrid(voxel(map_pts, 0.05), start_xy=(ax, ay))
    al = FrameAligner()
    # 전환 장치 모의 — 노드 /backup/status 는 1 Hz(아래 report 시각에 health 갱신), 전환 장치 tick 은 다리 사건 7개마다(≈21 Hz)
    #   정렬없음: 예전(지도 좌표 그대로, health 그대로)  정렬만: T 적용, 가림 없음  정렬+가림: T 적용 + 고정 전 'aligning'
    #   D_v2: night backup_v2/odom_mux_node_v2.GuardedMux(백업 ok 일 때만 전환) + T 적용 + 가림 = 실제 v2 런치 구성
    mux = {k: MuxLogic() for k in ('정렬없음', '정렬만', '정렬+가림')}
    try:
        from odom_mux_node_v2 import GuardedMux, HealthGuard
        mux['D_v2+정렬+가림'] = GuardedMux(guard=HealthGuard())
    except Exception as e:      # noqa: BLE001
        print('  (odom_mux_node_v2 import 실패 — D_v2 모의 생략)', e)
    hpub = {'raw': 'init', 'mask': 'init'}; t_sw = {}; n_leg = 0; killed = False; t_kill = None
    hook_k = 'D_v2+정렬+가림' if 'D_v2+정렬+가림' in mux else '정렬+가림'
    def pub_status(tt):
        hr = est.health(tt); hpub['raw'] = hr; hpub['mask'] = 'aligning' if (hr == 'ok' and al.mask_health(tt)) else hr
        if 'D_v2+정렬+가림' in mux: mux['D_v2+정렬+가림'].guard.on_status(tt, hpub['mask'])
    leg = None; t_init = None; seed = None; rows = []; t_tick = None; fl_last = None; ms = []
    def bk_xy(k):
        return (est.x, est.y) if k == '정렬없음' else al.map_to_cam(est.x, est.y, est.yaw)[:2]
    def mux_tick(tt):
        for k, mx in mux.items():
            if k.startswith('D_v2'):
                mx.guard.on_odom(tt); m_ = mx.tick(tt, bk_xy(k))
            else:
                m_ = mx.tick(tt, bk_xy(k), hpub['raw'] if k in ('정렬없음', '정렬만') else hpub['mask'])
            if m_ == 'backup' and k not in t_sw:
                t_sw[k] = (tt, mx.mux.reason if k.startswith('D_v2') else mx.reason)
                if k == hook_k and al.on_backup_in_use(tt):                # 노드 on_mux_status 와 같은 동작(실제 구성 = D_v2)
                    pub_status(tt)
    for t, kind, val in ev:
        if dur is not None and t_init is not None and t > t_init + dur: break
        if kind == 0:
            leg = val
            if est.inited:
                est.on_leg(val); al.on_backup_pose(t, est.x, est.y, est.yaw)
                n_leg += 1
                if n_leg % 7 == 0: mux_tick(t)
            continue
        if kind == 2:
            if kill_after is not None and t_init is not None and t > t_init + kill_after:
                if not killed: killed, t_kill = True, t
                continue
            fl_last = (t, val)
            if est.inited:
                al.on_fastlio(t + latency, val[0], val[1], val[3], stamp=t if use_stamp else None)
                for k, mx in mux.items():
                    mx.on_fastlio(t + latency, val[0], val[1], val[2], ts=t)
            continue
        # 스캔 5 Hz
        if leg is None or t < t0 + start_rel: continue
        if not est.inited:
            if fl_last is None: continue
            fx, fy, fz, fyaw = fl_last[1]
            est.init(fx, fy, fz, fyaw, leg, t); al.on_init(t); t_init = t; t_tick = t   # 노드와 같은 씨앗: FAST-LIO 자세를 지도 좌표 씨앗으로
            sx, sy, syaw = to_map(fx, fy, fyaw); seed = (math.hypot(sx - fx, sy - fy), math.degrees(wrap(syaw - fyaw)))
        _t = time.perf_counter()
        prep = est.prepare(val.astype(np.float64))
        if prep is None: continue
        T, rms = est.matcher.align(prep['P'], prep['T0'])
        cm, cr = match_correction(prep, T)
        res = est.apply(prep, T, rms, t); ms.append((time.perf_counter() - _t) * 1000)
        al.on_match(t, res['accepted'], cm, cr); al.on_backup_pose(t, est.x, est.y, est.yaw)
        if al.tick(t):                                                     # 노드 on_cloud 와 같이 정합마다 판단, 고정 순간 상태 바로 발행
            pub_status(t)
        while t_tick is not None and t >= t_tick + 1.0:                    # 노드 report 타이머 1 Hz
            t_tick += 1.0; al.tick(t_tick); pub_status(t_tick)
        i = gt_i(t); gx, gy, gyw = GT[i, 2], GT[i, 3], gyaw[i]
        mx_, my_, myaw = to_map(gx, gy, gyw)
        cx, cy, cyaw = al.map_to_cam(est.x, est.y, est.yaw)
        rows.append((t - t_init, math.hypot(est.x - mx_, est.y - my_), math.degrees(wrap(est.yaw - myaw)),  # 지도좌표 추정 오차
                     math.hypot(cx - gx, cy - gy),                       # 낸 값(정렬 적용, 고정 전 항등) − FAST-LIO
                     math.hypot(est.x - gx, est.y - gy),                 # 정렬 안 했을 때(항등) − FAST-LIO
                     1.0 if al.frozen() else 0.0, res['accepted'], t))
    R_ = np.array(rows); st = al.status()
    out = dict(지도변환=dict(yaw_deg=A[0], tx=A[1], ty=A[2]), 시험초=round(float(R_[-1, 0]), 1), 지도적재s=round(t_build, 1),
               씨앗오차=dict(m=round(seed[0], 3), deg=round(seed[1], 2)), 정합=len(R_), 수락=est.n_ok, 거부=est.n_bad,
               정합ms_중앙=round(float(np.median(ms)), 1), 정렬상태=st['state'], 정렬사유=st['why'])
    conv = np.where((R_[:, 1] < 0.10) & (np.abs(R_[:, 2]) < 1.0))[0]
    out['백업수렴s'] = round(float(R_[conv[0], 0]), 1) if len(conv) else None
    out['지도좌표오차_수렴후_p95'] = round(float(np.percentile(R_[conv[0]:, 1], 95)), 3) if len(conv) else None
    if al.t_frozen is not None:
        out['고정까지s'] = round(al.t_frozen - t_init, 1)
    if st['state'] == 'frozen':
        tx, ty, th = al.T
        dT = (tx - T_true[0], ty - T_true[1], math.degrees(wrap(th - T_true[2])))
        # 코스 전체에서 T 오차가 만드는 위치 차 최대 (FAST-LIO 궤적 점에서)
        P = GT[:, 2:4]; c1, s1 = math.cos(th), math.sin(th); c0, s0 = math.cos(T_true[2]), math.sin(T_true[2])
        Mx, My = ca * P[:, 0] - sa * P[:, 1] + ax, sa * P[:, 0] + ca * P[:, 1] + ay
        e_course = np.hypot(c1 * Mx - s1 * My + tx - (c0 * Mx - s0 * My + T_true[0]), s1 * Mx + c1 * My + ty - (s0 * Mx + c0 * My + T_true[1]))
        out['T추정'] = dict(x=round(tx, 3), y=round(ty, 3), yaw_deg=round(math.degrees(th), 3))
        out['T정답'] = dict(x=round(T_true[0], 3), y=round(T_true[1], 3), yaw_deg=round(math.degrees(T_true[2]), 3))
        out['T오차'] = dict(xy_m=round(math.hypot(dT[0], dT[1]), 4), yaw_deg=round(dT[2], 3), 코스최대위치차_m=round(float(e_course.max()), 3))
        out['잔차'] = st.get('res'); out['쌍'] = st['pairs']; out['회전중_버린쌍'] = st.get('turn_skip')
    fz = R_[:, 5] > 0.5
    def stats(e):
        return dict(중앙=round(float(np.median(e)), 3), p95=round(float(np.percentile(e, 95)), 3), 최대=round(float(e.max()), 3),
                    초과05=round(float((e > 0.5).mean()), 3)) if len(e) else None
    out['정렬후_고정뒤'] = stats(R_[fz, 3])
    out['정렬안함_같은구간'] = stats(R_[fz, 4]) if fz.any() else stats(R_[:, 4])
    out['정렬안함_전체'] = stats(R_[:, 4])
    out['고정전_항등출력_최대'] = round(float(R_[~fz, 3].max()), 3) if (~fz).any() else None
    out['정렬status'] = st
    out['전환장치모의'] = {k: (f"{t_sw[k][0] - t_init:.1f}s 전환 — {t_sw[k][1]}" if k in t_sw else '전환 없음') for k in mux}
    if killed:
        out['FAST-LIO끊은시각s'] = round(t_kill - t_init, 1)
        sw = {}
        for k in mux:
            if k not in t_sw: sw[k] = '전환 안 됨'; continue
            ts_ = t_sw[k][0]; after = R_[:, 7] >= ts_
            e = R_[after, 4] if k == '정렬없음' else R_[after, 3]          # 전환 뒤 플래너가 받는 값 − 정답
            sw[k] = dict(전환지연s=round(ts_ - t_kill, 2), 전환뒤오차=stats(e))
        out['끊김뒤'] = sw
    np.save(os.path.join(HERE, f'traj_{name}.npy'), R_)
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--unit', action='store_true'); ap.add_argument('--quick', action='store_true')
    ap.add_argument('--tag', default=''); ap.add_argument('--only', default='', help='쉼표로 고른 경우만 (예: kill4_y3_t03,start0_kill4_y3_t03)')
    a = ap.parse_args()
    if not unit():
        sys.exit(1)
    if a.unit:
        return
    sys.path.insert(1, os.path.expanduser('~/ws/v_gosu/backup'))
    from backup_core import load_map
    t_ = time.time(); (ev, ev_start), tL, GT, gyaw, t0 = load_events(((SPLIT - 2, None), (0.0, 90.0)))
    print(f'bag 읽기 {time.time() - t_:.0f}s, 사건 {len(ev)} + 시작부 {len(ev_start)}', flush=True)
    rot3 = make_rotated_npz(HALF_NPZ, 3.0, 0.3 / math.sqrt(2), 0.3 / math.sqrt(2), os.path.join(HERE, 'map_first270s_rot3deg.npz'))
    # (이름, (yaw°, tx, ty), 길이 s|None=끝까지, stamp 사용)
    t3 = 0.3 / math.sqrt(2)
    # 시작부(0~90 s: 50초 정지 후 출발) — 실전과 같은 씨앗(FAST-LIO≈원점 → 지도 오차 = A 그 자체). 지도가 이 구간을 포함하므로 정합은 낙관, 정렬 논리(정지 중 고정) 확인용
    res = {}
    mp3 = load_map(rot3)
    only = set(x for x in a.only.split(',') if x)
    for nm, kl in (('start0_y3_t03', None), ('start0_kill4_y3_t03', 4.0)):   # 정지 출발, 정렬 중(4초) FAST-LIO 끊김
        if only and nm not in only: continue
        t_ = time.time(); res[nm] = run_case(nm, (3.0, t3, t3), ev_start, tL, GT, gyaw, t0, mp3, dur=88.0, start_rel=0.0, kill_after=kl)
        print(f'[{nm}] {time.time() - t_:.0f}s', json.dumps(res[nm], ensure_ascii=False), flush=True)
    cases = [('ctrl_0', (0.0, 0.0, 0.0), None, True),
             ('y2_t03', (2.0, t3, t3), None, True),
             ('y3_t03', (3.0, t3, t3), None, True),
             ('y5_t03', (5.0, t3, t3), None, True),
             ('y3_t03_nostamp', (3.0, t3, t3), 60.0, False),           # 재생(stamp 가 녹화 시계) 경로: 받은 시각 − 기본 지연
             # FAST-LIO 를 끊는다 — 정렬 중(4초)·고정 뒤(30초). 전환 지연과 전환 뒤 플래너가 받는 위치 오차
             ('kill4_y3_t03', (3.0, t3, t3), 90.0, True, 4.0),
             ('kill30_ctrl', (0.0, 0.0, 0.0), None, True, 30.0),
             ('kill30_y3_t03', (3.0, t3, t3), None, True, 30.0),
             ('kill30_y5_t03', (5.0, t3, t3), None, True, 30.0)]
    if not a.quick:   # 씨앗 한계 — 수렴만 보면 되므로 60초
        for yd, tm in ((5, 0.5), (8, 0.5), (10, 0.5), (15, 0.5), (20, 0.3), (30, 0.3), (45, 0.3), (3, 1.0), (3, 1.5), (3, 2.0), (3, 3.0), (3, 4.0)):
            cases.append((f'seed_y{yd}_t{tm}', (float(yd), tm / math.sqrt(2), tm / math.sqrt(2)), 60.0, True))
        for tm in (3.0, 4.0):   # 지도 원점이 바닥이 아닌 모의 결함 제거판 (지면표 기준 = 실제 출발점)
            cases.append((f'seed_y3_t{tm}_floorstart', (3.0, tm / math.sqrt(2), tm / math.sqrt(2)), 60.0, True))
    for name, A, dur, us, *kl in cases:
        if only and name not in only: continue
        if A == (3.0, t3, t3):
            mp = load_map(rot3)
        else:
            tmp = os.path.join(HERE, f'_tmp_{name}.npz'); make_rotated_npz(HALF_NPZ, *A, tmp); mp = load_map(tmp); os.remove(tmp)
        t_ = time.time(); res[name] = run_case(name, A, ev, tL, GT, gyaw, t0, mp, dur=dur, use_stamp=us, ground_start=name.endswith('_floorstart'), kill_after=kl[0] if kl else None)
        print(f'[{name}] {time.time() - t_:.0f}s', json.dumps(res[name], ensure_ascii=False), flush=True)
    out = os.path.join(HERE, f'result_frame_align{a.tag}.json')
    json.dump(res, open(out, 'w'), ensure_ascii=False, indent=1); print('저장', out)


if __name__ == '__main__':
    main()
