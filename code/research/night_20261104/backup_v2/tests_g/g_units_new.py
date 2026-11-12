# [G] 새 단위 항목 — test_g_units.py 가 exec 로 부른다 (chk, mux_run, feed_aligner, FrameAligner, GuardedMux, HealthGuard, MuxLogic 공유)
import math as _m

def feedT(T, fl_fn_rel, t1=12.0, v=0.0, al=None, cut_after=None, **kw):
    """정답 T(cam<-map). 로봇은 지도 x 방향 v m/s 직진(정지면 v=0). 백업 150 Hz(정답), 정합 5 Hz 수락, FAST-LIO 20 Hz = T(정답) + fl_fn_rel(t)."""
    al = al or FrameAligner(**kw); al.on_init(0.0); tx, ty, tth = T; c, s = _m.cos(tth), _m.sin(tth); out = []
    for k in range(int(t1 * 300)):
        t = k / 300.0; bx, by, byaw = 5.0 + v * t, 2.0, 0.0
        if k % 2 == 0: al.on_backup_pose(t, bx, by, byaw)
        if k % 60 == 0:
            al.on_match(t, True, 0.01, 0.0); r = al.tick(t)
            if r: out.append((round(t, 2), r))
        if k % 15 == 0 and (cut_after is None or t < cut_after):
            dx = fl_fn_rel(t)
            qx_, qy_ = c * bx - s * by + tx + dx, s * bx + c * by + ty
            r = al.on_fastlio(t + 0.008, qx_, qy_, byaw + tth, stamp=t)
            if r: out.append((round(t, 2), r))
    return al, out

def terr(al, T):
    return _m.hypot(al.T[0] - T[0], al.T[1] - T[1]), _m.degrees(al.T[2] - T[2])

T03 = (0.3 / _m.sqrt(2), 0.3 / _m.sqrt(2), _m.radians(3.0))
print('── G①) 점프 = 튄 상태가 0.3초 넘게 이어질 때만')
for d in (0.4, 0.7, 1.0):
    for ts in (0.5, 1.0, 2.0, 3.0):
        al, _ = feedT(T03, lambda t, d=d, ts=ts: d if ts <= t < ts + 0.05 else 0.0)
        e = terr(al, T03)
        chk(f"한 메시지 튐 {d} m (init+{ts}) → 그 메시지만 버리고 정상 고정", al.state == 'frozen' and al.n_jump == 0 and al.n_spike_drop == 1 and e[0] < 0.02,
            f"{al.state} T오차 {e[0]:.3f} m 버림 {al.n_spike_drop}")
al, _ = feedT(T03, lambda t: 1.0 if 1.0 <= t < 1.1 else 0.0)
chk("두 메시지 튐 1.0 m (0.1초) → 정상 고정", al.state == 'frozen' and al.n_jump == 0 and al.n_spike_drop == 2 and terr(al, T03)[0] < 0.02, al.why)
al, _ = feedT(T03, lambda t: 1.0 if 1.0 <= t < 1.27 else 0.0)
chk("튐 0.25초(6 메시지) → 아직 이상 아님, 정상 고정", al.state == 'frozen' and al.n_jump == 0 and terr(al, T03)[0] < 0.02, f'버림 {al.n_spike_drop}')
al, ev = feedT(T03, lambda t: 1.0 if t >= 3.0 else 0.0)
chk("지속 점프 1.0 m (init+3초, 쌍 ≥3) → 0.3초 뒤 이상 → 점프 전 쌍으로 즉시 고정", al.state == 'frozen' and al.n_jump == 1 and terr(al, T03)[0] < 0.02
    and 3.29 < al.t_frozen < 3.4, f"{al.t_frozen:.2f}s {al.why[:60]}")
al, ev = feedT(T03, lambda t: 1.0 if t >= 1.0 else 0.0)
chk("지속 점프 1.0 m (init+1초, 쌍 0개) → 항등 고정 안 함·계속 모음 → T 상한 밖 → align_failed", al.state == 'align_failed' and al.n_anomaly_wait == 1, al.why[:90])
al, ev = feedT(T03, lambda t: 1.0 if t >= 1.0 else 0.0, max_offset_m=0.0)
chk("  (상한 끔 대조) 같은 경우 → 점프 뒤 FAST-LIO 기준으로 고정(T 가 1 m 흡수 = 상한이 잡는 몫)", al.state == 'frozen' and abs(terr(al, T03)[0] - 1.0) < 0.05, f'{terr(al, T03)}')
al, _ = feedT(T03, lambda t: 0.0, v=0.6)
chk("정상 직진 0.6 m/s → 튐 의심 0, 정상 고정", al.state == 'frozen' and al.n_spike_drop == 0 and al.n_jump == 0 and terr(al, T03)[0] < 0.02, al.why)
al, _ = feedT(T03, lambda t: 1.0 if 2.0 <= t < 2.05 else 0.0, v=0.6)
chk("직진 0.6 m/s 중 한 메시지 튐 1.0 m → 정상 고정", al.state == 'frozen' and al.n_jump == 0 and al.n_spike_drop == 1 and terr(al, T03)[0] < 0.02, al.why)
al, _ = feedT(T03, lambda t: 1.0 if t >= 3.0 else 0.0, v=0.6)
chk("직진 0.6 m/s 중 지속 점프 1.0 m (init+3) → 즉시 고정(점프 전 쌍)", al.state == 'frozen' and al.n_jump == 1 and terr(al, T03)[0] < 0.03, f"{al.t_frozen:.2f}")
al = FrameAligner(); al.on_init(0.0); r = al.on_anomaly(0.5, 'mux pending'); s1 = al.state
r2 = al.on_anomaly(0.6, 'mux backup', in_use=True)
chk("쌍 0개: 보류(in_use=False) → 계속 모음 / 백업 사용(in_use=True) → 즉시 항등 고정", s1 == 'identity' and r is None and al.state == 'identity_frozen' and r2, f'{r2}')
al = FrameAligner(); al.on_init(0.0); al.on_anomaly(0.5, 'mux pending')
al, _ = feedT(T03, lambda t: 0.0, al=al)
chk("보류 신호(쌍 0개) 뒤 정상 → 보통 고정", al.state == 'frozen' and terr(al, T03)[0] < 0.02, al.why)

print('── G④) 출발 오차 상한 (0.6 m · 8°)')
for nm, T in (('0.3 m·3°', T03), ('0.3 m·5°', (T03[0], T03[1], _m.radians(5.0))), ('0.5 m·7°', (0.5, 0.0, _m.radians(7.0)))):
    al, _ = feedT(T, lambda t: 0.0)
    chk(f"정상 출발 오차 {nm} → frozen (오탐 없음)", al.state == 'frozen' and terr(al, T)[0] < 0.02, al.why)
for nm, T in (('0.7 m·0°', (0.7, 0.0, 0.0)), ('0.1 m·9°', (0.1, 0.0, _m.radians(9.0))), ('0.3 m·-9°', (T03[0], T03[1], _m.radians(-9.0)))):
    al, _ = feedT(T, lambda t: 0.0)
    chk(f"T {nm} → align_failed (항등·가림 풂)", al.state == 'align_failed' and al.T == (0.0, 0.0, 0.0) and not al.mask_health(12.0) and al.warn, al.why[-60:])
al, _ = feedT((0.7, 0.0, 0.0), lambda t: 0.0, max_offset_m=0.0)
chk("max_offset_m=0 → 상한 끔(0.7 m 그대로 고정)", al.state == 'frozen', al.why)
al, _ = feedT((0.7, 0.0, 0.0), lambda t: 0.0, cut_after=3.0)
chk("끊김 조기 고정 경로도 상한 적용 (T 0.7 m, init+3초 끊김) → align_failed", al.state == 'align_failed', al.why[-70:])
al, _ = feedT((0.7, 0.0, 0.0), lambda t: 0.0, t1=3.0); r = al.on_anomaly(3.0, 'mux backup', in_use=True)
chk("이상 즉시 고정 경로도 상한 적용 (T 0.7 m, 백업 사용) → align_failed", al.state == 'align_failed' and r, (r or '')[-60:])
al, _ = feedT(T03, lambda t: 0.8 * min(1.0, max(0.0, (t - 0.5) / 2.0)))
chk("N1형: init+0.5~2.5초 매끈 이동 0.8 m (쌍 생기기 전) → T 상한 밖 → align_failed", al.state == 'align_failed', al.why[-70:])

print('── G②) 보류 해제: 무소식 사유면 degraded·aligning 에서도')
def u8b(resume_h, st_gap=(8.0, 16.0), fl_gap=(12.0, 12.7), fl_off=0.0):
    gm = GuardedMux(mux=MuxLogic(), guard=HealthGuard()); sw = None; k = 0; t = 0.0
    while t < 25:
        k += 1; t = k * 0.002; x = 0.4 * t
        h = resume_h if st_gap[1] <= t < st_gap[1] + 3 else 'ok'
        if k % 500 == 0 and not (st_gap[0] <= t < st_gap[1]): gm.guard.on_status(t, h)
        if k % 10 == 0: gm.guard.on_odom(t)
        if k % 25 == 0 and not (fl_gap[0] <= t < fl_gap[1]): gm.on_fastlio(t + 0.008, x + (fl_off if t >= fl_gap[1] else 0.0), 0.0, 0.0, ts=t)
        if k % 5 == 0: gm.tick(t, (x, 0.0))
        gm.pop_events()
        if gm.mode == 'backup' and sw is None: sw = round(t, 3)
    return gm, sw
for rh in ('ok', 'degraded', 'aligning'):
    gm, sw = u8b(rh)
    chk(f"U8b /backup/status 끊김 8~16초 + FAST-LIO 끊김 12.0~12.7초, 16초 재개 health={rh} → 해제·전환 없음", sw is None and gm.n_clear == 1, f'{sw}')
gm, sw = u8b('lost')
chk("  같은데 16초 재개 health=lost → 보류 유지, 19초 ok 로 돌면 해제", sw is None and gm.n_clear == 1, f'{sw}')
gm, sw = u8b('ok', fl_off=1.0)
chk("  같은데 FAST-LIO 가 1 m 틀린 채 회복, 재개 health=ok → 차이 비교로 전환(16초)", sw is not None and 16.0 <= sw < 16.1 and gm.n_clear == 0, f'{sw}')
gm, ts, _ = mux_run(lambda t: 'lost' if t < 30 else 'degraded', lambda t: (0.0, 0.0) if t < 10 else (1.0, 0.0))
chk("속도 사유 보류 + 백업 degraded 로 회복 → 해제 안 함(전환 30초) — 무소식 사유만 확장", ts is not None and 30.0 <= ts < 30.1 and gm.n_clear == 0, f'{ts} {gm.mux.reason}')
gm, ts, _ = mux_run(lambda t: 'lost' if t < 30 else 'degraded', lambda t: (0.0, 0.0) if (t < 8 or t > 11) else None)
chk("무소식 보류 + FAST-LIO 회복 + 백업 lost→degraded(30초) → 30초에 해제(전환 없음)", ts is None and gm.n_clear == 1 and gm.pending is None, f'{ts}')

print('── G③) HealthGuard.blocks 허용 목록')
for h in (None, 'None', 'stale', '', 'init', 'lost', '상태없음', '상태끊김 3.1초', '위치없음', '위치끊김 0.6초', 'OK'):
    chk(f"blocks({h!r}) = True", HealthGuard.blocks(h))
for h in ('ok', 'degraded', 'aligning'):
    chk(f"blocks({h!r}) = False", not HealthGuard.blocks(h))
g = HealthGuard(); g.on_status(0.0, None); g.on_odom(0.0)
chk("JSON null health → state 'None' → 막음", g.state(0.1) == 'None' and HealthGuard.blocks(g.state(0.1)))
gm, ts, _ = mux_run(lambda t: 'stale', lambda t: (0.0, 0.0) if t < 8 else None)
chk("모르는 health 'stale' + FAST-LIO 사망 → 보류(전환 안 함)", ts is None and gm.pending, f'{gm.pending}')

print('── G④b) FAST-LIO 처음부터 없음 / 0.3초만 내고 죽음')
for h in ('ok', 'degraded', 'aligning'):
    gm, ts, _ = mux_run(lambda t, h=h: h, lambda t: None, T=30.0)
    chk(f"FAST-LIO 처음부터 없음, 백업 {h} → 노드 시작+5초에 전환", ts is not None and 5.0 <= ts < 5.05 and '처음부터 무소식' in gm.mux.reason, f'{ts} {gm.mux.reason}')
gm, ts, _ = mux_run(lambda t: 'lost' if t < 20 else 'ok', lambda t: None, T=30.0)
chk("FAST-LIO 처음부터 없음, 백업 lost→ok(20초) → 보류 뒤 20초에 전환", ts is not None and 20.0 <= ts < 20.1, f'{ts}')
gm, ts, ev = mux_run(lambda t: 'init' if t < 12 else 'aligning', lambda t: (0.0, 0.0) if t >= 9.0 else None, T=40.0)
chk("FAST-LIO 늦게 옴(9초), 백업 init→aligning(12초) → 5초 보류 → 12초 해제, 이후 전환 없음", ts is None and gm.n_clear == 1 and gm.mode == 'fastlio', f'{ts}')
gm, ts, ev = mux_run(lambda t: 'aligning', lambda t: (0.0, 0.0) if t >= 3.0 else None, T=30.0)
chk("FAST-LIO 3초에 처음 옴(예열 안) → 전환 없음, 예열은 첫 메시지 기준(MuxLogic.t0=3)", ts is None and abs(gm.mux.t0 - 3.0) < 0.02, f'{ts} t0={gm.mux.t0}')
gm, ts, ev = mux_run(lambda t: 'aligning', lambda t: ((0.0, 0.0) if t < 9.0 else (1.0, 0.0)) if t >= 3.0 else None, T=30.0)
chk("  같은데 9초(첫 메시지+6초)에 점프 1 m → 속도 조건 전환(예열 지남)", ts is not None and abs(ts - 9.0) < 0.05, f'{ts} {gm.mux.reason}')
gm, ts, ev = mux_run(lambda t: 'aligning', lambda t: ((0.0, 0.0) if t < 6.0 else (1.0, 0.0)) if t >= 3.0 else None, T=30.0)
chk("  같은데 6초(첫 메시지+3초, 예열 안)에 점프 → 속도 조건 안 봄(v1 과 같음)", ts is None, f'{ts}')
for h in ('ok', 'degraded'):
    gm, ts, _ = mux_run(lambda t, h=h: h, lambda t: (0.0, 0.0) if t < 0.3 else None, T=30.0)
    chk(f"FAST-LIO 0.3초만 내고 사망, 백업 {h} → 5.0초 전환(무소식)", ts is not None and 5.0 <= ts < 5.05 and '무소식' in gm.mux.reason, f'{ts} {gm.mux.reason}')
gm = GuardedMux(mux=MuxLogic(), guard=HealthGuard()); gm.guard.on_status(0.0, 'ok'); gm.guard.on_odom(0.0); gm.tick(0.0, (0, 0))
chk("t=0 첫 tick 은 시작 시각으로 안 씀(가짜 시계 0 대비)", gm.t_start is None)
