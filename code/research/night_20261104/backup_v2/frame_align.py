#!/usr/bin/env python3
"""지도 좌표 → 오늘 camera_init 정렬기 (T_cam_map, 2D 강체 x·y·yaw). ROS 와 분리 — tests/test_frame_align.py 로 녹화본 검증.

왜   백업 추정기는 '지도 만든 날 camera_init'(사전 지도 좌표)으로 수렴한다. FAST-LIO /Odometry·목표 /goal_pose 는 '오늘 camera_init'.
     출발 yaw 가 2° 만 틀어져도 10 m 에서 0.35 m → 전환 장치(MuxLogic: 백업과 0.5 m·1초)가 멀쩡한 FAST-LIO 를 버리고,
     전환 뒤엔 목표가 엉뚱한 곳. → 백업 출력(/backup/odometry, /backup/cloud)을 이 T 로 camera_init 좌표로 바꿔 낸다.
     추정기 내부 상태는 지도 좌표 그대로 둔다(정합은 지도와 해야 하므로).

어떻게
  · 쌍 = (같은 시각의 백업 자세[지도], FAST-LIO 자세[camera_init]). FAST-LIO 메시지마다 그 시각의 백업 자세를 기록에서 보간해 짝짓는다
    (백업은 다리 오도로 150 Hz 갱신 — 기록이 항상 FAST-LIO 시각을 감싼다. 시각 대응 없이 '최근 값'끼리 짝지으면 0.5 m/s 에서 수 cm 끌림)
  · 쌍을 받는 조건: init 뒤, 백업 정합이 연속 수락 min_streak 회 이상(보정량이 작아 '수렴한' 수락만 셈), 마지막 수락이 fresh_match_s 안,
    FAST-LIO yaw 속도 ≤ max_yaw_rate_dps (회전 중엔 백업 yaw 가 늦어 방향 쌍이 틀린다 — 10-30 실측)
  · 추정 = 2D Umeyama(가중 최소제곱, 닫힌 해)  H = Σ(p−p̄)(q−q̄)ᵀ + L²·Σ h(ψp)h(ψq)ᵀ,  θ = atan2(H01−H10, H00+H11),  t = q̄ − R p̄
    L(lever_m) 은 '방향 쌍'의 무게. 서 있기만 해도(점이 한 곳) 방향으로 yaw 가 정해진다. 0.5 m 기선의 점만으론 yaw 가 수 ° 흔들림
  · 고정 조건: 쌍 ≥ min_pairs, 쌍 시간폭 ≥ min_span_s 또는 이동 ≥ min_travel_m, 위치 잔차 rms ≤ max_rms_m·최대 ≤ max_res_m, 방향 잔차 rms ≤ max_yaw_rms_deg
    이상 쌍(0.10 m·2° 밖)을 빼고 다시 풀며, inlier 비율 ≥ min_inlier_frac 도 본다.
    고정 뒤엔 다시 안 바꾼다(경기 중 좌표가 움직이면 플래너가 흔들린다). 고정 전엔 항등.
  · 전환 장치가 백업으로 넘어가면(노드가 /mux/status 를 봄) 그 순간 고정 — 쓰이는 도중 T 가 바뀌면 위치가 튄다.
  · FAST-LIO 가 처음부터 없으면(no_fastlio_s 동안 무소식) 항등으로 고정 + 경고. 추정 중 FAST-LIO 가 끊기면(stale_freeze_s 0.5 s, 쌍이 모자라면 stale_identity_s 3 s)
    쌍이 fallback_min_pairs(3) 이상이고 잔차가 문턱 안이면 그 추정으로 고정, 아니면 항등 고정 + 경고.
    (쌍 하나가 x·y·yaw 를 다 정한다 — 수렴한 정합에서만 모은 쌍이라 몇 개라도 항등(0.5 m 틀림)보다 낫다)
상태  'identity'(아직 쌍 없음) → 'estimating' → 'frozen' | 'identity_frozen'(FAST-LIO 없음/추정 실패) | 'align_failed'(아래 F)

11-04 08:25 고침 [F] (검증 회귀 (1)(2)(5) — 메인 결정 b)
  · 이상 감지 즉시 고정 on_anomaly(): 전환 장치 /mux/status 가 pending 새로 걸림 또는 mode=backup, 또는 정렬기 스스로 FAST-LIO 점프를 봄
    → 그 순간 고정. 쌍 ≥ fallback_min_pairs(3) 이고 inlier 가 과반이면 그 추정, 아니면 항등. (예전: 엄격 잔차 판정 실패 시 항등)
    점프 = 연속 FAST-LIO 두 메시지(0.5초 안) 사이 이동이 같은 구간 백업(다리) 이동보다 jump_m(0.3 m) 넘게 큼 → 점프 메시지는 쌍에 안 넣음
    (전환 장치는 시작 5초 동안 속도 판정을 안 한다 — init 직후 점프를 정렬기가 쌍으로 삼켜 틀린 T 를 고정하던 문제(1.0 m) 대책)
  · init 전에 이상 신호가 오면(유지형 /mux/status) 기억했다가 init 순간 항등 고정 — 예전엔 버려졌다
  · 'align_failed': 쌍 ≥ fail_min_pairs(40) · 쌍 시간폭 ≥ fail_span_s(8초)인데 고정 판정이 계속 실패 → 항등으로 고정하고 가림을 푼다
    (+ 조기: 쌍 ≥ 40 · 시간폭 ≥ min_span_s(4초) 인데 inlier < fail_inlier(0.5) — 쌍 과반이 최적 T 에서 0.1 m·2° 밖 = 한쪽이 움직이는 중)
    (= 정렬 전 예전 동작: 차이 판정이 바로 산다) + 경고. 고정 전 FAST-LIO 표류가 60초 동안 전환을 막던 문제 대책
  · 가림 최대 mask_max_s 60 → 20초
  · max_res_m: inlier 판정(inlier_m 0.10) 뒤 잔차라 0.15 는 구조상 안 걸린다 — 정상 고정 의미를 안 바꾸려고 그대로 둠(주석만)
"""
import math
from collections import deque
import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def umeyama2d(P, Q, yp=None, yq=None, lever=2.0, w=None):
    """P(지도)·Q(camera_init) N×2, yp·yq 방향(rad). Q ≈ R(θ)P + t 의 가중 최소제곱. 반환 (tx, ty, θ)."""
    P = np.asarray(P, float); Q = np.asarray(Q, float)
    w = np.ones(len(P)) if w is None else np.asarray(w, float)
    w = w / w.sum()
    pb, qb = (w[:, None] * P).sum(0), (w[:, None] * Q).sum(0)
    X, Y = P - pb, Q - qb
    s = float((w * (X[:, 0] * Y[:, 1] - X[:, 1] * Y[:, 0])).sum())
    c = float((w * (X * Y).sum(1)).sum())
    if yp is not None and lever > 0:
        d = np.asarray(yq, float) - np.asarray(yp, float)          # h(ψp)·h(ψq) = cos(ψq−ψp), h(ψp)×h(ψq) = sin(ψq−ψp)
        s += lever * lever * float((w * np.sin(d)).sum())
        c += lever * lever * float((w * np.cos(d)).sum())
    th = math.atan2(s, c)
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    t = qb - R @ pb
    return float(t[0]), float(t[1]), th


class FrameAligner:
    def __init__(self, min_streak=10, corr_ok_m=0.05, corr_ok_deg=1.0, fresh_match_s=0.5, fresh_fl_s=0.2,
                 min_pairs=40, min_span_s=4.0, min_travel_m=0.5, max_rms_m=0.05, max_res_m=0.15, max_yaw_rms_deg=1.0,
                 lever_m=2.0, max_pairs=400, no_fastlio_s=8.0, stale_freeze_s=0.5, stale_identity_s=3.0, fallback_min_pairs=3,
                 default_latency_s=0.01, hist_s=3.0, max_yaw_rate_dps=10.0, inlier_m=0.10, inlier_deg=2.0,
                 min_inlier_frac=0.8, mask_max_s=20.0, fail_min_pairs=40, fail_span_s=8.0, jump_m=0.3, jump_dt_s=0.5,
                 anomaly_min_inlier=0.5, fail_inlier=0.5):
        self.min_streak, self.corr_ok_m, self.corr_ok_rad = min_streak, corr_ok_m, math.radians(corr_ok_deg)
        self.fresh_match_s, self.fresh_fl_s = fresh_match_s, fresh_fl_s
        self.min_pairs, self.min_span_s, self.min_travel_m = min_pairs, min_span_s, min_travel_m
        self.max_rms_m, self.max_res_m, self.max_yaw_rms = max_rms_m, max_res_m, math.radians(max_yaw_rms_deg)
        self.lever, self.max_pairs = lever_m, max_pairs
        self.no_fastlio_s, self.stale_freeze_s, self.fallback_min_pairs = no_fastlio_s, stale_freeze_s, fallback_min_pairs
        self.stale_identity_s = stale_identity_s
        self.default_latency_s, self.hist_s = default_latency_s, hist_s
        self.max_yaw_rate = math.radians(max_yaw_rate_dps)     # 회전 중 쌍은 버림 — 백업 yaw 가 회전 중 늦다(10-30: 10°/s 미만 rms 0.57°, 20~40°/s 2.5°, 40°/s+ 4.4°)
        self.inlier_m, self.inlier_rad, self.min_inlier_frac = inlier_m, math.radians(inlier_deg), min_inlier_frac
        self.mask_max_s = mask_max_s
        self.fail_min_pairs, self.fail_span_s, self.fail_inlier = fail_min_pairs, fail_span_s, fail_inlier
        self.jump_m, self.jump_dt_s, self.anomaly_min_inlier = jump_m, jump_dt_s, anomaly_min_inlier   # jump_m=0 이면 점프 감지 끔
        self.pre_init_anomaly = None; self.n_jump = 0; self.anomaly = None
        self.fl_prev = None; self.n_turn_skip = 0
        self.state = 'identity'; self.T = (0.0, 0.0, 0.0); self._cs = (1.0, 0.0)
        self.est = None; self.res = None                 # 추정 중 최신 (tx, ty, θ), 잔차 dict
        self.pairs = deque(maxlen=max_pairs)              # (t, px, py, pyaw, qx, qy, qyaw)
        self.bk = deque()                                 # 백업 자세 기록 (t, x, y, yaw) — 지도 좌표
        self.streak = 0; self.t_last_ok = None; self.t_start = None
        self.t_fl = None; self.n_fl = 0; self.t_frozen = None; self.why = ''; self.warn = None
        self.n_rejected_pairs = 0

    # ── 입력 ────────────────────────────────────────────────────────────────────
    def on_init(self, t):
        """백업 추정기 init 시각(노드 시계)."""
        if self.t_start is None:
            self.t_start = t
            if self.pre_init_anomaly is not None and not self.frozen():     # init 전에 온 이상 신호(유지형 /mux/status) — 쌍이 없으니 항등
                return self._freeze_identity(t, f'init 전 이상 신호({self.pre_init_anomaly}) — 항등 고정')
        return None

    def on_backup_pose(self, t, x, y, yaw):
        """백업 자세(지도 좌표) 기록 — 다리 갱신마다. 시각 보간용."""
        self.bk.append((t, x, y, yaw))
        while self.bk and self.bk[0][0] < t - self.hist_s:
            self.bk.popleft()

    def on_match(self, t, accepted, corr_m=0.0, corr_rad=0.0):
        """정합 결과. corr_* = 씨앗 대비 보정량 — 크면 아직 수렴 중이라 연속 수락으로 안 친다."""
        if accepted and corr_m <= self.corr_ok_m and abs(corr_rad) <= self.corr_ok_rad:
            self.streak += 1; self.t_last_ok = t
        else:
            self.streak = 0

    def on_fastlio(self, t_recv, x, y, yaw, stamp=None):
        """FAST-LIO 자세(camera_init). stamp 가 같은 시계(받은 시각과 0~1초 차)면 그 시각, 아니면(재생: 녹화 시계) 받은 시각 − 기본 지연."""
        """반환: 이 메시지로 점프를 보고 고정했으면 그 설명 문자열(노드가 상태를 바로 발행), 아니면 None."""
        self.t_fl = t_recv; self.n_fl += 1
        te = stamp if (stamp is not None and 0.0 <= t_recv - stamp < 1.0) else t_recv - self.default_latency_s
        prev, self.fl_prev = self.fl_prev, (te, yaw, x, y)
        if self.state not in ('identity', 'estimating') or self.t_start is None:
            return None
        if self.jump_m > 0 and prev is not None and 1e-3 < te - prev[0] <= self.jump_dt_s:
            dfl = math.hypot(x - prev[2], y - prev[3])
            if dfl > self.jump_m:
                b0, b1 = self._bk_at(prev[0]), self._bk_at(te)
                dbk = math.hypot(b1[0] - b0[0], b1[1] - b0[1]) if (b0 is not None and b1 is not None) else 0.0
                if dfl - dbk > self.jump_m:
                    self.n_jump += 1
                    return self.on_anomaly(t_recv, f'FAST-LIO 점프 {dfl:.2f} m / {te - prev[0]:.2f}s (다리 {dbk:.2f} m)')
        if prev is not None and te - prev[0] > 1e-3 and abs(wrap(yaw - prev[1])) / (te - prev[0]) > self.max_yaw_rate:
            self.n_turn_skip += 1; return None
        if self.streak < self.min_streak or self.t_last_ok is None or te - self.t_last_ok > self.fresh_match_s:
            return None
        b = self._bk_at(te)
        if b is None:
            self.n_rejected_pairs += 1; return None
        self.pairs.append((te, b[0], b[1], b[2], x, y, yaw))
        if self.state == 'identity':
            self.state = 'estimating'
        return None

    def tick(self, t):
        """주기 호출(노드 1 Hz 이상). 고정 판단·FAST-LIO 없음 처리. 반환: 고정한 순간의 설명 문자열(경고면 self.warn 도 채움) 또는 None."""
        if self.frozen():
            return None
        if self.t_start is None:
            return None
        if self.t_fl is None:
            if t - self.t_start > self.no_fastlio_s:
                return self._freeze_identity(t, f'FAST-LIO {self.no_fastlio_s:.0f}초 무소식 — 지도 좌표를 그대로 camera_init 으로 냄(항등)')
            return None
        if len(self.pairs) >= 3:
            self._estimate()
        if t - self.t_fl > self.stale_freeze_s:      # 추정 중 FAST-LIO 끊김(전환 장치 무소식 문턱 0.5 s 와 같게) → 있는 것으로 고정
            if self.res and len(self.pairs) >= self.fallback_min_pairs and self._res_ok():
                return self._freeze(t, f'FAST-LIO 끊김({t - self.t_fl:.1f}s) — 쌍 {len(self.pairs)}개로 조기 고정', warn=True)
            if t - self.t_fl > self.stale_identity_s:  # 쌍이 없으면 조금 더 기다렸다가 항등 고정(그동안 출력도 항등이라 손해 없음 — 잠깐 끊긴 거면 계속 추정)
                return self._freeze_identity(t, f'FAST-LIO 끊김({t - self.t_fl:.1f}s), 쌍 {len(self.pairs)}개·잔차 부족 — 항등 고정')
            return None
        if self.res and len(self.pairs) >= self.min_pairs and self._res_ok() and \
                (self.res['span_s'] >= self.min_span_s or self.res['travel_m'] >= self.min_travel_m):
            return self._freeze(t, f"쌍 {len(self.pairs)}개 · {self.res['span_s']:.1f}s · 이동 {self.res['travel_m']:.2f} m")
        span_all = self.pairs[-1][0] - self.pairs[0][0] if self.pairs else 0.0
        gross = self.res is not None and span_all >= self.min_span_s and self.res['inlier'] < self.fail_inlier   # 과반이 어긋남 = 둘 중 하나가 움직임
        if len(self.pairs) >= self.fail_min_pairs and (span_all >= self.fail_span_s or gross):   # [F] 정렬 실패 → 가림 풂
            r = self.res or {}
            return self._freeze_identity(t, f"정렬 실패 — 쌍 {len(self.pairs)}개 · {self.pairs[-1][0] - self.pairs[0][0]:.1f}s 동안 판정 불합격"
                                            f"(rms {r.get('rms_m', float('nan')):.3f} m, inlier {r.get('inlier', float('nan')):.2f}) — 항등·가림 풂(예전 동작)",
                                         state='align_failed')
        return None

    # ── 출력 ────────────────────────────────────────────────────────────────────
    def map_to_cam(self, x, y, yaw):
        tx, ty, th = self.T; c, s = self._cs
        return c * x - s * y + tx, s * x + c * y + ty, wrap(yaw + th)

    def points_to_cam(self, P):
        """N×3 (지도) → camera_init. z 는 그대로."""
        if self.T == (0.0, 0.0, 0.0):
            return P
        tx, ty, _ = self.T; c, s = self._cs
        Q = P.copy()
        Q[:, 0] = c * P[:, 0] - s * P[:, 1] + tx
        Q[:, 1] = s * P[:, 0] + c * P[:, 1] + ty
        return Q

    def frozen(self):
        return self.state in ('frozen', 'identity_frozen', 'align_failed')

    def mask_health(self, t):
        """노드가 health 'ok' 를 'aligning' 으로 가릴지. 고정 전 + init 뒤 mask_max_s 안 + FAST-LIO 가 살아 있을 때만.
        FAST-LIO 가 stale_freeze_s 넘게 끊기면 안 가린다 — 그때는 차이 판정이 무의미하고, 전환 장치 v2(백업 ok 일 때만 전환)가 바로 넘겨받아야 한다."""
        return (not self.frozen()) and (self.t_start is None or t - self.t_start < self.mask_max_s) \
            and (self.t_fl is None or t - self.t_fl <= self.stale_freeze_s)

    def on_backup_in_use(self, t, why='전환 장치가 백업 사용 시작'):
        """전환 장치가 백업으로 넘어갔다(/mux/status mode=backup) → 지금 고정. 쓰이는 도중 T 가 바뀌면 위치가 튀므로 더 기다리지 않는다."""
        return self.on_anomaly(t, why)

    def on_anomaly(self, t, why):
        """[F] 이상 감지(전환 장치 pending 새로 걸림 / mode=backup / FAST-LIO 점프) → 지금 고정. 이 뒤의 FAST-LIO 쌍은 믿을 수 없다.
        쌍 ≥ fallback_min_pairs 이고 inlier 과반(anomaly_min_inlier)이면 그 추정(이상 직후 섞인 쌍 몇 개는 inlier 판정이 뺀다), 아니면 항등.
        init 전이면 기억했다가 init 순간 항등 고정."""
        if self.frozen():
            return None
        self.anomaly = why
        if self.t_start is None:
            self.pre_init_anomaly = self.pre_init_anomaly or why
            return None
        if len(self.pairs) >= 3:
            self._estimate()
        if self.res and len(self.pairs) >= self.fallback_min_pairs and self.res['inlier'] >= self.anomaly_min_inlier:
            ok = '' if self._res_ok() else ' (엄격 판정은 미달)'
            return self._freeze(t, f'{why} — 쌍 {len(self.pairs)}개로 즉시 고정{ok}', warn=True)
        return self._freeze_identity(t, f'{why}, 쌍 {len(self.pairs)}개·잔차 부족 — 항등 고정')

    def status(self, t=None):
        tx, ty, th = self.T
        d = dict(state=self.state, T=dict(x=round(tx, 3), y=round(ty, 3), yaw_deg=round(math.degrees(th), 2)),
                 pairs=len(self.pairs), streak=self.streak, turn_skip=self.n_turn_skip, why=self.why)
        if self.n_jump:
            d['fl_jumps'] = self.n_jump
        if self.est is not None and self.state == 'estimating':
            d['est'] = dict(x=round(self.est[0], 3), y=round(self.est[1], 3), yaw_deg=round(math.degrees(self.est[2]), 2))
        if self.res:
            d['res'] = {k: round(v, 3) for k, v in self.res.items()}
        if self.t_frozen is not None and self.t_start is not None:
            d['frozen_after_s'] = round(self.t_frozen - self.t_start, 1)
        if self.warn:
            d['warn'] = self.warn
        return d

    # ── 내부 ────────────────────────────────────────────────────────────────────
    def _bk_at(self, te):
        if len(self.bk) < 2 or te < self.bk[0][0] or te > self.bk[-1][0] + self.fresh_fl_s:
            return None
        if te >= self.bk[-1][0]:
            return self.bk[-1][1:]
        lo, hi = 0, len(self.bk) - 1
        while hi - lo > 1:                                # 이분 탐색 (deque 인덱스 접근)
            m = (lo + hi) // 2
            if self.bk[m][0] <= te: lo = m
            else: hi = m
        a, b = self.bk[lo], self.bk[hi]
        if b[0] - a[0] > 0.2:                             # 기록 공백(다리 오도 끊김) — 보간 안 함
            return None
        f = 0.0 if b[0] <= a[0] else (te - a[0]) / (b[0] - a[0])
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), wrap(a[3] + f * wrap(b[3] - a[3])))

    def _fit(self, A):
        tx, ty, th = umeyama2d(A[:, 1:3], A[:, 4:6], A[:, 3], A[:, 6], lever=self.lever)
        c, s = math.cos(th), math.sin(th)
        px = c * A[:, 1] - s * A[:, 2] + tx; py = s * A[:, 1] + c * A[:, 2] + ty
        e = np.hypot(px - A[:, 4], py - A[:, 5])
        ey = (A[:, 6] - A[:, 3] - th + math.pi) % (2 * math.pi) - math.pi
        return (tx, ty, th), e, ey

    def _estimate(self):
        """전 쌍으로 한 번 → 이상 쌍(위치 inlier_m·방향 inlier_deg 밖) 빼고 한 번 더. 잔차·고정 판정은 inlier 로, inlier 비율도 본다."""
        A = np.array(self.pairs)
        T, e, ey = self._fit(A)
        inl = (e <= self.inlier_m) & (np.abs(ey) <= self.inlier_rad)
        if inl.sum() >= max(3, 0.5 * len(A)) and not inl.all():
            T, e2, ey2 = self._fit(A[inl])
            _, e, ey = self._fit_eval(A, T)
            inl = (e <= self.inlier_m) & (np.abs(ey) <= self.inlier_rad)
        self.est = T
        ei, eyi = (e[inl], ey[inl]) if inl.any() else (e, ey)
        Ai = A[inl] if inl.any() else A
        self.res = dict(rms_m=float(np.sqrt((ei ** 2).mean())), max_m=float(ei.max()),
                        yaw_rms_deg=float(math.degrees(np.sqrt((eyi ** 2).mean()))),
                        inlier=float(inl.mean()), span_s=float(Ai[-1, 0] - Ai[0, 0]),
                        travel_m=float(np.hypot(np.diff(Ai[:, 4]), np.diff(Ai[:, 5])).sum()))

    @staticmethod
    def _fit_eval(A, T):
        tx, ty, th = T; c, s = math.cos(th), math.sin(th)
        px = c * A[:, 1] - s * A[:, 2] + tx; py = s * A[:, 1] + c * A[:, 2] + ty
        return T, np.hypot(px - A[:, 4], py - A[:, 5]), (A[:, 6] - A[:, 3] - th + math.pi) % (2 * math.pi) - math.pi

    def _res_ok(self):
        r = self.res
        return r['rms_m'] <= self.max_rms_m and r['max_m'] <= self.max_res_m and r['yaw_rms_deg'] <= math.degrees(self.max_yaw_rms) \
            and r['inlier'] >= self.min_inlier_frac

    def _set_T(self, T):
        self.T = (float(T[0]), float(T[1]), float(T[2])); self._cs = (math.cos(self.T[2]), math.sin(self.T[2]))

    def _freeze(self, t, why, warn=False):
        self._set_T(self.est); self.state = 'frozen'; self.t_frozen = t; self.why = why
        if warn:
            self.warn = why
        return why

    def _freeze_identity(self, t, why, state='identity_frozen'):
        self._set_T((0.0, 0.0, 0.0)); self.state = state; self.t_frozen = t; self.why = why; self.warn = why
        return why
