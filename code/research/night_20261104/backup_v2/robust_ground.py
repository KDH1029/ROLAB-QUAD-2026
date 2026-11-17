#!/usr/bin/env python3
"""지면 높이표 — backup_core.GroundGrid 의 대체 (같은 인터페이스 at(x, y)).

11-04 발견: 앞부분 지도에서 바닥이 안 보인 칸은 천장 점(2.23 m)만 남아, GroundGrid(칸 하위 10%)가 천장을 지면으로 냈다
→ 씨앗 z 가 2.5 m 높아 정합이 11초 거부, 경사로에서 위치오차 0.25 m. 경기장 지도도 바닥이 덜 찍힌 칸이 있으면 똑같이 터진다.

고친 점
① 바닥 기준 z_ref = 칸별 최저점 높이의 최빈값(바닥 칸이 압도적으로 많다)
② z_ref + max_rise(기본 1.2 m) 위의 점은 지면 후보에서 뺀다 (천장·현수막·벽 윗부분)
③ 빈 칸은 fill_r 칸(기본 3칸 = 0.3 m) 안의 가장 가까운 유효 칸만 쓴다. 그보다 멀면 '모름'
④ 직전 값 대비 max_step(기본 0.30 m, 계단 한 단 + 여유)을 넘게 튀거나 '모름'이면 직전 값을 유지 (상태가 있다)
   단 튄 값이 confirm_n 번(기본 45번 ≈ 다리 오도 150 Hz 로 0.3초) 연속 비슷하면 받아들인다 — 단상 끝에서 내려올 때 갇히지 않게
   (11-04 1차: 이게 없어서 경사 단상에서 내려온 뒤 z 가 +0.5 m 에 60초 갇혔다)

11-04 07:40 수정 (검토 ① — 최빈값이 천장으로 뒤집힐 수 있음. 기본 지도에서 최빈 칸 5.3% 뿐, 3위가 천장 2.28 m)
① 바닥 기준 z_ref 를 '출발점(지도 원점 = 로봇이 바닥에 서서 시작한 곳) 반경 start_r(1.5 m) 칸들의 최저점'에서 잡는다
   - 그냥 중앙값은 안 된다: 앞 270초 지도는 반경 1.5 m 칸의 36% 가 천장만 찍힌 칸이라 중앙값이 +0.03 m (바닥 −0.35)
     → 그 칸들 최저점의 하위 10% 값 + 0.25 m 안 (= 가장 낮은 무리, 바닥)만 모아 중앙값
   - 반경 안 유효 칸이 min_cells(20) 개 미만이면('모름') 출발점에서 가까운 유효 칸 min_cells×2 개로 대신
   - 예전 정의(최빈값)와 0.3 m 넘게 다르면 경고 (stderr) — 지도 확인용. 판정엔 출발점 값을 쓴다
   진단: z_ref_mode(예전), z_ref_start_median(그냥 중앙값), z_ref_src, z_ref_warn
"""
import sys
import numpy as np
from scipy.spatial import cKDTree


class RobustGroundGrid:
    def __init__(self, mp, cell=0.10, max_rise=1.2, fill_r=3, max_step=0.30, pct=0.1, confirm_n=45,
                 start_xy=(0.0, 0.0), start_r=1.5, min_cells=20, low_band=0.25, warn_dz=0.30, verbose=True):
        self.c, self.fill_r, self.max_step, self.confirm_n = cell, fill_r, max_step, confirm_n
        self.cand = None; self.cand_n = 0; self.n_jump_ok = 0
        k = np.floor(mp[:, :2] / cell).astype(np.int64)
        o = np.lexsort((mp[:, 2], k[:, 1], k[:, 0])); ks, zs = k[o], mp[o, 2]
        brk = np.r_[True, np.any(ks[1:] != ks[:-1], axis=1)]
        starts = np.where(brk)[0]; ends = np.r_[starts[1:], len(ks)]
        low = zs[starts]                                            # 칸별 최저점
        h, e = np.histogram(low, bins=np.arange(low.min(), low.max() + 0.04, 0.02))
        self.z_ref_mode = float(e[np.argmax(h)] + 0.01)             # 예전 정의(최빈값) — 비교·경고용
        cen = (ks[starts] + 0.5) * cell
        dist = np.hypot(cen[:, 0] - start_xy[0], cen[:, 1] - start_xy[1])
        sel = dist <= start_r
        if sel.sum() >= min_cells:
            self.z_ref_src = f'출발점 반경 {start_r:.1f} m 칸 {int(sel.sum())}개'
        else:                                                       # 출발점 칸이 '모름' → 가까운 유효 칸
            sel = np.zeros(len(dist), bool); sel[np.argsort(dist)[:min_cells * 2]] = True
            self.z_ref_src = f'출발점 반경 {start_r:.1f} m 칸 부족 → 가까운 유효 칸 {int(sel.sum())}개 (최대 {dist[sel].max():.1f} m)'
        L = low[sel]
        self.z_ref_start_median = float(np.median(L))
        lo = float(np.percentile(L, 10))
        self.z_ref = float(np.median(L[L <= lo + low_band]))       # 가장 낮은 무리(바닥)의 중앙값 — 천장만 찍힌 칸 제외
        self.z_ref_warn = None
        if abs(self.z_ref - self.z_ref_mode) > warn_dz:
            self.z_ref_warn = (f'[RobustGroundGrid] 경고: 바닥 기준 z_ref {self.z_ref:+.2f} m ({self.z_ref_src}) 가 '
                               f'칸별 최저점 최빈값 {self.z_ref_mode:+.2f} m 와 {abs(self.z_ref - self.z_ref_mode):.2f} m 다름 '
                               f'— 지도 바닥이 덜 찍혔거나 출발점 주변이 이상. 출발점 값을 씀')
            if verbose:
                print(self.z_ref_warn, file=sys.stderr, flush=True)
        zmax = self.z_ref + max_rise
        keys, z = [], []
        for kk, a, b in zip(ks[starts], starts, ends):
            zz = zs[a:b]; zz = zz[zz <= zmax]                        # 이미 정렬됨
            if len(zz):
                keys.append(kk); z.append(zz[int(pct * (len(zz) - 1))])
        self.keys = np.array(keys); self.z = np.array(z)
        self.tree = cKDTree(self.keys.astype(np.float64))
        self.last = None; self.n_hold = 0; self.n_unknown = 0
        self.n_cells_dropped = int(len(starts) - len(keys))

    def raw_at(self, x, y):
        dist, i = self.tree.query([x / self.c, y / self.c])
        return float(self.z[i]) if dist <= self.fill_r else None

    def at(self, x, y):
        g = self.raw_at(x, y)
        if g is None:
            self.n_unknown += 1
            return self.last if self.last is not None else self.z_ref
        if self.last is not None and abs(g - self.last) > self.max_step:
            if self.cand is not None and abs(g - self.cand) <= self.max_step:
                self.cand_n += 1
            else:
                self.cand, self.cand_n = g, 1
            if self.cand_n < self.confirm_n:
                self.n_hold += 1
                return self.last
            self.n_jump_ok += 1                                      # 0.3초 내내 새 높이 → 진짜 단차
        self.cand, self.cand_n = None, 0
        self.last = g
        return g
