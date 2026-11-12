#!/usr/bin/env python3
"""dr_<태그>_<배치>/report.txt 모아 표: 완주 · 주행 시간 · 슬라럼 시간 · 접촉 δ10(슬라럼) · 콘 무시 직진."""
import sys, re, os
S = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); tag = sys.argv[1]; L = sys.argv[2:]
rows = []
for nm in L:
    try: t = open(f'{S}/runs_v2/dr_{tag}_{nm}/report.txt').read()
    except OSError: rows.append((nm, None)); continue
    ok = '완주 예' in t; drive = float(re.search(r'주행 시간 ([\d.]+)', t).group(1)); sl = float(re.search(r'슬라럼 구간 ([\d.]+)', t).group(1))
    c10 = re.search(r'접촉\(δ 10 cm\): 전체 ([\d.]+)초 (\d+)번', t); c0 = re.search(r'접촉\(δ 0 cm\): 전체 ([\d.]+)초', t)
    rf = re.search(r'ROUTE_FOLLOW\) ([\d.]+)초', t)
    rows.append((nm, (ok, drive, sl, float(c10.group(1)), int(c10.group(2)), float(c0.group(1)), float(rf.group(1)) if rf else 0)))
print(f'[{tag}] 배치      완주  주행s  슬라럼s  접촉δ10(s/번)  접촉δ0 s  콘무시직진s')
n_ok = 0; c = []; tt = []
for nm, r in rows:
    if r is None: print(f'  {nm:8s} 결과 없음'); continue
    ok, dr, sl, c10, n10, c0, rf = r; n_ok += ok; c.append(c10); tt.append(dr if ok else float('nan'))
    print(f'  {nm:8s} {"예" if ok else "아니오":4s} {dr:6.1f} {sl:7.1f}   {c10:5.1f}/{n10:<3d}      {c0:5.1f}    {rf:5.1f}')
import numpy as np
print(f'  → 완주 {n_ok}/{len(rows)} · 접촉δ10 평균 {np.mean(c):.1f}s(최대 {np.max(c):.1f}) · 완주 시간 중앙 {np.nanmedian(tt):.1f}s')
