#!/usr/bin/env python3
"""PCD 지도의 벽 날카로움 — compare_maps.py 와 같은 기준(세로 벽 기둥 칸, 반경 15 cm 이웃의 가장 얇은 방향 σ).
사용: pcd_wall_sharpness.py <이름=pcd 또는 npz> ...   바닥 = z 히스토그램 최빈값(가장 넓은 수평면)"""
import sys, os, numpy as np
from scipy.spatial import cKDTree


def read_pcd(p):
    with open(p, 'rb') as f:
        hdr = {}
        while True:
            l = f.readline().decode('ascii', 'ignore').strip()
            k, *v = l.split()
            hdr[k] = v
            if k == 'DATA':
                break
        names, sizes, types, counts = hdr['FIELDS'], list(map(int, hdr['SIZE'])), hdr['TYPE'], list(map(int, hdr.get('COUNT', ['1'] * len(hdr['FIELDS']))))
        n = int(hdr['POINTS'][0])
        dt = np.dtype([(nm if c == 1 else nm, {('F', 4): 'f4', ('F', 8): 'f8', ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4', ('I', 4): 'i4', ('I', 2): 'i2', ('I', 1): 'i1'}[(t, s)], (c,) if c > 1 else ())
                       for nm, s, t, c in zip(names, sizes, types, counts)])
        if hdr['DATA'][0] == 'binary':
            a = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
        else:
            a = np.loadtxt(f, max_rows=n); return a[:, :3]
    return np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)


def load(f):
    f = os.path.expanduser(f)
    if f.endswith('.npz'):
        d = np.load(f); return (d['vox'][d['cnt'] >= 2] + 0.5) * float(d['V'])
    return read_pcd(f)


if __name__ == '__main__':
    rng = np.random.default_rng(0)
    print('지도 | 점 | 바닥 z | 벽칸 | 벽두께σ 중앙 cm | 얇은벽(σ<2cm)% | 두꺼운벽(σ>4cm)%')
    for arg in sys.argv[1:]:
        name, f = arg.split('=', 1); P = load(f); P = P[np.isfinite(P).all(1)]
        h, e = np.histogram(P[:, 2], bins=np.arange(np.percentile(P[:, 2], 1) - 0.5, np.percentile(P[:, 2], 99) + 0.5, 0.02)); floor = e[h.argmax()] + 0.01
        hh = P[:, 2] - floor; k2 = np.round(P[:, :2] / 0.02).astype(np.int64); key = k2[:, 0] * 1000003 + k2[:, 1]
        both = np.array(sorted(set(key[(hh > 0.10) & (hh < 0.25)].tolist()) & set(key[(hh > 0.40) & (hh < 0.55)].tolist())), dtype=np.int64)
        if len(both) < 50:
            print(f'{name} | {len(P)} | {floor:.2f} | 벽 칸 {len(both)} — 너무 적음(점이 성기면 이 기준으로 못 잼)'); continue
        kx = np.floor_divide(both + 500000, 1000003); W = np.c_[kx, both - kx * 1000003].astype(float) * 0.02
        T = cKDTree(W); S = W[rng.choice(len(W), min(15000, len(W)), replace=False)]; sig = []
        for p, nb in zip(S, T.query_ball_point(S, 0.15)):
            if len(nb) < 8: continue
            Q = W[nb] - W[nb].mean(0); sig.append(np.sqrt(max(np.linalg.eigvalsh(Q.T @ Q / len(nb))[0], 0)))
        sig = np.array(sig)
        print(f'{name} | {len(P)} | {floor:.2f} | {len(W)} | {np.median(sig) * 100:.2f} | {(sig < 0.02).mean() * 100:.1f} | {(sig > 0.04).mean() * 100:.1f}')
