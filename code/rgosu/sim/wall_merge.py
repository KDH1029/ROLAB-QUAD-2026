#!/usr/bin/env python3
"""여러 겹으로 찍힌 벽을 하나로 묶는다 — 쌓은 지도(시뮬레이션 벽)용. 실제 주행(ego_planner 는 매 스캔만 봄)에는 필요 없다.
방법: 벽 칸(4 cm)을 반경 r 로 닫기(부풀렸다 줄이기 → r·2 안의 겹이 붙음) → 가운데 한 줄로 세선화(Zhang-Suen) → 두께 thick 칸으로.
  r 이 너무 크면 진짜로 떨어진 두 벽(통로 양쪽 약 1 m)까지 붙는다 → 10~20 cm 권장.
사용: python3 wall_merge.py <입력 npy> <출력 npy> [r=0.12] [두께 칸=2]     (파이썬에서: merge_cells(cells, r))"""
import sys
import numpy as np
from scipy import ndimage

G = 0.04


def _thin(img):
    """Zhang-Suen 세선화(numpy). img: bool 2D."""
    I = np.pad(img.astype(np.uint8), 1)
    while True:
        changed = False
        for step in (0, 1):
            P2, P3, P4 = I[:-2, 1:-1], I[:-2, 2:], I[1:-1, 2:]
            P5, P6, P7 = I[2:, 2:], I[2:, 1:-1], I[2:, :-2]
            P8, P9 = I[1:-1, :-2], I[:-2, :-2]
            nb = [P2, P3, P4, P5, P6, P7, P8, P9]
            B = sum(n.astype(int) for n in nb)
            A = sum(((nb[i] == 0) & (nb[(i + 1) % 8] == 1)).astype(int) for i in range(8))
            C = I[1:-1, 1:-1] == 1
            if step == 0:
                m = C & (B >= 2) & (B <= 6) & (A == 1) & (P2 * P4 * P6 == 0) & (P4 * P6 * P8 == 0)
            else:
                m = C & (B >= 2) & (B <= 6) & (A == 1) & (P2 * P4 * P8 == 0) & (P2 * P6 * P8 == 0)
            if m.any():
                I[1:-1, 1:-1][m] = 0; changed = True
        if not changed:
            return I[1:-1, 1:-1].astype(bool)


def merge_cells(cells, r=0.12, thick=2):
    """cells: {(ix, iy)} 4 cm 칸 → 묶은 칸 집합."""
    if not cells:
        return set()
    a = np.array(list(cells)); k = max(1, int(round(r / G))); pad = k + 3
    lo = a.min(0) - pad; img = np.zeros(tuple(a.max(0) - lo + pad + 1), bool); img[a[:, 0] - lo[0], a[:, 1] - lo[1]] = True
    yy, xx = np.mgrid[-k:k + 1, -k:k + 1]; disk = (xx * xx + yy * yy) <= k * k
    closed = ndimage.binary_closing(img, structure=disk)
    sk = _thin(closed)
    if thick > 1:
        sk = ndimage.binary_dilation(sk, iterations=thick - 1)
    ix, iy = np.nonzero(sk)
    return set(zip((ix + lo[0]).tolist(), (iy + lo[1]).tolist()))


def selftest():
    # 3 m 벽이 8 cm·16 cm 간격으로 세 겹 + 1 m 떨어진 다른 벽 → r=0.12 면 세 겹은 하나, 1 m 벽은 따로
    cells = set()
    for off in (0.0, 0.08, 0.16):
        for x in np.arange(0, 3, G): cells.add((int(x / G), int((1.0 + off) / G)))
    for x in np.arange(0, 3, G): cells.add((int(x / G), int(2.2 / G)))
    m = merge_cells(cells, 0.12, 1)
    ys = sorted({c[1] for c in m if 10 < c[0] < 60})
    rows = {}
    for c in m:
        if 10 < c[0] < 60: rows.setdefault(c[1], 0); rows[c[1]] += 1
    lines = [y for y, n in rows.items() if n > 30]
    print('입력 칸', len(cells), '→ 묶은 칸', len(m), '· 가로줄(y, 칸×4cm):', [round(y * G, 2) for y in sorted(lines)])
    assert len(lines) == 2 and abs(sorted(lines)[0] * G - 1.08) < 0.06 and abs(sorted(lines)[1] * G - 2.2) < 0.06, '자체 시험 실패'
    print('자체 시험 통과 — 세 겹(1.00/1.08/1.16 m)은 1.08 m 한 줄, 1 m 떨어진 벽은 그대로')


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--selftest':
        selftest(); sys.exit()
    src, dst = sys.argv[1], sys.argv[2]; r = float(sys.argv[3]) if len(sys.argv) > 3 else 0.12; th = int(sys.argv[4]) if len(sys.argv) > 4 else 2
    W = np.load(src)[:, :2]; cells = set(map(tuple, np.floor(W / G + 1e-6).astype(int).tolist()))
    m = merge_cells(cells, r, th); c = (np.array(sorted(m), float) + 0.5) * G
    np.save(dst, np.concatenate([np.c_[c, np.full(len(c), z)] for z in (0.15, 0.35)]).astype(np.float32))
    print(f'{src}: 칸 {len(cells)} → r={r} m 로 묶어서 {len(m)} 칸 → {dst}')
