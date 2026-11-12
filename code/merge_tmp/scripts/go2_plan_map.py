#!/usr/bin/env python3
"""Planning map for EGO-Planner: walls as closed lines, so a plan cannot slip through gaps in the walls and
go around the outside of the arena (11-05 test_plan_on_map: 2-3 of 8 course legs did).

    python3 scripts/go2_plan_map.py maps/arena1459_nocone.yaml maps/arena1459_plan \
        --close -1.55,-0.25,-0.60,-0.45        # + a line across an opening nobody drives through (IN)

1. wall lines: straight runs are found in the occupied cells (sequential RANSAC, a run splits at gaps
   > --split m) and kept when >= --min-len m long
2. gaps closed: two runs on the same line with a gap <= --gap m are joined; a run end that comes within
   --snap m of another wall is joined to it (corners, T-joints)
3. --close x0,y0,x1,y1 draws extra lines (openings that are real but never driven, e.g. IN behind the start)
Output: <out>.bin (EGO prior obstacle layer: uint32 n + n x (float x, float y), map frame), <out>.pgm/.yaml
(walls + lines), <out>_preview.png (added lines in red). The original occupied cells are all kept - only
lines are added. Map frame = the input map's frame (mapping-day FAST-LIO camera_init for arena1459).
"""
import argparse
import os
import struct
import sys

import numpy as np
import yaml


def read_map(path):
    meta = yaml.safe_load(open(path))
    img = meta['image'] if os.path.isabs(meta['image']) else os.path.join(os.path.dirname(os.path.abspath(path)), meta['image'])
    raw = open(img, 'rb').read().split(b'\n', 3)
    w, h = map(int, raw[1].split())
    im = np.frombuffer(raw[3][:w * h], np.uint8).reshape(h, w)[::-1]          # row 0 = origin y
    occ = (255 - im.astype(float)) / 255.0 > float(meta.get('occupied_thresh', 0.65))
    return meta, occ


def runs_on_line(P, p0, d, inl, split):
    """inlier points of the line (p0, d) -> list of (t0, t1) runs along d, split at gaps > split."""
    t = np.sort((P[inl] - p0) @ d)
    if len(t) == 0:
        return []
    cuts = np.where(np.diff(t) > split)[0]
    starts = np.r_[0, cuts + 1]
    ends = np.r_[cuts, len(t) - 1]
    return [(t[a], t[b], b - a + 1) for a, b in zip(starts, ends)]


def extract_segments(P, tol, split, min_len, min_pts, iters=600, seed=0):
    rng = np.random.default_rng(seed)
    left = np.ones(len(P), bool)
    segs = []
    for _ in range(200):
        idx = np.where(left)[0]
        if len(idx) < min_pts:
            break
        Q = P[idx]
        best = None
        for _ in range(iters):
            i, j = rng.choice(len(Q), 2, replace=False)
            v = Q[j] - Q[i]
            L = np.hypot(*v)
            if L < 0.15:
                continue
            d = v / L
            nrm = np.array([-d[1], d[0]])
            inl = np.abs((Q - Q[i]) @ nrm) < tol
            if best is None or inl.sum() > best[0]:
                best = (inl.sum(), Q[i], d, inl)
        if best is None or best[0] < min_pts:
            break
        _, p0, d, inl = best
        # refit direction on the inliers (PCA) for a clean line
        C = Q[inl] - Q[inl].mean(0)
        d = np.linalg.svd(C, full_matrices=False)[2][0]
        p0 = Q[inl].mean(0)
        nrm = np.array([-d[1], d[0]])
        inl = np.abs((Q - p0) @ nrm) < tol
        took = False
        for t0, t1, n in runs_on_line(Q, p0, d, inl, split):
            if t1 - t0 >= min_len and n >= min_pts:
                segs.append((p0 + t0 * d, p0 + t1 * d))
                took = True
        # remove all inliers of this line (also short runs: they are the same wall)
        left[idx[inl]] = False
        if not took and best[0] < 2 * min_pts:
            break
    return segs


def seg_point_dist(p, a, b):
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(ab @ ab, 1e-12), 0.0, 1.0)
    q = a + t * ab
    return np.hypot(*(p - q)), q


def close_gaps(segs, gap, snap, ang_tol_deg=10.0):
    added = []
    n = len(segs)
    dirs = [(b - a) / max(np.hypot(*(b - a)), 1e-9) for a, b in segs]
    # 1) collinear runs with a short gap between their nearest ends
    for i in range(n):
        for j in range(i + 1, n):
            if abs(dirs[i] @ dirs[j]) < np.cos(np.radians(ang_tol_deg)):
                continue
            a0, a1 = segs[i]
            b0, b1 = segs[j]
            ends = [(a0, b0), (a0, b1), (a1, b0), (a1, b1)]
            p, q = min(ends, key=lambda e: np.hypot(*(e[0] - e[1])))
            g = np.hypot(*(p - q))
            nrm = np.array([-dirs[i][1], dirs[i][0]])
            if 0.0 < g <= gap and abs((q - p) @ nrm) < 0.08:
                added.append((p, q))
    # 2) a run end near another wall (corner / T-joint): connect to the closest point of that wall
    for i in range(n):
        for e in segs[i]:
            best = None
            for j in range(n):
                if j == i:
                    continue
                dd, q = seg_point_dist(e, *segs[j])
                if dd > 1e-3 and (best is None or dd < best[0]):
                    best = (dd, q)
            if best and best[0] <= snap:
                added.append((e, best[1]))
    return added


def raster(lines, step):
    out = []
    for a, b in lines:
        L = np.hypot(*(b - a))
        k = max(2, int(np.ceil(L / step)) + 1)
        out.append(a + np.linspace(0, 1, k)[:, None] * (b - a))
    return np.concatenate(out) if out else np.zeros((0, 2))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map_yaml')
    ap.add_argument('out', help='output path without extension')
    ap.add_argument('--tol', type=float, default=0.05, help='m, point-to-line distance of a wall run')
    ap.add_argument('--split', type=float, default=0.15, help='m, a run splits at a larger gap')
    ap.add_argument('--min-len', type=float, default=0.30, help='m, shortest wall run kept')
    ap.add_argument('--min-pts', type=int, default=12)
    ap.add_argument('--gap', type=float, default=0.45, help='m, largest gap closed between runs on one line')
    ap.add_argument('--snap', type=float, default=0.35, help='m, a run end this close to another wall is joined')
    ap.add_argument('--close', action='append', default=[], help='x0,y0,x1,y1: extra line (repeatable)')
    ap.add_argument('--box', type=float, nargs=4, metavar=('X0', 'Y0', 'X1', 'Y1'),
                    help='only cells inside this box (drop the hall around the arena)')
    a = ap.parse_args()

    meta, occ = read_map(a.map_yaml)
    res = float(meta['resolution'])
    ox, oy = meta['origin'][:2]
    iy, ix = np.nonzero(occ)
    P = np.c_[ox + (ix + 0.5) * res, oy + (iy + 0.5) * res]
    if a.box:
        x0, y0, x1, y1 = a.box
        P = P[(P[:, 0] >= x0) & (P[:, 0] <= x1) & (P[:, 1] >= y0) & (P[:, 1] <= y1)]
    segs = extract_segments(P, a.tol, a.split, a.min_len, a.min_pts)
    added = close_gaps(segs, a.gap, a.snap)
    for c in a.close:
        v = [float(x) for x in c.split(',')]
        if len(v) != 4:
            sys.exit(f'--close needs x0,y0,x1,y1: {c}')
        added.append((np.array(v[:2]), np.array(v[2:])))
    L = raster(segs + added, res * 0.5)
    allp = np.concatenate([P, L]).astype(np.float32)
    # dedupe on the map grid
    key = np.unique(np.floor(allp / res).astype(np.int64), axis=0)
    out_pts = ((key + 0.5) * res).astype(np.float32)

    with open(a.out + '.bin', 'wb') as f:
        f.write(struct.pack('<I', len(out_pts)) + out_pts.tobytes())
    h, w = occ.shape
    grid = occ.copy()
    for x, y in L:
        cx, cy = int((x - ox) / res), int((y - oy) / res)
        if 0 <= cx < w and 0 <= cy < h:
            grid[cy, cx] = True
    img = np.where(grid, 0, 254).astype(np.uint8)[::-1]
    open(a.out + '.pgm', 'wb').write(b'P5\n%d %d\n255\n' % (w, h) + img.tobytes())
    m2 = dict(meta)
    m2['image'] = os.path.basename(a.out) + '.pgm'
    yaml.safe_dump(m2, open(a.out + '.yaml', 'w'), default_flow_style=None, sort_keys=False)
    print(f'{len(P)} wall cells -> {len(segs)} wall runs, {len(added)} lines added '
          f'({len(a.close)} by --close) -> {a.out}.bin ({len(out_pts)} points), {a.out}.pgm/.yaml')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(14, 9))
        ax.scatter(P[:, 0], P[:, 1], s=1, c='k')
        for s0, s1 in segs:
            ax.plot([s0[0], s1[0]], [s0[1], s1[1]], '-', color='tab:blue', lw=1)
        for s0, s1 in added:
            ax.plot([s0[0], s1[0]], [s0[1], s1[1]], '-', color='red', lw=3)
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.set_title(f'{len(segs)} wall runs (blue), {len(added)} closing lines (red)')
        fig.savefig(a.out + '_preview.png', dpi=70, bbox_inches='tight')
        print(f'preview: {a.out}_preview.png')
    except ImportError:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
