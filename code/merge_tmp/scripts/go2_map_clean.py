#!/usr/bin/env python3
"""Remove cones (and other small loose objects) that got into a 2D map while mapping.

    python3 scripts/go2_map_clean.py maps/arena.yaml                       # look: lists blobs, writes preview
    python3 scripts/go2_map_clean.py maps/arena.yaml --write               # -> maps/arena_clean.yaml/.pgm
    python3 scripts/go2_map_clean.py maps/arena.yaml --keep 3.2,1.5 --erase 4.0,2.0,0.3 --write

Occupied blobs (8-connected) whose footprint fits in --max-size (default 0.30 m: a cone at wall height is
0.1-0.25 m across) and that do not touch a bigger blob are turned into free space - a cone stands on the
floor. Walls, the traffic-light stand and anything larger stay. --keep x,y keeps the blob at that map point
(a thin pole you want in the map), --erase x,y,r frees all occupied cells within r of x,y (a cone that sits
against a wall merges with it, so the size test cannot take it). Points are in map coordinates (metres,
same as RViz).

Always writes <name>_clean_preview.png: removed blobs circled red, kept small blobs blue, with their
centres and sizes printed. The original map is never changed. EGO-Planner still sees the cones in the live
scan; this only stops AMCL matching against cones that moved since mapping.
"""
import argparse
import os
import sys

import numpy as np
import yaml
from scipy import ndimage as ndi


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    toks, i = [], 0
    while len(toks) < 4:                      # P5 W H maxval, '#' comments allowed
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        toks.append(data[i:j])
        i = j
    if toks[0] != b'P5' or int(toks[3]) > 255:
        sys.exit(f'{path}: only 8-bit binary pgm (P5)')
    w, h = int(toks[1]), int(toks[2])
    return np.frombuffer(data[i + 1:i + 1 + w * h], np.uint8).reshape(h, w).copy()


def write_pgm(path, img):
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())


def pt(s, n):
    v = [float(x) for x in s.split(',')]
    if len(v) != n:
        raise argparse.ArgumentTypeError(f'need {n} comma-separated numbers: {s}')
    return v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map_yaml')
    ap.add_argument('--max-size', type=float, default=0.30, help='largest blob footprint removed (m)')
    ap.add_argument('--keep', type=lambda s: pt(s, 2), action='append', default=[], help='x,y: keep this blob')
    ap.add_argument('--erase', type=lambda s: pt(s, 3), action='append', default=[], help='x,y,r: free cells')
    ap.add_argument('--region', type=float, nargs=4, metavar=('X0', 'Y0', 'X1', 'Y1'), action='append', default=[],
                    help='only remove blobs inside these boxes (repeatable; default: whole map)')
    ap.add_argument('--near-path', type=float, default=1.0,
                    help='only blobs within this distance (m) of the mapping trajectory <name>_traj.csv, if it '
                         'exists (cones are on the course; far walls hit sparsely break into specks). 0 = off')
    ap.add_argument('--write', action='store_true', help='write <name>_clean.yaml/.pgm (default: preview only)')
    a = ap.parse_args()

    meta = yaml.safe_load(open(a.map_yaml))
    base = os.path.dirname(os.path.abspath(a.map_yaml))
    img_path = meta['image'] if os.path.isabs(meta['image']) else os.path.join(base, meta['image'])
    img = read_pgm(img_path)
    res, (ox, oy) = float(meta['resolution']), meta['origin'][:2]
    neg = int(meta.get('negate', 0))
    occ_p = (img.astype(float) / 255.0) if neg else (255 - img.astype(float)) / 255.0
    occ = occ_p > float(meta.get('occupied_thresh', 0.65))
    H = img.shape[0]

    def to_xy(r, c):
        return ox + (c + 0.5) * res, oy + (H - 1 - r + 0.5) * res

    def to_rc(x, y):
        return int(H - 1 - np.floor((y - oy) / res)), int(np.floor((x - ox) / res))

    stem0 = os.path.splitext(a.map_yaml)[0]
    traj = None
    if a.near_path > 0 and os.path.exists(stem0 + '_traj.csv'):
        traj = np.loadtxt(stem0 + '_traj.csv', delimiter=',', ndmin=2)[:, :2]
        print(f'only blobs within {a.near_path:.1f} m of {stem0}_traj.csv ({len(traj)} poses)')

    lab, n = ndi.label(occ, structure=np.ones((3, 3)))
    sl = ndi.find_objects(lab)
    keep_labels = set()
    for x, y in a.keep:
        r, c = to_rc(x, y)
        win = lab[max(r - 3, 0):r + 4, max(c - 3, 0):c + 4]   # within ~3 cells of the point
        keep_labels |= set(win[win > 0].tolist())

    out = img.copy()
    free_val = 0 if neg else 254
    removed, kept = [], []
    for k, s in enumerate(sl, 1):
        size = max(s[0].stop - s[0].start, s[1].stop - s[1].start) * res
        if size > a.max_size:
            continue
        rr, cc = np.nonzero(lab[s] == k)
        x, y = to_xy(rr.mean() + s[0].start, cc.mean() + s[1].start)
        if traj is not None and np.min(np.hypot(traj[:, 0] - x, traj[:, 1] - y)) > a.near_path:
            continue
        inside = not a.region or any(min(b[0], b[2]) <= x <= max(b[0], b[2]) and min(b[1], b[3]) <= y <= max(b[1], b[3])
                                     for b in a.region)
        item = (x, y, size, len(rr))
        if k in keep_labels or not inside:
            kept.append(item)
            continue
        out[s][lab[s] == k] = free_val
        removed.append(item)
    erased = 0
    rr, cc = np.mgrid[0:H, 0:img.shape[1]]
    for x, y, rad in a.erase:
        cx, cy = to_xy(rr, cc)
        m = occ & ((cx - x) ** 2 + (cy - y) ** 2 <= rad ** 2)
        out[m] = free_val
        erased += int(m.sum())

    print(f'{img_path}: {img.shape[1]}x{H} @ {res} m, {n} occupied blobs, {occ.sum()} occupied cells')
    print(f'removed {len(removed)} blobs <= {a.max_size:.2f} m' + (f', erased {erased} cells' if a.erase else ''))
    for x, y, size, cells in removed:
        print(f'  - removed  ({x:6.2f}, {y:6.2f})  {size:.2f} m  {cells} cells')
    for x, y, size, cells in kept:
        print(f'  - kept     ({x:6.2f}, {y:6.2f})  {size:.2f} m  {cells} cells' + ('' if a.region else '  (--keep)'))

    stem = os.path.splitext(a.map_yaml)[0]
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        ext = [ox, ox + img.shape[1] * res, oy, oy + H * res]
        fig, axs = plt.subplots(1, 2, figsize=(16, 8), sharex=True, sharey=True)
        for axx, im, t in ((axs[0], img, 'original'), (axs[1], out, f'clean: {len(removed)} removed')):
            axx.imshow(im, cmap='gray', vmin=0, vmax=255, extent=ext, interpolation='nearest')
            axx.set_title(t)
            axx.grid(alpha=0.3)
        for x, y, size, _ in removed:
            axs[0].add_patch(plt.Circle((x, y), max(size, 0.15), fill=False, color='r', lw=1.5))
        for x, y, size, _ in kept:
            axs[0].add_patch(plt.Circle((x, y), max(size, 0.15), fill=False, color='b', lw=1.5))
        for x, y, rad in a.erase:
            axs[0].add_patch(plt.Circle((x, y), rad, fill=False, color='m', lw=1.5, ls='--'))
        fig.tight_layout()
        fig.savefig(stem + '_clean_preview.png', dpi=90)
        print(f'preview: {stem}_clean_preview.png')
    except ImportError:
        print('(no matplotlib: no preview)')

    if a.write:
        name = os.path.basename(stem) + '_clean'
        write_pgm(os.path.join(base, name + '.pgm'), out)
        meta['image'] = name + '.pgm'
        with open(stem + '_clean.yaml', 'w') as f:
            yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)
        print(f'wrote {stem}_clean.yaml / {name}.pgm  (start the stack with this map)')
    else:
        print('preview only - add --write to save the cleaned map')
    return 0


if __name__ == '__main__':
    sys.exit(main())
