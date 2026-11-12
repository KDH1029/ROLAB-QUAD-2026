#!/usr/bin/env python3
"""매핑한 PCD 를 map_server 가 쓰는 2D 지도로 바꾸고 venue.json 에 등록한다.

  python3 tools/make_map.py ~/ws/go2_3d/maps/대회장.pcd
  python3 tools/make_map.py <pcd> --box -11 4 -6 16     상자를 직접 줄 때
  python3 tools/make_map.py <pcd> --no-register          등록은 안 하고 굽기만

【왜 이 파일이 있나 — 2026-09-13 에 여기서 반나절을 썼다】
go2_map.sh 는 PCD 까지만 만들고 "2D 로 바꾸려면 잘라서 변환하세요" 하고 끝난다.
그 사이에서 세 번 틀렸다:
  ① 기울기 보정을 안 하고 수평으로 잘랐다 → **바닥이 장애물이 됐다.**
     실측: 8.16° 기울어 있으면 수평 절단 시 바닥 점 153,875개가 벽 대역에 들어온다.
     `--floor plane` 은 셀별 바닥 평면을 맞춰 **그 평면 위 높이**로 자른다. 반드시 쓴다.
  ② 경기장 밖(아트리움·계단)까지 넣고 구웠다 → 평면 맞춤이 망가져 잔차 1.7 m.
     점 밀도로 본체를 먼저 찾아 자른다.
  ③ 성긴 점군을 붙들고 파라미터만 만졌다 → 애초에 못 쓰는 원본이었다.
     10 cm 칸당 점 수(90분위)로 먼저 재고, 낮으면 **다시 매핑하라고 말한다.**
     실측 기준: 잘 맞던 지도 96, 쓸 만함 70 이상, 못 씀 10 안팎.

새 노드도 새 토픽도 만들지 않는다. 기존 pcd_to_map.py 를 부를 뿐이다.
"""
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

GOSU = os.path.expanduser('~/gosu')
MAPS = os.path.join(GOSU, 'maps')
VENUE = os.path.join(GOSU, 'config', 'venue.json')
PCD2MAP = os.path.expanduser('~/ws/go2_3d/pcd_to_map.py')

DENSITY_GOOD = 70      # 10cm 칸당 점 수 90분위. 이 위면 쓸 만하다
DENSITY_BAD = 25       # 이 아래면 다시 매핑하는 게 빠르다


def guard(p):
    a = os.path.abspath(os.path.expanduser(p))
    if not a.startswith(os.path.abspath(GOSU)):
        raise SystemExit('★ gosu 밖에는 쓰지 않는다: %s' % a)
    return a


def read_pcd(p):
    f = open(p, 'rb'); hdr = {}
    while True:
        ln = f.readline().decode('ascii', 'ignore').strip(); k = ln.split()
        if not k:
            continue
        hdr[k[0]] = k[1:]
        if k[0] == 'DATA':
            break
    n = int(hdr['POINTS'][0])
    if hdr['DATA'][0] == 'binary':
        flds = hdr['FIELDS']; sz = [int(x) for x in hdr['SIZE']]
        rec = np.dtype({'names': flds,
                        'formats': ['<f4' if s == 4 else '<f8' for s in sz]})
        a = np.frombuffer(f.read(n * rec.itemsize), dtype=rec, count=n)
        P = np.c_[a['x'], a['y'], a['z']].astype(float)
    else:
        P = np.loadtxt(f, usecols=(0, 1, 2))
    f.close()
    return P[np.isfinite(P).all(1)]


def write_pcd(p, P):
    with open(p, 'wb') as f:
        f.write(('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
                 'TYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\n'
                 'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'
                 % (len(P), len(P))).encode())
        f.write(P.astype('<f4').tobytes())


def find_core(P, margin=1.0):
    """점이 몰린 곳 = 로봇이 실제로 돈 곳 = 경기장. 1 m 격자 밀도로 찾는다."""
    lo, hi = np.floor(P[:, :2].min(0)) - 1, np.ceil(P[:, :2].max(0)) + 1
    xe = np.arange(lo[0], hi[0] + 1, 1.0)
    ye = np.arange(lo[1], hi[1] + 1, 1.0)
    H, _, _ = np.histogram2d(P[:, 0], P[:, 1], bins=[xe, ye])
    nz = H[H > 0]
    if not len(nz):
        raise SystemExit('★ 점이 없다')
    thr = np.percentile(nz, 85)
    xs, ys = np.where(H >= thr)
    return (xe[xs.min()] - margin, xe[xs.max() + 1] + margin,
            ye[ys.min()] - margin, ye[ys.max() + 1] + margin)


def level_by_local_floor(P, cell=0.5, q=5.0):
    """칸마다 바닥을 따로 잡아 z 를 **바닥 위 높이**로 바꾼다.

    【왜 평면 하나로는 안 되나 — 2026-09-13 실측】
    pcd_to_map 의 `--floor plane` 은 지도 전체에 평면 **하나**를 맞춘다.
    지도가 통째로 기울어졌을 때는 맞지만, 매핑 드리프트로 바닥이 **휘면** 못 편다.
    Coex 매핑본은 14 m 안에서 기울기 12°, 평면 잔차 0.157 m 였다.
    벽 높이가 40~80 cm 인데 바닥 추정이 ±16 cm 흔들리면 그만큼 바닥이 벽 대역에
    새어 들어온다 — 지도 안쪽이 얼룩덜룩해지는 것이 그것이다.

    칸(기본 0.5 m)마다 z 하위 5%를 그 칸의 바닥으로 본다. 국소적으로는 평평하므로
    기울기와 드리프트가 **동시에** 사라진다. 이 뒤에는 바닥이 z=0 이므로
    pcd_to_map 을 `--floor flat` 으로 부르면 된다.
    """
    ix = np.floor(P[:, 0] / cell).astype(np.int64)
    iy = np.floor(P[:, 1] / cell).astype(np.int64)
    key = ix * 1000000 + iy
    o = np.argsort(key, kind='stable')
    Ps, ks = P[o], key[o]
    bnd = np.r_[0, np.flatnonzero(ks[1:] != ks[:-1]) + 1, len(ks)]
    h = np.empty(len(Ps))
    for a, b in zip(bnd[:-1], bnd[1:]):
        z = Ps[a:b, 2]
        h[a:b] = z - (np.percentile(z, q) if b - a >= 4 else z.min())
    return np.c_[Ps[:, 0], Ps[:, 1], h]


def floor_plane(P):
    """셀마다 가장 낮은 점으로 바닥 평면을 맞춘다. pcd_to_map --floor plane 과 같은 생각."""
    res = 0.5
    ix = np.floor(P[:, 0] / res).astype(int)
    iy = np.floor(P[:, 1] / res).astype(int)
    key = ix.astype(np.int64) * 100000 + iy
    o = np.lexsort((P[:, 2], key))
    Ps, ks = P[o], key[o]
    F = Ps[np.r_[True, ks[1:] != ks[:-1]]]
    A = np.c_[F[:, 0], F[:, 1], np.ones(len(F))]
    c, *_ = np.linalg.lstsq(A, F[:, 2], rcond=None)
    r = F[:, 2] - A @ c
    keep = np.abs(r) < 0.3
    if keep.sum() > 20:
        c, *_ = np.linalg.lstsq(A[keep], F[keep, 2], rcond=None)
        r = F[keep, 2] - A[keep] @ c
    tilt = np.degrees(np.arccos(1 / np.sqrt(c[0] ** 2 + c[1] ** 2 + 1)))
    return tilt, float(np.sqrt((r ** 2).mean())), c


def density(P, c):
    """벽 대역(바닥 위 0.3~1.5 m)의 10 cm 칸당 점 수. 지도 품질의 단일 지표."""
    h = P[:, 2] - (P[:, 0] * c[0] + P[:, 1] * c[1] + c[2])
    Q = P[(h >= 0.3) & (h < 1.5)]
    if len(Q) < 100:
        return 0, 0
    gx = np.floor(Q[:, 0] / 0.10).astype(int)
    gy = np.floor(Q[:, 1] / 0.10).astype(int)
    _, cnt = np.unique(gx.astype(np.int64) * 1000000 + gy, return_counts=True)
    return int(np.percentile(cnt, 90)), len(Q)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd')
    ap.add_argument('--box', nargs=4, type=float, default=None,
                    metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'))
    ap.add_argument('--res', type=float, default=0.03)
    ap.add_argument('--zmin', type=float, default=0.10)
    ap.add_argument('--zmax', type=float, default=2.50)
    ap.add_argument('--min-pts', type=int, default=2)
    ap.add_argument('--min-blob', type=int, default=10)
    ap.add_argument('--free-pad', type=float, default=0.9)
    ap.add_argument('--name', default='')
    ap.add_argument('--no-register', action='store_true')
    ap.add_argument('--level', dest='level', action='store_true', default=True,
                    help='칸마다 바닥을 잡아 높이를 0 으로 편다 (기본 켬). 기울기+드리프트를 같이 없앤다')
    ap.add_argument('--no-level', dest='level', action='store_false')
    ap.add_argument('--level-cell', type=float, default=0.5,
                    help='바닥을 잡는 칸 크기(m). 작을수록 드리프트에 강하지만 낮은 물체도 바닥으로 본다')
    ap.add_argument('--force', action='store_true',
                    help='밀도가 낮아도 그냥 굽는다')
    a = ap.parse_args()

    src = os.path.expanduser(a.pcd)
    if not os.path.isfile(src):
        raise SystemExit('★ 그런 파일이 없다: %s' % src)
    name = a.name or os.path.splitext(os.path.basename(src))[0]
    os.makedirs(MAPS, exist_ok=True)

    print('  원본 %s' % src)
    P = read_pcd(src)
    print('  %d점   x %.1f~%.1f  y %.1f~%.1f  z %.1f~%.1f'
          % (len(P), P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max(),
             P[:, 2].min(), P[:, 2].max()))

    box = tuple(a.box) if a.box else find_core(P)
    print('  경기장 본체 x %.1f~%.1f  y %.1f~%.1f  (%.1f x %.1f m)%s'
          % (box[0], box[1], box[2], box[3], box[1] - box[0], box[3] - box[2],
             '' if a.box else '  <- 점 밀도로 자동'))
    m = ((P[:, 0] >= box[0]) & (P[:, 0] <= box[1]) &
         (P[:, 1] >= box[2]) & (P[:, 1] <= box[3]))
    Q = P[m]
    if len(Q) < 1000:
        raise SystemExit('★ 상자 안에 점이 %d개뿐이다 — --box 로 직접 줄 것' % len(Q))

    tilt, rms, c = floor_plane(Q)
    d90, nband = density(Q, c)
    print('  잘라낸 점 %d개 (%.0f%%)' % (len(Q), 100 * len(Q) / len(P)))
    print('  바닥 기울기 %.2f도   잔차 RMS %.3f m' % (tilt, rms))
    print('  벽 대역 %d점, 10cm 칸당 90분위 %d  (기준: %d 이상 좋음 / %d 이하 못 씀)'
          % (nband, d90, DENSITY_GOOD, DENSITY_BAD))
    if tilt > 3.0:
        print('  ★ %.1f도 기울어 있다 — 수평으로 자르면 바닥이 장애물이 된다.'
              ' --floor plane 으로 보정한다' % tilt)
    if rms > 0.30:
        print('  ★ 바닥이 평면에서 %.2f m 벗어난다 — 경사·계단이 섞였거나 매핑이 흘렀다' % rms)
    if d90 < DENSITY_BAD and not a.force:
        raise SystemExit(
            '\n★ 점군이 너무 성기다 (90분위 %d). 이 지도로는 위치추정이 안 된다.\n'
            '   기록: 성긴 지도(점유칸 627)에서 일치율 35.6%%, 조밀한 것으로 바꾸니 100%%.\n'
            '   다시 매핑하는 쪽이 빠르다:  ~/gosu/gosu.sh map <이름>\n'
            '   그래도 굽고 싶으면 --force' % d90)
    if d90 < DENSITY_GOOD:
        print('  ★ 밀도가 낮은 편이다 (90분위 %d). 결과를 반드시 눈으로 볼 것' % d90)

    crop = guard(os.path.join(MAPS, '%s_core.pcd' % name))
    floor_mode = 'plane'
    if a.level:
        Q = level_by_local_floor(Q, cell=a.level_cell)
        floor_mode = 'flat'       # 이미 폈으므로 수평으로 잘라야 맞다
        t2, r2, _ = floor_plane(Q)
        print('  국소 바닥으로 폄 (%.1f m 칸): 남은 기울기 %.2f도, 잔차 %.3f m'
              % (a.level_cell, t2, r2))
    write_pcd(crop, Q)
    out = guard(os.path.join(MAPS, '%s.pgm' % name))
    cmd = ['python3', PCD2MAP, crop, '--res', str(a.res), '--floor', floor_mode,
           '--zmin', str(a.zmin), '--zmax', str(a.zmax),
           '--min-pts', str(a.min_pts), '--min-blob', str(a.min_blob),
           '--free-pad', str(a.free_pad), '--out', out]
    print('\n  굽는다: %s' % ' '.join(cmd[2:]))
    r = subprocess.run(cmd, capture_output=True, text=True)
    for ln in (r.stdout or '').splitlines():
        if any(k in ln for k in ('격자', '점유', '대역', '잡음', '★', '원점', '바닥')):
            print('    ' + ln.strip())
    if r.returncode != 0:
        print(r.stderr[-800:])
        raise SystemExit('★ 변환 실패')

    # pcd_to_map 은 --out 뒤에 .pgm 을 또 붙인다. 이름을 바로잡는다.
    for ext in ('pgm', 'yaml'):
        bad = out + '.' + ext
        good = os.path.join(MAPS, '%s.%s' % (name, ext))
        if os.path.exists(bad):
            if ext == 'yaml':
                txt = open(bad, encoding='utf-8').read().replace(
                    os.path.basename(out) + '.pgm', '%s.pgm' % name)
                open(good, 'w', encoding='utf-8').write(txt)
                os.remove(bad)
            else:
                os.replace(bad, good)
    yml = os.path.join(MAPS, '%s.yaml' % name)
    if not os.path.exists(yml):
        raise SystemExit('★ %s 가 안 만들어졌다' % yml)
    print('\n  %s' % yml)

    if a.no_register:
        print('  등록은 안 했다 (--no-register)')
        return 0
    d = {}
    if os.path.exists(VENUE):
        try:
            d = json.load(open(VENUE, encoding='utf-8'))
        except Exception:
            d = {}
    prev = (d.get('map_2d') or {}).get('yaml')
    d['map_2d'] = {
        'yaml': '%s.yaml' % name, 'path': yml, 'resolution': a.res,
        'source': src, 'box': list(box), 'tilt_deg': round(float(tilt), 2),
        'floor_rms_m': round(rms, 3), 'density_p90': d90,
        'how': ' '.join(cmd[2:]),
        'prev': prev, 'set_at': time.strftime('%Y-%m-%d %H:%M:%S'),
    }
    # 지도가 바뀌면 좌표계가 바뀐다. 옛 좌표를 남기면 반드시 잘못 쓰인다.
    for k in ('zone_poses', 'start_pose'):
        if d.pop(k, None) is not None:
            d.setdefault('_버린_좌표', {})['%s_%s' % (k, prev or '이전지도')] = {
                '왜': '지도가 바뀌어 좌표계가 달라졌다', '버린시각': d['map_2d']['set_at']}
    guard(VENUE)
    json.dump(d, open(VENUE, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('  venue.json 등록: %s -> %s.yaml' % (prev or '(없음)', name))
    print('\n  이제:  ~/gosu/gosu.sh run --no-motor   (지도가 자동으로 걸린다)')
    print('        초기위치·존 좌표는 지도가 바뀌었으니 다시 찍어야 한다')
    return 0


if __name__ == '__main__':
    sys.exit(main())
