#!/usr/bin/env python3
"""매핑 지도(PCD)에서 발산 잔해를 걷어낸다.

  python3 clean_map.py maps/go2_prior_map.pcd --out maps/go2_prior_map_clean.pcd
  python3 clean_map.py ... --radius 15 --min-neighbors 8 --preview

【왜 필요한가】
FAST-LIO 매핑 중 오도메트리가 발산하면 그 순간의 잘못된 자세로 스캔이 지도에 쌓인다.
go2_prior_map.pcd 는 152만 점 중 21%가 수백~수만 m 밖에 있었다(z 가 -21 km 까지).
이 점들은 정합(scan matching)에서 KD 트리를 키우고, 드물게 엉뚱한 대응점을 만든다.

【무엇을 지우나】
  1. 코스 반경 밖    --radius (기본 20 m). 원점 기준. 웨이포인트가 이 좌표계에 있으므로 원점을 옮기지 않는다.
  2. 높이 밖         --zmin/--zmax (바닥 평면 기준)
  3. 고립된 점       반경 --nn-radius 안에 이웃이 --min-neighbors 개 미만이면 잡음으로 본다

【좌표는 건드리지 않는다】
점을 지우기만 한다. 이미 찍어 둔 웨이포인트·구역 좌표가 그대로 맞아야 하기 때문이다.
"""
import argparse
import os
import sys

import numpy as np


def read_pcd(path):
    """PCD(binary, float32 필드) 를 읽어 (N,3) 을 돌려준다. 필드 순서는 헤더에서 찾는다."""
    with open(path, 'rb') as f:
        hdr = {}
        while True:
            ln = f.readline()
            if not ln:
                raise RuntimeError('헤더가 안 끝남: %s' % path)
            k = ln.decode('ascii', 'ignore').strip().split()
            if not k:
                continue
            hdr[k[0]] = k[1:]
            if k[0] == 'DATA':
                break
        n = int(hdr['POINTS'][0])
        sizes = [int(x) for x in hdr['SIZE']]
        rec = sum(sizes)
        if hdr['DATA'][0] != 'binary':
            raise RuntimeError('binary PCD 만 읽는다')
        a = np.frombuffer(f.read(n * rec), dtype=np.float32).reshape(-1, rec // 4)
    fields = hdr['FIELDS']
    idx = [fields.index(c) for c in ('x', 'y', 'z')]
    xyz = a[:, idx].astype(np.float64)
    return xyz[np.isfinite(xyz).all(1)]


def write_pcd(path, xyz):
    """x y z 만 담은 binary PCD 로 쓴다."""
    a = xyz.astype(np.float32)
    with open(path, 'wb') as f:
        f.write(('# .PCD v0.7 - Point Cloud Data file format\n'
                 'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                 'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'
                 % (len(a), len(a))).encode())
        f.write(a.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd')
    ap.add_argument('--out', default='')
    ap.add_argument('--radius', type=float, default=20.0, help='원점에서 이 거리 밖은 버린다 (m)')
    ap.add_argument('--zmin', type=float, default=-0.5, help='바닥 평면 기준 하한 (m)')
    ap.add_argument('--zmax', type=float, default=3.0, help='바닥 평면 기준 상한 (m)')
    ap.add_argument('--nn-radius', type=float, default=0.30, help='이웃을 셀 반경 (m)')
    ap.add_argument('--min-neighbors', type=int, default=6, help='이웃이 이보다 적으면 잡음')
    ap.add_argument('--voxel', type=float, default=0.0, help='0 보다 크면 이 크기로 복셀 다운샘플')
    a = ap.parse_args()

    src = os.path.expanduser(a.pcd)
    out = os.path.expanduser(a.out) if a.out else src.replace('.pcd', '_clean.pcd')
    xyz = read_pcd(src)
    n0 = len(xyz)
    print('원본 %s  점 %d개' % (os.path.basename(src), n0))

    # 1) 코스 반경
    r = np.hypot(xyz[:, 0], xyz[:, 1])
    xyz = xyz[r < a.radius]
    print('  반경 %.0f m 안: %d개 (%.1f%% 남음)' % (a.radius, len(xyz), 100.0 * len(xyz) / n0))
    if len(xyz) < 1000:
        print('  ★ 남은 점이 너무 적다. --radius 를 키울 것')
        return 1

    # 2) 바닥 평면 (하위 1% 를 바닥으로 보고 평면을 맞춘다)
    lo = np.percentile(xyz[:, 2], 1.0)
    base = xyz[np.abs(xyz[:, 2] - lo) < 0.5]
    if len(base) > 100:
        A = np.c_[base[:, 0], base[:, 1], np.ones(len(base))]
        coef, *_ = np.linalg.lstsq(A, base[:, 2], rcond=None)
        tilt = np.degrees(np.arctan(np.hypot(coef[0], coef[1])))
        h = xyz[:, 2] - (coef[0] * xyz[:, 0] + coef[1] * xyz[:, 1] + coef[2])
        print('  바닥 평면: 기울기 %.2f°' % tilt)
    else:
        h = xyz[:, 2] - lo
        print('  바닥: 수평 z=%.2f (평면 맞춤 실패)' % lo)
    keep = (h > a.zmin) & (h < a.zmax)
    xyz = xyz[keep]
    print('  높이 %.1f~%.1f m 안: %d개' % (a.zmin, a.zmax, len(xyz)))

    # 3) 고립점 제거
    from scipy.spatial import cKDTree
    tree = cKDTree(xyz)
    cnt = np.array(tree.query_ball_point(xyz, a.nn_radius, return_length=True))
    xyz = xyz[cnt >= a.min_neighbors]
    print('  이웃 %d개 미만 제거 (반경 %.2f m): %d개 남음' % (a.min_neighbors, a.nn_radius, len(xyz)))

    if a.voxel > 0:
        k = np.floor(xyz / a.voxel).astype(np.int64)
        _, first = np.unique(k, axis=0, return_index=True)
        xyz = xyz[np.sort(first)]
        print('  복셀 %.2f m 다운샘플: %d개' % (a.voxel, len(xyz)))

    write_pcd(out, xyz)
    print('저장 %s  (%d개, 원본의 %.1f%%)' % (out, len(xyz), 100.0 * len(xyz) / n0))
    print('  범위 x %.1f~%.1f  y %.1f~%.1f  z %.1f~%.1f'
          % (xyz[:, 0].min(), xyz[:, 0].max(), xyz[:, 1].min(), xyz[:, 1].max(),
             xyz[:, 2].min(), xyz[:, 2].max()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
