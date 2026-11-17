#!/usr/bin/env python3
"""PCD 판독 — 필드 순서를 헤더에서 찾는다.

지도마다 FIELDS 순서가 다르다(go2_prior_map 은 x y z intensity normal_*,
우리 매핑본은 x y z normal_* intensity). 앞 3개를 xyz 로 가정하면 조용히 틀어진다.
"""
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


