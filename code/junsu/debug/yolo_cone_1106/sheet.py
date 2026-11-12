#!/usr/bin/env python3
"""dets.json + frames → 상자 그린 모음 그림. 사용: sheet.py <yolo 폴더> <출력.jpg> <시작s> <끝s> <간격s> [열=5]"""
import sys, json, cv2, numpy as np
F, OUT, a, b, st = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5]); NC = int(sys.argv[6]) if len(sys.argv) > 6 else 5
d = json.load(open(F + '/dets.json')); cc = d['cone_cls']; col = {0: (0, 0, 255), 1: (0, 255, 0), 2: (255, 255, 255), cc: (0, 255, 255)}
tiles = []; nxt = a
for r in d['rows']:
    if r['t'] < nxt or r['t'] > b: continue
    nxt = r['t'] + st; im = cv2.imread(f"{F}/frames/{r['k']:04d}.jpg")
    for c, s, x1, y1, x2, y2, z in r['dets']:
        if s < 0.45 and c == cc: th = 1
        else: th = 3
        cv2.rectangle(im, (int(x1), int(y1)), (int(x2), int(y2)), col.get(c, (255, 0, 255)), th)
        cv2.putText(im, f"{d['names'][str(c)][:4]} {s:.2f}" + (f" {z:.1f}m" if z is not None else ''), (int(x1), max(14, int(y1) - 4)), 0, 0.6, col.get(c, (255, 0, 255)), 2)
    n = sum(1 for x in r['dets'] if x[0] == cc and x[1] >= 0.45)
    cv2.rectangle(im, (0, 0), (300, 34), (0, 0, 0), -1); cv2.putText(im, f"#{r['k']} t={r['t']:.1f}s cone {n}", (6, 25), 0, 0.8, (255, 255, 255), 2)
    tiles.append(cv2.resize(im, (530, 300)))
tiles += [np.zeros_like(tiles[0])] * ((-len(tiles)) % NC)
cv2.imwrite(OUT, np.vstack([np.hstack(tiles[i:i + NC]) for i in range(0, len(tiles), NC)]), [cv2.IMWRITE_JPEG_QUALITY, 80]); print(OUT, len(tiles))
