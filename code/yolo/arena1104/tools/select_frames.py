#!/usr/bin/env python3
"""후보 사진 중 신호등·라바콘이 나온 것을 골라 원본 그대로(박스·라벨 없음) 내보낸다. ~/venv/go2 에서 실행.
사용: select_frames.py <후보폴더> <출력폴더> [장수=1500]
고르는 법: 기존 6클래스 모델(tl5cone_640) 로 찾기만 한다 — 신호등(tl_red·tl_green·unknown) 확신 ≥0.30,
          콘 확신 ≥0.30 + 박스 안 주황 픽셀 ≥12%(벽 모서리 오검출 제거). 둘 다 나온 장면 → 신호등만 → 콘만 순으로
          고르게(시간 순 등간격) 뽑아 신호등 장면과 콘 장면이 반반쯤 되게 한다. 모자라면 확신 기준을 0.20 까지 낮춘다."""
import os, sys, glob, csv, json, shutil, numpy as np, cv2
os.environ.setdefault('YOLO_AUTOINSTALL', 'false')
from ultralytics import YOLO
C, OUT = sys.argv[1], sys.argv[2]; N = int(sys.argv[3]) if len(sys.argv) > 3 else 1500
MODEL = os.path.expanduser('~/ws/datasets/tl5/runs/tl5cone_640/weights/best.pt'); TL, CONE = {0, 1, 4}, 5
fs = sorted(glob.glob(f'{C}/*.jpg')); meta = {}
for f in glob.glob(f'{C}/frames_*.csv'):
    for r in csv.reader(open(f)): meta[r[0]] = r
cache = f'{C}/detections.json'   # 중간 저장 — 꺼져도 이어서 한다
D = json.load(open(cache)) if os.path.exists(cache) else {}
todo = [f for f in fs if os.path.basename(f) not in D]
if todo:
    m = YOLO(MODEL); print(f'  검출할 것 {len(todo)}장 (이미 {len(D)}장)', flush=True)
    for i in range(0, len(todo), 8):   # ★ 목록 통째로 넘기면 전부 메모리에 올림(6858장 30 GB → OOM) → 8장씩
        for f, r in zip(todo[i:i + 8], m.predict(todo[i:i + 8], imgsz=640, conf=0.20, verbose=False, device='cpu', batch=8)):
            hsv = cv2.cvtColor(r.orig_img, cv2.COLOR_BGR2HSV); bx = []
            for b, c, p in zip(r.boxes.xyxy.numpy(), r.boxes.cls.numpy().astype(int), r.boxes.conf.numpy()):
                x1, y1, x2, y2 = b.astype(int); orange = 0.0
                if c == CONE and x2 > x1 and y2 > y1:
                    h = hsv[y1:y2, x1:x2]; orange = float((((h[..., 0] <= 22) | (h[..., 0] >= 170)) & (h[..., 1] > 100) & (h[..., 2] > 80)).mean())
                bx.append([int(c), round(float(p), 3), round(orange, 3)])
            D[os.path.basename(f)] = bx
        if (i // 8) % 40 == 0 or i + 8 >= len(todo):
            json.dump(D, open(cache + '.tmp', 'w')); os.replace(cache + '.tmp', cache)
            print(f'  검출 {len(D)}/{len(fs)}', flush=True)
def classify(th):
    g = {'both': [], 'tl': [], 'cone': []}
    for fn, bx in D.items():
        t = any(c in TL and p >= th for c, p, o in bx); k = any(c == CONE and p >= th and o >= 0.12 for c, p, o in bx)
        if t or k: g['both' if t and k else 'tl' if t else 'cone'].append(fn)
    for v in g.values(): v.sort()
    return g
for th in (0.30, 0.25, 0.20):
    g = classify(th)
    if sum(map(len, g.values())) >= N: break
def even(lst, n): return [lst[i] for i in np.linspace(0, len(lst) - 1, n).astype(int)] if n < len(lst) else list(lst)
nb = min(len(g['both']), N // 3); R = N - nb; nt = min(len(g['tl']), R // 2); nc = min(len(g['cone']), R - nt); nt = min(len(g['tl']), R - nc)
pick = even(g['both'], nb) + even(g['tl'], nt) + even(g['cone'], nc)
os.makedirs(f'{OUT}/images', exist_ok=True)
rows = [['file', 'camera', 'bag', 'frame', 'time_ns', 'scene', 'tl_count', 'cone_count']]
for fn in sorted(pick):
    shutil.copy2(f'{C}/{fn}', f'{OUT}/images/{fn}')
    bx = D[fn]; nt_ = sum(c in TL and p >= th for c, p, o in bx); nc_ = sum(c == CONE and p >= th and o >= 0.12 for c, p, o in bx)
    sc = 'both' if nt_ and nc_ else 'tl' if nt_ else 'cone'; mrow = meta.get(fn, [fn, '', '', '', ''])
    rows.append(mrow[:5] + [sc, nt_, nc_])
csv.writer(open(f'{OUT}/frames.csv', 'w')).writerows(rows)
cams = {}; [cams.__setitem__(r[1], cams.get(r[1], 0) + 1) for r in rows[1:]]
print(json.dumps({'후보': len(fs), '확신기준': th, '찾은장면': {k: len(v) for k, v in g.items()}, '내보냄': len(pick),
                  '내보낸구성': {'신호등+콘': nb, '신호등만': nt, '콘만': nc}, '카메라별': cams}, ensure_ascii=False))
# 확인용 모아보기(박스 없음) — images 폴더 밖
S = even(sorted(pick), 48); tiles = []
for fn in S:
    im = cv2.imread(f'{OUT}/images/{fn}'); im = cv2.resize(im, (320, 180)); cv2.putText(im, fn[:-4], (4, 14), 0, 0.4, (255, 255, 255), 1); tiles.append(im)
while len(tiles) % 8: tiles.append(np.zeros_like(tiles[0]))
cv2.imwrite(f'{OUT}/preview.jpg', np.vstack([np.hstack(tiles[i:i + 8]) for i in range(0, len(tiles), 8)]), [cv2.IMWRITE_JPEG_QUALITY, 85])
