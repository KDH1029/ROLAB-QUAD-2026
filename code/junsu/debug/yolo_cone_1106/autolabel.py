#!/usr/bin/env python3
"""바뀐 콘 자동 라벨 — 16:50 카메라 녹화 → YOLO 형식(클래스 0 tl_red · 1 tl_green · 2 unknown · 3 cone).

규칙(프레임마다):
  1) 지금 모델(tl_cone_1104, conf 0.25)의 상자에서 출발
  2) 큰 콘이 회색 띠로 위·아래 두 상자로 갈라진 것 → 하나로 합침
     (가로로 겹침 + 위 상자 아래끝~아래 상자 위끝 틈이 작음 + 새 장착값으로 바닥 위 두 점이 0.2 m 안)
  3) 모델이 놓친 콘을 색+뎁스로 채움 — 주황(큰 콘 가까이·잘린 콘), 연두(새 색 콘, H 32~44)
     주황 덩어리들도 2) 와 같이 위·아래로 합침. 뎁스 3.0 m 넘는 것(TV 화면·관중)은 버림
  4) 콘 상자 안에 들어간 신호등 상자(회색 띠를 빨강으로 본 것) 지움. 신호등·unknown 은 0.5 이상만 둠
  5) 콘 상자 확률 0.25~0.45(모델이 애매해 한 것) → 그대로 두되 '검토' 표시
출력: <OUT>/images, labels(YOLO txt), review/*.jpg(색: 초록=모델 그대로, 보라=합침, 하늘=연두 추가, 주황=주황 추가, 노랑=애매),
      review.csv(검토가 필요한 프레임)
사용: source /opt/ros/jazzy/setup.bash; ~/venv/go2/bin/python3 autolabel.py [OUT=~/ws/datasets/cone_auto_1106]
"""
import sys, os, json, csv, numpy as np, cv2, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
OV = os.path.expanduser('~/ws/junsu_v2/onboard_vision'); sys.path.insert(0, OV)
import tl_yolo_common as C
import cone_cam_core as K
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.expanduser('~/bags/gosu_20261105_165023_rec/camera_nuctime_p1')
OUT = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/ws/datasets/cone_auto_1106')
for s in ('images', 'labels', 'review'): os.makedirs(f'{OUT}/{s}', exist_ok=True)
N = json.load(open(HERE + '/T_body_cam_1650.json')); RN, TN = np.array(N['R']), np.array(N['t'])   # 16:50 장착값
LIME_WIN = [(73.0, 76.0), (92.0, 95.6), (132.5, 134.2), (142.5, 144.2)]   # 연두 콘이 보이는 구간 → 15 Hz 로 촘촘히
CONE = 3


def body_xy(u, v, z, Kc):
    pc = np.array([(u - Kc[0, 2]) / Kc[0, 0] * z, (v - Kc[1, 2]) / Kc[1, 1] * z, z]) - np.array(K.DEPTH_TO_COLOR_T)
    return (RN @ pc + TN)[:2]


def region_depth(proj, x1, y1, x2, y2, mask=None, pct=25):
    uc, vc, zc = proj
    s = (uc >= x1) & (uc < x2) & (vc >= y1) & (vc < y2)
    if mask is not None and s.any():
        ui = np.clip(uc[s].astype(int), 0, mask.shape[1] - 1); vi = np.clip(vc[s].astype(int), 0, mask.shape[0] - 1)
        idx = np.where(s)[0]; s2 = np.zeros_like(s); s2[idx[mask[vi, ui] > 0]] = True; s = s2
    return float(np.percentile(zc[s], pct)) if s.sum() >= 4 else float('nan')


def same_cone(a, b, proj, Kc):
    """a, b = [x1,y1,x2,y2] — a 가 위. 같은 (큰) 콘의 위·아래 조각인가."""
    if a[1] > b[1]: a, b = b, a
    wa, wb = a[2] - a[0], b[2] - b[0]; ha = a[3] - a[1]
    ov = min(a[2], b[2]) - max(a[0], b[0])
    if ov < 0.5 * min(wa, wb): return False
    gap = b[1] - a[3]
    if gap > 1.8 * ha + 6 or gap < -0.6 * ha: return False
    if wa > 1.6 * wb: return False                           # 콘은 위가 좁다
    za = region_depth(proj, *a); zb = region_depth(proj, *b)
    if not (np.isfinite(za) and np.isfinite(zb)): return gap < 0.4 * ha + 4   # 뎁스 없으면(너무 가까움) 모양만
    if gap > 1.0 * ha + 6 and min(za, zb) > 1.2: return False                # 띠가 길어 보이는 건 가까운 콘뿐
    pa = body_xy((a[0] + a[2]) / 2, (a[1] + a[3]) / 2, za, Kc); pb = body_xy((b[0] + b[2]) / 2, (b[1] + b[3]) / 2, zb, Kc)
    return np.hypot(*(pa - pb)) < 0.20


def merge_stack(boxes, proj, Kc):
    """boxes: list of dict(b=[x1,y1,x2,y2], src, conf) → 같은 콘끼리 합침(union-find)."""
    n = len(boxes); par = list(range(n))
    def f(i):
        while par[i] != i: par[i] = par[par[i]]; i = par[i]
        return i
    for i in range(n):
        for j in range(i + 1, n):
            if f(i) != f(j) and same_cone(boxes[i]['b'], boxes[j]['b'], proj, Kc): par[f(i)] = f(j)
    g = {}
    for i in range(n): g.setdefault(f(i), []).append(boxes[i])
    out = []
    for grp in g.values():
        if len(grp) == 1: out.append(grp[0]); continue
        B = np.array([x['b'] for x in grp]); srcs = {x['src'] for x in grp}
        out.append({'b': [B[:, 0].min(), B[:, 1].min(), B[:, 2].max(), B[:, 3].max()], 'src': 'merge' if srcs <= {'model', 'weak'} else 'merge+' + '+'.join(sorted(srcs - {'model', 'weak'})),
                    'conf': max(x['conf'] for x in grp)})
    return out


def color_blobs(hsv, kind, proj, H, W):
    if kind == 'orange':
        m = cv2.inRange(hsv, (K.H_LO, K.S_MIN, K.V_MIN), (K.H_HI, 255, 255))
    else:
        m = cv2.inRange(hsv, (32, 110, 80), (44, 255, 255)); m[:H // 3] = 0        # 위 1/3 = 천장 조명·전시장
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)); m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    out = []
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if a < (150 if kind == 'lime' else 300): continue
        fill = a / float(w * h)
        if fill < 0.30 or w > 4 * h + 10: continue                              # 가늘고 긴 것(바닥 선) 버림
        z = region_depth(proj, x, y, x + w, y + h, (lab == i).astype(np.uint8))
        touch = y + h >= H - 2 or y <= 1 or x <= 1 or x + w >= W - 2
        if kind == 'orange' and w < 0.3 * h and not touch: continue             # 세로로 가는 주황 = 벽 모서리 반사
        if np.isfinite(z) and z > 3.0: continue                                  # 먼 것(TV 화면·관중·전시장)
        if not np.isfinite(z) and not touch: continue                            # 뎁스도 없고 화면 끝도 아님 → 모름
        out.append({'b': [float(x), float(y), float(x + w), float(y + h)], 'src': kind, 'conf': 0.0, 'area': int(a)})
    return out


def cover(b, boxes):
    """b 넓이 중 boxes 로 덮인 비율(최대)."""
    ab = max((b[2] - b[0]) * (b[3] - b[1]), 1)
    r = 0
    for o in boxes:
        ix = max(0, min(b[2], o[2]) - max(b[0], o[0])); iy = max(0, min(b[3], o[3]) - max(b[1], o[1])); r = max(r, ix * iy / ab)
    return r


def main():
    net = cv2.dnn.readNetFromONNX(C.onnx_path(OV))
    Img, Info = get_message('sensor_msgs/msg/Image'), get_message('sensor_msgs/msg/CameraInfo')
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=SRC, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw', '/camera/depth/image_rect_raw', '/camera/color/camera_info', '/camera/depth/camera_info']))
    Kc = Kd = None; dq = []; t0 = None; last = -1e9; rows = []; cnt = {}
    col = {'model': (0, 220, 0), 'merge': (255, 0, 255), 'lime': (255, 255, 0), 'orange': (0, 140, 255), 'weak': (0, 255, 255)}
    while r.has_next():
        tp, raw, _ = r.read_next()
        if tp.endswith('camera_info'):
            m = deserialize_message(raw, Info); Kx = np.array(m.k).reshape(3, 3)
            if 'color' in tp: Kc = Kx
            else: Kd = Kx
            continue
        m = deserialize_message(raw, Img); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if tp.endswith('image_rect_raw'): dq.append((t, m)); dq = dq[-6:]; continue
        if t0 is None: t0 = t
        tr = t - t0; hz = 15.0 if any(a <= tr <= b for a, b in LIME_WIN) else 5.0
        if t - last < 1 / hz - 1e-3 or Kc is None or Kd is None or not dq: continue
        last = t; td, dm = min(dq, key=lambda x: abs(x[0] - t))
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1); bgr = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()
        H, W = bgr.shape[:2]
        D = np.frombuffer(dm.data, np.uint16).reshape(dm.height, dm.step // 2)[:, :dm.width] if abs(td - t) < 0.05 else np.zeros((dm.height, dm.width), np.uint16)
        proj = C.project_depth(D, Kd.astype(np.float32), Kc.astype(np.float32))
        blob, sc, l, tp_ = C.letterbox(bgr); net.setInput(blob); dets = C.decode(net.forward(), sc, l, tp_, (H, W), conf_thr=0.25)
        cones = [{'b': list(x[2:6]), 'src': 'model' if x[1] >= 0.45 else 'weak', 'conf': x[1]} for x in dets if x[0] == CONE]
        cones = [c for c in cones if not (c['src'] == 'weak' and region_depth(proj, *c['b']) > 3.5)]   # 먼 데 약한 상자(전시장 물건)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        tl_boxes = [x for x in dets if x[0] in (0, 1) and x[1] >= 0.25]
        add = []
        for kind in ('lime',):        # 주황 덩어리는 반사·겹친 콘 때문에 오답이 더 많아 뺌(11-06 검토)
            for bl in color_blobs(hsv, kind, proj, H, W):
                if kind == 'lime' and cover(bl['b'], [x[2:6] for x in tl_boxes]) > 0.2: continue    # 신호등 초록 불
                lim = 0.1 if kind == 'orange' else 0.5     # 주황 덩어리는 겹친 콘 여럿을 한 덩어리로 묶는다 → 모델 상자와 조금이라도 겹치면 안 씀
                if cover(bl['b'], [c['b'] for c in cones]) > lim: continue
                add.append(bl)
        # 주황 덩어리 중 작은 것은 큰 콘 조각일 때만 의미 — 모델 상자와 합쳐지거나 서로 합쳐져야 남김
        allb = merge_stack(cones + add, proj, Kc)
        final = []
        for x in allb:
            w, h = x['b'][2] - x['b'][0], x['b'][3] - x['b'][1]
            x1_, y1_, x2_, y2_ = x['b']; touch = y2_ >= H - 2 or y1_ <= 1 or x1_ <= 1 or x2_ >= W - 2
            if x['src'] == 'orange' and not touch and x.get('area', 10 ** 6) < 3000: continue      # 홀로 남은 작은 주황 = 버림(콘 조각 아님)
            if x['src'] == 'lime' and h < 12: continue
            if h < 8 or w < 5: continue
            final.append(x)
        keep = []
        for i, x in enumerate(final):                 # 큰 상자 안에 80% 넘게 들어간 작은 상자 — 같은 거리면 같은 콘 조각
            a_ = (x['b'][2] - x['b'][0]) * (x['b'][3] - x['b'][1]); za = region_depth(proj, *x['b']); dup = False
            for j, y in enumerate(final):
                if i == j: continue
                b_ = (y['b'][2] - y['b'][0]) * (y['b'][3] - y['b'][1])
                if b_ <= a_ or cover(x['b'], [y['b']]) < 0.8: continue
                zb = region_depth(proj, *y['b'])
                if not (np.isfinite(za) and np.isfinite(zb)) or abs(za - zb) < 0.35: dup = True; break
            if not dup: keep.append(x)
        final = keep
        cone_b = [x['b'] for x in final]
        tls = [x for x in dets if x[0] in (0, 1, 2) and x[1] >= 0.5 and cover(list(x[2:6]), cone_b) < 0.5]
        dropped_tl = [x for x in dets if x[0] in (0, 1) and x[1] >= 0.25 and cover(list(x[2:6]), cone_b) >= 0.5]
        k = len(rows); name = f'gosu1650_{k:04d}_t{tr:06.1f}'
        cv2.imwrite(f'{OUT}/images/{name}.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
        with open(f'{OUT}/labels/{name}.txt', 'w') as fo:
            for x in final:
                x1, y1, x2, y2 = x['b']; fo.write(f'3 {(x1 + x2) / 2 / W:.6f} {(y1 + y2) / 2 / H:.6f} {(x2 - x1) / W:.6f} {(y2 - y1) / H:.6f}\n')
            for c, s, x1, y1, x2, y2 in tls:
                fo.write(f'{c} {(x1 + x2) / 2 / W:.6f} {(y1 + y2) / 2 / H:.6f} {(x2 - x1) / W:.6f} {(y2 - y1) / H:.6f}\n')
        srcs = [x['src'].split('+')[0] for x in final]
        for s_ in srcs: cnt[s_] = cnt.get(s_, 0) + 1
        need = any(s_ != 'model' for s_ in srcs) or bool(dropped_tl)
        rows.append({'name': name, 't': round(tr, 2), 'n_cone': len(final), 'src': '|'.join(x['src'] for x in final), 'tl': len(tls), 'tl_dropped': len(dropped_tl), 'review': int(need)})
        if need:
            vis = bgr.copy()
            for x in final:
                c = col.get(x['src'].split('+')[0], (255, 255, 255))
                cv2.rectangle(vis, (int(x['b'][0]), int(x['b'][1])), (int(x['b'][2]), int(x['b'][3])), c, 3)
                cv2.putText(vis, x['src'], (int(x['b'][0]), max(14, int(x['b'][1]) - 4)), 0, 0.55, c, 2)
            for c_, s, x1, y1, x2, y2 in tls: cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255) if c_ == 0 else (0, 255, 0), 2)
            for c_, s, x1, y1, x2, y2 in dropped_tl:
                cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (80, 80, 80), 1); cv2.putText(vis, 'tl drop', (int(x1), int(y2) + 14), 0, 0.5, (80, 80, 80), 2)
            cv2.rectangle(vis, (0, 0), (330, 30), (0, 0, 0), -1); cv2.putText(vis, f'{k} t={tr:.1f}s', (6, 22), 0, 0.7, (255, 255, 255), 2)
            cv2.imwrite(f'{OUT}/review/{name}.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if k % 150 == 0: print(k, f't={tr:.1f}', flush=True)
    with open(f'{OUT}/review.csv', 'w', newline='') as fo:
        w = csv.DictWriter(fo, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    open(f'{OUT}/classes.txt', 'w').write('tl_red\ntl_green\nunknown\ncone\n')
    print('끝', len(rows), '장 · 콘 상자 출처', cnt, '· 검토할 프레임', sum(r_['review'] for r_ in rows), '·', OUT)


main()
