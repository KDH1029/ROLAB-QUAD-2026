#!/usr/bin/env python3
"""큰 콘(가운데 회색 띠)이 위·아래 두 상자로 갈라지는지, 갈라지면 바닥 위(몸체 xy)에서 얼마나 떨어져 찍히는지.
로봇 노드(traffic_light_trt_node.cones)와 같은 계산: 상자 가운데 (u,v) + cone_depth → 몸체 좌표."""
import sys, os, json, numpy as np
sys.path.insert(0, os.path.expanduser('~/ws/junsu_v2/onboard_vision'))
import cone_cam_core as K
F = os.path.expanduser('~/bags/gosu_20261105_165023_rec/yolo_1106')
d = json.load(open(F + '/dets.json')); cc = d['cone_cls']
# 컬러 내부파라미터(848x480) — 녹화의 camera_info 에서
Kc = np.array(json.load(open(os.path.dirname(os.path.abspath(__file__)) + '/Kc.json')))
def body(u, v, z):
    pc = np.array([(u - Kc[0, 2]) / Kc[0, 0] * z, (v - Kc[1, 2]) / Kc[1, 1] * z, z]) - np.array(K.DEPTH_TO_COLOR_T)
    return K.T_BODY_CAM[:, :3] @ pc + K.T_BODY_CAM[:, 3]
pairs = []; nbox = 0; nsplit_frames = 0
for r in d['rows']:
    B = [x for x in r['dets'] if x[0] == cc and x[1] >= 0.45 and x[6] is not None and x[6] <= 3.5]
    nbox += len(B); hit = False
    for i in range(len(B)):
        for j in range(len(B)):
            a, b = B[i], B[j]
            if a[3] >= b[3]: continue                      # a 가 위(y1 작음)
            wa, wb = a[4] - a[2], b[4] - b[2]
            ov = min(a[4], b[4]) - max(a[2], b[2])
            if ov < 0.5 * min(wa, wb): continue            # 가로로 겹쳐야
            gap = b[3] - a[5]                              # 위 상자 아래끝 ~ 아래 상자 위끝
            if gap > 0.6 * (a[5] - a[3]) or gap < -0.5 * (a[5] - a[3]): continue
            if abs(a[6] - b[6]) > 0.35: continue
            pa = body((a[2] + a[4]) / 2, (a[3] + a[5]) / 2, a[6]); pb = body((b[2] + b[4]) / 2, (b[3] + b[5]) / 2, b[6])
            pairs.append((r['k'], r['t'], a[1], b[1], a[6], b[6], float(np.hypot(*(pa[:2] - pb[:2]))), pa[:2].round(2).tolist(), pb[:2].round(2).tolist(), round(float(pa[2]), 2), round(float(pb[2]), 2))); hit = True
    nsplit_frames += hit
print(f'콘 상자(>=0.45, 3.5 m 안) {nbox}개, 위·아래로 쌓인 쌍 {len(pairs)}개 ({nsplit_frames} 프레임)')
D = np.array([p[6] for p in pairs])
if len(D): print('두 상자의 바닥 xy 거리 m: 최소 %.2f 중앙 %.2f 90%% %.2f 최대 %.2f' % (D.min(), np.median(D), np.percentile(D, 90), D.max()))
for p in pairs[:40]: print('#%d t=%.1f 위 %.2f(%.2fm) 아래 %.2f(%.2fm) xy차 %.2f  위%s z%.2f 아래%s z%.2f' % (p[0], p[1], p[2], p[4], p[3], p[5], p[6], p[7], p[9], p[8], p[10]))
json.dump(pairs, open(os.path.dirname(os.path.abspath(__file__)) + '/split_pairs.json', 'w'))
