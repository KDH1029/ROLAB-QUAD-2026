#!/usr/bin/env python3
"""녹화된 D435 영상에 학습 모델(ONNX)을 돌려 신호 판정을 시간 순으로 본다 — 신호가 5초마다 바뀌므로, 보고 있는 동안
판정이 약 5초 간격으로 RED↔GREEN 을 오가면 모델이 신호를 제대로 읽는 것이다. 재생·노드 없음(녹화 파일만 읽음).
전처리·후처리·판정은 내부 PC 노드와 같은 tl_yolo_common(letterbox · decode conf 0.25 · judge).
사용(OpenCV 4.11 필요 → venv): source /opt/ros/jazzy/setup.bash; ~/venv/go2/bin/python3 tl_timeline.py <카메라 mcap 폴더> <출력 앞이름> [Hz=5]"""
import sys, os, json, numpy as np, cv2, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tl_yolo_common as C

SRC, OUT = os.path.expanduser(sys.argv[1]), os.path.expanduser(sys.argv[2]); HZ = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
net = cv2.dnn.readNetFromONNX(C.onnx_path(os.path.dirname(os.path.abspath(__file__))))   # TL_MODEL(기본 tl_cone_1104)
Img = get_message('sensor_msgs/msg/Image')
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=SRC, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw']))
rows = []; last = -1e9; t0 = None; keep_imgs = {}
while r.has_next():
    _, raw, tb = r.read_next(); m = deserialize_message(raw, Img); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if t0 is None: t0 = t
    if t - last < 1.0 / HZ - 1e-3: continue
    last = t
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    bgr = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a
    blob, sc, left, top = C.letterbox(bgr); net.setInput(blob); out = net.forward()
    dets = C.decode(out, sc, left, top, (m.height, m.width), conf_thr=0.25); state, conf = C.judge(dets)
    tl = [d for d in dets if d[0] in (C.RED, C.GREEN)]
    hmax = max(((d[5] - d[3]) if len(d) == 6 else (d[2][3] - d[2][1]) for d in tl), default=0.0)   # 신호등 상자 높이(px) — 멀면 작다
    rows.append((t - t0, state, conf, hmax, sum(d[0] == C.RED for d in tl), sum(d[0] == C.GREEN for d in tl)))
    if state != 'UNKNOWN' and len(keep_imgs) < 400: keep_imgs[round(t - t0, 1)] = bgr
R = np.array([(x[0], {'RED': 0, 'GREEN': 1, 'UNKNOWN': -1}[x[1]], x[2], x[3]) for x in rows])
json.dump([list(map(lambda v: v if isinstance(v, str) else float(v), x)) for x in rows], open(OUT + '.json', 'w'))
t, s, cf, hh = R.T
print(f'{SRC}: {len(R)} 프레임({HZ:g} Hz, {t[-1]:.0f} 초) · 판정 RED {np.mean(s == 0) * 100:.1f}% · GREEN {np.mean(s == 1) * 100:.1f}% · UNKNOWN {np.mean(s == -1) * 100:.1f}%')
# 신호를 보고 있는 구간(판정이 2 초 넘게 끊기지 않음 — 1 초 이하 UNKNOWN 은 잇는다)
seen = s >= 0; segs = []; i = 0
while i < len(R):
    if not seen[i]: i += 1; continue
    j = i
    while j + 1 < len(R) and (seen[j + 1] or (j + 1 < len(R) and np.any(seen[j + 1:j + 1 + int(HZ)]) )):
        j += 1
    if t[j] - t[i] >= 2.0: segs.append((i, j))
    i = j + 1
print(f'신호를 2 초 넘게 계속 본 구간 {len(segs)} 개')
gaps = []
for i, j in segs:
    ss = s[i:j + 1]; tt = t[i:j + 1]; v = ss >= 0
    # 상태가 바뀐 시각(UNKNOWN 은 건너뛰고 앞뒤 상태로)
    st = ss[v]; tv = tt[v]; ch = np.where(np.diff(st) != 0)[0]
    trans = [(tv[k + 1], 'R→G' if st[k + 1] == 1 else 'G→R') for k in ch]
    gaps += list(np.diff([x[0] for x in trans]))
    print(f' {tt[0]:6.1f}~{tt[-1]:6.1f}초 ({tt[-1] - tt[0]:4.1f}초) · 판정률 {v.mean() * 100:3.0f}% · RED {np.mean(st == 0) * 100:3.0f}% · 바뀜 ' +
          ', '.join(f'{a:.1f}s {b}' for a, b in trans[:12]) + (' …' if len(trans) > 12 else '') + f' · 신호등 크기 중앙 {np.median(hh[i:j + 1][v]):.0f}px · 확신 중앙 {np.median(cf[i:j + 1][v]):.2f}')
if gaps:
    g = np.array(gaps); print(f'바뀜 사이 간격: {len(g)} 개 · 중앙 {np.median(g):.2f} 초 · 4~6 초 안 {np.mean((g > 4) & (g < 6)) * 100:.0f}% · 2 초 미만(깜빡임/오판) {np.mean(g < 2) * 100:.0f}%')
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = ['NanumGothic', 'DejaVu Sans']
fig, ax = plt.subplots(figsize=(16, 3.6), dpi=90)
col = np.where(s == 0, 'red', np.where(s == 1, 'limegreen', '0.8'))
ax.scatter(t, np.where(s >= 0, cf, 0.02), c=col, s=6); ax.set_ylim(0, 1.02); ax.set_xlabel('녹화 시각 (초)'); ax.set_ylabel('판정 확신')
ax.set_title(f'{os.path.basename(os.path.dirname(SRC))} — 학습 모델 신호 판정 (빨강=RED, 초록=GREEN, 회색=UNKNOWN)'); ax.grid(alpha=0.3)
fig.tight_layout(); fig.savefig(OUT + '.png'); print('그림', OUT + '.png')
