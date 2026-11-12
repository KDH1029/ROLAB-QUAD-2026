#!/usr/bin/env python3
"""FusionCore 를 YOLO 포함(노드와 같은 infer 경로)으로 녹화본 앞 N 초에 돌려본다 — 타이머 5 Hz 를 bag 시각으로 흉내.
실행: source /opt/ros/jazzy/setup.bash; ~/venv/go2/bin/python test_node_yolo.py [초=90]"""
import os, sys, time, json, numpy as np, rosbag2_py
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import yd_core as C, cone_fusion_node as N
import torch; torch.set_num_threads(2)
from ultralytics import YOLO
SEC = float(sys.argv[1]) if len(sys.argv) > 1 else 90
E = np.load(f'{HERE}/dets.npz')
d = np.load('/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'); odom, lid = d['odom'], d['lid']
h2b = {round(float(h), 4): b for h, b in E['hdr2bag']}
pairs = np.array([(w, h2b[round(h, 4)]) for w, h in lid if round(h, 4) in h2b])
Kp = float(np.median(pairs[:, 0] - pairs[:, 1])); wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
dmed = float(np.median(h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]))
tL = h_o - dmed - Kp; o = np.argsort(tL); tL = tL[o]; GT = odom[o]
m = YOLO(N.DEFAULTS['model']); m.predict(np.zeros((720, 1280, 3), np.uint8), imgsz=640, verbose=False)
core = N.FusionCore(dict(N.DEFAULTS), model=m)
rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri='/home/ilovehan9283/bags/gosu_20261030_001126', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
rd.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw', '/camera/depth/image_rect_raw']))
gi = 0; t0b = None; last = -1e9; st = {}; ncone = 0; nk = 0; tstep = []; lastout = None
while rd.has_next():
    tp, raw, t = rd.read_next(); tb = t * 1e-9; t0b = t0b or tb
    if tb - t0b > SEC: break
    while gi < len(tL) and tL[gi] <= tb: core.poses.add(tL[gi], GT[gi, 2:5], GT[gi, 5:9]); gi += 1
    if tp.endswith('depth/image_rect_raw'): core.on_depth(raw, tb)
    else: core.on_color(raw, tb)
    if tb - last >= 0.2:                                     # 타이머 5 Hz
        last = tb; a = time.perf_counter(); out = core.step(tb); tstep.append(time.perf_counter() - a)
        if out is None: continue
        st[out['status']] = st.get(out['status'], 0) + 1
        ncone += sum(1 for r in out['dets'] if r['cls'] == 5); nk += 1
        if out['dets']: lastout = out
pc = N.make_cloud(lastout['cloud'], 'camera_init', N.to_time_msg(lastout['stamp'])) if lastout else None
ref = int(sum(1 for f in E['dets'] if E['frames'][int(f[0]), 2] - t0b <= SEC and f[1] == 5))
res = dict(sec=SEC, ticks=nk, status=st, cone_dets_node=ncone, cone_dets_extract_same_span=ref,
           step_ms_p50_p90_max=[round(float(np.percentile(tstep, q)) * 1e3, 1) for q in (50, 90, 100)],
           infer_ms_p50=round(float(np.median(core.t_inf)) * 1e3, 1), last_cones=lastout['cones'] if lastout else None,
           cloud_width=pc.width if pc else 0, json_len=len(json.dumps(dict(dets=lastout['dets'], cones=lastout['cones']), ensure_ascii=False)) if lastout else 0)
print(json.dumps(res, ensure_ascii=False, indent=1))
