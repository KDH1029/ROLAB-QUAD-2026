#!/usr/bin/env python3
"""① 10-30 녹화본에서 컬러 5 Hz 로 YOLO(conf 0.4) + 그 프레임에 가장 가까운 깊이 프레임을 함께 저장.

실행: source /opt/ros/jazzy/setup.bash; ~/venv/go2/bin/python yd_extract.py [max_sec]
쓰기: 이 폴더 dets.npz (검출·stamp·bag 시각·추론시간), depth_frames.npy(검출 있는 프레임 깊이), hdr2bag.npy, crops/
DDS 참여 없음 — rosbag2_py 로 파일만 읽는다.
"""
import sys, os, time, collections, json
import numpy as np
import rosbag2_py

HERE = os.path.dirname(os.path.abspath(__file__))
BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
MODEL = '/home/ilovehan9283/ws/datasets/tl5/runs/tl5cone_640/weights/best.pt'
MAXSEC = float(sys.argv[1]) if len(sys.argv) > 1 else 1e9
HZ = 5.0
CONF = 0.4
NAMES = ['tl_red', 'tl_green', 'pedestrian', 'temi', 'unknown', 'cone']

import torch
torch.set_num_threads(4)
from ultralytics import YOLO
import cv2
cv2.setNumThreads(1)
sys.path.insert(0, HERE)
from cdr_parse import parse_image, hdr_stamp

model = YOLO(MODEL)
model.predict(np.zeros((720, 1280, 3), np.uint8), imgsz=640, conf=CONF, verbose=False)   # 예열

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/lidar_points', '/camera/color/image_raw', '/camera/depth/image_rect_raw']))

hdr2bag = []                    # (lidar header stamp, bag t)
color_st = []                   # 모든 컬러 (stamp, bag t)
depth_st = []                   # 모든 깊이 (stamp, bag t)
dbuf = collections.deque(maxlen=20)   # (stamp, bagt, raw)
pending = []                    # (frame_idx) 깊이 대기
frames = []                     # dict: stamp, bagt, dets[(cls,conf,x1,y1,x2,y2)], t_inf, depth_idx, depth_stamp
depth_store = []
crop_cnt = collections.Counter()
last_sel = -1e18; t_start = None; wall0 = time.time()


def resolve(force=False):
    keep = []
    for fi in pending:
        f = frames[fi]
        newest = dbuf[-1][0] if dbuf else -1e18
        if not force and newest < f['stamp'] + 0.04 and (dbuf and dbuf[-1][1] < f['bagt'] + 0.5):
            keep.append(fi); continue
        if not dbuf:
            continue
        st = np.array([d[0] for d in dbuf]); j = int(np.argmin(np.abs(st - f['stamp'])))
        img = parse_image(dbuf[j][2])
        f['depth_idx'] = len(depth_store); f['depth_stamp'] = float(dbuf[j][0])
        depth_store.append(img.copy())
    pending[:] = keep


while r.has_next():
    tp, raw, t = r.read_next(); tb = t * 1e-9
    if t_start is None: t_start = tb
    if tb - t_start > MAXSEC: break
    if tp == '/lidar_points':
        hdr2bag.append((hdr_stamp(raw), tb)); continue
    if tp == '/camera/depth/image_rect_raw':
        s = hdr_stamp(raw); depth_st.append((s, tb)); dbuf.append((s, tb, raw))
        if pending: resolve()
        continue
    # 컬러
    s = hdr_stamp(raw); color_st.append((s, tb))
    if tb < last_sel + 1.0 / HZ - 0.01: continue
    last_sel = tb
    img = parse_image(raw)                         # rgb8 HxWx3
    bgr = np.ascontiguousarray(img[:, :, ::-1])
    t0 = time.perf_counter()
    res = model.predict(bgr, imgsz=640, conf=CONF, verbose=False)[0]
    t_inf = time.perf_counter() - t0
    b = res.boxes
    dets = np.c_[b.cls.numpy(), b.conf.numpy(), b.xyxy.numpy()] if len(b) else np.zeros((0, 6))
    f = dict(stamp=s, bagt=tb, dets=dets, t_inf=t_inf, sp=dict(res.speed), depth_idx=-1, depth_stamp=np.nan)
    frames.append(f)
    for d in dets:                                 # 클래스별 처음 12개 + 이후 20개마다 하나 잘라 저장(눈 확인용)
        c = int(d[0]); crop_cnt[c] += 1
        if crop_cnt[c] <= 12 or crop_cnt[c] % 20 == 0:
            x1, y1, x2, y2 = [int(round(v)) for v in d[2:]]
            cr = bgr[max(0, y1 - 10):y2 + 10, max(0, x1 - 10):x2 + 10]
            if cr.size: cv2.imwrite(f'{HERE}/crops/{NAMES[c]}_{len(frames) - 1:05d}_{d[1]:.2f}.jpg', cr)
    if len(dets): pending.append(len(frames) - 1)
    if len(frames) % 200 == 0:
        print(f'{tb - t_start:6.1f}s 프레임 {len(frames)} 검출 {dict(crop_cnt)} 추론 중앙 {np.median([x["t_inf"] for x in frames]) * 1e3:.1f} ms 경과 {time.time() - wall0:.0f}s', flush=True)
resolve(force=True)

D = np.array([(i, f['stamp'], f['bagt'], f['t_inf'], f['depth_idx'], f['depth_stamp'], f['sp']['preprocess'], f['sp']['inference'], f['sp']['postprocess'])
              for i, f in enumerate(frames)])
DET = np.array([(i, *d) for i, f in enumerate(frames) for d in f['dets']]).reshape(-1, 7)
np.savez(f'{HERE}/dets.npz', frames=D, dets=DET, color_st=np.array(color_st), depth_st=np.array(depth_st),
         hdr2bag=np.array(hdr2bag), names=np.array(NAMES))
np.save(f'{HERE}/depth_frames.npy', np.array(depth_store, dtype=np.uint16) if depth_store else np.zeros((0, 480, 848), np.uint16))
print('끝: 프레임', len(frames), '검출', len(DET), '깊이 저장', len(depth_store), '경과', round(time.time() - wall0), 's')
print('클래스별 검출', {NAMES[int(c)]: int((DET[:, 1] == c).sum()) for c in np.unique(DET[:, 1])} if len(DET) else {})
