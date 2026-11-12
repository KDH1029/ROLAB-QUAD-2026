"""YOLOv8n(best.pt) CPU 추론 시간 — torch 스레드 1/2/4, imgsz 640, 1280x720 입력 (녹화본 컬러 60장)."""
import sys, time, numpy as np, rosbag2_py, torch
from ultralytics import YOLO
from cdr_parse import parse_image
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri='/home/ilovehan9283/bags/gosu_20261030_001126', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw']))
ims = []; k = 0
while r.has_next() and len(ims) < 60:
    tp, raw, t = r.read_next(); k += 1
    if k % 40 == 0: ims.append(np.ascontiguousarray(parse_image(raw)[:, :, ::-1]))
m = YOLO('/home/ilovehan9283/ws/datasets/tl5/runs/tl5cone_640/weights/best.pt')
print('| torch 스레드 | 전체 p50 ms | p90 ms | 신경망만 p50 ms | 전처리 p50 | 후처리 p50 | 입력 텐서 |')
print('|---|---|---|---|---|---|---|')
for th in [int(x) for x in (sys.argv[1] if len(sys.argv) > 1 else "1,2,4").split(",")]:
    torch.set_num_threads(th)
    for im in ims[:5]: m.predict(im, imgsz=640, conf=0.4, verbose=False)
    T = []; S = []
    for im in ims:
        t0 = time.perf_counter(); res = m.predict(im, imgsz=640, conf=0.4, verbose=False)[0]; T.append(time.perf_counter() - t0); S.append([res.speed['preprocess'], res.speed['inference'], res.speed['postprocess']])
    S = np.array(S)
    print(f'| {th} | {np.median(T)*1e3:.1f} | {np.percentile(T,90)*1e3:.1f} | {np.median(S[:,1]):.1f} | {np.median(S[:,0]):.1f} | {np.median(S[:,2]):.1f} | {tuple(res.orig_shape)}→640 rect |')
