#!/usr/bin/env python3
"""Check a (new) YOLO model before it goes on the robot - 5 minutes, no ROS.

    python3 yolo_model_check.py models/yolo_mission.onnx --images <dir or .mcap bag dir> [--n 40]
        [--save out_dir] [--classes "traffic_light=0 cone=1 parking=2"] [--conf 0.35]

Prints: which backend loads it (onnxruntime / OpenVINO / OpenCV - OpenCV < 4.7 cannot load
YOLOv8 heads), input size, raw output shape and the layout the node will decode it as
(v8 / v5 / e2e), the class names in the model metadata against yolo_params.yaml's class ids,
detections per class on the images. With ultralytics installed it also runs the model through
ultralytics and compares boxes (the reference decoder): our decode must match it.
Layout self-test: the same raw output re-packed as v5 and as e2e must decode to the same boxes.
Exit code 1 when something does not match.
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import yolo_detector_node as ydn  # noqa: E402  (pure functions + engines; rclpy imported, not started)


def images_from(src, n):
    if os.path.isdir(src) and glob.glob(os.path.join(src, '*.mcap')):
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=src, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
        r.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw']))
        out, k = [], 0
        while r.has_next() and len(out) < n:
            _, data, _ = r.read_next()
            k += 1
            if k % 30:
                continue
            m = deserialize_message(data, Image)
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
            out.append((f'bag#{k}', a if m.encoding.startswith('bgr') else cv2.cvtColor(a, cv2.COLOR_RGB2BGR)))
        return out
    files = sorted(sum((glob.glob(os.path.join(src, '**', e), recursive=True) for e in ('*.jpg', '*.png', '*.jpeg')), []))
    step = max(1, len(files) // n)
    return [(f, cv2.imread(f)) for f in files[::step][:n]]


def iou(a, b):
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / u if u > 0 else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('model')
    ap.add_argument('--images', required=True)
    ap.add_argument('--n', type=int, default=40)
    ap.add_argument('--conf', type=float, default=0.35)
    ap.add_argument('--size', default='640x640', help='network input WxH (yolo_params input_width/height)')
    ap.add_argument('--classes', default='traffic_light=0 cone=1 parking=2', help='yolo_params.yaml class ids')
    ap.add_argument('--save', default='')
    a = ap.parse_args()
    w, h = [int(v) for v in a.size.split('x')]
    fails = []

    eng = ydn.create_engine('auto', a.model, w, h, 'cpu', 0)
    backend = getattr(eng, 'backend', eng.__class__.__name__)
    names = getattr(eng, 'names', {})
    print(f'model   {a.model}\nbackend {backend}\nclasses {names or "(no metadata)"}')
    want = dict(kv.split('=') for kv in a.classes.split())
    for k, v in want.items():
        nm = names.get(int(v), '?')
        ok = (k.split('_')[0] in nm) or not names
        print(f'  yolo_params class_{k} = {v} -> model says "{nm}"' + ('' if ok else '   <- MISMATCH'))
        if not ok:
            fails.append(f'class id {k}={v} is "{nm}" in the model')

    imgs = images_from(a.images, a.n)
    if not imgs:
        sys.exit('no images')
    raw = eng.raw(imgs[0][1]) if hasattr(eng, 'raw') else None
    if raw is not None:
        print(f'raw output shape {tuple(raw.shape)}')

    # layout self-test on the first image: v8 raw -> v5 / e2e repack must decode the same
    if raw is not None and eng.num_classes:
        H, W = imgs[0][1].shape[:2]
        sx = sy = 1.0 / eng._r
        px, py = eng._px, eng._py
        dec = lambda arr, lay: ydn.decode_yolo(arr, eng.num_classes, lay, a.conf, 0.45, sx, sy, W, H, px, py)
        base = dec(raw, 'auto')
        p = raw[0] if raw.ndim == 3 else raw
        p = p.T if p.shape[0] < p.shape[1] else p
        v5 = np.concatenate([p[:, :4], np.ones((len(p), 1), np.float32), p[:, 4:]], axis=1)[None]
        d5 = dec(v5, 'auto')
        e2e = np.array([[d['bbox'][0] / sx + px, d['bbox'][1] / sy + py, d['bbox'][2] / sx + px, d['bbox'][3] / sy + py,
                         d['conf'], d['cls']] for d in base] or np.zeros((0, 6)), np.float32)[None]
        de = dec(e2e, 'e2e')
        same = lambda x, y: len(x) == len(y) and all(u['cls'] == v['cls'] and iou(u['bbox'], v['bbox']) > 0.95 for u, v in zip(x, y))
        print(f'layout self-test: v8 {len(base)} boxes, as v5 {len(d5)} ({"same" if same(base, d5) else "DIFFERENT"}), '
              f'as e2e {len(de)} ({"same" if same(base, de) else "DIFFERENT"})')
        if not (same(base, d5) and same(base, de)):
            fails.append('layout self-test')

    ref = None
    try:
        os.environ.setdefault('YOLO_AUTOINSTALL', 'false')   # never pip-install into the user's venv
        from ultralytics import YOLO
        ref = YOLO(a.model, task='detect')
    except Exception as e:
        print(f'(no ultralytics reference: {e})')
    counts, agree, total_ref = {}, 0, 0
    for name, img in imgs:
        dets = eng.predict(img, a.conf)
        for d in dets:
            counts[d['cls']] = counts.get(d['cls'], 0) + 1
        if ref is not None:
            try:
                r = ref.predict(img, conf=a.conf, imgsz=(h, w), verbose=False)[0]
            except Exception as e:
                print(f'(ultralytics reference failed: {str(e).splitlines()[0]})')
                ref = None
                continue
            rb = [([int(v) for v in b.xyxy[0].tolist()], int(b.cls[0])) for b in r.boxes]
            total_ref += len(rb)
            for bb, c in rb:
                if any(d['cls'] == c and iou(d['bbox'], bb) > 0.7 for d in dets):
                    agree += 1
        if a.save:
            os.makedirs(a.save, exist_ok=True)
            vis = img.copy()
            for d in dets:
                x1, y1, x2, y2 = d['bbox']
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 200, 255), 2)
                cv2.putText(vis, f"{names.get(d['cls'], d['cls'])} {d['conf']:.2f}", (x1, max(12, y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.imwrite(os.path.join(a.save, os.path.basename(str(name)).replace('#', '_') + '.jpg'), vis)
    print(f'{len(imgs)} images: detections per class ' +
          ', '.join(f'{names.get(k, k)} {v}' for k, v in sorted(counts.items())) if counts else f'{len(imgs)} images: no detections')
    if ref is not None:
        rate = agree / total_ref if total_ref else 1.0
        print(f'against ultralytics: {agree}/{total_ref} reference boxes matched (IoU > 0.7, same class) = {100 * rate:.0f} %')
        if rate < 0.9:
            fails.append(f'only {100 * rate:.0f} % of the ultralytics boxes matched')
    print('OK' if not fails else 'PROBLEMS: ' + '; '.join(fails))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
