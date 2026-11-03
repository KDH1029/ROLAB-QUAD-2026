#!/usr/bin/env python3
"""Colour frames from a bag -> images for YOLO labelling / training (no ROS graph, reads the file).

    python3 bag_to_images.py ~/bags/<bag> --out ~/ws/datasets/arena_1104 [--every 0.5] [--start 0] [--duration 0]

One frame every --every seconds (bag time); file name = bag time + camera stamp, so a frame can be
found again in the bag. Skips near-duplicate frames (the robot standing still) unless --keep-still.
Writes a list.csv (file, bag_s, stamp) next to the images. Label them (Roboflow / CVAT / labelImg)
with the class ids of yolo_params.yaml (0 traffic_light, 1 traffic_cone, 2 parking_sign).
"""
import argparse
import csv
import os
import sys

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--out', required=True)
    ap.add_argument('--topic', default='/camera/color/image_raw')
    ap.add_argument('--every', type=float, default=0.5, help='s of bag time between frames')
    ap.add_argument('--start', type=float, default=0.0)
    ap.add_argument('--duration', type=float, default=0.0, help='0 = to the end')
    ap.add_argument('--keep-still', action='store_true', help='keep frames that look the same as the last one')
    a = ap.parse_args()
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    if a.topic not in {t.name for t in r.get_all_topics_and_types()}:
        sys.exit(f'{a.topic} is not in the bag')
    r.set_filter(rosbag2_py.StorageFilter(topics=[a.topic]))
    os.makedirs(a.out, exist_ok=True)
    t0 = last_t = None
    last_small = None
    n = skipped = 0
    rows = []
    while r.has_next():
        _, data, t = r.read_next()
        t *= 1e-9
        t0 = t if t0 is None else t0
        s = t - t0
        if s < a.start:
            continue
        if a.duration and s > a.start + a.duration:
            break
        if last_t is not None and s - last_t < a.every:
            continue
        m = deserialize_message(data, Image)
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
        if not m.encoding.startswith('bgr'):
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        small = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (80, 45)).astype(np.float32)
        if not a.keep_still and last_small is not None and np.mean(np.abs(small - last_small)) < 2.0:
            skipped += 1
            last_t = s
            continue
        last_small, last_t = small, s
        name = f'{s:08.2f}s_{m.header.stamp.sec}_{m.header.stamp.nanosec:09d}.jpg'
        cv2.imwrite(os.path.join(a.out, name), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        rows.append((name, f'{s:.3f}', f'{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}'))
        n += 1
    with open(os.path.join(a.out, 'list.csv'), 'a', newline='') as f:
        csv.writer(f).writerows(rows)
    print(f'{n} frames -> {a.out} ({skipped} near-duplicates skipped)')


if __name__ == '__main__':
    main()
