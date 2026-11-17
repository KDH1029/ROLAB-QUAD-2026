#!/usr/bin/env python3
"""녹화에서 /lidar_points 의 한 방위 구간을 지운 사본을 만든다(+ /lowstate 그대로) — "라이다 뒤가 가려지면 FAST-LIO 가 흔들리나" A/B 용.
라이다 좌표 방위(도, atan2(y,x) 0~360) [az0, az1] 안 점을 뺀다. az0 == az1 이면 아무것도 안 지운 대조군.
사용: [RMAX=8] bag_cut_sector.py <원본 bag> <출력 bag> <az0> <az1> [앞 몇 초=0(전부)]
  RMAX 를 주면 수평 거리 RMAX m 넘는 점도 지운다(먼 관중·부스가 흔드는지 A/B)."""
import sys, os, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

src, dst = os.path.expanduser(sys.argv[1]), os.path.expanduser(sys.argv[2]); az0, az1 = float(sys.argv[3]), float(sys.argv[4])
TMAX = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
TOPICS = ['/lidar_points', '/lowstate']
RMAX = float(os.environ.get('RMAX', '0'))
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=src, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
w = rosbag2_py.SequentialWriter(); w.open(rosbag2_py.StorageOptions(uri=dst, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
for i, tp in enumerate(TOPICS):
    w.create_topic(rosbag2_py.TopicMetadata(id=i, name=tp, type=types[tp], serialization_format='cdr'))
Pc = get_message('sensor_msgs/msg/PointCloud2')
DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
t0 = None; nc = nl = 0; kept = tot = 0
while r.has_next():
    tp, raw, t = r.read_next()
    if t0 is None: t0 = t
    if TMAX and (t - t0) * 1e-9 > TMAX: break
    if tp == '/lidar_points' and (az0 != az1 or RMAX > 0):
        m = deserialize_message(raw, Pc)
        dt = np.dtype({'names': [f.name for f in m.fields], 'formats': ['<' + DT[f.datatype] for f in m.fields],
                       'offsets': [f.offset for f in m.fields], 'itemsize': m.point_step})
        a = np.frombuffer(bytes(m.data), dtype=dt, count=m.width * m.height)
        az = np.degrees(np.arctan2(a['y'].astype(np.float64), a['x'].astype(np.float64))) % 360
        keep = ~((az >= az0) & (az <= az1)) if az0 != az1 else np.ones(len(a), bool)
        if RMAX > 0: keep &= np.hypot(a['x'].astype(np.float64), a['y'].astype(np.float64)) <= RMAX
        b = a[keep]
        m.data = b.tobytes(); m.height = 1; m.width = int(len(b)); m.row_step = m.point_step * m.width
        kept += len(b); tot += len(a); raw = serialize_message(m)
    w.write(tp, raw, t); nc += tp == '/lidar_points'; nl += tp == '/lowstate'
del w
cut = (f' · 방위 {az0:.0f}~{az1:.0f}° 지움' if az0 != az1 else '') + (f' · {RMAX:g} m 넘는 점 지움' if RMAX > 0 else '')
print(f'{dst}: 스캔 {nc} · lowstate {nl}' + (f'{cut} → 점 {kept / max(tot, 1) * 100:.1f}% 남김' if cut else ' · 대조군(안 지움)'))
