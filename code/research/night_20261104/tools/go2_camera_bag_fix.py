#!/usr/bin/env python3
"""내부 PC(로봇 PC)에서 녹화한 카메라 bag 의 시각을 NUC 시계로 바꿔 mcap 으로 다시 쓴다.

시각 모델: go2_clock_probe.py 의 CSV(nuc_time, offset=로봇PC−NUC, rtt) 중 왕복이 짧은 표본으로
          offset(t) = a + b·(t − t0) 직선 맞춤(표본이 적으면 중앙값 상수). 오차 ≈ 잔차 + 왕복/2.
바꾸는 것: 메시지 수신 시각, 그리고 첫 필드가 std_msgs/Header 인 메시지(Image·CameraInfo·realsense Metadata·Imu)의
          header.stamp — 영상은 해석하지 않고 CDR 바이트(4~12)만 고친다(빠름). 나머지 메시지는 수신 시각만.
사용:  go2_camera_bag_fix.py <카메라bag 폴더> <clock_probe.csv> <출력 폴더>
"""
import sys, os, json, struct, numpy as np, yaml, rosbag2_py

SRC, CSV, DST = sys.argv[1], sys.argv[2], sys.argv[3]
HEADER_FIRST = {'sensor_msgs/msg/Image', 'sensor_msgs/msg/CameraInfo', 'sensor_msgs/msg/Imu', 'sensor_msgs/msg/CompressedImage',
                'realsense2_camera_msgs/msg/Metadata', 'sensor_msgs/msg/PointCloud2'}

# ── 시각 모델 ────────────────────────────────────────────────────────────────
A = np.loadtxt(CSV, delimiter=',', skiprows=1, ndmin=2)
if len(A) == 0: sys.exit('시각 표본이 없다: ' + CSV)
tn, off, rtt = A[:, 0], A[:, 1], A[:, 2]
good = rtt <= np.percentile(rtt, 50) if len(A) >= 6 else np.ones(len(A), bool)
t0 = float(tn[good].mean())
if good.sum() >= 3 and np.ptp(tn[good]) > 5:
    b, a = np.polyfit(tn[good] - t0, off[good], 1)
else:
    a, b = float(np.median(off[good])), 0.0
res = off[good] - (a + b * (tn[good] - t0))
model = dict(t0=t0, a_s=float(a), b_ppm=float(b * 1e6), 표본=int(len(A)), 사용=int(good.sum()),
             잔차_std_ms=round(float(res.std()) * 1e3, 3), 잔차_최대_ms=round(float(np.abs(res).max()) * 1e3, 3),
             왕복_중앙_ms=round(float(np.median(rtt)) * 1e3, 3), 측정구간_s=round(float(np.ptp(tn)), 1))
print('시각 모델 (로봇PC − NUC):', json.dumps(model, ensure_ascii=False))


def to_nuc(t_robot):   # 로봇 PC 시각(s) → NUC 시각(s):  t_robot = t_nuc + offset(t_nuc) 를 두 번 반복으로 푼다
    t = t_robot - (a + b * (t_robot - t0))
    return t_robot - (a + b * (t - t0))


# ── bag 다시 쓰기 ─────────────────────────────────────────────────────────────
meta = yaml.safe_load(open(os.path.join(SRC, 'metadata.yaml')))['rosbag2_bagfile_information']
sid = meta.get('storage_identifier', 'sqlite3')
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=SRC, storage_id=sid), rosbag2_py.ConverterOptions('', ''))
topics = r.get_all_topics_and_types(); types = {t.name: t.type for t in topics}
w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=DST, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
for t in topics: w.create_topic(t)
n = 0; patched = 0; corr = []
while r.has_next():
    tp, raw, t = r.read_next(); n += 1
    t_new = int(round(to_nuc(t * 1e-9) * 1e9)); corr.append(t_new - t)
    if types.get(tp) in HEADER_FIRST and len(raw) >= 12 and raw[1] == 0x01:   # CDR little-endian
        sec, nsec = struct.unpack_from('<iI', raw, 4)
        s = to_nuc(sec + nsec * 1e-9); si = int(np.floor(s)); ni = int(round((s - si) * 1e9))
        if ni >= 1_000_000_000: si += 1; ni -= 1_000_000_000
        raw = bytearray(raw); struct.pack_into('<iI', raw, 4, si, ni); raw = bytes(raw); patched += 1
    w.write(tp, raw, t_new)
del w
c = np.array(corr) * 1e-9
out = dict(모델=model, 메시지=n, header_고침=patched, 보정량_s=[round(float(c.min()), 6), round(float(c.max()), 6)], 저장소=sid, 출력=DST)
json.dump(out, open(os.path.join(DST, 'time_fix.json'), 'w'), ensure_ascii=False, indent=1)
print(json.dumps(out, ensure_ascii=False))
