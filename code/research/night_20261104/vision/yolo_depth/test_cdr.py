"""cdr_parse 단위시험: 녹화본 첫 메시지들에서 rclpy 역직렬화 결과와 바이트 해석 결과를 비교 + 시간."""
import time, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from cdr_parse import parse_image, parse_odom, hdr_stamp
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri='/home/ilovehan9283/bags/gosu_20261030_001126', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
tt = {t.name: t.type for t in r.get_all_topics_and_types()}
odt = [k for k in tt if tt[k] == 'nav_msgs/msg/Odometry']; print('Odometry 토픽', odt)
want = ['/camera/color/image_raw', '/camera/depth/image_rect_raw'] + odt
r.set_filter(rosbag2_py.StorageFilter(topics=want))
cnt = {k: 0 for k in want}; tp_de = {k: [] for k in want}; tp_raw = {k: [] for k in want}; bad = 0
while r.has_next() and min(cnt.values()) < 20:
    tp, raw, t = r.read_next()
    if cnt[tp] >= 20: continue
    cnt[tp] += 1
    t0 = time.perf_counter(); m = deserialize_message(raw, get_message(tt[tp])); tp_de[tp].append(time.perf_counter() - t0)
    t0 = time.perf_counter()
    if 'Image' in tt[tp]:
        a, me = parse_image(raw, meta=True); tp_raw[tp].append(time.perf_counter() - t0)
        ref = np.frombuffer(m.data, np.uint8).reshape(a.shape[0], -1)
        ok = (me['height'] == m.height and me['width'] == m.width and me['encoding'] == m.encoding and me['frame_id'] == m.header.frame_id
              and np.array_equal(a.view(np.uint8).reshape(a.shape[0], -1), ref)
              and abs(me['stamp'] - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)) < 1e-9)
    else:
        s, fid, ch, p, q = parse_odom(raw); tp_raw[tp].append(time.perf_counter() - t0)
        P = m.pose.pose; ok = (fid == m.header.frame_id and ch == m.child_frame_id and np.allclose(p, [P.position.x, P.position.y, P.position.z])
                                and np.allclose(q, [P.orientation.x, P.orientation.y, P.orientation.z, P.orientation.w]))
    bad += (not ok)
for k in want: print(f'{k:32s} n={cnt[k]} rclpy 역직렬화 {np.median(tp_de[k]) * 1e3:.2f} ms  바이트해석 {np.median(tp_raw[k]) * 1e3:.3f} ms')
print('불일치', bad)
