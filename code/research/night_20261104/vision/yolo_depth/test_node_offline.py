#!/usr/bin/env python3
"""cone_fusion_node.FusionCore 오프라인 시험 — DDS 없이 bag 바이트를 그대로 콜백에 넣는다.

- 수신 시각 = bag 기록 시각, 자세 = FAST-LIO(gosu1030v2) 를 tL 시각으로 on_odom 대신 직접 넣음
- 시계 오프셋은 노드처럼 (수신 − stamp) 10 s 최솟값으로 온라인 추정 → 녹화본 고정 보정(calib_depth_all.json)과 비교
- 검출은 yd_extract.py 의 YOLO 결과(dets.npz)를 같은 컬러 프레임에 재사용 (같은 프레임만 on_color)
- 단위: cone_column, make_cloud(+직렬화 왕복), ClockOffset, PoseBuffer, ConeMemory
실행: source /opt/ros/jazzy/setup.bash; ~/venv/go2/bin/python test_node_offline.py
쓰기: node_offline.json
"""
import os, sys, json, time
import numpy as np, rosbag2_py

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import yd_core as C
import cone_fusion_node as N
from scipy.spatial.transform import Rotation as Rot

BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
E = np.load(f'{HERE}/dets.npz'); FR, DET = E['frames'], E['dets']
T_bd, ST, DELTA = C.load_calib()
res = {}

# ── 단위 1: 점기둥
col = C.cone_column([1.0, 2.0], -0.30)
r = np.hypot(col[:, 0] - 1.0, col[:, 1] - 2.0)
res['column'] = dict(n=len(col), r_max=round(float(r.max()), 4), z_min=round(float(col[:, 2].min()), 3), z_max=round(float(col[:, 2].max()), 3))
assert r.max() <= 0.0701 and abs(col[:, 2].min() + 0.30) < 1e-9 and abs(col[:, 2].max() + 0.05) < 1e-9

# ── 단위 2: PointCloud2 만들기 + 직렬화 왕복 (DDS 안 씀)
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
m = N.make_cloud(col, 'camera_init', N.to_time_msg(1793286700.25))
m2 = deserialize_message(serialize_message(m), PointCloud2)
back = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(m2, field_names=('x', 'y', 'z'))], float)
res['cloud_roundtrip_max_err'] = float(np.abs(back - col).max())
res['cloud_stamp'] = [m2.header.stamp.sec, m2.header.stamp.nanosec]
assert res['cloud_roundtrip_max_err'] < 1e-6 and m2.header.stamp.nanosec == 250000000
empty = deserialize_message(serialize_message(N.make_cloud(np.zeros((0, 3)), 'camera_init', N.to_time_msg(1.0))), PointCloud2)
assert empty.width == 0

# ── 단위 3: ClockOffset (창 밖 최솟값이 빠지는지)
co = C.ClockOffset(10.0)
for k in range(300):
    co.add(100.0 + k * 0.1, 100.0 + k * 0.1 - (0.5 if k == 5 else 1.0 + 0.01 * (k % 7)))
res['clock_unit_offset'] = co.offset(); assert abs(co.offset() - 1.0) < 1e-9

# ── 단위 4: PoseBuffer 보간
pb = C.PoseBuffer()
pb.add(0.0, [0, 0, 0], Rot.from_euler('z', 0).as_quat()); pb.add(1.0, [1, 0, 0], Rot.from_euler('z', 90, degrees=True).as_quat())
Rm, pm = pb.at(0.5); yaw = np.degrees(np.arctan2(Rm[1, 0], Rm[0, 0]))
res['pose_interp'] = dict(p=pm.tolist(), yaw=round(float(yaw), 3)); assert abs(yaw - 45) < 1e-6 and abs(pm[0] - 0.5) < 1e-9
assert pb.at(1.2) is None and pb.at(1.05) is not None

# ── 단위 5: ConeMemory
cm = C.ConeMemory(0.3, 2, 30.0)
cm.add([0, 0], -0.3, 0.0, 0.5); assert len(cm.active(0.0)) == 0
cm.add([0.1, 0], -0.3, 0.2, 0.6); a = cm.active(0.2); assert len(a) == 1 and abs(a[0]['xy'][0] - 0.05) < 1e-9
cm.add([1.0, 0], -0.3, 0.4, 0.6); assert len(cm.c) == 2
assert len(cm.active(31.0)) == 0
res['units'] = 'ok'

# ── 재생: FusionCore 에 bag 바이트 주입
P = dict(N.DEFAULTS); core = N.FusionCore(P, model=None)
d = np.load('/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'); odom, lid = d['odom'], d['lid']
h2b = {round(float(h), 4): b for h, b in E['hdr2bag']}
pairs = np.array([(w, h2b[round(h, 4)]) for w, h in lid if round(h, 4) in h2b])
Kp = float(np.median(pairs[:, 0] - pairs[:, 1])); wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
dmed = float(np.median(h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]))
tL = h_o - dmed - Kp; o = np.argsort(tL); tL = tL[o]; GT = odom[o]
sel = {round(float(f[1]), 6): int(f[0]) for f in FR}           # 컬러 stamp → 프레임 번호
detby = {int(i): DET[DET[:, 0] == i][:, 1:] for i in np.unique(DET[:, 0])}
ref = {}
for rr in json.load(open(f'{HERE}/det3d.json')):
    if rr['ok']: ref.setdefault(rr['frame'], []).append(rr)

rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
rd.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw', '/camera/depth/image_rect_raw']))
ever = set(); gi = 0; diffs = []; offs = []; status = {}; ncone_t = []; steps = 0; t_step = []; wall0 = time.time()
while rd.has_next():
    tp, raw, t = rd.read_next(); tb = t * 1e-9
    while gi < len(tL) and tL[gi] <= tb:                         # 자세는 수신 시각까지 들어온 것만
        core.poses.add(tL[gi], GT[gi, 2:5], GT[gi, 5:9]); gi += 1
    if tp == '/camera/depth/image_rect_raw':
        core.on_depth(raw, tb)
        if core.clock.offset() is not None:
            s = N.hdr_stamp(raw)
            offs.append(core.clock.to_nuc(s) - C.stamp_to_nuc(s, ST))
        continue
    s = N.hdr_stamp(raw); fi = sel.get(round(s, 6))
    if fi is None: continue
    core.on_color(raw, tb)
    t0 = time.perf_counter(); out = core.step(tb, dets=detby.get(fi, np.zeros((0, 6)))); t_step.append(time.perf_counter() - t0)
    steps += 1; status[out['status']] = status.get(out['status'], 0) + 1
    ncone_t.append((tb, len(out['cones']))); ever.update(c['id'] for c in core.mem.active(out['stamp']) if out['stamp'])
    for r_ in out['dets']:
        if not r_['ok']: continue
        for q in ref.get(fi, []):
            if q['box'] == r_['box']:
                diffs.append(np.linalg.norm(np.array(r_['p_world']) - np.array(q['p_world'])))
# 깊이 프레임 고르기가 다를 수 있음(노드는 그 순간 버퍼 안의 최근접) → 차이는 거의 시각 오프셋 + 깊이 프레임 차
offs = np.array(offs); diffs = np.array(diffs); nc = np.array(ncone_t)
res['replay'] = dict(steps=steps, status=status, wall_s=round(time.time() - wall0, 1),
                     step_ms_p50_p90=[round(float(np.percentile(t_step, q)) * 1e3, 2) for q in (50, 90)],
                     online_minus_fixed_clock_ms_p1_p50_p99=[round(float(np.percentile(offs, q)) * 1e3, 2) for q in (1, 50, 99)],
                     world_diff_vs_offline_cm_p50_p90_max=[round(float(np.percentile(diffs, q)) * 100, 2) for q in (50, 90)] + [round(float(diffs.max()) * 100, 2)] if len(diffs) else None,
                     n_matched=int(len(diffs)),
                     phantom_cones_final=len(core.mem.active(nc[-1, 0])) if len(nc) else 0,
                     phantom_cones_max_at_once=int(nc[:, 1].max()) if len(nc) else 0,
                     frac_time_with_phantom=round(float((nc[:, 1] > 0).mean()), 3) if len(nc) else 0,
                     phantom_cones_total_ever=len(ever))
print(json.dumps(res, ensure_ascii=False, indent=1))
json.dump(res, open(f'{HERE}/node_offline.json', 'w'), ensure_ascii=False, indent=1)
