#!/usr/bin/env python3
"""Depth camera <-> LiDAR alignment from a recorded bag (no target, no laptop).

Where the robot stood still, the D435i depth cloud and the XT16 cloud see the same walls and floor
(their views overlap from -15 to -3 deg). Point-to-plane ICP of the LiDAR points onto the depth
surface, over many still spots at once, gives where the depth camera sits relative to the LiDAR.
The colour camera follows from the RealSense factory depth->colour extrinsic in the bag.

    source /opt/ros/jazzy/setup.bash
    python3 depth_lidar_align.py ~/bags/gosu_20261030_001126            # report only
    python3 depth_lidar_align.py BAG --write ../config/camera_extrinsics.yaml   # + save (backup kept)

Accuracy is checked, not assumed: the still spots are split in two halves solved independently,
and every spot's residual is listed. Do not write a result whose halves disagree.
"""
import argparse
import math
import shutil
import struct
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

DEPTH = '/camera/depth/image_rect_raw'
DEPTH_INFO = '/camera/depth/camera_info'
D2C = '/camera/extrinsics/depth_to_color'
LIDAR = '/lidar_points'
ODOM = '/utlidar/robot_odom'


# ---------------------------------------------------------------- small geometry helpers
def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    w = math.sqrt(max(0.0, 1 + np.trace(R))) / 2
    if w > 1e-6:
        return [(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w]
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1 + R[i, i] - R[j, j] - R[k, k])) * 2
    q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
    q[i], q[j], q[k] = s / 4, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
    return q


def T_of(R, t):
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = R, t
    return M


def expm(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    a = w / th
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(th) * k + (1 - math.cos(th)) * k @ k


def rot_deg(R):
    return math.degrees(math.acos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def rpy_deg(R):
    return np.degrees([math.atan2(R[2, 1], R[2, 2]), -math.asin(np.clip(R[2, 0], -1, 1)), math.atan2(R[1, 0], R[0, 0])])


# ---------------------------------------------------------------- depth surface
def depth_surface(img_mm, K, step=3, zmin=0.4, zmax=3.0):
    """Depth image -> points and unit normals (from the pixel grid) in the depth optical frame."""
    z = img_mm[::step, ::step].astype(float) / 1000.0
    h, w = z.shape
    v, u = np.mgrid[0:h, 0:w]
    u, v = u * step, v * step
    X = np.dstack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z])
    dx = X[1:-1, 2:] - X[1:-1, :-2]
    dy = X[2:, 1:-1] - X[:-2, 1:-1]
    n = np.cross(dx, dy)
    P = X[1:-1, 1:-1]
    zc = z[1:-1, 1:-1]
    ok = (zc > zmin) & (zc < zmax)
    # all four neighbours valid and close in depth (no normals across depth edges)
    for nb in (z[1:-1, 2:], z[1:-1, :-2], z[2:, 1:-1], z[:-2, 1:-1]):
        ok &= (nb > zmin) & (np.abs(nb - zc) < 0.03 + 0.02 * zc)
    nn = np.linalg.norm(n, axis=2)
    ok &= nn > 1e-9
    P, n = P[ok], n[ok] / nn[ok][:, None]
    n *= -np.sign(np.sum(n * P, axis=1))[:, None]         # normals towards the camera
    return P, n


# ---------------------------------------------------------------- joint ICP
def align(samples, depth_T_lidar0, iters=60, fix_t=True, verbose=False):
    """samples: list of (depth_points, depth_normals, kdtree, lidar_points, weight). Point-to-plane ICP of the
    LiDAR points onto the depth surfaces, one rigid transform for all samples, Huber weights.
    fix_t: keep the lever arm from the drawing (the bag runs showed camera pitch and height trade
    against each other: free translation drifted 15 cm up for 5 deg - physically impossible).
    Returns depth_T_lidar, per-sample (inliers, rms cm), covariance (rot rad[, trans m])."""
    nx = 3 if fix_t else 6
    c_l = -depth_T_lidar0[:3, :3].T @ depth_T_lidar0[:3, 3]
    R, t = depth_T_lidar0[:3, :3].copy(), depth_T_lidar0[:3, 3].copy()
    gates = np.r_[np.linspace(0.30, 0.06, 12), np.full(iters - 12, 0.05)]
    for it in range(iters):
        A, b, per = [], [], []
        for P, N, tree, L, pw in samples:
            Q = L @ R.T + t
            d, j = tree.query(Q, distance_upper_bound=gates[it])
            m = np.isfinite(d)
            Qm, n, p = Q[m], N[j[m]], P[j[m]]
            r = np.sum((Qm - p) * n, axis=1)
            keep = np.abs(r) < max(0.02, gates[it] / 2)
            Qm, n, r = Qm[keep], n[keep], r[keep]
            per.append((len(r), float(np.sqrt(np.mean(r ** 2))) * 100 if len(r) else float('nan')))
            if len(r) < 50:
                continue
            hub = np.minimum(1.0, 0.01 / np.maximum(np.abs(r), 1e-9))          # Huber, 1 cm
            wgt = np.sqrt(hub * min(1.0, 2000.0 / len(r)) * pw)                # places weigh alike
            # fix_t: rotate about the camera (camera position in the LiDAR frame stays put)
            A.append(wgt[:, None] * np.c_[np.cross(Qm if fix_t else Qm - t, n), n][:, :nx])
            b.append(-wgt * r)
        A, b = np.vstack(A), np.concatenate(b)
        dx = np.linalg.lstsq(A, b, rcond=None)[0]
        if fix_t:                                          # Q = R (L - c), c = camera in the LiDAR frame
            R = expm(dx[:3]) @ R
            t = -R @ c_l
        else:
            R = expm(dx[:3]) @ R
            t = t + dx[3:]
        if verbose:
            print(f'    it {it:2d} gate {gates[it]:.2f} m  pts {len(b)}  step {math.degrees(np.linalg.norm(dx[:3])):.3f} deg '
                  f'{np.linalg.norm(dx[3:]) * 100:.2f} cm')
        if it > 14 and np.linalg.norm(dx) < 1e-5:
            break
    res = b
    cov = np.linalg.pinv(A.T @ A) * (res @ res) / max(1, len(res) - 6)
    return T_of(R, t), per, cov


# ---------------------------------------------------------------- bag
def _reader(bag, topics):
    import rosbag2_py
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def still_times(bag, v_max=0.05, w_max=0.05, min_len=1.0, every=3.0, min_move=0.4):
    """Sample times (bag receive clock) where the leg odometry says the robot stands still."""
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    r = _reader(bag, [ODOM])
    T, still, pose = [], [], []
    while r.has_next():
        _, data, t = r.read_next()
        m = deserialize_message(data, Odometry)
        tw, p, q = m.twist.twist, m.pose.pose.position, m.pose.pose.orientation
        T.append(t * 1e-9)
        still.append(math.hypot(tw.linear.x, tw.linear.y) < v_max and abs(tw.angular.z) < w_max)
        pose.append((p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
    T, still, pose = np.array(T), np.array(still), np.array(pose)
    out, s = [], None
    for i in range(len(T)):
        if still[i] and s is None:
            s = i
        if (not still[i] or i == len(T) - 1) and s is not None:
            a, b = T[s] + 0.3, T[i - 1] - 0.3                   # keep away from the start/stop
            if b - a >= min_len - 0.6:
                out += list(np.arange(a + (b - a) % every / 2, b + 1e-6, every)) or [(a + b) / 2]
            s = None
    # place id per sample: samples of one place share its weight and stay in the same half
    places, ids = [], []
    for ts in out:
        x, y, yaw = pose[min(len(T) - 1, int(np.searchsorted(T, ts)))]
        for i, (kx, ky, kyaw) in enumerate(places):
            if math.hypot(x - kx, y - ky) < min_move and abs((yaw - kyaw + math.pi) % (2 * math.pi) - math.pi) < math.radians(20):
                ids.append(i)
                break
        else:
            places.append((x, y, yaw))
            ids.append(len(places) - 1)
    return np.array(out), np.array(ids), T[0]


def load_samples(bag, times, n_scans=5):
    """Depth frame + n_scans LiDAR scans closest to each sample time (receive clock; the robot stands
    still, so the ~10 ms receive jitter does not matter)."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from sensor_msgs_py import point_cloud2
    Ext = get_message('realsense2_camera_msgs/msg/Extrinsics')
    r = _reader(bag, [DEPTH, DEPTH_INFO, D2C, LIDAR])
    K = c_T_d = None
    best = [None] * len(times)                                 # (|dt|, raw depth)
    scans = [[] for _ in times]                                 # (|dt|, points)
    while r.has_next():
        tp, data, t = r.read_next()
        t *= 1e-9
        i = int(np.searchsorted(times, t))
        near = [k for k in (i - 1, i) if 0 <= k < len(times) and abs(times[k] - t) < 0.15]
        if tp == DEPTH_INFO and K is None:
            K = np.array(deserialize_message(data, CameraInfo).k).reshape(3, 3)
        elif tp == D2C and c_T_d is None:
            m = deserialize_message(data, Ext)                 # librealsense: column-major rotation
            c_T_d = T_of(np.array(m.rotation).reshape(3, 3).T, np.array(m.translation))
        elif tp == DEPTH:
            for k in near:
                if best[k] is None or abs(times[k] - t) < best[k][0]:
                    best[k] = (abs(times[k] - t), data)
        elif tp == LIDAR:
            for k in near:
                scans[k].append((abs(times[k] - t), data))
                scans[k] = sorted(scans[k], key=lambda x: x[0])[:n_scans]
    out = []
    for k in range(len(times)):
        if best[k] is None or not scans[k]:
            continue
        img = deserialize_message(best[k][1], Image)
        dimg = np.frombuffer(img.data, np.uint16).reshape(img.height, img.width)
        L = np.vstack([point_cloud2.read_points_numpy(deserialize_message(d, PointCloud2), field_names=('x', 'y', 'z'))
                       for _, d in scans[k]]).astype(float)
        L = L[np.isfinite(L).all(1)]
        rng = np.linalg.norm(L, axis=1)
        out.append((times[k], dimg, L[(rng > 0.5) & (rng < 6.0)]))
    return K, c_T_d, out


# ---------------------------------------------------------------- config
def read_extrinsics(path):
    import yaml
    p = yaml.safe_load(open(path))['cone_tracker_node']['ros__parameters']
    f = lambda k: T_of(quat_to_R(p[k]['q_xyzw']), np.array(p[k]['xyz'], float))
    return f('base_to_color_optical'), f('base_to_lidar')


def write_extrinsics(path, base_T_color, note):
    import re
    s = open(path).read()
    xyz = '[' + ', '.join(f'{v:.4f}' for v in base_T_color[:3, 3]) + ']'
    q = '[' + ', '.join(f'{v:.5f}' for v in R_to_quat(base_T_color[:3, :3])) + ']'
    s = re.sub(r'(base_to_color_optical:\s*\n\s*xyz: )\[[^\]]*\]', r'\g<1>' + xyz, s)
    s = re.sub(r'(base_to_color_optical:\s*\n\s*xyz: \[[^\]]*\]\s*\n\s*q_xyzw: )\[[^\]]*\]', r'\g<1>' + q, s)
    s = re.sub(r'calibrated: false[^\n]*', f'calibrated: true         # {note}', s)
    bak = path + time.strftime('.bak_%Y%m%d_%H%M%S')
    shutil.copy(path, bak)
    open(path, 'w').write(s)
    return bak


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--config', default=None, help='camera_extrinsics.yaml with the prior (default: package config)')
    ap.add_argument('--write', default='', help='write the result into this camera_extrinsics.yaml (backup kept)')
    ap.add_argument('--max-samples', type=int, default=60)
    ap.add_argument('--free-translation', action='store_true',
                    help='also solve the lever arm (poorly constrained by room scenes; diagnostic)')
    ap.add_argument('-v', action='store_true')
    a = ap.parse_args()
    cfg = a.config or str(__import__('pathlib').Path(__file__).resolve().parents[1] / 'config' / 'camera_extrinsics.yaml')
    base_T_color0, base_T_lidar = read_extrinsics(cfg)

    t0 = time.time()
    times, place, tstart = still_times(a.bag)
    if len(times) > a.max_samples:
        sel = np.linspace(0, len(times) - 1, a.max_samples).round().astype(int)
        times, place = times[sel], place[sel]
    print(f'{len(times)} still sample times; reading the bag ...', flush=True)
    K, c_T_d, raw = load_samples(a.bag, times)
    if K is None or c_T_d is None:
        sys.exit('bag has no depth camera_info or depth_to_color extrinsics')
    base_T_depth0 = base_T_color0 @ c_T_d
    d_T_l0 = np.linalg.inv(base_T_depth0) @ base_T_lidar
    samples, tags = [], []
    pid = []
    for ts, dimg, L in raw:
        P, N = depth_surface(dimg, K, zmax=4.0)
        if len(P) < 2000:
            continue
        k = int(np.argmin(np.abs(times - ts)))
        samples.append((P, N, cKDTree(P), L, 1.0 / np.sum(place == place[k])))
        tags.append(ts - tstart)
        pid.append(place[k])
    pid = np.array(pid)
    print(f'{len(set(pid))} distinct places')
    print(f'{len(samples)} usable spots ({time.time() - t0:.0f} s)')
    if len(samples) < 6:
        sys.exit('too few still spots with depth + LiDAR')

    if a.free_translation:
        print('free translation (diagnostic only):')
    d_T_l, per, cov = align(samples, d_T_l0, fix_t=not a.free_translation, verbose=a.v)
    ups = sorted(set(pid))                                    # halves split by PLACE, not by sample
    halves = [align([s for s, p in zip(samples, pid) if ups.index(p) % 2 == h], d_T_l0,
                    fix_t=not a.free_translation)[0] for h in (0, 1)]
    base_T_depth = base_T_lidar @ np.linalg.inv(d_T_l)
    base_T_color = base_T_depth @ np.linalg.inv(c_T_d)

    dR = base_T_color0[:3, :3].T @ base_T_color[:3, :3]
    print('\nper spot (time s: LiDAR points on the depth surface, rms cm)')
    print('  ' + '  '.join(f'{tg:5.0f}:{n:4d}/{e:.1f}' for tg, (n, e) in zip(tags, per)))
    split_rot = rot_deg(halves[0][:3, :3].T @ halves[1][:3, :3])
    split_t = np.linalg.norm(halves[0][:3, 3] - halves[1][:3, 3]) * 100
    sig = np.sqrt(np.diag(cov))
    print(f'\nresult vs prior (colour camera in base): rotation {rot_deg(dR):.2f} deg '
          f'(roll/pitch/yaw in camera {np.round(rpy_deg(dR), 2)}), '
          f'translation {np.round((base_T_color[:3, 3] - base_T_color0[:3, 3]) * 100, 1)} cm')
    print(f'two halves of the spots solved apart: differ {split_rot:.2f} deg, {split_t:.1f} cm   '
          f'(formal sigma {math.degrees(sig[:3].max()):.3f} deg)')
    print(f'base_to_color_optical:\n  xyz: [{", ".join(f"{v:.4f}" for v in base_T_color[:3, 3])}]\n'
          f'  q_xyzw: [{", ".join(f"{v:.5f}" for v in R_to_quat(base_T_color[:3, :3]))}]')
    good = split_rot < 0.5 and split_t < 2.0
    print('CONSISTENT' if good else 'NOT CONSISTENT - do not use', '(halves within 0.5 deg / 2 cm)')
    if a.write:
        if not good:
            sys.exit('not writing an inconsistent result')
        bak = write_extrinsics(a.write, base_T_color,
                               f'depth_lidar_align {time.strftime("%Y-%m-%d")}, halves {split_rot:.2f} deg/{split_t:.1f} cm')
        print(f'written {a.write} (backup {bak})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
