#!/usr/bin/env python3
"""Camera mount angle from the floor: robot standing still on a flat floor -> camera_extrinsics.yaml.

After the D435i is re-mounted or re-angled: stand the robot on a flat floor with nothing but floor in
the lower part of the camera view, then

    ros2 run mission_vision camera_floor_calib.py --write <path>/camera_extrinsics.yaml   # live, 10 s
    python3 camera_floor_calib.py --bag ~/bags/<bag> --start 5                          # from a bag

The depth image gives the floor plane in the camera: its normal is "up" seen by the camera, its
distance the camera height. The four feet (leg kinematics from /lowstate joint angles, the model of
go2_bridge/go2_kinematics.hpp that imu_bridge uses for base_stab) give the same floor in base_link:
"up" and the floor height, measured under the robot - no gravity, no IMU sign, no LiDAR view needed
(the XT16's lowest ring reaches the floor only 1.7 m ahead; at the 10-30 bag start a wall 2 m ahead
hid it). Fallback: IMU roll/pitch with floor_z from the params. Pitch and roll of the mount are set so
the two "up"s agree; yaw and the x/y position (not visible from the floor) are kept exactly;
z = camera height above the floor + floor height in base_link.

Refuses to write when the robot moved, the floor was not found in enough frames, or the frames
disagree (spread > 0.5 deg). The old file is kept as a .bak_<time> copy.
"""
import argparse
import math
import re
import shutil
import sys
import time

import numpy as np

DEPTH = '/camera/depth/image_rect_raw'
DEPTH_INFO = '/camera/depth/camera_info'
D2C = '/camera/extrinsics/depth_to_color'
LOWSTATE = '/lowstate'
LIDAR = '/lidar_points'
ODOM = '/utlidar/robot_odom'


# ---------------------------------------------------------------- geometry
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


def align(a, b):
    """Smallest rotation taking unit vector a onto unit vector b."""
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1 + c)


def up_in_base(roll, pitch):
    """Gravity 'up' in base_link from the IMU roll/pitch (ZYX)."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return (Ry @ Rx).T @ np.array([0.0, 0.0, 1.0])


def mount_angles(base_R_cam):
    """Pitch (optical axis below the base horizontal, deg), roll (image x-axis tilt, deg), yaw (deg)."""
    z = base_R_cam[:, 2]                      # optical axis in base
    x = base_R_cam[:, 0]                      # image right in base
    pitch = math.degrees(math.atan2(-z[2], math.hypot(z[0], z[1])))
    yaw = math.degrees(math.atan2(z[1], z[0]))
    roll = math.degrees(math.asin(np.clip(x[2], -1, 1)))
    return pitch, roll, yaw


def floor_in_depth(img_mm, K, up_pred, max_dev_deg=35.0, step=4, iters=300):
    """Floor plane in the depth camera: (up unit vector, camera height m, inlier fraction) or None.
    Only planes within max_dev_deg of the predicted up count (walls are ~90 deg off)."""
    z = img_mm[::step, ::step].astype(float) / 1000.0
    v, u = np.mgrid[0:z.shape[0], 0:z.shape[1]] * step
    P = np.dstack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z]).reshape(-1, 3)
    P = P[(P[:, 2] > 0.3) & (P[:, 2] < 3.0)]
    if len(P) < 500:
        return None
    rng = np.random.default_rng(0)
    best, cmin = None, math.cos(math.radians(max_dev_deg))
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        if abs(n @ up_pred) < cmin:
            continue
        inl = np.abs((P - a) @ n) < 0.012
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 300:
        return None
    Q = P[best]
    c = Q.mean(0)
    n = np.linalg.svd(Q - c)[2][-1]
    if n @ up_pred < 0:
        n = -n                                # up: from the floor towards the camera
    return n, float(-(n @ c)), float(best.mean())


# ---------------------------------------------------------------- solve
# Go2 leg model (go2_bridge/go2_kinematics.hpp): hips [FR, FL, RR, RL], thigh offset, link lengths
HIP = np.array([[0.1934, -0.0465, 0.0], [0.1934, 0.0465, 0.0], [-0.1934, -0.0465, 0.0], [-0.1934, 0.0465, 0.0]])
THIGH_Y = np.array([-0.0955, 0.0955, -0.0955, 0.0955])
L1 = L2 = 0.213
FOOT_RADIUS = 0.022


def feet_floor(q, force, thresh=15):
    """Floor plane under the feet in base_link: (up, floor z along up) or None."""
    feet = []
    for i in range(4):
        q1, q2, q3 = q[3 * i:3 * i + 3]
        reach = L1 * math.cos(q2) + L2 * math.cos(q2 + q3)
        feet.append(HIP[i] + [-L1 * math.sin(q2) - L2 * math.sin(q2 + q3),
                              THIGH_Y[i] * math.cos(q1) + reach * math.sin(q1),
                              THIGH_Y[i] * math.sin(q1) - reach * math.cos(q1)])
    F = np.array(feet)
    st = [i for i in range(4) if force[i] > thresh]
    if len(st) < 3:
        st = [0, 1, 2, 3] if len(st) < 2 else st
    if len(st) < 3:
        return None
    c = F[st].mean(0)
    n = np.linalg.svd(F[st] - c)[2][-1]
    n = n if n[2] > 0 else -n
    return n, float(n @ c) - FOOT_RADIUS


def rot_about(axis, ang):
    k = _skew(axis / np.linalg.norm(axis))
    return np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k


def _skew(a):
    return np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])


def solve(samples, up_b, floor_z, base_T_color0, c_T_d):
    """samples: list of floor (up_depth, height, frac). up_b / floor_z: floor in base_link.
    Returns new base_T_color and a report dict."""
    ups = np.array([s[0] for s in samples])
    up_d = np.median(ups, axis=0)
    up_d /= np.linalg.norm(up_d)
    spread = max(math.degrees(math.acos(np.clip(u @ up_d, -1, 1))) for u in ups)
    h = float(np.median([s[1] for s in samples]))
    up_c = c_T_d[:3, :3] @ up_d                             # up seen by the colour camera
    R0 = base_T_color0[:3, :3]
    Ra = align(R0 @ up_c, up_b) @ R0
    # every rotation about up_b keeps "up" right; pick the one with the old optical-axis yaw
    yaw0 = mount_angles(R0)[2]
    phis = np.radians(np.linspace(-15, 15, 3001))
    errs = [abs((mount_angles(rot_about(up_b, f) @ Ra)[2] - yaw0 + 180) % 360 - 180) for f in phis]
    R1 = rot_about(up_b, phis[int(np.argmin(errs))]) @ Ra
    T1 = base_T_color0.copy()
    T1[:3, :3] = R1
    # camera height above the floor -> z in base_link (floor at floor_z), measured along gravity
    cam_h = h + float(up_d @ c_T_d[:3, :3].T @ (-c_T_d[:3, 3]))   # depth origin -> colour origin
    T1[2, 3] = cam_h + floor_z
    a0, a1 = mount_angles(R0), mount_angles(R1)
    return T1, {'frames': len(samples), 'spread_deg': spread, 'height_m': cam_h,
                'old': a0, 'new': a1, 'old_z': float(base_T_color0[2, 3]), 'new_z': float(T1[2, 3]),
                'floor_frac': float(np.median([s[2] for s in samples]))}


# ---------------------------------------------------------------- config
def read_config(path, key='base_to_color_optical'):
    import yaml
    p = yaml.safe_load(open(path))['cone_tracker_node']['ros__parameters'][key]
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = quat_to_R(p['q_xyzw']), p['xyz']
    return T


def read_floor_z(path):
    import yaml
    try:
        return float(yaml.safe_load(open(path))['cone_tracker_node']['ros__parameters']['fusion']['floor_z'])
    except Exception:
        return -0.30


def write_config(path, T, note):
    s = open(path).read()
    xyz = '[' + ', '.join(f'{v:.4f}' for v in T[:3, 3]) + ']'
    q = '[' + ', '.join(f'{v:.5f}' for v in R_to_quat(T[:3, :3])) + ']'
    s2 = re.sub(r'(base_to_color_optical:\s*\n\s*xyz: )\[[^\]]*\]', r'\g<1>' + xyz, s)
    s2 = re.sub(r'(base_to_color_optical:\s*\n\s*xyz: \[[^\]]*\]\s*\n\s*q_xyzw: )\[[^\]]*\]', r'\g<1>' + q, s2)
    if s2 == s:
        raise RuntimeError(f'base_to_color_optical not found in {path}')
    if 'floor_measured:' in s2:
        s2 = re.sub(r'floor_measured: [^\n]*', f'floor_measured: "{note}"', s2)
    else:
        s2 = re.sub(r'(calibration_info:\s*\n)', r'\g<1>' + f'      floor_measured: "{note}"\n', s2)
    bak = path + time.strftime('.bak_%Y%m%d_%H%M%S')
    shutil.copy(path, bak)
    open(path, 'w').write(s2)
    return bak


# ---------------------------------------------------------------- data sources
def from_bag(bag, start, duration, base_T_color0, c_T_d_default):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in (DEPTH, DEPTH_INFO, D2C, LOWSTATE, ODOM, LIDAR) if t in types]))
    K, c_T_d, imgs, imu, speed, t0, clouds = None, c_T_d_default, [], [], [], None, []
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    LowState = get_message('unitree_go/msg/LowState') if LOWSTATE in types else None
    Ext = get_message('realsense2_camera_msgs/msg/Extrinsics') if D2C in types else None
    while r.has_next():
        tp, data, t = r.read_next()
        t *= 1e-9
        t0 = t0 if t0 is not None else t
        if tp == DEPTH_INFO and K is None:
            K = np.array(deserialize_message(data, CameraInfo).k).reshape(3, 3)
        elif tp == D2C:
            m = deserialize_message(data, Ext)
            c_T_d = np.eye(4)
            c_T_d[:3, :3], c_T_d[:3, 3] = np.array(m.rotation).reshape(3, 3).T, m.translation
        if t - t0 < start:
            continue
        if t - t0 > start + duration:
            break
        if tp == DEPTH and len(imgs) < 60 and int((t - t0) * 6) != int((t - t0 - 1 / 30) * 6):   # ~6 Hz
            m = deserialize_message(data, Image)
            imgs.append(np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy())
        elif tp == LOWSTATE and LowState and len(imu) < 2000:
            m = deserialize_message(data, LowState)
            imu.append((m.imu_state.rpy[0], m.imu_state.rpy[1],
                        [m.motor_state[i].q for i in range(12)], list(m.foot_force)))
        elif tp == ODOM:
            tw = deserialize_message(data, Odometry).twist.twist
            speed.append(max(math.hypot(tw.linear.x, tw.linear.y), abs(tw.angular.z)))
        elif tp == LIDAR and len(clouds) < 20 and len(clouds) < (t - t0 - start) * 2:          # ~2 Hz
            clouds.append(point_cloud2.read_points_numpy(deserialize_message(data, PointCloud2),
                                                         field_names=('x', 'y', 'z')).astype(float))
    return K, c_T_d, imgs, imu, speed, clouds


def from_live(duration, c_T_d_default):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image
    rclpy.init()
    node = Node('camera_floor_calib')
    st = {'K': None, 'c_T_d': c_T_d_default, 'imgs': [], 'imu': [], 'speed': [], 'last': 0.0,
          'clouds': [], 'last_c': 0.0}
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    def on_cloud(m):
        now = time.time()
        if now - st['last_c'] >= 0.5 and len(st['clouds']) < 20:
            st['last_c'] = now
            st['clouds'].append(point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z')).astype(float))

    def on_depth(m):
        now = time.time()
        if now - st['last'] >= 1 / 6 and len(st['imgs']) < 60:
            st['last'] = now
            st['imgs'].append(np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy())

    def on_info(m):
        st['K'] = np.array(m.k).reshape(3, 3)

    def on_imu(m):
        if len(st['imu']) < 2000:
            st['imu'].append((m.imu_state.rpy[0], m.imu_state.rpy[1],
                              [m.motor_state[i].q for i in range(12)], list(m.foot_force)))

    def on_odom(m):
        tw = m.twist.twist
        st['speed'].append(max(math.hypot(tw.linear.x, tw.linear.y), abs(tw.angular.z)))

    def on_ext(m):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = np.array(m.rotation).reshape(3, 3).T, m.translation
        st['c_T_d'] = T

    node.create_subscription(Image, DEPTH, on_depth, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, DEPTH_INFO, on_info, qos_profile_sensor_data)
    node.create_subscription(get_message('unitree_go/msg/LowState'), LOWSTATE, on_imu, qos_profile_sensor_data)
    node.create_subscription(Odometry, ODOM, on_odom, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, LIDAR, on_cloud, qos_profile_sensor_data)
    try:
        from rclpy.qos import QoSProfile, DurabilityPolicy
        node.create_subscription(get_message('realsense2_camera_msgs/msg/Extrinsics'), D2C, on_ext,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    except Exception:
        pass
    print(f'collecting {duration:.0f} s - keep the robot still on a flat floor ...', flush=True)
    t_end = time.time() + duration
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return st['K'], st['c_T_d'], st['imgs'], st['imu'], st['speed'], st['clouds']


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', default='', help='read from a bag instead of live topics')
    ap.add_argument('--start', type=float, default=0.0, help='bag: seconds from the start')
    ap.add_argument('--duration', type=float, default=10.0)
    ap.add_argument('--config', default='', help='camera_extrinsics.yaml with the current mount (default: package config)')
    ap.add_argument('--params', default='', help='cone_tracker_params.yaml for floor_z (default: package config)')
    ap.add_argument('--write', default='', help='camera_extrinsics.yaml to update (backup kept)')
    a = ap.parse_args()
    cfgdir = __import__('pathlib').Path(__file__).resolve().parents[1] / 'config'
    if not cfgdir.exists():                       # installed: lib/mission_vision -> share/mission_vision
        from ament_index_python.packages import get_package_share_directory
        cfgdir = __import__('pathlib').Path(get_package_share_directory('mission_vision')) / 'config'
    cfg = a.config or a.write or str(cfgdir / 'camera_extrinsics.yaml')
    base_T_color0 = read_config(cfg)
    floor_z = read_floor_z(a.params or str(cfgdir / 'cone_tracker_params.yaml'))
    c_T_d0 = np.eye(4)
    c_T_d0[:3, 3] = [-0.0147, -0.0009, -0.0008]                 # D435i, used if the topic is missing

    if a.bag:
        K, c_T_d, imgs, imu, speed, clouds = from_bag(a.bag, a.start, a.duration, base_T_color0, c_T_d0)
    else:
        K, c_T_d, imgs, imu, speed, clouds = from_live(a.duration, c_T_d0)
    if K is None or not imgs:
        sys.exit(f'no depth images / camera_info ({DEPTH}) - is the camera running?')
    problems = []
    if speed and max(speed) > 0.05:
        problems.append(f'robot moved (max {max(speed):.2f} m/s or rad/s)')
    if not speed:
        print(f'note: no {ODOM}, cannot check that the robot stood still')
    # floor in base_link: the feet (leg kinematics), else IMU, else level
    ff = [f for f in (feet_floor(q, fo) for _, _, q, fo in imu if any(abs(v) > 1e-5 for v in q)) if f is not None]
    rp = np.median(np.array([(r_, p_) for r_, p_, _, _ in imu]), axis=0) if imu else None
    up_imu = up_in_base(rp[0], rp[1]) if rp is not None else None
    if len(ff) >= 20:
        up_b = np.median([f[0] for f in ff], axis=0)
        up_b /= np.linalg.norm(up_b)
        floor_z_used, ref = float(np.median([f[1] for f in ff])), f'feet / leg kinematics ({len(ff)} samples)'
    elif up_imu is not None:
        up_b, floor_z_used, ref = up_imu, floor_z, f'IMU (no LiDAR floor), floor_z {floor_z:+.3f} from params'
    else:
        up_b, floor_z_used, ref = np.array([0, 0, 1.0]), floor_z, 'level body assumed (no LiDAR, no IMU)'
    tilt = lambda u: (math.degrees(math.atan2(u[0], u[2])), math.degrees(math.atan2(-u[1], u[2])))
    print(f'floor reference: {ref}; body nose-up {tilt(up_b)[0]:+.2f} deg, right-side-up {tilt(up_b)[1]:+.2f} deg'
          + (f' | IMU says {tilt(up_imu)[0]:+.2f} / {tilt(up_imu)[1]:+.2f}' if up_imu is not None else ''))
    if len(ff) >= 20 and abs(floor_z_used - floor_z) > 0.02:
        print(f'note: floor under the feet at z {floor_z_used:+.3f} in base_link, cone_tracker fusion.floor_z is '
              f'{floor_z:+.3f} - consider updating floor_z (and refine.z_min/z_max with it)')

    up_pred = c_T_d[:3, :3].T @ base_T_color0[:3, :3].T @ np.array([0, 0, 1.0])
    samples = [f for f in (floor_in_depth(im, K, up_pred) for im in imgs) if f is not None and f[2] > 0.25]
    if len(samples) < max(5, len(imgs) // 2):
        sys.exit(f'floor found in only {len(samples)}/{len(imgs)} frames - clear the floor in front of the robot')
    T1, rep = solve(samples, up_b, floor_z_used, base_T_color0, c_T_d)
    if rep['spread_deg'] > 0.5:
        problems.append(f"frames disagree by {rep['spread_deg']:.2f} deg")

    p0, r0, y0 = rep['old']
    p1, r1, y1 = rep['new']
    print(f"{rep['frames']} depth frames, floor {100 * rep['floor_frac']:.0f} % of the view, spread {rep['spread_deg']:.2f} deg")
    print(f'             pitch down   roll     yaw    z in base   (camera height above floor {rep["height_m"]:.3f} m)')
    print(f'  config     {p0:7.2f}  {r0:+6.2f}  {y0:+6.2f}   {rep["old_z"]:.3f}')
    print(f'  measured   {p1:7.2f}  {r1:+6.2f}  {y1:+6.2f}   {rep["new_z"]:.3f}   (yaw, x, y kept)')
    print(f'base_to_color_optical:\n  xyz: [{", ".join(f"{v:.4f}" for v in T1[:3, 3])}]\n'
          f'  q_xyzw: [{", ".join(f"{v:.5f}" for v in R_to_quat(T1[:3, :3]))}]')
    if problems:
        print('NOT USABLE: ' + '; '.join(problems))
    if a.write:
        if problems:
            sys.exit('not writing')
        bak = write_config(a.write, T1, f'{time.strftime("%Y-%m-%d %H:%M")} pitch {p1:.2f} roll {r1:+.2f} '
                                         f'height {rep["height_m"]:.3f} m ({rep["frames"]} frames)')
        print(f'written {a.write} (backup {bak}) - restart cone_tracker_node (colcon build if not --symlink-install)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
