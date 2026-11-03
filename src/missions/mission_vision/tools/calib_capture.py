#!/usr/bin/env python3
"""Camera <-> LiDAR extrinsic calibration with a chessboard shown on a laptop screen.

Robot side (where /camera/... and /velodyne_points are visible). The laptop runs calib_target.py.

    python3 calib_capture.py --square-mm 31.5 --out camera_extrinsics.yaml

For each capture the camera gives the screen plane from the chessboard (solvePnP) and the LiDAR
gives the same plane from the points on the laptop lid. Rotation comes from the plane normals
(needs >= 3 clearly different tilts), translation from the plane distances. The result is
written as base_link -> colour optical frame (via the known base_link -> LiDAR extrinsic).

Keys (in the image window):
    c  capture (board must be detected and the LiDAR plane found)
    u  undo last capture
    s  solve and show residuals
    v  toggle LiDAR overlay with the current extrinsic (check: points should land on objects)
    w  write --out (after s)
    q  quit
Tips: 1.0-1.5 m away; tilt the laptop clearly left, right, up and down between captures (5-8 total);
keep the lid away from walls so the LiDAR plane is only the screen; matte or inverted board if glare.

Fallback without LiDAR: --board-pose "x y z roll pitch yaw" (board origin = first inner corner,
measured in base_link, degrees) solves the camera pose from one capture.
"""
import argparse
import datetime
import math
import threading

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-6:
        return [(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w]
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1 + R[i, i] - R[j, j] - R[k, k])) * 2
    q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    return q


def T(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def fit_plane(P, thr=0.02, iters=300):
    """RANSAC plane, normal oriented away from the sensor origin. Returns (n, d, inliers)."""
    if len(P) < 30:
        return None
    rng = np.random.default_rng(0)
    best = None
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((P - a) @ n) < thr
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 30:
        return None
    Q = P[best]
    c = Q.mean(0)
    n = np.linalg.svd(Q - c)[2][-1]
    d = n @ c
    if d < 0:
        n, d = -n, -d
    return n, d, Q


class Capture(Node):
    def __init__(self, a):
        super().__init__('calib_capture')
        self.a = a
        self.lock = threading.Lock()
        self.img = None
        self.K = None
        self.D = None
        self.cloud = None
        self.create_subscription(Image, a.image_topic, self.on_img, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, a.info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, a.cloud_topic, self.on_cloud, qos_profile_sensor_data)

    def on_img(self, m):
        ch = 3 if m.encoding in ('rgb8', 'bgr8') else (4 if m.encoding in ('rgba8', 'bgra8') else 1)
        arr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * ch].reshape(m.height, m.width, ch)
        if m.encoding.startswith('rgb'):
            arr = cv2.cvtColor(arr[:, :, :3], cv2.COLOR_RGB2BGR)
        elif ch >= 3:
            arr = arr[:, :, :3].copy()
        else:
            arr = cv2.cvtColor(arr[:, :, 0], cv2.COLOR_GRAY2BGR)
        with self.lock:
            self.img = arr

    def on_info(self, m):
        with self.lock:
            self.K = np.array(m.k, float).reshape(3, 3)
            self.D = np.array(m.d, float) if len(m.d) else np.zeros(5)

    def on_cloud(self, m):
        pts = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
        with self.lock:
            self.cloud = np.asarray(pts, float)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--square-mm', type=float, required=True, help='measured square size on the laptop screen')
    ap.add_argument('--cols', type=int, default=8, help='squares across (calib_target.py --cols)')
    ap.add_argument('--rows', type=int, default=6, help='squares down (calib_target.py --rows)')
    ap.add_argument('--image-topic', default='/camera/color/image_raw')
    ap.add_argument('--info-topic', default='/camera/color/camera_info')
    ap.add_argument('--cloud-topic', default='/velodyne_points')
    ap.add_argument('--extrinsics', default='', help='existing camera_extrinsics.yaml (initial guess)')
    ap.add_argument('--out', default='camera_extrinsics.yaml')
    ap.add_argument('--board-pose', default='', help='"x y z roll pitch yaw" of the board in base_link (no LiDAR mode)')
    a = ap.parse_args()

    # Initial guess (only used to crop the LiDAR around the screen and for the overlay)
    base_T_lidar = T(quat_to_R([0.0024, 0.0063, 0.7071, 0.7070]), [0.25, 0.0, 0.1588])
    base_T_color = T(quat_to_R([-0.6187, 0.6187, -0.3423, 0.3423]), [0.30, 0.0, 0.117])
    color_to_depth = {'xyz': [-0.0148, 0.0, 0.0], 'q_xyzw': [0.0, 0.0, 0.0, 1.0]}
    if a.extrinsics:
        p = yaml.safe_load(open(a.extrinsics))['cone_tracker_node']['ros__parameters']
        base_T_lidar = T(quat_to_R(p['base_to_lidar']['q_xyzw']), p['base_to_lidar']['xyz'])
        base_T_color = T(quat_to_R(p['base_to_color_optical']['q_xyzw']), p['base_to_color_optical']['xyz'])
        color_to_depth = p.get('color_to_depth', color_to_depth)

    inner = (a.cols - 1, a.rows - 1)
    sq = a.square_mm / 1000.0
    obj = np.zeros((inner[0] * inner[1], 3))
    obj[:, :2] = np.mgrid[0:inner[0], 0:inner[1]].T.reshape(-1, 2) * sq
    board_c = np.array([(inner[0] - 1) * sq / 2, (inner[1] - 1) * sq / 2, 0])
    board_r = 0.5 * math.hypot((a.cols + 1) * sq, (a.rows + 1) * sq)

    rclpy.init()
    node = Capture(a)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    caps = []          # (n_c, d_c, n_l, d_l)
    solved = None      # lidar_T_color
    overlay = False
    win = 'calib_capture'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    print(__doc__.split('Keys')[1])

    while rclpy.ok():
        with node.lock:
            img = None if node.img is None else node.img.copy()
            K, D = node.K, node.D
            cloud = None if node.cloud is None else node.cloud.copy()
        if img is None or K is None:
            cv2.waitKey(100)
            continue
        view = img.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCornersSB(gray, inner, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        cam = None
        lid = None
        if ok:
            cv2.drawChessboardCorners(view, inner, corners, ok)
            okp, rvec, tvec = cv2.solvePnP(obj, corners, K, D)
            if okp:
                Rb = cv2.Rodrigues(rvec)[0]
                n_c = Rb[:, 2]
                d_c = float(n_c @ tvec.ravel())
                if d_c < 0:
                    n_c, d_c = -n_c, -d_c
                center_c = Rb @ board_c + tvec.ravel()
                cam = (n_c, d_c, center_c, Rb, tvec.ravel())
                if cloud is not None and len(cloud):
                    lT = (np.linalg.inv(base_T_lidar) @ base_T_color) if solved is None else solved
                    center_l = lT[:3, :3] @ center_c + lT[:3, 3]
                    roi = cloud[np.linalg.norm(cloud - center_l, axis=1) < board_r + 0.15]
                    lid = fit_plane(roi)
        if overlay and cloud is not None and len(cloud):
            lT = (np.linalg.inv(base_T_lidar) @ base_T_color) if solved is None else solved
            cT = np.linalg.inv(lT)
            pc = cloud @ cT[:3, :3].T + cT[:3, 3]
            pc = pc[(pc[:, 2] > 0.2) & (pc[:, 2] < 8)]
            uv, _ = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), K, D)
            for (u, v), z in zip(uv.reshape(-1, 2), pc[:, 2]):
                if 0 <= u < view.shape[1] and 0 <= v < view.shape[0]:
                    col = cv2.applyColorMap(np.uint8([[min(255, z / 4 * 255)]]), cv2.COLORMAP_JET)[0, 0].tolist()
                    cv2.circle(view, (int(u), int(v)), 2, col, -1)
        status = (f"captures {len(caps)} | board {'OK' if cam else '--'} | lidar plane "
                  f"{(str(len(lid[2])) + ' pts') if lid else '--'} | {'SOLVED' if solved is not None else ''} | c s v w u q")
        cv2.putText(view, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow(win, view)
        k = cv2.waitKey(30) & 0xFF

        if k == ord('q'):
            break
        if k == ord('v'):
            overlay = not overlay
        if k == ord('u') and caps:
            caps.pop()
            print(f'undo -> {len(caps)} captures')
        if k == ord('c'):
            if a.board_pose and cam:
                x, y, z, r, p, yw = [float(v) for v in a.board_pose.split()]
                cr, sr, cp, sp, cy, sy = (math.cos(math.radians(r)), math.sin(math.radians(r)),
                                          math.cos(math.radians(p)), math.sin(math.radians(p)),
                                          math.cos(math.radians(yw)), math.sin(math.radians(yw)))
                Rbb = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                                [-sp, cp * sr, cp * cr]])
                base_T_board = T(Rbb, [x, y, z])
                cam_T_board = T(cam[3], cam[4])
                solved = np.linalg.inv(base_T_lidar) @ base_T_board @ np.linalg.inv(cam_T_board)
                print('board-pose mode: solved from one capture')
            elif cam and lid:
                caps.append((cam[0], cam[1], lid[0], lid[1]))
                print(f'capture {len(caps)}: camera normal {np.round(cam[0], 3)} d {cam[1]:.3f} m | '
                      f'lidar normal {np.round(lid[0], 3)} d {lid[1]:.3f} m ({len(lid[2])} pts)')
            else:
                print('not captured: need the board detected AND a LiDAR plane on the screen')
        if k == ord('s') and not a.board_pose:
            if len(caps) < 3:
                print('need >= 3 captures with different tilts')
                continue
            Nc = np.array([c[0] for c in caps])
            Nl = np.array([c[2] for c in caps])
            if np.linalg.matrix_rank(Nc, tol=0.15) < 3:
                print('normals too similar: tilt the laptop more (left/right AND up/down)')
                continue
            U, _, Vt = np.linalg.svd(Nl.T @ Nc)
            R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt          # lidar <- camera
            b = np.array([c[3] - c[1] for c in caps])
            t, *_ = np.linalg.lstsq(Nl, b, rcond=None)
            solved = T(R, t)
            ang = [math.degrees(math.acos(np.clip(nl @ (R @ nc), -1, 1))) for nc, _, nl, _ in caps]
            res = Nl @ t - b
            print(f'solved: normal residual mean {np.mean(ang):.2f} deg (max {np.max(ang):.2f}), '
                  f'distance residual RMS {np.sqrt(np.mean(res ** 2)) * 100:.1f} cm')
            print('check with v: LiDAR points must land on the laptop, walls and floor edges')
        if k == ord('w'):
            if solved is None:
                print('solve first (s)')
                continue
            bc = base_T_lidar @ solved
            out = {'cone_tracker_node': {'ros__parameters': {
                'base_to_color_optical': {'xyz': [round(float(v), 4) for v in bc[:3, 3]],
                                          'q_xyzw': [round(float(v), 5) for v in R_to_quat(bc[:3, :3])]},
                'base_to_lidar': {'xyz': [round(float(v), 4) for v in base_T_lidar[:3, 3]],
                                  'q_xyzw': [round(float(v), 5) for v in R_to_quat(base_T_lidar[:3, :3])]},
                'color_to_depth': color_to_depth,
                'calibration_info': {'calibrated': True,
                                     'date': datetime.datetime.now().isoformat(timespec='seconds'),
                                     'captures': len(caps), 'square_mm': a.square_mm,
                                     'mode': 'board_pose' if a.board_pose else 'lidar_plane'}}}}
            with open(a.out, 'w') as f:
                f.write('# Written by calib_capture.py. base_link -> colour optical frame (z forward, x right, y down).\n')
                yaml.safe_dump(out, f, sort_keys=False)
            print(f'written {a.out}')
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
