#!/usr/bin/env python3
"""
3D point cloud mapper and 2D occupancy grid converter for Unitree Go2.

Supports two modes:
  1. live    : Accumulates keyframes from /scan or /velodyne_points in the
               gravity-stabilized frame (base_stab) and exports .pcd/.pgm/.yaml.
  2. pcd2map : Converts an existing 3D .pcd file into a 2.5D elevation grid (.npz)
               and a local-floor-leveled 2D occupancy grid (.pgm/.yaml).

Parameters are loaded from config/system_params.yaml.
"""

import argparse
import math
import os
import shutil
import sys
import time
from pathlib import Path

import numpy as np

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

WS_ROOT = Path(__file__).resolve().parent.parent
MAPS_DIR = WS_ROOT / 'maps'
SYS_CONFIG_FILE = WS_ROOT / 'config' / 'system_params.yaml'


def load_mapping_config():
    defaults = {
        'resolution': 0.03,
        'local_floor_cell_m': 0.50,
        'local_floor_percentile': 5.0,
        'wall_z_min': 0.22,
        'wall_z_max': 1.15,
        'min_pts_per_cell': 2,
        'free_pad_m': 0.80,
        'keyframe_dist_m': 0.10,
        'keyframe_angle_deg': 5.0,
    }
    if HAS_YAML and SYS_CONFIG_FILE.exists():
        try:
            with open(SYS_CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
                defaults.update(data.get('mapping', {}))
        except Exception:
            pass
    return defaults


def read_pcd(path):
    hdr = {}
    with open(path, 'rb') as f:
        while True:
            ln = f.readline()
            if not ln:
                raise RuntimeError('Unexpected EOF while parsing PCD header')
            parts = ln.decode('ascii', 'ignore').strip().split()
            if not parts:
                continue
            hdr[parts[0]] = parts[1:]
            if parts[0] == 'DATA':
                break

        n_pts = int(hdr['POINTS'][0])
        data_mode = hdr['DATA'][0].lower()

        if data_mode == 'binary':
            fields = hdr.get('FIELDS', ['x', 'y', 'z'])
            sizes = [int(x) for x in hdr.get('SIZE', [4, 4, 4])]
            types = hdr.get('TYPE', ['F'] * len(sizes))
            fmt_list = []
            for s, t in zip(sizes, types):
                if t == 'F':
                    fmt_list.append('<f4' if s == 4 else '<f8')
                elif t == 'U':
                    fmt_list.append(f'<u{s}')
                else:
                    fmt_list.append(f'<i{s}')
            dt = np.dtype({'names': fields, 'formats': fmt_list})
            raw = np.frombuffer(f.read(n_pts * dt.itemsize), dtype=dt, count=n_pts)
            xyz = np.c_[raw['x'], raw['y'], raw['z']].astype(np.float64)
        else:
            xyz = np.loadtxt(f, usecols=(0, 1, 2), max_rows=n_pts, dtype=np.float64)

    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid]


def write_pcd(path, xyz):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    pts = np.asarray(xyz, dtype='<f4')
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(pts)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(pts)}\n"
        "DATA binary\n"
    )
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(pts.tobytes())


def extract_local_floor_and_elevation(xyz, cell_size=0.5, percentile=5.0):
    """Estimates cell-wise ground elevation to remove hill slopes before wall slicing."""
    if len(xyz) == 0:
        return xyz, {}

    x_min, y_min = float(xyz[:, 0].min()), float(xyz[:, 1].min())
    x_max, y_max = float(xyz[:, 0].max()), float(xyz[:, 1].max())
    nx = max(1, int(np.ceil((x_max - x_min) / cell_size)) + 1)
    ny = max(1, int(np.ceil((y_max - y_min) / cell_size)) + 1)

    ix = np.clip(((xyz[:, 0] - x_min) / cell_size).astype(np.int64), 0, nx - 1)
    iy = np.clip(((xyz[:, 1] - y_min) / cell_size).astype(np.int64), 0, ny - 1)
    key = iy * nx + ix

    order = np.argsort(key, kind='stable')
    pts_sorted = xyz[order]
    keys_sorted = key[order]

    bounds = np.r_[0, np.flatnonzero(keys_sorted[1:] != keys_sorted[:-1]) + 1, len(keys_sorted)]
    dz = np.empty(len(pts_sorted), dtype=np.float64)
    elev_grid = np.full((ny, nx), np.nan, dtype=np.float32)

    for start, end in zip(bounds[:-1], bounds[1:]):
        z_slice = pts_sorted[start:end, 2]
        floor_z = float(np.percentile(z_slice, percentile)) if (end - start) >= 5 else float(z_slice.min())
        dz[start:end] = z_slice - floor_z
        k = int(keys_sorted[start])
        elev_grid[k // nx, k % nx] = floor_z

    valid_elev = elev_grid[np.isfinite(elev_grid)]
    hill_relief = float(valid_elev.max() - valid_elev.min()) if len(valid_elev) > 0 else 0.0

    elev_info = {
        'elevation_grid': elev_grid,
        'origin_x': x_min,
        'origin_y': y_min,
        'cell_size': cell_size,
        'hill_relief_m': hill_relief,
    }
    return np.c_[pts_sorted[:, 0], pts_sorted[:, 1], dz], elev_info


def points_to_occupancy_grid(
    xyz,
    res=0.03,
    z_min=0.22,
    z_max=1.15,
    min_pts=2,
    local_level=True,
    cell_size=0.50,
    percentile=5.0,
    free_pad_m=0.80
):
    if len(xyz) == 0:
        raise ValueError("Input point cloud is empty.")

    if local_level:
        pts_proc, elev_info = extract_local_floor_and_elevation(
            xyz, cell_size=cell_size, percentile=percentile)
    else:
        pts_proc = xyz.copy()
        floor_z = float(np.percentile(pts_proc[:, 2], 2.0))
        pts_proc[:, 2] -= floor_z
        elev_info = {'hill_relief_m': 0.0}

    band_mask = (pts_proc[:, 2] >= z_min) & (pts_proc[:, 2] <= z_max)
    wall_pts = pts_proc[band_mask]
    if len(wall_pts) < 10:
        wall_pts = pts_proc

    pad = 1.0
    x0 = float(wall_pts[:, 0].min() - pad)
    y0 = float(wall_pts[:, 1].min() - pad)
    width = int(np.ceil((wall_pts[:, 0].max() + pad - x0) / res))
    height = int(np.ceil((wall_pts[:, 1].max() + pad - y0) / res))

    gx = ((wall_pts[:, 0] - x0) / res).astype(np.int32)
    gy = ((wall_pts[:, 1] - y0) / res).astype(np.int32)
    valid = (gx >= 0) & (gy >= 0) & (gx < width) & (gy < height)

    counts = np.zeros((height, width), dtype=np.int32)
    np.add.at(counts, (gy[valid], gx[valid]), 1)

    img = np.full((height, width), 205, dtype=np.uint8)

    all_gx = ((pts_proc[:, 0] - x0) / res).astype(np.int32)
    all_gy = ((pts_proc[:, 1] - y0) / res).astype(np.int32)
    all_valid = (all_gx >= 0) & (all_gy >= 0) & (all_gx < width) & (all_gy < height)
    img[all_gy[all_valid], all_gx[all_valid]] = 254

    if free_pad_m > 0.0:
        rad_cells = max(1, int(round(free_pad_m / res)))
        free_mask = (img == 254)
        for dy in range(-min(rad_cells, 8), min(rad_cells, 8) + 1):
            for dx in range(-min(rad_cells, 8), min(rad_cells, 8) + 1):
                if dx * dx + dy * dy <= 64:
                    shifted = np.roll(np.roll(free_mask, dy, axis=0), dx, axis=1)
                    img[shifted] = 254

    img[counts >= min_pts] = 0

    meta = {
        'width': width,
        'height': height,
        'resolution': res,
        'origin_x': x0,
        'origin_y': y0,
        'occupied_cells': int((img == 0).sum()),
        'wall_points': len(wall_pts),
        'total_points': len(xyz),
        'hill_relief_m': elev_info.get('hill_relief_m', 0.0),
        'elev_info': elev_info,
    }
    return img, meta


def save_map_files(img, meta, map_name, set_default=False):
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    pgm_path = MAPS_DIR / f"{map_name}.pgm"
    yaml_path = MAPS_DIR / f"{map_name}.yaml"
    elev_path = MAPS_DIR / f"{map_name}_elevation.npz"

    pgm_img = np.flipud(img)
    h, w = pgm_img.shape
    with open(pgm_path, 'wb') as f:
        f.write(f"P5\n{w} {h}\n255\n".encode('ascii'))
        f.write(pgm_img.tobytes())

    yaml_content = (
        f"image: {map_name}.pgm\n"
        "mode: trinary\n"
        f"resolution: {meta['resolution']:.3f}\n"
        f"origin: [{meta['origin_x']:.3f}, {meta['origin_y']:.3f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(yaml_content)

    elev_info = meta.get('elev_info', {})
    if 'elevation_grid' in elev_info:
        np.savez_compressed(
            elev_path,
            elevation_grid=elev_info['elevation_grid'],
            origin_x=elev_info['origin_x'],
            origin_y=elev_info['origin_y'],
            cell_size=elev_info['cell_size']
        )

    print(f"[DONE] Map saved:")
    print(f"  - PGM      : {pgm_path} ({w}x{h} px, res={meta['resolution']}m)")
    print(f"  - YAML     : {yaml_path} (origin=[{meta['origin_x']:.2f}, {meta['origin_y']:.2f}])")
    if 'elevation_grid' in elev_info:
        print(f"  - Elevation: {elev_path} (relief={meta['hill_relief_m']:.2f}m)")
    print(f"  - Occupied : {meta['occupied_cells']} cells ({meta['wall_points']:,} wall pts)")

    if set_default and map_name != "map_default":
        shutil.copyfile(pgm_path, MAPS_DIR / "map_default.pgm")
        with open(MAPS_DIR / "map_default.yaml", 'w', encoding='utf-8') as f:
            f.write(yaml_content.replace(f"image: {map_name}.pgm", "image: map_default.pgm"))
        if elev_path.exists():
            shutil.copyfile(elev_path, MAPS_DIR / "map_default_elevation.npz")
        print("  - Updated default map (map_default.yaml / map_default_elevation.npz)")

    return str(yaml_path)


def run_live_mapper(args, cfg):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan, PointCloud2
    import tf2_ros

    class Go2LiveMapper(Node):
        def __init__(self):
            super().__init__('go2_live_mapper')
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

            self.accumulated_points = []
            self.scan_count = 0
            self.last_pose = None

            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.scan_sub = self.create_subscription(
                LaserScan, args.scan_topic, self.on_scan, qos)
            self.cloud_sub = self.create_subscription(
                PointCloud2, args.cloud_topic, self.on_cloud, qos)

            self.get_logger().info(
                f"live mapper started: name={args.name} (scan={args.scan_topic}, cloud={args.cloud_topic})"
            )

        def lookup_odom_transform(self, source_frame):
            for frame in (source_frame, 'base_stab', 'base_link'):
                try:
                    t = self.tf_buffer.lookup_transform('odom', frame, rclpy.time.Time())
                    tx = t.transform.translation.x
                    ty = t.transform.translation.y
                    tz = t.transform.translation.z
                    q = t.transform.rotation
                    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                    return tx, ty, tz, yaw
                except Exception:
                    continue
            return None

        def should_record_keyframe(self, tx, ty, yaw):
            if self.last_pose is None:
                self.last_pose = (tx, ty, yaw)
                return True
            lx, ly, lyaw = self.last_pose
            dist = math.hypot(tx - lx, ty - ly)
            dyaw = abs((yaw - lyaw + math.pi) % (2.0 * math.pi) - math.pi)
            if dist >= args.keyframe_dist or dyaw >= math.radians(args.keyframe_deg):
                self.last_pose = (tx, ty, yaw)
                return True
            if self.scan_count < 10:
                return True
            return False

        def on_scan(self, msg: LaserScan):
            pose = self.lookup_odom_transform(msg.header.frame_id or 'base_stab')
            if pose is None:
                return
            tx, ty, tz, yaw = pose
            if not self.should_record_keyframe(tx, ty, yaw):
                return

            angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
            ranges = np.asarray(msg.ranges, dtype=np.float64)
            valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
            if not np.any(valid):
                return

            r_v = ranges[valid]
            a_v = angles[valid] + yaw
            px = tx + r_v * np.cos(a_v)
            py = ty + r_v * np.sin(a_v)
            pz = np.full_like(px, tz + 0.5)

            robot_seed = np.array([[tx, ty, tz]], dtype=np.float64)
            scan_pts = np.c_[px, py, pz]
            self.accumulated_points.append(np.vstack([robot_seed, scan_pts]))
            self.scan_count += 1

            if self.scan_count % 10 == 0:
                total_p = sum(len(x) for x in self.accumulated_points)
                print(
                    f"\r  keyframes={self.scan_count} | pts={total_p:,} | "
                    f"pose=({tx:+.2f}, {ty:+.2f}, z={tz:+.2f}m, {math.degrees(yaw):+.0f} deg)",
                    end='', flush=True)

        def on_cloud(self, msg: PointCloud2):
            pose = self.lookup_odom_transform(msg.header.frame_id or 'base_stab')
            if pose is None:
                return
            tx, ty, tz, yaw = pose
            if not self.should_record_keyframe(tx, ty, yaw):
                return

            step = msg.point_step
            n_pts = msg.width * msg.height
            if n_pts == 0 or step < 12:
                return
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_pts, step)
            xyz_local = raw[:, :12].view('<f4').reshape(n_pts, 3).astype(np.float64)
            valid = np.isfinite(xyz_local).all(axis=1)
            xyz_local = xyz_local[valid][::4]
            if len(xyz_local) == 0:
                return

            c, s = math.cos(yaw), math.sin(yaw)
            px = tx + c * xyz_local[:, 0] - s * xyz_local[:, 1]
            py = ty + s * xyz_local[:, 0] + c * xyz_local[:, 1]
            pz = tz + xyz_local[:, 2]

            self.accumulated_points.append(np.c_[px, py, pz])
            self.scan_count += 1

    rclpy.init()
    node = Go2LiveMapper()
    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration > 0 and (time.time() - t0) >= args.duration:
                print(f"\n[INFO] Duration ({args.duration}s) reached. Exporting map...")
                break
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted. Exporting accumulated map...")
    finally:
        if node.accumulated_points:
            all_xyz = np.vstack(node.accumulated_points)
            pcd_out = MAPS_DIR / f"{args.name}.pcd"
            write_pcd(str(pcd_out), all_xyz)
            print(f"\n[DONE] 3D PCD saved: {pcd_out} ({len(all_xyz):,} pts)")

            img, meta = points_to_occupancy_grid(
                all_xyz, res=args.res, z_min=args.zmin, z_max=args.zmax,
                min_pts=1, local_level=True,
                cell_size=cfg['local_floor_cell_m'],
                percentile=cfg['local_floor_percentile'],
                free_pad_m=args.free_pad)
            save_map_files(img, meta, args.name, set_default=args.set_default)
        else:
            print("\n[WARN] No scan data accumulated.")
        node.destroy_node()
        rclpy.shutdown()


def run_pcd2map(args, cfg):
    src_path = os.path.expanduser(args.pcd)
    if not os.path.isfile(src_path):
        print(f"[ERROR] File not found: {src_path}")
        sys.exit(1)

    map_name = args.name or os.path.splitext(os.path.basename(src_path))[0]
    print(f"[INFO] Loading PCD: {src_path} (config={SYS_CONFIG_FILE.name})")
    xyz = read_pcd(src_path)
    print(
        f"  points={len(xyz):,} | x=[{xyz[:,0].min():.1f}, {xyz[:,0].max():.1f}]m, "
        f"y=[{xyz[:,1].min():.1f}, {xyz[:,1].max():.1f}]m, z=[{xyz[:,2].min():.2f}, {xyz[:,2].max():.2f}]m"
    )

    img, meta = points_to_occupancy_grid(
        xyz, res=args.res, z_min=args.zmin, z_max=args.zmax,
        min_pts=args.min_pts, local_level=not args.no_level,
        cell_size=cfg['local_floor_cell_m'],
        percentile=cfg['local_floor_percentile'],
        free_pad_m=args.free_pad)
    save_map_files(img, meta, map_name, set_default=args.set_default)


def main():
    cfg = load_mapping_config()

    parser = argparse.ArgumentParser(description='Go2 3D point cloud mapper & 2D grid converter')
    sub = parser.add_subparsers(dest='mode', help='Mode: live or pcd2map')

    p_live = sub.add_parser('live', help='Real-time odometry + LiDAR mapping')
    p_live.add_argument('--name', type=str, default=f"map_{time.strftime('%m%d_%H%M')}", help='Output map name')
    p_live.add_argument('--scan-topic', type=str, default='/scan', help='2D LaserScan topic')
    p_live.add_argument('--cloud-topic', type=str, default='/velodyne_points', help='3D PointCloud2 topic')
    p_live.add_argument('--res', type=float, default=cfg['resolution'], help='Grid resolution (m)')
    p_live.add_argument('--zmin', type=float, default=cfg['wall_z_min'], help='Min wall height above local floor (m)')
    p_live.add_argument('--zmax', type=float, default=cfg['wall_z_max'], help='Max wall height above local floor (m)')
    p_live.add_argument('--free-pad', type=float, default=cfg['free_pad_m'], help='Free space padding radius (m)')
    p_live.add_argument('--keyframe-dist', type=float, default=cfg['keyframe_dist_m'], help='Keyframe distance (m)')
    p_live.add_argument('--keyframe-deg', type=float, default=cfg['keyframe_angle_deg'], help='Keyframe angle (deg)')
    p_live.add_argument('--duration', type=float, default=0.0, help='Auto-stop duration (s)')
    p_live.add_argument('--set-default', action='store_true', help='Set as map_default')

    p_pcd = sub.add_parser('pcd2map', help='Convert 3D .pcd to 2D .pgm/.yaml and elevation grid')
    p_pcd.add_argument('pcd', type=str, help='Input .pcd path')
    p_pcd.add_argument('--name', type=str, default='', help='Output map name')
    p_pcd.add_argument('--res', type=float, default=cfg['resolution'], help='Grid resolution (m)')
    p_pcd.add_argument('--zmin', type=float, default=cfg['wall_z_min'], help='Min wall height above local floor (m)')
    p_pcd.add_argument('--zmax', type=float, default=cfg['wall_z_max'], help='Max wall height above local floor (m)')
    p_pcd.add_argument('--min-pts', type=int, default=cfg['min_pts_per_cell'], help='Min points per occupied cell')
    p_pcd.add_argument('--free-pad', type=float, default=cfg['free_pad_m'], help='Free space padding radius (m)')
    p_pcd.add_argument('--no-level', action='store_true', help='Disable local floor leveling')
    p_pcd.add_argument('--set-default', action='store_true', help='Set as map_default')

    args = parser.parse_args()
    if args.mode == 'live':
        run_live_mapper(args, cfg)
    elif args.mode == 'pcd2map':
        run_pcd2map(args, cfg)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
