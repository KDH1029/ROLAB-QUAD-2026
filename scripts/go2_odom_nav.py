#!/usr/bin/env python3
"""
Direct odometry-based waypoint controller for Unitree Go2.

Reads control parameters from config/system_params.yaml (navigation_control section)
and tracks 3D pose (x, y, z, roll, pitch, yaw) with front LaserScan obstacle check.
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

try:
    from unitree_api.msg import Request
    HAS_UNITREE_API = True
except ImportError:
    Request = None
    HAS_UNITREE_API = False

WS_ROOT = Path(__file__).resolve().parent.parent
ODOM_WP_FILE = str(WS_ROOT / 'config' / 'targets.json')
SYS_CONFIG_FILE = WS_ROOT / 'config' / 'system_params.yaml'
DEFAULT_ELEV_FILE = WS_ROOT / 'maps' / 'map_default_elevation.npz'


def load_nav_config():
    defaults = {
        'default_frame': 'odom',
        'max_vx': 0.65,
        'max_vy': 0.35,
        'max_wz': 1.20,
        'min_vx': 0.08,
        'min_vy': 0.08,
        'min_wz': 0.15,
        'xy_goal_tolerance_m': 0.18,
        'yaw_goal_tolerance_deg': 6.0,
        'heading_align_thresh_deg': 25.0,
        'obstacle_stop_dist_m': 0.35,
        'obstacle_check_fov_deg': 20.0,
        'kp_linear': 0.85,
        'kp_angular': 1.60,
    }
    if HAS_YAML and SYS_CONFIG_FILE.exists():
        try:
            with open(SYS_CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
                defaults.update(data.get('navigation_control', {}))
        except Exception:
            pass
    return defaults


def quat_to_rpy(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def normalize_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def load_waypoints():
    if os.path.exists(ODOM_WP_FILE):
        try:
            with open(ODOM_WP_FILE, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            return {}
    return {}


def save_waypoints(d):
    os.makedirs(os.path.dirname(ODOM_WP_FILE), exist_ok=True)
    with open(ODOM_WP_FILE, 'w', encoding='utf-8') as f:
        json.dump(d, f, ensure_ascii=False, indent=2)


class ElevationMapQuery:
    def __init__(self, npz_path=DEFAULT_ELEV_FILE):
        self.valid = False
        if os.path.exists(npz_path):
            try:
                data = np.load(npz_path)
                self.grid = data['elevation_grid']
                self.ox = float(data['origin_x'])
                self.oy = float(data['origin_y'])
                self.cell = float(data['cell_size'])
                self.valid = True
            except Exception:
                self.valid = False

    def get_elevation(self, x, y):
        if not self.valid:
            return None
        ix = int((x - self.ox) / self.cell)
        iy = int((y - self.oy) / self.cell)
        ny, nx = self.grid.shape
        if 0 <= ix < nx and 0 <= iy < ny:
            val = float(self.grid[iy, ix])
            if math.isfinite(val):
                return val
        return None


class Go2OdomNavigator(Node):
    def __init__(self, cfg, ref_frame=None):
        super().__init__('go2_odom_navigator')
        self.cfg = cfg
        self.ref_frame = ref_frame or cfg['default_frame']
        self.elev_map = ElevationMapQuery()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sport_pub = (
            self.create_publisher(Request, '/api/sport/request', 10)
            if HAS_UNITREE_API else None
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.raw_odom = None
        self.front_clear_dist = 99.9

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            Odometry, '/utlidar/robot_odom', self.on_odom, qos)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos)

    def on_odom(self, msg: Odometry):
        self.raw_odom = msg

    def on_scan(self, msg: LaserScan):
        if not msg.ranges:
            return
        fov_rad = math.radians(self.cfg['obstacle_check_fov_deg'])
        min_d = 99.9
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            ang = msg.angle_min + i * msg.angle_increment
            if abs(ang) <= fov_rad and r < min_d:
                min_d = r
        self.front_clear_dist = min_d

    def spin_wait(self, sec=0.05):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)

    def get_pose(self, timeout_sec=1.0):
        t0 = time.time()
        frames = [self.ref_frame, 'odom', 'map']
        while time.time() - t0 < timeout_sec:
            self.spin_wait(0.05)
            for f in frames:
                try:
                    tr = self.tf_buffer.lookup_transform(f, 'base_link', rclpy.time.Time())
                    x = tr.transform.translation.x
                    y = tr.transform.translation.y
                    z = tr.transform.translation.z
                    r, p, yaw = quat_to_rpy(tr.transform.rotation)
                    return {
                        'frame': f, 'x': x, 'y': y, 'z': z,
                        'roll_deg': math.degrees(r),
                        'pitch_deg': math.degrees(p),
                        'yaw_deg': math.degrees(yaw),
                        'yaw_rad': yaw
                    }
                except Exception:
                    pass

            if self.raw_odom is not None:
                p = self.raw_odom.pose.pose
                r, pt, yaw = quat_to_rpy(p.orientation)
                return {
                    'frame': 'odom(topic)',
                    'x': p.position.x, 'y': p.position.y, 'z': p.position.z,
                    'roll_deg': math.degrees(r),
                    'pitch_deg': math.degrees(pt),
                    'yaw_deg': math.degrees(yaw),
                    'yaw_rad': yaw
                }
        return None

    def get_stab_tilt(self):
        try:
            tr = self.tf_buffer.lookup_transform('base_link', 'base_stab', rclpy.time.Time())
            r, p, _ = quat_to_rpy(tr.transform.rotation)
            return math.degrees(-r), math.degrees(-p)
        except Exception:
            return 0.0, 0.0

    def stop_robot(self):
        cmd = Twist()
        for _ in range(3):
            self.cmd_pub.publish(cmd)
            self.spin_wait(0.03)

    def send_sport_cmd(self, api_id):
        if not self.sport_pub:
            return
        req = Request()
        req.header.identity.id = self.get_clock().now().nanoseconds
        req.header.identity.api_id = api_id
        req.parameter = "{}"
        self.sport_pub.publish(req)

    def drive_to_target(
        self,
        tx,
        ty,
        tyaw_deg,
        max_vx=None,
        sit_on_arrival=False,
        timeout_sec=90.0
    ):
        pose = self.get_pose(timeout_sec=2.0)
        if pose is None:
            print("[ERROR] Odometry unavailable.")
            return False

        max_vx = max_vx or self.cfg['max_vx']
        max_wz = self.cfg['max_wz']
        min_vx = self.cfg['min_vx']
        min_wz = self.cfg['min_wz']
        xy_tol = self.cfg['xy_goal_tolerance_m']
        yaw_tol_deg = self.cfg['yaw_goal_tolerance_deg']
        align_thresh_rad = math.radians(self.cfg['heading_align_thresh_deg'])
        obstacle_stop_m = self.cfg['obstacle_stop_dist_m']
        kp_lin = self.cfg['kp_linear']
        kp_ang = self.cfg['kp_angular']

        tyaw_rad = math.radians(tyaw_deg)
        target_z = self.elev_map.get_elevation(tx, ty)
        z_str = f", map_z={target_z:+.2f}m" if target_z is not None else ""

        print(f"[INFO] Starting odometry navigation (frame={pose['frame']})")
        print(f"  start : x={pose['x']:+.3f}m, y={pose['y']:+.3f}m, z={pose['z']:+.3f}m, yaw={pose['yaw_deg']:+.1f} deg")
        print(f"  target: x={tx:+.3f}m, y={ty:+.3f}m, yaw={tyaw_deg:+.1f} deg{z_str}")

        t_start = time.time()
        last_print = 0.0
        phase = "ALIGN"

        while rclpy.ok():
            elapsed = time.time() - t_start
            if elapsed > timeout_sec:
                print(f"\n[WARN] Timeout ({timeout_sec:.0f}s) exceeded. Stopping.")
                self.stop_robot()
                return False

            pose = self.get_pose(timeout_sec=0.1)
            if pose is None:
                continue

            dx = tx - pose['x']
            dy = ty - pose['y']
            dist_2d = math.hypot(dx, dy)
            bearing = math.atan2(dy, dx)
            heading_err = normalize_angle(bearing - pose['yaw_rad'])
            final_yaw_err = normalize_angle(tyaw_rad - pose['yaw_rad'])

            sr, sp = self.get_stab_tilt()
            cmd = Twist()

            if dist_2d <= xy_tol:
                phase = "YAW_LOCK"
                if abs(math.degrees(final_yaw_err)) <= yaw_tol_deg:
                    self.stop_robot()
                    print(
                        f"\n[DONE] Target reached: err_xy={dist_2d*100:.1f}cm, "
                        f"z={pose['z']:+.2f}m, err_yaw={math.degrees(final_yaw_err):+.1f} deg ({elapsed:.1f}s)"
                    )
                    if sit_on_arrival:
                        self.send_sport_cmd(1005)
                        time.sleep(0.5)
                    return True

                wz = max(-max_wz, min(max_wz, kp_ang * final_yaw_err))
                if abs(wz) < min_wz:
                    wz = math.copysign(min_wz, final_yaw_err)
                cmd.angular.z = wz

            else:
                if self.front_clear_dist < obstacle_stop_m and abs(heading_err) < math.radians(35.0):
                    self.stop_robot()
                    print(f"\r  [HOLD] Front obstacle at {self.front_clear_dist:.2f}m", end='', flush=True)
                    time.sleep(0.2)
                    continue

                if abs(heading_err) > align_thresh_rad:
                    phase = "ALIGN"
                    cmd.linear.x = 0.0
                    wz = max(-max_wz, min(max_wz, kp_ang * heading_err))
                    if abs(wz) < min_wz:
                        wz = math.copysign(min_wz, heading_err)
                    cmd.angular.z = wz
                else:
                    phase = "CRUISE"
                    hill_boost = 0.04 if abs(sp) > 3.0 else 0.0
                    vx = min(max_vx, max(min_vx + 0.04 + hill_boost, kp_lin * dist_2d))
                    wz = max(-max_wz, min(max_wz, kp_ang * heading_err))
                    cmd.linear.x = vx
                    cmd.angular.z = wz

            self.cmd_pub.publish(cmd)

            now = time.time()
            if now - last_print >= 0.3:
                print(
                    f"\r  [{phase:<8}] pos=({pose['x']:+.2f}, {pose['y']:+.2f}, z={pose['z']:+.2f}m, {pose['yaw_deg']:+.0f} deg) | "
                    f"rem={dist_2d:.2f}m | stab_rp=({sr:+.1f}, {sp:+.1f}) deg",
                    end='', flush=True
                )
                last_print = now


def main():
    cfg = load_nav_config()

    parser = argparse.ArgumentParser(description='Go2 odometry waypoint controller')
    parser.add_argument('--x', type=float, default=None, help='Target X (m)')
    parser.add_argument('--y', type=float, default=None, help='Target Y (m)')
    parser.add_argument('--yaw', type=float, default=0.0, help='Target Yaw (deg)')
    parser.add_argument('--frame', type=str, default=cfg['default_frame'], choices=['odom', 'map'], help='Reference frame')
    parser.add_argument('--speed', type=float, default=cfg['max_vx'], help='Max linear speed (m/s)')
    parser.add_argument('--sit', action='store_true', help='Stand down on arrival')
    parser.add_argument('subcommand', nargs='?', default=None, help='Subcommand: status, go, save, list')
    parser.add_argument('target_name', nargs='?', default=None, help='Waypoint name')

    args = parser.parse_args()
    wps = load_waypoints()

    if args.subcommand == 'list':
        print(f"Waypoints ({len(wps)}): {ODOM_WP_FILE}")
        for k, v in wps.items():
            z_val = v.get('z', 0.0)
            print(f"  - {k:<12}: x={v['x']:+.2f}m, y={v['y']:+.2f}m, z={z_val:+.2f}m, yaw={v.get('yaw', 0.0):+.1f} deg")
        return

    rclpy.init()
    nav = Go2OdomNavigator(cfg=cfg, ref_frame=args.frame)

    try:
        if args.subcommand == 'status':
            p = nav.get_pose(timeout_sec=2.0)
            if p is None:
                print("[ERROR] No odometry received.")
            else:
                sr, sp = nav.get_stab_tilt()
                map_z = nav.elev_map.get_elevation(p['x'], p['y'])
                print(f"frame       : {p['frame']} -> base_link")
                print(f"position    : x={p['x']:+.3f}m, y={p['y']:+.3f}m, z={p['z']:+.3f}m")
                if map_z is not None:
                    print(f"map_elev    : z_map={map_z:+.3f}m")
                print(f"yaw         : {p['yaw_deg']:+.2f} deg")
                print(f"stab_rp     : roll={sr:+.2f} deg, pitch={sp:+.2f} deg")
                print(f"front_clear : {nav.front_clear_dist:.2f}m")
            return

        if args.subcommand == 'save':
            if not args.target_name:
                print("Usage: python3 scripts/go2_odom_nav.py save <name>")
                return
            p = nav.get_pose(timeout_sec=2.0)
            if p is None:
                print("[ERROR] Failed to read current pose.")
            else:
                wps[args.target_name] = {
                    'x': round(p['x'], 3),
                    'y': round(p['y'], 3),
                    'z': round(p['z'], 3),
                    'yaw': round(p['yaw_deg'], 1),
                    'frame': p['frame']
                }
                save_waypoints(wps)
                print(
                    f"[SAVED] '{args.target_name}': "
                    f"x={p['x']:+.3f}m, y={p['y']:+.3f}m, z={p['z']:+.3f}m, yaw={p['yaw_deg']:+.1f} deg"
                )
            return

        tx, ty, tyaw = None, None, 0.0
        if args.subcommand == 'go' or (args.subcommand in wps):
            tname = args.target_name if args.subcommand == 'go' else args.subcommand
            if tname not in wps:
                print(f"[ERROR] Unknown waypoint '{tname}'. Available: {list(wps.keys())}")
                return
            t = wps[tname]
            tx, ty, tyaw = t['x'], t['y'], t.get('yaw', 0.0)
        elif args.x is not None and args.y is not None:
            tx, ty, tyaw = args.x, args.y, args.yaw

        if tx is None or ty is None:
            parser.print_help()
            return

        nav.drive_to_target(
            tx, ty, tyaw,
            max_vx=args.speed,
            sit_on_arrival=args.sit
        )

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted. Stopping robot.")
        nav.stop_robot()
    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
