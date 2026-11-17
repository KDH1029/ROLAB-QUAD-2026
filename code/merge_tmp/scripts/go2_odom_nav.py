#!/usr/bin/env python3
"""
CLI launcher for the C++ Go2 odometry controller (`go2_bridge/odom_nav_node`).

Reads parameters from config/system_params.yaml and waypoints from config/targets.json,
then replaces the Python process in-place with the compiled C++ `odom_nav_node`
binary via os.execvp (zero Python runtime overhead during 20Hz control).
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

WS_ROOT = Path(__file__).resolve().parent.parent
ODOM_WP_FILE = str(WS_ROOT / 'config' / 'targets.json')
SYS_CONFIG_FILE = WS_ROOT / 'config' / 'system_params.yaml'
CPP_NAV_BIN = WS_ROOT / 'install' / 'go2_bridge' / 'lib' / 'go2_bridge' / 'odom_nav_node'


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


def build_cpp_cmd(ros_params):
    if CPP_NAV_BIN.exists():
        cmd = [str(CPP_NAV_BIN)]
    elif shutil.which('ros2'):
        cmd = ['ros2', 'run', 'go2_bridge', 'odom_nav_node']
    else:
        print("[ERROR] C++ odom_nav_node binary not found. Run 'colcon build' first.")
        sys.exit(1)

    cmd.append('--ros-args')
    for k, v in ros_params.items():
        if isinstance(v, bool):
            val_str = 'true' if v else 'false'
        else:
            val_str = str(v)
        cmd.extend(['-p', f'{k}:={val_str}'])
    return cmd


def save_current_pose_via_cpp(name, ref_frame, wps):
    cmd = build_cpp_cmd({'mode': 'status', 'reference_frame': ref_frame})
    res = subprocess.run(cmd, capture_output=True, text=True, check=False)
    out = res.stdout
    if 'position' not in out:
        print("[ERROR] Failed to read current pose from C++ odom_nav_node.")
        return

    x = y = z = yaw = 0.0
    for line in out.splitlines():
        if line.startswith('position'):
            parts = line.split(':')[1].replace('m', '').split(',')
            x = float(parts[0].split('=')[1])
            y = float(parts[1].split('=')[1])
            z = float(parts[2].split('=')[1])
        elif line.startswith('yaw'):
            yaw = float(line.split(':')[1].replace('deg', '').strip())

    wps[name] = {'x': round(x, 3), 'y': round(y, 3), 'z': round(z, 3), 'yaw': round(yaw, 1), 'frame': ref_frame}
    save_waypoints(wps)
    print(f"[SAVED] '{name}': x={x:+.3f}m, y={y:+.3f}m, z={z:+.3f}m, yaw={yaw:+.1f} deg")


def main():
    cfg = load_nav_config()

    parser = argparse.ArgumentParser(description='Go2 C++ odometry waypoint controller CLI')
    parser.add_argument('--x', type=float, default=None, help='Target X (m)')
    parser.add_argument('--y', type=float, default=None, help='Target Y (m)')
    parser.add_argument('--yaw', type=float, default=0.0, help='Target Yaw (deg)')
    parser.add_argument('--frame', type=str, default=cfg['default_frame'], choices=['odom', 'map'], help='Reference frame')
    parser.add_argument('--speed', type=float, default=cfg['max_vx'], help='Max linear speed (m/s)')
    parser.add_argument('--sit', action='store_true', help='Stand down on arrival')
    parser.add_argument('--daemon', action='store_true', help='Run C++ node in continuous /odom_goal subscriber mode')
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

    if args.subcommand == 'save':
        if not args.target_name:
            print("Usage: python3 scripts/go2_odom_nav.py save <name>")
            return
        save_current_pose_via_cpp(args.target_name, args.frame, wps)
        return

    if args.subcommand == 'status':
        cmd = build_cpp_cmd({'mode': 'status', 'reference_frame': args.frame})
        os.execvp(cmd[0], cmd)

    if args.daemon:
        cmd = build_cpp_cmd({
            'mode': 'daemon',
            'reference_frame': args.frame,
            'max_vx': float(args.speed),
            'max_wz': float(cfg['max_wz']),
            'min_vx': float(cfg['min_vx']),
            'min_wz': float(cfg['min_wz']),
            'xy_goal_tolerance_m': float(cfg['xy_goal_tolerance_m']),
            'yaw_goal_tolerance_deg': float(cfg['yaw_goal_tolerance_deg']),
            'heading_align_thresh_deg': float(cfg['heading_align_thresh_deg']),
            'obstacle_stop_dist_m': float(cfg['obstacle_stop_dist_m']),
            'obstacle_check_fov_deg': float(cfg['obstacle_check_fov_deg']),
            'kp_linear': float(cfg['kp_linear']),
            'kp_angular': float(cfg['kp_angular']),
        })
        os.execvp(cmd[0], cmd)

    tx, ty, tyaw = None, None, 0.0
    if args.subcommand == 'go' or (args.subcommand in wps):
        tname = args.target_name if args.subcommand == 'go' else args.subcommand
        if tname not in wps:
            print(f"[ERROR] Unknown waypoint '{tname}'. Available: {list(wps.keys())}")
            return
        t = wps[tname]
        tx, ty, tyaw = float(t['x']), float(t['y']), float(t.get('yaw', 0.0))
    elif args.x is not None and args.y is not None:
        tx, ty, tyaw = float(args.x), float(args.y), float(args.yaw)

    if tx is None or ty is None:
        parser.print_help()
        return

    cmd = build_cpp_cmd({
        'mode': 'goto',
        'reference_frame': args.frame,
        'target_x': tx,
        'target_y': ty,
        'target_yaw_deg': tyaw,
        'sit_on_arrival': bool(args.sit),
        'max_vx': float(args.speed),
        'max_wz': float(cfg['max_wz']),
        'min_vx': float(cfg['min_vx']),
        'min_wz': float(cfg['min_wz']),
        'xy_goal_tolerance_m': float(cfg['xy_goal_tolerance_m']),
        'yaw_goal_tolerance_deg': float(cfg['yaw_goal_tolerance_deg']),
        'heading_align_thresh_deg': float(cfg['heading_align_thresh_deg']),
        'obstacle_stop_dist_m': float(cfg['obstacle_stop_dist_m']),
        'obstacle_check_fov_deg': float(cfg['obstacle_check_fov_deg']),
        'kp_linear': float(cfg['kp_linear']),
        'kp_angular': float(cfg['kp_angular']),
    })
    os.execvp(cmd[0], cmd)


if __name__ == '__main__':
    main()
