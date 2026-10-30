#!/usr/bin/env python3
"""
CLI wrapper for the C++ Go2 3D mapper (`go2_bridge/mapper_node`).

Loads parameters from config/system_params.yaml and replaces the current
process with the compiled C++ `mapper_node` executable via os.execvp,
eliminating Python runtime overhead during mapping and PCD conversion.
"""

import argparse
import os
import shutil
import sys
import time
from pathlib import Path

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

WS_ROOT = Path(__file__).resolve().parent.parent
MAPS_DIR = WS_ROOT / 'maps'
SYS_CONFIG_FILE = WS_ROOT / 'config' / 'system_params.yaml'
CPP_MAPPER_BIN = WS_ROOT / 'install' / 'go2_bridge' / 'lib' / 'go2_bridge' / 'mapper_node'


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


def resolve_cpp_binary():
    if CPP_MAPPER_BIN.exists():
        return str(CPP_MAPPER_BIN)
    if shutil.which('ros2'):
        return 'ros2'
    return None


def exec_cpp_mapper(ros_params):
    bin_path = resolve_cpp_binary()
    if bin_path is None:
        print("[ERROR] C++ mapper_node binary not found. Run 'colcon build' first.")
        sys.exit(1)

    param_args = ['--ros-args']
    for k, v in ros_params.items():
        if isinstance(v, bool):
            val_str = 'true' if v else 'false'
        else:
            val_str = str(v)
        param_args.extend(['-p', f'{k}:={val_str}'])

    if bin_path == 'ros2':
        cmd = ['ros2', 'run', 'go2_bridge', 'mapper_node'] + param_args
    else:
        cmd = [bin_path] + param_args

    os.execvp(cmd[0], cmd)


def main():
    cfg = load_mapping_config()

    parser = argparse.ArgumentParser(description='Go2 C++ 3D point cloud mapper & 2D grid converter CLI')
    sub = parser.add_subparsers(dest='mode', help='Mode: live or pcd2map')

    p_live = sub.add_parser('live', help='Real-time odometry + LiDAR mapping (C++ mapper_node)')
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

    p_pcd = sub.add_parser('pcd2map', help='Convert 3D .pcd to 2D .pgm/.yaml (C++ mapper_node)')
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
        exec_cpp_mapper({
            'mode': 'live',
            'maps_dir': str(MAPS_DIR),
            'map_name': args.name,
            'scan_topic': args.scan_topic,
            'cloud_topic': args.cloud_topic,
            'resolution': float(args.res),
            'local_floor_cell_m': float(cfg['local_floor_cell_m']),
            'local_floor_percentile': float(cfg['local_floor_percentile']),
            'wall_z_min': float(args.zmin),
            'wall_z_max': float(args.zmax),
            'free_pad_m': float(args.free_pad),
            'keyframe_dist_m': float(args.keyframe_dist),
            'keyframe_angle_deg': float(args.keyframe_deg),
            'duration_sec': float(args.duration),
            'set_default': bool(args.set_default),
        })
    elif args.mode == 'pcd2map':
        src_path = os.path.abspath(os.path.expanduser(args.pcd))
        map_name = args.name or os.path.splitext(os.path.basename(src_path))[0]
        exec_cpp_mapper({
            'mode': 'pcd2map',
            'pcd_path': src_path,
            'maps_dir': str(MAPS_DIR),
            'map_name': map_name,
            'resolution': float(args.res),
            'local_floor_cell_m': float(cfg['local_floor_cell_m']),
            'local_floor_percentile': float(cfg['local_floor_percentile']),
            'wall_z_min': float(args.zmin),
            'wall_z_max': float(args.zmax),
            'min_pts_per_cell': int(args.min_pts),
            'free_pad_m': float(args.free_pad),
            'local_level': not bool(args.no_level),
            'set_default': bool(args.set_default),
        })
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
