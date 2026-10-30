#!/usr/bin/env python3
"""
2026 RoboWorld Quadruped Competition Map Generator & Hybrid LiDAR Map Merger.

Supports generating the exact competition map from:
  1. ref    : Converts the official 3D orthographic CAD reference texture
              (6.0m width x 8.0m height, upright IN/P/OUT orientation) into 2D grid (.pgm/.yaml).
  2. merge  : Combines a live LiDAR-scanned map (.pgm) with the CAD corridor mask,
              removing traffic cones (23cm) and dynamic clutter from walkable aisles
              while preserving real LiDAR wall measurements near track boundaries.

Usage:
  python3 scripts/build_hybrid_map.py ref --set-default
  python3 scripts/build_hybrid_map.py merge --scan-pgm maps/live_scan.pgm --name roboworld_hybrid
"""

import argparse
import math
import shutil
from pathlib import Path
import cv2
import numpy as np

WS_ROOT = Path(__file__).resolve().parent.parent
MAPS_DIR = WS_ROOT / 'maps'
REF_IMAGE_PATH = MAPS_DIR / 'uploaded_ref.png'


def build_from_ref_image(res=0.03, pad_m=1.0):
    if not REF_IMAGE_PATH.exists():
        raise FileNotFoundError(f"Reference image not found at {REF_IMAGE_PATH}")

    img = cv2.imread(str(REF_IMAGE_PATH))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Crop the track container bounding box
    x1, x2 = 31, 675
    y1, y2 = 70, 961
    cropped = gray[y1:y2+1, x1:x2+1]

    # Target arena dimensions: 6.000m width (X) x 8.000m height (Y)
    target_w = int(round(6.0 / res))  # 200 px at 0.03m
    target_h = int(round(8.0 / res))  # 267 px at 0.03m

    resized = cv2.resize(cropped, (target_w, target_h), interpolation=cv2.INTER_AREA)

    # Walkable aisles: gray road surface + white dashed line marks
    walkable = (resized > 55)

    # Build Occupancy Grid: 205 = unknown, 254 = free, 0 = wall
    grid = np.full((target_h, target_w), 205, dtype=np.uint8)
    grid[walkable] = 254

    wall_mask = np.zeros_like(walkable)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            shifted = np.roll(np.roll(walkable, dy, axis=0), dx, axis=1)
            wall_mask |= (shifted & ~walkable)
    grid[wall_mask] = 0

    # Add 1.0m safety padding around the 6.0m x 8.0m arena
    pad_px = int(round(pad_m / res))
    full_h = target_h + 2 * pad_px
    full_w = target_w + 2 * pad_px
    padded = np.full((full_h, full_w), 205, dtype=np.uint8)
    padded[pad_px:pad_px + target_h, pad_px:pad_px + target_w] = grid

    walkable_full = np.zeros((full_h, full_w), dtype=bool)
    walkable_full[pad_px:pad_px + target_h, pad_px:pad_px + target_w] = walkable

    # 2.5D Elevation Grid for the central diamond plateau (+28cm at X=3.0m, Y=4.0m)
    cell_size = 0.25
    nx = int(math.ceil((6.0 + 2.0 * pad_m) / cell_size))
    ny = int(math.ceil((8.0 + 2.0 * pad_m) / cell_size))
    ex = -pad_m + (np.arange(nx) + 0.5) * cell_size
    ey = -pad_m + (np.arange(ny) + 0.5) * cell_size
    egx, egy = np.meshgrid(ex, ey)

    dist_center = np.hypot(egx - 3.0, egy - 4.0)
    elev_grid = np.clip(0.28 * np.exp(-0.5 * (dist_center / 1.0) ** 2), 0.0, 0.28).astype(np.float32)

    meta = {
        'width': full_w,
        'height': full_h,
        'resolution': res,
        'origin_x': -pad_m,
        'origin_y': -pad_m,
        'elev_grid': elev_grid,
        'elev_cell_size': cell_size,
        'walkable_mask': walkable_full,
    }
    return padded, meta


def read_pgm(path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError(f"Unsupported PGM format: {magic}")
        while True:
            line = f.readline()
            if not line.startswith(b'#'):
                break
        w, h = [int(x) for x in line.split()]
        _maxval = int(f.readline().strip())
        data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape((h, w))
        return np.flipud(data)


def save_map_bundle(img, meta, map_name, set_default=False):
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    pgm_path = MAPS_DIR / f"{map_name}.pgm"
    yaml_path = MAPS_DIR / f"{map_name}.yaml"
    elev_path = MAPS_DIR / f"{map_name}_elevation.npz"

    pgm_img = np.flipud(img)
    h, w = pgm_img.shape
    with open(pgm_path, 'wb') as f:
        f.write(f"P5\n{w} {h}\n255\n".encode('ascii'))
        f.write(pgm_img.tobytes())

    yaml_text = (
        f"image: {map_name}.pgm\n"
        "mode: trinary\n"
        f"resolution: {meta['resolution']:.3f}\n"
        f"origin: [{meta['origin_x']:.3f}, {meta['origin_y']:.3f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(yaml_text)

    if 'elev_grid' in meta:
        np.savez_compressed(
            elev_path,
            elevation_grid=meta['elev_grid'],
            origin_x=meta['origin_x'],
            origin_y=meta['origin_y'],
            cell_size=meta['elev_cell_size'],
        )

    # Save visual PNG preview
    preview_path = MAPS_DIR / f"{map_name}_preview.png"
    cv2.imwrite(str(preview_path), img)

    print(f"[DONE] Saved map bundle '{map_name}':")
    print(f"  - PGM      : {pgm_path} ({w}x{h} px, res={meta['resolution']:.3f}m)")
    print(f"  - YAML     : {yaml_path} (origin=[{meta['origin_x']:.2f}, {meta['origin_y']:.2f}])")
    print(f"  - Preview  : {preview_path}")
    if 'elev_grid' in meta:
        print(f"  - Elevation: {elev_path}")

    if set_default and map_name != 'map_default':
        shutil.copyfile(pgm_path, MAPS_DIR / 'map_default.pgm')
        with open(MAPS_DIR / 'map_default.yaml', 'w', encoding='utf-8') as f:
            f.write(yaml_text.replace(f"image: {map_name}.pgm", "image: map_default.pgm"))
        if elev_path.exists():
            shutil.copyfile(elev_path, MAPS_DIR / 'map_default_elevation.npz')
        print("  - Default  : updated map_default.pgm / map_default.yaml")


def merge_cad_and_lidar(cad_img, cad_meta, scan_pgm_path, wall_band_m=0.18):
    scan_img = read_pgm(scan_pgm_path)
    if scan_img.shape != cad_img.shape:
        sh, sw = scan_img.shape
        ch, cw = cad_img.shape
        yi = np.clip((np.arange(ch) * sh / ch).astype(int), 0, sh - 1)
        xi = np.clip((np.arange(cw) * sw / cw).astype(int), 0, sw - 1)
        scan_img = scan_img[np.ix_(yi, xi)]

    walkable = cad_meta['walkable_mask']
    res = cad_meta['resolution']
    margin_px = max(2, int(round(wall_band_m / res)))

    inner_core = walkable.copy()
    for dy in range(-margin_px, margin_px + 1):
        for dx in range(-margin_px, margin_px + 1):
            if dx * dx + dy * dy <= margin_px * margin_px:
                shifted = np.roll(np.roll(walkable, dy, axis=0), dx, axis=1)
                inner_core &= shifted

    merged = cad_img.copy()
    wall_zone = walkable & ~inner_core
    merged[wall_zone & (scan_img < 50)] = 0
    merged[inner_core] = 254
    return merged


def main():
    parser = argparse.ArgumentParser(description='2026 RoboWorld Exact Map Builder')
    sub = parser.add_subparsers(dest='mode', help='Mode: ref or merge')

    p_ref = sub.add_parser('ref', help='Generate exact map from 3D reference CAD image')
    p_ref.add_argument('--name', type=str, default='roboworld_2026', help='Output map name')
    p_ref.add_argument('--res', type=float, default=0.03, help='Grid resolution (m/px)')
    p_ref.add_argument('--set-default', action='store_true', help='Also copy to map_default')

    p_merge = sub.add_parser('merge', help='Merge live LiDAR PGM with CAD corridor mask')
    p_merge.add_argument('--scan-pgm', type=str, required=True, help='Input LiDAR-scanned .pgm file')
    p_merge.add_argument('--name', type=str, default='roboworld_hybrid', help='Output map name')
    p_merge.add_argument('--res', type=float, default=0.03, help='Grid resolution (m/px)')
    p_merge.add_argument('--wall-band', type=float, default=0.18, help='Wall tolerance band (m)')
    p_merge.add_argument('--set-default', action='store_true', help='Also copy to map_default')

    args = parser.parse_args()
    if args.mode == 'ref' or args.mode is None:
        img, meta = build_from_ref_image(res=getattr(args, 'res', 0.03))
        name = getattr(args, 'name', 'roboworld_2026')
        set_def = getattr(args, 'set_default', True)
        save_map_bundle(img, meta, name, set_default=set_def)
    elif args.mode == 'merge':
        img, meta = build_from_ref_image(res=args.res)
        merged = merge_cad_and_lidar(img, meta, args.scan_pgm, wall_band_m=args.wall_band)
        save_map_bundle(merged, meta, args.name, set_default=args.set_default)


if __name__ == '__main__':
    main()
