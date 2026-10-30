#!/usr/bin/env python3
"""
2026 RoboWorld Quadruped Competition Map Generator & Hybrid LiDAR Map Merger.

Modes:
  1. cad    : Generates a noise-free 2D occupancy grid (.pgm/.yaml) and 2.5D elevation
              grid (_elevation.npz) from the official rulebook dimensions (8.0m x 6.0m).
  2. merge  : Combines a live LiDAR-scanned map (.pgm) with the CAD corridor mask,
              removing traffic cones (23cm) and dynamic clutter from walkable aisles
              while preserving real LiDAR wall measurements near track boundaries.

Usage:
  python3 scripts/build_hybrid_map.py cad --name roboworld_2026 --set-default
  python3 scripts/build_hybrid_map.py merge --scan-pgm maps/live_scan.pgm --name roboworld_hybrid
"""

import argparse
import math
import shutil
from pathlib import Path
import numpy as np

WS_ROOT = Path(__file__).resolve().parent.parent
MAPS_DIR = WS_ROOT / 'maps'


def point_in_poly(x, y, poly):
    """Vectorized ray-casting point-in-polygon test for numpy arrays x, y."""
    n = len(poly)
    inside = np.zeros(x.shape, dtype=bool)
    p1x, p1y = poly[0]
    for i in range(1, n + 1):
        p2x, p2y = poly[i % n]
        cond_y = (y > min(p1y, p2y)) & (y <= max(p1y, p2y))
        cond_x = x <= max(p1x, p2x)
        mask = cond_y & cond_x
        if p1y != p2y:
            xinters = (y[mask] - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
            if p1x == p2x:
                inside[mask] = ~inside[mask]
            else:
                flip = x[mask] <= xinters
                inside_sub = inside[mask]
                inside_sub[flip] = ~inside_sub[flip]
                inside[mask] = inside_sub
        p1x, p1y = p2x, p2y
    return inside


def dist_to_segment(px, py, x1, y1, x2, y2):
    """Vectorized distance from grid points (px, py) to line segment (x1,y1)-(x2,y2)."""
    dx = x2 - x1
    dy = y2 - y1
    l2 = dx * dx + dy * dy
    if l2 < 1e-12:
        return np.hypot(px - x1, py - y1)
    t = np.clip(((px - x1) * dx + (py - y1) * dy) / l2, 0.0, 1.0)
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return np.hypot(px - proj_x, py - proj_y)


def build_cad_track(res=0.03, pad_m=1.0):
    """
    Constructs the 2026 RoboWorld 8.0m x 6.0m track in a metric coordinate frame
    where (0.0, 0.0) is the bottom-left corner of the 8.0m x 6.0m arena.

    Key dimensions from Rulebook Section 4.1 & 4.2:
      - Total arena: 8.000m (X) x 6.000m (Y)
      - Left slalom corridor: X in [0.0, 1.0], Y in [0.5, 5.5] (5.018m straight, 1.4m top/bottom turns)
      - Central X-intersection: center at (3.60, 3.00), corridor width = 1.00m
        * Diagonal 1 (Ramp): bottom-left (1.0, 0.9) <-> center (3.6, 3.0) <-> top-right (5.8, 5.1)
        * Diagonal 2 (Stairs): top-left (1.0, 5.1) <-> center (3.6, 3.0) <-> bottom-right (5.8, 0.9)
      - Right IN corridor (top): X in [5.6, 8.0] (length 2.400m), Y in [4.982, 6.000] (width 1.018m)
      - Right OUT corridor (bottom): X in [5.6, 8.0] (length 2.400m), Y in [0.000, 1.018] (width 1.018m)
      - Parking bay (P): X in [6.5, 8.0] (1.500m), Y in [2.25, 3.75] (height 1.500m)
    """
    arena_w = 8.0
    arena_h = 6.0
    x0 = -pad_m
    y0 = -pad_m
    width = int(math.ceil((arena_w + 2.0 * pad_m) / res))
    height = int(math.ceil((arena_h + 2.0 * pad_m) / res))

    xs = x0 + (np.arange(width) + 0.5) * res
    ys = y0 + (np.arange(height) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)

    # Walkable free-space mask
    walkable = np.zeros((height, width), dtype=bool)

    # 1. Left Slalom straight corridor (X: 0.0..1.0, Y: 0.5..5.5) + top/bottom caps (width 1.4m)
    walkable |= (gx >= 0.0) & (gx <= 1.0) & (gy >= 0.5) & (gy <= 5.5)
    walkable |= (gx >= 0.0) & (gx <= 1.4) & (gy >= 4.6) & (gy <= 5.5)
    walkable |= (gx >= 0.0) & (gx <= 1.4) & (gy >= 0.5) & (gy <= 1.4)

    # 2. Central X-intersection corridors (width 1.0m -> half-width 0.50m)
    d_ramp = dist_to_segment(gx, gy, 1.0, 0.9, 5.8, 5.1)
    d_stair = dist_to_segment(gx, gy, 1.0, 5.1, 5.8, 0.9)
    walkable |= (d_ramp <= 0.50)
    walkable |= (d_stair <= 0.50)

    # 3. Top-right IN entrance (length 2.400m: X 5.6..8.0, width 1.018m: Y 4.982..6.0)
    #    plus vertical connector (X 5.6..6.6, Y 4.4..6.0)
    walkable |= (gx >= 5.6) & (gx <= 8.0) & (gy >= 4.982) & (gy <= 6.0)
    walkable |= (gx >= 5.5) & (gx <= 6.5) & (gy >= 4.4) & (gy <= 6.0)

    # 4. Bottom-right OUT exit (length 2.400m: X 5.6..8.0, width 1.018m: Y 0.0..1.018)
    #    plus vertical connector (X 5.5..6.5, Y 0.0..3.8) linking to Parking bay
    walkable |= (gx >= 5.6) & (gx <= 8.0) & (gy >= 0.0) & (gy <= 1.018)
    walkable |= (gx >= 5.5) & (gx <= 6.5) & (gy >= 0.0) & (gy <= 3.8)

    # 5. Right-middle Parking Bay 'P' (1.5m x 1.5m: X 6.5..8.0, Y 2.25..3.75)
    walkable |= (gx >= 6.4) & (gx <= 8.0) & (gy >= 2.25) & (gy <= 3.75)

    # Clip walkable area strictly inside the 8.0m x 6.0m outer boundary (except small margin)
    walkable &= (gx >= 0.0) & (gx <= arena_w) & (gy >= 0.0) & (gy <= arena_h)

    # Build Occupancy Grid Image: 205 = unknown, 254 = free, 0 = occupied wall
    img = np.full((height, width), 205, dtype=np.uint8)
    img[walkable] = 254

    # Extract wall boundary (2-pixel thick outline around walkable region)
    wall_mask = np.zeros((height, width), dtype=bool)
    for dy in (-2, -1, 0, 1, 2):
        for dx in (-2, -1, 0, 1, 2):
            if dx == 0 and dy == 0:
                continue
            shifted = np.roll(np.roll(walkable, dy, axis=0), dx, axis=1)
            wall_mask |= (shifted & ~walkable)
    img[wall_mask] = 0

    # Build 2.5D Elevation Grid (cell_size = 0.25m) modeling the central X-intersection hill/stairs
    cell_size = 0.25
    nx = int(math.ceil((arena_w + 2.0 * pad_m) / cell_size))
    ny = int(math.ceil((arena_h + 2.0 * pad_m) / cell_size))
    ex = x0 + (np.arange(nx) + 0.5) * cell_size
    ey = y0 + (np.arange(ny) + 0.5) * cell_size
    egx, egy = np.meshgrid(ex, ey)

    # Central intersection rises to +0.28m plateau at (3.6, 3.0)
    dist_center = np.hypot(egx - 3.6, egy - 3.0)
    elev_grid = np.clip(0.28 * np.exp(-0.5 * (dist_center / 1.1) ** 2), 0.0, 0.28).astype(np.float32)

    meta = {
        'width': width,
        'height': height,
        'resolution': res,
        'origin_x': x0,
        'origin_y': y0,
        'elev_grid': elev_grid,
        'elev_cell_size': cell_size,
        'walkable_mask': walkable,
    }
    return img, meta


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

    print(f"[DONE] Saved map bundle '{map_name}':")
    print(f"  - PGM      : {pgm_path} ({w}x{h} px, res={meta['resolution']:.3f}m)")
    print(f"  - YAML     : {yaml_path} (origin=[{meta['origin_x']:.2f}, {meta['origin_y']:.2f}])")
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
    """
    Cleans dynamic obstacles (23cm traffic cones, people) inside the walkable aisle core
    using the CAD free-space mask, while retaining real LiDAR wall measurements near boundaries.
    """
    scan_img = read_pgm(scan_pgm_path)
    if scan_img.shape != cad_img.shape:
        print(
            f"[WARN] Scan shape {scan_img.shape} differs from CAD shape {cad_img.shape}. "
            "Resampling scan to CAD canvas..."
        )
        sh, sw = scan_img.shape
        ch, cw = cad_img.shape
        yi = np.clip((np.arange(ch) * sh / ch).astype(int), 0, sh - 1)
        xi = np.clip((np.arange(cw) * sw / cw).astype(int), 0, sw - 1)
        scan_img = scan_img[np.ix_(yi, xi)]

    walkable = cad_meta['walkable_mask']
    res = cad_meta['resolution']
    margin_px = max(2, int(round(wall_band_m / res)))

    # Erode walkable mask by margin_px to get the "inner aisle core" where cones sit
    inner_core = walkable.copy()
    for dy in range(-margin_px, margin_px + 1):
        for dx in range(-margin_px, margin_px + 1):
            if dx * dx + dy * dy <= margin_px * margin_px:
                shifted = np.roll(np.roll(walkable, dy, axis=0), dx, axis=1)
                inner_core &= shifted

    merged = cad_img.copy()
    # Near walls (walkable & ~inner_core), trust real LiDAR occupied pixels (0)
    wall_zone = walkable & ~inner_core
    merged[wall_zone & (scan_img < 50)] = 0

    # Inside inner_core, force free space (254) to erase any scanned 23cm traffic cones
    merged[inner_core] = 254
    return merged


def main():
    parser = argparse.ArgumentParser(description='2026 RoboWorld CAD & Hybrid Map Builder')
    sub = parser.add_subparsers(dest='mode', help='Mode: cad or merge')

    p_cad = sub.add_parser('cad', help='Generate clean CAD map from rulebook dimensions')
    p_cad.add_argument('--name', type=str, default='roboworld_2026', help='Output map name')
    p_cad.add_argument('--res', type=float, default=0.03, help='Grid resolution (m/px)')
    p_cad.add_argument('--set-default', action='store_true', help='Also copy to map_default')

    p_merge = sub.add_parser('merge', help='Merge live LiDAR PGM with CAD corridor mask')
    p_merge.add_argument('--scan-pgm', type=str, required=True, help='Input LiDAR-scanned .pgm file')
    p_merge.add_argument('--name', type=str, default='roboworld_hybrid', help='Output map name')
    p_merge.add_argument('--res', type=float, default=0.03, help='Grid resolution (m/px)')
    p_merge.add_argument('--wall-band', type=float, default=0.18, help='Wall tolerance band (m)')
    p_merge.add_argument('--set-default', action='store_true', help='Also copy to map_default')

    args = parser.parse_args()
    if args.mode == 'cad':
        img, meta = build_cad_track(res=args.res)
        save_map_bundle(img, meta, args.name, set_default=args.set_default)
    elif args.mode == 'merge':
        cad_img, meta = build_cad_track(res=args.res)
        merged = merge_cad_and_lidar(cad_img, meta, args.scan_pgm, wall_band_m=args.wall_band)
        save_map_bundle(merged, meta, args.name, set_default=args.set_default)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
