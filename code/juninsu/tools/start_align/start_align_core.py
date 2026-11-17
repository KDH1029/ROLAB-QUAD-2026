"""Start alignment (no ROS): where is today's FAST-LIO origin on the mapping-day map?

FAST-LIO starts at (0, 0, 0) every run. The saved waypoints are in the mapping-day FAST-LIO frame (= the map).
The robot is put on the start mark by hand, so today's origin is a few cm / degrees off the mapping start
(11-04 arena4 bag: 13 cm, 3-4 deg -> walls 7 cm off at the median, 10 % scan match at worst).

align() takes the first seconds of scans while the robot stands on the start mark, each with its FAST-LIO
pose, and finds T (x, y, yaw) with  map_pose = T * lio_pose  by matching the scan points to the map walls:
coarse grid inside the plausible hand-placement range, then a local refine.

Gates (fallback = identity, i.e. what the stack did before):
  - the result must stay inside the search box (+-max_xy, +-max_yaw): the X arena has look-alike places,
    and on a bag that did not start on the mark the matcher found a 25.7 deg answer that scored as well
    as a right one - the score alone cannot tell, the box can
  - enough scan points on walls (match >= min_match within match_m)
"""
import math

import numpy as np
import yaml
from scipy import ndimage as ndi
from scipy.optimize import minimize


class WallMap:
    """Occupancy pgm/yaml -> distance (m) from any cell to the nearest wall."""

    def __init__(self, yaml_path):
        meta = yaml.safe_load(open(yaml_path))
        img_path = meta['image']
        if not img_path.startswith('/'):
            import os
            img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
        raw = open(img_path, 'rb').read().split(b'\n', 3)
        if raw[0] != b'P5':
            raise ValueError(f'{img_path}: only binary pgm (P5)')
        w, h = map(int, raw[1].split())
        img = np.frombuffer(raw[3][:w * h], np.uint8).reshape(h, w)[::-1]       # row 0 = origin y
        occ = (255 - img.astype(float)) / 255.0 > float(meta.get('occupied_thresh', 0.65))
        if int(meta.get('negate', 0)):
            occ = img.astype(float) / 255.0 > float(meta.get('occupied_thresh', 0.65))
        self.res = float(meta['resolution'])
        self.ox, self.oy = meta['origin'][:2]
        self.w, self.h = w, h
        self.dist = ndi.distance_transform_edt(~occ) * self.res

    def lookup(self, x, y):
        ix = ((x - self.ox) / self.res).astype(int)
        iy = ((y - self.oy) / self.res).astype(int)
        ok = (ix >= 0) & (iy >= 0) & (ix < self.w) & (iy < self.h)
        d = np.full(x.shape, 9.0)
        d[ok] = self.dist[iy[ok], ix[ok]]
        return d


def compose(T, pose):
    """T * pose for 2D poses (x, y, yaw)."""
    c, s = math.cos(T[2]), math.sin(T[2])
    return (c * pose[0] - s * pose[1] + T[0], s * pose[0] + c * pose[1] + T[1], pose[2] + T[2])


def inverse_point(T, x, y):
    """Map point -> today's FAST-LIO frame (T^-1 applied to a point)."""
    c, s = math.cos(T[2]), math.sin(T[2])
    dx, dy = x - T[0], y - T[1]
    return c * dx + s * dy, -s * dx + c * dy


def scan_points(ranges, angle_min, angle_inc, rmin=0.4, rmax=10.0):
    r = np.asarray(ranges, float)
    a = angle_min + angle_inc * np.arange(len(r))
    k = np.isfinite(r) & (r > rmin) & (r < rmax)
    return np.c_[r[k] * np.cos(a[k]), r[k] * np.sin(a[k])]


def align(wall_map, scans, max_xy=0.30, max_yaw_deg=10.0, match_m=0.10, min_match=0.70,
          clip_m=0.30, step_xy=0.05, step_yaw_deg=1.0, max_points=4000):
    """scans: list of (points Nx2 in the scan frame = base frame, (x, y, yaw) FAST-LIO pose of that scan).
    Returns dict(ok, T=(x, y, yaw), match, reason, ...). T is identity when not ok."""
    if not scans:
        return dict(ok=False, T=(0.0, 0.0, 0.0), match=0.0, reason='no scans')
    # every point once in the FAST-LIO frame (the robot stands, but take its pose anyway)
    P = []
    for pts, pose in scans:
        if len(pts):
            c, s = math.cos(pose[2]), math.sin(pose[2])
            P.append(np.c_[c * pts[:, 0] - s * pts[:, 1] + pose[0], s * pts[:, 0] + c * pts[:, 1] + pose[1]])
    if not P:
        return dict(ok=False, T=(0.0, 0.0, 0.0), match=0.0, reason='scans have no points')
    P = np.concatenate(P)
    if len(P) > max_points:
        P = P[np.random.default_rng(0).choice(len(P), max_points, replace=False)]

    def dists(T):
        c, s = math.cos(T[2]), math.sin(T[2])
        return wall_map.lookup(c * P[:, 0] - s * P[:, 1] + T[0], s * P[:, 0] + c * P[:, 1] + T[1])

    def cost(T):
        return float(np.mean(np.minimum(dists(T), clip_m)))

    my = math.radians(max_yaw_deg)
    xs = np.arange(-max_xy, max_xy + 1e-9, step_xy)
    yaws = np.radians(np.arange(-max_yaw_deg, max_yaw_deg + 1e-9, step_yaw_deg))
    grid = [(x, y, a) for x in xs for y in xs for a in yaws]
    costs = np.array([cost(T) for T in grid])
    fits = []
    for j in np.argsort(costs)[:5]:
        r = minimize(cost, grid[j], method='Powell', options=dict(xtol=1e-3, ftol=1e-6))
        fits.append((float(r.fun), tuple(float(v) for v in r.x)))
    fits.sort()

    def inside(T):
        return abs(T[0]) <= max_xy and abs(T[1]) <= max_xy and abs(T[2]) <= my

    fits_in = [f for f in fits if inside(f[1])]
    best_cost, T = fits_in[0] if fits_in else fits[0]
    match = float(np.mean(dists(T) <= match_m))
    out = dict(T=T, match=match, cost=best_cost, identity_match=float(np.mean(dists((0, 0, 0)) <= match_m)),
               points=len(P), scans=len(scans))
    # the refine may walk out of the box; a clearly better fit outside means the robot is not on the mark
    out_better = not inside(fits[0][1]) and fits[0][0] < 0.8 * best_cost
    if not fits_in or out_better:
        To = fits[0][1]
        out.update(ok=False, reason=f'best fit outside the start box (+-{max_xy} m, +-{max_yaw_deg} deg): '
                   f'{To[0]:+.2f} {To[1]:+.2f} m {math.degrees(To[2]):+.1f} deg - robot not on the start mark?')
    elif match < min_match:
        out.update(ok=False, reason=f'only {match * 100:.0f} % of scan points on walls (< {min_match * 100:.0f} %)')
    else:
        out.update(ok=True, reason='ok')
    if not out['ok']:
        out['T_rejected'], out['T'] = T, (0.0, 0.0, 0.0)
    return out
