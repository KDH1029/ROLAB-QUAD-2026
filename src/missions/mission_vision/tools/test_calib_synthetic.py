#!/usr/bin/env python3
"""Synthetic end-to-end test of the guided calibration (calib_core.CalibSession), no robot needed.

A virtual D435 colour camera and an XT16 (true extrinsic deliberately different from the initial
guess) look at a laptop screen held by a simulated user who follows the robot's guidance
(moves, then holds still with hand jitter). The screen shows the pattern the session requests.
A wall 1.5 m behind the laptop checks that the LiDAR picks the screen, not the wall.

    python3 test_calib_synthetic.py            # all scenarios
"""
import math
import sys

import cv2
import numpy as np

import calib_core as cc

rng = np.random.default_rng(7)
K = np.array([[600.0, 0, 424], [0, 600.0, 240], [0, 0, 1]])
D = np.zeros(5)
CAM_W, CAM_H = 848, 480
SCREEN_PX = (1920, 1080)
SCREEN_MM_TRUE = (344.0, 193.5)          # 0.179 mm/px


def rot(axis, deg):
    a = np.asarray(axis, float)
    a /= np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    t = math.radians(deg)
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k


def T(R, t):
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = R, t
    return M


BASE_T_LIDAR = T(cc.quat_to_R([0.0024, 0.0063, 0.7071, 0.7070]), [0.25, 0.0, 0.1588])
BASE_T_COLOR_GUESS = T(cc.quat_to_R([-0.6187, 0.6187, -0.3423, 0.3423]), [0.30, 0.0, 0.117])
# Truth: guess off by 2 deg pitch, 1.5 deg yaw, 3 cm forward, -2 cm up
TRUE_ERR = T(rot([0, 1, 0], 2.0) @ rot([0, 0, 1], 1.5), [0.03, 0.0, -0.02])
BASE_T_COLOR_TRUE = TRUE_ERR @ BASE_T_COLOR_GUESS
LIDAR_T_COLOR_TRUE = np.linalg.inv(BASE_T_LIDAR) @ BASE_T_COLOR_TRUE


def screen_pose(target, jitter):
    """Screen frame (x right, y down, z into the screen) in the camera frame for a guide target."""
    name, dist = target if target else ('front', 'far')
    d = {'near': 0.9, 'far': 1.5}.get(dist, 1.2) + rng.normal(0, 0.03)
    yaw = {'left': 25, 'right': -25}.get(name, 0) + rng.normal(0, 3)
    pitch = {'up': 25, 'down': -25}.get(name, 0) + rng.normal(0, 3)
    R = rot([0, 1, 0], yaw) @ rot([1, 0, 0], -pitch)
    t = np.array([rng.normal(0, 0.05), rng.normal(0, 0.04), d])
    return R, t


def jitter_pose(R, t, pos_m, ang_deg):
    return rot(rng.normal(size=3), rng.normal(0, ang_deg)) @ R, t + rng.normal(0, pos_m, 3)


def render_camera(screen_img, R, t):
    mm = SCREEN_MM_TRUE[0] / SCREEN_PX[0] / 1000.0
    S = np.array([[mm, 0, -SCREEN_PX[0] / 2 * mm], [0, mm, -SCREEN_PX[1] / 2 * mm], [0, 0, 1]])
    H = K @ np.c_[R[:, 0], R[:, 1], t] @ S
    img = cv2.warpPerspective(screen_img, H, (CAM_W, CAM_H), borderValue=90)
    img = cv2.GaussianBlur(img, (3, 3), 0.8)
    return np.clip(img.astype(float) + rng.normal(0, 3, img.shape), 0, 255).astype(np.uint8)


def lidar_scan(R, t):
    """XT16 rays toward the screen; screen rectangle, else a wall 1.5 m behind it. Lidar frame."""
    lT = LIDAR_T_COLOR_TRUE
    n_cam = R[:, 2]
    c_cam = t
    cT = np.linalg.inv(lT)                               # lidar -> camera
    o_cam = cT[:3, 3]
    wall_d = n_cam @ c_cam + 1.5
    c_l = lT[:3, :3] @ c_cam + lT[:3, 3]
    az0 = math.degrees(math.atan2(c_l[1], c_l[0]))
    pts = []
    hw, hh = SCREEN_MM_TRUE[0] / 2000, SCREEN_MM_TRUE[1] / 2000
    for el in np.arange(-15, 16, 2.0):
        for az in np.arange(az0 - 35, az0 + 35, 0.36):
            dl = np.array([math.cos(math.radians(el)) * math.cos(math.radians(az)),
                           math.cos(math.radians(el)) * math.sin(math.radians(az)), math.sin(math.radians(el))])
            dc = cT[:3, :3] @ dl
            den = n_cam @ dc
            if abs(den) < 1e-6:
                continue
            s = n_cam @ (c_cam - o_cam) / den
            hit = None
            if s > 0:
                p = o_cam + s * dc
                q = R.T @ (p - c_cam)
                if abs(q[0]) <= hw and abs(q[1]) <= hh:
                    hit = s
            if hit is None:
                s2 = (wall_d - n_cam @ o_cam) / den
                if s2 <= 0:
                    continue
                hit = s2
            pts.append(dl * (hit + rng.normal(0, 0.01)))
    return np.array(pts)


def run_session(screen_mm_reported, square_mm_override=None, max_frames=900):
    sess = cc.CalibSession(BASE_T_LIDAR, BASE_T_COLOR_GUESS, square_mm_override)
    laptop_pat = cc.fit_pattern({'id': 0, 'kind': 'charuco', 'cols': 9, 'rows': 6, 'square_px': 10 ** 4,
                                 'marker_ratio': 0.7}, *SCREEN_PX)
    screen = {'w_px': SCREEN_PX[0], 'h_px': SCREEN_PX[1], 'w_mm': screen_mm_reported[0], 'h_mm': screen_mm_reported[1]}
    target, R, t, hold = None, None, None, 0
    switches = 0
    for f in range(max_frames):
        tt = f / 15.0
        sess.on_hello({'type': 'hello', 'screen': screen, 'pattern': laptop_pat})
        want = sess.guide.next_target()
        if want != target or R is None:                      # user moves to the next pose
            target, hold = want, 0
            R, t = screen_pose(target, True)
        hold += 1
        Rj, tj = (jitter_pose(R, t, 0.02, 2.0) if hold < 6 else jitter_pose(R, t, 0.0005, 0.05))
        img, _ = cc.render_pattern(laptop_pat, *SCREEN_PX)
        gray = render_camera(img, Rj, tj)
        st = sess.stab
        cloud = lidar_scan(Rj, tj) if (hold >= 6) else None
        for msg in sess.step(tt, gray, K, D, cloud):
            if msg['type'] == 'pattern':
                laptop_pat = cc.fit_pattern({k: msg[k] for k in ('id', 'kind', 'cols', 'rows', 'square_px', 'marker_ratio')},
                                            *SCREEN_PX)
                switches += 1
            if msg['type'] == 'result':
                E = np.linalg.inv(LIDAR_T_COLOR_TRUE) @ sess.lidar_T_color
                rot_err = math.degrees(math.acos(np.clip((np.trace(E[:3, :3]) - 1) / 2, -1, 1)))
                k_true = (SCREEN_MM_TRUE[0] / SCREEN_PX[0]) / (sess.caps[0][4] * 1000 / laptop_pat['square_px'])
                return {'frames': f + 1, 'seconds': tt, 'captures': len(sess.caps), 'switches': switches,
                        'rot_err_deg': rot_err, 'trans_err_cm': np.linalg.norm(E[:3, 3]) * 100,
                        'k': msg['scale'], 'k_true': k_true, 'ok': msg['ok'], 'res': msg}
    return {'frames': max_frames, 'captures': len(sess.caps), 'failed': True, 'done': sorted(sess.guide.done)}


def run_baseline(ruler_err_mm=0.5):
    """Old method: ruler-measured square (+0.5 mm), clustered poses chosen by eye, scale fixed,
    captures while the hand still moves a little."""
    pat = cc.fit_pattern({'id': 0, 'kind': 'chess', 'cols': 8, 'rows': 6, 'square_px': 10 ** 4}, *SCREEN_PX)
    true_sq = SCREEN_MM_TRUE[0] / SCREEN_PX[0] * pat['square_px']
    sq_m = (true_sq + ruler_err_mm) / 1000
    caps = []
    for target in [('front', 'near'), ('left', 'any'), ('left', 'any'), ('up', 'any'), ('front', 'far')]:
        R, t = screen_pose(target, True)
        R = rot([0, 1, 0], -12) @ R if target[0] == 'left' else R          # by eye: tilts too small
        Rj, tj = jitter_pose(R, t, 0.004, 0.4)
        img, _ = cc.render_pattern(pat, *SCREEN_PX)
        det = cc.detect_board(render_camera(img, Rj, tj), pat, sq_m, K, D)
        Rj2, tj2 = jitter_pose(R, t, 0.004, 0.4)                       # LiDAR sees a slightly different pose
        if det is None:
            continue
        n_c, d_c = cc.board_plane(det[0], det[1])
        pl = cc.screen_plane_in_lidar(lidar_scan(Rj2, tj2), np.linalg.inv(BASE_T_LIDAR) @ BASE_T_COLOR_GUESS,
                                      det[0], det[1], pat, sq_m)
        if pl is not None:
            caps.append((n_c, d_c, pl[0], pl[1]))
    try:
        Tm, k, res = cc.solve(caps, scale_prior_sigma=1e-6)             # scale fixed to the ruler value
    except ValueError as e:
        return {'frames': 5, 'captures': len(caps), 'failed': True, 'done': [], 'reason': str(e)}
    E = np.linalg.inv(LIDAR_T_COLOR_TRUE) @ Tm
    return {'captures': len(caps), 'rot_err_deg': math.degrees(math.acos(np.clip((np.trace(E[:3, :3]) - 1) / 2, -1, 1))),
            'trans_err_cm': np.linalg.norm(E[:3, 3]) * 100, 'res': res}


def main():
    print(f"truth vs initial guess: 2.0 deg pitch, 1.5 deg yaw, 3 cm fwd, 2 cm down; screen {SCREEN_MM_TRUE[0]:.0f} mm wide")
    rows = []
    b = run_baseline()
    rows.append(('baseline: ruler +0.5 mm, poses by eye, no still check', b))
    for name, mm, ov in [('guided, screen size unknown (scale from LiDAR)', (0.0, 0.0), None),
                         ('guided, screen size reported +2 % wrong', (SCREEN_MM_TRUE[0] * 1.02, SCREEN_MM_TRUE[1] * 1.02), None),
                         ('guided, screen size correct', SCREEN_MM_TRUE, None)]:
        rows.append((name, run_session(mm, ov)))
    for name, r in rows:
        if r.get('failed'):
            print(f"{name:55s} FAILED after {r['frames']} frames, captures {r['captures']}, done {r['done']}"
                  + (f" ({r['reason']})" if r.get('reason') else ''))
            continue
        extra = (f" | {r['seconds']:.0f} s sim, {r['captures']} captures, {r['switches']} pattern switches, "
                 f"scale {r['k']:.3f} (true {r['k_true']:.3f})") if 'k' in r else f" | {r['captures']} captures"
        print(f"{name:55s} rot {r['rot_err_deg']:.2f} deg, trans {r['trans_err_cm']:.1f} cm{extra}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
