"""Shared logic for the laptop-screen camera/LiDAR calibration (no ROS imports).

Used by calib_target.py (laptop: render patterns, talk to the robot) and calib_capture.py
(robot: detect, guide, capture, solve), and by test_calib_synthetic.py.

Protocol (UDP, JSON, one message per datagram):
  laptop -> robot  {"type": "hello", "screen": {"w_px", "h_px", "w_mm", "h_mm"}, "pattern": PATTERN}
                   sent every 0.2 s; PATTERN is what is on screen right now (with its "id")
  robot  -> laptop {"type": "pattern", ...PATTERN fields...}        request a pattern
                   {"type": "guide", "text", "arrow", "progress": [done, total], "status"}
                   {"type": "result", "ok", "rot_res_deg", "dist_res_cm", "scale"}
PATTERN = {"id": int, "kind": "charuco"|"chess", "cols": squares across, "rows": squares down,
           "square_px": int, "marker_ratio": float (charuco)}
"""
import json
import math
import socket

import cv2
import numpy as np

LAPTOP_PORT = 47800
ROBOT_PORT = 47801
ARUCO_DICT = cv2.aruco.DICT_5X5_250


# ---------------------------------------------------------------- patterns (both versions of cv2.aruco)
def _dict():
    return cv2.aruco.getPredefinedDictionary(ARUCO_DICT)


def charuco_board(cols, rows, square_m, marker_ratio):
    if hasattr(cv2.aruco, 'CharucoBoard_create'):          # OpenCV <= 4.6
        return cv2.aruco.CharucoBoard_create(cols, rows, square_m, square_m * marker_ratio, _dict())
    return cv2.aruco.CharucoBoard((cols, rows), square_m, square_m * marker_ratio, _dict())


def fit_pattern(pat, w, h):
    """Clamp the square size so the board fits on a w x h screen with room for the guidance bands.
    The laptop applies this before drawing and reports the clamped pattern in its hello."""
    p = dict(pat)
    p['square_px'] = int(max(8, min(p['square_px'], w / (p['cols'] + 1), 0.8 * h / (p['rows'] + 1))))
    return p


def render_pattern(pat, w, h):
    """Pattern image (uint8, h x w) centred on a white screen, plus top-left pixel of the board."""
    img = np.full((h, w), 255, np.uint8)
    sq = int(pat['square_px'])
    bw, bh = pat['cols'] * sq, pat['rows'] * sq
    x0, y0 = (w - bw) // 2, (h - bh) // 2
    if pat['kind'] == 'charuco':
        b = charuco_board(pat['cols'], pat['rows'], 1.0, pat.get('marker_ratio', 0.7))
        board = b.generateImage((bw, bh)) if hasattr(b, 'generateImage') else b.draw((bw, bh))
        img[y0:y0 + bh, x0:x0 + bw] = board
    else:
        for r in range(pat['rows']):
            for c in range(pat['cols']):
                if (r + c) % 2 == 0:
                    img[y0 + r * sq:y0 + (r + 1) * sq, x0 + c * sq:x0 + (c + 1) * sq] = 0
    return img, (x0, y0)


def square_mm(pat, screen):
    """Physical square size from the reported screen size (None if unknown)."""
    if screen.get('w_mm', 0) > 50 and screen.get('w_px', 0) > 0:
        return pat['square_px'] * screen['w_mm'] / screen['w_px']
    return None


# ---------------------------------------------------------------- detection + PnP
def detect_board(gray, pat, sq_m, K, D):
    """Returns (R, t, n_corners, mean square size in px) of the board in the camera, or None.
    Board frame: origin at the first inner corner, z out of the screen."""
    if pat['kind'] == 'charuco':
        board = charuco_board(pat['cols'], pat['rows'], sq_m, pat.get('marker_ratio', 0.7))
        if hasattr(cv2.aruco, 'CharucoDetector'):
            det = cv2.aruco.CharucoDetector(board)
            cc, ci, _, _ = det.detectBoard(gray)
        else:
            mc, mi, _ = cv2.aruco.detectMarkers(gray, _dict())
            if mi is None or len(mi) < 2:
                return None
            _, cc, ci = cv2.aruco.interpolateCornersCharuco(mc, mi, gray, board)
        if ci is None or len(ci) < 6:
            return None
        obj = (board.getChessboardCorners() if hasattr(board, 'getChessboardCorners')
               else board.chessboardCorners)[ci.ravel()]
        img = cc.reshape(-1, 2)
    else:
        inner = (pat['cols'] - 1, pat['rows'] - 1)
        ok, c = cv2.findChessboardCornersSB(gray, inner, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            return None
        obj = np.zeros((inner[0] * inner[1], 3))
        obj[:, :2] = np.mgrid[0:inner[0], 0:inner[1]].T.reshape(-1, 2) * sq_m
        img = c.reshape(-1, 2)
    obj = np.asarray(obj, float).reshape(-1, 3)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    # apparent square size in px (for the dynamic pattern policy)
    proj = cv2.projectPoints(obj, rvec, tvec, K, D)[0].reshape(-1, 2)
    d = np.linalg.norm(np.diff(proj, axis=0), axis=1)
    sq_px = float(np.median(d[d < 3 * np.median(d)])) if len(d) else 0.0
    return cv2.Rodrigues(rvec)[0], tvec.ravel(), len(obj), sq_px


def board_plane(R, t):
    """Board plane in the camera: unit normal pointing away from the camera, distance."""
    n = R[:, 2]
    d = float(n @ t)
    return (-n, -d) if d < 0 else (n, d)


def fit_plane(P, thr=0.02, iters=300, min_pts=30):
    """RANSAC plane, normal away from the sensor origin: (n, d, inliers) or None."""
    if len(P) < min_pts:
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
    if best is None or best.sum() < min_pts:
        return None
    Q = P[best]
    c = Q.mean(0)
    n = np.linalg.svd(Q - c)[2][-1]
    d = float(n @ c)
    return (-n, -d, Q) if d < 0 else (n, d, Q)


# ---------------------------------------------------------------- guidance
# Poses that make the solve well conditioned: two distances and four tilts.
TARGETS = [
    ('front', 'near', 0, 0), ('front', 'far', 0, 0),
    ('left', 'any', -1, 0), ('right', 'any', 1, 0), ('up', 'any', 0, -1), ('down', 'any', 0, 1),
]
TILT_DEG = 20.0
NEAR_FAR_SPLIT_M = 1.2
GUIDE_TEXT = {
    ('front', 'near'): 'Face the camera, about 0.9 m away',
    ('front', 'far'): 'Face the camera, about 1.5 m away',
    ('left', 'any'): 'Turn the screen to face LEFT of the camera (~25 deg)',
    ('right', 'any'): 'Turn the screen to face RIGHT of the camera (~25 deg)',
    ('up', 'any'): 'Tilt the screen to face UP (~25 deg)',
    ('down', 'any'): 'Tilt the screen to face DOWN (~25 deg)',
}


def classify(n_c, d_c):
    """Which target a board pose satisfies. n_c: board normal in the camera (away from camera)."""
    yaw = math.degrees(math.atan2(n_c[0], n_c[2]))      # >0: normal points right -> screen faces left
    pitch = math.degrees(math.atan2(n_c[1], n_c[2]))    # >0: normal points down (image y)
    near = d_c < NEAR_FAR_SPLIT_M
    if abs(yaw) < 8 and abs(pitch) < 8:
        return ('front', 'near' if near else 'far')
    if abs(yaw) >= abs(pitch) and abs(yaw) > TILT_DEG:
        return ('left' if yaw > 0 else 'right', 'any')
    if abs(pitch) > TILT_DEG:
        return ('up' if pitch > 0 else 'down', 'any')
    return None


class Guide:
    def __init__(self):
        self.done = set()

    def next_target(self):
        for name, dist, _, _ in TARGETS:
            if (name, dist) not in self.done:
                return (name, dist)
        return None

    def message(self, current):
        nxt = self.next_target()
        if nxt is None:
            return {'type': 'guide', 'text': 'All poses captured - solving', 'arrow': 'none',
                    'progress': [len(self.done), len(TARGETS)], 'status': 'complete'}
        status = 'hold still' if current == nxt else 'move'
        return {'type': 'guide', 'text': GUIDE_TEXT[nxt], 'arrow': nxt[0] if nxt[0] != 'front' else nxt[1],
                'progress': [len(self.done), len(TARGETS)], 'status': status}


class Stability:
    """Board counts as still when its pose moved < pos_mm and < ang_deg over window_s."""

    def __init__(self, window_s=0.5, pos_mm=3.0, ang_deg=0.4):
        self.window_s, self.pos, self.ang = window_s, pos_mm / 1000.0, ang_deg
        self.hist = []

    def update(self, t, R, tvec):
        self.hist.append((t, R, tvec))
        self.hist = [h for h in self.hist if t - h[0] <= self.window_s + 1e-9]
        if self.hist[-1][0] - self.hist[0][0] < self.window_s * 0.9:
            return False
        _, R0, t0 = self.hist[0]
        for _, Ri, ti in self.hist[1:]:
            ang = math.degrees(math.acos(np.clip((np.trace(R0.T @ Ri) - 1) / 2, -1, 1)))
            if np.linalg.norm(ti - t0) > self.pos or ang > self.ang:
                return False
        return True

    def reset(self):
        self.hist = []


def pattern_policy(pat, sq_px_seen, next_id):
    """Bigger squares when they look small in the camera, smaller (more corners) when big."""
    new = dict(pat)
    if sq_px_seen and sq_px_seen < 14 and pat['cols'] > 5:
        new.update(cols=pat['cols'] - 2, rows=max(4, pat['rows'] - 1))
    elif sq_px_seen and sq_px_seen > 45 and pat['cols'] < 13:
        new.update(cols=pat['cols'] + 2, rows=pat['rows'] + 1)
    else:
        return None
    new['id'] = next_id
    return new


# ---------------------------------------------------------------- solve
def solve(caps, scale_prior_sigma=None):
    """caps: list of (n_c, d_c, n_l, d_l) with d_c from PnP at the assumed square size.
    Returns lidar_T_camera (4x4), scale k (true size = k * assumed), residuals.
    Rotation: normals (Kabsch). Translation + scale: n_l . t + k d_c = d_l (least squares);
    an optional prior k ~ 1 +- sigma (e.g. from the screen EDID size) is added as one more row."""
    Nc = np.array([c[0] for c in caps])
    Nl = np.array([c[2] for c in caps])
    U, _, Vt = np.linalg.svd(Nl.T @ Nc)
    R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    A = np.c_[Nl, [c[1] for c in caps]]
    b = np.array([c[3] for c in caps])
    if scale_prior_sigma:
        w = 0.01 / scale_prior_sigma            # 1 cm residual per sigma
        A = np.vstack([A, [0, 0, 0, w]])
        b = np.append(b, w)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    t, k = x[:3], x[3]
    Tm = np.eye(4)
    Tm[:3, :3], Tm[:3, 3] = R, t
    ang = [math.degrees(math.acos(np.clip(nl @ (R @ nc), -1, 1))) for nc, _, nl, _ in caps]
    res = Nl @ t + k * np.array([c[1] for c in caps]) - np.array([c[3] for c in caps])
    cond = np.linalg.svd(Nl, compute_uv=False)
    return Tm, float(k), {'rot_res_deg': float(np.mean(ang)), 'dist_res_cm': float(np.sqrt(np.mean(res ** 2)) * 100),
                          'normal_spread': float(cond[-1] / cond[0])}


# ---------------------------------------------------------------- UDP
class Link:
    def __init__(self, bind_port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', bind_port))
        self.sock.setblocking(False)

    def send(self, msg, addr):
        self.sock.sendto(json.dumps(msg).encode(), addr)

    def recv_all(self):
        out = []
        while True:
            try:
                data, addr = self.sock.recvfrom(65536)
            except BlockingIOError:
                return out
            try:
                out.append((json.loads(data.decode()), addr))
            except ValueError:
                pass


def board_center(pat, sq_m):
    """Board centre in the board frame used by detect_board."""
    if pat['kind'] == 'charuco':       # OpenCV charuco: origin at the outer board corner
        return np.array([pat['cols'] * sq_m / 2, pat['rows'] * sq_m / 2, 0.0])
    return np.array([(pat['cols'] - 2) * sq_m / 2, (pat['rows'] - 2) * sq_m / 2, 0.0])


def screen_plane_in_lidar(cloud_lidar, lidar_T_color, R, tv, pat, sq_m):
    """LiDAR plane of the laptop screen. Uses only the DIRECTION to the board (scale-free, the
    distance from PnP is off by the unknown scale), keeps the nearest surface in that cone so the
    wall behind the laptop is not picked, then RANSAC."""
    c_cam = R @ board_center(pat, sq_m) + tv
    u = c_cam / np.linalg.norm(c_cam)
    half_ang = math.atan2(0.5 * math.hypot(pat['cols'], pat['rows']) * sq_m, np.linalg.norm(c_cam))
    cT = np.linalg.inv(lidar_T_color)
    P = cloud_lidar @ cT[:3, :3].T + cT[:3, 3]           # lidar points in the camera frame
    r = np.linalg.norm(P, axis=1)
    ok = (r > 0.3) & (r < 4.0)
    P, r, idx = P[ok], r[ok], np.nonzero(ok)[0]
    cosang = (P @ u) / r
    cone = cosang > math.cos(0.8 * half_ang + math.radians(2))
    if cone.sum() < 30:
        return None
    rc = np.sort(r[cone])
    first = rc[min(len(rc) - 1, 10)]                      # robust nearest range
    sel = cone & (r < first + 0.30)
    return fit_plane(cloud_lidar[idx[sel]])


# ---------------------------------------------------------------- robot-side session (ROS-free)
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
    q[i], q[j], q[k] = s / 4, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
    return q


DEFAULT_MM_PER_PX = 0.25   # only a starting point when the screen size is unknown; the scale is solved


class CalibSession:
    """Detect -> guide -> auto-capture when still -> solve with scale. step() returns UDP messages."""

    def __init__(self, base_T_lidar, base_T_color_guess, square_mm_override=None):
        self.base_T_lidar = base_T_lidar
        self.lidar_T_color = np.linalg.inv(base_T_lidar) @ base_T_color_guess
        self.square_mm_override = square_mm_override
        self.guide = Guide()
        self.stab = Stability()
        self.caps = []               # (n_c, d_c, n_l, d_l, sq_m)
        self.screen = None
        self.shown = None            # pattern currently on the laptop (from hello)
        self.requested = None
        self.next_id = 1
        self.last_policy_t = -1e9
        self.result = None
        self.last = {}               # for the UI

    def sq_m(self, pat):
        if self.square_mm_override:
            return self.square_mm_override / 1000.0
        mm = square_mm(pat, self.screen or {})
        return (mm if mm else pat['square_px'] * DEFAULT_MM_PER_PX) / 1000.0

    def prior_sigma(self):
        if self.square_mm_override:
            return 0.005                          # ruler: about 0.5 %
        return 0.03 if (self.screen and self.screen.get('w_mm', 0) > 50) else None   # EDID: ~3 %

    def on_hello(self, msg):
        self.screen, self.shown = msg['screen'], msg['pattern']

    def step(self, t, gray, K, D, cloud_lidar):
        out = []
        self.last = {}
        if self.shown is None or self.result is not None:
            return out
        if self.requested and self.shown.get('id') != self.requested['id']:
            return [dict(self.requested, type='pattern')]          # still switching: re-send
        pat = self.shown
        sq_m = self.sq_m(pat)
        det = detect_board(gray, pat, sq_m, K, D)
        if det is None:
            self.stab.reset()
            out.append({'type': 'guide', 'text': 'Board not visible: point the screen at the robot camera',
                        'arrow': 'none', 'progress': [len(self.guide.done), len(TARGETS)], 'status': 'move'})
            return out
        R, tv, n_corners, sq_px = det
        n_c, d_c = board_plane(R, tv)
        current = classify(n_c, d_c)
        still = self.stab.update(t, R, tv)
        self.last = {'R': R, 't': tv, 'n_c': n_c, 'd_c': d_c, 'current': current, 'still': still,
                     'corners': n_corners, 'sq_px': sq_px}
        # adapt the pattern to how big the squares look (not while holding a pose)
        if not still and t - self.last_policy_t > 2.0:
            new = pattern_policy(pat, sq_px, self.next_id)
            if new:
                self.requested, self.next_id, self.last_policy_t = new, self.next_id + 1, t
                self.stab.reset()
                return [dict(new, type='pattern')]
        target = self.guide.next_target()
        if still and current == target and cloud_lidar is not None:
            pl = screen_plane_in_lidar(cloud_lidar, self.lidar_T_color, R, tv, pat, sq_m)
            self.last['lidar_pts'] = 0 if pl is None else len(pl[2])
            if pl is not None:
                self.caps.append((n_c, d_c, pl[0], pl[1], sq_m))
                self.guide.done.add(target)
                self.stab.reset()
        msg = self.guide.message(current)
        out.append(msg)
        if msg['status'] == 'complete':
            out.append(self.finish())
        return out

    def finish(self):
        # Every pattern used the same assumed mm-per-pixel, so one scale k fits all captures.
        caps = [(nc, dc, nl, dl) for nc, dc, nl, dl, _ in self.caps]
        Tm, k, res = solve(caps, self.prior_sigma())
        self.lidar_T_color = Tm
        mm_px = self.caps[0][4] * 1000 / self.shown['square_px'] if not self.square_mm_override else None
        ok = res['rot_res_deg'] < 0.5 and res['dist_res_cm'] < 1.5 and res['normal_spread'] > 0.2
        self.result = {'type': 'result', 'ok': bool(ok), 'scale': k,
                       'mm_per_px': (k * mm_px) if mm_px else None, **res}
        return self.result

    def base_T_color(self):
        return self.base_T_lidar @ self.lidar_T_color
