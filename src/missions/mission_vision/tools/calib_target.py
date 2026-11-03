#!/usr/bin/env python3
"""Calibration target on a laptop screen, driven by the robot.

Runs anywhere with python3 + OpenCV (no ROS). Copy this file and calib_core.py to the laptop.

    python3 calib_target.py --robot 192.168.123.99        # talk to calib_capture.py on the robot
    python3 calib_target.py                               # standalone chessboard (manual mode)

With --robot the laptop reports its screen (pixels + physical size from xrandr, or --screen-mm)
every 0.2 s, shows whatever pattern the robot requests (ChArUco by default, square size adapted
to the distance), and shows the robot's guidance (which way to turn the screen, progress, result)
around the pattern. Hold the laptop so the camera AND the LiDAR see the screen.

The physical screen size often comes back wrong or empty; the robot solves the square size
from the LiDAR anyway. If you know it, pass --screen-mm "344 194" to give it a prior.
Keys: q / ESC quit, i invert (screen glare).
"""
import argparse
import math
import re
import subprocess
import time

import cv2
import numpy as np

import calib_core as cc

ARROWS = {'left': (-1, 0), 'right': (1, 0), 'up': (0, -1), 'down': (0, 1)}


def screen_mm():
    """Physical size of the primary screen from xrandr (EDID); (0, 0) if unknown."""
    try:
        out = subprocess.run(['xrandr'], capture_output=True, text=True, timeout=2).stdout
        for line in out.splitlines():
            if ' connected' in line:
                m = re.search(r'(\d+)mm x (\d+)mm', line)
                if m and int(m.group(1)) > 50:
                    return float(m.group(1)), float(m.group(2))
    except Exception:
        pass
    return 0.0, 0.0


def compose(w, h, pat, guide, result, invert):
    img, _ = cc.render_pattern(pat, w, h)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if invert:
        img = 255 - img
    band = max(40, h // 14)
    if guide:
        done, total = guide.get('progress', [0, 0])
        col = (0, 160, 0) if guide.get('status') == 'hold still' else (0, 0, 200)
        cv2.putText(img, f"[{done}/{total}] {guide.get('text', '')}", (20, int(band * 0.7)),
                    cv2.FONT_HERSHEY_SIMPLEX, band / 45, col, 2, cv2.LINE_AA)
        a = ARROWS.get(guide.get('arrow'))
        if a:
            cx, cy = w // 2, h - band // 2
            cv2.arrowedLine(img, (cx - a[0] * 80, cy - a[1] * 15), (cx + a[0] * 80, cy + a[1] * 15),
                            col, 6, tipLength=0.4)
    if result:
        ok = result.get('ok')
        cv2.putText(img, f"RESULT {'OK' if ok else 'RETRY'}: rot {result['rot_res_deg']:.2f} deg, "
                         f"dist {result['dist_res_cm']:.1f} cm, scale {result['scale']:.3f}",
                    (20, h - 15), cv2.FONT_HERSHEY_SIMPLEX, band / 55, (0, 140, 0) if ok else (0, 0, 220), 2)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robot', default='', help='robot PC IP (calib_capture.py); empty = standalone')
    ap.add_argument('--screen-mm', default='', help='"width height" of the visible screen area in mm')
    ap.add_argument('--headless', default='', help='"WxH": no window (tests)')
    ap.add_argument('--kind', default='charuco', choices=['charuco', 'chess'])
    a = ap.parse_args()

    if a.headless:
        w, h = [int(v) for v in a.headless.split('x')]
    else:
        win = 'calib_target'
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow(win, np.zeros((10, 10), np.uint8))
        cv2.waitKey(200)
        try:
            _, _, w, h = cv2.getWindowImageRect(win)
        except Exception:
            w, h = 1920, 1080
        if w < 200 or h < 200:
            w, h = 1920, 1080
    wmm, hmm = ([float(v) for v in a.screen_mm.split()] if a.screen_mm else screen_mm())
    screen = {'w_px': w, 'h_px': h, 'w_mm': wmm, 'h_mm': hmm}

    cols, rows = 9, 6
    pat = cc.fit_pattern({'id': 0, 'kind': a.kind, 'cols': cols, 'rows': rows, 'square_px': 10 ** 4,
                          'marker_ratio': 0.7}, w, h)
    link = cc.Link(cc.LAPTOP_PORT) if a.robot else None
    guide = result = None
    invert = False
    last_hello = 0.0
    print(f"screen {w}x{h} px, {wmm:.0f}x{hmm:.0f} mm ({'unknown' if wmm <= 50 else 'from xrandr/arg'})")

    while True:
        if link:
            now = time.time()
            if now - last_hello > 0.2:
                link.send({'type': 'hello', 'screen': screen, 'pattern': pat}, (a.robot, cc.ROBOT_PORT))
                last_hello = now
            for msg, _ in link.recv_all():
                if msg.get('type') == 'pattern':
                    pat = cc.fit_pattern({k: msg[k] for k in ('id', 'kind', 'cols', 'rows', 'square_px', 'marker_ratio')
                                          if k in msg}, w, h)
                elif msg.get('type') == 'guide':
                    guide = msg
                elif msg.get('type') == 'result':
                    result = msg
        img = compose(w, h, pat, guide, result, invert)
        if a.headless:
            time.sleep(0.05)
            continue
        cv2.imshow(win, img)
        k = cv2.waitKey(50) & 0xFF
        if k in (27, ord('q')):
            break
        if k == ord('i'):
            invert = not invert
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
