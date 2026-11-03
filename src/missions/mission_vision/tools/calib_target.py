#!/usr/bin/env python3
"""Full-screen chessboard on a laptop screen, used as the calibration target.

Runs anywhere with python3 + OpenCV (no ROS). Hold/prop the laptop where the robot's camera and
LiDAR both see the screen, then capture on the robot side with calib_capture.py.

    python3 calib_target.py                      # 8x6 squares (7x5 inner corners)
    python3 calib_target.py --cols 9 --rows 7    # other board

The physical square size depends on the screen, so it is MEASURED, not computed:
    1. Start this script; the board appears full screen.
    2. Measure the ruler bar under the board (exactly 4 squares long) with a ruler,
       divide by 4 -> square size in mm.
    3. Give that number to calib_capture.py --square-mm.
Keys: q / ESC quit, + / - change square size, i invert colours (helps with screen glare).
"""
import argparse

import cv2
import numpy as np


def render(w, h, cols, rows, sq, invert):
    img = np.full((h, w), 255, np.uint8)
    bw, bh = cols * sq, rows * sq
    x0, y0 = (w - bw) // 2, (h - bh) // 2 - sq // 2
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                img[y0 + r * sq:y0 + (r + 1) * sq, x0 + c * sq:x0 + (c + 1) * sq] = 0
    # Ruler bar: exactly 4 squares long, for measuring the physical square size
    ry = min(h - 20, y0 + bh + sq // 2)
    img[ry:ry + 8, x0:x0 + 4 * sq] = 0
    for k in range(5):
        img[ry - 10:ry + 18, x0 + k * sq - 1:x0 + k * sq + 1] = 0
    txt = f"{cols}x{rows} squares ({cols - 1}x{rows - 1} inner corners), {sq}px/square. Bar = 4 squares: measure it, /4 = --square-mm"
    cv2.putText(img, txt, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 0, 1, cv2.LINE_AA)
    return 255 - img if invert else img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cols', type=int, default=8, help='squares across')
    ap.add_argument('--rows', type=int, default=6, help='squares down')
    ap.add_argument('--square-px', type=int, default=0, help='square size in pixels (0: largest that fits)')
    a = ap.parse_args()

    win = 'calib_target'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    # Ask the window for its real size after going full screen
    cv2.imshow(win, np.zeros((10, 10), np.uint8))
    cv2.waitKey(200)
    try:
        _, _, w, h = cv2.getWindowImageRect(win)
    except Exception:
        w, h = 1920, 1080
    if w < 200 or h < 200:
        w, h = 1920, 1080
    sq = a.square_px or int(min(w / (a.cols + 1), (h - 120) / (a.rows + 1)))
    invert = False
    while True:
        cv2.imshow(win, render(w, h, a.cols, a.rows, sq, invert))
        k = cv2.waitKey(100) & 0xFF
        if k in (27, ord('q')):
            break
        if k in (ord('+'), ord('=')):
            sq += 4
        if k == ord('-'):
            sq = max(20, sq - 4)
        if k == ord('i'):
            invert = not invert
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
