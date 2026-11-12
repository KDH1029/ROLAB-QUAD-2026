#!/usr/bin/env python3
"""사진 묶음으로 신호등 판정을 시험한다. 로봇도 ROS 도 필요 없다.

  # 젯슨 (실제로 올라갈 INT8 엔진으로)
  python3 tools/test_traffic_light.py --weights ~/gosu/models/tl5_aug.engine \
      --images ~/tl5_int8/calib/images --hz 5
  # 노트북 (CPU, .pt — 느리다)
  ~/venv/go2/bin/python tools/test_traffic_light.py --weights ~/ws/go2_vision/tl5_aug.pt \
      --images ~/ws/go2_vision/ds_0911/images/train --limit 120

【무엇을 시험하나】
  ① YOLO 가 신호등을 실제로 찾는가, 빨강/초록을 맞게 가르는가
  ② 시간 필터가 한 프레임짜리 흔들림을 걸러내는가
  ③ 30초 규칙 — 빨강->초록 전환을 본 경우에만 "건너도 됨" 이 나오는가

판정 로직은 `src/traffic_light_logic.py` 를 그대로 쓴다.
**로봇에 올라가는 것과 같은 코드다.** 그래서 여기서 통과하면 노드도 같게 동작한다.

【사진 순서 = 시간】
파일 이름 순서를 시간 순서로 보고, --hz 로 프레임 간격을 준다.
그래서 실제 영상이 아니어도 시간 규칙을 시험할 수 있다.
"""
import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from traffic_light_logic import TrafficLightLogic, RED, GREEN, UNKNOWN


def main():
    ap = argparse.ArgumentParser()
    # 기본값은 젯슨 기준. 노트북에서는 --weights/--images 를 줘야 한다.
    ap.add_argument('--images', default=os.path.expanduser('~/tl5_int8/calib/images'))
    ap.add_argument('--weights', default=os.path.expanduser('~/gosu/models/tl5_aug.engine'))
    ap.add_argument('--imgsz', type=int, default=1280)
    ap.add_argument('--conf', type=float, default=0.35)
    ap.add_argument('--hz', type=float, default=5.0, help='사진 사이의 가상 간격 (초 = 1/hz)')
    ap.add_argument('--limit', type=int, default=0, help='앞에서 이만큼만 (0 이면 전부)')
    ap.add_argument('--min-box', type=float, default=14.0)
    ap.add_argument('--vote-of', type=int, default=5)
    ap.add_argument('--vote-need', type=int, default=3)
    ap.add_argument('--green-seconds', type=float, default=30.0)
    ap.add_argument('--cross-seconds', type=float, default=12.0)
    ap.add_argument('--margin', type=float, default=5.0)
    ap.add_argument('--labels', default='', help='정답 라벨 폴더 (없으면 이미지 폴더에서 추측)')
    ap.add_argument('--png', default='')
    a = ap.parse_args()

    files = sorted(glob.glob(os.path.join(os.path.expanduser(a.images), '*.jpg')) +
                   glob.glob(os.path.join(os.path.expanduser(a.images), '*.png')))
    if a.limit:
        files = files[:a.limit]
    if not files:
        raise SystemExit('★ 사진이 없다: %s' % a.images)

    lbl_dir = os.path.expanduser(a.labels) if a.labels else \
        os.path.expanduser(a.images).replace('/images/', '/labels/')
    have_truth = os.path.isdir(lbl_dir)

    from ultralytics import YOLO
    w = os.path.expanduser(a.weights)
    # .engine 은 스스로 task 를 모른다. 명시해야 경고 없이 뜬다.
    m = YOLO(w, task='detect') if w.endswith('.engine') else YOLO(w)
    names = m.names
    print('모델 %s  클래스 %s' % (os.path.basename(a.weights), list(names.values())))
    print('사진 %d장, 가상 간격 %.2f초 (%.0f Hz)' % (len(files), 1.0 / a.hz, a.hz))

    L = TrafficLightLogic(a.vote_of, a.vote_need, a.green_seconds, a.cross_seconds, a.margin)
    print(L.explain())
    print()

    dt = 1.0 / a.hz
    raw_hist, out_hist, ok_hist, t_hist = [], [], [], []
    truth_hist = []
    n_det = 0
    import time as _t
    ms = []
    for i, p in enumerate(files):
        now = i * dt
        _a = _t.perf_counter()
        r = m.predict(p, conf=a.conf, imgsz=a.imgsz, verbose=False)[0]
        ms.append((_t.perf_counter() - _a) * 1000)
        best = None
        for b in r.boxes:
            nm = names.get(int(b.cls), '')
            if nm not in ('tl_red', 'tl_green'):
                continue
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
            if (y2 - y1) < a.min_box:
                continue
            c = float(b.conf)
            if best is None or c > best[0]:
                best = (c, nm)
        raw = UNKNOWN if best is None else (RED if best[1] == 'tl_red' else GREEN)
        if best is not None:
            n_det += 1
        state, el, ok, note = L.update(raw, now)
        raw_hist.append(raw); out_hist.append(state); ok_hist.append(ok); t_hist.append(now)

        if have_truth:
            tp = os.path.join(lbl_dir, os.path.splitext(os.path.basename(p))[0] + '.txt')
            gt = UNKNOWN
            nlights = 0
            if os.path.exists(tp):
                cls = [int(l.split()[0]) for l in open(tp) if l.strip()]
                nlights = sum(1 for c in cls if c in (0, 1))
                if 0 in cls and 1 in cls:
                    gt = 'BOTH'
                elif 0 in cls:
                    gt = RED
                elif 1 in cls:
                    gt = GREEN
            truth_hist.append((gt, nlights))

    print('── 상태가 바뀐 지점 ──')
    if not L.changes:
        print('   (한 번도 안 바뀜)')
    for t, old, new, note in L.changes:
        print('   %6.1f초  %-7s -> %-7s  %s' % (t, old, new, note))

    print()
    print('── 요약 ──')
    tot = len(files)
    import numpy as _np
    print('   추론 시간 중앙 %.1f ms (%.1f Hz 까지 가능)' % (_np.median(ms), 1000.0 / _np.median(ms)))
    print('   신호등을 찾은 프레임        %d / %d (%.0f%%)' % (n_det, tot, 100.0 * n_det / tot))
    for s in (RED, GREEN, UNKNOWN):
        print('   걸러진 상태 %-8s       %4d (%.0f%%)'
              % (s, out_hist.count(s), 100.0 * out_hist.count(s) / tot))
    nok = sum(ok_hist)
    print('   "건너도 됨" 이 참인 프레임  %d (%.0f%%)' % (nok, 100.0 * nok / tot))
    flips_raw = sum(1 for i in range(1, tot) if raw_hist[i] != raw_hist[i - 1])
    flips_out = sum(1 for i in range(1, tot) if out_hist[i] != out_hist[i - 1])
    print('   원시 판정이 뒤집힌 횟수     %d' % flips_raw)
    print('   필터 뒤 뒤집힌 횟수         %d   <- 필터가 %d번을 걸렀다'
          % (flips_out, flips_raw - flips_out))

    if have_truth:
        # 신호등이 **한 개만** 보이는 프레임과 여러 개 보이는 프레임을 갈라서 본다.
        # 4거리라 빨강과 초록이 동시에 보이는 순간이 많고, 그때는 '가장 확신 높은 것' 이
        # 어느 쪽을 고를지 알 수 없다. 그건 검출 실력의 문제가 아니라 **고르는 규칙**의 문제다.
        one_ok = one_bad = multi_ok = multi_bad = both_n = 0
        for (gt, nl), out in zip(truth_hist, out_hist):
            if gt == 'BOTH':
                both_n += 1
                continue
            if gt not in (RED, GREEN) or out not in (RED, GREEN):
                continue
            if nl <= 1:
                if out == gt: one_ok += 1
                else: one_bad += 1
            else:
                if out == gt: multi_ok += 1
                else: multi_bad += 1
        print()
        print('── 정답 라벨과 대조 ──')
        print('   신호등이 1개만 보일 때   맞음 %3d   틀림 %3d   <- 검출 실력' % (one_ok, one_bad))
        print('   여러 개 보일 때          맞음 %3d   틀림 %3d   <- 고르는 규칙' % (multi_ok, multi_bad))
        print('   빨강+초록 동시           %3d 프레임 (판정 불가 — 어느 신호등인지 정해야 한다)'
              % both_n)

    if a.png:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        code = {RED: 2, GREEN: 1, UNKNOWN: 0, 'BOTH': 3}
        fig, ax = plt.subplots(3, 1, figsize=(14, 7), sharex=True)
        ax[0].step(t_hist, [code[v] for v in raw_hist], where='post', lw=0.8, color='0.5')
        ax[0].set_ylabel('raw'); ax[0].set_yticks([0, 1, 2]); ax[0].set_yticklabels(['?', 'G', 'R'])
        ax[1].step(t_hist, [code[v] for v in out_hist], where='post', lw=1.4, color='tab:blue')
        ax[1].set_ylabel('filtered'); ax[1].set_yticks([0, 1, 2]); ax[1].set_yticklabels(['?', 'G', 'R'])
        ax[2].step(t_hist, [1 if v else 0 for v in ok_hist], where='post', lw=1.4, color='tab:green')
        ax[2].set_ylabel('cross OK'); ax[2].set_yticks([0, 1]); ax[2].set_xlabel('virtual time [s]')
        for x in ax:
            x.grid(alpha=.25)
        plt.tight_layout(); plt.savefig(a.png, dpi=100)
        print('\n  그림: %s' % a.png)
    return 0


if __name__ == '__main__':
    sys.exit(main())
