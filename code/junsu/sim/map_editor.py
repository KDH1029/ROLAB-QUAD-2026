#!/usr/bin/env python3
"""시뮬레이션 벽 지도 편집기 — 마우스로 벽을 긋고 지운다. 저장하면 run_sim.sh 가 이 지도를 쓴다.
실행:  ~/venv/go2/bin/python3 ~/ws/junsu/sim/map_editor.py
배경  14:59 FAST-LIO 3D 지도를 위에서 본 높이(진할수록 높음 — 벽 60 cm 는 검게, 경사로·단은 회색) · 파랑 = 14:59 사람이 몬 경로 · 빨강 = 저장 좌표
벽    빨간 점 = 시뮬레이션 장애물(4 cm 칸)
조작
  왼쪽 드래그    벽 모드(w): 선 긋기  /  지우개 모드(e): 사각형 지우기
  오른쪽 드래그  언제나 사각형 지우기
  w / e          벽 모드 / 지우개 모드        t  선 굵기 4 cm ↔ 8 cm
  u              되돌리기(30번)               s  저장(sim_walls_edited.npy, 이전 것은 백업)
  r              자동 생성 지도로 되돌리기    b  배경 켜고 끄기      p  경로 켜고 끄기
  m              여러 겹 벽 묶기(반경 안의 겹을 한 줄로, wall_merge.py)   [ / ]  묶기 반경 −2 / +2 cm (기본 12 cm)
  q              끝내기(저장 안 했으면 묻지 않고 그냥 닫힘 — 저장은 s)
  확대·이동은 창 아래 도구 막대(돋보기·십자)를 쓰고, 다시 편집하려면 그 버튼을 한 번 더 눌러 끈다.
"""
import os, sys, json, time, shutil
import numpy as np

H = os.path.dirname(os.path.abspath(__file__))
AUTO = os.path.join(H, 'sim_walls.npy'); EDIT = os.path.join(H, 'sim_walls_edited.npy')
MAPNPZ = os.path.expanduser('~/ws/v_gosu/eval/out/arena1104b_map.npz'); BG = os.path.join(H, 'bg_map1459.npz')
WP = os.path.expanduser('~/ws/junsu/maps/real_waypoints_venue.json')
G = 0.04   # 벽 칸 크기


def load_cells(path):
    W = np.load(path)[:, :2]
    return set(map(tuple, np.floor(W / G + 1e-6).astype(int).tolist()))


def save_cells(cells, path):
    if os.path.exists(path):
        shutil.copy2(path, path.replace('.npy', time.strftime('.bak_%H%M%S.npy')))
    c = (np.array(sorted(cells), dtype=float) + 0.5) * G if cells else np.zeros((0, 2))
    W = np.concatenate([np.c_[c, np.full(len(c), z)] for z in (0.15, 0.35)]).astype(np.float32)
    np.save(path, W)
    return len(c)


def background():
    if os.path.exists(BG):
        d = np.load(BG); return d['img'], tuple(d['ext']), d['path']
    d = np.load(MAPNPZ); V = float(d['V']); P = (d['vox'][d['cnt'] >= 2] + 0.5) * V
    od = d['odom']; od = od[np.argsort(od[:, 1])]; path = od[:, 2:4]
    lo, hi = path.min(0) - 2.5, path.max(0) + 2.5
    P = P[(P[:, 0] > lo[0]) & (P[:, 0] < hi[0]) & (P[:, 1] > lo[1]) & (P[:, 1] < hi[1])]
    floor = np.percentile(od[:, 4], 50) - 0.31
    c = 0.05; k = np.floor((P[:, :2] - lo) / c).astype(int); Wd, Hd = np.ceil((hi - lo) / c).astype(int) + 1
    img = np.full((Wd, Hd), np.nan); np.fmax.at(img, (k[:, 0], k[:, 1]), np.clip(P[:, 2] - floor, -0.1, 1.0))
    ext = (lo[0], lo[0] + Wd * c, lo[1], lo[1] + Hd * c)
    np.savez_compressed(BG, img=img, ext=np.array(ext), path=path[::5])
    return img, ext, path[::5]


def line_cells(p0, p1, thick):
    n = max(2, int(np.hypot(p1[0] - p0[0], p1[1] - p0[1]) / (G / 2)) + 1)
    pts = np.c_[np.linspace(p0[0], p1[0], n), np.linspace(p0[1], p1[1], n)]
    out = set()
    r = 0 if thick == 1 else 1
    for x, y in pts:
        ix, iy = int(np.floor(x / G)), int(np.floor(y / G))
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                out.add((ix + dx, iy + dy))
    return out


def rect_cells(cells, p0, p1):
    x0, x1 = sorted((p0[0], p1[0])); y0, y1 = sorted((p0[1], p1[1]))
    return {c for c in cells if x0 <= (c[0] + 0.5) * G <= x1 and y0 <= (c[1] + 0.5) * G <= y1}


def main(selftest=False):
    import matplotlib
    if selftest:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    plt.rcParams['font.family'] = ['NanumGothic', 'DejaVu Sans']
    start = EDIT if os.path.exists(EDIT) else AUTO
    cells = load_cells(start); hist = []
    img, ext, path = background()
    wps = json.load(open(WP)) if os.path.exists(WP) else []
    fig, ax = plt.subplots(figsize=(13, 9))
    bg = ax.imshow(img.T, origin='lower', extent=ext, cmap='Greys', vmin=0, vmax=0.7, alpha=0.85, interpolation='nearest')
    pl, = ax.plot(path[:, 0], path[:, 1], '-', c='tab:blue', lw=0.8, alpha=0.7)
    for w in wps:
        ax.scatter(w['x'], w['y'], c='red', s=40, zorder=6); ax.annotate(w['label'], (w['x'], w['y']), xytext=(4, 4), textcoords='offset points', color='darkred', fontsize=9, zorder=6)
    sc = ax.scatter([], [], s=4, c='red', marker='s', zorder=5)
    preview, = ax.plot([], [], 'm-', lw=2, zorder=7)
    rect = Rectangle((0, 0), 0, 0, fill=False, ec='orange', lw=1.5, zorder=7); ax.add_patch(rect); rect.set_visible(False)
    st = {'mode': 'w', 'thick': 1, 'p0': None, 'btn': None, 'dirty': False, 'mr': 0.12}
    ax.set_xlim(ext[0], ext[1]); ax.set_ylim(ext[2], ext[3]); ax.set_aspect('equal'); ax.grid(alpha=0.25)
    ax.set_xticks(np.arange(np.floor(ext[0]), ext[1], 0.5), minor=True); ax.set_yticks(np.arange(np.floor(ext[2]), ext[3], 0.5), minor=True); ax.grid(which='minor', alpha=0.1)

    def redraw():
        c = (np.array(list(cells)) + 0.5) * G if cells else np.zeros((0, 2))
        sc.set_offsets(c)
        ax.set_title(f"{'벽 그리기' if st['mode'] == 'w' else '지우개'} 모드 · 굵기 {st['thick'] * 4} cm · 묶기 반경 {st['mr'] * 100:.0f} cm · 벽 칸 {len(cells)}"
                     f"{' · ★ 저장 안 함(s)' if st['dirty'] else ' · 저장됨'}   [w 벽 · e 지우개 · 오른쪽 드래그 지우기 · m 겹 묶기 · [ ] 반경 · u 되돌리기 · s 저장 · t 굵기 · b 배경 · p 경로]",
                     fontsize=10)
        fig.canvas.draw_idle()

    def busy():
        tb = getattr(fig.canvas, 'toolbar', None)
        return tb is not None and getattr(tb, 'mode', '') not in ('', None)

    def push():
        hist.append(set(cells))
        del hist[:-30]

    def on_press(e):
        if e.inaxes != ax or busy() or e.xdata is None:
            return
        st['p0'] = (e.xdata, e.ydata); st['btn'] = 'erase' if (e.button == 3 or st['mode'] == 'e') else 'wall'

    def on_move(e):
        if st['p0'] is None or e.inaxes != ax or e.xdata is None:
            return
        x0, y0 = st['p0']
        if st['btn'] == 'wall':
            preview.set_data([x0, e.xdata], [y0, e.ydata])
        else:
            rect.set_visible(True); rect.set_xy((min(x0, e.xdata), min(y0, e.ydata))); rect.set_width(abs(e.xdata - x0)); rect.set_height(abs(e.ydata - y0))
        fig.canvas.draw_idle()

    def on_release(e):
        if st['p0'] is None:
            return
        p1 = (e.xdata, e.ydata) if (e.inaxes == ax and e.xdata is not None) else st['p0']
        push()
        if st['btn'] == 'wall':
            cells.update(line_cells(st['p0'], p1, st['thick']))
        else:
            cells.difference_update(rect_cells(cells, st['p0'], p1))
        st['p0'] = None; st['dirty'] = True; preview.set_data([], []); rect.set_visible(False); redraw()

    def on_key(e):
        nonlocal cells
        k = e.key
        if k in ('w', 'e'):
            st['mode'] = k
        elif k == 't':
            st['thick'] = 2 if st['thick'] == 1 else 1
        elif k == 'u' and hist:
            cells = hist.pop(); st['dirty'] = True
        elif k == 's':
            n = save_cells(cells, EDIT); st['dirty'] = False; print(f'저장 {EDIT} (벽 칸 {n})', flush=True)
        elif k == 'r':
            push(); cells = load_cells(AUTO); st['dirty'] = True
        elif k == 'm':
            sys.path.insert(0, H); from wall_merge import merge_cells
            push(); n0 = len(cells); cells = merge_cells(cells, st['mr'], 2); st['dirty'] = True
            print(f'겹 묶기 r={st["mr"]:.2f} m: 칸 {n0} → {len(cells)}', flush=True)
        elif k == '[':
            st['mr'] = max(0.04, round(st['mr'] - 0.02, 2))
        elif k == ']':
            st['mr'] = min(0.40, round(st['mr'] + 0.02, 2))
        elif k == 'b':
            bg.set_visible(not bg.get_visible())
        elif k == 'p':
            pl.set_visible(not pl.get_visible())
        elif k == 'q':
            plt.close(fig); return
        redraw()

    for ev, fn in (('button_press_event', on_press), ('motion_notify_event', on_move), ('button_release_event', on_release), ('key_press_event', on_key)):
        fig.canvas.mpl_connect(ev, fn)
    for k in ('s', 'q', 'p', 'b'):   # 기본 단축키(저장 대화상자·닫기 등)와 겹치지 않게
        for name, keys in list(plt.rcParams.items()):
            if name.startswith('keymap.') and isinstance(keys, list) and k in keys:
                plt.rcParams[name] = [x for x in keys if x != k]
    redraw(); plt.tight_layout()
    if not selftest:
        print(f'편집 시작: {start} (벽 칸 {len(cells)}) — 창에서 s 로 저장', flush=True)
        plt.show(); return
    # ---- 자체 시험(창 없이): 선 긋기·지우기·되돌리기·저장이 맞는지
    n0 = len(cells)
    class Ev:  # 가짜 마우스 이벤트
        def __init__(s, x, y, b=1): s.xdata, s.ydata, s.button, s.inaxes, s.key = x, y, b, ax, None
    on_press(Ev(0.0, 6.5)); on_move(Ev(2.0, 6.5)); on_release(Ev(2.0, 6.5)); n1 = len(cells)
    on_press(Ev(0.5, 6.3, 3)); on_release(Ev(1.5, 6.7, 3)); n2 = len(cells)
    hist_len = len(hist); k = type('K', (), {'key': 'u'})(); on_key(k); n3 = len(cells)
    tmp = os.path.join(os.environ.get('TMPDIR', '/tmp'), 'map_editor_selftest.npy'); save_cells(cells, tmp); n4 = len(load_cells(tmp))
    print(json.dumps({'처음': n0, '2 m 선 그은 뒤': n1, '선 가운데 1 m 지운 뒤': n2, '되돌린 뒤': n3, '저장→다시 읽기': n4, '되돌리기 기록': hist_len}, ensure_ascii=False))
    assert n1 > n0 and n2 < n1 and n3 == n1 and n4 == n3, '자체 시험 실패'
    print('자체 시험 통과')


if __name__ == '__main__':
    main(selftest='--selftest' in sys.argv)
