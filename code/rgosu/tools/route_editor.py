#!/usr/bin/env python3
"""주행 경로 편집기 — 고친 지도(11-04 14:59, FAST-LIO max_range 15, 2 cm) 위에서 경로 점과 순서를 정한다.
저장하면 maps/route_map.json(이전 것은 백업). 러너: python3 src/real_course_runner.py maps/route_map.json — 파일에 적힌 순서대로 간다.
실행:  ~/venv/go2/bin/python3 ~/ws/Rgosu/tools/route_editor.py [시작 파일=maps/route_map.json 있으면 그것, 없으면 real_waypoints_venue_run.json]
좌표 원점 = 14:59 녹화를 시작한 자리·방향(저장 좌표와 같음). ★ 대회 날 로봇을 같은 자리·같은 방향으로 놓을 것(1° 틀어지면 7 m 앞에서 12 cm).

점 종류(숫자 키 — 고른 점의 종류를 바꾸고, 새로 찍는 점의 종류도 이것이 된다)
  1 start   출발(첫 점, 하나)            2 line     지나가는 경유점(멈추지 않음)
  3 top     이 점으로 가는 동안 느린 걸음(STATICWALK) — 경사로·계단 오르내리는 구간 끝에 찍는다
  4 stopline 정지선: 멈춤 2.5초 + 신호등(녹색 1초 연속이면 출발) — 앞다리가 표식 안에 들어가는 자리
  5 rabain  콘 구간 시작              6 rabaout 콘 구간 끝
  7 parking 주차(앉기 5.5초)            8 exit    끝 — Out Line 을 몸 전체가 넘는 자리(마지막 점)
조작
  왼쪽 클릭(빈 곳)  고른 점 다음에 새 점 추가        왼쪽 드래그(점)  옮기기       오른쪽 클릭(점)  지우기
  n / b   다음 / 이전 점 고르기        [ / ]   고른 점을 순서에서 앞 / 뒤로
  c       고른 점을 차선 가운데로(0.35 m 안에서 벽에서 가장 먼 곳)     C  start·stopline·exit 빼고 전부 가운데로
  u  되돌리기     s  저장     v  14:59 경로 켜기/끄기     q  끝내기(저장은 s)
색: 점 테두리 — 벽까지 0.25 m 미만 빨강 · 0.35 m 미만 주황 · 그 이상 초록(로봇 반폭 0.155 m, ego 벽 여유 0.30 m)
"""
import os, sys, json, time, shutil, math
import numpy as np
from scipy.spatial import cKDTree

H = os.path.dirname(os.path.abspath(__file__)); R = os.path.dirname(H)
MAPNPZ = os.path.expanduser('~/ws/v_gosu/eval/out/fixed1459_2cm_map.npz')
CACHE = os.path.join(H, 'route_editor_cache.npz')
OUT = os.path.join(R, 'maps', 'route_map.json')
SEED = os.path.join(R, 'maps', 'real_waypoints_venue_run.json')
TYPES = {'1': 'start', '2': 'line', '3': 'top', '4': 'stopline', '5': 'rabain', '6': 'rabaout', '7': 'parking', '8': 'exit'}
NUMBERED = ('line', 'top', 'stopline')           # 여러 개 — 순서대로 번호(line1, line2 …)
# 한글 자판(두벌식)일 때 들어오는 글자 → 영문 키 — 한글 입력 상태에서도 s·c·u 등이 먹게
HANGUL = {'ㄴ': 's', 'ㅊ': 'c', 'ㅕ': 'u', 'ㅜ': 'n', 'ㅠ': 'b', 'ㅍ': 'v', 'ㅂ': 'q'}
AUTOSAVE = os.path.join(R, 'maps', 'route_map.autosave.json')
COL = {'start': 'white', 'line': 'deepskyblue', 'top': 'gold', 'stopline': 'red', 'rabain': 'orange', 'rabaout': 'darkorange',
       'parking': 'magenta', 'exit': 'lime'}


def kind(label):
    for k in ('stopline', 'top', 'line', 'rabain', 'rabaout', 'parking', 'exit', 'start'):
        if label.startswith(k): return k
    return 'line'


def build_cache():
    src = open(os.path.join(H, 'render_like_1030.py')).read()
    g = {'np': np, 'math': math}; exec(src[src.index('def revox'):src.index('def wall_band_thickness')], g)
    d = np.load(MAPNPZ); P = (d['vox'][d['cnt'] >= 2] + 0.5) * float(d['V']); od = d['odom']; od = od[np.argsort(od[:, 1])]; tr = od[:, 2:5]
    REG = (tr[:, 0].min() - 1, tr[:, 0].max() + 1, tr[:, 1].min() - 1, tr[:, 1].max() + 1)
    coef, tilt = g['fit_floor'](P, REG); h = P[:, 2] - (coef[0] * P[:, 0] + coef[1] * P[:, 1] + coef[2])
    ext = (math.floor(tr[:, 0].min() - 2.5), math.ceil(tr[:, 0].max() + 2.5), math.floor(tr[:, 1].min() - 2.5), math.ceil(tr[:, 1].max() + 2.5))
    k = (h > -0.3) & (h < 1.8); img = g['grid_max'](P[k, :2], h[k], 0.03, ext)
    # 벽 = 4 cm 칸 안에서 위아래로 0.25 m 넘게 서 있는 세로면(높이와 무관 — 경사로·단 윗면은 바닥, 벽 판넬은 벽)
    # 걸을 수 있는 곳 = 같은 칸의 높이 폭 0.08 m 미만인 수평면(바닥 위 0.6 m 아래)
    q = (h > -0.15) & (h < 1.5); Q, hq = P[q, :2], h[q]
    key = np.floor(Q / 0.04).astype(np.int64); u, inv = np.unique(key, axis=0, return_inverse=True); inv = inv.ravel()
    zmin = np.full(len(u), 9.0); zmax = np.full(len(u), -9.0); np.minimum.at(zmin, inv, hq); np.maximum.at(zmax, inv, hq)
    ctr = (u + 0.5) * 0.04; rng = zmax - zmin
    walls = ctr[rng >= 0.25]; floor = ctr[(rng < 0.08) & (zmax < 0.6)]
    np.savez_compressed(CACHE, img=img, ext=np.array(ext, float), walls=walls.astype(np.float32), floor=floor.astype(np.float32),
                        path=tr[:, :2].astype(np.float32), mtime=os.path.getmtime(MAPNPZ), tilt=tilt, ver=2)


def load_cache():
    if not os.path.exists(CACHE) or float(np.load(CACHE)['mtime']) != os.path.getmtime(MAPNPZ) or int(np.load(CACHE).get('ver', 1)) != 2:
        print('지도 준비 중(처음 한 번, 약 20초)…', flush=True); build_cache()
    return dict(np.load(CACHE))


def seed_route(path):
    d = json.load(open(path))
    if isinstance(d, dict) and 'route' in d:
        return [dict(label=e['label'], x=float(e['x']), y=float(e['y'])) for e in d['route']]
    # 예전 형식(로봇으로 저장한 좌표 목록) → 코스 순서로
    last = {e['label']: e for e in d}
    order = ['start', 'stopline1', 'rabain', 'rabaout', 'stopline2', 'line', 'parking', 'parkout', 'exit']
    return [dict(label=('line' if l in ('line', 'parkout') else l), x=float(last[l]['x']), y=float(last[l]['y'])) for l in order if l in last]


def relabel(route):
    cnt = {}
    for e in route:
        k = kind(e['label'])
        if k in NUMBERED:
            cnt[k] = cnt.get(k, 0) + 1; e['label'] = f'{k}{cnt[k]}'
        else:
            e['label'] = k
    return route


class Model:
    def __init__(self, cache, route):
        self.c = cache; self.T = cKDTree(cache['walls']); self.F = cache['floor']; self.TF = cKDTree(self.F)
        self.route = relabel(route); self.sel = len(self.route) - 1; self.hist = []

    def clear(self, x, y):
        return float(self.T.query([x, y])[0])

    def push(self):
        self.hist.append(json.dumps(self.route)); self.hist = self.hist[-50:]

    def undo(self):
        if self.hist: self.route = json.loads(self.hist.pop()); self.sel = min(self.sel, len(self.route) - 1)

    def snap(self, i, r=0.35, cap=0.5):
        e = self.route[i]; idx = self.TF.query_ball_point([e['x'], e['y']], r)
        if not idx: return None
        C = self.F[idx]; d = np.minimum(self.T.query(C)[0], cap); best = C[d >= d.max() - 0.02]
        q = best[np.argmin(np.hypot(*(best - [e['x'], e['y']]).T))]; old = (e['x'], e['y'])
        e['x'], e['y'] = round(float(q[0]), 3), round(float(q[1]), 3); return old, (e['x'], e['y']), self.clear(*q)

    def problems(self):
        p = []; ks = [kind(e['label']) for e in self.route]
        if not ks or ks[0] != 'start': p.append('첫 점이 start 가 아님')
        if not ks or ks[-1] != 'exit': p.append('마지막 점이 exit 가 아님')
        for k in ('start', 'parking', 'exit', 'rabain', 'rabaout'):
            if ks.count(k) > 1: p.append(f'{k} 가 {ks.count(k)}개')
        if ks.count('stopline') != 2: p.append(f'정지선 {ks.count("stopline")}개(규정 2곳)')
        if 'rabain' in ks and 'rabaout' in ks and ks.index('rabain') > ks.index('rabaout'): p.append('rabaout 이 rabain 보다 앞')
        for e in self.route:
            if kind(e['label']) != 'start' and self.clear(e['x'], e['y']) < 0.25: p.append(f"{e['label']} 벽 {self.clear(e['x'], e['y']):.2f} m")
        return p

    def save(self, path=OUT):
        if os.path.exists(path):
            shutil.copy2(path, path.replace('.json', time.strftime('.bak_%H%M%S.json')))
        out = {'format': 'route_v1', 'frame': 'FAST-LIO camera_init — 11-04 14:59 녹화 시작 자리·방향이 원점(저장 좌표와 같음)',
               'map': MAPNPZ, 'saved': time.strftime('%Y-%m-%d %H:%M:%S'), 'problems': self.problems(),
               'route': [dict(label=e['label'], x=round(e['x'], 3), y=round(e['y'], 3), clear_m=round(self.clear(e['x'], e['y']), 2)) for e in self.route]}
        json.dump(out, open(path, 'w'), ensure_ascii=False, indent=1); return path

    def autosave(self):
        try:
            json.dump({'format': 'route_v1', 'autosave': time.strftime('%H:%M:%S'), 'route': self.route}, open(AUTOSAVE, 'w'), ensure_ascii=False, indent=1)
        except Exception:
            pass


def gui(m):
    import matplotlib
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = ['NanumGothic', 'DejaVu Sans']; plt.rcParams['axes.unicode_minus'] = False
    for k in list(plt.rcParams):
        if k.startswith('keymap.') and k not in ('keymap.zoom', 'keymap.pan', 'keymap.home', 'keymap.back', 'keymap.forward'): plt.rcParams[k] = []
    fig, ax = plt.subplots(figsize=(14, 10)); c = m.c; ext = tuple(c['ext'])
    ax.imshow(c['img'], origin='lower', extent=ext, cmap='turbo', vmin=0, vmax=1.5, interpolation='nearest')
    pth, = ax.plot(c['path'][:, 0], c['path'][:, 1], '-', color='w', lw=0.5, alpha=0.6)
    ax.set_aspect('equal'); ax.set_xticks(np.arange(math.floor(ext[0]), ext[1] + 0.01, 0.5)); ax.set_yticks(np.arange(math.floor(ext[2]), ext[3] + 0.01, 0.5))
    ax.tick_params(labelsize=7); ax.grid(alpha=0.25, lw=0.5)
    st = {'type': 'line', 'drag': None, 'msg': ''}
    line, = ax.plot([], [], '-', color='w', lw=1.6, alpha=0.9); arts = []

    def draw():
        m.sel = max(0, min(m.sel, len(m.route) - 1)); m.autosave()
        for a in arts: a.remove()
        arts.clear(); xs = [e['x'] for e in m.route]; ys = [e['y'] for e in m.route]; line.set_data(xs, ys)
        for i, e in enumerate(m.route):
            k = kind(e['label']); cl = m.clear(e['x'], e['y']); ec = 'red' if cl < 0.25 else ('orange' if cl < 0.35 else 'lime')
            arts.append(ax.scatter(e['x'], e['y'], s=170 if i == m.sel else 90, c=COL[k], edgecolors=ec, linewidths=3 if i == m.sel else 2, zorder=5))
            arts.append(ax.annotate(f"{i + 1} {e['label']}\n벽 {cl:.2f}", (e['x'], e['y']), xytext=(7, 6), textcoords='offset points', fontsize=8.5,
                                    bbox=dict(fc='yellow' if i == m.sel else 'white', alpha=0.85, lw=0, pad=1), zorder=6))
        p = m.problems()
        ax.set_title(f"새 점 종류: {st['type']}  (1 start 2 line 3 top 4 stopline 5 rabain 6 rabaout 7 parking 8 exit · c 가운데로 · s 저장)\n"
                     + (('★ ' + ' · '.join(p[:4])) if p else '✔ 문제 없음') + (f"   |  {st['msg']}" if st['msg'] else ''), fontsize=10)
        fig.canvas.draw_idle()

    def hit(ev):
        if ev.xdata is None: return None
        P = ax.transData.transform([[e['x'], e['y']] for e in m.route]) if m.route else np.zeros((0, 2))
        if not len(P): return None
        d = np.hypot(P[:, 0] - ev.x, P[:, 1] - ev.y); i = int(np.argmin(d)); return i if d[i] < 12 else None

    def on_press(ev):
        if ev.inaxes != ax or fig.canvas.toolbar.mode: return
        m.sel = max(0, min(m.sel, len(m.route) - 1)); i = hit(ev)
        if ev.button == 3 and i is not None:
            m.push(); del m.route[i]; m.sel = max(0, min(m.sel, len(m.route) - 1)); relabel(m.route); st['msg'] = '지움'
        elif ev.button == 1 and i is not None:
            m.push(); m.sel = i; st['drag'] = i
        elif ev.button == 1:
            m.push(); m.route.insert(m.sel + 1, dict(label=st['type'], x=round(ev.xdata, 3), y=round(ev.ydata, 3))); m.sel += 1; relabel(m.route)
            st['msg'] = f"{m.sel + 1}번에 {st['type']} 추가"
        draw()

    def on_move(ev):
        if st['drag'] is None or ev.xdata is None: return
        e = m.route[st['drag']]; e['x'], e['y'] = round(ev.xdata, 3), round(ev.ydata, 3); draw()

    def on_release(ev):
        st['drag'] = None

    def on_key(ev):
        try:
            _on_key(ev)
        except Exception as e:   # 키 하나 잘못돼도 편집 내용은 그대로 — 제목에 오류를 보여 준다
            st['msg'] = f'오류: {e!r}'; print('키 처리 오류:', repr(e), flush=True); draw()

    def _on_key(ev):
        k = HANGUL.get(ev.key, ev.key)
        m.sel = max(0, min(m.sel, len(m.route) - 1))
        if k in TYPES:
            st['type'] = TYPES[k]
            if m.route: m.push(); m.route[m.sel]['label'] = TYPES[k]; relabel(m.route)
            st['msg'] = f'{m.sel + 1}번 → {TYPES[k]}'
        elif k == 'n': m.sel = min(m.sel + 1, len(m.route) - 1)
        elif k == 'b': m.sel = max(m.sel - 1, 0)
        elif k in ('[', ']') and m.route:
            j = m.sel + (-1 if k == '[' else 1)
            if 0 <= j < len(m.route): m.push(); m.route[m.sel], m.route[j] = m.route[j], m.route[m.sel]; m.sel = j; relabel(m.route)
        elif k == 'c' and m.route:
            m.push(); r = m.snap(m.sel); st['msg'] = (f'가운데로 {math.hypot(r[1][0] - r[0][0], r[1][1] - r[0][1]) * 100:.0f} cm 옮김 → 벽 {r[2]:.2f} m' if r else '근처에 바닥 없음')
        elif k == 'C':
            m.push(); n = 0
            for i, e in enumerate(m.route):
                if kind(e['label']) not in ('start', 'stopline', 'exit') and m.snap(i): n += 1
            st['msg'] = f'{n}개 가운데로'
        elif k == 'u': m.undo(); st['msg'] = '되돌림'
        elif k == 's': on_save(None); return
        elif k == 'v': pth.set_visible(not pth.get_visible())
        elif k == 'q': plt.close(fig); return
        draw()

    from matplotlib.widgets import Button
    bax = fig.add_axes([0.005, 0.005, 0.07, 0.04]); bsave = Button(bax, '저장(s)', color='lightgreen')

    def on_save(_):
        p = m.save(); st['msg'] = f'저장 {os.path.basename(p)}'; print('저장', p, '· 문제:', m.problems() or '없음', flush=True); draw()
    bsave.on_clicked(on_save)
    fig.canvas.mpl_connect('button_press_event', on_press); fig.canvas.mpl_connect('motion_notify_event', on_move)
    fig.canvas.mpl_connect('button_release_event', on_release); fig.canvas.mpl_connect('key_press_event', on_key)
    draw(); plt.tight_layout(); plt.show()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--selftest':
        import tempfile
        m = Model(load_cache(), seed_route(SEED)); print('시작 경로:', [(e['label'], e['x'], e['y'], round(m.clear(e['x'], e['y']), 2)) for e in m.route])
        i = [kind(e['label']) for e in m.route].index('parking'); r = m.snap(i); print('parking 가운데로:', r)
        m.route.insert(1, dict(label='top', x=2.0, y=1.2)); relabel(m.route); print('top 넣은 뒤 라벨:', [e['label'] for e in m.route])
        print('문제:', m.problems()); p = m.save(os.path.join(tempfile.gettempdir(), 'route_selftest.json')); print('저장', p); sys.exit()
    seed = sys.argv[1] if len(sys.argv) > 1 else (OUT if os.path.exists(OUT) else SEED)
    m = Model(load_cache(), seed_route(seed)); print('불러옴', seed, '·', len(m.route), '점', flush=True); gui(m)
