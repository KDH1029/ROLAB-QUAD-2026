#!/usr/bin/env python3
"""주행 경로 편집기 (junk) — make_map.sh 로 만든 지도(routes/<이름>) 위에서 경로 점·순서·각도를 정한다.
저장하면 routes/<이름>/route.json(이전 것은 .bak_시각). 러너는 파일에 적힌 순서대로 간다.
실행:  ~/venv/go2/bin/python3 ~/ws/junk/tools/route_editor.py <이름> [--new]    (--new = 빈 경로부터. 저장은 routes/<이름>/route.json, 예전 것은 .bak)
좌표 원점 = 그 녹화를 시작한 자리·방향. ★ 대회 날 로봇을 같은 자리·같은 방향으로 놓을 것(출발 정렬이 ±0.3 m·±10° 까지 맞춤).

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
  a       각도: 누른 뒤 바라볼 쪽을 클릭(분홍 화살표)   ← / →  5° 씩 돌리기   z  각도 지움
          각도 있는 점 = 거기서 멈춰 그 방향으로 제자리 회전 → 그 점 동작(정지선 대기·주차) 또는 다음 점. 각도 없는 점은 멈추지 않고 지나감
  d / 오른쪽 클릭  지우기     u  되돌리기     s  저장     v  녹화 길 켜기/끄기     q  끝내기(저장은 s)
색: 점 테두리 — 벽까지 0.25 m 미만 빨강 · 0.35 m 미만 주황 · 그 이상 초록(로봇 반폭 0.155 m, ego 벽 여유 0.30 m)
"""
import os, sys, json, time, shutil, math
import numpy as np
from scipy.spatial import cKDTree

H = os.path.dirname(os.path.abspath(__file__)); R = os.path.dirname(H)
# ★ junk: 이름 하나로 묶음 — routes/<이름>/map.npz(make_map.sh 결과) · editor_cache.npz · route.json · map.pgm/yaml(출발 정렬용)
NAME = next((a for a in sys.argv[1:] if not a.startswith('--')), None)
if NAME is None:
    print('사용: route_editor.py <이름>   (routes/<이름>/map.npz 가 있어야 함 — make_map.sh <bag> <이름>)'); sys.exit(1)
RD = os.path.join(R, 'routes', NAME)
MAPNPZ = os.path.join(RD, 'map.npz')
CACHE = os.path.join(RD, 'editor_cache.npz')
OUT = os.path.join(RD, 'route.json')
SEED = OUT
TYPES = {'1': 'start', '2': 'line', '3': 'top', '4': 'stopline', '5': 'rabain', '6': 'rabaout', '7': 'parking', '8': 'exit'}
NUMBERED = ('line', 'top', 'stopline')           # 여러 개 — 순서대로 번호(line1, line2 …)
# 한글 자판(두벌식)일 때 들어오는 글자 → 영문 키 — 한글 입력 상태에서도 s·c·u 등이 먹게
HANGUL = {'ㄴ': 's', 'ㅊ': 'c', 'ㅕ': 'u', 'ㅜ': 'n', 'ㅠ': 'b', 'ㅍ': 'v', 'ㅂ': 'q', 'ㅁ': 'a', 'ㅋ': 'z', 'ㅇ': 'd'}
AUTOSAVE = os.path.join(RD, 'route.autosave.json')
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


def write_pgm():
    """편집기와 같은 벽(세로 25 cm 넘는 면)으로 2D 지도 map.pgm/yaml — 출발 정렬(ICP)이 쓴다."""
    c = dict(np.load(CACHE)); Wl = real_walls(c); res = 0.03
    x0, y0 = Wl.min(0) - 1.0; x1, y1 = Wl.max(0) + 1.0
    w, h = int((x1 - x0) / res) + 1, int((y1 - y0) / res) + 1
    img = np.full((h, w), 254, np.uint8)
    ix = ((Wl[:, 0] - x0) / res).astype(int); iy = ((Wl[:, 1] - y0) / res).astype(int)
    for dx in (0, 1):                                  # 4 cm 칸 → 3 cm 칸에서 비지 않게
        for dy in (0, 1):
            img[np.clip(iy + dy, 0, h - 1), np.clip(ix + dx, 0, w - 1)] = 0
    img = img[::-1]
    with open(os.path.join(RD, 'map.pgm'), 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h)); f.write(img.tobytes())
    open(os.path.join(RD, 'map.yaml'), 'w').write(f'image: map.pgm\nresolution: {res}\norigin: [{x0:.3f}, {y0:.3f}, 0.0]\n'
                                                   'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n')


SELF_R = 0.18   # m — 녹화 때 로봇이 지나간 자리 이 안의 '벽' = 로봇 다리·따라 걷던 사람(진짜 벽은 몸 반폭 0.155 보다 멀다)


def real_walls(c):
    W = c['walls'].astype(float)
    return W[cKDTree(c['path'].astype(float)).query(W)[0] >= SELF_R]


def load_cache():
    if not os.path.exists(CACHE) or float(np.load(CACHE)['mtime']) != os.path.getmtime(MAPNPZ) or int(np.load(CACHE).get('ver', 1)) != 2:
        print('지도 준비 중(처음 한 번, 약 20초)…', flush=True); build_cache()
    return dict(np.load(CACHE))


def seed_route(path):
    d = json.load(open(path))
    if isinstance(d, dict) and 'route' in d:
        return [dict(label=e['label'], x=float(e['x']), y=float(e['y']), **({'yaw': float(e['yaw'])} if e.get('yaw') is not None else {}))
                for e in d['route']]
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
        self.c = cache; self.T = cKDTree(real_walls(cache)); self.F = cache['floor']; self.TF = cKDTree(self.F)   # junk: 로봇 몸·사람 점 뺀 벽
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
        out = {'format': 'route_v1', 'frame': 'FAST-LIO camera_init — 그 녹화(routes/<이름>)를 시작한 자리·방향이 원점', 'yaw': '도(°), 지도 x 축 기준 반시계 — 있으면 그 점에서 멈춰 그 방향으로 돎',
               'map': MAPNPZ, 'saved': time.strftime('%Y-%m-%d %H:%M:%S'), 'problems': self.problems(),
               'route': [dict(label=e['label'], x=round(e['x'], 3), y=round(e['y'], 3), clear_m=round(self.clear(e['x'], e['y']), 2),
                              **({'yaw': round(e['yaw'], 1)} if e.get('yaw') is not None else {})) for e in self.route]}
        try:    # ★ 11-06: 출구 선(out_line_x, 지도 좌표)은 편집기에 없음 → 예전 파일 값을 이어받음(없으면 러너 기본 −0.54 = rec1650 좌표)
            old = json.load(open(path))
            if 'out_line_x' in old: out['out_line_x'] = old['out_line_x']
        except (OSError, ValueError):
            pass
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
            if e.get('yaw') is not None:   # 바라볼 방향 화살표
                a_ = math.radians(e['yaw'])
                arts.append(ax.annotate('', xy=(e['x'] + 0.35 * math.cos(a_), e['y'] + 0.35 * math.sin(a_)), xytext=(e['x'], e['y']),
                                        arrowprops=dict(arrowstyle='-|>', color='magenta', lw=2.5), zorder=7))
            arts.append(ax.annotate(f"{i + 1} {e['label']}" + (f" ∠{e['yaw']:.0f}°" if e.get('yaw') is not None else '') + f"\n벽 {cl:.2f}",
                                    (e['x'], e['y']), xytext=(7, 6), textcoords='offset points', fontsize=8.5,
                                    bbox=dict(fc='yellow' if i == m.sel else 'white', alpha=0.85, lw=0, pad=1), zorder=6))
        p = m.problems()
        ax.set_title(f"새 점 종류: {st['type']}  (1 start 2 line 4 stopline 7 parking 8 exit · 끌기 옮기기 · 오른클릭/d 지우기 · n/b 다음/앞 · a 각도(클릭) ←→ 5° z 각도 지움 · u 되돌리기 · s 저장)\n"
                     + (('★ ' + ' · '.join(p[:4])) if p else '✔ 문제 없음') + (f"   |  {st['msg']}" if st['msg'] else ''), fontsize=10)
        fig.canvas.draw_idle()

    def hit(ev):
        if ev.xdata is None: return None
        P = ax.transData.transform([[e['x'], e['y']] for e in m.route]) if m.route else np.zeros((0, 2))
        if not len(P): return None
        d = np.hypot(P[:, 0] - ev.x, P[:, 1] - ev.y); i = int(np.argmin(d)); return i if d[i] < 12 else None

    def on_press(ev):
        if ev.inaxes != ax or getattr(fig.canvas.toolbar, 'mode', ''): return
        m.sel = max(0, min(m.sel, len(m.route) - 1)); i = hit(ev)
        if st.get('angle') and ev.button == 1 and m.route:      # a 다음 클릭 = 고른 점이 바라볼 쪽
            e = m.route[m.sel]; m.push(); e['yaw'] = round(math.degrees(math.atan2(ev.ydata - e['y'], ev.xdata - e['x'])), 1)
            st['angle'] = False; st['msg'] = f"{e['label']} 각도 {e['yaw']:.0f}°"; draw(); return
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
        elif k == 'a' and m.route:
            st['angle'] = True; st['msg'] = f"{m.route[m.sel]['label']} — 바라볼 쪽을 클릭"
        elif k in ('left', 'right') and m.route:
            e = m.route[m.sel]; m.push()
            if e.get('yaw') is None:                           # 처음이면 다음(없으면 앞) 점 쪽에서 시작
                j = m.sel + 1 if m.sel + 1 < len(m.route) else m.sel - 1
                e['yaw'] = math.degrees(math.atan2(m.route[j]['y'] - e['y'], m.route[j]['x'] - e['x'])) if j >= 0 else 0.0
            e['yaw'] = round(((e['yaw'] + (5 if k == 'left' else -5)) + 180) % 360 - 180, 1); st['msg'] = f"{e['label']} 각도 {e['yaw']:.0f}°"
        elif k == 'z' and m.route:
            m.push(); m.route[m.sel].pop('yaw', None); st['msg'] = '각도 지움'
        elif k in ('d', 'delete', 'backspace', 'ㅇ') and m.route:   # junk: 고른 점 지우기(연달아 누르면 이어서)
            m.push(); lab = m.route[m.sel]['label']; del m.route[m.sel]; m.sel = max(0, min(m.sel, len(m.route) - 1)); relabel(m.route)
            st['msg'] = f'{lab} 지움 — 지금 고른 점 {m.sel + 1}번'
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
    if '--selftest' in sys.argv:
        import tempfile
        m = Model(load_cache(), seed_route(SEED)); print('시작 경로:', [(e['label'], e['x'], e['y'], round(m.clear(e['x'], e['y']), 2)) for e in m.route])
        i = [kind(e['label']) for e in m.route].index('parking'); r = m.snap(i); print('parking 가운데로:', r)
        m.route.insert(1, dict(label='top', x=2.0, y=1.2)); relabel(m.route); print('top 넣은 뒤 라벨:', [e['label'] for e in m.route])
        print('문제:', m.problems()); p = m.save(os.path.join(tempfile.gettempdir(), 'route_selftest.json')); print('저장', p); sys.exit()
    cache = load_cache()
    if not os.path.exists(os.path.join(RD, 'map.pgm')) or '--build' in sys.argv:
        write_pgm(); print('2D 지도(출발 정렬용):', os.path.join(RD, 'map.pgm'), flush=True)
    if '--build' in sys.argv:
        sys.exit()
    if os.path.exists(OUT) and '--new' not in sys.argv:
        route = seed_route(OUT)
    else:                                              # 처음: 녹화 출발 자리 = start, 끝 자리 = exit
        p0, p1 = cache['path'][0], cache['path'][-1]
        route = [dict(label='start', x=float(p0[0]), y=float(p0[1]))]   # --new 또는 처음: 출발 자리만(저장할 때 예전 것은 .bak 로 남음)
    m = Model(cache, route); print('불러옴', NAME, '·', len(m.route), '점', flush=True); gui(m)
