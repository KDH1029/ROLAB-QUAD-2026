#!/usr/bin/env python3
"""경기장에 관한 모든 것을 **한 파일**에 모은다 — ~/gosu/config/venue.json

  python3 venue.py show                  전부 보기 + 빠진 것 표시
  python3 venue.py check                 참조하는 파일이 실제로 있는지 검사
  python3 venue.py set-start 1.2,-4.8,8  초기위치 기록
  python3 venue.py set-start --from-robot 지금 로봇 자세를 초기위치로 기록
  python3 venue.py set-map <경로.pcd>    사전 매핑 데이터 등록 (gosu 안으로 복사한다)
  python3 venue.py import                waypoints.json / zones_box.json 을 요약해 넣는다
  python3 venue.py env                   쉘에서 쓸 환경변수로 출력

【왜 한 파일인가】
지금 경기장 정보가 여섯 군데에 흩어져 있다 — 지도 PCD, 2D 지도 yaml, 장애물 층,
웨이포인트 json, 구역 json, 그리고 초기위치는 아예 사람 머릿속에 있다.
어느 게 최신인지, 어느 지도로 만든 좌표인지를 아무도 모른다.
**이 파일 하나만 보면 다 나오게** 한다. 실제 데이터는 각자 자리에 두고, 여기는 가리킨다.

【절대 규칙 — 남의 영역에 쓰지 않는다】
읽기는 어디서든 하되, **쓰기는 ~/gosu 안에서만** 한다.
매핑 결과도 gosu 로 복사해 들여온다. `~/go2_ws`(팀원) 에는 한 바이트도 안 쓴다.
set-map 은 원본을 **복사**하지 옮기지 않는다.
"""
import argparse
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import time

GOSU = os.path.expanduser('~/gosu')
VENUE = os.path.join(GOSU, 'config', 'venue.json')
MAPS = os.path.join(GOSU, 'maps')

TEMPLATE = {
    'name': '2026 강남 AI 4족보행 로봇경진대회 (COEX)',
    'frame': 'map',
    'note': '경기장에 관한 단일 출처. 실제 파일은 각자 자리에 두고 여기서 가리킨다.',
    'map': {},
    'start_pose': {},
    'arena': {},
    'rules': {
        'green_seconds': 30.0,
        'cross_seconds': 12.0,
        'margin_seconds': 5.0,
        'crosswalk_min_feet': 1,
        'stopzone_min_feet': 3,
        'note': 'cross_seconds 는 실측해서 고칠 것. 지금은 임시값이다',
    },
    'waypoints_file': '~/gosu/config/waypoints.json',
    'zones_file': '~/gosu/config/zones_box.json',
}


def load():
    if os.path.exists(VENUE):
        d = json.load(open(VENUE))
        for k, v in TEMPLATE.items():          # 새 항목이 생겨도 깨지지 않게
            d.setdefault(k, v)
        return d
    return json.loads(json.dumps(TEMPLATE))


def save(d):
    os.makedirs(os.path.dirname(VENUE), exist_ok=True)
    d['updated_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
    json.dump(d, open(VENUE, 'w'), ensure_ascii=False, indent=1)


def guard_path(p):
    """gosu 밖에 쓰려 하면 막는다. 남의 작업환경을 오염시키지 않기 위해서다."""
    a = os.path.abspath(os.path.expanduser(p))
    if not a.startswith(os.path.abspath(GOSU)):
        raise SystemExit('★ gosu 밖에는 쓰지 않는다: %s\n'
                         '   남의 작업환경을 오염시키지 않는 것이 대원칙이다.' % a)
    return a


def md5(p, limit=64 << 20):
    h = hashlib.md5()
    with open(p, 'rb') as f:
        while True:
            b = f.read(1 << 20)
            if not b:
                break
            h.update(b)
            limit -= len(b)
            if limit <= 0:
                break
    return h.hexdigest()[:12]


def cmd_show(a, d):
    miss = []
    print('═══ %s ═══' % d.get('name', '?'))
    print('  파일      %s' % VENUE)
    print('  프레임    %s' % d.get('frame'))
    print('  갱신      %s' % d.get('updated_at', '(없음)'))
    print()
    print('── 사전 매핑 데이터 ──')
    m = d.get('map') or {}
    if not m:
        print('   ★ 없음 — venue.py set-map <경로.pcd>'); miss.append('map')
    for k, v in m.items():
        if isinstance(v, dict):
            ok = os.path.exists(os.path.expanduser(v.get('path', '')))
            print('   %-16s %s%s  %s' % (k, v.get('path'), '' if ok else '  ★ 파일 없음',
                                         '%s점' % v.get('points') if v.get('points') else ''))
            if not ok:
                miss.append(k)
        else:
            print('   %-16s %s' % (k, v))
    print()
    print('── 초기위치 ──')
    s = d.get('start_pose') or {}
    if s.get('x') is None:
        print('   ★ 없음 — venue.py set-start x,y,yaw   (매번 400초 탐색을 하게 된다)')
        miss.append('start_pose')
    else:
        print('   x %.3f  y %.3f  yaw %.1f도   (%s)' %
              (s['x'], s['y'], s['yaw_deg'], s.get('set_at', '')))
        if s.get('match_ratio') is not None:
            flag = '  ★ 낮다' if s['match_ratio'] < 0.7 else ''
            print('   저장 시 일치율 %.3f%s' % (s['match_ratio'], flag))
    print()
    print('── 좌표 ──')
    for label, key, need in (('웨이포인트', 'waypoints_file', 1), ('구역(박스)', 'zones_file', 4)):
        p = os.path.expanduser(d.get(key, ''))
        n = 0
        if os.path.exists(p):
            try:
                j = json.load(open(p))
                n = len([1 for v in j.values() if isinstance(v, dict)])
            except Exception:
                n = -1
        print('   %-12s %3s개   %s%s' % (label, n if n >= 0 else '?', p,
                                         '' if n >= need else '   ★ 부족'))
        if n < need:
            miss.append(key)
    print()
    print('── 규정 ──')
    r = d.get('rules') or {}
    print('   초록 %.0f초 지속, 건너는 데 %.0f초, 여유 %.0f초 -> 켜진 지 %.0f초 안에 출발'
          % (r.get('green_seconds', 0), r.get('cross_seconds', 0), r.get('margin_seconds', 0),
             r.get('green_seconds', 0) - r.get('cross_seconds', 0) - r.get('margin_seconds', 0)))
    print('   횡단보도 최소 %d발, 정지구역 최소 %d발'
          % (r.get('crosswalk_min_feet', 0), r.get('stopzone_min_feet', 0)))
    if r.get('note'):
        print('   ※ %s' % r['note'])
    print()
    if miss:
        print('★ 빠진 것 %d개: %s' % (len(miss), ', '.join(miss)))
    else:
        print('✔ 다 채워져 있다')
    return 1 if miss else 0


def cmd_set_start(a, d):
    if a.from_robot:
        # 자세를 읽는 방법을 두 벌 두지 않는다 — mission.py 의 것을 그대로 쓴다.
        # 거기엔 stderr 노출·재시도·무엇을 하라는 안내가 들어 있다.
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from mission import robot_pose
        x, y, yaw = robot_pose()          # yaw 는 도 단위
    else:
        if not a.value:
            raise SystemExit('★ "x,y,yaw" 또는 --from-robot 이 필요하다')
        x, y, yaw = (float(t) for t in a.value.split(','))
    d['start_pose'] = {'x': round(x, 4), 'y': round(y, 4), 'yaw_deg': round(yaw, 2),
                       'match_ratio': a.match_ratio,
                       'set_at': time.strftime('%Y-%m-%d %H:%M:%S')}
    save(d)
    print('초기위치 기록: x %.3f  y %.3f  yaw %.1f도' % (x, y, yaw))
    print('이제 이렇게 쓰면 400초 탐색을 건너뛴다:')
    print('  GO2_START_POSE="%.4f,%.4f,%.2f" ~/gosu/gosu.sh run' % (x, y, yaw))


def cmd_set_map(a, d):
    src = os.path.expanduser(a.value or '')
    if not os.path.isfile(src):
        raise SystemExit('★ 그런 파일이 없다: %s' % src)
    os.makedirs(MAPS, exist_ok=True)
    dst = guard_path(os.path.join(MAPS, os.path.basename(src)))
    if os.path.abspath(src) != dst:
        shutil.copy2(src, dst)                 # 원본은 그대로 둔다. 옮기지 않는다
        print('gosu 안으로 복사: %s -> %s' % (src, dst))
    n = None
    if dst.endswith('.pcd'):
        try:
            for line in open(dst, 'rb'):
                if line.startswith(b'POINTS'):
                    n = int(line.split()[1]); break
                if line.startswith(b'DATA'):
                    break
        except Exception:
            pass
    key = a.key or ('pcd_prior' if dst.endswith('.pcd') else 'file')
    d.setdefault('map', {})[key] = {
        'path': dst.replace(os.path.expanduser('~'), '~'),
        'points': n, 'md5': md5(dst), 'size': os.path.getsize(dst),
        'from': src.replace(os.path.expanduser('~'), '~'),
        'added_at': time.strftime('%Y-%m-%d %H:%M:%S'),
    }
    save(d)
    print('등록: map.%s  %s%s' % (key, dst, '  %d점' % n if n else ''))


def cmd_import(a, d):
    got = []
    for key, label in (('waypoints_file', '웨이포인트'), ('zones_file', '구역')):
        p = os.path.expanduser(d.get(key, ''))
        if os.path.exists(p):
            j = json.load(open(p))
            got.append('%s %d개' % (label, len([1 for v in j.values() if isinstance(v, dict)])))
    # 경기장 범위를 좌표에서 뽑는다
    xs, ys = [], []
    for key in ('waypoints_file', 'zones_file'):
        p = os.path.expanduser(d.get(key, ''))
        if not os.path.exists(p):
            continue
        for v in json.load(open(p)).values():
            if not isinstance(v, dict):
                continue
            if 'x' in v:
                xs.append(v['x']); ys.append(v['y'])
            for c in v.get('corners', []):
                xs.append(c[0]); ys.append(c[1])
    if xs:
        d['arena'] = {'xmin': round(min(xs) - 1, 2), 'xmax': round(max(xs) + 1, 2),
                      'ymin': round(min(ys) - 1, 2), 'ymax': round(max(ys) + 1, 2),
                      'note': '저장된 좌표에서 1 m 여유를 두고 뽑은 값'}
    save(d)
    print('들여옴: %s' % (', '.join(got) or '(없음)'))
    if xs:
        print('경기장 범위: x %.2f~%.2f  y %.2f~%.2f'
              % (d['arena']['xmin'], d['arena']['xmax'], d['arena']['ymin'], d['arena']['ymax']))


def cmd_env(a, d):
    s = d.get('start_pose') or {}
    if s.get('x') is not None:
        print('export GO2_START_POSE="%.4f,%.4f,%.2f"' % (s['x'], s['y'], s['yaw_deg']))
    m = (d.get('map') or {}).get('pcd_prior') or {}
    if m.get('path'):
        print('export GOSU_MAP="%s"' % m['path'])
    r = d.get('rules') or {}
    for k, e in (('green_seconds', 'GOSU_GREEN_S'), ('cross_seconds', 'GOSU_CROSS_S')):
        if r.get(k) is not None:
            print('export %s=%s' % (e, r[k]))


def cmd_check(a, d):
    bad = 0
    for key, v in (d.get('map') or {}).items():
        if not isinstance(v, dict):
            continue
        p = os.path.expanduser(v.get('path', ''))
        if not os.path.exists(p):
            print('★ 없음  map.%s  %s' % (key, p)); bad += 1
        elif v.get('md5') and md5(p) != v['md5']:
            print('★ 내용이 바뀜  map.%s  %s' % (key, p)); bad += 1
        else:
            print('✔ map.%s  %s' % (key, p))
    for key in ('waypoints_file', 'zones_file'):
        p = os.path.expanduser(d.get(key, ''))
        print('%s %s  %s' % ('✔' if os.path.exists(p) else '★ 없음', key, p))
        if not os.path.exists(p):
            bad += 1
    # gosu 밖을 가리키는 게 있으면 알린다
    for key, v in (d.get('map') or {}).items():
        if isinstance(v, dict):
            p = os.path.abspath(os.path.expanduser(v.get('path', '')))
            if not p.startswith(os.path.abspath(GOSU)):
                print('⚠ gosu 밖을 가리킨다: map.%s -> %s' % (key, p))
    return 1 if bad else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['show', 'check', 'set-start', 'set-map', 'import', 'env'])
    ap.add_argument('value', nargs='?', default='')
    ap.add_argument('--from-robot', action='store_true')
    ap.add_argument('--match-ratio', type=float, default=None)
    ap.add_argument('--key', default='', help='set-map: 어떤 이름으로 등록할지')
    a = ap.parse_args()
    d = load()
    if a.cmd == 'show':
        return cmd_show(a, d)
    if a.cmd == 'check':
        return cmd_check(a, d)
    if a.cmd == 'set-start':
        cmd_set_start(a, d)
    elif a.cmd == 'set-map':
        cmd_set_map(a, d)
    elif a.cmd == 'import':
        cmd_import(a, d)
    else:
        cmd_env(a, d)
    return 0


if __name__ == '__main__':
    sys.exit(main())
