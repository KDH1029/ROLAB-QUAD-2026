#!/usr/bin/env python3
"""구간 시험용 경로 — 출발점에서 N 번 점까지만 가고 거기서 끝(exit). 출발은 원래 start 그대로라 출발 정렬이 된다.
사용: python3 ~/ws/junk/tools/sub_route.py <이름> <끝 번호> [--stops]
  예) python3 ~/ws/junk/tools/sub_route.py rec1650 30      → routes/rec1650_to30 (출발 → 슬라럼 → #30 교차로에서 멈춤)
  --stops 가 없으면 중간 정지선·주차는 지나가는 점으로 바꿈(구간만 빨리 봄). 있으면 원래 동작 그대로.
  만든 경로는 원래 경로보다 '오래된' 파일로 둬서 관리자 기본 경로는 그대로다 → 관리자에서 s → 1 로 골라 쓰고, 끝나면 s → 1 로 원래 경로."""
import json
import os
import shutil
import sys

J = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    name, end = args[0], int(args[1]); stops = '--stops' in sys.argv
    src = os.path.join(J, 'routes', name); dst = os.path.join(J, 'routes', f'{name}_to{end}')
    d = json.load(open(os.path.join(src, 'route.json'))); r = d['route']
    if not 2 <= end <= len(r):
        sys.exit(f'끝 번호는 2~{len(r)}')
    out = [dict(e) for e in r[:end]]
    for e in out[1:-1]:
        if not stops and (e['label'].startswith('stopline') or e['label'] in ('parking', 'exit')):
            e['label'] = 'line'
    out[-1]['label'] = 'exit'
    os.makedirs(dst, exist_ok=True)
    for f in ('map.pgm', 'map.yaml'):                      # 출발 정렬용 지도(같은 것)
        shutil.copy2(os.path.join(src, f), os.path.join(dst, f))
    d.update(route=out, sub_of=f'{name} #1~#{end}', problems=[])
    p = os.path.join(dst, 'route.json'); json.dump(d, open(p, 'w'), ensure_ascii=False, indent=1)
    t = os.path.getmtime(os.path.join(src, 'route.json')) - 60
    os.utime(p, (t, t))                                     # 원래 경로가 계속 '가장 최근'(관리자 기본)
    labs = [(i + 1, e['label']) for i, e in enumerate(out) if not e['label'].startswith('line')]
    print(f'{p}: {len(out)}점 {labs} · 길이 약 {sum(((out[i + 1]["x"] - out[i]["x"]) ** 2 + (out[i + 1]["y"] - out[i]["y"]) ** 2) ** 0.5 for i in range(len(out) - 1)):.1f} m')


if __name__ == '__main__':
    main()
