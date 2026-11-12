"""summary*.json → 표 (RESULTS.md 용)."""
import json, sys
for tag in sys.argv[1:] or ['']:
    S = json.load(open(f'summary{("_" + tag) if tag else ""}.json'))
    print(f'\n## {tag or "기본"} shrink={S["shrink"]} flip={S["flip"]} 프레임 {S["n_frames"]} 검출 {S["n_det"]}')
    print('| 클래스 | 검출 | 3D | conf p10/50/90 | 박스안 깊이점 p10/50/90 | ±10cm 일치 p50 | 거리 m p10/50/90 | 지도 최근접 cm p10/50/90 | <5cm | <10cm | 군집(≥3) | 군집에 든 비율 |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|')
    for k, s in S['classes'].items():
        nn = s['map_nn_m_p10_50_90']
        print(f"| {k} | {s['n_det']} | {s['n_3d']} | {'/'.join(map(str, s['conf_p10_50_90']))} | {'/'.join(str(int(x)) for x in s['n_pts_p10_50_90'])} | {s['consist_p10_50_90'][1]} | {'/'.join(map(str, s['range_m_p10_50_90']))} | {'/'.join(str(round(x * 100, 1)) for x in nn)} | {s['map_nn_lt_5cm']} | {s['map_nn_lt_10cm']} | {s.get('n_clusters_ge3', 0)} | {s.get('clustered_frac', '-')} |")
    for k, s in S['classes'].items():
        if not s.get('clusters'): continue
        print(f'\n{k} 군집 (n≥3)')
        print('| n | 중심 x,y,z | 시간폭 s | 거리 min~max m | xy 퍼짐 중앙/p90/최대 cm | z std cm | 바닥z(깊이) | 바닥z(지도) | 지도최근접 cm |')
        print('|---|---|---|---|---|---|---|---|---|')
        for c in s['clusters']:
            print(f"| {c['n']} | {','.join(map(str, c['center']))} | {c['span_s']} | {c['range_min_max'][0]}~{c['range_min_max'][1]} | {c['xy_med_cm']}/{c['xy_p90_cm']}/{c['xy_max_cm']} | {c['z_std_cm']} | {c['ground_z_med']} | {c['map_floor_z_med']} | {c['map_nn_med_cm']} |")
