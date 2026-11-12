#!/usr/bin/env python3
"""comp_specs()·loc_spec·align·러너 명령 안의 파일 경로가 다 있는지, 설정 파일 안 절대 경로가 있는지(ls 대신 os.path)."""
import os, re, sys, glob
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ctl_loader
g = ctl_loader.load()
J = g.J
print('J =', J)
miss = 0
def chk(p, why):
    global miss
    ok = os.path.exists(p)
    if not ok: miss += 1
    print(f"  {'OK ' if ok else '★없음'} {p}   ({why})")
    return ok
specs = g.comp_specs()
# planner junsu 판도
g.OPS['planner'] = 'junsu'; specs_j = {s[0]: s for s in g.comp_specs()}; g.OPS['planner'] = 'review'
allcmds = [(s[0], c) for s in specs for c in (s[1] or [])] + [('ego(junsu)', c) for c in specs_j['ego'][1]]
pat = re.compile(r"(?:(?<=[\s=:])|^)(/home/[^\s'\";|&]+|~/[^\s'\";|&]+)")
for name, c in allcmds:
    print(f'[{name}] {c[:160]}{"…" if len(c) > 160 else ""}')
    for p in pat.findall(c):
        if p.startswith('~/') and 'junsu_vision' in p:
            print(f'  (원격) {p} — 내부 PC 경로, NUC 에서 확인 불가'); continue
        chk(os.path.expanduser(p), name)
print('[runner/align]')
chk(os.path.join(J, 'src', 'real_course_runner.py'), 'runner')
chk(os.path.join(J, 'tools', 'start_align', 'go2_start_align.py'), 'align')
chk(os.path.join(J, 'tools', 'start_align', 'start_align_core.py'), 'align core')
chk(os.path.join(J, 'maps', 'align', 'arena1459_nocone_slalomfree.yaml'), 'align map')
chk(g.OPS['route'], 'OPS 기본 경로')
chk(os.path.join(J, 'maps', 'route_map_1106_hybrid.json'), '권장 경로')
chk(g.BRIDGE_PARAMS, 'BRIDGE_PARAMS'); chk(g.HESAI_CFG, 'HESAI_CFG'); chk(os.path.join(J, 'config', 'cyclonedds_junsu.xml'), 'DDS xml')
print('[yaw_trim]', g.yaw_trim_vals(), '← maps/segments/logs/yaw_trim.json', os.path.realpath(os.path.join(g.LOG, 'yaw_trim.json')))
# 설정 파일 안 절대 경로
cfgs = [g.BRIDGE_PARAMS, g.HESAI_CFG, f'{J}/config/fastlio_leg_nurigwan_derange30.yaml', f'{J}/config/map_server_junsu.yaml',
        f'{J}/config/amcl_junsu.yaml', f'{J}/config/cone_tracker_junsu.yaml', f'{J}/maps/align/arena1459_amcl_slalomfree.yaml',
        f'{J}/maps/align/arena1459_nocone_slalomfree.yaml', f'{J}/ws_ego_review/src/ego_planner/config/ego_planner_params.yaml']
for f in cfgs:
    if not os.path.exists(f):
        continue
    txt = open(f, errors='ignore').read()
    ps = sorted(set(re.findall(r"(/home/[^\s'\"#,]+|/opt/[^\s'\"#,]+|~/[^\s'\"#,]+)", txt)))
    rel = re.findall(r"^\s*(?:image|yaml_filename|map_file|correction_file_path|firetimes_path|pcap_path)\s*:\s*['\"]?([^\s'\"#]+)", txt, re.M)
    print(f'[설정] {f}: 절대경로 {len(ps)}개, 상대/파일 키 {rel}')
    for p in ps:
        chk(os.path.expanduser(p.rstrip('/')), os.path.basename(f))
    for r in rel:
        if not r.startswith('/') and not r.startswith('~'):
            chk(os.path.join(os.path.dirname(f), r), os.path.basename(f) + ' (상대)')
print('없음 합계', miss)
