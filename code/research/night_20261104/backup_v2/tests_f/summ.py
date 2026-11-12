import json,sys
for line in open(sys.argv[1]):
    if not (line.startswith('①') or line.startswith('③')): continue
    nm=line[line.index('[')+1:line.index(']')]; d=json.loads(line[line.index('{'):])
    print('##', nm)
    for k,v in d.items():
        print(f"  {k:4s} 전환 {v['전환']} ({v['사유'][:28]}) FL오차@전환 {v.get('전환때_FL오차m')} 전환뒤백업 {v.get('전환뒤_백업오차')} 플래너최대 {v['플래너입력_최대오차m']} 정전 {v['정전s']} | 정렬 {v.get('정렬')} 고정 {v.get('고정s')} T오차 {v.get('T오차')} 보류 {v.get('보류경고')} | {v.get('정렬사유','')[:70]}")
