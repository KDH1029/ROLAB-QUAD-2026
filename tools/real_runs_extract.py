import re, sys, datetime as dt, collections
KST=dt.timezone(dt.timedelta(hours=9))
pat=re.compile(r'^\[(\w+)\] \[(\d+)\.\d+\] \[real_course_runner\]: (.*)$')
def t(s): return dt.datetime.fromtimestamp(int(s),KST)
for f in sys.argv[1:]:
    runs=[]; cur=None
    for line in open(f,errors='replace'):
        m=pat.match(line.rstrip('\n'))
        if not m: continue
        lvl,ts,msg=m.groups(); ts=int(ts)
        if '러너 시작' in msg:
            cur={'start':ts,'goals':re.search(r'(\d+)개 목표',msg).group(1),'last':ts,'msgs':[],'cnt':collections.Counter(),'done':None}
            runs.append(cur); continue
        if cur is None: continue
        cur['last']=ts; cur['msgs'].append((ts,lvl,msg))
        for k,p in [('STOP','위치 감시 STOP('),('회전타임아웃','회전 타임아웃'),('물러남','물러남'),('넘어','넘어'),('막힘','막힘')]:
            if p in msg: cur['cnt'][k]+=1
        if '완주' in msg and cur['done'] is None: cur['done']=(ts,msg)
    print('=====',f)
    for r in runs:
        dur=r['last']-r['start']
        if dur<5 and not r['msgs']: 
            print(f"  {t(r['start']):%m-%d %H:%M:%S}  목표{r['goals']}  즉시 종료"); continue
        reached=[m for m in r['msgs'] if '도착' in m[2] or '통과' in m[2]]
        print(f"  {t(r['start']):%m-%d %H:%M:%S} → {t(r['last']):%H:%M:%S} ({dur}s)  목표{r['goals']}  {dict(r['cnt'])}")
        if r['done']: print(f"     완주: +{r['done'][0]-r['start']}s  {r['done'][1][:110]}")
        for ts,lvl,msg in r['msgs'][-3:]:
            print(f"     끝 +{ts-r['start']}s [{lvl}] {msg[:120]}")
