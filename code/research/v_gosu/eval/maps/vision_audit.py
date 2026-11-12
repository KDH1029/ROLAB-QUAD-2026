# gosu_20261030_001126 비전 자료 점검: 무엇을 봤나(밀착 인화), 흔들림, 밝기, 카메라 높이·숙임(뎁스 바닥 평면)
import numpy as np, rosbag2_py, json, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo
for f in ['NanumGothic','NanumBarunGothic','Noto Sans CJK KR']:
    if any(f in x.name for x in fm.fontManager.ttflist): plt.rcParams['font.family']=f; break
B='/home/ilovehan9283/bags/gosu_20261030_001126'; T0=1793286688.362388823
def rd(topics):
    r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=B,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r
def lapvar(g):
    l=g[1:-1,1:-1]*4-g[:-2,1:-1]-g[2:,1:-1]-g[1:-1,:-2]-g[1:-1,2:]; return float(l.var())
# 1) 컬러: 매 10장마다 흔들림(라플라시안 분산)·밝기, 20초마다 사진
r=rd(['/camera/color/image_raw']); k=0; sh=[]; br=[]; ts=[]; thumbs=[]; nxt=5
while r.has_next():
    tp,d,t=r.read_next(); k+=1; s=t*1e-9-T0
    if k%10 and s<nxt: continue
    m=deserialize_message(d,Image); a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.width,3)
    g=a[::2,::2].mean(axis=2).astype(np.float32); sh.append(lapvar(g)); br.append(float(g.mean())); ts.append(s)
    if s>=nxt: thumbs.append((s,a[::4,::4].copy())); nxt+=20
sh=np.array(sh); br=np.array(br); ts=np.array(ts)
# 2) 뎁스 바닥 평면 → 카메라 높이·숙임
ci=None; r=rd(['/camera/depth/camera_info'])
tp,d,t=r.read_next(); ci=deserialize_message(d,CameraInfo); fx,fy,cx,cy=ci.k[0],ci.k[4],ci.k[2],ci.k[5]
r=rd(['/camera/depth/image_rect_raw']); k=0; H=[]; P=[]
while r.has_next() and len(H)<60:
    tp,d,t=r.read_next(); k+=1
    if k%270: continue
    m=deserialize_message(d,Image); z=np.frombuffer(m.data,np.uint16).reshape(m.height,m.width).astype(np.float32)/1000
    v,u=np.mgrid[0:m.height:4,0:m.width:4]; z=z[::4,::4]; ok=(z>0.2)&(z<3.0)
    X=np.c_[(u[ok]-cx)*z[ok]/fx,(v[ok]-cy)*z[ok]/fy,z[ok]]
    X=X[X[:,1]>np.percentile(X[:,1],40)]   # 아래쪽(바닥 후보)
    best=None; rng=np.random.default_rng(k)
    for _ in range(150):
        s3=X[rng.choice(len(X),3,replace=False)]; n=np.cross(s3[1]-s3[0],s3[2]-s3[0]); nn=np.linalg.norm(n)
        if nn<1e-6: continue
        n/=nn; dd=-n@s3[0]; inl=np.abs(X@n+dd)<0.02
        if best is None or inl.sum()>best[0]: best=(inl.sum(),n,dd)
    if best is None or best[0]<0.3*len(X): continue
    n,dd=best[1],best[2]
    if dd<0: n,dd=-n,-dd
    H.append(dd); P.append(np.degrees(np.arcsin(abs(n[2]))))   # 광축(z)과 바닥 법선 사이 → 숙인 각
cam_h=float(np.median(H)); pitch=float(np.median(P))
vfov=2*np.degrees(np.arctan(ci.height/2/fy)); top=pitch-vfov/2
S=dict(컬러_표본=len(sh), 흔들림_라플라시안_중앙=round(float(np.median(sh)),1), 흐린장면비율_기준값의절반이하=round(float((sh<0.5*np.median(sh)).mean())*100,1),
       밝기_중앙=round(float(np.median(br)),1), 카메라높이_m=round(cam_h,3), 숙임_deg=round(pitch,1), 세로시야_deg=round(vfov,1),
       시야위끝_수평아래_deg=round(top,1), 평면표본=len(H))
# 위끝 시선이 수평 아래 top° → 높이 h 인 물체 꼭대기를 볼 수 있는 최소 거리 등
if top>0: S['바닥이_보이기_시작하는_먼거리_m']=round(cam_h/np.tan(np.radians(top)),2)
S['130cm_신호등_꼭대기를_보려면'] = '안 보임(시야 위끝이 수평 아래)' if top>0 else f'{(1.30-cam_h)/np.tan(np.radians(-top)):.1f} m 이상 떨어져야'
json.dump(S,open('vision_audit.json','w'),ensure_ascii=False,indent=1); print(json.dumps(S,ensure_ascii=False,indent=1))
# 밀착 인화
n=len(thumbs); cols=7; rows=int(np.ceil(n/cols))
fig,axs=plt.subplots(rows,cols,figsize=(cols*3.2,rows*2.1),dpi=110)
for ax in axs.ravel(): ax.axis('off')
for ax,(s,a) in zip(axs.ravel(),thumbs): ax.imshow(a); ax.set_title(f'{s:.0f}초',fontsize=9)
fig.suptitle(f'D435i 컬러 20초 간격 — 카메라 높이 {cam_h:.2f} m, 아래로 {pitch:.0f}° 숙임, 세로 시야 {vfov:.0f}° → 시야 위끝이 수평보다 {top:.0f}° 아래',fontsize=12)
fig.tight_layout(); fig.savefig('vision_contact_1030.png'); print('saved')
