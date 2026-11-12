# FAST-LIO2 대 FAST-LIVO2(라이다만, img_en 0) — 같은 bag·같은 팀 브리지·같은 평가
import numpy as np, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
for f in ['NanumGothic','NanumBarunGothic','Noto Sans CJK KR']:
    if any(f in x.name for x in fm.fontManager.ttflist): plt.rcParams['font.family']=f; break
plt.rcParams['axes.unicode_minus']=False
O='/home/ilovehan9283/ws/v_gosu/eval/out/'
def L(f):
    d=np.load(O+f); o=d['odom']; return o[:,0]-o[0,0], o[:,1:4], d['leg']
def plen(p): return float(np.sum(np.linalg.norm(np.diff(p,axis=0),axis=1)))
A_lio=[L(f) for f in ['base_fastlio.npz','base_r2.npz','base_r3.npz']]; A_vo=L('livo_lio_full.npz')
G_lio=L('gosu1030_fastlio.npz'); G_vo=L('gosu1030_livo_imgoff.npz')
CL,CV='#1f77b4','#d62728'
fig=plt.figure(figsize=(17,10.5),dpi=120); gs=fig.add_gridspec(2,3,width_ratios=[1,1.25,0.9])
# (a) arena xy
ax=fig.add_subplot(gs[0,0]); t,p,leg=A_lio[0]
ax.plot(p[:,0],p[:,1],color=CL,lw=0.8,label='FAST-LIO2'); t2,p2,_=A_vo
ax.plot(p2[:,0],p2[:,1],color=CV,lw=0.8,alpha=0.8,label='FAST-LIVO2 (라이다만)')
ax.plot(0,0,'o',mfc='w',mec='k'); ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(fontsize=9,loc='upper left')
ax.set_title('① arena_0921 (877초) 궤적 xy',fontsize=11); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
# (b) arena z
ax=fig.add_subplot(gs[0,1])
for i,(t,p,_) in enumerate(A_lio): ax.plot(t,p[:,2],color=CL,lw=0.8,alpha=0.9 if i==0 else 0.45,label='FAST-LIO2 (3회)' if i==0 else None)
ax.plot(t2,p2[:,2],color=CV,lw=0.9,label='FAST-LIVO2 (라이다만)')
ax.axhline(0,color='k',lw=0.5); ax.grid(alpha=.3); ax.legend(fontsize=9)
ax.set_title('② arena_0921 고도 z — 같은 바닥으로 돌아와야 0 근처',fontsize=11); ax.set_xlabel('재생 시각 (s)'); ax.set_ylabel('z (m)')
# (c) gosu1030 xy (발산 전까지)
ax=fig.add_subplot(gs[1,0]); t,p,_=G_lio; tv,pv,_=G_vo; k=int(np.argmax(np.linalg.norm(np.diff(pv,axis=0),axis=1)/np.maximum(np.diff(tv),1e-3)>3))
ax.plot(p[:,0],p[:,1],color=CL,lw=0.8,label='FAST-LIO2'); ax.plot(pv[:k,0],pv[:k,1],color=CV,lw=0.8,alpha=0.8,label=f'FAST-LIVO2 (라이다만, {tv[k]:.0f}초까지)')
ax.plot(0,0,'o',mfc='w',mec='k'); ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(fontsize=9,loc='lower left')
ax.set_xlim(-2,9); ax.set_ylim(-6.5,3)
ax.set_title('③ gosu_20261030 (542초) 궤적 xy',fontsize=11); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
# (d) gosu1030 출발점 거리 (로그)
ax=fig.add_subplot(gs[1,1]); r=np.linalg.norm(p[:,:2],axis=1); rv=np.linalg.norm(pv[:,:2],axis=1)
ax.semilogy(t,np.maximum(r,1e-2),color=CL,lw=0.8,label='FAST-LIO2'); ax.semilogy(tv,np.maximum(rv,1e-2),color=CV,lw=0.9,label='FAST-LIVO2 (라이다만)')
ax.axhline(7.5,color='gray',ls='--',lw=0.8); ax.text(5,8.5,'경기장 대각선 약 7.5 m',fontsize=9,color='gray')
j=np.argmax(np.linalg.norm(np.diff(pv,axis=0),axis=1)/np.maximum(np.diff(tv),1e-3)>3)
ax.annotate(f'{tv[j]:.0f}초 발산 → 끝 {rv[-1]/1000:.1f} km',(tv[j],max(rv[j],1)),xytext=(tv[j]-200,3e3),arrowprops=dict(arrowstyle='->',color=CV),color=CV,fontsize=10)
ax.grid(alpha=.3,which='both'); ax.legend(fontsize=9,loc='upper left'); ax.set_ylim(1e-2,3e5)
from matplotlib.ticker import FixedLocator, FixedFormatter
ax.yaxis.set_major_locator(FixedLocator([0.01,0.1,1,10,100,1e3,1e4,1e5])); ax.yaxis.set_major_formatter(FixedFormatter(['1 cm','10 cm','1 m','10 m','100 m','1 km','10 km','100 km']))
ax.set_title('④ gosu_20261030 출발점에서의 거리 (로그 눈금)',fontsize=11); ax.set_xlabel('재생 시각 (s)'); ax.set_ylabel('거리 (m)')
# (e) 지표 막대
ax=fig.add_subplot(gs[:,2]); ax.axis('off')
legA=plen(A_lio[0][2][:,1:4]); legG=plen(G_lio[2][:,1:4])
ra=[plen(x[1])/legA for x in A_lio]; rva=plen(A_vo[1])/legA; rg=plen(G_lio[1])/legG; rvg=plen(G_vo[1])/legG
zA=[np.ptp(x[1][:,2]) for x in A_lio]; zvA=np.ptp(A_vo[1][:,2])
rows=[['지표','FAST-LIO2','FAST-LIVO2\n(라이다만)'],
      ['arena_0921\n경로/다리오도',f'{min(ra):.3f}~{max(ra):.3f}',f'{rva:.2f}'],
      ['arena_0921\n재방문 바닥 σ 중앙','28~38 mm','115 mm'],
      ['arena_0921\n고도 z 범위',f'{min(zA):.2f}~{max(zA):.2f} m',f'{zvA:.2f} m'],
      ['gosu_20261030\n경로/다리오도',f'{rg:.3f}',f'발산\n({rvg:,.0f})'],
      ['gosu_20261030\n끝 위치 (출발=0)',f'{np.linalg.norm(G_lio[1][-1,:2]):.2f} m',f'{np.linalg.norm(G_vo[1][-1,:2])/1000:.1f} km'],
      ['CPU (1코어)','26%','측정 안 함']]
tb=ax.table(cellText=rows,loc='center',cellLoc='center',colWidths=[0.42,0.3,0.3]); tb.auto_set_font_size(False); tb.set_fontsize(10); tb.scale(1,3.1)
for (i,j),c in tb.get_celld().items():
    if i==0: c.set_facecolor('#eeeeee'); c.set_text_props(weight='bold')
    elif j==2: c.set_text_props(color=CV)
ax.set_title('⑤ 같은 bag · 같은 팀 브리지 · 같은 평가',fontsize=11)
fig.suptitle('FAST-LIO2 → FAST-LIVO2 교체 검증 (1단계: 카메라 없이 라이다·IMU만)  —  현재 설정에서 FAST-LIVO2 가 열세, 10-30 bag 에서는 발산',fontsize=13)
fig.tight_layout(rect=(0,0,1,0.96)); fig.savefig('lio_vs_livo_1030.png'); print('saved', dict(ra=ra,rva=rva,rg=rg,rvg=rvg,zA=zA,zvA=zvA,legA=legA,legG=legG))
