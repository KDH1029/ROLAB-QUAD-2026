# gosu_20261030_001126 의 D435i 표본 프레임 (컬러·뎁스·적외선 좌우) — 같은 순간 4장씩, 3개 시점
import sys, numpy as np, rosbag2_py, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
for f in ['NanumGothic','NanumBarunGothic','Noto Sans CJK KR']:
    if any(f in x.name for x in fm.fontManager.ttflist): plt.rcParams['font.family']=f; break
B='/home/ilovehan9283/bags/gosu_20261030_001126'
T=['/camera/color/image_raw','/camera/depth/image_rect_raw','/camera/infra1/image_rect_raw','/camera/infra2/image_rect_raw']
T0=1793286688.362388823
def grab(sec):
    r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=B,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    r.set_filter(rosbag2_py.StorageFilter(topics=T)); r.seek(int((T0+sec)*1e9)); got={}
    while r.has_next() and len(got)<4:
        tp,d,t=r.read_next()
        if tp not in got: got[tp]=(t*1e-9-T0,deserialize_message(d,Image))
    return got
def arr(m):
    if m.encoding=='rgb8': return np.frombuffer(m.data,np.uint8).reshape(m.height,m.width,3)
    if m.encoding=='16UC1': return np.frombuffer(m.data,np.uint16).reshape(m.height,m.width).astype(np.float32)/1000
    return np.frombuffer(m.data,np.uint8).reshape(m.height,m.width)
SEC=[float(s) for s in sys.argv[1:]] or [60,200,400]
fig,axs=plt.subplots(len(SEC),4,figsize=(19,3.9*len(SEC)),dpi=110)
for row,sec in zip(axs,SEC):
    g=grab(sec)
    for ax,tp,name in zip(row,T,['컬러 1280×720','뎁스 848×480 (m)','적외선 왼쪽 848×480','적외선 오른쪽 848×480']):
        ts,m=g[tp]; a=arr(m)
        if tp.endswith('depth/image_rect_raw'):
            a=np.where(a>0,a,np.nan); im=ax.imshow(a,cmap='turbo',vmin=0.2,vmax=4.0)
            fig.colorbar(im,ax=ax,fraction=0.035,pad=0.01)
            ax.set_title(f'{name}  유효 {np.isfinite(a).mean()*100:.0f}%  ({ts:.1f}s)',fontsize=10)
        else:
            ax.imshow(a,cmap=None if a.ndim==3 else 'gray'); ax.set_title(f'{name}  ({ts:.1f}s)',fontsize=10)
        ax.axis('off')
fig.suptitle('D435i (로봇 내부 PC) · gosu_20261030_001126 · 같은 순간의 컬러·뎁스·적외선 스테레오',fontsize=13)
fig.tight_layout(); fig.savefig('cam_frames_1030.png'); print('saved cam_frames_1030.png')
