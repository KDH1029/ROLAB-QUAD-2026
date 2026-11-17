#!/usr/bin/env python3
"""녹화 → 후보 사진(JPEG 95). 시스템 python3 + ROS Jazzy 환경에서 실행.
사용: extract_frames.py <출력폴더> <D435i 간격(장)> <앞머리 간격(장)> [최대장수/스트림=0(전부)] <녹화폴더>...
  D435i  : 녹화폴더/camera_robotpc_p*/ *.db3 (sqlite — 필요한 장만 바로 읽음) 의 /camera/color/image_raw
  앞머리 : 녹화폴더 NUC bag 의 /frontvideostream 중 720p 프레임(H.264, 360p 와 번갈아 옴) → 파일로 이어 붙여 OpenCV(FFMPEG)로 풂
출력: <출력폴더>/<cam>_<녹화시각>_f<프레임번호>.jpg + frames_<녹화시각>_<cam>.csv"""
import os, sys, glob, csv, sqlite3, struct, numpy as np, cv2, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
OUT, DS, FS, MAXN = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]); BAGS = sys.argv[5:]
os.makedirs(OUT, exist_ok=True)
Img = get_message('sensor_msgs/msg/Image')
def pick(n, step):
    idx = list(range(0, n, step))
    if MAXN and len(idx) > MAXN: idx = list(np.linspace(0, n - 1, MAXN).astype(int))
    return idx
def save(path, bgr): cv2.imwrite(path, bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
DIFF = 4.0   # 32x18 흑백 축소본 평균 차이(0~255) — 이보다 작으면 앞 장과 같은 장면으로 보고 건너뜀
class Dedup:
    def __init__(s): s.last = None; s.skipped = 0
    def new(s, bgr):
        th = cv2.resize(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (32, 18), interpolation=cv2.INTER_AREA).astype(np.float32)
        if s.last is not None and np.abs(th - s.last).mean() < DIFF: s.skipped += 1; return False
        s.last = th; return True
for B in BAGS:
    B = B.rstrip('/'); tag = os.path.basename(B).replace('gosu_', '').split('_arena')[0].split('_')[1]   # 145915
    # ---- D435i
    for db in sorted(glob.glob(f'{B}/camera_robotpc_p*/*.db3')):
        c = sqlite3.connect(f'file:{db}?mode=ro', uri=True); tid = dict(c.execute('select name,id from topics'))['/camera/color/image_raw']
        ts = [r[0] for r in c.execute('select timestamp from messages where topic_id=? order by timestamp', (tid,))]
        rows = []; dd = Dedup()
        for i in pick(len(ts), DS):
            raw = c.execute('select data from messages where topic_id=? and timestamp=?', (tid, ts[i])).fetchone()[0]
            m = deserialize_message(raw, Img); a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
            bgr = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a
            if not dd.new(bgr): continue
            fn = f'd435_{tag}_f{i:06d}.jpg'; save(os.path.join(OUT, fn), bgr); rows.append((fn, 'd435', tag, i, ts[i]))
        csv.writer(open(f'{OUT}/frames_{tag}_d435.csv', 'w')).writerows(rows); print(f'{tag} D435i {len(rows)}장 저장 · 같은 장면 {dd.skipped}장 건너뜀 / 전체 {len(ts)}', flush=True)
    # ---- 앞머리
    if not os.path.exists(f'{B}/metadata.yaml'): continue
    sid = 'mcap' if glob.glob(f'{B}/*.mcap') else 'sqlite3'
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=B, storage_id=sid), rosbag2_py.ConverterOptions('', ''))
    if '/frontvideostream' not in {t.name for t in r.get_all_topics_and_types()}: continue
    r.set_filter(rosbag2_py.StorageFilter(topics=['/frontvideostream']))
    h264 = f'{OUT}/.front_{tag}.h264'; ts = []
    with open(h264, 'wb') as f:
        while r.has_next():
            _, raw, t = r.read_next(); raw = bytes(raw)
            # 실제 형식(NUC 의 unitree_go 정의와 다름, 11-04 바이트 확인): CDR머리4 · uint64 time_frame · uint32 해상도(720|360) · uint32 길이 · H.264
            res, n = struct.unpack_from('<II', raw, 12)
            if res == 720 and n: f.write(raw[20:20 + n]); ts.append(t)
    cap = cv2.VideoCapture(h264, cv2.CAP_FFMPEG); frames = 0; rows = []; want = set(pick(len(ts), FS)); dd = Dedup()
    while True:
        ok, bgr = cap.read()
        if not ok: break
        if frames in want and dd.new(bgr):
            fn = f'front_{tag}_f{frames:06d}.jpg'; save(os.path.join(OUT, fn), bgr)
            rows.append([fn, 'front', tag, frames, 0])
        frames += 1
    os.remove(h264); off = max(0, len(ts) - frames)   # 처음 키프레임 전 off 장은 안 풀림
    for rw in rows: rw[4] = ts[min(rw[3] + off, len(ts) - 1)]
    csv.writer(open(f'{OUT}/frames_{tag}_front.csv', 'w')).writerows(rows)
    print(f'{tag} 앞머리 {len(rows)}장 저장 · 같은 장면 {dd.skipped}장 건너뜀 / 풀린 {frames} (720p 메시지 {len(ts)}, 앞 {len(ts) - frames}장 키프레임 전)', flush=True)
