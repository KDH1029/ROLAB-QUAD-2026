import rosbag2_py, sys, json
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
B = sys.argv[1]
TY = {'/api/sport/request': 'unitree_api/msg/Request', '/api/sport/response': 'unitree_api/msg/Response', '/sportmodestate': 'unitree_go/msg/SportModeState', '/mission/runner_status': 'std_msgs/msg/String'}
M = {k: get_message(v) for k, v in TY.items()}
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=B, storage_id='mcap'), rosbag2_py.ConverterOptions('', '')); r.set_filter(rosbag2_py.StorageFilter(topics=list(TY)))
ev = []; st = []; first_rs = None
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9; m = deserialize_message(raw, M[tp])
    if tp == '/sportmodestate': st.append((t, m.mode, m.gait_type, m.progress, round(m.body_height, 3)))
    elif tp == '/api/sport/request' and m.header.identity.api_id != 1008: ev.append((t, '요청', m.header.identity.api_id, m.parameter[:30], m.header.identity.id))
    elif tp == '/api/sport/response' and m.header.identity.api_id != 1008: ev.append((t, '응답', m.header.identity.api_id, m.header.status.code, m.header.identity.id))
    elif tp == '/mission/runner_status' and first_rs is None: first_rs = t
import bisect
ts = [s[0] for s in st]
def mode_at(t):
    k = bisect.bisect_right(ts, t) - 1; return st[k][1:] if k >= 0 else None
print('첫 러너 상태', round(first_rs or 0, 2))
for e in ev:
    if e[2] in (2049, 1002, 1004, 1005, 1001, 1006, 2048) or e[1] == '응답' and e[2] == 2049:
        print(f'  {e[0]:.3f} {e[1]} api {e[2]} {e[3]}  id {e[4]}  · 그때 mode/gait/progress/높이 {mode_at(e[0])}  · 0.5초 전 {mode_at(e[0] - 0.5)}')
# 앞 3초 sportmodestate 모드 변화
if st:
    prev = None
    for s in st:
        if (s[1], s[2]) != prev and s[0] < (first_rs or st[0][0]) + 5: print('   모드변화', round(s[0], 3), 'mode', s[1], 'gait', s[2], '높이', s[4]); prev = (s[1], s[2])
