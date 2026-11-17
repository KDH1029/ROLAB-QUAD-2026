#!/usr/bin/env python3
"""정지선에서 실제 신호등 확인 — 러너와 같은 경로(/mission/camera_active 켜기 → /mission/traffic_light_state 받기)로 N 초 동안 본다.
로봇을 움직이지 않는다(카메라 추론만 켰다가 끝나면 끈다). 신호가 5초마다 바뀌므로 바뀜 간격이 4~6 초면 통과.
또 러너 규칙(GREEN 1.0 초 연속이면 출발)을 그대로 적용해 '여기서 출발했을 시각'과 그때 판정 흐름을 보여준다.
사용(NUC, 로봇 연결 상태): source ~/gosu/env_drive.sh; python3 ~/ws/junsu/onboard_vision/check_tl_live.py [초=20]
  내부 PC 노드가 먼저 떠 있어야 한다: ~/ws/junsu/onboard_vision/deploy_onboard_vision.sh start"""
import sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
import os, re
try:   # 러너 파일에서 출발 기준을 읽는다(가져오기는 안 함 — 러너 모듈은 실행 인자를 좌표 파일로 읽는다)
    _src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'real_course_runner.py')).read()
    GREEN_CONFIRM_SECONDS = float(re.search(r'^GREEN_CONFIRM_SECONDS\s*=\s*([0-9.]+)', _src, re.M).group(1))
except Exception:
    GREEN_CONFIRM_SECONDS = 1.0


class Check(Node):
    def __init__(self):
        super().__init__('tl_live_check')
        self.pub = self.create_publisher(Bool, '/mission/camera_active', 10)
        self.create_subscription(String, '/mission/traffic_light_state', self.cb, 10)
        self.rows = []

    def cb(self, m):
        st, _, cf = (m.data or 'UNKNOWN|0').partition('|')
        self.rows.append((time.monotonic(), st, float(cf or 0)))


rclpy.init(); n = Check()
t_end = time.monotonic() + DUR; last_pub = 0.0
print(f'카메라 추론 켜고 {DUR:.0f} 초 동안 신호 판정 받기 (러너 출발 기준 GREEN {GREEN_CONFIRM_SECONDS} 초 연속)')
while time.monotonic() < t_end:
    if time.monotonic() - last_pub > 0.5:
        n.pub.publish(Bool(data=True)); last_pub = time.monotonic()
    rclpy.spin_once(n, timeout_sec=0.05)
for _ in range(3):
    n.pub.publish(Bool(data=False)); rclpy.spin_once(n, timeout_sec=0.05)
R = n.rows; n.destroy_node(); rclpy.shutdown()
if not R:
    print('★ 판정이 하나도 안 왔다 — 내부 PC 노드가 안 떠 있거나(deploy_onboard_vision.sh status), DDS 가 안 이어짐'); sys.exit(2)
t0 = R[0][0]; hz = len(R) / max(R[-1][0] - t0, 1e-3)
cnt = {k: sum(r[1] == k for r in R) for k in ('RED', 'GREEN', 'UNKNOWN')}
print(f'받은 판정 {len(R)} 개 ({hz:.1f} Hz) · RED {cnt["RED"]} · GREEN {cnt["GREEN"]} · UNKNOWN {cnt["UNKNOWN"]} ({cnt["UNKNOWN"] / len(R) * 100:.0f}%)')
seq = [(r[0] - t0, r[1]) for r in R if r[1] != 'UNKNOWN']
trans = [(b[0], f'{a[1]}→{b[1]}') for a, b in zip(seq[:-1], seq[1:]) if a[1] != b[1]]
print('바뀜: ' + (', '.join(f'{t:.1f}s {s}' for t, s in trans) if trans else '없음'))
gaps = [b[0] - a[0] for a, b in zip(trans[:-1], trans[1:])]
short = [g for g in gaps if g < 2.0]
# 러너 규칙: GREEN 이 끊기지 않고 GREEN_CONFIRM_SECONDS → 출발
since = None; go = None
for t, st, _ in R:
    if st == 'GREEN':
        since = t if since is None else since
        if t - since >= GREEN_CONFIRM_SECONDS: go = t - t0; break
    else:
        since = None
print(f'러너였다면 출발: {go:.1f} 초' if go is not None else '러너였다면 출발: 없음(이 시간 안에 GREEN 이 1초 연속 안 됨)')
ok = len(gaps) >= 1 and all(4.0 <= g <= 6.0 for g in gaps) and not short and cnt['UNKNOWN'] / len(R) < 0.3
print('결과: ' + ('✔ 통과 — 신호를 5초 주기로 읽고 있다' if ok else
                 '✘ 확인 필요 — ' + ('바뀜이 2번 미만(더 길게 보기: 인자 30) ' if len(gaps) < 1 else '') +
                 (f'2초 미만 깜빡임 {len(short)}번 ' if short else '') + ('간격이 4~6초 밖 ' if gaps and not all(4.0 <= g <= 6.0 for g in gaps) else '') +
                 (f'UNKNOWN {cnt["UNKNOWN"] / len(R) * 100:.0f}% (신호등이 화면에 안 잡힘?) ' if cnt['UNKNOWN'] / len(R) >= 0.3 else '')))
sys.exit(0 if ok else 1)
