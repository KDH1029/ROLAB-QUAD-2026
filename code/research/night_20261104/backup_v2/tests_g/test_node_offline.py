#!/usr/bin/env python3
"""backup_loc_node_v2.BackupLoc 콜백을 DDS 없이 직접 부르는 연기 시험 — rclpy.init·Node 생성 없음(가짜 시계·가짜 발행기).
녹화본 시작부(0~25 s, 정지)를 돌린 지도(tests/map_first270s_rot3deg.npz)로 넣고 확인한다:
  · 노드 코드 경로(on_fastlio/on_leg/on_cloud/report)가 예외 없이 돈다
  · align 이 고정되고 T ≈ 정답 A⁻¹, 고정 전 health 'aligning', 고정 뒤 'ok'
  · /backup/odometry·/backup/cloud 가 camera_init 좌표(= FAST-LIO 와 수 cm)
  · 깨진 PointCloud2 바이트를 넣어도 콜백이 예외를 잡고 노드가 계속 돈다
실행: source /opt/ros/jazzy/setup.bash && ~/ws/night_20261104/venv/bin/python tests/test_node_offline.py
"""
import sys, os, math, json, threading, types
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); V2 = os.path.dirname(HERE)
sys.path.insert(0, V2); sys.path.insert(0, HERE)
from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rclpy.serialization import serialize_message
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import backup_loc_node_v2 as N
from backup_core_v2 import make_estimator, load_map
from frame_align import FrameAligner
from test_frame_align import load_events

SIM = {'t': 0.0}


class _Now:
    def __init__(s, t): s.nanoseconds = int(t * 1e9)
    def to_msg(s):
        m = TimeMsg(); m.sec = int(s.nanoseconds // 1_000_000_000); m.nanosec = int(s.nanoseconds % 1_000_000_000); return m


class _Clock:
    def now(s): return _Now(SIM['t'])


class _Log:
    def __init__(s): s.lines = []
    def info(s, m): s.lines.append(('I', m))
    def warn(s, m): s.lines.append(('W', m))
    def error(s, m): s.lines.append(('E', m))


class _Pub:
    def __init__(s): s.msgs = []
    def publish(s, m): s.msgs.append((SIM['t'], m))


class FakeLoc(N.BackupLoc):
    def __init__(self, mp):                    # Node.__init__ 을 부르지 않는다 (DDS 참여 없음)
        self.n_exc, self.t_exc_log = {}, {}
        self.est = make_estimator(mp); self.hz, self.init_mode, self.frame = 5.0, 'fastlio', 'camera_init'
        self.align_on, self.align_mask = True, True; self.al = FrameAligner()
        self.lock = threading.Lock(); self.leg = None; self.fl0 = None; self.t_first_leg = None
        self.last_scan = 0.0; self.busy = False; self.n_leg = 0
        self.pub_odom, self.pub_cloud, self.pub_stat = _Pub(), _Pub(), _Pub(); self._log = _Log(); self._clk = _Clock()
    def get_clock(self): return self._clk
    def get_logger(self): return self._log


def main():
    N.time = types.SimpleNamespace(monotonic=lambda: SIM['t'], time=__import__('time').time)   # 노드의 5 Hz 거르기를 가짜 시계로
    sys.path.insert(1, os.path.expanduser('~/ws/v_gosu/backup'))
    mpath = os.path.join(HERE, 'map_first270s_rot3deg.npz'); z = np.load(mpath)
    ya = math.radians(float(z['A_yaw_deg'])); ax, ay = map(float, z['A_t']); c, s = math.cos(ya), math.sin(ya)
    Tt = (-(c * ax + s * ay), -(-s * ax + c * ay), -ya)
    (ev,), tL, GT, gyaw, t0 = load_events(((0.0, 25.0),))
    n = FakeLoc(load_map(mpath)); fl = []; t_next_rep = None; bad_fed = False; ok = True
    for t, kind, val in ev:
        SIM['t'] = t
        if kind == 2:
            m = Odometry(); m.header.stamp = _Now(t).to_msg(); m.header.frame_id = 'camera_init'
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = val[0], val[1], val[2]
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(val[3] / 2), math.cos(val[3] / 2)
            SIM['t'] = t + 0.0085; n.on_fastlio(m); fl.append((t, val[0], val[1]))
        elif kind == 0:
            m = Odometry(); m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = val[0], val[1], val[2]
            yy, rr, pp = val[3], val[4], val[5]
            cr, sr, cp, sp, cy, sy = math.cos(rr / 2), math.sin(rr / 2), math.cos(pp / 2), math.sin(pp / 2), math.cos(yy / 2), math.sin(yy / 2)
            q = m.pose.pose.orientation
            q.w = cr * cp * cy + sr * sp * sy; q.x = sr * cp * cy - cr * sp * sy; q.y = cr * sp * cy + sr * cp * sy; q.z = cr * cp * sy - sr * sp * cy
            n.on_leg(m)
        else:
            if not bad_fed and n.est.inited and t > t0 + 12.0:      # 깨진 바이트 한 번
                n.last_scan = -1e9; n.on_cloud(b'\x00\x01\x00\x00garbage'); bad_fed = True
            hdr = Header(); hdr.frame_id = 'hesai_lidar'; hdr.stamp = _Now(t).to_msg()
            n.on_cloud(serialize_message(pc2.create_cloud_xyz32(hdr, val.astype(np.float32))))
        if t_next_rep is None: t_next_rep = t + 1.0
        if t >= t_next_rep:
            t_next_rep += 1.0; n.report()
    st = [json.loads(m.data) for _, m in n.pub_stat.msgs]
    def chk(name, cond, info=''):
        nonlocal ok; ok &= bool(cond); print(('  통과 ' if cond else '  ★실패 ') + name, info)
    last = st[-1]; a = last['align']
    chk('정렬 고정', a['state'] == 'frozen', f"{a['state']} {a.get('why')} 고정 {a.get('frozen_after_s')}s")
    Te = a['T']; dxy = math.hypot(Te['x'] - Tt[0], Te['y'] - Tt[1]); dyw = Te['yaw_deg'] - math.degrees(Tt[2])
    chk('T ≈ 정답 A⁻¹ (0.05 m·0.5°)', dxy < 0.05 and abs(dyw) < 0.5, f"추정 {Te} 정답 {np.round([Tt[0], Tt[1], math.degrees(Tt[2])], 3)} → {dxy:.3f} m {dyw:+.2f}°")
    hs = [(s_['health'], s_['align']['state']) for s_ in st]
    chk("고정 전 health 'aligning' → 고정 뒤 'ok'", any(h == 'aligning' for h, _ in hs) and hs[-1][0] == 'ok', str(sorted(set(hs))))
    # /backup/odometry(camera_init) vs FAST-LIO (고정 뒤)
    F = np.array(fl); tf = None
    for s_ in st:
        pass
    frz_t = next((tt for (tt, m) in n.pub_stat.msgs if json.loads(m.data)['align']['state'] == 'frozen'), None)
    O = np.array([(tt, m.pose.pose.position.x, m.pose.pose.position.y) for tt, m in n.pub_odom.msgs if frz_t and tt >= frz_t])
    if len(O):
        j = np.clip(np.searchsorted(F[:, 0], O[:, 0]), 0, len(F) - 1); e = np.hypot(O[:, 1] - F[j, 1], O[:, 2] - F[j, 2])
        chk('/backup/odometry ≈ FAST-LIO (p95 < 0.1 m)', np.percentile(e, 95) < 0.1, f'{len(O)}개 중앙 {np.median(e):.3f} p95 {np.percentile(e, 95):.3f} 최대 {e.max():.3f} m')
    else:
        chk('/backup/odometry 고정 뒤 발행', False)
    if n.pub_cloud.msgs:
        cm = n.pub_cloud.msgs[-1][1]; W = pc2.read_points_numpy(cm, field_names=('x', 'y', 'z'))
        chk('/backup/cloud camera_init 좌표', cm.header.frame_id == 'camera_init' and len(W) > 100, f'{len(W)}점')
    chk('깨진 스캔 → 예외 잡고 계속', bad_fed and n.n_exc.get('on_cloud', 0) == 1 and len(n.pub_cloud.msgs) > 10 and last.get('exceptions', {}).get('on_cloud') == 1,
        f"예외 {n.n_exc} 로그 {[l[1][:60] for l in n._log.lines if l[0] == 'E'][:1]}")
    # /mux/status mode=backup → 정렬 즉시 고정 (새 노드: init 직후, 쌍 없음 → 항등 고정 + 경고). 이미 고정된 노드는 그대로
    from std_msgs.msg import String
    ne = dict(n.n_exc); n.on_mux_status(String(data='{"mode": "backup", "reason": "x"}'))
    n2 = FakeLoc(load_map(mpath)); n2.est.init(0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 0.3, 0.0, 0.0, 0.0), SIM['t']); n2.al.on_init(SIM['t'])
    n2.on_mux_status(String(data='{"mode": "fastlio"}')); s_a = n2.al.state
    n2.on_mux_status(String(data='{"mode": "backup", "reason": "x"}'))
    st2 = json.loads(n2.pub_stat.msgs[-1][1].data) if n2.pub_stat.msgs else {}
    chk('/mux/status backup → 즉시 고정', n.n_exc == ne and a['state'] == 'frozen' and s_a == 'identity' and n2.al.state == 'identity_frozen'
        and st2.get('align', {}).get('state') == 'identity_frozen' and not n2.n_exc, f"{n2.al.why} / 경고로그 {[m[:50] for k, m in n2._log.lines if k == 'W']}")
    print('노드 로그:', *[f'[{k}] {m[:140]}' for k, m in n._log.lines if k != 'E'], sep='\n  ')
    print('노드 연기 시험', '전부 통과' if ok else '★실패 있음', flush=True)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
