"""raw=True 로 받은 CDR 바이트를 numpy 로 직접 해석 (sensor_msgs/Image, nav_msgs/Odometry, std 헤더).

rclpy 역직렬화(파이썬 객체 생성)를 건너뛰기 위함. XCDR1 리틀엔디언만 (ROS 2 기본).
정렬은 4바이트 캡슐 머리 뒤를 기준(base=4)으로 잡는다.
"""
import struct
import numpy as np

_ENC = {'rgb8': (np.uint8, 3), 'bgr8': (np.uint8, 3), 'mono8': (np.uint8, 1), '16UC1': (np.uint16, 1),
        'mono16': (np.uint16, 1), '32FC1': (np.float32, 1), 'rgba8': (np.uint8, 4), 'bgra8': (np.uint8, 4)}


def _al(pos, n):
    return 4 + ((pos - 4 + n - 1) // n) * n


def _str(buf, pos):
    pos = _al(pos, 4)
    L = struct.unpack_from('<I', buf, pos)[0]
    s = bytes(buf[pos + 4:pos + 4 + L]).rstrip(b'\0').decode('utf-8', 'replace')
    return s, pos + 4 + L


def _check(buf):
    if buf[1] != 1:   # 0x0001 = CDR_LE
        raise ValueError('big-endian / XCDR2 CDR 은 지원 안 함: %r' % bytes(buf[:4]))


def hdr_stamp(buf):
    """header.stamp (초, float)."""
    s, ns = struct.unpack_from('<iI', buf, 4)
    return s + ns * 1e-9


def parse_header(buf):
    _check(buf)
    s, ns = struct.unpack_from('<iI', buf, 4)
    fid, pos = _str(buf, 12)
    return s + ns * 1e-9, fid, pos


def parse_image(buf, meta=False):
    """sensor_msgs/Image → numpy (H,W[,C]) — 복사 없이 버퍼를 본다(원본 bytes 가 살아있는 동안)."""
    stamp, fid, pos = parse_header(buf)
    pos = _al(pos, 4)
    h, w = struct.unpack_from('<II', buf, pos); pos += 8
    enc, pos = _str(buf, pos)
    big = buf[pos]; pos += 1
    pos = _al(pos, 4)
    step = struct.unpack_from('<I', buf, pos)[0]; pos += 4
    n = struct.unpack_from('<I', buf, pos)[0]; pos += 4
    dt, c = _ENC[enc]
    a = np.frombuffer(buf, np.uint8, count=n, offset=pos)
    itm = np.dtype(dt).itemsize
    if step != w * c * itm:          # 행 패딩
        a = a.reshape(h, step)[:, :w * c * itm]
    a = np.ascontiguousarray(a).view(dt) if step != w * c * itm else a.view(dt)
    a = a.reshape(h, w, c) if c > 1 else a.reshape(h, w)
    if big:
        a = a.byteswap()
    if meta:
        return a, dict(stamp=stamp, frame_id=fid, height=h, width=w, encoding=enc, step=step)
    return a


def parse_odom(buf):
    """nav_msgs/Odometry → (stamp, frame_id, child_frame_id, p[3], q[x,y,z,w])."""
    stamp, fid, pos = parse_header(buf)
    child, pos = _str(buf, pos)
    pos = _al(pos, 8)
    v = struct.unpack_from('<7d', buf, pos)
    return stamp, fid, child, np.array(v[:3]), np.array(v[3:7])
