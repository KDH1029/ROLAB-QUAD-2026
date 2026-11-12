#!/usr/bin/env python3
"""사전 지도를 화면에서 볼 수 있게 내보낸다 — 스캔과 겹쳐 봐야 정합이 맞는지 눈으로 안다.

  python3 map_publisher.py                        기본 지도를 한 번 내보낸다
  python3 map_publisher.py --voxel 0.15 --band    벽 대역만, 더 성기게 (화면이 가벼워진다)

내는 것
  map/points   sensor_msgs/PointCloud2   `map` 프레임. 늦게 붙는 뷰어도 받도록 latched 로 낸다.

【왜 필요한가】
정합 노드는 TF 와 자세만 낸다. 지도 자체를 내보내지 않으면 뷰어에는 스캔만 보이고
"지도와 얼마나 맞는지" 를 눈으로 확인할 수 없다. 그 확인이 주행 전 마지막 관문이다.

【화면 무게】
정합용 지도는 175,480점이다. 그대로 내보내면 뷰어가 버벅인다.
기본 0.10 m 복셀로 줄여서 내보내고, --band 를 주면 바닥·천장을 빼고 벽 대역만 남긴다.
한 번만 발행하고(latched) 끝나므로 통신 부하는 계속 들지 않는다.
"""
import argparse
import math
import os
import struct
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from .read_pcd import read_pcd


def cloud_msg(pts, frame, stamp):
    m = PointCloud2()
    m.header = Header(frame_id=frame)
    m.header.stamp = stamp
    m.height = 1
    m.width = len(pts)
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate(('x', 'y', 'z'))]
    m.is_bigendian = False
    m.point_step = 12
    m.row_step = 12 * len(pts)
    m.is_dense = True
    m.data = pts.astype('<f4').tobytes()
    return m



def parse_args(ap):
    """ROS 인자(--ros-args …)와 우리 인자를 갈라서 읽는다.

    argparse 는 --ros-args 를 모르는 인자로 보고 거절한다. 그래서 rclpy 의 유틸로
    ROS 인자를 먼저 걷어낸 뒤 나머지만 argparse 에 준다.
    ROS 인자는 rclpy.init(args=sys.argv) 로 그대로 넘겨야 use_sim_time 같은 게 먹는다.
    """
    from rclpy.utilities import remove_ros_args
    return ap.parse_args(remove_ros_args(sys.argv)[1:])

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.path.expanduser(
        '~/gosu/lab_visual/maps/go2_prior_map_v05.pcd'))
    ap.add_argument('--frame', default='map')
    ap.add_argument('--topic', default='map/points')
    ap.add_argument('--voxel', type=float, default=0.10,
                    help='이 크기 격자마다 한 점만 남긴다. 크게 줄수록 화면이 가볍다')
    ap.add_argument('--band', action='store_true',
                    help='바닥면 위 0.3~2.0 m 만 내보낸다 (벽·기둥만 보여 정합 확인이 쉽다)')
    ap.add_argument('--repeat', type=float, default=0.0,
                    help='0 이면 한 번만(latched). 뷰어가 자꾸 놓치면 이 주기(초)로 다시 낸다')
    a = parse_args(ap)

    p = os.path.expanduser(a.map)
    if not os.path.exists(p):
        raise SystemExit('★ 지도가 없다: %s' % p)
    P = read_pcd(p)
    n0 = len(P)

    if a.band:
        # 지도가 기울어 있을 수 있으므로 바닥면을 맞춰서 자른다 (z 값 하나로 자르면 안 된다)
        sel = P[:, 2] < np.percentile(P[:, 2], 30)
        for _ in range(10):
            X = np.c_[np.ones(sel.sum()), P[sel, 0], P[sel, 1]]
            c, *_ = np.linalg.lstsq(X, P[sel, 2], rcond=None)
            r = P[:, 2] - (c[0] + c[1] * P[:, 0] + c[2] * P[:, 1])
            sel = np.abs(r) < 0.05
        h = P[:, 2] - (c[0] + c[1] * P[:, 0] + c[2] * P[:, 1])
        tilt = math.degrees(math.atan(math.hypot(c[1], c[2])))
        P = P[(h > 0.3) & (h < 2.0)]
        print('  바닥면 맞춤: 기울기 %.2f° -> 그 위 0.3~2.0 m 만 남김' % tilt)

    if a.voxel > 0:
        k = np.floor(P / a.voxel).astype(np.int64)
        _, idx = np.unique(k, axis=0, return_index=True)
        P = P[np.sort(idx)]

    print('지도 %s: %d점 -> 내보낼 %d점 (복셀 %.2f m%s)'
          % (os.path.basename(p), n0, len(P), a.voxel, ', 대역만' if a.band else ''))

    rclpy.init(args=sys.argv)
    node = Node('map_publisher')
    qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
    pub = node.create_publisher(PointCloud2, a.topic, qos)
    msg = cloud_msg(P, a.frame, node.get_clock().now().to_msg())
    pub.publish(msg)
    node.get_logger().info('%s 에 %d점 발행 (프레임 %s, latched — 늦게 붙는 뷰어도 받는다)'
                           % (a.topic, len(P), a.frame))
    if a.repeat > 0:
        def again():
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
        node.create_timer(a.repeat, again)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass          # Ctrl-C 나 SIGTERM. 정상 종료로 본다
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
