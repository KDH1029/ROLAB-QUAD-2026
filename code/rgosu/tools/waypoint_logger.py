#!/usr/bin/env python3
"""조종기로 로봇을 직접 몰면서, 지나는 지점마다 라벨을 입력하면 그 순간의
(/Odometry 기준 x,y)를 저장하는 대화형 도구.

사용법:
  (다른 터미널에서 bridge+FAST-LIO가 떠서 /Odometry가 나오는 상태에서)
  python3 waypoint_logger.py [출력파일경로]

터미널에 라벨을 입력하고 Enter를 누르면 그 순간의 현재 위치가 즉시
출력파일에 append 저장됨(라벨 비우고 Enter만 누르면 현재 위치만 확인).
"""
import sys
import json
import threading
import time
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

OUT_PATH = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/ws/Rgosu/maps/real_waypoints.json')


class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        self.pos = None
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, 10)
        self.waypoints = []
        if os.path.exists(OUT_PATH):
            try:
                with open(OUT_PATH) as f:
                    self.waypoints = json.load(f)
                self.get_logger().info(f'기존 저장파일 불러옴: {len(self.waypoints)}개')
            except Exception:
                pass

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y)

    def save(self, label):
        if self.pos is None:
            print('!! 아직 /Odometry 안 들어옴 — 저장 불가')
            return
        x, y = self.pos
        entry = {'label': label, 'x': round(x, 3), 'y': round(y, 3), 't': time.time()}
        self.waypoints.append(entry)
        with open(OUT_PATH, 'w') as f:
            json.dump(self.waypoints, f, ensure_ascii=False, indent=2)
        print(f'>> 저장됨 [{label}] -> x={x:.3f}, y={y:.3f}  (총 {len(self.waypoints)}개, 파일: {OUT_PATH})')


def input_loop(node: WaypointLogger):
    print(f'저장 파일: {OUT_PATH}')
    print('라벨 입력 후 Enter = 현재 위치 저장. 빈 줄 Enter = 현재 위치만 출력. Ctrl+C = 종료.')
    print('예시 라벨: start, stopline1(=경사로 진입), ramp_top, stopline2, parking, exit(=start와 동일지점 재통과)')
    while True:
        try:
            label = input('label> ').strip()
        except EOFError:
            break
        if label == '':
            if node.pos:
                print(f'(현재 위치) x={node.pos[0]:.3f}, y={node.pos[1]:.3f}')
            else:
                print('!! 아직 /Odometry 없음')
            continue
        node.save(label)


def main():
    rclpy.init()
    node = WaypointLogger()
    t = threading.Thread(target=input_loop, args=(node,), daemon=True)
    t.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
