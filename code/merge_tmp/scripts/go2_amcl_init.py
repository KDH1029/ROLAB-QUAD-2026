#!/usr/bin/env python3
"""Give AMCL a start pose with uncertainty, then let it converge while the robot stands still.

A start pose set without covariance (amcl set_initial_pose) puts every particle on one point, so a
wrong start is never corrected, and AMCL does not update at all while the robot is not moving.
This publishes /initialpose with covariance and calls request_nomotion_update repeatedly.

Bag check (누리관, start pose off by 0.49 m / 20 deg): 6 cm / 0 deg after 10 s standing still.

Usage: go2_amcl_init.py <x> <y> <yaw_deg> [--sigma-xy 0.5] [--sigma-yaw-deg 20] [--updates 20]
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('x', type=float)
    ap.add_argument('y', type=float)
    ap.add_argument('yaw_deg', type=float)
    ap.add_argument('--sigma-xy', type=float, default=0.5)
    ap.add_argument('--sigma-yaw-deg', type=float, default=20.0)
    ap.add_argument('--updates', type=int, default=20)
    ap.add_argument('--frame', default='map')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('go2_amcl_init')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    cli = node.create_client(Empty, '/request_nomotion_update')

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = a.frame
    msg.pose.pose.position.x = a.x
    msg.pose.pose.position.y = a.y
    yaw = math.radians(a.yaw_deg)
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    cov = [0.0] * 36
    cov[0] = cov[7] = a.sigma_xy ** 2
    cov[35] = math.radians(a.sigma_yaw_deg) ** 2
    msg.pose.covariance = cov
    for _ in range(3):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.3)

    if not cli.wait_for_service(timeout_sec=5.0):
        print('[ERROR] /request_nomotion_update not available (is AMCL active?)')
        return
    for i in range(a.updates):
        rclpy.spin_until_future_complete(node, cli.call_async(Empty.Request()), timeout_sec=2.0)
        time.sleep(0.5)
    print(f'[DONE] initial pose ({a.x:.2f}, {a.y:.2f}, {a.yaw_deg:.0f} deg) +{a.updates} no-motion updates')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
