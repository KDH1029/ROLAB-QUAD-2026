#!/usr/bin/env python3
"""
Nav2 goal client and waypoint manager for Unitree Go2.

Usage:
  python3 scripts/go2_goto.py --x 2.5 --y -1.2 --yaw 90 [--sit]
  python3 scripts/go2_goto.py --goal "2.5, -1.2, 90"
  python3 scripts/go2_goto.py save point_A
  python3 scripts/go2_goto.py go point_A
  python3 scripts/go2_goto.py list
  python3 scripts/go2_goto.py pose
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf2_ros

try:
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus
    HAS_NAV2_MSGS = True
except ImportError:
    NavigateToPose = None
    GoalStatus = None
    HAS_NAV2_MSGS = False

try:
    from unitree_api.msg import Request
    HAS_UNITREE_API = True
except ImportError:
    Request = None
    HAS_UNITREE_API = False

WS_ROOT = Path(__file__).resolve().parent.parent
TARGETS_FILE = str(WS_ROOT / 'config' / 'targets.json')


def euler_to_quat(yaw_rad):
    return (0.0, 0.0, math.sin(yaw_rad * 0.5), math.cos(yaw_rad * 0.5))


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def load_targets():
    if os.path.exists(TARGETS_FILE):
        try:
            with open(TARGETS_FILE, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            return {}
    return {}


def save_targets(d):
    os.makedirs(os.path.dirname(TARGETS_FILE), exist_ok=True)
    with open(TARGETS_FILE, 'w', encoding='utf-8') as f:
        json.dump(d, f, ensure_ascii=False, indent=2)


class Go2GotoClient(Node):
    def __init__(self):
        super().__init__('go2_goto_client')
        self._action_client = (
            ActionClient(self, NavigateToPose, 'navigate_to_pose')
            if HAS_NAV2_MSGS else None
        )
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.sport_pub = (
            self.create_publisher(Request, '/api/sport/request', 10)
            if HAS_UNITREE_API else None
        )
        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_robot_pose(self, timeout_sec=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            for parent_frame in ('map', 'odom'):
                try:
                    tf = self.tf_buffer.lookup_transform(
                        parent_frame, 'base_link', rclpy.time.Time())
                    x = tf.transform.translation.x
                    y = tf.transform.translation.y
                    yaw = math.degrees(quat_to_yaw(tf.transform.rotation))
                    return x, y, yaw
                except Exception:
                    pass
            time.sleep(0.05)
        return None, None, None

    def send_sport_cmd(self, api_id, param_str="{}"):
        if not self.sport_pub:
            return
        req = Request()
        req.header.identity.id = self.get_clock().now().nanoseconds
        req.header.identity.api_id = api_id
        req.parameter = param_str
        self.sport_pub.publish(req)

    def build_pose_stamped(self, target_x, target_y, target_yaw_deg):
        yaw_rad = math.radians(target_yaw_deg)
        qx, qy, qz, qw = euler_to_quat(yaw_rad)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(target_x)
        pose.pose.position.y = float(target_y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def navigate(self, target_x, target_y, target_yaw_deg, sit_on_arrival=False, timeout_sec=120.0):
        print(f"[INFO] Navigating to target: x={target_x:+.3f}m, y={target_y:+.3f}m, yaw={target_yaw_deg:+.1f} deg")

        pose_msg = self.build_pose_stamped(target_x, target_y, target_yaw_deg)

        use_action = False
        result_future = None
        goal_handle = None

        if self._action_client is not None and self._action_client.wait_for_server(timeout_sec=2.0):
            use_action = True
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose_msg
            send_goal_future = self._action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                print("[ERROR] Goal rejected by Nav2 planner.")
                return False

            print("  goal accepted by action server.")
            result_future = goal_handle.get_result_async()
        else:
            print("  publishing goal to /goal_pose...")
            for _ in range(3):
                self.goal_pub.publish(pose_msg)
                rclpy.spin_once(self, timeout_sec=0.05)

        t_start = time.time()
        last_print = 0.0
        rx, ry, ryaw = None, None, None
        reached_radius = False

        while rclpy.ok():
            if use_action and result_future.done():
                break
            if time.time() - t_start > timeout_sec:
                print(f"\n[WARN] Navigation timeout ({timeout_sec:.0f}s).")
                break

            rclpy.spin_once(self, timeout_sec=0.1)
            rx, ry, ryaw = self.get_robot_pose(timeout_sec=0.1)
            now = time.time()

            if rx is not None and (now - last_print >= 0.4):
                dist = math.hypot(target_x - rx, target_y - ry)
                print(
                    f"\r  pose=({rx:+.2f}, {ry:+.2f}, {ryaw:+.0f} deg) | "
                    f"rem={dist:.2f}m | elapsed={now - t_start:.1f}s",
                    end='', flush=True)
                last_print = now

                if dist <= 0.20:
                    print("\n  within goal tolerance (0.20m).")
                    reached_radius = True
                    break

        print()
        action_succeeded = (
            use_action and goal_handle is not None and
            GoalStatus is not None and goal_handle.status == GoalStatus.STATUS_SUCCEEDED
        )
        success = reached_radius or action_succeeded or (
            rx is not None and math.hypot(target_x - rx, target_y - ry) <= 0.25
        )

        if success:
            print(f"[DONE] Navigation complete ({time.time() - t_start:.1f}s)")
            if sit_on_arrival:
                self.send_sport_cmd(1005)
                time.sleep(1.0)
            return True
        else:
            print("[WARN] Navigation did not complete.")
            return False


def main():
    parser = argparse.ArgumentParser(description='Go2 Nav2 goal client')
    parser.add_argument('--x', type=float, default=None, help='Target X (m)')
    parser.add_argument('--y', type=float, default=None, help='Target Y (m)')
    parser.add_argument('--yaw', type=float, default=0.0, help='Target Yaw (deg)')
    parser.add_argument('--goal', type=str, default=None, help='Target string: "x, y, yaw"')
    parser.add_argument('--sit', action='store_true', help='Stand down on arrival')
    parser.add_argument('subcommand', nargs='?', default=None, help='go, save, list, pose')
    parser.add_argument('target_name', nargs='?', default=None, help='Waypoint name')

    args = parser.parse_args()
    targets = load_targets()

    if args.subcommand == 'list':
        print(f"Waypoints ({len(targets)}): {TARGETS_FILE}")
        for k, v in targets.items():
            print(f"  - {k:<12}: x={v['x']:+.2f}m, y={v['y']:+.2f}m, yaw={v.get('yaw', 0.0):+.1f} deg")
        return

    rclpy.init()
    client = Go2GotoClient()

    try:
        if args.subcommand == 'pose':
            x, y, yaw = client.get_robot_pose(timeout_sec=3.0)
            if x is not None:
                print(f"pose: x={x:+.3f}m, y={y:+.3f}m, yaw={yaw:+.1f} deg")
            else:
                print("[ERROR] Failed to lookup TF (map/odom -> base_link).")
            return

        if args.subcommand == 'save':
            if not args.target_name:
                print("Usage: python3 scripts/go2_goto.py save <name>")
                return
            x, y, yaw = client.get_robot_pose(timeout_sec=3.0)
            if x is None:
                print("[ERROR] Failed to read current pose.")
            else:
                targets[args.target_name] = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 1)}
                save_targets(targets)
                print(f"[SAVED] '{args.target_name}': x={x:+.3f}m, y={y:+.3f}m, yaw={yaw:+.1f} deg")
            return

        tx, ty, tyaw = None, None, 0.0
        if args.subcommand == 'go' or (args.subcommand in targets):
            tname = args.target_name if args.subcommand == 'go' else args.subcommand
            if tname not in targets:
                print(f"[ERROR] Unknown waypoint '{tname}'. Available: {list(targets.keys())}")
                return
            t = targets[tname]
            tx, ty, tyaw = t['x'], t['y'], t.get('yaw', 0.0)
        elif args.goal is not None:
            parts = [float(p.strip()) for p in args.goal.split(',')]
            tx, ty = parts[0], parts[1]
            tyaw = parts[2] if len(parts) > 2 else 0.0
        elif args.x is not None and args.y is not None:
            tx, ty, tyaw = args.x, args.y, args.yaw

        if tx is None or ty is None:
            parser.print_help()
            return

        client.navigate(tx, ty, tyaw, sit_on_arrival=args.sit)

    except KeyboardInterrupt:
        print("\n[INFO] Navigation cancelled.")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
