#!/usr/bin/env python3
"""
Closed-loop kinematics and odometry simulator for Unitree Go2.

Publishes:
  - /utlidar/robot_odom (50Hz)
  - /lowstate (50Hz, 12-joint states, foot contact forces, IMU with gait impact noise)
  - /scan (20Hz)
Subscribes:
  - /api/sport/request, /cmd_vel, /goal_pose
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

try:
    from unitree_go.msg import LowState, MotorState
    from unitree_api.msg import Request
    HAS_UNITREE = True
except ImportError:
    HAS_UNITREE = False


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Go2RobotSimulator(Node):
    def __init__(self, use_cpp_bridges=True):
        super().__init__('go2_robot_simulator')
        self.use_cpp_bridges = use_cpp_bridges

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.gait_phase = 0.0

        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None

        self.br = TransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, '/utlidar/robot_odom', 20)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        if HAS_UNITREE:
            sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.lowstate_pub = self.create_publisher(LowState, '/lowstate', sensor_qos)
            self.sport_sub = self.create_subscription(
                Request, '/api/sport/request', self.on_sport_req, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.on_goal_pose, 10)

        self.last_t = time.time()
        self.create_timer(1.0 / 50.0, self.update_motion)
        self.create_timer(1.0 / 20.0, self.publish_scan)
        self.create_timer(1.0 / 20.0, self.update_controller)

        self.get_logger().info(
            f"simulator started (use_cpp_bridges={self.use_cpp_bridges})")

    def on_goal_pose(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_yaw = quat_to_yaw(msg.pose.orientation)
        self.get_logger().info(
            f"received goal_pose: x={self.goal_x:.2f}, y={self.goal_y:.2f}, "
            f"yaw={math.degrees(self.goal_yaw):.1f} deg")

    def on_cmd_vel(self, msg: Twist):
        if not self.use_cpp_bridges or not HAS_UNITREE:
            self.vx = msg.linear.x
            self.vy = msg.linear.y
            self.wz = msg.angular.z

    def on_sport_req(self, msg):
        if msg.header.identity.api_id == 1008:
            try:
                params = json.loads(msg.parameter)
                self.vx = float(params.get('x', 0.0))
                self.vy = float(params.get('y', 0.0))
                self.wz = float(params.get('z', 0.0))
            except Exception:
                pass
        elif msg.header.identity.api_id in (1003, 1005):
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0

    def update_controller(self):
        if self.goal_x is None or self.goal_y is None:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)

        cmd = Twist()
        if dist > 0.12:
            target_heading = math.atan2(dy, dx)
            heading_err = normalize_angle(target_heading - self.yaw)
            if abs(heading_err) > 0.45:
                cmd.linear.x = 0.0
                cmd.angular.z = max(-1.2, min(1.2, 2.0 * heading_err))
            else:
                cmd.linear.x = min(0.65, max(0.15, 0.8 * dist))
                cmd.angular.z = max(-1.0, min(1.0, 1.8 * heading_err))
        else:
            yaw_err = normalize_angle(self.goal_yaw - self.yaw)
            if abs(yaw_err) > 0.08:
                cmd.angular.z = max(-0.8, min(0.8, 1.5 * yaw_err))
            else:
                self.goal_x = None
                self.goal_y = None

        self.cmd_pub.publish(cmd)

    def update_motion(self):
        now = time.time()
        dt = now - self.last_t
        self.last_t = now

        self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        self.yaw += self.wz * dt

        is_walking = (math.hypot(self.vx, self.vy) > 0.02 or abs(self.wz) > 0.05)
        if is_walking:
            self.gait_phase += dt * 3.0 * 2.0 * math.pi

        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        if HAS_UNITREE:
            ls = LowState()
            impact_jitter = 0.05 * math.sin(self.gait_phase * 2.0) if is_walking else 0.0
            ls.imu_state.quaternion = [1.0, 0.0, 0.0, 0.0]
            ls.imu_state.gyroscope = [0.0, 0.0, float(self.wz)]
            ls.imu_state.accelerometer = [0.0, 0.0, float(9.81 + impact_jitter * 30.0)]
            ls.imu_state.rpy = [float(impact_jitter), float(-impact_jitter * 0.8), float(self.yaw)]

            s_val = math.sin(self.gait_phase) if is_walking else 0.0
            for leg in range(4):
                in_stance = True
                if is_walking:
                    in_stance = (s_val >= 0.0) if leg in (0, 3) else (s_val < 0.0)
                ls.foot_force[leg] = 60 if in_stance else 0

                m_hip = MotorState()
                m_thigh = MotorState()
                m_calf = MotorState()
                m_hip.q = 0.0
                m_thigh.q = 0.67 if in_stance else 0.85
                m_calf.q = -1.30 if in_stance else -1.60
                ls.motor_state[leg * 3 + 0] = m_hip
                ls.motor_state[leg * 3 + 1] = m_thigh
                ls.motor_state[leg * 3 + 2] = m_calf

            self.lowstate_pub.publish(ls)

        t_map = TransformStamped()
        t_map.header.stamp = stamp
        t_map.header.frame_id = 'map'
        t_map.child_frame_id = 'odom'
        t_map.transform.rotation.w = 1.0
        self.br.sendTransform(t_map)

        if not self.use_cpp_bridges:
            t_odom = TransformStamped()
            t_odom.header.stamp = stamp
            t_odom.header.frame_id = 'odom'
            t_odom.child_frame_id = 'base_link'
            t_odom.transform.translation.x = self.x
            t_odom.transform.translation.y = self.y
            t_odom.transform.rotation.z = qz
            t_odom.transform.rotation.w = qw
            self.br.sendTransform(t_odom)

            t_stab = TransformStamped()
            t_stab.header.stamp = stamp
            t_stab.header.frame_id = 'base_link'
            t_stab.child_frame_id = 'base_stab'
            t_stab.transform.rotation.w = 1.0
            self.br.sendTransform(t_stab)

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_stab'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / 360.0
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        scan.range_min = 0.40
        scan.range_max = 12.0
        scan.ranges = [8.0] * 360
        self.scan_pub.publish(scan)


def run_sim_node():
    rclpy.init()
    node = Go2RobotSimulator(use_cpp_bridges=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run_sim_node()
