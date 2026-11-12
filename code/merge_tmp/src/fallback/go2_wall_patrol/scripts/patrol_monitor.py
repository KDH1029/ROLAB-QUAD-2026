#!/usr/bin/env python3
import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Color:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN = "\033[36m"
    WHITE = "\033[37m"
    BG_DARK = "\033[40m"

class PatrolMonitor(Node):
    def __init__(self):
        super().__init__('patrol_monitor')
        self.sub_status = self.create_subscription(
            String,
            '/patrol/status',
            self.status_callback,
            10
        )
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.cmd_pub = self.create_publisher(String, '/patrol/cmd', 10)

        self.last_status = None
        self.last_cmd_vel = Twist()
        self.update_count = 0

        # Render timer (10Hz)
        self.timer = self.create_timer(0.1, self.render_dashboard)

    def status_callback(self, msg):
        try:
            self.last_status = json.loads(msg.data)
            self.update_count += 1
        except Exception:
            pass

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg

    def render_dashboard(self):
        if not self.last_status:
            sys.stdout.write("\033[2J\033[H")
            print(f"{Color.YELLOW}[PatrolMonitor] Waiting for /patrol/status topic...{Color.RESET}")
            return

        s = self.last_status
        state = s.get("state", "UNKNOWN")
        path = s.get("current_path", "N/A")
        corner = s.get("target_corner", "N/A")
        d_left = s.get("d_left", -1.0)
        d_front = s.get("d_front", 99.0)
        yaw = s.get("yaw_deg", 0.0)
        target_yaw = s.get("target_yaw_deg", 0.0)
        yaw_err = yaw - target_yaw
        while yaw_err > 180.0: yaw_err -= 360.0
        while yaw_err < -180.0: yaw_err += 360.0

        vx = s.get("vx", 0.0)
        vy = s.get("vy", 0.0)
        wz = s.get("wz", 0.0)
        tl = s.get("traffic_light", "N/A")
        left_inliers = s.get("left_inliers", 0)
        front_inliers = s.get("front_inliers", 0)
        laps = s.get("laps", 0)
        elapsed = float(s.get("elapsed_sec", 0.0))
        cycle_rem = float(s.get("traffic_cycle_rem", 0.0))

        # Color badges
        state_color = Color.GREEN if "DRIV" in state else (Color.YELLOW if "ALIGN" in state else Color.MAGENTA)
        tl_color = Color.GREEN if "GREEN" in tl else (Color.RED if "RED" in tl else Color.CYAN)

        # ANSI clear screen and home cursor
        sys.stdout.write("\033[2J\033[H")

        print(f"{Color.BOLD}======================================================================{Color.RESET}")
        print(f"{Color.CYAN}{Color.BOLD}  Unitree Go2 CW Patrol Live Debug Dashboard  (Loop # {laps}){Color.RESET}")
        print(f"{Color.BOLD}======================================================================{Color.RESET}")
        print(f"  {Color.BOLD}FSM State      :{Color.RESET} {state_color}{Color.BOLD}[{state}]{Color.RESET}")
        print(f"  {Color.BOLD}Current Course :{Color.RESET} {Color.CYAN}{path}{Color.RESET}  ➔  Target Corner: {Color.YELLOW}{corner}{Color.RESET}")
        tl_info = f"{tl_color}{Color.BOLD}[{tl}]{Color.RESET}"
        if elapsed > 0.0:
            tl_info += f"  (신호 남은시간: {cycle_rem:4.1f}s | 글로벌 경과: {elapsed:5.1f}s)"
        print(f"  {Color.BOLD}Traffic Light  :{Color.RESET} {tl_info}")
        print(f"----------------------------------------------------------------------")
        print(f"  {Color.BOLD}[LiDAR 2D RANSAC Wall Tracking]{Color.RESET}")
        print(f"    Left Wall  : {Color.GREEN}{d_left:6.2f} m{Color.RESET}  (Inliers: {left_inliers:3d})")
        print(f"    Front Wall : {Color.RED}{d_front:6.2f} m{Color.RESET}  (Inliers: {front_inliers:3d})")
        print(f"----------------------------------------------------------------------")
        print(f"  {Color.BOLD}[IMU Orientation Lock]{Color.RESET}")
        print(f"    Current Yaw: {yaw:6.1f}°  |  Target Yaw: {target_yaw:6.1f}°  |  Error: {Color.YELLOW}{yaw_err:+6.1f}°{Color.RESET}")
        print(f"----------------------------------------------------------------------")
        print(f"  {Color.BOLD}[Velocity Command (/cmd_vel)]{Color.RESET}")
        print(f"    Vx (Forward): {vx:+5.2f} m/s | Vy (Lateral): {vy:+5.2f} m/s | Wz (Yaw): {wz:+5.2f} rad/s")
        print(f"======================================================================\n")
        print(f"{Color.WHITE}Quick Commands (Open another terminal):{Color.RESET}")
        print(f"  ros2 topic pub --once /patrol/cmd std_msgs/msg/String \"{{data: 'STOP'}}\"")
        print(f"  ros2 topic pub --once /patrol/cmd std_msgs/msg/String \"{{data: 'START'}}\"")
        print(f"  ros2 topic pub --once /patrol/cmd std_msgs/msg/String \"{{data: 'RESET'}}\"")

def main(args=None):
    rclpy.init(args=args)
    monitor = PatrolMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

