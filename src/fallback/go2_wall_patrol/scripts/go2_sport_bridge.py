#!/usr/bin/env python3
"""
Unitree Go2 Sport Mode Bridge Node
----------------------------------
Subscribes to:
  - /patrol/posture (std_msgs/msg/String): "SIT" or "STAND"

Commands Unitree Go2 to:
  - SIT   -> SportClient.StandDown()  (or /api/sport/request api_id=1005)
  - STAND -> SportClient.StandUp()    (or /api/sport/request api_id=1004)

Works with either:
  1) Unitree SDK2 Python (unitree_sdk2py) pre-installed on Go2 Jetson
  2) unitree_api ROS 2 topic (/api/sport/request)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Try importing unitree_sdk2py (pre-installed on Go2 onboard Jetson)
SDK2_AVAILABLE = False
try:
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    SDK2_AVAILABLE = True
except ImportError:
    SDK2_AVAILABLE = False

# Try importing unitree_api ROS 2 message
UNITREE_API_AVAILABLE = False
try:
    from unitree_api.msg import Request
    UNITREE_API_AVAILABLE = True
except ImportError:
    UNITREE_API_AVAILABLE = False


class Go2SportBridge(Node):
    def __init__(self):
        super().__init__('go2_sport_bridge')

        self.sub_posture = self.create_subscription(
            String,
            '/patrol/posture',
            self.posture_callback,
            10
        )

        self.sub_motion = self.create_subscription(
            String,
            '/robot/motion_cmd',
            self.motion_callback,
            10
        )

        self.sub_cmd = self.create_subscription(
            String,
            '/patrol/cmd',
            self.cmd_callback,
            10
        )

        self._balance_timer = None

        # Optional ROS 2 unitree_api publisher
        if UNITREE_API_AVAILABLE:
            self.sport_req_pub = self.create_publisher(Request, '/api/sport/request', 10)
            self.get_logger().info("[Bridge] unitree_api ROS 2 topic publisher enabled (/api/sport/request)")

        # Unitree SDK2 Client
        self.sport_client = None
        if SDK2_AVAILABLE:
            try:
                self.sport_client = SportClient()
                self.sport_client.SetTimeout(5.0)
                self.sport_client.Init()
                self.get_logger().info("[Bridge] unitree_sdk2py SportClient initialized successfully!")
            except Exception as e:
                self.get_logger().warn(f"[Bridge] Failed to init SportClient: {e}")

        if not SDK2_AVAILABLE and not UNITREE_API_AVAILABLE:
            self.get_logger().warn(
                "[Bridge] Neither unitree_sdk2py nor unitree_api found in this environment. "
                "Running in Simulation/Log-only mode."
            )

    def cmd_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd in ["DOWN", "STAND_DOWN", "SIT"]:
            self.execute_stand_down()
        elif cmd in ["STAND", "STAND_UP", "UP"]:
            self.execute_stand()

    def motion_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        if "DOWN" in cmd or "SIT" in cmd:
            self.execute_stand_down()
        elif "UP" in cmd or "STAND" in cmd:
            self.execute_stand()

    def posture_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f"[Bridge] Received posture command: {cmd}")

        if cmd in ["DOWN", "STAND_DOWN", "SIT"]:
            self.execute_stand_down()
        elif cmd in ["STAND", "STAND_UP", "UP"]:
            self.execute_stand()

    def execute_stand_down(self):
        self.get_logger().info(">>> 🧎 Executing Go2 STAND_DOWN (API 1005 - 엎드리기) <<<")
        self.is_sitting = False
        if self._balance_timer is not None:
            self._balance_timer.cancel()
            self._balance_timer = None

        if UNITREE_API_AVAILABLE and hasattr(self, 'sport_req_pub'):
            req = Request()
            req.header.identity.api_id = 1005  # SPORT_CMD_STANDDOWN
            req.header.lease.id = 0
            req.parameter = ""
            self.sport_req_pub.publish(req)
            self.get_logger().info("[unitree_api] Published api_id=1005 (StandDown)")

    def execute_sit(self):
        self.get_logger().info(">>> 🧎 Executing Go2 STAND_DOWN (API 1005 - 엎드리기) <<<")
        self.execute_stand_down()

    def execute_stand(self):
        self.get_logger().info(">>> 🐕 Executing Go2 STAND_UP (API 1004 - 일어서기) <<<")
        if self._balance_timer is not None:
            self._balance_timer.cancel()
            self._balance_timer = None

        # Method 1: SDK2 Python
        if self.sport_client:
            try:
                self.sport_client.StandUp()
                self.get_logger().info("[SDK2] sport_client.StandUp() sent!")
            except Exception as e:
                self.get_logger().error(f"[SDK2] StandUp error: {e}")

        # Method 2: unitree_api ROS 2 message (API 1004 StandUp)
        if UNITREE_API_AVAILABLE and hasattr(self, 'sport_req_pub'):
            req = Request()
            req.header.identity.api_id = 1004  # StandUp
            req.header.lease.id = 0
            req.parameter = ""
            self.sport_req_pub.publish(req)
            self.is_sitting = False
            self.get_logger().info("[unitree_api] Published api_id=1004 (StandUp)")

        # Automatically schedule BalanceStand (API 1002) after 1.8s
        self._balance_timer = self.create_timer(1.8, self._balance_stand_timer_cb)

    def _balance_stand_timer_cb(self):
        if self._balance_timer is not None:
            self._balance_timer.cancel()
            self._balance_timer = None

        self.get_logger().info(">>> ⚖️ Executing Go2 BalanceStand (API 1002 - 보행 모드 활성화) <<<")
        if UNITREE_API_AVAILABLE and hasattr(self, 'sport_req_pub'):
            req = Request()
            req.header.identity.api_id = 1002  # BalanceStand
            req.header.lease.id = 0
            req.parameter = ""
            self.sport_req_pub.publish(req)
            self.get_logger().info("[unitree_api] Published api_id=1002 (BalanceStand - Gait Ready)")


def main(args=None):
    rclpy.init(args=args)
    bridge = Go2SportBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

