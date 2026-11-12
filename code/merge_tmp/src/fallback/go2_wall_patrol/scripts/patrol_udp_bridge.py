#!/usr/bin/env python3
import os
import socket
import json
import threading
import time
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class PatrolUdpBridge(Node):
    def __init__(self):
        super().__init__('patrol_udp_bridge')
        self.declare_parameter('udp_port', 9880)
        self.port = self.get_parameter('udp_port').value

        self.sub_status = self.create_subscription(String, '/patrol/status', self.cb_status, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
        self.pub_cmd = self.create_publisher(String, '/patrol/cmd', 10)

        self.latest_status_raw = "{}"
        self.latest_status_dict = {}
        self.last_cmd_vel = {"vx": 0.0, "vy": 0.0, "wz": 0.0}

        self.last_client_addr = None
        self.client_lock = threading.Lock()

        # UDP Socket initialization
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.settimeout(0.2)

        self.running = True
        self.thread_recv = threading.Thread(target=self.udp_recv_loop, daemon=True)
        self.thread_stream = threading.Thread(target=self.udp_stream_loop, daemon=True)

        self.thread_recv.start()
        self.thread_stream.start()

        self.get_logger().info(f"[Patrol UDP Bridge] Listening on 0.0.0.0:{self.port} (Telemetry & Controls)")

    def cb_status(self, msg: String):
        self.latest_status_raw = msg.data
        try:
            self.latest_status_dict = json.loads(msg.data)
            # Event-driven immediate push to PC client (Zero latency!)
            with self.client_lock:
                target = self.last_client_addr
            if target is not None:
                payload = self.build_telemetry_payload()
                self.sock.sendto(payload.encode('utf-8'), target)
        except Exception:
            pass

    def cb_cmd_vel(self, msg: Twist):
        self.last_cmd_vel = {
            "vx": round(msg.linear.x, 3),
            "vy": round(msg.linear.y, 3),
            "wz": round(msg.angular.z, 3)
        }

    def udp_recv_loop(self):
        while self.running and rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(4096)
                if not data:
                    continue

                with self.client_lock:
                    self.last_client_addr = addr

                text = data.decode('utf-8', errors='ignore').strip()
                if not text:
                    continue

                # Handle JSON or direct text commands
                cmd_to_publish = None
                req_json = None
                try:
                    req_json = json.loads(text)
                    if isinstance(req_json, dict) and "cmd" in req_json:
                        cmd_name = str(req_json["cmd"]).upper()
                        if cmd_name == "SET_PARAM":
                            params = req_json.get("params", {})
                            if isinstance(params, dict):
                                kv_pairs = [f"{k}={v}" for k, v in params.items()]
                                cmd_to_publish = f"SET_PARAM:{','.join(kv_pairs)}"
                                self.save_params_to_yaml(params)
                        else:
                            val = req_json.get("val", req_json.get("arg", ""))
                            if val != "":
                                cmd_to_publish = f"{cmd_name} {val}".strip()
                            else:
                                cmd_to_publish = cmd_name
                except Exception:
                    cmd_to_publish = text.upper()

                if cmd_to_publish:
                    if not (cmd_to_publish.startswith("POLL") or cmd_to_publish.startswith("PING")):
                        msg = String()
                        msg.data = cmd_to_publish
                        self.pub_cmd.publish(msg)
                        self.get_logger().info(f"[UDP -> ROS] Forwarded command '{cmd_to_publish}' to /patrol/cmd")

                    # Immediate response with current telemetry
                    resp = self.build_telemetry_payload()
                    self.sock.sendto(resp.encode('utf-8'), addr)

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.get_logger().warn(f"[UDP Recv Error] {e}")
                time.sleep(0.05)

    def save_params_to_yaml(self, params: dict):
        yaml_paths = [
            os.path.expanduser("~/go2_patrol_ws/src/go2_wall_patrol/config/patrol_params.yaml"),
            os.path.expanduser("~/go2_patrol_ws/install/go2_wall_patrol/share/go2_wall_patrol/config/patrol_params.yaml"),
        ]

        mapping = {
            "path_1_left_dist": ("path_1", "target_left_dist"),
            "path_1_front_dist": ("path_1", "front_trigger_dist"),
            "path_1_yaw": ("path_1", "drive_yaw_deg"),
            "path_2_left_dist": ("path_2", "target_left_dist"),
            "path_2_front_dist": ("path_2", "front_trigger_dist"),
            "path_2_yaw": ("path_2", "drive_yaw_deg"),
            "path_3_left_dist": ("path_3", "target_left_dist"),
            "path_3_front_dist": ("path_3", "front_trigger_dist"),
            "path_3_yaw": ("path_3", "drive_yaw_deg"),
            "path_4_left_dist": ("path_4", "target_left_dist"),
            "path_4_front_dist": ("path_4", "front_trigger_dist"),
            "path_4_yaw": ("path_4", "drive_yaw_deg"),
            "corner_a_front_dist": ("corner_a", "target_front_dist"),
            "corner_a_left_dist": ("corner_a", "target_left_dist"),
            "corner_a_yaw": ("corner_a", "target_yaw_deg"),
            "corner_b_front_dist": ("corner_b", "target_front_dist"),
            "corner_b_left_dist": ("corner_b", "target_left_dist"),
            "corner_b_yaw": ("corner_b", "target_yaw_deg"),
            "corner_c_front_dist": ("corner_c", "target_front_dist"),
            "corner_c_left_dist": ("corner_c", "target_left_dist"),
            "corner_c_yaw": ("corner_c", "target_yaw_deg"),
            "corner_d_front_dist": ("corner_d", "target_front_dist"),
            "corner_d_left_dist": ("corner_d", "target_left_dist"),
            "corner_d_yaw": ("corner_d", "target_yaw_deg"),
        }

        for p in yaml_paths:
            if not os.path.exists(p):
                continue
            try:
                with open(p, 'r', encoding='utf-8') as f:
                    doc = yaml.safe_load(f) or {}

                ros_params = doc.setdefault("/**", {}).setdefault("ros__parameters", {})

                for k, v in params.items():
                    if k in ["target_left_dist", "front_trigger_dist"]:
                        continue  # Skip legacy defaults to prevent overwriting paths 1-4
                    if k in mapping:
                        section, subkey = mapping[k]
                        sub_dict = ros_params.setdefault(section, {})
                        sub_dict[subkey] = float(v) if not isinstance(v, bool) else v
                    else:
                        if k in ["enable_ring_filter", "enable_start_sit"]:
                            ros_params[k] = bool(float(v) > 0.5) if not isinstance(v, bool) else v
                        elif k in ["ring_min", "ring_max", "ransac_iterations", "ransac_min_inliers"]:
                            try:
                                ros_params[k] = int(round(float(v)))
                            except Exception:
                                ros_params[k] = int(v)
                        elif isinstance(v, (int, float, bool, str)):
                            ros_params[k] = v
                        else:
                            try:
                                ros_params[k] = float(v)
                            except Exception:
                                ros_params[k] = v

                with open(p, 'w', encoding='utf-8') as f:
                    yaml.dump(doc, f, sort_keys=False, default_flow_style=False, allow_unicode=True)

                self.get_logger().info(f"[YAML Persist] Saved parameters to {p}")
            except Exception as e:
                self.get_logger().error(f"[YAML Persist Error] Failed to write {p}: {e}")

    def build_telemetry_payload(self):
        payload = dict(self.latest_status_dict)
        payload["cmd_vel"] = self.last_cmd_vel
        payload["server_time"] = time.time()
        return json.dumps(payload)

    def udp_stream_loop(self):
        """Streams telemetry at 10Hz to the last active PC client"""
        while self.running and rclpy.ok():
            time.sleep(0.10)
            target = None
            with self.client_lock:
                target = self.last_client_addr

            if target is not None and self.latest_status_dict:
                try:
                    payload = self.build_telemetry_payload()
                    self.sock.sendto(payload.encode('utf-8'), target)
                except Exception:
                    pass

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PatrolUdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
