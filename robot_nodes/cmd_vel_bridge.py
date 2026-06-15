#!/usr/bin/env python3
"""
cmd_vel_bridge.py
=================
Subscribe /cmd_vel dari Nav2
Forward ke ESP32 via UDP port 5007
"""

import rclpy
import json
import socket
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_system_default

ESP32_IP   = '192.168.100.94'
ESP32_PORT = 5007


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('esp32_ip',   ESP32_IP)
        self.declare_parameter('esp32_port', ESP32_PORT)

        self.ip   = self.get_parameter('esp32_ip').value
        self.port = self.get_parameter('esp32_port').value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.create_subscription(
            Twist, '/cmd_vel', self._cb, qos_profile_system_default)

        self.get_logger().info(
            f'cmd_vel_bridge siap → {self.ip}:{self.port}')

    def _cb(self, msg: Twist):
        payload = json.dumps({
            'lx': round(msg.linear.x,  4),
            'az': round(msg.angular.z, 4),
        })
        try:
            self.sock.sendto(payload.encode(), (self.ip, self.port))
        except Exception as e:
            self.get_logger().warn(f'UDP send error: {e}')

    def destroy_node(self):
        # Kirim stop saat shutdown
        try:
            self.sock.sendto(b'{"lx":0.0,"az":0.0}', (self.ip, self.port))
        except Exception:
            pass
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CmdVelBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()