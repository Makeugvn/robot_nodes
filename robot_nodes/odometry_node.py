#!/usr/bin/env python3
"""
odometry_node.py
================
Terima JSON sensor dari ESP32 via UDP port 5050
Publish:
  /odom   → nav_msgs/Odometry   (untuk slam_toolbox dan Nav2)
  /imu    → sensor_msgs/Imu     (untuk robot_localization EKF)
  /tf     → odom → base_footprint transform
"""

import rclpy
import math
import json
import socket
import threading
from rclpy.node import Node
from nav_msgs.msg        import Odometry
from sensor_msgs.msg     import Imu
from geometry_msgs.msg   import TransformStamped, Quaternion
from rclpy.qos           import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf2_ros


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Konversi Euler (radian) ke quaternion."""
    cy = math.cos(yaw   * 0.5)
    sy = math.sin(yaw   * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll  * 0.5)
    sr = math.sin(roll  * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # ── Parameter ──────────────────────────────────────
        self.declare_parameter('port_sensor',  5050)
        self.declare_parameter('base_frame',   'base_footprint')
        self.declare_parameter('odom_frame',   'odom')
        self.declare_parameter('imu_frame',    'imu_link')

        self.port     = self.get_parameter('port_sensor').value
        self.base_frm = self.get_parameter('base_frame').value
        self.odom_frm = self.get_parameter('odom_frame').value
        self.imu_frm  = self.get_parameter('imu_frame').value

        # ── QoS BEST_EFFORT ────────────────────────────────
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # ── Publisher ──────────────────────────────────────
        self.pub_odom = self.create_publisher(Odometry, '/odom', qos)
        self.pub_imu  = self.create_publisher(Imu,      '/imu',  qos)

        # ── TF broadcaster ─────────────────────────────────
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── UDP socket ─────────────────────────────────────
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.settimeout(1.0)

        # ── State ──────────────────────────────────────────
        self.pkt_count = 0
        self._running  = True

        # ── Thread ─────────────────────────────────────────
        self._thread = threading.Thread(
            target=self._recv_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'odometry_node siap, listen port {self.port}')

    # ══════════════════════════════════════════════════════
    #  UDP RECEIVE LOOP
    # ══════════════════════════════════════════════════════
    def _recv_loop(self):
        while self._running:
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    self.get_logger().warn(f'UDP error: {e}')
                continue

            try:
                d = json.loads(data.decode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f'JSON error: {e} | raw: {data[:60]}')
                continue

            try:
                self._publish_all(d)
            except Exception as e:
                self.get_logger().error(f'Publish error: {e}')

    # ══════════════════════════════════════════════════════
    #  PUBLISH /odom, /imu, dan TF
    # ══════════════════════════════════════════════════════
    def _publish_all(self, d: dict):
        now = self.get_clock().now().to_msg()
        self.pkt_count += 1

        # ── Quaternion dari fused yaw ───────────────────────
        roll  = float(d.get('roll',       0.0))
        pitch = float(d.get('pitch',      0.0))
        yaw   = float(d.get('yaw',        0.0))
        theta = float(d.get('odom_theta', 0.0))

        q_imu  = euler_to_quaternion(roll, pitch, yaw)
        q_odom = euler_to_quaternion(0.0, 0.0, theta)

        # ════════════════════════════════════════
        #  /odom — nav_msgs/Odometry
        # ════════════════════════════════════════
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.odom_frm
        odom.child_frame_id  = self.base_frm

        odom.pose.pose.position.x  = float(d.get('odom_x', 0.0))
        odom.pose.pose.position.y  = float(d.get('odom_y', 0.0))
        odom.pose.pose.position.z  = 0.0
        odom.pose.pose.orientation = q_odom

        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[35] = 0.05   # yaw

        odom.twist.twist.linear.x  = float(d.get('odom_vx', 0.0))
        odom.twist.twist.angular.z = float(d.get('odom_wz', 0.0))
        odom.twist.covariance[0]   = 0.01
        odom.twist.covariance[35]  = 0.05

        self.pub_odom.publish(odom)

        # ════════════════════════════════════════
        #  TF: odom → base_footprint
        # ════════════════════════════════════════
        tf = TransformStamped()
        tf.header.stamp            = now
        tf.header.frame_id         = self.odom_frm
        tf.child_frame_id          = self.base_frm
        tf.transform.translation.x = float(d.get('odom_x', 0.0))
        tf.transform.translation.y = float(d.get('odom_y', 0.0))
        tf.transform.translation.z = 0.0
        tf.transform.rotation      = q_odom
        self.tf_broadcaster.sendTransform(tf)

        # ════════════════════════════════════════
        #  /imu — sensor_msgs/Imu
        # ════════════════════════════════════════
        imu = Imu()
        imu.header.stamp    = now
        imu.header.frame_id = self.imu_frm

        imu.orientation = q_imu
        imu.orientation_covariance = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.05
        ]

        imu.angular_velocity.x = float(d.get('gx', 0.0))
        imu.angular_velocity.y = float(d.get('gy', 0.0))
        imu.angular_velocity.z = float(d.get('gz', 0.0))
        imu.angular_velocity_covariance = [
            0.001, 0.0,   0.0,
            0.0,   0.001, 0.0,
            0.0,   0.0,   0.001
        ]

        imu.linear_acceleration.x = float(d.get('ax', 0.0))
        imu.linear_acceleration.y = float(d.get('ay', 0.0))
        imu.linear_acceleration.z = float(d.get('az', 9.81))
        imu.linear_acceleration_covariance = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01
        ]

        self.pub_imu.publish(imu)

        # Log setiap 100 paket
        if self.pkt_count % 100 == 0:
            self.get_logger().info(
                f'#{self.pkt_count} | '
                f'Odom: x={d.get("odom_x",0):.2f}m '
                f'y={d.get("odom_y",0):.2f}m '
                f'θ={math.degrees(theta):.1f}° | '
                f'IMU gz={d.get("gz",0):.3f}rad/s | '
                f'MAG={math.degrees(d.get("heading",0)):.1f}°'
            )

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=2.0)
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OdometryNode()
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