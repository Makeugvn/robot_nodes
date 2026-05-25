#!/usr/bin/env python3
"""
lidar_serial_node.py
====================
Baca data JSON dari Arduino (TFMini + servo) via Serial,
publish sebagai sensor_msgs/LaserScan ke /scan.

Format JSON dari Arduino:
  {"distances":[d0, d1, ..., d180]}
  dimana d = jarak dalam cm, -1 = gagal baca
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import json
import math
import time


class LidarSerialNode(Node):
    def __init__(self):
        super().__init__('lidar_serial_node')

        # ── Parameter ──────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('range_min', 0.10)   # meter
        self.declare_parameter('range_max', 12.0)   # meter (TFMini Plus max)
        self.declare_parameter('mount_angle_deg', 0.0)  # offset rotasi sensor jika miring

        port      = self.get_parameter('serial_port').value
        baud      = self.get_parameter('baud_rate').value
        self.frame_id  = self.get_parameter('frame_id').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        mount_deg      = self.get_parameter('mount_angle_deg').value
        self.mount_rad = math.radians(mount_deg)

        # ── Publisher ──────────────────────────────────────
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        # ── Buka Serial ────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=2.0)
            time.sleep(2.0)  # tunggu Arduino reset setelah koneksi
            self.ser.flushInput()
            self.get_logger().info(f'Serial terbuka: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Gagal buka serial {port}: {e}')
            self.get_logger().error('Cek port dengan: ls /dev/ttyUSB* atau ls /dev/ttyACM*')
            raise

        # ── Timer baca serial ──────────────────────────────
        # Baca non-blocking via timer 10ms
        self.create_timer(0.01, self.read_serial)
        self.scan_count = 0

        self.get_logger().info('lidar_serial_node siap — menunggu data dari Arduino...')

    def read_serial(self):
        """Baca satu baris dari serial, parse JSON, publish LaserScan."""
        if not self.ser.in_waiting:
            return

        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception as e:
            self.get_logger().warn(f'Error baca serial: {e}')
            return

        if not line.startswith('{'):
            # Abaikan baris non-JSON (misal status "ready")
            if line:
                self.get_logger().debug(f'Non-JSON: {line}')
            return

        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            self.get_logger().warn(f'JSON invalid: {line[:60]}')
            return

        if 'distances' not in data:
            return

        distances_cm = data['distances']
        n = len(distances_cm)

        if n < 2:
            self.get_logger().warn(f'Data terlalu pendek: {n} titik')
            return

        # ── Bangun LaserScan ────────────────────────────────
        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Servo sweep 0°→180°, tapi laser menghadap depan robot di 90°
        # Konvensi ROS: angle_min ke angle_max, CCW positif
        # Servo 0°   = kanan robot  = -π/2
        # Servo 90°  = depan robot  =  0
        # Servo 180° = kiri robot   = +π/2
        msg.angle_min       = -math.pi / 2 + self.mount_rad   # -90°
        msg.angle_max       =  math.pi / 2 + self.mount_rad   # +90°
        msg.angle_increment = math.pi / (n - 1)               # π/180 per step
        msg.range_min       = self.range_min
        msg.range_max       = self.range_max

        # Waktu satu sweep: 181 step × 30ms = ~5.4 detik
        msg.scan_time       = 5.4
        msg.time_increment  = msg.scan_time / (n - 1)

        # ── Konversi cm → meter, handle nilai invalid ───────
        ranges = []
        for d_cm in distances_cm:
            if d_cm <= 0:
                # Gagal baca → inf (artinya tidak ada obstacle)
                ranges.append(float('inf'))
            else:
                d_m = d_cm / 100.0
                if d_m < self.range_min or d_m > self.range_max:
                    ranges.append(float('inf'))
                else:
                    ranges.append(d_m)

        msg.ranges = ranges

        self.pub.publish(msg)
        self.scan_count += 1

        self.get_logger().info(
            f'Scan #{self.scan_count} dipublish — {n} titik, '
            f'min={min(r for r in ranges if r != float("inf") and not math.isinf(r)) if any(not math.isinf(r) for r in ranges) else "N/A":.2f}m',
            throttle_duration_sec=2.0
        )

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial ditutup')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LidarSerialNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()