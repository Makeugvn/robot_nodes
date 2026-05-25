#!/usr/bin/env python3
"""
lidar_serial_wifi.py
=====================
Terima data scan TFMini dari ESP32 via UDP WiFi,
publish sebagai sensor_msgs/LaserScan ke /scan.

Format UDP dari ESP32:
  {"seq":N,"distances":[d0, d1, ..., d180]}
  dimana d = jarak dalam cm, -1 = gagal baca

Cara jalankan:
  ros2 run robot_nodes lidar_serial_wifi.py

Atau dengan parameter custom:
  ros2 run robot_nodes lidar_serial_wifi.py \
    --ros-args -p udp_port:=5005 -p range_max:=12.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import socket
import json
import math
import threading
import time


class LidarWifiNode(Node):
    def __init__(self):
        super().__init__('lidar_wifi_node')

        # ── Parameter ──────────────────────────────────────
        self.declare_parameter('udp_port',        5005)
        self.declare_parameter('udp_host',        '0.0.0.0')  # listen semua interface
        self.declare_parameter('frame_id',        'laser')
        self.declare_parameter('range_min',       0.10)   # meter
        self.declare_parameter('range_max',       12.0)   # meter
        self.declare_parameter('angle_min_deg',  -90.0)   # servo 0°   = kanan = -90°
        self.declare_parameter('angle_max_deg',   90.0)   # servo 180° = kiri  = +90°
        self.declare_parameter('scan_time',        5.4)   # detik per sweep (181 × 30ms)
        self.declare_parameter('invert_scan',     False)  # balik arah jika sensor dipasang terbalik

        self.udp_port   = self.get_parameter('udp_port').value
        self.udp_host   = self.get_parameter('udp_host').value
        self.frame_id   = self.get_parameter('frame_id').value
        self.range_min  = self.get_parameter('range_min').value
        self.range_max  = self.get_parameter('range_max').value
        self.scan_time  = self.get_parameter('scan_time').value
        self.invert     = self.get_parameter('invert_scan').value

        angle_min_deg = self.get_parameter('angle_min_deg').value
        angle_max_deg = self.get_parameter('angle_max_deg').value
        self.angle_min = math.radians(angle_min_deg)
        self.angle_max = math.radians(angle_max_deg)

        # ── Publisher ──────────────────────────────────────
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        # ── UDP socket ─────────────────────────────────────
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.udp_host, self.udp_port))
        self.sock.settimeout(1.0)  # timeout agar thread bisa berhenti

        self.get_logger().info(
            f'UDP listener: {self.udp_host}:{self.udp_port}')
        self.get_logger().info(
            f'range: {self.range_min}~{self.range_max}m | '
            f'FOV: {angle_min_deg}°~{angle_max_deg}°')

        # ── Statistik ──────────────────────────────────────
        self.scan_count   = 0
        self.error_count  = 0
        self.last_seq     = -1

        # ── Thread penerima UDP ────────────────────────────
        self._running = True
        self._thread  = threading.Thread(target=self._udp_loop, daemon=True)
        self._thread.start()

        self.get_logger().info('lidar_wifi_node siap — menunggu data dari ESP32...')
        self.get_logger().info(
            f'Pastikan ESP32 mengirim ke IP laptop ini, port {self.udp_port}')
        self._print_local_ip()

    def _print_local_ip(self):
        """Tampilkan IP laptop agar mudah dikonfigurasi di ESP32."""
        import subprocess
        try:
            result = subprocess.run(
                ['hostname', '-I'], capture_output=True, text=True)
            ips = result.stdout.strip()
            self.get_logger().info(f'IP laptop ini: {ips}')
            self.get_logger().info(
                'Masukkan salah satu IP di atas ke LAPTOP_IP di kode ESP32')
        except Exception:
            pass

    def _udp_loop(self):
        """Thread yang terus-menerus menerima paket UDP dari ESP32."""
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
                payload = json.loads(data.decode('utf-8'))
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                self.error_count += 1
                self.get_logger().warn(
                    f'JSON parse error [{self.error_count}]: {e}')
                continue

            if 'distances' not in payload:
                self.get_logger().warn('Paket tidak punya field "distances"')
                continue

            # Cek sequence untuk deteksi paket hilang
            seq = payload.get('seq', -1)
            if self.last_seq >= 0 and seq > 0:
                dropped = seq - self.last_seq - 1
                if dropped > 0:
                    self.get_logger().warn(
                        f'{dropped} paket hilang (seq {self.last_seq}→{seq})')
            self.last_seq = seq

            self._publish_scan(payload['distances'], addr[0])

    def _publish_scan(self, distances_cm: list, sender_ip: str):
        """Konversi array jarak cm ke LaserScan dan publish."""
        n = len(distances_cm)

        if n < 2:
            self.get_logger().warn(f'Data terlalu pendek: {n} titik')
            return

        # Balik urutan jika sensor dipasang terbalik
        if self.invert:
            distances_cm = list(reversed(distances_cm))

        # ── Bangun LaserScan ────────────────────────────────
        msg               = LaserScan()
        msg.header.stamp  = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min      = self.angle_min
        msg.angle_max      = self.angle_max
        msg.angle_increment = (self.angle_max - self.angle_min) / (n - 1)
        msg.range_min      = self.range_min
        msg.range_max      = self.range_max
        msg.scan_time      = self.scan_time
        msg.time_increment = self.scan_time / (n - 1)

        # ── Konversi cm → meter ─────────────────────────────
        ranges = []
        valid  = 0
        for d_cm in distances_cm:
            if d_cm <= 0:
                ranges.append(float('inf'))
            else:
                d_m = d_cm / 100.0
                if d_m < self.range_min or d_m > self.range_max:
                    ranges.append(float('inf'))
                else:
                    ranges.append(d_m)
                    valid += 1

        msg.ranges = ranges

        self.pub.publish(msg)
        self.scan_count += 1

        # Log ringkas setiap scan
        valid_ranges = [r for r in ranges if not math.isinf(r)]
        min_r = min(valid_ranges) if valid_ranges else float('nan')
        max_r = max(valid_ranges) if valid_ranges else float('nan')

        self.get_logger().info(
            f'Scan #{self.scan_count} dari {sender_ip} | '
            f'{valid}/{n} titik valid | '
            f'min={min_r:.2f}m max={max_r:.2f}m',
            throttle_duration_sec=2.0
        )

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=2.0)
        self.sock.close()
        self.get_logger().info('UDP socket ditutup')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LidarWifiNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        raise
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()