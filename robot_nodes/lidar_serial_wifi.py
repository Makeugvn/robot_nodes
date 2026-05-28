#!/usr/bin/env python3
"""
lidar_wifi.py
=============
- Kirim perintah 'scan' ke ESP32 via UDP port 5005
- Terima data scan dari ESP32 via UDP port 5006
- Publish sebagai sensor_msgs/LaserScan ke /scan
"""

import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

import socket
import json
import math
import threading


class LidarWifiNode(Node):
    def __init__(self):
        super().__init__('lidar_wifi_node')

        # ── Parameter ──────────────────────────────────────
        self.declare_parameter('esp32_ip',        '10.42.93.127')
        self.declare_parameter('port_cmd',         5005)  # kirim ke ESP32
        self.declare_parameter('port_scan',        5006)  # terima dari ESP32
        self.declare_parameter('frame_id',        'laser')
        self.declare_parameter('range_min',        0.10)
        self.declare_parameter('range_max',        12.0)
        self.declare_parameter('angle_min_deg',   -90.0)
        self.declare_parameter('angle_max_deg',    90.0)
        self.declare_parameter('scan_time',         7.5)
        self.declare_parameter('timeout_sec',      15.0)

        self.esp32_ip    = self.get_parameter('esp32_ip').value
        self.port_cmd    = self.get_parameter('port_cmd').value
        self.port_scan   = self.get_parameter('port_scan').value
        self.frame_id    = self.get_parameter('frame_id').value
        self.range_min   = self.get_parameter('range_min').value
        self.range_max   = self.get_parameter('range_max').value
        self.scan_time   = self.get_parameter('scan_time').value
        self.timeout_sec = self.get_parameter('timeout_sec').value
        self.angle_min   = math.radians(self.get_parameter('angle_min_deg').value)
        self.angle_max   = math.radians(self.get_parameter('angle_max_deg').value)

        # ── Publisher ──────────────────────────────────────
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)

        # ── Subscriber: trigger scan dari luar ────────────
        # Publish True ke /trigger_scan untuk mulai scan
        self.create_subscription(Bool, '/trigger_scan', self._trigger_cb, 10)

        # ── Socket 1: KIRIM perintah ke ESP32 ─────────────
        # Tidak perlu bind — hanya untuk sendto
        self.sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ── Socket 2: TERIMA data scan dari ESP32 ─────────
        # Bind ke port 5006 untuk mendengarkan data masuk
        self.sock_scan = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_scan.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_scan.bind(('0.0.0.0', self.port_scan))
        self.sock_scan.settimeout(1.0)

        self.get_logger().info(
            f'Kirim perintah  → ESP32 {self.esp32_ip}:{self.port_cmd}')
        self.get_logger().info(
            f'Terima scan     ← port {self.port_scan}')

        # Tampilkan IP laptop
        import subprocess
        try:
            r = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
            self.get_logger().info(f'IP laptop: {r.stdout.strip()}')
        except Exception:
            pass

        # ── State ──────────────────────────────────────────
        self.scan_count    = 0
        self.last_seq      = -1
        self.waiting_data  = False
        self.trigger_time  = None
        self.packets_recv  = 0   # ESP32 kirim 2 paket per scan (maju + balik)

        # ── Thread terima scan ─────────────────────────────
        self._running = True
        self._scan_thread = threading.Thread(
            target=self._scan_recv_loop, daemon=True)
        self._scan_thread.start()

        # ── Timer cek timeout ──────────────────────────────
        self.create_timer(1.0, self._check_timeout)

        self.get_logger().info('Node siap. Publish True ke /trigger_scan untuk scan.')

    # ── Kirim perintah "scan" ke ESP32 ────────────────────
    def send_scan_command(self):
        try:
            self.sock_cmd.sendto(b'scan\n', (self.esp32_ip, self.port_cmd))
            self.waiting_data  = True
            self.trigger_time  = time.time()
            self.packets_recv  = 0
            self.get_logger().info(
                f'→ Perintah "scan" dikirim ke {self.esp32_ip}:{self.port_cmd}')
        except Exception as e:
            self.get_logger().error(f'Gagal kirim perintah: {e}')

    # ── Callback trigger dari topic ────────────────────────
    def _trigger_cb(self, msg: Bool):
        if msg.data:
            self.send_scan_command()

    # ── Cek timeout — kirim ulang kalau tidak ada respons ──
    def _check_timeout(self):
        if not self.waiting_data:
            return
        if self.trigger_time and (time.time() - self.trigger_time > self.timeout_sec):
            self.get_logger().warn(
                f'Timeout {self.timeout_sec}s! Kirim ulang perintah scan...')
            self.send_scan_command()

    # ── Thread: terima data scan dari ESP32 ───────────────
    def _scan_recv_loop(self):
        self.get_logger().info(
            f'Thread scan receiver jalan, listen port {self.port_scan}')

        while self._running:
            try:
                data, addr = self.sock_scan.recvfrom(2048)
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    self.get_logger().warn(f'UDP recv error: {e}')
                continue

            # Parse JSON
            try:
                payload = json.loads(data.decode('utf-8'))
            except Exception as e:
                self.get_logger().warn(
                    f'JSON error: {e} | raw: {data[:60]}')
                continue

            if 'distances' not in payload:
                self.get_logger().warn('Paket tidak punya field distances')
                continue

            # Cek sequence
            seq = payload.get('seq', -1)
            if self.last_seq >= 0 and seq > 0:
                lost = seq - self.last_seq - 1
                if lost > 0:
                    self.get_logger().warn(
                        f'{lost} paket hilang (seq {self.last_seq}→{seq})')
            self.last_seq = seq

            # Hitung paket — ESP32 kirim 2x per scan (maju + balik)
            self.packets_recv += 1
            if self.packets_recv == 1:
                # Paket pertama: reset timeout agar tidak kirim ulang
                self.trigger_time = time.time()
                self.get_logger().info('Paket pertama diterima (sweep maju)')
            elif self.packets_recv >= 2:
                # Paket kedua: scan selesai
                self.waiting_data = False
                self.packets_recv = 0
                self.get_logger().info('Scan selesai (2 paket diterima)')

            self._publish_scan(payload['distances'], addr[0])

    # ── Publish LaserScan ─────────────────────────────────
    def _publish_scan(self, distances_cm: list, sender_ip: str):
        n = len(distances_cm)
        if n < 2:
            return

        msg                 = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min       = self.angle_min
        msg.angle_max       = self.angle_max
        msg.angle_increment = (self.angle_max - self.angle_min) / (n - 1)
        msg.range_min       = self.range_min
        msg.range_max       = self.range_max
        msg.scan_time       = self.scan_time
        msg.time_increment  = self.scan_time / (n - 1)

        ranges = []
        valid  = 0
        for d_cm in distances_cm:
            if d_cm <= 0:
                ranges.append(float('inf'))
            else:
                d_m = d_cm / 100.0
                if self.range_min <= d_m <= self.range_max:
                    ranges.append(d_m)
                    valid += 1
                else:
                    ranges.append(float('inf'))

        msg.ranges = ranges
        self.pub_scan.publish(msg)
        self.scan_count += 1

        valid_r = [r for r in ranges if not math.isinf(r)]
        min_r   = min(valid_r) if valid_r else float('nan')
        max_r   = max(valid_r) if valid_r else float('nan')

        self.get_logger().info(
            f'✓ Scan #{self.scan_count} dari {sender_ip} | '
            f'{valid}/{n} valid | '
            f'min={min_r:.2f}m max={max_r:.2f}m',
            throttle_duration_sec=1.0
        )

    def destroy_node(self):
        self._running = False
        self._scan_thread.join(timeout=2.0)
        self.sock_scan.close()
        self.sock_cmd.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LidarWifiNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()