#!/usr/bin/env python3
"""
lidar_wifi_node.py
==================
Terima data scan RPLidar C1 dari ESP32 via UDP port 5006
Kontrol ON/OFF RPLidar via UDP port 5005
Publish sebagai sensor_msgs/LaserScan ke /scan

Perubahan dari versi TFMini + servo:
- Scan 360° penuh (bukan 180°)
- ON/OFF RPLidar via port 5005 (bukan state scan/navigation)
- ESP32 kirim 2 paket per putaran (0°-179° dan 180°-359°)
- Tidak ada servo, tidak ada sweep manual
"""

import rclpy
import time
import math
import json
import socket
import threading
import subprocess
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class LidarWifiNode(Node):
    def __init__(self):
        super().__init__('lidar_wifi_node')

        # ── Parameter ──────────────────────────────────────
        self.declare_parameter('esp32_ip',    '192.168.100.94')
        self.declare_parameter('port_lidar',   5005)   # kirim on/off ke ESP32
        self.declare_parameter('port_scan',    5006)   # terima scan dari ESP32
        self.declare_parameter('frame_id',    'laser')
        self.declare_parameter('range_min',    0.15)   # RPLidar C1 min range
        self.declare_parameter('range_max',   12.0)    # RPLidar C1 max range
        self.declare_parameter('auto_start',   False)  # ON otomatis saat node start

        self.esp32_ip   = self.get_parameter('esp32_ip').value
        self.port_lidar = self.get_parameter('port_lidar').value
        self.port_scan  = self.get_parameter('port_scan').value
        self.frame_id   = self.get_parameter('frame_id').value
        self.range_min  = self.get_parameter('range_min').value
        self.range_max  = self.get_parameter('range_max').value
        self.auto_start = self.get_parameter('auto_start').value

        # ── Publisher ──────────────────────────────────────
        self.pub_scan   = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_status = self.create_publisher(String, '/lidar_status', 10)

        # ── Subscriber ─────────────────────────────────────
        # /lidar_power: publish 'on' atau 'off' untuk kontrol RPLidar
        self.create_subscription(
            String, '/lidar_power', self._power_cb, 10)

        # ── Socket kirim: kontrol ON/OFF ke ESP32 ──────────
        self.sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ── Socket terima: data scan dari ESP32 ────────────
        self.sock_scan = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_scan.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_scan.bind(('0.0.0.0', self.port_scan))
        self.sock_scan.settimeout(1.0)

        # ── State scan buffer ──────────────────────────────
        # RPLidar C1: 360° dengan resolusi 1° = 360 samples
        # ESP32 kirim 2 paket per putaran: part=1 (0-179°) dan part=2 (180-359°)
        self._scan_buf   = [float('inf')] * 360   # index = sudut integer
        self._buf_lock   = threading.Lock()
        self._part1_recv = False
        self._part2_recv = False
        self._scan_count = 0
        self._last_seq   = -1
        self._lidar_on   = False

        # ── Thread terima scan ─────────────────────────────
        self._running = True
        self._recv_thread = threading.Thread(
            target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        # ── Timer status ───────────────────────────────────
        self.create_timer(5.0, self._publish_status)

        # Info startup
        try:
            r = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
            self.get_logger().info(f'IP laptop   : {r.stdout.strip()}')
        except Exception:
            pass

        self.get_logger().info(
            f'ESP32 IP    : {self.esp32_ip}:{self.port_lidar}')
        self.get_logger().info(
            f'Terima scan : port {self.port_scan}')
        self.get_logger().info(
            'Kontrol RPLidar: publish ke /lidar_power ("on" atau "off")')

        # Auto start jika dikonfigurasi
        if self.auto_start:
            self.get_logger().info('auto_start=True → menyalakan RPLidar...')
            time.sleep(1.0)
            self._send_power('on')

    # ══════════════════════════════════════════════════════
    #  KONTROL ON/OFF RPLidar
    # ══════════════════════════════════════════════════════
    def _send_power(self, cmd: str):
        """Kirim 'on' atau 'off' ke ESP32 port 5005."""
        try:
            self.sock_cmd.sendto(
                cmd.encode(), (self.esp32_ip, self.port_lidar))
            self.get_logger().info(
                f'→ Kirim "{cmd}" ke ESP32 {self.esp32_ip}:{self.port_lidar}')
            self._lidar_on = (cmd.lower() == 'on')
        except Exception as e:
            self.get_logger().error(f'Gagal kirim perintah: {e}')

    def _power_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ('on', 'off'):
            self._send_power(cmd)
        elif cmd == 'status':
            self._send_power('status')
        else:
            self.get_logger().warn(
                f'Perintah tidak valid: "{cmd}" (gunakan: on / off / status)')

    # ══════════════════════════════════════════════════════
    #  THREAD TERIMA DATA SCAN
    # ══════════════════════════════════════════════════════
    def _recv_loop(self):
        self.get_logger().info(
            f'Thread scan receiver jalan, listen port {self.port_scan}')

        while self._running:
            try:
                data, addr = self.sock_scan.recvfrom(4096)
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    self.get_logger().warn(f'UDP recv error: {e}')
                continue

            self.get_logger().info(f'RECV {len(data)} bytes from {addr}')  # TAMBAHAN

            try:
                payload = json.loads(data.decode('utf-8'))
            except Exception as e:
                self.get_logger().warn(
                    f'JSON parse error: {e} | raw[:60]: {data[:60]}')
                continue

            self._process_packet(payload, addr[0])

    def _process_packet(self, payload: dict, sender_ip: str):
        seq   = payload.get('seq',   -1)
        part  = payload.get('part',   0)
        start = payload.get('start',  0)
        dists = payload.get('distances', [])

        self.get_logger().info(
            f'seq={seq} part={part} start={start} n_dist={len(dists)}')

        if not dists:
            return

        with self._buf_lock:
            for i, d_mm in enumerate(dists):
                idx = start + i
                if idx >= 360:
                    break
                if d_mm <= 0:
                    self._scan_buf[idx] = float('inf')
                else:
                    d_m = d_mm / 1000.0
                    self._scan_buf[idx] = (
                        d_m if self.range_min <= d_m <= self.range_max
                        else float('inf')
                    )

        # Publish setiap kali part==2 datang (akhir satu putaran)
        if part == 2:
            self._publish_scan()

    # ══════════════════════════════════════════════════════
    #  PUBLISH LaserScan
    # ══════════════════════════════════════════════════════
    def _publish_scan(self):
        with self._buf_lock:
            buf_copy = list(self._scan_buf)

        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min       = 0.0
        msg.angle_max       = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / 360.0
        msg.range_min       = self.range_min
        msg.range_max       = self.range_max
        msg.scan_time       = 0.1
        msg.time_increment  = msg.scan_time / 360.0

        # Balik arah untuk koreksi orientasi kiri-kanan
        msg.ranges = [float(buf_copy[(360 - i) % 360]) for i in range(360)]

        self.pub_scan.publish(msg)
        self._scan_count += 1

        # Log ringkas setiap 10 scan
        if self._scan_count % 10 == 0:
            valid = sum(1 for r in msg.ranges if not math.isinf(r))
            valid_r = [r for r in msg.ranges if not math.isinf(r)]
            min_r = min(valid_r) if valid_r else float('nan')
            max_r = max(valid_r) if valid_r else float('nan')
            self.get_logger().info(
                f'Scan #{self._scan_count} | '
                f'{valid}/360 valid | '
                f'min={min_r:.2f}m max={max_r:.2f}m'
            )

    # ══════════════════════════════════════════════════════
    #  STATUS
    # ══════════════════════════════════════════════════════
    def _publish_status(self):
        msg = String()
        msg.data = (f'lidar={"ON" if self._lidar_on else "OFF"} '
                    f'scans={self._scan_count}')
        self.pub_status.publish(msg)

    def destroy_node(self):
        self._running = False
        self._recv_thread.join(timeout=2.0)
        # Matikan lidar sebelum shutdown
        if self._lidar_on:
            self.get_logger().info('Shutdown: matikan RPLidar...')
            self._send_power('off')
            time.sleep(0.5)
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