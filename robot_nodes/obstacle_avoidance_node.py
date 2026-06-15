#!/usr/bin/env python3
"""
obstacle_avoidance_node.py
==========================
Reactive obstacle avoidance dari data /scan (LaserScan)
Publish ke /cmd_vel (Twist)

Strategi:
  - Bagi scan menjadi sektor: kiri, tengah-kiri, depan, tengah-kanan, kanan
  - Kalau depan bebas → maju
  - Kalau depan terhalang → belok ke arah yang paling bebas
  - Kalau semua terhalang → mundur sementara
  - Stop saat semua sisi terhalang (finish — tembok di semua arah)

Parameter yang bisa diatur via ROS2:
  linear_speed        : kecepatan maju (m/s)
  angular_speed       : kecepatan belok (rad/s)
  front_clear_dist    : jarak minimum depan dianggap bebas (m)
  side_clear_dist     : jarak minimum samping dianggap bebas (m)
  stop_dist           : jarak terlalu dekat, harus mundur (m)
  front_angle_width   : lebar sektor depan (derajat, kiri-kanan dari 0°)
  side_angle_width    : lebar sektor samping (derajat)
  finish_detect_dist  : jarak deteksi finish/tembok semua arah (m)
  finish_sectors      : jumlah sektor yang harus terhalang untuk dianggap finish
  scan_topic          : topic LaserScan
  cmd_vel_topic       : topic cmd_vel output
"""

import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # ── Parameter ──────────────────────────────────────
        self.declare_parameter('linear_speed',      0.5)    # m/s maju
        self.declare_parameter('angular_speed',     2.0)    # rad/s belok
        self.declare_parameter('front_clear_dist',  0.6)    # m, depan dianggap bebas
        self.declare_parameter('side_clear_dist',   0.4)    # m, samping dianggap bebas
        self.declare_parameter('stop_dist',         0.25)   # m, terlalu dekat → mundur
        self.declare_parameter('front_angle_width', 30.0)   # derajat kiri-kanan dari depan
        self.declare_parameter('side_angle_width',  60.0)   # derajat lebar sektor samping
        self.declare_parameter('finish_detect_dist',0.5)    # m, tembok dianggap finish
        self.declare_parameter('finish_sectors',    5)      # jumlah sektor terhalang = finish
        self.declare_parameter('backup_duration',   0.8)    # detik mundur
        self.declare_parameter('scan_topic',        '/scan')
        self.declare_parameter('cmd_vel_topic',     '/cmd_vel')
        self.declare_parameter('enabled',           True)   # aktif/nonaktif

        self._load_params()

        # ── Publisher ──────────────────────────────────────
        self.pub_cmd = self.create_publisher(
            Twist, self.cmd_vel_topic, 10)
        self.pub_state = self.create_publisher(
            String, '/avoidance_state', 10)

        # ── Subscriber ─────────────────────────────────────
        self.create_subscription(
            LaserScan, self.scan_topic,
            self._scan_cb, qos_profile_sensor_data)

        # Subscribe untuk enable/disable dari luar
        self.create_subscription(
            String, '/avoidance_enable',
            self._enable_cb, 10)

        # ── State ──────────────────────────────────────────
        self._state       = 'IDLE'
        self._backup_time = 0.0
        self._last_turn   = 'left'   # arah belok terakhir (untuk konsistensi)
        self._scan_count  = 0

        # Timer log status
        self.create_timer(1.0, self._log_status)

        self.get_logger().info('Obstacle Avoidance Node siap')
        self._log_params()

    def _load_params(self):
        self.linear_speed      = self.get_parameter('linear_speed').value
        self.angular_speed     = self.get_parameter('angular_speed').value
        self.front_clear_dist  = self.get_parameter('front_clear_dist').value
        self.side_clear_dist   = self.get_parameter('side_clear_dist').value
        self.stop_dist         = self.get_parameter('stop_dist').value
        self.front_angle_width = self.get_parameter('front_angle_width').value
        self.side_angle_width  = self.get_parameter('side_angle_width').value
        self.finish_detect_dist= self.get_parameter('finish_detect_dist').value
        self.finish_sectors    = self.get_parameter('finish_sectors').value
        self.backup_duration   = self.get_parameter('backup_duration').value
        self.scan_topic        = self.get_parameter('scan_topic').value
        self.cmd_vel_topic     = self.get_parameter('cmd_vel_topic').value
        self.enabled           = self.get_parameter('enabled').value

    def _log_params(self):
        self.get_logger().info(
            f'Params: linear={self.linear_speed}m/s '
            f'angular={self.angular_speed}rad/s '
            f'front_clear={self.front_clear_dist}m '
            f'stop={self.stop_dist}m '
            f'front_width={self.front_angle_width}° '
            f'finish_dist={self.finish_detect_dist}m'
        )

    def _enable_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'on':
            self.enabled = True
            self.get_logger().info('Obstacle avoidance: ON')
        elif cmd == 'off':
            self.enabled = False
            self._publish_cmd(0.0, 0.0)
            self.get_logger().info('Obstacle avoidance: OFF')

    # ══════════════════════════════════════════════════════
    #  UTILS: ambil jarak minimum di sektor tertentu
    # ══════════════════════════════════════════════════════
    def _sector_min(self, ranges, angle_min_deg, angle_max_deg,
                    scan_angle_min, scan_angle_inc):
        """
        Ambil jarak minimum di sektor antara angle_min_deg dan angle_max_deg.
        Sudut dalam derajat, 0° = depan robot.
        LaserScan: sudut 0 = angle_min dari scan (biasanya 0 atau -π).
        """
        n = len(ranges)
        vals = []
        for deg in range(int(angle_min_deg), int(angle_max_deg) + 1):
            rad = math.radians(deg)
            # Index di array ranges
            idx = int((rad - scan_angle_min) / scan_angle_inc)
            if 0 <= idx < n:
                r = ranges[idx]
                if not math.isinf(r) and not math.isnan(r) and r > 0.05:
                    vals.append(r)
        return min(vals) if vals else float('inf')

    def _sector_avg(self, ranges, angle_min_deg, angle_max_deg,
                    scan_angle_min, scan_angle_inc):
        """Rata-rata jarak di sektor."""
        n = len(ranges)
        vals = []
        for deg in range(int(angle_min_deg), int(angle_max_deg) + 1):
            rad = math.radians(deg)
            idx = int((rad - scan_angle_min) / scan_angle_inc)
            if 0 <= idx < n:
                r = ranges[idx]
                if not math.isinf(r) and not math.isnan(r) and r > 0.05:
                    vals.append(r)
        return sum(vals)/len(vals) if vals else float('inf')

    # ══════════════════════════════════════════════════════
    #  SCAN CALLBACK — logika utama
    # ══════════════════════════════════════════════════════
    def _scan_cb(self, msg: LaserScan):
        if not self.enabled:
            return

        self._scan_count += 1
        ranges        = msg.ranges
        angle_min     = msg.angle_min      # radian
        angle_inc     = msg.angle_increment  # radian per index

        fw  = self.front_angle_width
        sw  = self.side_angle_width

        # ── Hitung jarak minimum per sektor ────────────────
        # Depan: -fw° sampai +fw° dari 0°
        front      = self._sector_min(ranges, -fw,    +fw,    angle_min, angle_inc)
        # Kiri: fw° sampai fw+sw°
        left       = self._sector_min(ranges, fw,     fw+sw,  angle_min, angle_inc)
        # Kanan: -(fw+sw)° sampai -fw°
        right      = self._sector_min(ranges, -(fw+sw), -fw,  angle_min, angle_inc)
        # Depan-kiri: 0° sampai fw°
        front_left = self._sector_min(ranges, 0,      fw,     angle_min, angle_inc)
        # Depan-kanan: -fw° sampai 0°
        front_right= self._sector_min(ranges, -fw,    0,      angle_min, angle_inc)
        # Belakang: 150° sampai 210° (atau -150° sampai -210°)
        back       = self._sector_min(ranges, 150,    180,    angle_min, angle_inc)

        # ── Deteksi FINISH: semua arah terhalang ───────────
        # Bagi 360° jadi 8 sektor, hitung berapa yang terhalang
        blocked = 0
        sector_size = 45
        for start in range(-180, 180, sector_size):
            d = self._sector_min(ranges, start, start+sector_size,
                                  angle_min, angle_inc)
            if d < self.finish_detect_dist:
                blocked += 1

        if blocked >= self.finish_sectors:
            self._state = 'FINISH'
            self._publish_cmd(0.0, 0.0)
            self._publish_state('FINISH')
            return

        # ── Logika gerak ───────────────────────────────────
        fd  = self.front_clear_dist
        sd  = self.side_clear_dist
        std = self.stop_dist
        lin = self.linear_speed
        ang = self.angular_speed

        if front < std:
            # Terlalu dekat — mundur
            self._state = 'BACKUP'
            self._publish_cmd(-lin * 0.5, 0.0)
            self._publish_state(f'BACKUP front={front:.2f}m')

        elif front < fd:
            # Depan terhalang — tentukan arah belok
            if left > right:
                # Kiri lebih bebas
                self._state    = 'TURN_LEFT'
                self._last_turn = 'left'
                # Kecepatan belok proporsional dengan seberapa terhalang depan
                turn_strength = ang * (1.0 - front / fd)
                self._publish_cmd(0.0, turn_strength)
                self._publish_state(
                    f'TURN_LEFT L={left:.2f} R={right:.2f} F={front:.2f}')
            else:
                # Kanan lebih bebas
                self._state    = 'TURN_RIGHT'
                self._last_turn = 'right'
                turn_strength  = ang * (1.0 - front / fd)
                self._publish_cmd(0.0, -turn_strength)
                self._publish_state(
                    f'TURN_RIGHT L={left:.2f} R={right:.2f} F={front:.2f}')

        elif front_left < sd and front_right >= sd:
            # Kiri-depan terhalang, geser kanan sedikit
            self._state = 'STEER_RIGHT'
            self._publish_cmd(lin * 0.6, -ang * 0.3)
            self._publish_state(f'STEER_RIGHT FL={front_left:.2f}m')

        elif front_right < sd and front_left >= sd:
            # Kanan-depan terhalang, geser kiri sedikit
            self._state = 'STEER_LEFT'
            self._publish_cmd(lin * 0.6, ang * 0.3)
            self._publish_state(f'STEER_LEFT FR={front_right:.2f}m')

        else:
            # Depan bebas — maju
            self._state = 'FORWARD'
            # Slight correction kalau mendekati dinding samping
            correction = 0.0
            if left < sd * 1.5 and right >= sd * 1.5:
                correction = -ang * 0.15   # terlalu dekat kiri, belok kanan sedikit
            elif right < sd * 1.5 and left >= sd * 1.5:
                correction = ang * 0.15    # terlalu dekat kanan, belok kiri sedikit
            self._publish_cmd(lin, correction)
            self._publish_state(
                f'FORWARD F={front:.2f} L={left:.2f} R={right:.2f}')

    # ══════════════════════════════════════════════════════
    #  PUBLISH
    # ══════════════════════════════════════════════════════
    def _publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.pub_cmd.publish(msg)

    def _publish_state(self, state: str):
        msg = String()
        msg.data = state
        self.pub_state.publish(msg)

    def _log_status(self):
        if self._scan_count > 0:
            self.get_logger().info(
                f'State: {self._state} | '
                f'Scans: {self._scan_count} | '
                f'Enabled: {self.enabled}')

    def destroy_node(self):
        self._publish_cmd(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObstacleAvoidanceNode()
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