#!/usr/bin/env python3
"""
grid_navigator_node.py
======================
Navigasi grid dengan koreksi posisi via LiDAR:
  - Koreksi lateral saat MOVE_CELL (jarak ke tembok kiri/belakang)
  - Deteksi drift saat menghadap ke depan (cek obstacle vs tembok)
"""

import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data

from path_planner import get_path, path_to_relative_commands, MANUAL_PATH_1, MANUAL_PATH_2, MANUAL_PATH_3, MANUAL_PATH_4
from grid_map import (START_ROBOT1, FINISH_ROBOT1, START_ROBOT2, FINISH_ROBOT2, START_ROBOT3, FINISH_ROBOT3, START_ROBOT4, FINISH_ROBOT4, CELL_SIZE,
                      ARENA_MAP, ROWS, COLS)


def euler_from_quaternion(x, y, z, w):
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return yaw


class GridNavigatorNode(Node):
    def __init__(self):
        super().__init__('grid_navigator')

        # ── Parameter Robot ───────────────────────────────
        self.cell_size     = CELL_SIZE
        self.linear_speed  = 0.3
        self.angular_speed = 1.0

        # ── Konfigurasi Path ──────────────────────────────
        self.current_cell = START_ROBOT4
        self.goal_cell    = FINISH_ROBOT4
        self.manual_path  = MANUAL_PATH_4

        # ── State Sensor ──────────────────────────────────
        self.initial_yaw  = None
        self.current_yaw  = 0.0
        self.dist_front   = float('inf')
        self.dist_back    = float('inf')
        self.dist_left    = float('inf')   # 90° kiri robot
        self.dist_right   = float('inf')   # 90° kanan robot
        self._raw_ranges  = []

        # ── State Machine ─────────────────────────────────
        self.state             = 'INIT'
        self.commands          = []
        self.current_step_idx  = 0
        self.target_yaw        = 0.0
        self.use_back_laser    = False
        self.start_move_time   = None
        self.brake_start_time  = None
        self.BRAKE_DURATION    = 0.4

        # ── Delta Tracking & Kompensasi ───────────────────
        self.accumulated_distance = 0.0
        self.last_laser           = 0.0
        self.lateral_drift        = 0.0
        self.kompensasi_jarak     = 0.0

        # ── Koreksi lateral LiDAR ─────────────────────────
        # Jarak ideal ke tembok (dihitung dari grid_map)
        self._ideal_wall_left  = None   # meter, tembok di sisi kiri robot
        self._ideal_wall_back  = None   # meter, tembok di belakang robot
        self._wall_correction_active = False

        # Parameter koreksi dinding
        self.WALL_KP           = 0.8    # gain proporsional koreksi dinding
        self.WALL_DEADBAND     = 0.04   # toleransi ±4cm sebelum koreksi
        self.WALL_MAX_CORRECT  = 0.4    # maksimum angular koreksi dinding (rad/s)

        # ── Heading tracker (untuk tahu robot menghadap ke mana) ──
        # 'N','S','E','W' dalam koordinat arena
        self._current_heading  = 'S'    # sesuaikan dengan initial_heading di path_planner

        # ── Publisher & Subscriber ────────────────────────
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/imu', self._imu_cb, qos_profile_sensor_data)

        self.timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info('Grid Navigator dengan koreksi dinding aktif')

    # ══════════════════════════════════════════════════════
    #  IMU CALLBACK
    # ══════════════════════════════════════════════════════
    def _imu_cb(self, msg: Imu):
        yaw = euler_from_quaternion(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w)

        if self.initial_yaw is None:
            self.initial_yaw = yaw
            self.target_yaw  = yaw
            path = get_path(
                self.current_cell, self.goal_cell,
                manual_matrix=self.manual_path, obstacle_margin=0)
            self.commands = path_to_relative_commands(
                path, initial_heading='S')
            self.state = 'READ_COMMAND' if self.commands else 'FINISHED'

        self.current_yaw = yaw

    # ══════════════════════════════════════════════════════
    #  SCAN CALLBACK
    # ══════════════════════════════════════════════════════
    def _scan_cb(self, msg: LaserScan):
        ranges    = np.array(msg.ranges)
        num_rays  = len(ranges)
        if num_rays == 0:
            return
        self._raw_ranges = ranges

        def get_sector_min(center_deg, half_width=4):
            vals = []
            for d in range(center_deg - half_width,
                           center_deg + half_width + 1):
                idx = d % num_rays
                r   = ranges[idx]
                if not math.isinf(r) and not math.isnan(r) and r > 0.05:
                    vals.append(r)
            return min(vals) if vals else float('inf')

        self.dist_front = get_sector_min(0)
        self.dist_back  = get_sector_min(180)
        self.dist_left  = get_sector_min(90)
        self.dist_right = get_sector_min(270)

        # Edge detection untuk kompensasi drift
        self.lateral_drift = 0.0
        if self.dist_front < 0.6:
            base_r       = self.dist_front
            edge_left_y  = None
            edge_right_y = None

            for deg in range(1, 81):
                r = ranges[deg % num_rays]
                if math.isinf(r) or math.isnan(r) or (r - base_r) > 0.2:
                    edge_left_y = base_r * math.sin(math.radians(deg - 1))
                    break

            for deg in range(1, 81):
                idx = (-deg) % num_rays
                r   = ranges[idx]
                if math.isinf(r) or math.isnan(r) or (r - base_r) > 0.2:
                    edge_right_y = base_r * math.sin(math.radians(-(deg - 1)))
                    break

            Y_center = None
            if edge_left_y is not None and edge_right_y is not None:
                Y_center = (edge_left_y + edge_right_y) / 2.0
            elif edge_left_y is not None:
                Y_center = edge_left_y - 0.2
            elif edge_right_y is not None:
                Y_center = edge_right_y + 0.2

            if Y_center is not None:
                self.lateral_drift = -Y_center

    # ══════════════════════════════════════════════════════
    #  HITUNG JARAK IDEAL KE DINDING
    #  Dari posisi grid saat ini, lihat berapa sel ke tembok/obstacle
    #  di sisi kiri dan belakang robot
    # ══════════════════════════════════════════════════════
    def _compute_ideal_wall_distances(self):
        """
        Hitung jarak ideal robot ke dinding/obstacle di sisi
        kiri dan belakang berdasarkan posisi grid dan heading saat ini.

        Robot berada di self.current_cell = (row, col).
        Heading menentukan orientasi — setelah RR dari 'S', robot menghadap 'W'
        (ke kiri/barat), sehingga:
          - 'depan robot' = arah W = col berkurang
          - 'belakang robot' = arah E = col bertambah → tembok kanan arena
          - 'kiri robot' = arah S = row bertambah → tembok bawah arena
          - 'kanan robot' = arah N = row berkurang → tembok atas arena
        """
        row, col = self.current_cell
        heading  = self._current_heading

        # Vektor arah: (dr, dc) untuk masing-masing heading
        dirs = {'N': (-1, 0), 'S': (1, 0), 'E': (0, 1), 'W': (0, -1)}

        # Dari heading robot, tentukan vektor untuk sisi kiri dan belakang
        # Kiri robot = heading dirotasi +90° (CCW)
        left_map  = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}
        back_map  = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}

        left_dir  = left_map[heading]
        back_dir  = back_map[heading]

        def dist_to_wall(r, c, dr, dc):
            """Hitung jarak (meter) ke obstacle/dinding dari sel (r,c) ke arah (dr,dc)."""
            n_cells = 0
            cr, cc  = r + dr, c + dc
            while 0 <= cr < ROWS and 0 <= cc < COLS:
                if ARENA_MAP[cr, cc] == 1:
                    break
                n_cells += 1
                cr += dr
                cc += dc
            # n_cells = jumlah sel kosong sebelum obstacle/dinding
            # Jarak = n_cells × cell_size + 0.5 × cell_size (ke tengah sel saat ini)
            return (n_cells + 0.5) * self.cell_size

        dr_l, dc_l = dirs[left_dir]
        dr_b, dc_b = dirs[back_dir]

        self._ideal_wall_left = dist_to_wall(row, col, dr_l, dc_l)
        self._ideal_wall_back = dist_to_wall(row, col, dr_b, dc_b)

        self.get_logger().info(
            f'[WALL] Heading={heading} | '
            f'ideal_left={self._ideal_wall_left:.2f}m '
            f'ideal_back={self._ideal_wall_back:.2f}m')

    # ══════════════════════════════════════════════════════
    #  KOREKSI DINDING — dipanggil saat MOVE_CELL
    #  Menggunakan jarak ke dinding kiri (90°) dan belakang (180°)
    #  untuk menghasilkan koreksi angular tambahan
    # ══════════════════════════════════════════════════════
    def _wall_correction(self) -> float:
        """
        Hitung koreksi angular dari jarak dinding.
        Return: nilai angular.z tambahan (rad/s)
                positif = belok kiri, negatif = belok kanan
        """
        if (self._ideal_wall_left is None or
                self._ideal_wall_back is None):
            return 0.0

        correction = 0.0

        # Koreksi dari dinding kiri (sensor di 90°)
        err_left = self.dist_left - self._ideal_wall_left
        if abs(err_left) > self.WALL_DEADBAND:
            # err_left > 0 → robot terlalu jauh dari dinding kiri → belok kiri
            # err_left < 0 → robot terlalu dekat dinding kiri → belok kanan
            correction += self.WALL_KP * err_left
            self.get_logger().debug(
                f'[WALL] err_left={err_left:.3f}m corr={correction:.3f}')

        # Koreksi dari dinding belakang (sensor di 180°)
        # Dinding belakang tidak langsung menghasilkan angular correction
        # tapi bisa dipakai untuk konfirmasi drift lateral
        # (jika dist_back sangat berbeda dari ideal, robot sudah bergeser)
        err_back = self.dist_back - self._ideal_wall_back
        if abs(err_back) > self.WALL_DEADBAND * 2:
            self.get_logger().debug(
                f'[WALL] err_back={err_back:.3f}m '
                f'(konfirmasi drift, tidak langsung dikoreksi)')

        # Clamp
        correction = max(-self.WALL_MAX_CORRECT,
                         min(self.WALL_MAX_CORRECT, correction))
        return correction

    # ══════════════════════════════════════════════════════
    #  CEK DRIFT SAAT MENGHADAP KE DEPAN
    #  Bandingkan dist_front dengan jarak ideal ke obstacle
    #  dari grid_map. Jika tidak sesuai → robot drift lateral.
    # ══════════════════════════════════════════════════════
    def _check_forward_drift(self) -> float:
        """
        Saat robot menghadap depan (heading 'S' misalnya),
        cek apakah ada obstacle di grid_map pada jarak tertentu.

        Jika di grid_map ada obstacle di depan pada jarak N sel,
        dist_front seharusnya ≈ N × cell_size.

        Jika dist_front jauh lebih besar (tembok jauh), berarti
        robot drift ke samping dan tidak lurus dengan obstacle.

        Return: estimasi drift lateral (meter)
                positif = drift ke kanan, negatif = drift ke kiri
                0.0 jika tidak bisa ditentukan
        """
        row, col = self.current_cell
        heading  = self._current_heading
        dirs     = {'N': (-1, 0), 'S': (1, 0), 'E': (0, 1), 'W': (0, -1)}
        dr, dc   = dirs[heading]

        # Cari obstacle pertama di depan dari grid_map
        n_cells  = 0
        cr, cc   = row + dr, col + dc
        obs_found = False
        while 0 <= cr < ROWS and 0 <= cc < COLS:
            if ARENA_MAP[cr, cc] == 1:
                obs_found = True
                break
            n_cells += 1
            cr += dr
            cc += dc

        if not obs_found:
            # Tidak ada obstacle di depan, tidak bisa deteksi drift
            return 0.0

        # Jarak ideal ke obstacle = n_cells + 0.5 sel (ke tengah sel kosong terakhir)
        ideal_front = (n_cells + 0.5) * self.cell_size
        actual_front = self.dist_front

        # Toleransi: jika actual sangat jauh dari ideal → drift
        diff = actual_front - ideal_front
        DRIFT_THRESHOLD = self.cell_size * 0.6  # 60% dari cell_size

        if abs(diff) > DRIFT_THRESHOLD:
            # Robot tidak sejajar dengan obstacle
            # Untuk menentukan arah drift, gunakan dist_left vs dist_right
            err_lr = self.dist_left - self.dist_right
            drift  = err_lr * 0.3   # skala kasar
            self.get_logger().warn(
                f'[DRIFT] actual_front={actual_front:.2f}m '
                f'ideal={ideal_front:.2f}m diff={diff:.2f}m '
                f'est_drift={drift:.3f}m')
            return drift

        return 0.0

    # ══════════════════════════════════════════════════════
    #  UPDATE HEADING setelah TURN
    # ══════════════════════════════════════════════════════
    def _update_heading(self, action: str):
        turn_map = {
            ('N', 'RL'): 'W', ('N', 'RR'): 'E',
            ('S', 'RL'): 'E', ('S', 'RR'): 'W',
            ('E', 'RL'): 'N', ('E', 'RR'): 'S',
            ('W', 'RL'): 'S', ('W', 'RR'): 'N',
        }
        self._current_heading = turn_map.get(
            (self._current_heading, action), self._current_heading)

    # ══════════════════════════════════════════════════════
    #  CONTROL LOOP
    # ══════════════════════════════════════════════════════
    def _control_loop(self):
        if self.state in ('INIT', 'FINISHED'):
            return

        cmd       = Twist()
        yaw_error = self.target_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # ── READ_COMMAND ──────────────────────────────────
        if self.state == 'READ_COMMAND':
            if self.current_step_idx >= len(self.commands):
                self.get_logger().info('FINISH REACHED!')
                self.state = 'FINISHED'
                return

            action = self.commands[self.current_step_idx]

            if action in ('RL', 'RR'):
                self._update_heading(action)
                delta = 1.5708 if action == 'RL' else -1.5708
                self.target_yaw = math.atan2(
                    math.sin(self.target_yaw + delta),
                    math.cos(self.target_yaw + delta))
                self.state = 'TURN'

            elif action == 'F':
                self.use_back_laser   = (self.dist_front > 3.0)
                self.start_move_time  = self.get_clock().now()
                self.accumulated_distance = self.kompensasi_jarak
                self.last_laser       = (self.dist_back
                                          if self.use_back_laser
                                          else self.dist_front)
                self.kompensasi_jarak = 0.0

                # Hitung jarak ideal ke dinding untuk koreksi lateral
                self._compute_ideal_wall_distances()
                self._wall_correction_active = True

                # Cek drift sebelum mulai maju (jika menghadap ke obstacle)
                drift = self._check_forward_drift()
                if abs(drift) > 0.05:
                    self.get_logger().warn(
                        f'[DRIFT] Drift terdeteksi sebelum maju: '
                        f'{drift:.3f}m → koreksi kompensasi')
                    self.kompensasi_jarak += drift * 0.5  # partial correction

                self.get_logger().info(
                    f'[{self.current_step_idx}] Maju (F) '
                    f'heading={self._current_heading}')
                self.state = 'MOVE_CELL'

        # ── TURN ─────────────────────────────────────────
        elif self.state == 'TURN':
            if abs(yaw_error) > 0.05:
                cmd.angular.z = 30.5 * yaw_error
                cmd.angular.z = max(-self.angular_speed,
                                    min(self.angular_speed, cmd.angular.z))
            else:
                cmd.angular.z     = 0.0
                cmd.linear.x      = 0.0
                self.pub_cmd.publish(cmd)
                self.brake_start_time = self.get_clock().now()
                self.state = 'BRAKE'

        # ── BRAKE ────────────────────────────────────────
        elif self.state == 'BRAKE':
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            elapsed = (self.get_clock().now()
                       - self.brake_start_time).nanoseconds / 1e9
            if elapsed >= self.BRAKE_DURATION:
                self.current_step_idx += 1
                self.state = 'READ_COMMAND'

        # ── MOVE_CELL ─────────────────────────────────────
        elif self.state == 'MOVE_CELL':
            current_laser = (self.dist_back
                             if self.use_back_laser
                             else self.dist_front)

            # Delta tracking
            if (not math.isinf(current_laser) and
                    not math.isinf(self.last_laser)):
                delta = (self.last_laser - current_laser
                         if not self.use_back_laser
                         else current_laser - self.last_laser)
                if abs(delta) > 0.15:
                    self.get_logger().warn(
                        f'Slip {delta:.2f}m diabaikan')
                else:
                    self.accumulated_distance += delta
            self.last_laser = current_laser

            # Timer failsafe
            waktu_berlalu  = (self.get_clock().now()
                              - self.start_move_time).nanoseconds / 1e9
            waktu_maksimal = (self.cell_size / self.linear_speed) + 0.4

            obstacle_stop = (not math.isinf(self.dist_front)
                             and self.dist_front < 0.3)

            # Kondisi berhenti
            if (self.accumulated_distance >= self.cell_size
                    or waktu_berlalu >= waktu_maksimal
                    or obstacle_stop):

                if obstacle_stop:
                    drift_nyata  = self.lateral_drift
                    next_action  = ('F' if self.current_step_idx + 1
                                    >= len(self.commands)
                                    else self.commands[
                                        self.current_step_idx + 1])
                    if next_action == 'RR':
                        self.kompensasi_jarak = -drift_nyata
                    elif next_action == 'RL':
                        self.kompensasi_jarak = drift_nyata
                    else:
                        self.kompensasi_jarak = 0.0
                    alasan = 'HALANGAN'
                elif self.accumulated_distance >= self.cell_size:
                    alasan = 'LiDAR Delta'
                else:
                    alasan = 'TIMER'

                self._wall_correction_active = False
                self.get_logger().info(
                    f'Pindah kotak via {alasan}. '
                    f'Dist={self.accumulated_distance:.2f}m')

                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                self.current_step_idx += 1
                self.state = 'READ_COMMAND'
                return

            else:
                err_deg = abs(math.degrees(yaw_error))

                # Jika error besar → koreksi di tempat dulu
                if err_deg > 10.0:
                    cmd.linear.x  = 0.0
                    cmd.angular.z = max(-self.angular_speed,
                                        min(self.angular_speed,
                                            4.0 * yaw_error))
                    self.get_logger().warn(
                        f'Koreksi di tempat: err={err_deg:.1f}°')

                else:
                    cmd.linear.x = self.linear_speed

                    # Koreksi yaw dari IMU
                    yaw_corr = 30.13 * yaw_error
                    yaw_corr = max(-5.0, min(5.0, yaw_corr))

                    # Koreksi tambahan dari dinding (jika aktif)
                    wall_corr = 0.0
                    if self._wall_correction_active:
                        wall_corr = self._wall_correction()

                    cmd.angular.z = yaw_corr + wall_corr

                    if wall_corr != 0.0:
                        self.get_logger().debug(
                            f'yaw_corr={yaw_corr:.3f} '
                            f'wall_corr={wall_corr:.3f} '
                            f'total={cmd.angular.z:.3f}')

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GridNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()