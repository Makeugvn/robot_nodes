#!/usr/bin/env python3
"""
grid_navigator_node_v1.py
=========================
Versi 1: Delta Tracking / Akumulasi Jarak (Gagasan Kamu).
Menggunakan penjumlahan delta (selisih). Jika laser melompat, 
lompatan diabaikan, namun jarak yang sudah ditempuh tetap disimpan 
dan acuan laser diperbarui.
"""

import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data

from path_planner import get_path, path_to_relative_commands, MANUAL_PATH
from grid_map import START_ROBOT2, FINISH_ROBOT2, CELL_SIZE

def euler_from_quaternion(x, y, z, w):
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sinp  = 2*(w*y - z*x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))    
    return yaw


class GridNavigatorNode(Node):
    def __init__(self):
        super().__init__('grid_navigator')

        # --- Parameter Robot ---
        self.cell_size = CELL_SIZE       
        self.linear_speed = 0.3          
        self.angular_speed = 1.0         

        # --- Konfigurasi Path ---
        self.current_cell = START_ROBOT2
        self.goal_cell    = FINISH_ROBOT2
        self.manual_path = MANUAL_PATH
        
        # --- State Sensor Default ---
        self.initial_yaw = None
        self.current_yaw = 0.0
        self.dist_front = 0.0
        self.dist_back  = 0.0
        
        # --- State Machine ---
        self.state = 'INIT' 
        self.commands = []
        self.current_step_idx = 0
        self.target_yaw = 0.0
        self.use_back_laser = False
        self.start_move_time = None
        
        # --- STATE DELTA TRACKING & TRIGONOMETRI KOMPENSASI ---
        self.accumulated_distance = 0.0  # Tabungan jarak saat maju
        self.last_laser = 0.0            # Jarak laser di siklus sebelumnya (untuk cari Delta)
        self.lateral_drift = 0.0         # Jarak melenceng kiri/kanan (hasil Trigonometri)
        self.kompensasi_jarak = 0.0      # Nilai kompensasi yang akan dipakai untuk maju (F) selanjutnya

        # --- Publisher & Subscriber ---
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self._imu_cb, qos_profile_sensor_data)

        self.timer = self.create_timer(0.05, self._control_loop) 
        self.get_logger().info("Eksekutor Grid (Delta + Trigonometri) Activating...")

    def _imu_cb(self, msg: Imu):
        yaw = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        
        if self.initial_yaw is None:
            self.initial_yaw = yaw
            self.target_yaw = yaw
            path = get_path(self.current_cell, self.goal_cell, manual_matrix=self.manual_path, obstacle_margin=0)
            self.commands = path_to_relative_commands(path, initial_heading='S')
            self.state = 'READ_COMMAND' if self.commands else 'FINISHED'
            
        self.current_yaw = yaw

    def _scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        num_rays = len(ranges)
        if num_rays == 0: return

        # 1. Jarak lurus ke depan
        front_rays = [ranges[i] for i in [-3, -2, -1, 0, 1, 2, 3] if 0 <= i < num_rays or num_rays+i < num_rays]
        valid_front = [r for r in front_rays if not math.isinf(r) and not math.isnan(r) and r > 0.1]
        self.dist_front = np.min(valid_front) if valid_front else float('inf')

        # ── ALGORITMA TRIGONOMETRI: DETEKSI TEPI (EDGE DETECTION) ──
        self.lateral_drift = 0.0  # Positif = Robot melenceng ke Kiri, Negatif = ke Kanan
        
        if self.dist_front < 0.6: # Hanya jalankan jika ada halangan dekat
            edge_left_y = None
            edge_right_y = None
            base_r = self.dist_front
            
            # Sapu ke Kiri (0 s/d +80 derajat)
            for deg in range(1, 81):
                r = ranges[deg]
                if math.isinf(r) or math.isnan(r) or (r - base_r) > 0.2:
                    prev_deg = deg - 1
                    # Menggunakan base_r untuk mencegah error NaN saat mengalikan infinity
                    edge_left_y = base_r * math.sin(math.radians(prev_deg))
                    break

            # Sapu ke Kanan (0 s/d -80 derajat)
            for deg in range(1, 81):
                idx = -deg
                r = ranges[idx]
                if math.isinf(r) or math.isnan(r) or (r - base_r) > 0.2:
                    prev_deg = -(deg - 1)
                    # Menggunakan base_r untuk mencegah error NaN saat mengalikan infinity
                    edge_right_y = base_r * math.sin(math.radians(prev_deg))
                    break

            # Hitung Titik Tengah Obstakel (Y_center)
            Y_center = None
            if edge_left_y is not None and edge_right_y is not None:
                # Terlihat kedua ujung kardus
                Y_center = (edge_left_y + edge_right_y) / 2.0
            elif edge_left_y is not None:
                # Hanya terlihat ujung Kiri. (Pusat kardus = Ujung Kiri - 20cm)
                Y_center = edge_left_y - 0.2
            elif edge_right_y is not None:
                # Hanya terlihat ujung Kanan. (Pusat kardus = Ujung Kanan + 20cm)
                Y_center = edge_right_y + 0.2

            # Jika Y_center ketemu, kita bisa hitung drift robot!
            if Y_center is not None:
                # Jika Y_center Positif (Obstakel di Kiri robot), berarti Robot berada di KANAN obstakel.
                # Drift kita set: Positif = Robot Kiri, Negatif = Robot Kanan.
                self.lateral_drift = -Y_center

        # Laser Belakang
        mid_idx = int(num_rays / 2)
        back_rays = [ranges[i] for i in range(mid_idx-3, mid_idx+4) if 0 <= i < num_rays]
        back_rays = [r for r in back_rays if not math.isinf(r) and not math.isnan(r) and r > 0.1]
        self.dist_back = np.min(back_rays) if back_rays else float('inf')

    # ══════════════════════════════════════════════════════
    #  CONTROL LOOP: State Machine Eksekutor Murni
    # ══════════════════════════════════════════════════════
    def _control_loop(self):
        if self.state == 'INIT' or self.state == 'FINISHED':
            return

        cmd = Twist()
        yaw_error = self.target_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # ── 1. STATE BACA PERINTAH ──
        if self.state == 'READ_COMMAND':
            if self.current_step_idx >= len(self.commands):
                self.get_logger().info("FINISH REACHED!")
                self.state = 'FINISHED'
                return

            action = self.commands[self.current_step_idx]
            
            if action == 'RL':
                self.target_yaw += 1.5708
                self.target_yaw = math.atan2(math.sin(self.target_yaw), math.cos(self.target_yaw))
                self.state = 'TURN'
                
            elif action == 'RR':
                self.target_yaw -= 1.5708
                self.target_yaw = math.atan2(math.sin(self.target_yaw), math.cos(self.target_yaw))
                self.state = 'TURN'
                
            elif action == 'F':
                self.use_back_laser = (self.dist_front > 3.0) 
                self.start_move_time = self.get_clock().now()
                
                # --- APLIKASIKAN KOMPENSASI DI SINI ---
                # Jangan mulai dari 0.0, tapi gunakan sisa kompensasi sebelumnya!
                self.accumulated_distance = self.kompensasi_jarak
                self.last_laser = self.dist_back if self.use_back_laser else self.dist_front
                
                if self.kompensasi_jarak != 0.0:
                    self.get_logger().info(f"[{self.current_step_idx}] Maju 1 Kotak (F) - Bawa Kompensasi: {self.kompensasi_jarak:+.2f}m")
                else:
                    self.get_logger().info(f"[{self.current_step_idx}] Maju 1 Kotak (F) - Normal")
                
                # Reset kompensasi agar tidak dipakai berkali-kali
                self.kompensasi_jarak = 0.0
                self.state = 'MOVE_CELL'

        # ── 2. STATE PUTAR BADAN ──
        elif self.state == 'TURN':
            if abs(yaw_error) > 0.05:  
                cmd.angular.z = 30.5 * yaw_error
                cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, cmd.angular.z))
            else:
                cmd.angular.z = 0.0
                self.current_step_idx += 1
                self.state = 'READ_COMMAND'

        # ── 3. STATE MAJU 1 KOTAK (Delta Tracking + Obstacle Skip) ──
        elif self.state == 'MOVE_CELL':
            current_laser = self.dist_back if self.use_back_laser else self.dist_front
            
            # --- LOGIKA AKUMULASI JARAK (DELTA TRACKING) ---
            if not math.isinf(current_laser) and not math.isinf(self.last_laser):
                if not self.use_back_laser:
                    delta = self.last_laser - current_laser 
                else:
                    delta = current_laser - self.last_laser 
                
                if abs(delta) > 0.15:
                    self.get_logger().warn(f"🚨 Slip Detected! Lompatan {delta:.2f}m diabaikan.")
                else:
                    self.accumulated_distance += delta 
            
            self.last_laser = current_laser

            waktu_berlalu = (self.get_clock().now() - self.start_move_time).nanoseconds / 1e9
            waktu_maksimal = (self.cell_size / self.linear_speed) + 0.3 

            obstacle_stop = (not math.isinf(self.dist_front) and self.dist_front < 0.3)

            # --- KONDISI BERHENTI ---
            if self.accumulated_distance >= self.cell_size or waktu_berlalu >= waktu_maksimal or obstacle_stop:
                
                if obstacle_stop:
                    # Ambil nilai drift lateral dari perhitungan Trigonometri LiDAR
                    drift_nyata = self.lateral_drift 
                    
                    # Cek perintah aksi selanjutnya
                    next_action = "F"
                    if (self.current_step_idx + 1) < len(self.commands):
                        next_action = self.commands[self.current_step_idx + 1]

                    # --- KONVERSI DRIFT LATERAL KE KOMPENSASI LONGITUDINAL ---
                    if next_action == 'RR':
                        # Belok Kanan: Jika drift Kiri (+), butuh jarak tempuh ekstra (-)
                        self.kompensasi_jarak = -drift_nyata 
                    elif next_action == 'RL':
                        # Belok Kiri: Jika drift Kiri (+), hemat jarak tempuh (+)
                        self.kompensasi_jarak = drift_nyata
                    else:
                        self.kompensasi_jarak = 0.0

                    posisi_str = "KIRI" if drift_nyata > 0 else "KANAN"
                    self.get_logger().warn(
                        f"🛑 HALANGAN! Robot melenceng {abs(drift_nyata)*100:.1f}cm ke {posisi_str}.\n"
                        f"➡️ Tabungan kompensasi untuk langkah maju selanjutnya: {self.kompensasi_jarak:+.3f}m"
                    )
                    alasan = "HALANGAN DEPAN"

                elif self.accumulated_distance >= self.cell_size:
                    alasan = "LiDAR Delta"
                else:
                    alasan = "TIMER"
                    
                self.get_logger().info(f">> Pindah kotak via {alasan}. Total Tempuh (Delta): {self.accumulated_distance:.2f}m")
                
                # Berhenti Total
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                
                # Pindah ke perintah selanjutnya
                self.current_step_idx += 1
                self.state = 'READ_COMMAND' 
                return
            else:
                # Terus maju dengan koreksi yaw yang agresif
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 30.13 * yaw_error 
                cmd.angular.z = max(-5.0, min(5.0, cmd.angular.z))

        self.pub_cmd.publish(cmd)

        
def main(args=None):
    rclpy.init(args=args)
    node = GridNavigatorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()
