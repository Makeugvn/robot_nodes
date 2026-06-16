#!/usr/bin/env python3
"""
obstacle_avoidance_node.py (IMU + Adjustable Bias)
==================================================
Reactive obstacle avoidance dengan IMU, Cutoff Angle, 
dan parameter Bias Kiri/Kanan yang bisa diatur via ROS2.
"""

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

def euler_from_quaternion(x, y, z, w):
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sinp  = 2*(w*y - z*x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # ── Parameter Utama ────────────────────────────────
        self.declare_parameter('linear_speed',      0.8)    
        self.declare_parameter('angular_speed',     4.5)    
        self.declare_parameter('stop_dist',         0.5)   
        self.declare_parameter('finish_detect_dist',0.35)   # Dikecilkan agar tidak mudah finish
        self.declare_parameter('finish_sectors',    6)      # Dinaikkan agar tidak mudah finish
        self.declare_parameter('scan_topic',        '/scan')
        self.declare_parameter('cmd_vel_topic',     '/cmd_vel')
        self.declare_parameter('enabled',           True)
        
        # ── Parameter Tuning Navigasi ──────────────────────
        # cutoff_angle: Sudut mati IMU. 70.0 = ketat lurus, 90.0 = lebih longgar belok.
        self.declare_parameter('cutoff_angle',      85.0)
        # bias_weight: Nilai > 0.0 (Bias KIRI), Nilai < 0.0 (Bias KANAN), 0.0 (Netral namun rentan bias kanan bawaan)
        self.declare_parameter('bias_weight',       0.05)

        self._load_params()

        # ── Publisher & Subscriber ─────────────────────────
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_state = self.create_publisher(String, '/avoidance_state', 10)

        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/avoidance_enable', self._enable_cb, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, qos_profile_sensor_data)

        self._state       = 'IDLE'
        self._scan_count  = 0
        self.initial_yaw  = None
        self.current_yaw  = 0.0

        self.create_timer(1.0, self._log_status)
        self.get_logger().info('Obstacle Avoidance Node (Adjustable Bias) siap')

    def _load_params(self):
        self.linear_speed       = self.get_parameter('linear_speed').value
        self.angular_speed      = self.get_parameter('angular_speed').value
        self.stop_dist          = self.get_parameter('stop_dist').value
        self.finish_detect_dist = self.get_parameter('finish_detect_dist').value
        self.finish_sectors     = self.get_parameter('finish_sectors').value
        self.scan_topic         = self.get_parameter('scan_topic').value
        self.cmd_vel_topic      = self.get_parameter('cmd_vel_topic').value
        self.enabled            = self.get_parameter('enabled').value
        
        self.cutoff_angle       = self.get_parameter('cutoff_angle').value
        self.bias_weight        = self.get_parameter('bias_weight').value

    def _enable_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'on':
            self.enabled = True
            self.get_logger().info('Obstacle avoidance: ON')
        elif cmd == 'off':
            self.enabled = False
            self._publish_cmd(0.0, 0.0)
            self.get_logger().info('Obstacle avoidance: OFF')

    def _imu_cb(self, msg: Imu):
        o = msg.orientation
        _, _, yaw = euler_from_quaternion(o.x, o.y, o.z, o.w)
        if self.initial_yaw is None:
            self.initial_yaw = yaw
            self.get_logger().info(f'>> INITIAL YAW DIKUNCI: {math.degrees(yaw):.2f}° <<')
        self.current_yaw = yaw

    def _get_idx(self, check_deg, angle_min, angle_inc, num_ranges):
        rad = math.radians(check_deg)
        diff = rad - angle_min
        while diff < 0.0:
            diff += 2.0 * math.pi
        while diff >= 2.0 * math.pi:
            diff -= 2.0 * math.pi
        idx = int(diff / angle_inc)
        if 0 <= idx < num_ranges:
            return idx
        return -1

    def _sector_min(self, ranges, angle_min_deg, angle_max_deg, scan_angle_min, scan_angle_inc):
        vals = []
        for deg in range(int(angle_min_deg), int(angle_max_deg) + 1):
            idx = self._get_idx(deg, scan_angle_min, scan_angle_inc, len(ranges))
            if idx != -1:
                r = ranges[idx]
                if not math.isinf(r) and not math.isnan(r) and r > 0.05:
                    vals.append(r)
        return min(vals) if vals else float('inf')

    # ══════════════════════════════════════════════════════
    #  LOGIKA FIND THE GAP (IMU + Corridor Mode)
    # ══════════════════════════════════════════════════════
    def _best_direction_imu(self, ranges, scan_angle_min, scan_angle_inc):
        target_deg = 0.0 
        if self.initial_yaw is not None:
            heading_error_rad = self.initial_yaw - self.current_yaw
            heading_error_rad = math.atan2(math.sin(heading_error_rad), math.cos(heading_error_rad))
            target_deg = math.degrees(heading_error_rad)

        best_score = -9999.0
        best_angle = 0

        # Tarik nilai parameter bias (aman dari error jika parameter belum dideklarasi ulang)
        try:
             current_bias = self.get_parameter('bias_weight').value
             current_cutoff = self.get_parameter('cutoff_angle').value
        except rclpy.exceptions.ParameterNotDeclaredException:
             current_bias = self.bias_weight
             current_cutoff = self.cutoff_angle

        # ── 1. DETEKSI LORONG (CORRIDOR MODE) ──
        # Cek sisi Kiri (30° s.d 90°) dan Kanan (-90° s.d -30°)
        dist_left  = self._sector_min(ranges, 30, 90, scan_angle_min, scan_angle_inc)
        dist_right = self._sector_min(ranges, -90, -30, scan_angle_min, scan_angle_inc)
        
        # Jika kedua bahu robot diapit pilar pada jarak < 0.6 meter
        is_in_corridor = (dist_left < 0.6 and dist_right < 0.6)

        for candidate_deg in range(-90, 90, 2):
            score_sum = 0.0
            valid_rays = 0
            is_blocked = False 

            for offset in range(-4, 5, 1):
                check_deg = candidate_deg + offset
                idx = self._get_idx(check_deg, scan_angle_min, scan_angle_inc, len(ranges))
                
                if idx != -1:
                    r = ranges[idx]
                    
                    if not math.isnan(r) and not math.isinf(r) and r < 0.3:
                        is_blocked = True
                        break 
                        
                    if math.isnan(r) or math.isinf(r) or r > 2.0:
                        score_sum += 2.0
                    else:
                        score_sum += r
                        
                    valid_rays += 1

            if is_blocked or valid_rays == 0:
                continue

            avg_distance = score_sum / valid_rays

            # Pembobotan IMU (Cutoff Dinamis)
            diff_deg = candidate_deg - target_deg
            safe_cutoff = max(1.0, current_cutoff)
            scaled_diff = (abs(diff_deg) / safe_cutoff) * 90.0
            
            imu_weight = math.cos(math.radians(scaled_diff))
            imu_weight = max(0.0, imu_weight)

            final_score = avg_distance * imu_weight
            
            # Tie-Breaker (Meminimalkan Putaran)
            final_score -= (abs(candidate_deg) * 0.001)

            # Aplikasi Parameter Bias
            if current_bias > 0.0 and candidate_deg > 0:
                final_score += current_bias
            elif current_bias < 0.0 and candidate_deg < 0:
                final_score += abs(current_bias)

            # ── 2. EKSEKUSI BONUS LORONG ──
            # Jika sedang diapit pilar Kiri & Kanan, beri poin super masif (+10.0)
            # HANYA untuk arah lurus (0°).
            # Karena skor normal maksimal hanya ~2.0, arah 0° pasti menang telak!
            if is_in_corridor and candidate_deg == 0:
                final_score += 10.0

            if final_score > best_score:
                best_score = final_score
                best_angle = candidate_deg

        return best_angle
    # ══════════════════════════════════════════════════════
    #  SCAN CALLBACK
    # ══════════════════════════════════════════════════════
    def _scan_cb(self, msg: LaserScan):
        if not self.enabled:
            return

        self._scan_count += 1
        ranges        = msg.ranges
        angle_min     = msg.angle_min      
        angle_inc     = msg.angle_increment

        blocked = 0
        sector_size = 45
        for start in range(-180, 180, sector_size):
            d = self._sector_min(ranges, start, start+sector_size, angle_min, angle_inc)
            # Menggunakan parameter yang sudah diload
            if d < self.finish_detect_dist:
                blocked += 1

        if blocked >= self.finish_sectors:
            self._state = 'FINISH'
            self._publish_cmd(0.0, 0.0)
            self._publish_state('FINISH')
            return

        front_min = self._sector_min(ranges, -15, 15, angle_min, angle_inc)
        if front_min < self.stop_dist:
            self._state = 'BACKUP'
            self._publish_cmd(-self.linear_speed * 1.0, 0.0)
            self._publish_state(f'BACKUP (Front: {front_min:.2f}m)')
            return

        best_angle_deg = self._best_direction_imu(ranges, angle_min, angle_inc)
        best_angle_rad = math.radians(best_angle_deg)

        Kp = 1.5 
        angular_z = Kp * best_angle_rad
        angular_z = max(-self.angular_speed, min(self.angular_speed, angular_z))

        front_min_check = self._sector_min(ranges, -20, 20, angle_min, angle_inc)
        
        if front_min_check < 0.45 and abs(best_angle_deg) > 15:
            linear_x = 0.0
            self._state = f'TURN_IN_PLACE (Target: {best_angle_deg}°)'
        else:
            speed_factor = max(0.0, 1.0 - (abs(best_angle_deg) / 70.0))
            linear_x = self.linear_speed * speed_factor

            if front_min_check < 0.45 and abs(best_angle_deg) > 25:
                linear_x = 0.0

            self._state = f'FOLLOW_GAP (Target: {best_angle_deg}°)'

        self._publish_cmd(linear_x, angular_z)
        self._publish_state(self._state)

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
            self.get_logger().info(f'State: {self._state}')

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