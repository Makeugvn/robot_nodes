#!/usr/bin/env python3
"""
robot_visualizer_node.py
========================
Visualisasi Lengkap (Portrait Layout):
- Orientasi robot 3D (dari /imu)
- Radar LiDAR & Target Gap (dari /scan & /avoidance_state)
- Kecepatan linear & angular (dari /odom)
"""

import rclpy
import math
import re
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from rclpy.node import Node

# Pesan ROS
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

HISTORY = 100

def euler_from_quaternion(x, y, z, w):
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sinp  = 2*(w*y - z*x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw

def rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    return Rz @ Ry @ Rx


class RobotVisualizerNode(Node):
    def __init__(self):
        super().__init__('robot_visualizer_node')
        
        # ── Subscriber ──────────────────────────────────────────
        self.create_subscription(Imu, '/imu', self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/avoidance_state', self._state_cb, 10)

        # ── Data IMU & Odom ─────────────────────────────────────
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_th = 0.0
        self.odom_vx = 0.0
        self.odom_wz = 0.0

        self.yaw_hist = deque([0.0]*HISTORY, maxlen=HISTORY)
        self.vx_hist  = deque([0.0]*HISTORY, maxlen=HISTORY)
        self.wz_hist  = deque([0.0]*HISTORY, maxlen=HISTORY)
        self.pkt = 0

        # ── Data LiDAR & Avoidance ──────────────────────────────
        self.current_ranges = []
        self.angle_min = 0.0
        self.angle_inc = 0.0
        self.target_angle_rad = None
        self.robot_state_str = "IDLE"

    # ── Callbacks ───────────────────────────────────────────────
    def _imu_cb(self, msg):
        o = msg.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(o.x, o.y, o.z, o.w)
        self.yaw_hist.append(math.degrees(self.yaw))
        self.pkt += 1

    def _odom_cb(self, msg):
        p = msg.pose.pose
        o = p.orientation
        self.odom_x  = p.position.x
        self.odom_y  = p.position.y
        self.odom_th = math.degrees(math.atan2(2*(o.w*o.z + o.x*o.y), 1 - 2*(o.y*o.y + o.z*o.z)))
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_wz = msg.twist.twist.angular.z
        self.vx_hist.append(self.odom_vx)
        self.wz_hist.append(self.odom_wz)

    def _scan_cb(self, msg):
        self.current_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment

    def _state_cb(self, msg):
        self.robot_state_str = msg.data
        match = re.search(r'Target:\s*([-\d.]+)', msg.data)
        if match:
            deg = float(match.group(1))
            self.target_angle_rad = math.radians(deg)
        else:
            if "FINISH" in msg.data or "BACKUP" in msg.data:
                self.target_angle_rad = None


def main():
    rclpy.init()
    node = RobotVisualizerNode()
    # ROS2 berjalan di background thread agar tidak bentrok dengan Matplotlib
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # ── Figure Setup: Format Vertikal (3 Baris x 2 Kolom) ────────
    fig = plt.figure(figsize=(10, 14)) # Tinggi jendela dimaksimalkan
    fig.patch.set_facecolor('#f5f5f3')
    gs = fig.add_gridspec(3, 2, hspace=0.45, wspace=0.35)

    ax3d     = fig.add_subplot(gs[0, 0], projection='3d')   # Baris 1 Kiri
    ax_lidar = fig.add_subplot(gs[0, 1], projection='polar')# Baris 1 Kanan
    ax_yaw   = fig.add_subplot(gs[1, 0])                    # Baris 2 Kiri
    ax_inf   = fig.add_subplot(gs[1, 1])                    # Baris 2 Kanan
    ax_vx    = fig.add_subplot(gs[2, 0])                    # Baris 3 Kiri
    ax_wz    = fig.add_subplot(gs[2, 1])                    # Baris 3 Kanan
    
    ax_inf.axis('off')

    # ── 1. Setup 3D IMU ──────────────────────────────────────────
    ax3d.set_xlim(-1.2, 1.2); ax3d.set_ylim(-1.2, 1.2); ax3d.set_zlim(-1.2, 1.2)
    ax3d.set_xlabel('X'); ax3d.set_ylabel('Y'); ax3d.set_zlabel('Z')
    ax3d.set_title('Orientasi 3D', fontsize=11)
    ax3d.set_facecolor('#f0f0ee')
    ax3d.tick_params(labelsize=7)
    ax3d.set_box_aspect([1, 1, 1])

    for vec, col in [([1,0,0],'#FFAAAA'),([0,1,0],'#AAFFAA'),([0,0,1],'#AAAAFF')]:
        ax3d.quiver(0,0,0,*vec,length=1.0,color=col,linewidth=0.8, arrow_length_ratio=0.15,alpha=0.4)

    qX = ax3d.quiver(0,0,0,1,0,0,length=1.0,color='#CC2222',linewidth=2.5)
    qY = ax3d.quiver(0,0,0,0,1,0,length=1.0,color='#228822',linewidth=2.5)
    qZ = ax3d.quiver(0,0,0,0,0,1,length=1.0,color='#2244CC',linewidth=2.5)
    txt3d = ax3d.text2D(0.02, 0.97, '', transform=ax3d.transAxes, fontsize=9, va='top', fontfamily='monospace', bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))

    # ── 2. Setup Radar LiDAR ─────────────────────────────────────
    ax_lidar.set_theta_zero_location("N")
    ax_lidar.set_theta_direction(1)
    ax_lidar.set_ylim(0, 3.5)
    ax_lidar.set_title('LiDAR & Avoidance', fontsize=11, pad=15)
    ax_lidar.set_facecolor('#f0f0ee')
    ax_lidar.tick_params(labelsize=8)
    
    scatter_lidar = ax_lidar.scatter([], [], c='red', s=8, label='Obstacle')
    line_target,  = ax_lidar.plot([], [], c='green', linewidth=4, label='Target')
    scatter_target= ax_lidar.scatter([], [], c='green', marker='^', s=50)
    txt_state     = ax_lidar.text(0.5, -0.15, '', transform=ax_lidar.transAxes, ha='center', color='blue', fontsize=10, fontweight='bold')
    ax_lidar.legend(loc='upper right', bbox_to_anchor=(1.15, 1.15), fontsize=8)

    # ── 3. Setup Grafik History ──────────────────────────────────
    def setup_ax(ax, title, ylabel, color):
        ax.set_facecolor('#f0f0ee')
        ax.set_title(title, fontsize=10)
        ax.set_ylabel(ylabel)
        ax.axhline(0, color='#ccc', linewidth=0.8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
        line, = ax.plot([], color=color, linewidth=1.5)
        txt = ax.text(0.03, 0.92, '', transform=ax.transAxes, fontsize=10, fontweight='bold', color=color)
        return line, txt

    line_yaw, txt_yaw = setup_ax(ax_yaw, 'Yaw / Heading', '°', '#185FA5')
    ax_yaw.set_ylim(-185, 185)
    
    line_vx, txt_vx = setup_ax(ax_vx, 'Linear (Vx)', 'm/s', '#D85A30')
    line_wz, txt_wz = setup_ax(ax_wz, 'Angular (Wz)', 'rad/s', '#639922')

    txt_info = ax_inf.text(0.05, 0.95, '', transform=ax_inf.transAxes, fontsize=11, va='top', fontfamily='monospace', bbox=dict(boxstyle='round', facecolor='white', edgecolor='#ddd', alpha=0.9))

    # ── 4. ANIMATION LOOP ────────────────────────────────────────
    def animate(_):
        nonlocal qX, qY, qZ

        # -- Update 3D IMU --
        R = rotation_matrix(node.roll, node.pitch, node.yaw)
        x_ax = R @ np.array([1,0,0])
        y_ax = R @ np.array([0,1,0])
        z_ax = R @ np.array([0,0,1])
        qX.remove(); qY.remove(); qZ.remove()
        qX = ax3d.quiver(0,0,0,*x_ax,length=1.0,color='#CC2222',linewidth=2.5)
        qY = ax3d.quiver(0,0,0,*y_ax,length=1.0,color='#228822',linewidth=2.5)
        qZ = ax3d.quiver(0,0,0,*z_ax,length=1.0,color='#2244CC',linewidth=2.5)

        rd, pd, yd = math.degrees(node.roll), math.degrees(node.pitch), math.degrees(node.yaw)
        txt3d.set_text(f'Roll : {rd:+7.1f}°\nPitch: {pd:+7.1f}°\nYaw  : {yd:+7.1f}°')

        # -- Update LiDAR Radar --
        if len(node.current_ranges) > 0:
            angles = np.array([node.angle_min + i * node.angle_inc for i in range(len(node.current_ranges))])
            ranges = np.array(node.current_ranges)
            
            valid_mask = ~np.isinf(ranges) & ~np.isnan(ranges) & (ranges > 0.05)
            scatter_lidar.set_offsets(np.c_[angles[valid_mask], ranges[valid_mask]])

            if node.target_angle_rad is not None:
                line_target.set_data([0, node.target_angle_rad], [0, 2.0])
                scatter_target.set_offsets(np.c_[node.target_angle_rad, 2.0])
            else:
                line_target.set_data([], [])
                scatter_target.set_offsets(np.empty((0, 2)))

            txt_state.set_text(f"[{node.robot_state_str}]")

        # -- Update Grafik Histori --
        line_yaw.set_ydata(list(node.yaw_hist))
        line_yaw.set_xdata(range(len(node.yaw_hist)))
        ax_yaw.set_xlim(0, max(1, len(node.yaw_hist)))
        txt_yaw.set_text(f'{yd:.1f}°')

        vx_data = list(node.vx_hist)
        line_vx.set_ydata(vx_data)
        line_vx.set_xdata(range(len(vx_data)))
        ax_vx.set_xlim(0, max(1, len(vx_data)))
        txt_vx.set_text(f'{node.odom_vx:.3f} m/s')
        maxvx = max(abs(v) for v in vx_data) if vx_data else 0.5
        ax_vx.set_ylim(-(maxvx*1.3+0.05), maxvx*1.3+0.05)

        wz_data = list(node.wz_hist)
        line_wz.set_ydata(wz_data)
        line_wz.set_xdata(range(len(wz_data)))
        ax_wz.set_xlim(0, max(1, len(wz_data)))
        txt_wz.set_text(f'{node.odom_wz:.3f} rad/s')
        maxwz = max(abs(v) for v in wz_data) if wz_data else 1.0
        ax_wz.set_ylim(-(maxwz*1.3+0.1), maxwz*1.3+0.1)

        # -- Update Teks Odom --
        txt_info.set_text(
            f'Data Odometry\n'
            f'─────────────\n'
            f'X    : {node.odom_x:+.3f} m\n'
            f'Y    : {node.odom_y:+.3f} m\n'
            f'θ    : {node.odom_th:+.1f}°\n'
            f'─────────────\n'
            f'Vx   : {node.odom_vx:+.3f} m/s\n'
            f'Wz   : {node.odom_wz:+.3f} rad/s\n'
            f'─────────────\n'
            f'Paket: {node.pkt}'
        )

        fig.suptitle(f'Robot Dashboard  —  Yaw: {yd:.1f}°  |  Vx: {node.odom_vx:.3f} m/s', fontsize=12, fontweight='bold', color='#333')

    ani = animation.FuncAnimation(fig, animate, interval=50, blit=False, cache_frame_data=False)
    plt.show()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()