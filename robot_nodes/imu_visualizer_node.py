#!/usr/bin/env python3
"""
imu_visualizer_node.py
======================
Visualisasi orientasi robot 3D + kecepatan dari encoder
Subscribe: /imu, /odom
"""

import rclpy
import math
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
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


class ImuVisualizerNode(Node):
    def __init__(self):
        super().__init__('imu_visualizer_node')
        self.create_subscription(Imu,      '/imu',  self._imu_cb,  qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_profile_sensor_data)

        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0

        self.odom_x   = 0.0
        self.odom_y   = 0.0
        self.odom_th  = 0.0
        self.odom_vx  = 0.0   # m/s dari encoder (odom_vx)
        self.odom_wz  = 0.0   # rad/s dari encoder (odom_wz)

        self.yaw_hist = deque([0.0]*HISTORY, maxlen=HISTORY)
        self.vx_hist  = deque([0.0]*HISTORY, maxlen=HISTORY)
        self.wz_hist  = deque([0.0]*HISTORY, maxlen=HISTORY)
        self.pkt = 0

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
        self.odom_th = math.degrees(
            math.atan2(2*(o.w*o.z + o.x*o.y), 1 - 2*(o.y*o.y + o.z*o.z)))
        # Kecepatan dari encoder (dikirim di twist)
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_wz = msg.twist.twist.angular.z
        self.vx_hist.append(self.odom_vx)
        self.wz_hist.append(self.odom_wz)


def main():
    rclpy.init()
    node = ImuVisualizerNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # ── Figure: 2x2 grid + 3D axes ─────────────────────
    fig = plt.figure(figsize=(14, 7))
    fig.patch.set_facecolor('#f5f5f3')
    gs = fig.add_gridspec(2, 3, hspace=0.45, wspace=0.35)

    ax3d   = fig.add_subplot(gs[:, 0], projection='3d')   # kiri penuh
    ax_yaw = fig.add_subplot(gs[0, 1])                    # tengah atas
    ax_vx  = fig.add_subplot(gs[0, 2])                    # kanan atas
    ax_wz  = fig.add_subplot(gs[1, 1])                    # tengah bawah
    ax_inf = fig.add_subplot(gs[1, 2])                    # kanan bawah (info teks)
    ax_inf.axis('off')

    # ── 3D axes setup ──────────────────────────────────
    ax3d.set_xlim(-1.2, 1.2)
    ax3d.set_ylim(-1.2, 1.2)
    ax3d.set_zlim(-1.2, 1.2)
    ax3d.set_xlabel('X', color='#AA2222', fontsize=10)
    ax3d.set_ylabel('Y', color='#228822', fontsize=10)
    ax3d.set_zlabel('Z', color='#2244AA', fontsize=10)
    ax3d.set_title('Orientasi Robot (3D)', fontsize=11)
    ax3d.set_facecolor('#f0f0ee')
    ax3d.tick_params(labelsize=7)
    ax3d.set_box_aspect([1, 1, 1])

    # Referensi dunia (abu-abu tipis)
    for vec, col in [([1,0,0],'#FFAAAA'),([0,1,0],'#AAFFAA'),([0,0,1],'#AAAAFF')]:
        ax3d.quiver(0,0,0,*vec,length=1.0,color=col,linewidth=0.8,
                    arrow_length_ratio=0.15,alpha=0.4)

    qX = ax3d.quiver(0,0,0,1,0,0,length=1.0,color='#CC2222',linewidth=2.5,arrow_length_ratio=0.2)
    qY = ax3d.quiver(0,0,0,0,1,0,length=1.0,color='#228822',linewidth=2.5,arrow_length_ratio=0.2)
    qZ = ax3d.quiver(0,0,0,0,0,1,length=1.0,color='#2244CC',linewidth=2.5,arrow_length_ratio=0.2)

    txt3d = ax3d.text2D(0.02, 0.97, '', transform=ax3d.transAxes,
                         fontsize=9, verticalalignment='top', fontfamily='monospace',
                         bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))

    # ── Yaw history ────────────────────────────────────
    ax_yaw.set_facecolor('#f0f0ee')
    ax_yaw.set_title('Yaw / Heading', fontsize=10)
    ax_yaw.set_ylabel('°')
    ax_yaw.set_ylim(-185, 185)
    ax_yaw.axhline(0,  color='#ccc', linewidth=0.8)
    ax_yaw.axhline( 90,color='#ddd', linewidth=0.5, linestyle='--')
    ax_yaw.axhline(-90,color='#ddd', linewidth=0.5, linestyle='--')
    ax_yaw.grid(True, alpha=0.3)
    ax_yaw.tick_params(labelsize=8)
    line_yaw, = ax_yaw.plot(list(node.yaw_hist), color='#185FA5', linewidth=1.5)
    txt_yaw = ax_yaw.text(0.03, 0.92, '', transform=ax_yaw.transAxes,
                           fontsize=11, fontweight='bold', color='#185FA5')

    # ── Kecepatan linear (Vx) dari encoder ─────────────
    ax_vx.set_facecolor('#f0f0ee')
    ax_vx.set_title('Kecepatan Linear (encoder)', fontsize=10)
    ax_vx.set_ylabel('m/s')
    ax_vx.set_ylim(-0.6, 0.6)
    ax_vx.axhline(0, color='#ccc', linewidth=0.8)
    ax_vx.grid(True, alpha=0.3)
    ax_vx.tick_params(labelsize=8)
    line_vx, = ax_vx.plot(list(node.vx_hist), color='#D85A30', linewidth=1.5)
    txt_vx = ax_vx.text(0.03, 0.92, '', transform=ax_vx.transAxes,
                         fontsize=11, fontweight='bold', color='#D85A30')

    # ── Kecepatan angular (Wz) dari encoder ────────────
    ax_wz.set_facecolor('#f0f0ee')
    ax_wz.set_title('Kecepatan Angular (encoder)', fontsize=10)
    ax_wz.set_ylabel('rad/s')
    ax_wz.set_ylim(-4.0, 4.0)
    ax_wz.axhline(0, color='#ccc', linewidth=0.8)
    ax_wz.grid(True, alpha=0.3)
    ax_wz.tick_params(labelsize=8)
    line_wz, = ax_wz.plot(list(node.wz_hist), color='#639922', linewidth=1.5)
    txt_wz = ax_wz.text(0.03, 0.92, '', transform=ax_wz.transAxes,
                         fontsize=11, fontweight='bold', color='#639922')

    # ── Info teks (posisi odom) ─────────────────────────
    txt_info = ax_inf.text(0.05, 0.95, '', transform=ax_inf.transAxes,
                            fontsize=10, verticalalignment='top',
                            fontfamily='monospace',
                            bbox=dict(boxstyle='round', facecolor='white',
                                      edgecolor='#ddd', alpha=0.9))

    def animate(_):
        nonlocal qX, qY, qZ

        # Update 3D axes
        R = rotation_matrix(node.roll, node.pitch, node.yaw)
        x_ax = R @ np.array([1,0,0])
        y_ax = R @ np.array([0,1,0])
        z_ax = R @ np.array([0,0,1])
        qX.remove(); qY.remove(); qZ.remove()
        qX = ax3d.quiver(0,0,0,*x_ax,length=1.0,color='#CC2222',linewidth=2.5,arrow_length_ratio=0.2)
        qY = ax3d.quiver(0,0,0,*y_ax,length=1.0,color='#228822',linewidth=2.5,arrow_length_ratio=0.2)
        qZ = ax3d.quiver(0,0,0,*z_ax,length=1.0,color='#2244CC',linewidth=2.5,arrow_length_ratio=0.2)

        rd = math.degrees(node.roll)
        pd = math.degrees(node.pitch)
        yd = math.degrees(node.yaw)
        txt3d.set_text(f'Roll : {rd:+7.1f}°\n'
                       f'Pitch: {pd:+7.1f}°\n'
                       f'Yaw  : {yd:+7.1f}°')

        # Update yaw
        line_yaw.set_ydata(list(node.yaw_hist))
        txt_yaw.set_text(f'{yd:.1f}°')

        # Update Vx
        vx_data = list(node.vx_hist)
        line_vx.set_ydata(vx_data)
        txt_vx.set_text(f'{node.odom_vx:.3f} m/s')
        # Auto scale Vx jika melebihi batas
        maxvx = max(abs(v) for v in vx_data) if vx_data else 0.5
        ax_vx.set_ylim(-(maxvx*1.3+0.05), maxvx*1.3+0.05)

        # Update Wz
        wz_data = list(node.wz_hist)
        line_wz.set_ydata(wz_data)
        txt_wz.set_text(f'{node.odom_wz:.3f} rad/s')
        maxwz = max(abs(v) for v in wz_data) if wz_data else 1.0
        ax_wz.set_ylim(-(maxwz*1.3+0.1), maxwz*1.3+0.1)

        # Update info
        txt_info.set_text(
            f'Posisi Odom\n'
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

        fig.suptitle(
            f'IMU & Odometry Visualizer  —  '
            f'Yaw: {yd:.1f}°  |  '
            f'Vx: {node.odom_vx:.3f} m/s',
            fontsize=11, color='#333'
        )

    ani = animation.FuncAnimation(
        fig, animate, interval=50, blit=False, cache_frame_data=False)

    plt.show()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()