#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import math
from collections import deque

# --- FUNGSI HELPER: Konversi Quaternion ke Euler (Derajat) ---
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z)

# --- NODE ROS 2 ---
class RpyVisualizerNode(Node):
    def __init__(self):
        super().__init__('rpy_visualizer')
        
        # Subscribe ke /imu dan /odom menggunakan QoS Sensor Data (BEST_EFFORT)
        self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        
        # Buat array dinamis dengan panjang maksimum 100 data poin (rolling graph)
        self.roll_data = deque([0.0]*100, maxlen=100)
        self.pitch_data = deque([0.0]*100, maxlen=100)
        self.yaw_data = deque([0.0]*100, maxlen=100)
        
    def imu_callback(self, msg):
        q = msg.orientation
        r, p, _ = euler_from_quaternion(q.x, q.y, q.z, q.w)
        # Ambil Roll dan Pitch dari data IMU
        self.roll_data.append(r)
        self.pitch_data.append(p)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, y = euler_from_quaternion(q.x, q.y, q.z, q.w)
        # Ambil Yaw dari data Odometri
        self.yaw_data.append(y)

# --- FUNGSI UTAMA ---
def main(args=None):
    rclpy.init(args=args)
    node = RpyVisualizerNode()

    # Jalankan ROS 2 spin di thread terpisah (Daemon) agar matplotlib tidak freeze
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    # --- SETUP MATPLOTLIB ---
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_title('Real-time Orientasi Robot Beroda (Roll, Pitch, Yaw)')
    ax.set_ylabel('Sudut (Derajat °)')
    ax.set_xlabel('Sampel Data Terakhir')
    ax.set_ylim(-180, 180) # Batas -180 hingga 180 derajat
    ax.grid(True)
    
    # Inisialisasi garis plot
    line_r, = ax.plot(node.roll_data, label='Roll (IMU)', color='red')
    line_p, = ax.plot(node.pitch_data, label='Pitch (IMU)', color='green')
    line_y, = ax.plot(node.yaw_data, label='Yaw (Odom)', color='blue')
    ax.legend(loc='upper right')

    # Fungsi yang dipanggil oleh FuncAnimation secara berulang
    def update_plot(frame):
        line_r.set_ydata(node.roll_data)
        line_p.set_ydata(node.pitch_data)
        line_y.set_ydata(node.yaw_data)
        return line_r, line_p, line_y

    # Animasi berjalan setiap 50 milidetik
    _ani = animation.FuncAnimation(fig, update_plot, interval=50, blit=True)
    
    # Menampilkan window grafik (Ini bersifat blocking)
    plt.show()

    # Jika window ditutup (X diklik), program akan membersihkan node dan mati
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()