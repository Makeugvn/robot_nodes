#!/usr/bin/env python3
"""
visual_tracker_node.py
==========================================================
ROS 2 Node: Deteksi warna via klik mouse (UDP ESP32-CAM)
dan kontrol pergerakan otomatis (Maju + Centering Yaw)
berhenti di jarak 0.2m dari target.
"""

import os
import cv2
import numpy as np
import socket
import threading
import queue
import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

# --- Konfigurasi Jaringan Kamera ---
CAM_UDP_IP = "0.0.0.0"
CAM_UDP_PORT = 5009

# --- Variabel Global OpenCV ---
TARGET_COLOR = "KLIK_LAYAR"  
CLICKED_HSV = None
IS_CUSTOM_COLOR = False
latest_hsv_frame = None

raw_data_queue = queue.Queue(maxsize=2)

# =====================================================================
# KELAS ROS 2 NODE (OTAK PERGERAKAN)
# =====================================================================
class VisualTrackerNode(Node):
    def __init__(self):
        super().__init__('visual_tracker')
        
        # --- Parameter Kendali ---
        self.linear_speed = 0.3       # Kecepatan maju (m/s)
        self.max_angular_speed = 5.0  # Batas maksimal putaran (rad/s)
        self.kp_yaw = -10.5             # Gain Proporsional untuk setir
        
        # --- State Visual & Sensor ---
        self.dist_front = float('inf')
        self.target_visible = False
        self.error_x_normalized = 0.0 # Rentang -1.0 (Kiri) s/d 1.0 (Kanan)

        # --- Publisher & Subscriber ---
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        
        # Loop kendali utama berjalan di 20 Hz
        self.timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info("Visual Tracker Node Activating...")

    def _scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        num_rays = len(ranges)
        if num_rays == 0: return

        # Ambil jarak tepat di depan robot (Sektor depan)
        front_rays = [ranges[i] for i in [-3, -2, -1, 0, 1, 2, 3] if 0 <= i < num_rays or num_rays+i < num_rays]
        valid_front = [r for r in front_rays if not math.isinf(r) and not math.isnan(r) and r > 0.05]
        self.dist_front = np.min(valid_front) if valid_front else float('inf')

    def _control_loop(self):
        cmd = Twist()
        
        if self.target_visible:
            # 1. KONTROL ROTASI (Centering Objek)
            # Karena di ROS: Kiri = Positif, Kanan = Negatif
            # Jika objek di kanan layar (error_x positif), kita butuh belok kanan (angular negatif)
            cmd.angular.z = - self.kp_yaw * self.error_x_normalized
            cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd.angular.z))
            
            # 2. KONTROL MAJU (Linear)
            if self.dist_front > 0.25: # Toleransi 5cm agar tidak menabrak keras di 0.2
                cmd.linear.x = self.linear_speed
                self.get_logger().debug(f"Mengejar Target... Jarak depan: {self.dist_front:.2f}m")
            else:
                # Target sudah di depan mata!
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0 # Opsional: hentikan putaran juga
                self.get_logger().info(f"🛑 TARGET TERCAPAI! Jarak: {self.dist_front:.2f}m", throttle_duration_sec=1.0)
        else:
            # Jika warna tidak terlihat, berhenti total
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        self.pub_cmd.publish(cmd)


# =====================================================================
# PENERIMA STREAM VIDEO UDP DARI ESP32-CAM
# =====================================================================
def udp_receiver_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((CAM_UDP_IP, CAM_UDP_PORT))
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024) 
    
    data_buffer = bytearray()
    while True:
        try:
            packet, _ = sock.recvfrom(8192)
            if len(packet) == 1 and packet[0] == 0xFF:
                if len(data_buffer) > 0:
                    if raw_data_queue.full():
                        raw_data_queue.get_nowait()
                    raw_data_queue.put(bytes(data_buffer))
                data_buffer = bytearray()
            else:
                data_buffer.extend(packet)
        except:
            pass

# =====================================================================
# DETEKSI KLIK MOUSE OPENCV
# =====================================================================
def mouse_click_event(event, x, y, flags, param):
    global CLICKED_HSV, IS_CUSTOM_COLOR, latest_hsv_frame
    if event == cv2.EVENT_LBUTTONDOWN:
        if latest_hsv_frame is not None:
            pixel = latest_hsv_frame[y, x]
            CLICKED_HSV = (int(pixel[0]), int(pixel[1]), int(pixel[2]))
            IS_CUSTOM_COLOR = True
            print(f"\n🎯 [TARGET DIKUNCI] Nilai HSV: {CLICKED_HSV}\nRobot akan mulai mengejar!\n")

# =====================================================================
# MAIN THREAD: OPENCV & MENGHIDUPKAN ROS 2
# =====================================================================
def main(args=None):
    global latest_hsv_frame, IS_CUSTOM_COLOR, CLICKED_HSV
    
    os.environ["OPENCV_LOG_LEVEL"] = "OFF"
    
    # Inisialisasi ROS 2
    rclpy.init(args=args)
    node = VisualTrackerNode()
    
    # Jalankan ROS 2 di thread terpisah agar OpenCV bisa jalan di Main Thread
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    # Jalankan UDP Receiver
    threading.Thread(target=udp_receiver_thread, daemon=True).start()
    
    # Persiapan Jendela OpenCV
    window_name = "Mata Robot: Klik untuk Kejar!"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_click_event)
    
    print("[VISION] Menunggu stream video masuk...")

    try:
        while rclpy.ok():
            try:
                jpeg_bytes = raw_data_queue.get(timeout=0.1)
                np_arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame is None:
                    continue

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                latest_hsv_frame = hsv.copy() 
                mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
                
                # Update status Node ke tidak terlihat secara default
                node.target_visible = False
                
                # Hanya cari kontur jika sudah mengklik layar
                if IS_CUSTOM_COLOR and CLICKED_HSV is not None:
                    h, s, v = CLICKED_HSV
                    h_margin = 10
                    s_margin = 60
                    v_margin = 60
                    
                    lower_bound = np.array([max(0, h - h_margin), max(20, s - s_margin), max(20, v - v_margin)])
                    upper_bound = np.array([min(179, h + h_margin), min(255, s + s_margin), min(255, v + v_margin)])
                    mask = cv2.inRange(hsv, lower_bound, upper_bound)
                    
                    mask = cv2.erode(mask, None, iterations=2)
                    mask = cv2.dilate(mask, None, iterations=2)

                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    
                    frame_h, frame_w = frame.shape[:2]
                    camera_center_x = frame_w // 2
                    
                    if contours:
                        largest_contour = max(contours, key=cv2.contourArea)
                        area = cv2.contourArea(largest_contour)
                        
                        if area > 500: # Filter noise
                            M = cv2.moments(largest_contour)
                            if M["m00"] != 0:
                                center_x = int(M["m10"] / M["m00"])
                                center_y = int(M["m01"] / M["m00"])
                                
                                # HITUNG ERROR NORMALISASI UNTUK ROS 2
                                # Jarak piksel dari tengah (Kiri negatif, Kanan positif)
                                error_x_pixels = center_x - camera_center_x 
                                # Skala menjadi -1.0 sampai 1.0
                                error_x_normalized = error_x_pixels / (frame_w / 2.0)
                                
                                # Lempar data ke otak ROS 2
                                node.target_visible = True
                                node.error_x_normalized = error_x_normalized

                                cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
                                cv2.circle(frame, (center_x, center_y), 7, (255, 255, 0), -1)
                                
                                # Visualisasi Garis Panduan Centering
                                cv2.line(frame, (camera_center_x, 0), (camera_center_x, frame_h), (0, 0, 255), 1)
                                cv2.line(frame, (camera_center_x, center_y), (center_x, center_y), (255, 0, 255), 2)

                # Informasi Overlay di Layar
                status_teks = "MENGEJAR TARGET!" if node.target_visible else "MENUNGGU KLIK / TARGET HILANG"
                warna_teks = (0, 255, 0) if node.target_visible else (0, 0, 255)
                cv2.putText(frame, status_teks, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, warna_teks, 2)

                cv2.imshow(window_name, frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except queue.Empty:
                pass
            
    except KeyboardInterrupt:
        pass
    finally:
        # Hentikan robot dengan aman saat mematikan program
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()