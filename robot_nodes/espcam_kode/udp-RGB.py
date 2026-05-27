import os
import cv2
import numpy as np
import socket
import threading
import queue
import time

# --- ROS 2 Imports ---
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Matikan spam log error biner dari OpenCV libjpeg
os.environ["OPENCV_LOG_LEVEL"] = "OFF"

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

# Tiga gerobak antrean untuk menjembatani antar-thread 
# (Frame Tracking, Masking, dan Frame Bersih untuk VO)
raw_data_queue = queue.Queue(maxsize=1)
processed_display_queue = queue.Queue(maxsize=1)

# =====================================================================
# THREAD 1: JALUR KHUSUS JARINGAN (Hanya menangkap paket biner UDP)
# =====================================================================
def udp_receiver_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 524288) 
    
    data_buffer = bytearray()
    print(f"Thread 1 (Jaringan) siap mendengarkan Port {UDP_PORT}...")

    while True:
        try:
            packet, addr = sock.recvfrom(2048)
            if not packet: continue

            if len(packet) == 1 and packet[0] == 0xFF:
                if len(data_buffer) > 0:
                    if raw_data_queue.full():
                        try: raw_data_queue.get_nowait()
                        except queue.Empty: pass
                    raw_data_queue.put_nowait(bytes(data_buffer))
                data_buffer = bytearray()
            else:
                data_buffer.extend(packet)
        except Exception: pass

# =====================================================================
# THREAD 2: JALUR KHUSUS KOMPUTASI VISI (DETEKSI MULTI-WARNA: R, G, B)
# =====================================================================
def image_processing_thread():
    print("Thread 2 (Multi-Color Tracking - Normalized RGB) Aktif...")
    while True:
        if not raw_data_queue.empty():
            try:
                jpeg_bytes = raw_data_queue.get_nowait()
                np_arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    # --- PENTING UNTUK VISUAL ODOMETRY ---
                    # Simpan salinan frame sebelum digambar/dianotasi.
                    # VO akan gagal/kacau jika melacak teks atau lingkaran buatan.
                    clean_frame = frame.copy()

                    # 1. Ubah ke float32 untuk pembagian presisi
                    frame_float = frame.astype(np.float32)
                    b_ch, g_ch, r_ch = cv2.split(frame_float)

                    # Hitung total biner cahaya, beri delta kecil anti-crash
                    total_rgb = r_ch + g_ch + b_ch + 0.0001

                    # Hitung Rasio Normalisasi r, g, dan b
                    r_norm = r_ch / total_rgb
                    g_norm = g_ch / total_rgb
                    b_norm = b_ch / total_rgb  

                    # Batas bawah intensitas cahaya agar area gelap gulita/hitam diabaikan
                    min_brightness = 40

                    # 2. PROSES MASKING
                    mask_merah = ((r_norm > 0.38) & ((r_norm - g_norm) > 0.08) & (total_rgb > min_brightness)).astype(np.uint8) * 255
                    mask_hijau = ((g_norm > 0.38) & ((g_norm - r_norm) > 0.08) & (total_rgb > min_brightness)).astype(np.uint8) * 255
                    mask_biru  = ((b_norm > 0.38) & ((b_norm - g_norm) > 0.08) & (total_rgb > min_brightness)).astype(np.uint8) * 255

                    # 3. TRACKING & ANOTASI (Menggambar Kotak/Titik untuk Tiap Warna pada 'frame')
                    # --- Pelacak Merah ---
                    cnt_m, _ = cv2.findContours(mask_merah, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if cnt_m:
                        largest = max(cnt_m, key=cv2.contourArea)
                        if cv2.contourArea(largest) > 500:
                            M = cv2.moments(largest)
                            if M["m00"] != 0:
                                cx = int(M["m10"] / M["m00"])
                                cy = int(M["m01"] / M["m00"])
                                cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1) 
                                cv2.putText(frame, "MERAH", (cx+15, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    # --- Pelacak Hijau ---
                    cnt_h, _ = cv2.findContours(mask_hijau, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if cnt_h:
                        largest = max(cnt_h, key=cv2.contourArea)
                        if cv2.contourArea(largest) > 500:
                            M = cv2.moments(largest)
                            if M["m00"] != 0:
                                cx = int(M["m10"] / M["m00"])
                                cy = int(M["m01"] / M["m00"])
                                cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1) 
                                cv2.putText(frame, "HIJAU", (cx+15, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # --- Pelacak Biru ---
                    cnt_b, _ = cv2.findContours(mask_biru, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if cnt_b:
                        largest = max(cnt_b, key=cv2.contourArea)
                        if cv2.contourArea(largest) > 500:
                            M = cv2.moments(largest)
                            if M["m00"] != 0:
                                cx = int(M["m10"] / M["m00"])
                                cy = int(M["m01"] / M["m00"])
                                cv2.circle(frame, (cx, cy), 8, (255, 0, 0), -1) 
                                cv2.putText(frame, "BIRU", (cx+15, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    # 4. Gabungkan 3 Mask
                    m_small = cv2.resize(mask_merah, (160, 120))
                    h_small = cv2.resize(mask_hijau, (160, 120))
                    b_small = cv2.resize(mask_biru, (160, 120))
                    
                    jendela_mask_gabungan = np.hstack((m_small, h_small, b_small))

                    # 5. Kirim hasil jadi ke antrean GUI (Sertakan clean_frame)
                    if processed_display_queue.full():
                        try: processed_display_queue.get_nowait()
                        except queue.Empty: pass
                    processed_display_queue.put_nowait((frame, jendela_mask_gabungan, clean_frame))
            except Exception: pass
        else:
            time.sleep(0.001)

# =====================================================================
# THREAD 3 (MAIN THREAD): JALUR ANTARMUKA GRAFIS & ROS 2 PUBLISHER
# =====================================================================
if __name__ == "__main__":
    # Inisialisasi ROS 2 Node
    rclpy.init()
    ros_node = rclpy.create_node('udp_camera_publisher')
    image_pub = ros_node.create_publisher(Image, '/camera/image_raw', 10)
    bridge = CvBridge()

    # Bangunkan Thread 1 dan Thread 2 di latar belakang
    threading.Thread(target=udp_receiver_thread, daemon=True).start()
    threading.Thread(target=image_processing_thread, daemon=True).start()

    print("Thread 3 (Main GUI & ROS 2 Publisher) Berjalan...")

    try:
        while rclpy.ok():
            if not processed_display_queue.empty():
                try:
                    frame, mask_gabungan, clean_frame = processed_display_queue.get_nowait()
                    
                    # --- PUBLISH KE TOPIK ROS ---
                    # Gunakan clean_frame agar VO node tidak mendeteksi tulisan "MERAH/HIJAU" sebagai fitur
                    img_msg = bridge.cv2_to_imgmsg(clean_frame, encoding="bgr8")
                    
                    # Stamp sangat krusial untuk TF tree dan sinkronisasi EKF
                    img_msg.header.stamp = ros_node.get_clock().now().to_msg()
                    img_msg.header.frame_id = "camera_link" # Sesuaikan dengan definisi URDF robot Anda
                    
                    image_pub.publish(img_msg)

                    # --- TAMPILAN OPENCV GUI ---
                    cv2.imshow("Mata Robot (Triple-Thread)", frame)
                    cv2.imshow("Filter RGB (Triple-Thread)", mask_gabungan)
                except Exception as e: 
                    print(f"Error publishing/displaying: {e}")

            # Biarkan rclpy memproses event internal publisher (non-blocking)
            rclpy.spin_once(ros_node, timeout_sec=0)

            # waitKey(1) ideal untuk performa realtime berkecepatan tinggi
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # Bersihkan ROS 2 dan OpenCV secara rapi saat keluar
        ros_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()