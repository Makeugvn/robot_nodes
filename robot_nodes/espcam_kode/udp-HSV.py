import os
import cv2
import numpy as np
import socket
import threading
import queue
import time

# Matikan spam log error biner dari OpenCV libjpeg
os.environ["OPENCV_LOG_LEVEL"] = "OFF"

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

# Dua gerobak antrean untuk menjembatani antar-thread
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
                    # 1. Konversi ke HSV untuk threshold warna yang lebih stabil
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                    # Rentang HSV untuk tiap warna (nilai awal yang umum — tweak jika perlu)
                    # Red: note ada dua rentang karena hue merah melingkar di 0/180
                    lower_red1 = np.array([0, 120, 50])
                    upper_red1 = np.array([10, 255, 255])
                    lower_red2 = np.array([160, 120, 50])
                    upper_red2 = np.array([180, 255, 255])

                    # Green
                    lower_green = np.array([40, 70, 50])
                    upper_green = np.array([90, 255, 255])

                    # Blue
                    lower_blue = np.array([100, 70, 50])
                    upper_blue = np.array([140, 255, 255])

                    # Buat mask untuk setiap warna
                    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
                    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
                    mask_merah = cv2.bitwise_or(mask_red1, mask_red2)
                    mask_hijau = cv2.inRange(hsv, lower_green, upper_green)
                    mask_biru = cv2.inRange(hsv, lower_blue, upper_blue)

                    # 3. TRACKING & ANOTASI (Menggambar Kotak/Titik untuk Tiap Warna)
                    # --- Pelacak Merah ---
                    cnt_m, _ = cv2.findContours(mask_merah, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if cnt_m:
                        largest = max(cnt_m, key=cv2.contourArea)
                        if cv2.contourArea(largest) > 500:
                            M = cv2.moments(largest)
                            if M["m00"] != 0:
                                cx = int(M["m10"] / M["m00"])
                                cy = int(M["m01"] / M["m00"])
                                cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1) # Titik Merah
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
                                cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1) # Titik Hijau
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
                                cv2.circle(frame, (cx, cy), 8, (255, 0, 0), -1) # Titik Biru
                                cv2.putText(frame, "BIRU", (cx+15, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    # 4. Gabungkan 3 Mask menjadi satu layar monitor (samping-sampingan/atas-bawah) biar tidak kebanyakan window
                    # Kita kecilkan resolusi mask-nya biar muat di monitor saat dijejerkan
                    m_small = cv2.resize(mask_merah, (160, 120))
                    h_small = cv2.resize(mask_hijau, (160, 120))
                    b_small = cv2.resize(mask_biru, (160, 120))
                    
                    # Satukan 3 gambar biner secara horizontal
                    jendela_mask_gabungan = np.hstack((m_small, h_small, b_small))

                    # 5. Kirim hasil jadi ke antrean GUI
                    if processed_display_queue.full():
                        try: processed_display_queue.get_nowait()
                        except queue.Empty: pass
                    processed_display_queue.put_nowait((frame, jendela_mask_gabungan))
            except Exception: pass
        else:
            time.sleep(0.001)

# =====================================================================
# THREAD 3 (MAIN THREAD): JALUR ANTARMUKA GRAFIS (GUI Render)
# =====================================================================
if __name__ == "__main__":
    # Bangunkan Thread 1 dan Thread 2 di latar belakang
    threading.Thread(target=udp_receiver_thread, daemon=True).start()
    threading.Thread(target=image_processing_thread, daemon=True).start()

    print("Thread 3 (Main GUI) Berjalan. Membuka layar monitor...")

    while True:
        if not processed_display_queue.empty():
            try:
                frame, mask_merah = processed_display_queue.get_nowait()
                
                # Tampilkan hasil tracking ter-optimasi
                cv2.imshow("Mata Robot (Triple-Thread)", frame)
                cv2.imshow("Filter Merah (Triple-Thread)", mask_merah)
            except Exception: pass

        # waitKey(1) ideal untuk performa realtime berkecepatan tinggi
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()