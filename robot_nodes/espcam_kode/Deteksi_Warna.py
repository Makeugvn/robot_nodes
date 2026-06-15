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

# Variabel Target
TARGET_COLOR = "HIJAU"
CENTER_TOLERANCE = 20
prev_time = time.time()

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
# THREAD 2: ALGORITMA ARAH GERAK ROBOT dari Visual
# =====================================================================
def image_processing_thread():
    global prev_time

    print("Thread 2 Algoritma Arah Gerak RObot dari Visual Aktif...")

    while True:

        if raw_data_queue.empty():
            time.sleep(0.001)
            continue

        try:
            jpeg_bytes = raw_data_queue.get_nowait()

            np_arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)

            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                continue

            frame_float = frame.astype(np.float32)

            b_ch, g_ch, r_ch = cv2.split(frame_float)

            total_rgb = r_ch + g_ch + b_ch + 0.0001

            r_norm = r_ch / total_rgb
            g_norm = g_ch / total_rgb
            b_norm = b_ch / total_rgb

            min_brightness = 40

            mask_merah = (
                (r_norm > 0.38) &
                ((r_norm - g_norm) > 0.08) &
                (total_rgb > min_brightness)
            ).astype(np.uint8) * 255

            mask_hijau = (
                (g_norm > 0.38) &
                ((g_norm - r_norm) > 0.08) &
                (total_rgb > min_brightness)
            ).astype(np.uint8) * 255

            mask_biru = (
                (b_norm > 0.38) &
                ((b_norm - g_norm) > 0.08) &
                (total_rgb > min_brightness)
            ).astype(np.uint8) * 255

            masks = {
                "MERAH": mask_merah,
                "HIJAU": mask_hijau,
                "BIRU": mask_biru
            }

            target_mask = masks[TARGET_COLOR]

            frame_h, frame_w = frame.shape[:2]

            camera_center_x = frame_w // 2
            camera_center_y = frame_h // 2

            error = 0
            command = "STOP"

            contours, _ = cv2.findContours(
                target_mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            cv2.line(
                frame,
                (camera_center_x, 0),
                (camera_center_x, frame_h),
                (255,255,255),
                2
            )

            left_limit = camera_center_x - CENTER_TOLERANCE
            right_limit = camera_center_x + CENTER_TOLERANCE

            cv2.line(
                frame,
                (left_limit,0),
                (left_limit,frame_h),
                (0,255,0),
                1
            )

            cv2.line(
                frame,
                (right_limit,0),
                (right_limit,frame_h),
                (0,255,0),
                1
            )

            if contours:

                largest = max(contours, key=cv2.contourArea)

                if cv2.contourArea(largest) > 500:

                    x,y,w,h = cv2.boundingRect(largest)

                    target_x = x + w//2
                    target_y = y + h//2

                    error = target_x - camera_center_x

                    if abs(error) < CENTER_TOLERANCE:
                        command = "F"
                    elif error < 0:
                        command = "L"
                    else:
                        command = "R"

                    cv2.rectangle(
                        frame,
                        (x,y),
                        (x+w,y+h),
                        (0,255,255),
                        2
                    )

                    cv2.circle(
                        frame,
                        (target_x,target_y),
                        8,
                        (0,255,255),
                        -1
                    )

                    cv2.line(
                        frame,
                        (camera_center_x,camera_center_y),
                        (target_x,target_y),
                        (0,255,255),
                        2
                    )

            current_time = time.time()

            fps = 1.0 / max(
                current_time - prev_time,
                0.001
            )

            prev_time = current_time

            cv2.putText(
                frame,
                f"TARGET: {TARGET_COLOR}",
                (10,30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0,255,255),
                2
            )

            cv2.putText(
                frame,
                f"ERROR: {error}",
                (10,60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255,255,255),
                2
            )

            cv2.putText(
                frame,
                f"CMD: {command}",
                (10,90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0,255,0),
                2
            )

            cv2.putText(
                frame,
                f"FPS: {fps:.1f}",
                (10,120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255,255,0),
                2
            )

            if processed_display_queue.full():
                try:
                    processed_display_queue.get_nowait()
                except queue.Empty:
                    pass

            processed_display_queue.put_nowait(
                (frame, target_mask, command)
            )

        except Exception as e:
            print("PROCESS ERROR:", e)

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

                frame, target_mask, command = \
                    processed_display_queue.get_nowait()

                target_mask_bgr = cv2.cvtColor(
                    target_mask,
                    cv2.COLOR_GRAY2BGR
                )

                target_mask_bgr = cv2.resize(
                    target_mask_bgr,
                    (frame.shape[1], frame.shape[0])
                )

                dashboard = np.hstack(
                    (frame, target_mask_bgr)
                )

                cv2.imshow(
                    "Color Tracking Dashboard",
                    dashboard
                )

                print(command)

            except Exception as e:
                print("GUI ERROR:", e)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()