import os
import json
import cv2
import numpy as np
import socket
import threading
import queue
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

# Matikan spam log error biner dari OpenCV libjpeg
os.environ["OPENCV_LOG_LEVEL"] = "OFF"

UDP_IP = "0.0.0.0"
UDP_PORT = 5009

DASHBOARD_UDP_IP = "127.0.0.1"
DASHBOARD_UDP_PORT = 5006
TARGET_COMMAND_IP = "127.0.0.1"
TARGET_COMMAND_PORT = 5007

# Gerobak antrean untuk menjembatani antar-thread
raw_data_queue = queue.Queue(maxsize=1)
processed_display_queue = queue.Queue(maxsize=1)

# Variabel Global untuk Video Stream ke Web
stream_frame = None
stream_lock = threading.Lock()

dashboard_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target_color_lock = threading.Lock()

def send_dashboard_telemetry(payload):
    try:
        dashboard_socket.sendto(
            json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            (DASHBOARD_UDP_IP, DASHBOARD_UDP_PORT),
        )
    except Exception:
        pass

def get_target_color():
    with target_color_lock:
        return TARGET_COLOR

def set_target_color(target_color):
    global TARGET_COLOR
    if target_color in {"MERAH", "HIJAU", "BIRU"}:
        with target_color_lock:
            TARGET_COLOR = target_color
        print(f"TARGET COLOR UPDATED: {TARGET_COLOR}")

def target_command_listener_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((TARGET_COMMAND_IP, TARGET_COMMAND_PORT))
    print(f"Target command listener aktif di port {TARGET_COMMAND_PORT}...")

    while True:
        try:
            packet, _ = sock.recvfrom(1024)
            if not packet:
                continue

            message = json.loads(packet.decode("utf-8"))
            target_color = message.get("target_color") or message.get("targetColor")
            if isinstance(target_color, str):
                set_target_color(target_color.strip().upper())
        except Exception:
            pass

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
    print("Thread 2 (Algoritma Vision) Aktif...")

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
            mask_merah = ((r_norm > 0.38) & ((r_norm - g_norm) > 0.08) & (total_rgb > min_brightness)).astype(np.uint8) * 255
            mask_hijau = ((g_norm > 0.38) & ((g_norm - r_norm) > 0.08) & (total_rgb > min_brightness)).astype(np.uint8) * 255
            mask_biru  = ((b_norm > 0.38) & ((b_norm - g_norm) > 0.08) & (total_rgb > min_brightness)).astype(np.uint8) * 255

            masks = {"MERAH": mask_merah, "HIJAU": mask_hijau, "BIRU": mask_biru}
            current_target_color = get_target_color()
            target_mask = masks[current_target_color]

            frame_h, frame_w = frame.shape[:2]
            camera_center_x = frame_w // 2
            camera_center_y = frame_h // 2
            error = 0
            command = "STOP"
            target_visible = False
            target_x = None
            target_y = None
            target_area = 0

            contours, _ = cv2.findContours(target_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            cv2.line(frame, (camera_center_x, 0), (camera_center_x, frame_h), (255,255,255), 2)
            left_limit = camera_center_x - CENTER_TOLERANCE
            right_limit = camera_center_x + CENTER_TOLERANCE
            cv2.line(frame, (left_limit,0), (left_limit,frame_h), (0,255,0), 1)
            cv2.line(frame, (right_limit,0), (right_limit,frame_h), (0,255,0), 1)

            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 500:
                    target_visible = True
                    target_area = int(cv2.contourArea(largest))
                    x,y,w,h = cv2.boundingRect(largest)
                    target_x = x + w//2
                    target_y = y + h//2
                    error = target_x - camera_center_x

                    if abs(error) < CENTER_TOLERANCE: command = "F"
                    elif error < 0: command = "L"
                    else: command = "R"

                    cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,255), 2)
                    cv2.circle(frame, (target_x,target_y), 8, (0,255,255), -1)
                    cv2.line(frame, (camera_center_x,camera_center_y), (target_x,target_y), (0,255,255), 2)

            current_time = time.time()
            fps = 1.0 / max(current_time - prev_time, 0.001)
            prev_time = current_time

            cv2.putText(frame, f"TARGET: {current_target_color}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            cv2.putText(frame, f"ERROR: {error}", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(frame, f"CMD: {command}", (10,90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(frame, f"FPS: {fps:.1f}", (10,120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)

            if processed_display_queue.full():
                try: processed_display_queue.get_nowait()
                except queue.Empty: pass

            telemetry_payload = {
                "source": "deteksi_warna",
                "mode": "computer_vision",
                "target_color": current_target_color,
                "command": command,
                "error": int(error),
                "fps": round(float(fps), 2),
                "target_visible": target_visible,
                "target_area": int(target_area),
                "target_x": target_x,
                "target_y": target_y,
                "frame_width": int(frame_w),
                "frame_height": int(frame_h),
                "timestamp": time.time(),
            }

            processed_display_queue.put_nowait((frame, target_mask, command, telemetry_payload))

        except Exception as e:
            print("PROCESS ERROR:", e)

# =====================================================================
# THREAD 4: SERVER VIDEO STREAM (Kirim Tampilan OpenCV ke Web)
# =====================================================================
class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args): pass # Matikan log spam HTTP server

    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            while True:
                try:
                    with stream_lock:
                        frame_to_send = stream_frame.copy() if stream_frame is not None else None

                    if frame_to_send is not None:
                        ret, jpeg = cv2.imencode('.jpg', frame_to_send)
                        if ret:
                            self.wfile.write(b'--frame\r\n')
                            self.send_header('Content-Type', 'image/jpeg')
                            self.send_header('Content-Length', str(len(jpeg.tobytes())))
                            self.end_headers()
                            self.wfile.write(jpeg.tobytes())
                            self.wfile.write(b'\r\n')
                    time.sleep(0.05) # Batasi stream ~20 FPS biar enteng
                except Exception:
                    break
        else:
            self.send_response(404)
            self.end_headers()

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer): pass

def mjpeg_server_thread():
    server = ThreadedHTTPServer(('0.0.0.0', 5008), MJPEGHandler)
    print("Thread 4 (Video Stream) aktif di port 5008...")
    server.serve_forever()

# =====================================================================
# THREAD 3 (MAIN THREAD): JALUR ANTARMUKA GRAFIS (GUI Render)
# =====================================================================
if __name__ == "__main__":
    threading.Thread(target=udp_receiver_thread, daemon=True).start()
    threading.Thread(target=target_command_listener_thread, daemon=True).start()
    threading.Thread(target=image_processing_thread, daemon=True).start()
    threading.Thread(target=mjpeg_server_thread, daemon=True).start() # Jalankan Web Stream

    print("Main System Berjalan. Membaca gambar & update web...")

    while True:
        if not processed_display_queue.empty():
            try:
                frame, target_mask, command, telemetry_payload = processed_display_queue.get_nowait()

                target_mask_bgr = cv2.cvtColor(target_mask, cv2.COLOR_GRAY2BGR)
                target_mask_bgr = cv2.resize(target_mask_bgr, (frame.shape[1], frame.shape[0]))
                dashboard = np.hstack((frame, target_mask_bgr))

                # Update frame untuk Web Dashboard Stream
                with stream_lock:
                    stream_frame = dashboard

                # Kalau kamu masih mau window OpenCV muncul di laptop, biarkan ini:
                cv2.imshow("Color Tracking Dashboard", dashboard)
                
                send_dashboard_telemetry(telemetry_payload)
            except Exception as e:
                print("GUI ERROR:", e)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()