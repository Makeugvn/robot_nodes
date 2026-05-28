import os
import cv2
import numpy as np
import threading
import queue
import time

os.environ["OPENCV_LOG_LEVEL"] = "OFF"

raw_data_queue = queue.Queue(maxsize=1)
processed_display_queue = queue.Queue(maxsize=1)

# =====================================================================
# THREAD 1: WEBCAM CAPTURE
# =====================================================================
def webcam_capture_thread(cap):
    print("Thread 1 (Webcam Capture) Aktif...")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Gagal membaca frame dari webcam.")
            time.sleep(0.01)
            continue
        if raw_data_queue.full():
            try: raw_data_queue.get_nowait()
            except queue.Empty: pass
        raw_data_queue.put_nowait(frame)

# =====================================================================
# HELPER: Gabungkan kontur-kontur yang berdekatan menjadi satu mask solid
# Caranya: dilasi mask -> temukan kontur gabungan -> isi -> erosi balik
# Ini menangani kasus objek yang "terpotong-potong" akibat pantulan cahaya
# =====================================================================
def merge_nearby_contours(mask, merge_kernel_size=25):
    merge_kernel = np.ones((merge_kernel_size, merge_kernel_size), np.uint8)
    dilated    = cv2.dilate(mask, merge_kernel, iterations=1)
    filled     = dilated.copy()
    cnts, _    = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        cv2.drawContours(filled, [c], -1, 255, thickness=cv2.FILLED)
    eroded = cv2.erode(filled, merge_kernel, iterations=1)
    return eroded

# =====================================================================
# HELPER: Pipeline mask lengkap -> morfologi -> merge -> ambil kontur terbesar
# =====================================================================
def get_best_contour(mask, morph_kernel, area_threshold=800):
    # OPEN: hapus noise bintik kecil
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  morph_kernel, iterations=2)
    # CLOSE: tutup lubang/retakan akibat pantulan
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, morph_kernel, iterations=2)
    # Gabungkan fragmen kontur yang berdekatan
    merged = merge_nearby_contours(closed, merge_kernel_size=25)
    # Ambil kontur terbesar
    cnts, _ = cv2.findContours(merged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, merged
    largest = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(largest) < area_threshold:
        return None, merged
    return largest, merged

# =====================================================================
# HELPER: Gambar bounding box + titik pusat + label dari convex hull
# =====================================================================
def annotate(frame, contour, color_bgr, label):
    hull       = cv2.convexHull(contour)
    x, y, w, h = cv2.boundingRect(hull)
    cx = int(x + w / 2)
    cy = int(y + h / 2)
    cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 2)
    cv2.circle(frame, (cx, cy), 8, color_bgr, -1)
    cv2.putText(frame, label, (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

# =====================================================================
# THREAD 2: IMAGE PROCESSING -- DETEKSI MULTI-WARNA (R, G, B)
# =====================================================================
def image_processing_thread():
    print("Thread 2 (Multi-Color Tracking) Aktif...")

    # Kernel morfologi -- ukuran 9x9 cukup agresif menutup lubang sedang
    morph_kernel = np.ones((9, 9), np.uint8)

    # ------------------------------------------------------------------
    # RENTANG HSV
    # Merah: dua rentang karena hue merah melingkar di 0/180
    lower_red1 = np.array([0,   120,  60]);  upper_red1 = np.array([10,  255, 255])
    lower_red2 = np.array([160, 120,  60]);  upper_red2 = np.array([180, 255, 255])

    # Hijau: diperlebar ke bawah (35) agar hijau tua tertangkap,
    #        dan ke atas (95) agar hijau kekuningan juga ikut
    lower_green = np.array([35,  60,  40]);  upper_green = np.array([95,  255, 255])

    # Biru: diperlebar ke bawah (85) agar cyan/biru muda (botol) tertangkap,
    #       saturation diturunkan ke 50 agar warna pucat tetap terdeteksi
    lower_blue  = np.array([85,  50,  40]);  upper_blue  = np.array([145, 255, 255])
    # ------------------------------------------------------------------

    while True:
        if not raw_data_queue.empty():
            try:
                frame = raw_data_queue.get_nowait()
                if frame is None:
                    continue

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                # Buat mask mentah
                mask_r1    = cv2.inRange(hsv, lower_red1,  upper_red1)
                mask_r2    = cv2.inRange(hsv, lower_red2,  upper_red2)
                mask_merah = cv2.bitwise_or(mask_r1, mask_r2)
                mask_hijau = cv2.inRange(hsv, lower_green, upper_green)
                mask_biru  = cv2.inRange(hsv, lower_blue,  upper_blue)

                # Proses tiap warna: morfologi -> merge -> hull -> anotasi
                cnt_m, clean_m = get_best_contour(mask_merah, morph_kernel)
                if cnt_m is not None:
                    annotate(frame, cnt_m, (0, 0, 255), "MERAH")

                cnt_h, clean_h = get_best_contour(mask_hijau, morph_kernel)
                if cnt_h is not None:
                    annotate(frame, cnt_h, (0, 255, 0), "HIJAU")

                cnt_b, clean_b = get_best_contour(mask_biru, morph_kernel)
                if cnt_b is not None:
                    annotate(frame, cnt_b, (255, 0, 0), "BIRU")

                # Jendela mask gabungan (tampilkan mask SETELAH morfologi)
                m_small = cv2.resize(clean_m, (160, 120))
                h_small = cv2.resize(clean_h, (160, 120))
                b_small = cv2.resize(clean_b, (160, 120))
                jendela_mask = np.hstack((m_small, h_small, b_small))

                if processed_display_queue.full():
                    try: processed_display_queue.get_nowait()
                    except queue.Empty: pass
                processed_display_queue.put_nowait((frame, jendela_mask))

            except Exception:
                pass
        else:
            time.sleep(0.001)

# =====================================================================
# THREAD 3 (MAIN THREAD): GUI RENDER
# =====================================================================
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("ERROR: Tidak dapat membuka webcam.")
        exit()

    print("Webcam berhasil dibuka. Tekan 'q' untuk keluar...")

    threading.Thread(target=webcam_capture_thread, args=(cap,), daemon=True).start()
    threading.Thread(target=image_processing_thread, daemon=True).start()

    while True:
        if not processed_display_queue.empty():
            try:
                frame, mask_gabungan = processed_display_queue.get_nowait()
                cv2.imshow("Mata Robot (Triple-Thread)", frame)
                cv2.imshow("Mask Gabungan M|H|B", mask_gabungan)
            except Exception:
                pass

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Webcam dilepas. Program selesai."),