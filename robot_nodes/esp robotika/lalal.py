import cv2
import numpy as np

# --- 2. PASTIKAN IP INI MASIH SAMA DENGAN DI ARDUINO ---
URL = "http://192.168.137.169/stream" 

if __name__ == "__main__":
    print(f"Mencoba terhubung ke kamera robot di: {URL}")
    
    # Membaca stream HTTP secara native (Jauh lebih stabil & anti-lag)
    cap = cv2.VideoCapture(URL)
    
    if not cap.isOpened():
        print("Gagal membuka aliran video. Cek IP Address atau koneksi WiFi!")
        exit()

    print("Terhubung! Membuka layar video...")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Gagal mengambil frame.")
            break

        # Proses filter warna merah
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        mask_merah = mask1 + mask2

        cv2.imshow("Kamera Mata Robot (HTTP)", frame)
        cv2.imshow("Filter Benda Merah (HTTP)", mask_merah)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()