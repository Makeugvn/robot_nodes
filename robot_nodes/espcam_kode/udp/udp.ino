#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_camera.h"
#include "esp_wifi.h" // --- WAJIB: Untuk membuka fitur eksklusif 802.11n & TX Power ---

// --- PARAMETER JARINGAN KAMU ---
const char* ssid = "THINKDAN";
const char* password = "A7-66o91";
const char* udpAddress = "192.168.137.1"; // IPv4 Laptop ThinkPad Kamu
const int udpPort = 5009;

WiFiUDP udp;

// Pin AI-Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define UDP_PACKET_MAX_SIZE 1430 
#define FLASH_LED_PIN      4

void setup() {
  Serial.begin(115200);

  // --- NYALAKAN LED FLASH ---
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW); // Set ke HIGH untuk menyalakan, LOW untuk mematikan
  // --------------------------

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM; config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  
  config.xclk_freq_hz = 12000000;    
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA; 
  config.jpeg_quality = 35;          
  config.fb_count = 1;

  // 1. Inisialisasi Kamera
  if(esp_camera_init(&config) != ESP_OK){
    Serial.println("Kamera Gagal!");
    return;
  }

  // 2. Kunci Sensor Kamera ke Mode Manual (Anti-Lag Perubahan Cahaya)
  sensor_t * s = esp_camera_sensor_get(); // <-- Deklarasi pointer 's' di sini
  if (s != NULL) {
    //s->set_whitebal(s, 0);       // Matikan Auto White Balance
    //s->set_gain_ctrl(s, 0);      // Matikan Auto Gain
    //s->set_exposure_ctrl(s, 0);  // Matikan Auto Exposure
    s->set_contrast(s, 1);       // Dongkrak kontras agar warna merah tegas
  }

  // 3. Sambungkan ke Hotspot Laptop
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print("."); }
  Serial.println("\nWiFi Terhubung!");
  
  // 4. Pengondisian Hardware WiFi (Maksimal Bandwidth)
  WiFi.setSleep(false); // Matikan mode hemat daya WiFi
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N); 
  esp_wifi_set_max_tx_power(80); // Set daya pancar ke 20dBm (Maksimal)

  udp.begin(udpPort);
}

void loop() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) return;

  int imgSize = fb->len;
  uint8_t *imgBuf = fb->buf;
  int bufferPointer = 0;

  // Pemotongan paket gambar untuk dikirim via UDP
  while (imgSize > 0) {
    int packetSize = (imgSize > UDP_PACKET_MAX_SIZE) ? UDP_PACKET_MAX_SIZE : imgSize;

    udp.beginPacket(udpAddress, udpPort);
    udp.write(imgBuf + bufferPointer, packetSize);
    udp.endPacket();

    imgSize -= packetSize;
    bufferPointer += packetSize;
    
    // Beri jeda mikrodetik agar buffer internal modem radio tidak tersedak
    delayMicroseconds(10); 
  }

  // Akhir Frame (0xFF)
  udp.beginPacket(udpAddress, udpPort);
  udp.write((uint8_t)0xFF); 
  udp.endPacket();

  esp_camera_fb_return(fb);
  
  delay(10); 
}
