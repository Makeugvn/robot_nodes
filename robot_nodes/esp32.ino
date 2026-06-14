/*
 * esp32_rplidar.ino
 * =================
 * RPLidar C1 via UART2 → kirim data scan 360° ke laptop via UDP port 5006
 * Kontrol ON/OFF RPLidar via UDP port 5005
 * Terima cmd_vel via UDP port 5007
 * Kirim IMU + encoder via UDP port 5008
 *
 * Wiring RPLidar C1:
 *   RPLidar TX  → ESP32 GPIO16 (RX2)
 *   RPLidar RX  → ESP32 GPIO17 (TX2)
 *   RPLidar VCC → 5V
 *   RPLidar GND → GND
 */

#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

// ═══════════════════════════════════════════════════════════
//  KONFIGURASI
// ═══════════════════════════════════════════════════════════
const char* WIFI_SSID  = "SENDANG REJEKI";
const char* WIFI_PASS  = "HELLY123";
const char* LAPTOP_IP  = "192.168.100.160";

#define PORT_LIDAR    5005
#define PORT_SCAN     5006
#define PORT_CMDVEL   5007
#define PORT_SENSOR   5008

// ── RPLidar C1 ────────────────────────────────────────────
#define RPLIDAR_RX_PIN       16
#define RPLIDAR_TX_PIN       17
#define RPLIDAR_BAUDRATE     460800

#define RPLIDAR_CMD_SYNC_BYTE   0xA5
#define RPLIDAR_CMD_SCAN        0x20
#define RPLIDAR_CMD_STOP        0x25
#define RPLIDAR_CMD_RESET       0x40
#define RPLIDAR_CMD_GET_HEALTH  0x52

#define RPLIDAR_RESP_SYNC1      0xA5
#define RPLIDAR_RESP_SYNC2      0x5A
#define RPLIDAR_SCAN_PACKET_SIZE 5

// ── Motor L298N ───────────────────────────────────────────
#define MOTOR_A_IN1   25
#define MOTOR_A_IN2   26
#define MOTOR_A_EN    27
#define MOTOR_B_IN1   32
#define MOTOR_B_IN2   33
#define MOTOR_B_EN    14

#define PWM_FREQ      1000
#define PWM_RES       8

// ── Encoder ───────────────────────────────────────────────
#define ENCODER_A_PIN  34
#define ENCODER_B_PIN  35

// ── IMU MPU6050 ───────────────────────────────────────────
#define MPU6050_ADDR   0x68
#define PWR_MGMT_1     0x6B
#define ACCEL_XOUT_H   0x3B

#define MAX_SPEED_MS   0.5f
#define WHEEL_BASE     0.15f
#define HALF_SCAN      180
#define CMDVEL_TIMEOUT_MS  500
#define SENSOR_INTERVAL_MS  50

// ═══════════════════════════════════════════════════════════
//  VARIABEL GLOBAL
// ═══════════════════════════════════════════════════════════
bool     lidarRunning  = false;
bool     lidarScanning = false;
uint32_t scanSeq       = 0;

uint16_t scanBuffer[360];
bool     scanValid[360];
int      sampleCount = 0;

WiFiUDP udpLidar;
WiFiUDP udpScan;
WiFiUDP udpCmdVel;
WiFiUDP udpSensor;

volatile long encTicksA = 0;
volatile long encTicksB = 0;

float gyroZ  = 0.0f;
float accelX = 0.0f;
float accelY = 0.0f;

float cmdLinear    = 0.0f;
float cmdAngular   = 0.0f;
unsigned long lastCmdVelTime = 0;
unsigned long lastSensorSend = 0;

char jsonBuf[1600];

// ═══════════════════════════════════════════════════════════
//  ISR ENCODER
// ═══════════════════════════════════════════════════════════
void IRAM_ATTR isrEncoderA() { encTicksA++; }
void IRAM_ATTR isrEncoderB() { encTicksB++; }

// ═══════════════════════════════════════════════════════════
//  RPLidar
// ═══════════════════════════════════════════════════════════
void rplidarSendCmd(uint8_t cmd) {
  Serial2.write(RPLIDAR_CMD_SYNC_BYTE);
  Serial2.write(cmd);
}

void rplidarFlush() {
  delay(10);
  while (Serial2.available()) Serial2.read();
}

bool rplidarWaitDescriptor(uint32_t timeoutMs = 3000) {
  uint32_t start = millis();
  uint8_t  buf[7];
  int      idx   = 0;

  while (millis() - start < timeoutMs) {
    if (!Serial2.available()) continue;
    uint8_t b = Serial2.read();

    if (idx == 0) {
      if (b == RPLIDAR_RESP_SYNC1) buf[idx++] = b;
      continue;
    }
    if (idx == 1) {
      if (b == RPLIDAR_RESP_SYNC2) buf[idx++] = b;
      else idx = 0;
      continue;
    }
    buf[idx++] = b;
    if (idx >= 7) {
      Serial.print("[LIDAR] Descriptor: ");
      for (int i = 0; i < 7; i++) Serial.printf("0x%02X ", buf[i]);
      Serial.println();
      return true;
    }
  }
  Serial.println("[LIDAR] Descriptor timeout!");
  return false;
}

bool rplidarStartScan() {
  Serial.println("[LIDAR] Reset...");
  rplidarSendCmd(RPLIDAR_CMD_STOP);
  delay(200);
  rplidarFlush();

  rplidarSendCmd(RPLIDAR_CMD_RESET);
  Serial.print("[LIDAR] Boot");

  uint32_t lastByteTime = millis();
  uint32_t bootTimeout  = millis();
  while (millis() - bootTimeout < 5000) {
    if (Serial2.available()) {
      Serial2.read();
      lastByteTime = millis();
    } else if (millis() - lastByteTime > 500) {
      break;
    }
    if (millis() % 500 < 10) Serial.print(".");
  }
  Serial.println(" selesai");
  rplidarFlush();

  // Health check
  rplidarSendCmd(RPLIDAR_CMD_GET_HEALTH);
  uint8_t  hBuf[10];
  int      hIdx  = 0;
  bool     foundA5 = false;
  uint32_t hStart  = millis();
  while (millis() - hStart < 2000 && hIdx < 10) {
    if (Serial2.available()) {
      uint8_t b = Serial2.read();
      if (!foundA5 && b == 0xA5) foundA5 = true;
      if (foundA5) hBuf[hIdx++] = b;
    }
  }
  if (hIdx >= 10) {
    uint8_t status = hBuf[7];
    Serial.printf("[LIDAR] Health: %s\n",
      status == 0 ? "GOOD" : status == 1 ? "WARNING" : "ERROR");
    if (status == 2) return false;
  }
  rplidarFlush();

  // Kirim SCAN
  rplidarSendCmd(RPLIDAR_CMD_SCAN);
  if (!rplidarWaitDescriptor(5000)) return false;

  for (int i = 0; i < 360; i++) { scanBuffer[i] = 0; scanValid[i] = false; }
  sampleCount   = 0;
  lidarScanning = true;
  Serial.println("[LIDAR] Scan dimulai ✓");
  return true;
}

void rplidarStopScan() {
  rplidarSendCmd(RPLIDAR_CMD_STOP);
  delay(10);
  rplidarFlush();
  lidarScanning = false;
  Serial.println("[LIDAR] Scan berhenti");
}

void rplidarMotorOff() {
  rplidarSendCmd(RPLIDAR_CMD_STOP);
  delay(100);
  rplidarSendCmd(RPLIDAR_CMD_RESET);
  delay(500);
  rplidarFlush();
}

bool rplidarParseScanPacket(uint8_t* pkt,
                             float& angle_deg, float& dist_mm,
                             bool& newScan, bool& isDataValid) {
  uint8_t  quality  = pkt[0] >> 2;
  bool     startBit = (pkt[0] & 0x01) != 0;
  bool     checkBit = (pkt[1] & 0x01) != 0;
  if (!checkBit) return false;

  uint16_t angle_q6 = ((uint16_t)(pkt[2]) << 7) | (pkt[1] >> 1);
  uint32_t dist_q2  = ((uint32_t)(pkt[4]) << 8) | pkt[3];

  angle_deg   = angle_q6 / 64.0f;
  dist_mm     = dist_q2  /  4.0f;
  newScan     = startBit;
  isDataValid = (quality > 0 && dist_mm >= 10.0f);
  return true;
}

bool rplidarReadPackets() {
  static uint8_t pktBuf[RPLIDAR_SCAN_PACKET_SIZE];
  static int     pktIdx  = 0;
  static bool    synced  = false;
  bool fullScan = false;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (!synced) {
      if ((b & 0x03) == 0x01) { pktBuf[0] = b; pktIdx = 1; synced = true; }
      continue;
    }

    pktBuf[pktIdx++] = b;
    if (pktIdx < RPLIDAR_SCAN_PACKET_SIZE) continue;
    pktIdx = 0;

    float angle_deg, dist_mm;
    bool  newScan, isDataValid;
    if (!rplidarParseScanPacket(pktBuf, angle_deg, dist_mm, newScan, isDataValid)) {
      synced = false;
      continue;
    }

    if (newScan) { fullScan = true; scanSeq++; }

    if (isDataValid) {
      int idx = (int)angle_deg % 360;
      if (idx >= 0 && idx < 360) {
        if (!scanValid[idx] || dist_mm < scanBuffer[idx]) {
          scanBuffer[idx] = (uint16_t)dist_mm;
          scanValid[idx]  = true;
        }
      }
      sampleCount++;
    }
  }
  return fullScan;
}

// ═══════════════════════════════════════════════════════════
//  KIRIM SCAN UDP
// ═══════════════════════════════════════════════════════════
void sendScanUDP() {
  IPAddress targetIP;
  targetIP.fromString(LAPTOP_IP);

  // Paket 1: 0–179°
  int offset = 0;
  offset += snprintf(jsonBuf + offset, sizeof(jsonBuf) - offset,
    "{\"seq\":%lu,\"part\":1,\"start\":0,\"distances\":[", (unsigned long)scanSeq);
  for (int i = 0; i < HALF_SCAN; i++) {
    offset += snprintf(jsonBuf + offset, sizeof(jsonBuf) - offset,
      "%d", scanValid[i] ? (int)scanBuffer[i] : -1);
    if (i < HALF_SCAN - 1) jsonBuf[offset++] = ',';
  }
  offset += snprintf(jsonBuf + offset, sizeof(jsonBuf) - offset, "]}");
  udpScan.beginPacket(targetIP, PORT_SCAN);
  udpScan.write((uint8_t*)jsonBuf, offset);
  int r1 = udpScan.endPacket();

  delay(15);

  // Paket 2: 180–359°
  offset = 0;
  offset += snprintf(jsonBuf + offset, sizeof(jsonBuf) - offset,
    "{\"seq\":%lu,\"part\":2,\"start\":180,\"distances\":[", (unsigned long)scanSeq);
  for (int i = HALF_SCAN; i < 360; i++) {
    offset += snprintf(jsonBuf + offset, sizeof(jsonBuf) - offset,
      "%d", scanValid[i] ? (int)scanBuffer[i] : -1);
    if (i < 359) jsonBuf[offset++] = ',';
  }
  offset += snprintf(jsonBuf + offset, sizeof(jsonBuf) - offset, "]}");
  udpScan.beginPacket(targetIP, PORT_SCAN);
  udpScan.write((uint8_t*)jsonBuf, offset);
  int r2 = udpScan.endPacket();

  Serial.printf("[SCAN] Seq=%lu samples=%d pkt1=%s pkt2=%s\n",
    (unsigned long)scanSeq, sampleCount,
    r1 ? "OK" : "FAIL", r2 ? "OK" : "FAIL");

  for (int i = 0; i < 360; i++) { scanBuffer[i] = 0; scanValid[i] = false; }
  sampleCount = 0;
}

// ═══════════════════════════════════════════════════════════
//  LIDAR CONTROL
// ═══════════════════════════════════════════════════════════
void checkLidarControl() {
  int pktSize = udpLidar.parsePacket();
  if (pktSize <= 0) return;

  char buf[32];
  int  len = udpLidar.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;
  buf[len] = '\0';

  String cmd = String(buf);
  cmd.trim(); cmd.toLowerCase();

  if (cmd == "on") {
    if (!lidarRunning) {
      if (rplidarStartScan()) lidarRunning = true;
      else Serial.println("[CMD] Gagal start scan!");
    }
  } else if (cmd == "off") {
    if (lidarRunning) {
      rplidarStopScan();
      rplidarMotorOff();
      lidarRunning = false;
    }
  } else if (cmd == "status") {
    Serial.printf("[STATUS] lidar=%s scanning=%s\n",
      lidarRunning ? "ON" : "OFF", lidarScanning ? "YES" : "NO");
  }
}

// ═══════════════════════════════════════════════════════════
//  IMU
// ═══════════════════════════════════════════════════════════
void initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() < 14) return;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  Wire.read(); Wire.read();
  Wire.read(); Wire.read();
  Wire.read(); Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  accelX = ax / 16384.0f * 9.81f;
  accelY = ay / 16384.0f * 9.81f;
  gyroZ  = gz / 131.0f * (M_PI / 180.0f);
}

// ═══════════════════════════════════════════════════════════
//  MOTOR
// ═══════════════════════════════════════════════════════════
void initMotors() {
  ledcAttach(MOTOR_A_EN, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_B_EN, PWM_FREQ, PWM_RES);
  pinMode(MOTOR_A_IN1, OUTPUT); pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT); pinMode(MOTOR_B_IN2, OUTPUT);
  digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW); digitalWrite(MOTOR_B_IN2, LOW);
  ledcWrite(MOTOR_A_EN, 0);
  ledcWrite(MOTOR_B_EN, 0);
}

void setMotorA(float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  int pwm = (int)(fabsf(speed) * 255);
  if      (speed >  0.02f) { digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);  }
  else if (speed < -0.02f) { digitalWrite(MOTOR_A_IN1, LOW);  digitalWrite(MOTOR_A_IN2, HIGH); }
  else                     { digitalWrite(MOTOR_A_IN1, LOW);  digitalWrite(MOTOR_A_IN2, LOW); pwm = 0; }
  ledcWrite(MOTOR_A_EN, pwm);
}

void setMotorB(float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  int pwm = (int)(fabsf(speed) * 255);
  if      (speed >  0.02f) { digitalWrite(MOTOR_B_IN1, HIGH); digitalWrite(MOTOR_B_IN2, LOW);  }
  else if (speed < -0.02f) { digitalWrite(MOTOR_B_IN1, LOW);  digitalWrite(MOTOR_B_IN2, HIGH); }
  else                     { digitalWrite(MOTOR_B_IN1, LOW);  digitalWrite(MOTOR_B_IN2, LOW); pwm = 0; }
  ledcWrite(MOTOR_B_EN, pwm);
}

void stopMotors() { setMotorA(0); setMotorB(0); }

void applyCmdVel(float linear, float angular) {
  float v_left  = (linear - angular * WHEEL_BASE / 2.0f) / MAX_SPEED_MS;
  float v_right = (linear + angular * WHEEL_BASE / 2.0f) / MAX_SPEED_MS;
  Serial.print("Motor A :");
  Serial.println(v_left);

  Serial.print("Motor B :");
  Serial.println(v_right);
  setMotorA(v_left);
  setMotorB(v_right);
}

// ═══════════════════════════════════════════════════════════
//  CMD_VEL
// ═══════════════════════════════════════════════════════════
void checkCmdVel() {
  int pktSize = udpCmdVel.parsePacket();
  if (pktSize <= 0) return;

  char buf[128];
  int  len = udpCmdVel.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;
  buf[len] = '\0';

  float lx = 0.0f, az = 0.0f;
  char* p;
  p = strstr(buf, "\"lx\":"); if (p) lx = atof(p + 5);
  p = strstr(buf, "\"az\":"); if (p) az = atof(p + 5);

  cmdLinear      = lx;
  cmdAngular     = az;
  lastCmdVelTime = millis();
  applyCmdVel(cmdLinear, cmdAngular);
}

// ═══════════════════════════════════════════════════════════
//  SENSOR PUBLISH
// ═══════════════════════════════════════════════════════════
void sendSensorData() {
  readMPU6050();
  noInterrupts();
  long ticksA = encTicksA;
  long ticksB = encTicksB;
  interrupts();

  char buf[256];
  int n = snprintf(buf, sizeof(buf),
    "{\"enc_a\":%ld,\"enc_b\":%ld,"
    "\"gyro_z\":%.4f,"
    "\"accel_x\":%.4f,\"accel_y\":%.4f}",
    ticksA, ticksB, gyroZ, accelX, accelY);

  udpSensor.beginPacket(LAPTOP_IP, PORT_SENSOR);
  udpSensor.write((uint8_t*)buf, n);
  udpSensor.endPacket();
}

// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ESP32 Robot + RPLidar C1 ===");

  Wire.begin(21, 22);
  Wire.setClock(400000);

  Serial2.begin(RPLIDAR_BAUDRATE, SERIAL_8N1, RPLIDAR_RX_PIN, RPLIDAR_TX_PIN);
  Serial.printf("[LIDAR] UART2: RX=GPIO%d TX=GPIO%d baud=%d\n",
    RPLIDAR_RX_PIN, RPLIDAR_TX_PIN, RPLIDAR_BAUDRATE);

  rplidarSendCmd(RPLIDAR_CMD_STOP);
  delay(100);
  rplidarFlush();

  initMotors();
  Serial.println("[MOTOR] L298N siap");

  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), isrEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), isrEncoderB, RISING);

  initMPU6050();
  Serial.println("[IMU] MPU6050 siap");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Menghubungkan");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());

  udpLidar.begin(PORT_LIDAR);
  udpCmdVel.begin(PORT_CMDVEL);

  Serial.println("\n=== SISTEM SIAP ===");
  Serial.printf("PORT_LIDAR   %d  ← 'on'/'off'\n", PORT_LIDAR);
  Serial.printf("PORT_SCAN    %d  → scan 360°\n",   PORT_SCAN);
  Serial.printf("PORT_CMDVEL  %d  ← cmd_vel\n",     PORT_CMDVEL);
  Serial.printf("PORT_SENSOR  %d  → IMU+encoder\n", PORT_SENSOR);
}

// ═══════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    stopMotors();
    if (lidarRunning) { rplidarStopScan(); rplidarMotorOff(); lidarRunning = false; }
    Serial.println("[WiFi] Terputus! Reconnect...");
    WiFi.disconnect();
    WiFi.reconnect();
    delay(2000);
    return;
  }

  checkLidarControl();

  if (lidarRunning && lidarScanning) {
    if (rplidarReadPackets()) sendScanUDP();
  }

  checkCmdVel();

  if (millis() - lastCmdVelTime > CMDVEL_TIMEOUT_MS) {
    stopMotors();
  }

  if (millis() - lastSensorSend >= SENSOR_INTERVAL_MS) {
    sendSensorData();
    lastSensorSend = millis();
  }
}