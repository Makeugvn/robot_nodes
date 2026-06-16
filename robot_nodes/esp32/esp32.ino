/*
 * esp32_rplidar_l298n.ino
 * ========================================================================
 * Versi merge: L298N library + PWM via UDP + odometri + IMU + magnetometer
 *
 * Perubahan dari versi sebelumnya:
 *   - Motor dikendalikan via library L298N (bukan manual ledcWrite)
 *   - Input PWM dari UDP cmd_vel: {"lx":0.5,"az":0.0,"pwm":180}
 *   - PWM default 200 jika field "pwm" tidak ada di paket
 *   - Semua fitur lain tetap: RPLidar C1, MPU6050, HMC5883L, encoder, odom
 *
 * Wiring:
 *   RPLidar TX   → GPIO16 (RX2)
 *   RPLidar RX   → GPIO17 (TX2)
 *   MPU6050 SDA  → GPIO21
 *   MPU6050 SCL  → GPIO22
 *   HMC5883L SDA → GPIO21 (shared I2C bus)
 *   HMC5883L SCL → GPIO22 (shared I2C bus)
 *   Encoder A    → GPIO34 (input only)
 *   Encoder B    → GPIO35 (input only)
 *   Motor A EN   → GPIO27
 *   Motor A IN1  → GPIO25
 *   Motor A IN2  → GPIO26
 *   Motor B EN   → GPIO14
 *   Motor B IN1  → GPIO32
 *   Motor B IN2  → GPIO33
 *
 * Dependensi library:
 *   - L298N by Andrea Lombardo (install via Arduino Library Manager)
 */

#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <L298N.h>
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

// ═══════════════════════════════════════════════════════════
//  KONFIGURASI JARINGAN
// ═══════════════════════════════════════════════════════════
const char* WIFI_SSID = "SENDANG REJEKI";
const char* WIFI_PASS = "HELLY123";
const char* LAPTOP_IP = "192.168.100.161";
IPAddress targetIP(192, 168, 100, 161);

#define PORT_LIDAR   5005
#define PORT_SCAN    5006
#define PORT_CMDVEL  5007
#define PORT_SENSOR  5050

// ═══════════════════════════════════════════════════════════
//  RPLIDAR C1
// ═══════════════════════════════════════════════════════════
#define RPLIDAR_RX_PIN      16
#define RPLIDAR_TX_PIN      17
#define RPLIDAR_BAUDRATE    460800
#define RPLIDAR_CMD_SYNC    0xA5
#define RPLIDAR_CMD_SCAN    0x20
#define RPLIDAR_CMD_STOP    0x25
#define RPLIDAR_CMD_RESET   0x40
#define RPLIDAR_CMD_HEALTH  0x52
#define RPLIDAR_SYNC1       0xA5
#define RPLIDAR_SYNC2       0x5A
#define RPLIDAR_PKT_SIZE    5
#define HALF_SCAN           180

// ═══════════════════════════════════════════════════════════
//  MOTOR L298N (via library)
// ═══════════════════════════════════════════════════════════
#define MOTOR_A_IN1  4
#define MOTOR_A_IN2  5
#define MOTOR_A_EN   27
#define MOTOR_B_IN1  32
#define MOTOR_B_IN2  33
#define MOTOR_B_EN   14

// Instance library L298N
L298N motorA(MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2);
L298N motorB(MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2);

// Parameter robot
#define MAX_SPEED_MS    0.5f   // m/s maksimum
#define WHEEL_BASE      0.15f  // meter, jarak antar roda

// PWM default dan batas
#define PWM_DEFAULT     200    // 0–255
#define PWM_MIN         60     // batas bawah agar motor bergerak
#define PWM_MAX         255

// Tambah variabel timing terpisah
unsigned long lastScanSend   = 0;
// unsigned long lastSensorSend = 0;
#define SCAN_MIN_INTERVAL_MS    120   // minimal jarak antar scan UDP
#define SENSOR_INTERVAL_MS       500

// PWM aktif saat ini (bisa diubah via UDP)
uint8_t currentPwm = PWM_DEFAULT;

/*
// ═══════════════════════════════════════════════════════════
//  ENCODER
// ═══════════════════════════════════════════════════════════
#define ENCODER_A_PIN  34
#define ENCODER_B_PIN  35

volatile long encTicksA = 0;
volatile long encTicksB = 0;
long prevTicksA = 0;
long prevTicksB = 0;

#define WHEEL_RADIUS_M    0.033f
#define ENCODER_TPR       20.0f
#define METER_PER_TICK    (2.0f * M_PI * WHEEL_RADIUS_M / ENCODER_TPR)
*/

// ═══════════════════════════════════════════════════════════
//  MPU6050
// ═══════════════════════════════════════════════════════════
#define MPU6050_ADDR   0x68
#define MPU_PWR_MGMT   0x6B
#define MPU_ACCEL_CFG  0x1C
#define MPU_GYRO_CFG   0x1B
#define MPU_ACCEL_OUT  0x3B
#define MPU_GYRO_OUT   0x43

float imuAccX = 0, imuAccY = 0, imuAccZ = 0;
float imuGyrX = 0, imuGyrY = 0, imuGyrZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
#define CALIB_SAMPLES  200

float yawAngle = 0.0f;
unsigned long lastImuTime = 0;

float fusedYaw   = 0.0f;
float fusedRoll  = 0.0f;
float fusedPitch = 0.0f;
#define COMP_FILTER_ALPHA   0.95f
#define MAG_DECLINATION_RAD 0.021f
bool fusionInitialized = false;

// ═══════════════════════════════════════════════════════════
//  HMC5883L
// ═══════════════════════════════════════════════════════════
#define HMC5883L_ADDR     0x2C
#define HMC_REG_CONFIG_A  0x00
#define HMC_REG_CONFIG_B  0x01
#define HMC_REG_MODE      0x02
#define HMC_REG_DATA_X_H  0x03

float magX = 0, magY = 0, magZ = 0;
float magOffsetX = 0.0f, magOffsetY = 0.0f, magOffsetZ = 0.0f;
float magHeading = 0.0f;

/*
// ═══════════════════════════════════════════════════════════
//  ODOMETRI
// ═══════════════════════════════════════════════════════════
float odomX     = 0.0f;
float odomY     = 0.0f;
float odomTheta = 0.0f;
float odomVx    = 0.0f;
float odomWz    = 0.0f;
#define ODOM_INTERVAL_MS  50
*/

// ═══════════════════════════════════════════════════════════
//  UDP + TIMING
// ═══════════════════════════════════════════════════════════
WiFiUDP udpLidar, udpScan, udpCmdVel, udpSensor;

unsigned long lastSensorSend = 0;
unsigned long lastCmdVelTime = 0;
#define SENSOR_INTERVAL_MS  50
#define CMDVEL_TIMEOUT_MS   500

float cmdLinear  = 0.0f;
float cmdAngular = 0.0f;

// RPLidar
bool     lidarRunning  = false;
bool     lidarScanning = false;
uint32_t scanSeq       = 0;
uint16_t scanBuffer[360];
bool     scanValid[360];
int      sampleCount = 0;
char     jsonBuf[1600];


/*
// ═══════════════════════════════════════════════════════════
//  ISR ENCODER
// ═══════════════════════════════════════════════════════════
void IRAM_ATTR isrEncoderA() {
  if      (digitalRead(MOTOR_A_IN1) == HIGH && digitalRead(MOTOR_A_IN2) == LOW)  encTicksA++;
  else if (digitalRead(MOTOR_A_IN1) == LOW  && digitalRead(MOTOR_A_IN2) == HIGH) encTicksA--;
}

void IRAM_ATTR isrEncoderB() {
  if      (digitalRead(MOTOR_B_IN1) == HIGH && digitalRead(MOTOR_B_IN2) == LOW)  encTicksB++;
  else if (digitalRead(MOTOR_B_IN1) == LOW  && digitalRead(MOTOR_B_IN2) == HIGH) encTicksB--;
}
*/


// ═══════════════════════════════════════════════════════════
//  MOTOR — via L298N library
//  speed: -1.0 .. +1.0
//  PWM dihitung dari |speed| × currentPwm, di-clamp ke PWM_MIN..PWM_MAX
// ═══════════════════════════════════════════════════════════
void setMotorA(float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  if (fabsf(speed) <= 0.02f) {
    motorA.stop();
    return;
  }
  // Hitung PWM proporsional: skala speed terhadap currentPwm
  // Misal speed=1.0 → PWM=currentPwm, speed=0.5 → PWM=currentPwm/2 (tapi min PWM_MIN)
  uint8_t pwm = (uint8_t)constrain(
    (int)(fabsf(speed) * currentPwm), PWM_MIN, PWM_MAX);
  motorA.setSpeed(pwm);
  if (speed > 0) motorA.forward();
  else           motorA.backward();

  // Serial.printf("[MTR-A] speed=%.2f pwm=%d dir=%s\n",
  //   speed, pwm, speed > 0 ? "FWD" : "BWD");
}

void setMotorB(float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  if (fabsf(speed) <= 0.02f) {
    motorB.stop();
    return;
  }
  uint8_t pwm = (uint8_t)constrain(
    (int)(fabsf(speed) * currentPwm), PWM_MIN, PWM_MAX);
  motorB.setSpeed(pwm);
  if (speed > 0) motorB.forward();
  else           motorB.backward();

  // Serial.printf("[MTR-B] speed=%.2f pwm=%d dir=%s\n",
  //   speed, pwm, speed > 0 ? "FWD" : "BWD");
}

void stopMotors() {
  motorA.stop();
  motorB.stop();
}

/*
 * applyCmdVel — differential drive
 * linear  : m/s (+maju, -mundur)
 * angular : rad/s (+kiri, -kanan)
 *
 * Jika salah satu roda berputar terbalik saat maju,
 * balik tanda v_left atau v_right di sini:
 *   v_left  = -v_left;   // balik Motor A
 *   v_right = -v_right;  // balik Motor B
 */
void applyCmdVel(float linear, float angular) {
  float v_left  = (linear - angular * WHEEL_BASE / 2.0f) / MAX_SPEED_MS;
  float v_right = (linear + angular * WHEEL_BASE / 2.0f) / MAX_SPEED_MS;

  // Serial.printf("[CMD] lin=%.3f ang=%.3f → vL=%.3f vR=%.3f pwm=%d\n",
  //   linear, angular, v_left, v_right, currentPwm);

  setMotorA(v_left);
  setMotorB(v_right);
}

// ═══════════════════════════════════════════════════════════
//  MPU6050
// ═══════════════════════════════════════════════════════════
bool initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_PWR_MGMT);
  Wire.write(0x00);
  if (Wire.endTransmission(true) != 0) {
    Serial.println("[IMU] MPU6050 tidak ditemukan!");
    return false;
  }
  delay(100);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_ACCEL_CFG);
  Wire.write(0x00);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_GYRO_CFG);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(50);
  Serial.println("[IMU] MPU6050 OK");
  return true;
}

void readMPU6050Raw(int16_t& ax, int16_t& ay, int16_t& az,
                    int16_t& gx, int16_t& gy, int16_t& gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_ACCEL_OUT);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() < 14) return;
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // temp
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

void calibrateGyro() {
  Serial.print("[IMU] Kalibrasi gyro (robot harus diam)");
  double sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readMPU6050Raw(ax, ay, az, gx, gy, gz);
    sumX += gx; sumY += gy; sumZ += gz;
    delay(5);
    if (i % 50 == 0) Serial.print(".");
  }
  gyroOffsetX = sumX / CALIB_SAMPLES;
  gyroOffsetY = sumY / CALIB_SAMPLES;
  gyroOffsetZ = sumZ / CALIB_SAMPLES;
  Serial.printf("\n[IMU] Gyro offset: X=%.1f Y=%.1f Z=%.1f\n",
    gyroOffsetX, gyroOffsetY, gyroOffsetZ);
}

// ═══════════════════════════════════════════════════════════
//  HMC5883L
// ═══════════════════════════════════════════════════════════
bool initHMC5883L() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(HMC_REG_CONFIG_A);
  Wire.write(0x70);
  if (Wire.endTransmission(true) != 0) {
    Serial.println("[MAG] HMC5883L tidak ditemukan!");
    return false;
  }
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(HMC_REG_CONFIG_B);
  Wire.write(0x20);
  Wire.endTransmission(true);
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(HMC_REG_MODE);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);
  Serial.println("[MAG] HMC5883L OK");
  return true;
}

void readHMC5883L() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(HMC_REG_DATA_X_H);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)HMC5883L_ADDR, (uint8_t)6, (uint8_t)true);
  if (Wire.available() < 6) return;
  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  magX = (rawX - magOffsetX) / 1090.0f;
  magY = (rawY - magOffsetY) / 1090.0f;
  magZ = (rawZ - magOffsetZ) / 1090.0f;
  magHeading = atan2(magY, magX);
  if (magHeading < 0) magHeading += 2.0f * M_PI;
}

// ═══════════════════════════════════════════════════════════
//  SENSOR FUSION
// ═══════════════════════════════════════════════════════════
void updateSensorFusion() {
  unsigned long now = millis();
  float dt = (lastImuTime > 0) ? (now - lastImuTime) / 1000.0f : 0.01f;
  lastImuTime = now;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;

  int16_t ax, ay, az, gx, gy, gz;
  readMPU6050Raw(ax, ay, az, gx, gy, gz);

  imuAccX = ax / 16384.0f * 9.81f;
  imuAccY = ay / 16384.0f * 9.81f;
  imuAccZ = az / 16384.0f * 9.81f;
  imuGyrX = (gx - gyroOffsetX) / 131.0f * (M_PI / 180.0f);
  imuGyrY = (gy - gyroOffsetY) / 131.0f * (M_PI / 180.0f);
  imuGyrZ = (gz - gyroOffsetZ) / 131.0f * (M_PI / 180.0f);

  float accRoll  = atan2f(imuAccY, imuAccZ);
  float accPitch = atan2f(-imuAccX, sqrtf(imuAccY*imuAccY + imuAccZ*imuAccZ));

  readHMC5883L();

  float cosRoll  = cosf(fusedRoll);
  float sinRoll  = sinf(fusedRoll);
  float cosPitch = cosf(fusedPitch);
  float sinPitch = sinf(fusedPitch);

  float magXcomp = magX * cosPitch + magY * sinRoll * sinPitch + magZ * cosRoll * sinPitch;
  float magYcomp = magY * cosRoll  - magZ * sinRoll;

  float magYaw = atan2f(-magYcomp, magXcomp) + MAG_DECLINATION_RAD;
  if (magYaw < 0)            magYaw += 2.0f * M_PI;
  if (magYaw > 2.0f * M_PI)  magYaw -= 2.0f * M_PI;
  magHeading = magYaw;

  if (!fusionInitialized) {
    fusedYaw   = magYaw;
    fusedRoll  = accRoll;
    fusedPitch = accPitch;
    fusionInitialized = true;
    Serial.printf("[FUSION] Init: yaw=%.2f°\n", fusedYaw * 180.0f / M_PI);
    return;
  }

  fusedRoll  = COMP_FILTER_ALPHA * (fusedRoll  + imuGyrX * dt) + (1.0f - COMP_FILTER_ALPHA) * accRoll;
  fusedPitch = COMP_FILTER_ALPHA * (fusedPitch + imuGyrY * dt) + (1.0f - COMP_FILTER_ALPHA) * accPitch;

  float gyroYaw = fusedYaw + imuGyrZ * dt;
  float diff    = magYaw - gyroYaw;
  while (diff >  M_PI) diff -= 2.0f * M_PI;
  while (diff < -M_PI) diff += 2.0f * M_PI;
  fusedYaw = gyroYaw + (1.0f - COMP_FILTER_ALPHA) * diff;
  while (fusedYaw < 0)           fusedYaw += 2.0f * M_PI;
  while (fusedYaw > 2.0f * M_PI) fusedYaw -= 2.0f * M_PI;

  yawAngle = fusedYaw;
}


/*
// ═══════════════════════════════════════════════════════════
//  ODOMETRI
// ═══════════════════════════════════════════════════════════
void updateOdometry() {
  noInterrupts();
  long ticksA = encTicksA;
  long ticksB = encTicksB;
  interrupts();

  long dTicksA = ticksA - prevTicksA;
  long dTicksB = ticksB - prevTicksB;
  prevTicksA = ticksA;
  prevTicksB = ticksB;

  float dLeft   = dTicksA * METER_PER_TICK;
  float dRight  = dTicksB * METER_PER_TICK;
  float dCenter = (dLeft + dRight) / 2.0f;
  float dTheta  = (dRight - dLeft) / WHEEL_BASE;

  odomTheta += dTheta;
  odomX += dCenter * cosf(odomTheta - dTheta / 2.0f);
  odomY += dCenter * sinf(odomTheta - dTheta / 2.0f);
  while (odomTheta >  M_PI) odomTheta -= 2.0f * M_PI;
  while (odomTheta < -M_PI) odomTheta += 2.0f * M_PI;

  // Serial.printf("\n[ODOM] X=%.3f Y=%.3f \n",
  //   odomX, odomY);

  float dt = ODOM_INTERVAL_MS / 1000.0f;
  odomVx = dCenter / dt;
  odomWz = dTheta  / dt;

  // Serial.printf("\n[ODOM] Vx=%.3f Wz=%.3f \n",
  //   odomVx, odomWz);
}
  */

// ═══════════════════════════════════════════════════════════
//  KIRIM SENSOR DATA
// ═══════════════════════════════════════════════════════════
void sendSensorData() {
  updateSensorFusion();

  /*
updateOdometry();

  noInterrupts();
  long ticksA = encTicksA;
  long ticksB = encTicksB;
  interrupts();
*/

  char buf[512];
  int n = snprintf(buf, sizeof(buf),
    "{"
    "\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
    "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,"
    "\"mx\":%.4f,\"my\":%.4f,\"mz\":%.4f,"
    "\"heading\":%.4f,"
    "\"roll\":%.4f,\"pitch\":%.4f,\"yaw\":%.4f,"
    "\"pwm\":%d"
    "}",
    imuAccX, imuAccY, imuAccZ,
    imuGyrX, imuGyrY, imuGyrZ,
    magX, magY, magZ,
    magHeading,
    fusedRoll, fusedPitch, fusedYaw,
    currentPwm
  );

  if (n >= sizeof(buf)) {
      n = sizeof(buf) - 1; 
  }

  // IPAddress targetIP;
  // targetIP.fromString(LAPTOP_IP);

  udpSensor.beginPacket(targetIP, PORT_SENSOR);
  udpSensor.write((uint8_t*)buf, n);
  int r = udpSensor.endPacket();
  Serial.printf("[SENSOR] %d bytes → %s\n", n, r ? "OK" : "FAIL");
}

// ═══════════════════════════════════════════════════════════
//  CMD_VEL — format UDP diperluas:
//  {"lx":0.5,"az":0.0}          → pakai PWM saat ini
//  {"lx":0.5,"az":0.0,"pwm":180} → update PWM dan gerak
//  {"pwm":200}                   → hanya update PWM, tidak gerak
// ═══════════════════════════════════════════════════════════
void checkCmdVel() {
  int pktSize = udpCmdVel.parsePacket();
  if (pktSize <= 0) return;
  char buf[128];
  int  len = udpCmdVel.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;
  buf[len] = '\0';

  float lx = 0.0f, az = 0.0f;
  int   pwm = -1;   // -1 = tidak ada field pwm di paket
  char* p;

  p = strstr(buf, "\"lx\":");  if (p) lx  = atof(p + 5);
  p = strstr(buf, "\"az\":");  if (p) az  = atof(p + 5);
  p = strstr(buf, "\"pwm\":"); if (p) pwm = atoi(p + 6);

  // Update PWM jika field ada dan valid
  if (pwm >= PWM_MIN && pwm <= PWM_MAX) {
    currentPwm = (uint8_t)pwm;
    Serial.printf("[CMD] PWM diperbarui → %d\n", currentPwm);
  }

  // Hanya gerak jika ada field lx atau az
  if (strstr(buf, "\"lx\":") || strstr(buf, "\"az\":")) {
    cmdLinear      = lx;
    cmdAngular     = az;
    lastCmdVelTime = millis();
    applyCmdVel(cmdLinear, cmdAngular);
  }
}

// ═══════════════════════════════════════════════════════════
//  RPLIDAR
// ═══════════════════════════════════════════════════════════
void rplidarSendCmd(uint8_t cmd) {
  Serial2.write(RPLIDAR_CMD_SYNC);
  Serial2.write(cmd);
}

void rplidarFlush() {
  delay(10);
  while (Serial2.available()) Serial2.read();
}

bool rplidarWaitDescriptor(uint32_t timeoutMs = 3000) {
  uint32_t start = millis();
  uint8_t  buf[7];
  int      idx = 0;
  while (millis() - start < timeoutMs) {
    if (!Serial2.available()) continue;
    uint8_t b = Serial2.read();
    if (idx == 0) { if (b == RPLIDAR_SYNC1) buf[idx++] = b; continue; }
    if (idx == 1) { if (b == RPLIDAR_SYNC2) buf[idx++] = b; else idx = 0; continue; }
    buf[idx++] = b;
    if (idx >= 7) {
      Serial.print("[LIDAR] Descriptor:");
      for (int i = 0; i < 7; i++) Serial.printf(" 0x%02X", buf[i]);
      Serial.println();
      return true;
    }
  }
  return false;
}

bool rplidarStartScan() {
  Serial.println("[LIDAR] Reset...");
  rplidarSendCmd(RPLIDAR_CMD_STOP); delay(200); rplidarFlush();
  rplidarSendCmd(RPLIDAR_CMD_RESET);
  Serial.print("[LIDAR] Boot");
  uint32_t lastByte = millis(), boot = millis();
  while (millis() - boot < 5000) {
    if (Serial2.available()) { Serial2.read(); lastByte = millis(); }
    else if (millis() - lastByte > 500) break;
    if (millis() % 500 < 10) Serial.print(".");
  }
  Serial.println(" OK");
  rplidarFlush();

  rplidarSendCmd(RPLIDAR_CMD_HEALTH);
  uint8_t hBuf[10]; int hIdx = 0; bool f = false;
  uint32_t hStart = millis();
  while (millis() - hStart < 2000 && hIdx < 10) {
    if (Serial2.available()) {
      uint8_t b = Serial2.read();
      if (!f && b == 0xA5) f = true;
      if (f) hBuf[hIdx++] = b;
    }
  }
  if (hIdx >= 10) {
    Serial.printf("[LIDAR] Health: %s\n",
      hBuf[7] == 0 ? "GOOD" : hBuf[7] == 1 ? "WARN" : "ERROR");
    if (hBuf[7] == 2) return false;
  }
  rplidarFlush();

  rplidarSendCmd(RPLIDAR_CMD_SCAN);
  if (!rplidarWaitDescriptor(5000)) return false;

  for (int i = 0; i < 360; i++) { scanBuffer[i] = 0; scanValid[i] = false; }
  sampleCount = 0; lidarScanning = true;
  Serial.println("[LIDAR] Scan ✓");
  return true;
}

void rplidarStopScan() {
  rplidarSendCmd(RPLIDAR_CMD_STOP); delay(10);
  rplidarFlush(); lidarScanning = false;
}

void rplidarMotorOff() {
  rplidarSendCmd(RPLIDAR_CMD_STOP); delay(100);
  rplidarSendCmd(RPLIDAR_CMD_RESET); delay(500);
  rplidarFlush();
}

bool rplidarParsePkt(uint8_t* p, float& ang, float& dist, bool& newScan, bool& valid) {
  uint8_t q = p[0] >> 2;
  bool s = p[0] & 0x01, c = p[1] & 0x01;
  if (!c) return false;
  uint16_t aq = ((uint16_t)p[2] << 7) | (p[1] >> 1);
  uint32_t dq = ((uint32_t)p[4] << 8) | p[3];
  ang = aq / 64.0f; dist = dq / 4.0f;
  newScan = s; valid = (q > 0 && dist >= 10.0f);
  return true;
}

bool rplidarReadPackets() {
  static uint8_t pkt[RPLIDAR_PKT_SIZE];
  static int idx = 0; static bool synced = false;
  bool full = false;
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (!synced) {
      if ((b & 0x03) == 0x01) { pkt[0] = b; idx = 1; synced = true; }
      continue;
    }
    pkt[idx++] = b;
    if (idx < RPLIDAR_PKT_SIZE) continue;
    idx = 0;
    float ang, dist; bool newScan, valid;
    if (!rplidarParsePkt(pkt, ang, dist, newScan, valid)) { synced = false; continue; }
    if (newScan) { full = true; scanSeq++; }
    if (valid) {
      int i = (int)ang % 360;
      if (i >= 0 && i < 360 && (!scanValid[i] || dist < scanBuffer[i])) {
        scanBuffer[i] = (uint16_t)dist; scanValid[i] = true; sampleCount++;
      }
    }
  }
  return full;
}

void sendScanUDP() {
  IPAddress ip; ip.fromString(LAPTOP_IP);
  int o = 0;
  o += snprintf(jsonBuf+o, sizeof(jsonBuf)-o,
    "{\"seq\":%lu,\"part\":1,\"start\":0,\"distances\":[", (unsigned long)scanSeq);
  for (int i = 0; i < HALF_SCAN; i++) {
    o += snprintf(jsonBuf+o, sizeof(jsonBuf)-o, "%d", scanValid[i]?(int)scanBuffer[i]:-1);
    if (i < HALF_SCAN-1) jsonBuf[o++] = ',';
  }
  o += snprintf(jsonBuf+o, sizeof(jsonBuf)-o, "]}");
  udpScan.beginPacket(ip, PORT_SCAN); udpScan.write((uint8_t*)jsonBuf, o);
  int r1 = udpScan.endPacket(); delay(15);

  o = 0;
  o += snprintf(jsonBuf+o, sizeof(jsonBuf)-o,
    "{\"seq\":%lu,\"part\":2,\"start\":180,\"distances\":[", (unsigned long)scanSeq);
  for (int i = HALF_SCAN; i < 360; i++) {
    o += snprintf(jsonBuf+o, sizeof(jsonBuf)-o, "%d", scanValid[i]?(int)scanBuffer[i]:-1);
    if (i < 359) jsonBuf[o++] = ',';
  }
  o += snprintf(jsonBuf+o, sizeof(jsonBuf)-o, "]}");
  udpScan.beginPacket(ip, PORT_SCAN); udpScan.write((uint8_t*)jsonBuf, o);
  int r2 = udpScan.endPacket();

  Serial.printf("[SCAN] Seq=%lu n=%d p1=%s p2=%s\n",
    (unsigned long)scanSeq, sampleCount, r1?"OK":"FAIL", r2?"OK":"FAIL");
  for (int i = 0; i < 360; i++) { scanBuffer[i] = 0; scanValid[i] = false; }
  sampleCount = 0;

  lastScanSend = millis();
}

void checkLidarControl() {
  int sz = udpLidar.parsePacket(); if (sz <= 0) return;
  char buf[32]; int len = udpLidar.read(buf, 31); if (len <= 0) return;
  buf[len] = '\0';
  String cmd = String(buf); cmd.trim(); cmd.toLowerCase();
  if (cmd == "on") {
    if (!lidarRunning) { if (rplidarStartScan()) lidarRunning = true; }
  } else if (cmd == "off") {
    if (lidarRunning) { rplidarStopScan(); rplidarMotorOff(); lidarRunning = false; }
  } else if (cmd == "status") {
    Serial.printf("[STATUS] lidar=%s pwm=%d\n", lidarRunning?"ON":"OFF", currentPwm);
  }
}

// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ESP32 Robot + RPLidar C1 + IMU + MAG (L298N lib) ===");

  Wire.begin(21, 22);
  Wire.setClock(400000);

  bool imuOk = initMPU6050();
  bool magOk = initHMC5883L();
  if (imuOk) calibrateGyro();

  Serial2.begin(RPLIDAR_BAUDRATE, SERIAL_8N1, RPLIDAR_RX_PIN, RPLIDAR_TX_PIN);
  rplidarSendCmd(RPLIDAR_CMD_STOP); delay(100); rplidarFlush();
  Serial.println("[LIDAR] Siap (OFF)");

  // Inisialisasi motor dengan PWM default
  motorA.setSpeed(PWM_DEFAULT);
  motorB.setSpeed(PWM_DEFAULT);
  motorA.stop();
  motorB.stop();
  Serial.printf("[MOTOR] L298N siap | PWM default=%d\n", PWM_DEFAULT);

/*
// Block Encoder
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), isrEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), isrEncoderB, RISING);
  Serial.println("[ENC] Encoder siap");
*/


  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Menghubungkan");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());

  Serial.println(udpLidar.begin(PORT_LIDAR)   ? "[UDP] Lidar OK"  : "[UDP] Lidar FAIL");
  Serial.println(udpCmdVel.begin(PORT_CMDVEL) ? "[UDP] CmdVel OK" : "[UDP] CmdVel FAIL");
  Serial.println(udpSensor.begin(PORT_SENSOR) ? "[UDP] Sensor OK" : "[UDP] Sensor FAIL");
  Serial.println(udpScan.begin(PORT_SCAN)     ? "[UDP] Scan OK"   : "[UDP] Scan FAIL");

  lastImuTime  = millis();

  Serial.println("\n=== SIAP ===");
  Serial.printf("IMU: %s | MAG: %s\n", imuOk?"OK":"FAIL", magOk?"OK":"FAIL");
  Serial.println("Format cmd_vel UDP (port 5007):");
  Serial.println("  {\"lx\":0.3,\"az\":0.0}           → gerak, pwm tetap");
  Serial.println("  {\"lx\":0.3,\"az\":0.0,\"pwm\":180} → gerak + set pwm");
  Serial.println("  {\"pwm\":200}                     → hanya set pwm");
  Serial.println("Catatan: sesuaikan ENCODER_TPR dan WHEEL_RADIUS_M dengan hardware!");
}

// ═══════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    stopMotors();
    if (lidarRunning) { rplidarStopScan(); rplidarMotorOff(); lidarRunning = false; }
    WiFi.disconnect(); WiFi.reconnect(); delay(2000); return;
  }

  checkLidarControl();

  if (lidarRunning && lidarScanning) {
    if (rplidarReadPackets()) sendScanUDP();
  }

  checkCmdVel();
  if (millis() - lastCmdVelTime > CMDVEL_TIMEOUT_MS) stopMotors();

  // Kirim sensor HANYA kalau tidak sedang kirim scan
  // Cek dengan lastScanSend — beri jeda 20ms setelah scan
  unsigned long now = millis();
  bool scanRecentlySent = (now - lastScanSend < 20);

  if (!scanRecentlySent && now - lastSensorSend >= SENSOR_INTERVAL_MS) {
    sendSensorData();
    lastSensorSend = now;
  }
}