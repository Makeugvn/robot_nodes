// --- Konfigurasi Pin dan Variabel ---
const int sensorPin = 27; // Pin sensor OUT dihubungkan ke GPIO 27 pada ESP32

// Variabel untuk menghitung pulsa (volatile karena diubah di dalam interrupt)
volatile unsigned long pulseCount = 0; 
// Waktu pulsa terakhir (untuk debounce, dalam mikrodetik)
volatile unsigned long lastPulseTime = 0;
// Debounce threshold dalam mikrodetik (1 ms default)
const unsigned long debounceMicros = 1000;

// --- Konfigurasi Fisik Roda dan Encoder ---
const float wheelDiameter = 6; // Diameter roda dalam centimeter
const int slots = 20;            // Jumlah lubang pada piringan encoder

// --- Variabel Kalibrasi ---
// Nilai default adalah 1.0 (100% sesuai teori).
// Jika jarak di Serial Monitor 100 cm, tapi jarak aslinya 105 cm, ubah menjadi 1.05
// Jika jarak di Serial Monitor 100 cm, tapi jarak aslinya 95 cm, ubah menjadi 0.95
float calibrationFactor = 1.0; 

float distance = 0.0;            // Variabel penyimpan total jarak

// --- Fungsi Interrupt Service Routine (ISR) ---
// IRAM_ATTR digunakan pada ESP32 agar fungsi ini disimpan di RAM. 
// Ini membuat eksekusi interrupt jauh lebih cepat dan mencegah error (crash).
void IRAM_ATTR countPulse() {
  unsigned long now = micros();
  // Jika waktu dari pulsa terakhir sudah melewati ambang debounce, terima pulsa
  if ((now - lastPulseTime) >= debounceMicros) {
    pulseCount++;
    lastPulseTime = now;
  }
}

void setup() {
  // Baud rate standar ESP32 biasanya 115200
  Serial.begin(115200); 
  
  // Menggunakan INPUT_PULLUP bisa membantu menstabilkan pembacaan sinyal
  pinMode(sensorPin, INPUT_PULLUP);
  
  // Mengaktifkan interrupt pada GPIO 27
  attachInterrupt(digitalPinToInterrupt(sensorPin), countPulse, RISING);
  
  Serial.println("Program Pengukur Jarak ESP32 Dimulai...");
}

void loop() {
  // 1. Matikan interrupt sementara untuk membaca pulseCount dengan aman
  noInterrupts();
  unsigned long currentPulses = pulseCount/2;
  interrupts(); // Hidupkan kembali interrupt

  // 2. Kalkulasi Jarak Teoritis
  float circumference = 3.14159 * wheelDiameter;   // Menghitung keliling roda (π * d)
  float distancePerPulse = circumference / slots;  // Jarak tempuh per 1 pulsa/lubang
  
  // 3. Hitung Jarak Aktual dengan Faktor Kalibrasi
  distance = (currentPulses * distancePerPulse) * calibrationFactor;

  // 4. Tampilkan di Serial Monitor
  Serial.print("Total Pulsa: ");
  Serial.print(currentPulses);
  Serial.print("  |  Jarak Tempuh: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Jeda 500ms (setengah detik)
  delay(500); 
}