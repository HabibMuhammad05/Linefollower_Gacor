/*
  LINE FOLLOWER - PD CONTROLLER + FREE STATE MACHINE + FULL SERIAL LOG
  =============================================================================
  Berdasarkan pin dari tesMotor.ino, tesSensor.ino, tesServo.ino:

  SENSOR (analog, index 0=kiri ... 7=kanan):
    A0 A1 A2 A3 A4 A5 A6 A7

  TOMBOL (INPUT_PULLUP, aktif LOW) - BEBAS DITEKAN KAPAN AJA, LANGSUNG RESPON:
    D2  -> CALIBRATE  (mulai / ulang kalibrasi, dari state manapun)
    D4  -> START      (mulai / lanjut jalan, dari state manapun)
    D12 -> STOP       (berhenti, dari state manapun)

  MOTOR (dari tesMotor.ino, kiri-kanan SUDAH DIBALIK sesuai kondisi fisik robot):
    Motor KIRI  : IN3=3 (maju/PWM), IN4=5 (mundur/PWM)
    Motor KANAN : IN1=6 (maju/PWM), IN2=11 (mundur/PWM)

  INDIKATOR:
    WS2812 8 LED di D13, mode beda buat tiap state:
      IDLE       -> rainbow muter
      CALIBRATING-> dot putih scanning
      RUNNING    -> mirror posisi sensor (merah=kena garis)
      SIMPANG    -> kuning nyala penuh sesaat (200ms) pas persimpangan kedeteksi
      MAPPING    -> dot warna lari sesuai arah keputusan (lihat bagian MAPPING JALUR)
      DIANGKAT   -> putih blink cepat, motor mati sementara
      LINE LOST  -> blink merah cepat (dalam RUNNING)
      STOPPED    -> merah nyala terus (manual, tombol STOP), ga balik ke rainbow sendiri
      FINISH     -> hijau blink sesaat (rute mapping selesai), lalu auto balik IDLE

  DETEKSI PERSIMPANGAN:
    Simpel & general (ga bedain pertigaan/perempatan/silang): kalau sensor
    PALING KIRI (S0) dan PALING KANAN (S7) sama-sama kena hitam bersamaan
    DAN masih ada minimal 1 sensor tengah (S1-S6) yang putih, itu dianggap
    persimpangan (syarat "ada putih di tengah" ini yang bedain simpang asli
    dari kondisi robot diangkat / full black). Edge-triggered - cuma kehitung
    sekali per simpang, ada counter total & log di serial. Robot tetap jalan
    lurus ngikutin PD seperti biasa, belum ada maneuver belok otomatis.
    Threshold-nya (SENSOR_ON_THRESHOLD) satu konstanta yang sama dipakai LED
    indicator, cek simpang, dan cek diangkat - nilainya dari normalized[]
    (hasil kalibrasi 0-1000), bukan ADC mentah.

  MAPPING JALUR (BELOK/AKSI OTOMATIS DI SIMPANG):
    Keputusan robot di tiap simpang diatur sebagai SCRIPT di file
    "mapping.h" (fungsi executeMapStep(), 1 case per nomor simpang, bisa
    gabungin beberapa primitive: turnLeft/turnRight/turnAround/maju/
    mundur/stopSesaat/ambilKotak/lepasKotak/resetLineFollowing/
    selesaiRute). Kalau simpang yang dilewati belum di-case-in di
    mapping.h, robot otomatis LURUS seperti biasa (behavior lama, ga
    berubah sama sekali). stopSesaat() TETAP lanjut ke simpang
    berikutnya (bukan akhir rute) - beda dari selesaiRute() yang
    nandain SELURUH rute kelar (LED hijau blink / state FINISH, lalu
    OTOMATIS balik ke IDLE, siap run berikutnya tinggal pencet START).
    Servo ambil/simpan barang BELUM diintegrasikan (ambilKotak()/
    lepasKotak() masih placeholder kosong) - masih di tesServo.ino
    terpisah, menyusul di versi final.

    Rencana ke depan (BELUM diimplementasi, cuma catatan): mau ada
    beberapa PROFIL mapping (beberapa versi executeMapStep) yang bisa
    dipilih sebelum mulai jalan, masing-masing punya skenario sendiri.

  DETEKSI ROBOT DIANGKAT:
    Kalau SEMUA 8 sensor kebaca gelap barengan (ga ada permukaan buat
    mantulin cahaya, mirip kena hitam total), robot dianggap diangkat -
    motor langsung dimatikan otomatis biar roda ga muter sia-sia/berbahaya.
    Begitu diturunin lagi dan ada sensor yang balik putih, jalan normal lagi.

  CARA PAKAI (semua status keluar di Serial Monitor, baud 115200):
    1. Upload, buka Serial Monitor (115200).
    2. LED rainbow = IDLE, siap terima tombol apa aja.
    3. Tekan CALIBRATE (D2): geser robot manual ngelewatin garis hitam &
       lantai putih selama 5 detik biar tiap sensor kebaca rentangnya.
       Setelah selesai balik ke IDLE (rainbow).
    4. Taruh robot di garis, tekan START (D4) -> langsung RUNNING.
    5. Tekan STOP (D12) kapan aja -> motor berhenti, LED merah tetap nyala
       sampai lu tekan START atau CALIBRATE lagi.
    6. Kalau belum pernah kalibrasi & langsung START, sistem tetap jalan
       pakai rentang default 0-1023 (kurang presisi, tapi ga ngambek/crash).
    7. Kalibrasi TERSIMPAN OTOMATIS ke EEPROM. Reset/matiin Arduino ga bikin
       hilang - begitu boot, kalibrasi lama langsung kepakai lagi. Kalibrasi
       ulang cuma perlu kalau pindah arena/pencahayaan beda.

  Library yang perlu di-install lewat Library Manager:
    - Adafruit NeoPixel
  (EEPROM.h sudah built-in di Arduino core, tidak perlu install)
*/

#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
// mapping.h di-include di PALING BAWAH file ini (lihat baris paling akhir),
// bukan di sini - soalnya isinya (primitive turnLeft/mundur/dst) butuh
// strip/setMotor()/BASE_SPEED/dst yang baru didefinisikan di bawah sini.
// Prototype ini cuma buat ngasih tau compiler signature-nya duluan, biar
// runLineFollowing() di bawah bisa manggil executeMapStep() tanpa error.
void executeMapStep(unsigned int simpangKe, bool &actionDone);

// ================== PIN DEFINITION ==================
const uint8_t sensorPin[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

const uint8_t BTN_CALIB = 2;
const uint8_t BTN_START = 4;
const uint8_t BTN_STOP  = 12;

// kiri-kanan dibalik dari tesMotor.ino karena roda fisiknya kebalik
const uint8_t IN1 = 6;   // motor KANAN maju
const uint8_t IN2 = 11;  // motor KANAN mundur
const uint8_t IN3 = 3;   // motor KIRI maju
const uint8_t IN4 = 5;   // motor KIRI mundur

const uint8_t LED_PIN   = 13;
const uint8_t LED_COUNT = 8;

// ================== TUNING ==================
float KP = 0.8;
float KD = 0.4;

int BASE_SPEED = 120;     // kecepatan dasar saat lurus (0-255)
int MAX_SPEED  = 180;

int LINE_LOST_THRESHOLD = 400; // kalau semua sensor normalized-nya di bawah ini = line hilang
int PIVOT_SPEED = 110;         // kecepatan pivot saat nyari line

int SENSOR_ON_THRESHOLD = 500;         // >ini = dianggap "kena hitam" (dipakai LED, simpang, deteksi diangkat)
                                        // ini nilai normalized (0-1000) HASIL KALIBRASI, bukan ADC mentah
const unsigned long INTERSECTION_FLASH_MS = 200; // durasi LED kuning pas simpang kedeteksi

uint8_t LED_BRIGHTNESS = 20;   // 0-255, atur terang-redup semua LED sekaligus

const unsigned long DEBUG_PRINT_INTERVAL = 100;   // ms, biar serial ga banjir
const unsigned long CALIB_DURATION_MS   = 5000;   // 5 detik
const unsigned long FINISH_DURATION_MS  = 2000;   // durasi LED hijau di state FINISH sebelum auto balik IDLE

// ================== EEPROM (persist kalibrasi biar ga ilang tiap reset) ==================
const int   EEPROM_ADDR_MAGIC = 0;      // 1 byte penanda "sudah pernah kalibrasi"
const int   EEPROM_ADDR_DATA  = 1;      // calibMin[8] (16 byte) + calibMax[8] (16 byte)
const uint8_t EEPROM_MAGIC_VALUE = 0xA5;

// ================== STATE MACHINE ==================
// ST_FINISH = rute mapping selesai dilewati semua (bukan STOP manual dari
// tombol) - LED hijau blink sesaat, lalu OTOMATIS balik ke ST_IDLE lagi,
// state/counter di-reset kaya abis baru boot, siap buat run berikutnya.
enum RobotState { ST_IDLE, ST_CALIBRATING, ST_RUNNING, ST_STOPPED, ST_FINISH };
RobotState currentState = ST_IDLE;
unsigned long finishStateEnteredAt = 0;

// ================== VARIABEL GLOBAL ==================
int calibMin[8];
int calibMax[8];
int normalized[8]; // 0 (putih) - 1000 (hitam)

int lastError = 0;
int lastErrorSign = 1; // buat nentuin arah pivot pas line hilang

bool intersectionActive = false;        // lagi di atas persimpangan (edge-trigger guard)
unsigned long intersectionFlashUntil = 0;
unsigned int intersectionCount = 0;

bool robotLifted = false;               // edge-trigger guard buat log "robot diangkat"

unsigned long lastDebugPrint = 0;

bool lastReadCalib = HIGH;
bool lastReadStart = HIGH;
bool lastReadStop  = HIGH;
bool calibPressed;
bool startPressed;
bool stopPressed;

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("   LINE FOLLOWER - SETUP DIMULAI"));
  Serial.println(F("========================================"));

  for (uint8_t i = 0; i < 8; i++) pinMode(sensorPin[i], INPUT);
  Serial.println(F("[OK] Sensor A0-A7 diinisialisasi"));

  // default range aman kalau user belum sempat kalibrasi
  for (uint8_t i = 0; i < 8; i++) {
    calibMin[i] = 0;
    calibMax[i] = 1023;
  }

  if (loadCalibrationFromEEPROM()) {
    Serial.println(F("[OK] Kalibrasi lama ditemukan di EEPROM, langsung dipakai:"));
    for (uint8_t i = 0; i < 8; i++) {
      Serial.print(F("  S")); Serial.print(i);
      Serial.print(F(": ")); Serial.print(calibMin[i]);
      Serial.print(F(" - ")); Serial.println(calibMax[i]);
    }
  } else {
    Serial.println(F("[INFO] Belum ada kalibrasi tersimpan, pakai default 0-1023"));
  }

  pinMode(BTN_CALIB, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);
  Serial.println(F("[OK] Tombol D2(CALIBRATE) D4(START) D12(STOP) diinisialisasi"));

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotor();
  Serial.println(F("[OK] Motor IN1/IN2/IN3/IN4 diinisialisasi, motor OFF"));

  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.clear();
  strip.show();
  Serial.println(F("[OK] LED WS2812 (D13) diinisialisasi"));

  Serial.println(F("========================================"));
  Serial.println(F("SETUP SELESAI - IDLE, tombol bebas ditekan kapan aja:"));
  Serial.println(F("  D2 CALIBRATE | D4 START | D12 STOP"));
  Serial.println(F("========================================"));

  currentState = ST_IDLE;
}

// ================== LOOP (STATE MACHINE BEBAS) ==================
void loop() {
  calibPressed = checkButton(BTN_CALIB, lastReadCalib);
  startPressed  = checkButton(BTN_START, lastReadStart);
  stopPressed   = checkButton(BTN_STOP, lastReadStop);

  // tombol bisa ditekan dari state manapun, langsung dieksekusi
  if (calibPressed) {
    stopMotor();
    Serial.println(F("[BTN] CALIBRATE ditekan"));
    currentState = ST_CALIBRATING;
    doCalibration();              // blocking, ada log di dalam
    Serial.println(F("[STATE] -> IDLE (tekan START buat jalan)"));
    currentState = ST_IDLE;
    return;
  }
  if (startPressed && currentState != ST_RUNNING) {
    Serial.println(F("[BTN] START ditekan -> RUNNING"));
    lastError = 0;
    currentState = ST_RUNNING;
  }
  if (stopPressed && currentState != ST_STOPPED) {
    stopMotor();
    solidRed();
    Serial.println(F("[BTN] STOP ditekan -> motor berhenti"));
    currentState = ST_STOPPED;
  }

  switch (currentState) {
    case ST_IDLE:
      rainbowIdle();
      break;
    case ST_RUNNING:
      runLineFollowing();
      break;
    case ST_STOPPED:
      rainbowIdle();
      break;
    case ST_FINISH:
      indicateFinish();
      if (millis() - finishStateEnteredAt > FINISH_DURATION_MS) {
        Serial.println(F("[STATE] FINISH selesai -> balik ke IDLE, siap run berikutnya"));
        intersectionCount = 0;
        intersectionActive = false;
        currentState = ST_IDLE;
      }
      break;
    case ST_CALIBRATING:
      // ga akan sampai sini karena doCalibration() blocking di atas
      break;
  }
}

// ================== LINE FOLLOWING (dipanggil tiap loop saat RUNNING) ==================
void runLineFollowing() {
  readAndNormalize();

  // ---- cek robot diangkat: SEMUA sensor gelap barengan (ga ada permukaan buat mantul) ----
  bool allBlack = true;
  for (uint8_t i = 0; i < 8; i++) {
    if (normalized[i] <= SENSOR_ON_THRESHOLD) { allBlack = false; break; }
  }
  if (allBlack) {
    stopMotor();
    indicateLifted();
    if (!robotLifted) {
      robotLifted = true;
      Serial.println(F("[ANGKAT] Semua sensor gelap barengan - robot kemungkinan diangkat, motor dimatikan"));
    }
    return; // skip semua logic line-following/simpang selama diangkat
  } else if (robotLifted) {
    robotLifted = false;
    Serial.println(F("[ANGKAT] Robot balik ke track, lanjut jalan normal"));
  }

  long weightedSum = 0;
  long sumVal = 0;
  int maxVal = 0;
  for (uint8_t i = 0; i < 8; i++) {
    weightedSum += (long)normalized[i] * (i * 1000L);
    sumVal += normalized[i];
    if (normalized[i] > maxVal) maxVal = normalized[i];
  }

  bool lineDetected = (maxVal >= LINE_LOST_THRESHOLD);

  if (!lineDetected) {
    indicateLineLost();
    if (lastErrorSign >= 0) {
      setMotor(PIVOT_SPEED, -PIVOT_SPEED);   // pivot kanan
    } else {
      setMotor(-PIVOT_SPEED, PIVOT_SPEED);   // pivot kiri
    }
    if (millis() - lastDebugPrint > DEBUG_PRINT_INTERVAL) {
      lastDebugPrint = millis();
      Serial.print(F("[RUNNING] LINE HILANG (maxVal:"));
      Serial.print(maxVal);
      Serial.print(F(") - pivot ke "));
      Serial.println(lastErrorSign >= 0 ? F("KANAN") : F("KIRI"));
    }
    return;
  }

  int position = weightedSum / sumVal;   // 0 (paling kiri) .. 7000 (paling kanan)
  int error = position - 3500;           // 3500 = titik tengah 8 sensor

  lastErrorSign = (error >= 0) ? 1 : -1;

  int derivative = error - lastError;
  lastError = error;

  int correction = (int)(KP * error + KD * derivative);

  int leftSpeed  = constrain(BASE_SPEED + correction, 0, MAX_SPEED);
  int rightSpeed = constrain(BASE_SPEED - correction, 0, MAX_SPEED);

  setMotor(leftSpeed, rightSpeed);

  // ---- deteksi persimpangan: S0 & S7 hitam, TAPI masih ada sensor tengah yang putih ----
  // (syarat "ada putih di tengah" ini yang bedain simpang asli vs kondisi diangkat/full black)
  bool anyMiddleWhite = false;  
  for (uint8_t i = 0; i < 8; i++) {
    if (i == 1 || i == 6) continue;
    if (normalized[i] <= SENSOR_ON_THRESHOLD) { anyMiddleWhite = true; break; }
  }
  
  bool onIntersectionNow = (normalized[1] > SENSOR_ON_THRESHOLD) &&
                           (normalized[6] > SENSOR_ON_THRESHOLD) &&
                            anyMiddleWhite;
                            
  if (onIntersectionNow && !intersectionActive) {
    intersectionActive = true;
    intersectionCount++;
    intersectionFlashUntil = millis() + INTERSECTION_FLASH_MS;
    Serial.print(F("[SIMPANG] Persimpangan terdeteksi! Total: "));
    Serial.println(intersectionCount);

    // ---- cek mapping.h: ada script buat simpang ini? ----
    bool actionDone = false;
    executeMapStep(intersectionCount, actionDone);
    if (actionDone) {
      return; // simpang ini udah dihandle manual di mapping.h, skip sisa loop iterasi ini
    }
    // kalau actionDone masih false (case kosong/belum di-map di mapping.h)
    // -> lanjut ke behavior default LURUS di bawah, ga ada yang berubah sama sekali
  } else if (!onIntersectionNow && intersectionActive) {
    intersectionActive = false; // robot udah lewat, siap deteksi simpang berikutnya
  }

  if (millis() < intersectionFlashUntil) {
    indicateIntersection();
  } else {
    indicateSensorBar();
  }

  if (millis() - lastDebugPrint > DEBUG_PRINT_INTERVAL) {
    lastDebugPrint = millis();
    Serial.print(F("[RUNNING] sensor:"));
    for (uint8_t i = 0; i < 8; i++) {
      Serial.print(normalized[i]);
      Serial.print(i < 7 ? "," : " ");
    }
    Serial.print(F("pos:")); Serial.print(position);
    Serial.print(F(" err:")); Serial.print(error);
    Serial.print(F(" L:")); Serial.print(leftSpeed);
    Serial.print(F(" R:")); Serial.print(rightSpeed);
    Serial.print(F(" simpang:")); Serial.println(intersectionCount);
  }
}

// ================== SENSOR ==================
void readAndNormalize() {
  for (uint8_t i = 0; i < 8; i++) {
    int raw = analogRead(sensorPin[i]);
    int val = map(raw, calibMin[i], calibMax[i], 0, 1000);
    normalized[i] = constrain(val, 0, 1000);
  }
}

void doCalibration() {
  for (uint8_t i = 0; i < 8; i++) {
    calibMin[i] = 1023;
    calibMax[i] = 0;
  }

  Serial.println(F("----------------------------------------"));
  Serial.print(F("[KALIBRASI] DIMULAI - geser robot manual ngelewatin garis selama "));
  Serial.print(CALIB_DURATION_MS / 1000);
  Serial.println(F(" detik!"));

  unsigned long t0 = millis();
  unsigned long lastTick = 0;
  while (millis() - t0 < CALIB_DURATION_MS) {
    for (uint8_t i = 0; i < 8; i++) {
      int raw = analogRead(sensorPin[i]);
      if (raw < calibMin[i]) calibMin[i] = raw;
      if (raw > calibMax[i]) calibMax[i] = raw;
    }
    calibratingBlink();

    if (millis() - lastTick > 500) {
      lastTick = millis();    
      Serial.print(F("[KALIBRASI] sisa waktu: "));
      Serial.print((CALIB_DURATION_MS - (millis() - t0)) / 1000.0, 1);
      Serial.println(F(" detik"));
    }
    if (!digitalRead(BTN_STOP)) break;
  }

  for (uint8_t i = 0; i < 8; i++) {
    if (calibMax[i] - calibMin[i] < 20) calibMax[i] = calibMin[i] + 20;
  }

  Serial.println(F("[KALIBRASI] SELESAI. Hasil per sensor (min - max):"));
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(F("  S")); Serial.print(i);
    Serial.print(F(": ")); Serial.print(calibMin[i]);
    Serial.print(F(" - ")); Serial.println(calibMax[i]);
  }
  Serial.println(F("----------------------------------------"));

  saveCalibrationToEEPROM();
  Serial.println(F("[KALIBRASI] Tersimpan ke EEPROM - ga perlu diulang tiap reset"));

  strip.clear();
  strip.show();
  flashGreen();
}

// simpan/baca calibMin & calibMax dari EEPROM, ditandai 1 byte magic biar
// EEPROM kosong (chip baru) ga kebaca sebagai data valid
void saveCalibrationToEEPROM() {
  EEPROM.put(EEPROM_ADDR_DATA, calibMin);
  EEPROM.put(EEPROM_ADDR_DATA + sizeof(calibMin), calibMax);
  EEPROM.update(EEPROM_ADDR_MAGIC, EEPROM_MAGIC_VALUE);
}

bool loadCalibrationFromEEPROM() {
  if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC_VALUE) return false;
  EEPROM.get(EEPROM_ADDR_DATA, calibMin);
  EEPROM.get(EEPROM_ADDR_DATA + sizeof(calibMin), calibMax);
  return true;
}

// ================== TOMBOL ==================
// deteksi falling edge (HIGH -> LOW) dengan debounce sederhana
bool checkButton(uint8_t pin, bool &lastReading) {
  bool reading = digitalRead(pin);
  bool triggered = false;
  if (!reading)  solidRed(); 
  if (lastReading == HIGH && reading == LOW) {
    delay(30);
    if (digitalRead(pin) == LOW) triggered = true;
  }
  lastReading = reading;
  return triggered;
}

// ================== MOTOR ==================
// speed: -255..255, negatif = mundur
void setMotor(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed >= 0) {
    analogWrite(IN3, leftSpeed);
    analogWrite(IN4, 0);
  } else {
    analogWrite(IN3, 0);
    analogWrite(IN4, -leftSpeed);
  }

  if (rightSpeed >= 0) {
    analogWrite(IN1, rightSpeed);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, -rightSpeed);
  }
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ================== INDIKATOR WS2812 ==================
// tiap LED mewakili 1 sensor fisik: nyala merah kalau sensor itu kena garis hitam
void indicateSensorBar() {
  for (uint8_t i = 0; i < 8; i++) {
    if (normalized[i] > SENSOR_ON_THRESHOLD) {
      strip.setPixelColor(i, strip.Color(255, 0, 0)); // merah = kena garis
    } else {
      strip.setPixelColor(i, strip.Color(0, 0, 60));  // biru redup = area kosong
    }
  }
  strip.show();
}

// putih blink cepat, nandain "robot lagi diangkat, motor mati"
void indicateLifted() {
  static bool on = false;
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 100) {
    on = !on;
    lastBlink = millis();
    uint32_t c = on ? strip.Color(80, 80, 80) : strip.Color(0, 0, 0);
    for (uint8_t i = 0; i < 8; i++) strip.setPixelColor(i, c);
    strip.show();
  }
}

// kuning nyala penuh sesaat, nandain "persimpangan baru aja kedeteksi"
void indicateIntersection() {
  for (uint8_t i = 0; i < 8; i++) {
    strip.setPixelColor(i, strip.Color(255, 150, 0));
  }
  strip.show();
}

void indicateLineLost() {
  static bool on = false;
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 100) {
    on = !on;
    lastBlink = millis();
    uint32_t c = on ? strip.Color(255, 0, 0) : strip.Color(0, 0, 0);
    for (uint8_t i = 0; i < 8; i++) strip.setPixelColor(i, c);
    strip.show();
  }
}

void calibratingBlink() {
  static unsigned long lastUpdate = 0;
  static int8_t pos = 0;
  static int8_t direction = 1;  // 1 = kanan, -1 = kiri

  if (millis() - lastUpdate > 120) {
    lastUpdate = millis();
    strip.clear();
    strip.setPixelColor(pos, strip.Color(255, 255, 255));
    strip.show();
    pos += direction;
    // Balik arah saat mencapai ujung
    if (pos >= 7) {
      pos = 7;
      direction = -1;
    } 
    else if (pos <= 0) {
      pos = 0;
      direction = 1;
    }
  }
}

void rainbowIdle() {
  static unsigned long lastUpdate = 0;
  static uint16_t hue = 0;
  if (millis() - lastUpdate > 20) {
    lastUpdate = millis();
    for (uint8_t i = 0; i < 8; i++) {
      uint32_t c = strip.gamma32(strip.ColorHSV(hue + i * 8192));
      strip.setPixelColor(i, c);
    }
    strip.show();
    hue += 400;
  }
}

// merah nyala terus, nandain "robot lagi STOP" - dipanggil terus tiap loop
// selama currentState == ST_STOPPED, jadi ga bakal balik sendiri ke rainbow
void solidRed() {
  for (uint8_t i = 0; i < 8; i++) {
    strip.setPixelColor(i, strip.Color(255, 0, 0));
  }
  strip.setBrightness(70);
  strip.show();
  strip.setBrightness(LED_BRIGHTNESS);
}

// blink hijau, nandain "rute mapping selesai (FINISH)" - dipanggil terus
// tiap loop selama currentState == ST_FINISH, sebelum auto balik ke IDLE
void indicateFinish() {
  static bool on = false;
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 120) {
    on = !on;
    lastBlink = millis();
    uint32_t c = on ? strip.Color(0, 255, 0) : strip.Color(0, 0, 0);
    for (uint8_t i = 0; i < 8; i++) strip.setPixelColor(i, c);
    strip.show();
  }
}

// kedip hijau sekali, nandain "kalibrasi baru aja selesai"
void flashGreen() {
  for (uint8_t r = 0; r < 2; r++) {
    for (uint8_t i = 0; i < 8; i++) strip.setPixelColor(i, strip.Color(0, 255, 0));
    strip.show();
    delay(150);
    strip.clear();
    strip.show();
    delay(150);
  }
}

// mapping.h di-include di SINI (paling bawah) - biar semua yang dia butuhin
// (strip, setMotor, stopMotor, BASE_SPEED, PIVOT_SPEED, BTN_STOP, currentState,
// ST_FINISH, finishStateEnteredAt, lastError, lastErrorSign) udah pasti
// kekenal duluan sama compiler.
#include "mapping.h"
