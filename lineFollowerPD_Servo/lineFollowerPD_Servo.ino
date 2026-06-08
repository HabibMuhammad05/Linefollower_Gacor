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
      MAPPING    -> LED beda-beda sesuai aksi (lihat bagian MAPPING JALUR)
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

  MAPPING JALUR (BELOK/SERVO OTOMATIS DI SIMPANG):
    Keputusan robot di tiap simpang diatur di file "mapping.h", satu
    array berisi STRUCT { aksi, durasiMs } - urut simpang ke-1, ke-2,
    dst. durasiMs=0 pakai nilai default (TURN_KIRI_MS dst di mapping.h
    juga), atau isi angka buat override durasi KHUSUS baris itu aja
    (misal simpang ke-3 belok kiri 300ms, simpang ke-7 belok kiri
    500ms - beda-beda per simpang walau aksinya sama).
    Kalau simpang yang dilewati belum di-map, robot otomatis LURUS
    seperti biasa (behavior lama, ga berubah sama sekali).
    Aksi yang tersedia (detail & warna LED di mapping.h & komentar
    updateDecisionLED()): KIRI, KANAN, PUTAR, BERHENTI (selesai rute,
    masuk FINISH), STOP_SESAAT (jeda doang lalu lanjut jalan lagi),
    MUNDUR, dan 4 primitif servo (SERVO_NAIK/TURUN/BUKA/TUTUP - lihat
    bagian SERVO di bawah).
    ACT_BERHENTI beda dari tombol STOP manual: BERHENTI dianggap
    "sudah sampai tujuan" -> state FINISH (hijau blink) -> auto ke
    IDLE lagi. Tombol STOP manual tetap merah nempel nunggu user.

    Rencana ke depan (BELUM diimplementasi, cuma catatan): mau ada
    beberapa PROFIL mapping (misal jalurMap A/B/C) yang bisa dipilih,
    masing-masing punya urutan keputusan sendiri buat rute berbeda,
    dan servo dipicu BARENG aksi gerak lain di simpang yang sama
    (sekarang masih 1 aksi per simpang dulu, gantian).

  SERVO (gripper, dari tesServo.ino, pin & sudut di "servo_config.h"):
    4 fungsi PRIMITIF terpisah - servoNaik(), servoTurun(),
    servoBukaJepit(), servoTutupJepit() - SENGAJA ga disequence jadi
    "ambil barang"/"simpan barang" kayak tesServo.ino, biar bisa
    dipanggil satu-satu lewat mapping.h (ACT_SERVO_NAIK dst), sesuai
    rencana ke depan gripper dipicu sendiri-sendiri per simpang (misal
    "turun duluan" di satu simpang, "tutup jepit" di simpang lain).
    Gerakannya halus (1 derajat per step, delay singkat) niru pola
    tesServo.ino yang udah kebukti jalan.

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
  (EEPROM.h dan Servo.h sudah built-in di Arduino core, tidak perlu install)
*/

#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <Servo.h>
#include "mapping.h"       // jalur/keputusan robot per simpang - edit di file ini
#include "servo_config.h"  // pin & sudut servo gripper

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

Servo jepitServo;
Servo angkatServo;
// posisi TERAKHIR yang di-write ke tiap servo, dipakai buat gerak halus
// dari posisi SEKARANG ke target (bukan loncat langsung). Nilai awal
// diasumsikan jepit tertutup & gripper naik - SESUAIKAN kalau posisi
// fisik awal robot beda dari asumsi ini.
int jepitPos  = JEPIT_TUTUP_ANGLE;
int angkatPos = ANGKAT_NAIK_ANGLE;

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

  jepitServo.attach(SERVO_JEPIT_PIN);
  angkatServo.attach(SERVO_ANGKAT_PIN);
  // SENGAJA ga langsung .write() di sini (biar ga ada gerakan mendadak
  // pas boot) - servo baru gerak begitu salah satu fungsi aksi dipanggil.
  Serial.println(F("[OK] Servo jepit(D9) & angkat(D10) diinisialisasi"));

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
  for (uint8_t i = 1; i <= 6; i++) {
    if (normalized[i] <= SENSOR_ON_THRESHOLD) { anyMiddleWhite = true; break; }
  }
  bool onIntersectionNow = (normalized[0] > SENSOR_ON_THRESHOLD) &&
                            (normalized[7] > SENSOR_ON_THRESHOLD) &&
                            anyMiddleWhite;
  if (onIntersectionNow && !intersectionActive) {
    intersectionActive = true;
    intersectionCount++;
    intersectionFlashUntil = millis() + INTERSECTION_FLASH_MS;
    Serial.print(F("[SIMPANG] Persimpangan terdeteksi! Total: "));
    Serial.println(intersectionCount);

    // ---- cek mapping.h: ada keputusan buat simpang ini? ----
    MapEntry entry = getMapEntry(intersectionCount);
    if (entry.action != ACT_LURUS) {
      unsigned long durasi = (entry.durasiMs > 0) ? entry.durasiMs : defaultDurasi(entry.action);
      Serial.print(F("[SIMPANG] Keputusan mapping: "));
      Serial.print(actionName(entry.action));
      Serial.print(F(" (durasi: "));
      Serial.print(durasi);
      Serial.println(F("ms)"));
      executeIntersectionAction(entry.action, durasi); // eksekusi manuver + LED (non-blocking di dalam)
      lastError = 0;                                   // reset PD biar ga "kaget" abis pivot
      return;                                           // sudah dihandle, skip sisa loop iterasi ini
    }
    // kalau ACT_LURUS (default/belum di-map) -> lanjut ke behavior lama di bawah,
    // ga ada yang berubah sama sekali
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

// ================== MAPPING JALUR (baca mapping.h) ==================
// ambil entry (aksi+durasi) buat simpang ke-n (n mulai dari 1). Kalau
// belum di-map, ACT_LURUS.
MapEntry getMapEntry(unsigned int simpangKe) {
  if (simpangKe == 0 || simpangKe > JALUR_MAP_COUNT) return MapEntry{ACT_LURUS, 0};
  return jalurMap[simpangKe - 1];
}

// durasi default per tipe aksi, dipakai kalau durasiMs di mapping.h diisi 0
unsigned long defaultDurasi(TurnAction action) {
  switch (action) {
    case ACT_KIRI:        return TURN_KIRI_MS;
    case ACT_KANAN:       return TURN_KANAN_MS;
    case ACT_PUTAR:       return TURN_PUTAR_MS;
    case ACT_STOP_SESAAT: return STOP_SESAAT_MS;
    case ACT_MUNDUR:      return MUNDUR_MS;
    default:              return 0; // servo & LURUS/BERHENTI ga pakai durasi
  }
}

const char* actionName(TurnAction action) {
  switch (action) {
    case ACT_KIRI:         return "BELOK KIRI";
    case ACT_KANAN:        return "BELOK KANAN";
    case ACT_PUTAR:        return "PUTAR BALIK";
    case ACT_BERHENTI:     return "BERHENTI (sampai tujuan)";
    case ACT_STOP_SESAAT:  return "STOP SESAAT";
    case ACT_MUNDUR:       return "MUNDUR";
    case ACT_SERVO_NAIK:   return "SERVO NAIK";
    case ACT_SERVO_TURUN:  return "SERVO TURUN";
    case ACT_SERVO_BUKA:   return "SERVO BUKA JEPIT";
    case ACT_SERVO_TUTUP:  return "SERVO TUTUP JEPIT";
    default:               return "LURUS";
  }
}

// warna LED per jenis keputusan, biar gampang dibedain dari mode lain
uint32_t decisionColor(TurnAction action) {
  switch (action) {
    case ACT_KIRI:         return strip.Color(0, 255, 255);   // cyan
    case ACT_KANAN:        return strip.Color(255, 0, 150);   // pink/magenta
    case ACT_PUTAR:        return strip.Color(150, 0, 255);   // ungu
    case ACT_BERHENTI:     return strip.Color(255, 0, 0);     // merah
    case ACT_STOP_SESAAT:  return strip.Color(0, 120, 255);   // biru
    case ACT_MUNDUR:       return strip.Color(255, 100, 0);   // oranye
    case ACT_SERVO_NAIK:   return strip.Color(0, 255, 120);   // hijau tosca
    case ACT_SERVO_TURUN:  return strip.Color(0, 180, 180);   // teal
    case ACT_SERVO_BUKA:   return strip.Color(255, 255, 255); // putih
    case ACT_SERVO_TUTUP:  return strip.Color(180, 180, 0);   // kuning pudar
    default:               return strip.Color(255, 255, 255);
  }
}

// Animasi dot LED nunjukin arah keputusan. TIDAK PAKAI delay() ATAU
// for-loop nge-blok - tiap dipanggil cuma update posisi dot kalau sudah
// STEP_MS berlalu (dicek pakai millis()), jadi aman dipanggil berkali-kali
// di dalam while nunggu durasi manuver tanpa nge-freeze apapun:
//   ACT_KIRI        -> dot lari ke arah kiri (index 7 -> 0, ulang)
//   ACT_KANAN       -> dot lari ke arah kanan (index 0 -> 7, ulang)
//   ACT_PUTAR       -> 2 dot nyebar dari tengah ke pinggir (3,4 -> 0,7)
//   ACT_BERHENTI    -> 2 dot nyempit dari pinggir ke tengah (0,7 -> 3,4)
//   ACT_STOP_SESAAT -> seluruh strip blink biru (kayak lampu hazard)
//   ACT_MUNDUR      -> seluruh strip blink oranye (kayak alarm mundur mobil)
void updateDecisionLED(TurnAction action) {
  static unsigned long lastUpdate = 0;
  static uint8_t phase = 0;
  const unsigned long STEP_MS = 60;

  if (millis() - lastUpdate < STEP_MS) return;
  lastUpdate = millis();

  strip.clear();
  uint32_t color = decisionColor(action);

  switch (action) {
    case ACT_KIRI:
      strip.setPixelColor(7 - (phase % 8), color);
      break;
    case ACT_KANAN:
      strip.setPixelColor(phase % 8, color);
      break;
    case ACT_PUTAR: {
      uint8_t step = phase % 4;
      strip.setPixelColor(3 - step, color);
      strip.setPixelColor(4 + step, color);
      break;
    }
    case ACT_BERHENTI: {
      uint8_t step = phase % 4;
      strip.setPixelColor(step, color);
      strip.setPixelColor(7 - step, color);
      break;
    }
    case ACT_STOP_SESAAT:
    case ACT_MUNDUR: {
      bool on = (phase % 2) == 0;
      if (on) for (uint8_t i = 0; i < 8; i++) strip.setPixelColor(i, color);
      break;
    }
    default:
      break;
  }
  strip.show();
  phase++;
}

// blink 2x cepat pakai warna aksi servo - indikator SESAAT sebelum servo
// mulai gerak (bukan animasi jalan kaya updateDecisionLED, cuma flash
// singkat). Masih pakai delay() di sini, sama kategori kayak flashGreen()
// - one-shot state-transition flash yg emang udah dari awal blocking.
void indicateServoFlash(TurnAction action) {
  uint32_t color = decisionColor(action);
  for (uint8_t r = 0; r < 2; r++) {
    for (uint8_t i = 0; i < 8; i++) strip.setPixelColor(i, color);
    strip.show();
    delay(100);
    strip.clear();
    strip.show();
    delay(100);
  }
}

// eksekusi manuver fisik sesuai keputusan mapping. `durasi` sudah final
// (hasil override dari mapping.h ATAU default), dihitung di runLineFollowing.
// Manuver gerak (KIRI/KANAN/PUTAR/MUNDUR/STOP_SESAAT) nunggu pakai
// while+millis (BUKAN delay()), biar LED animasi arah (updateDecisionLED)
// bisa keupdate mulus selama proses. Servo pakai gerakan halus per-derajat
// (servoSweep) yang masih delay-based, sama pola kayak tesServo.ino.
void executeIntersectionAction(TurnAction action, unsigned long durasi) {
  switch (action) {
    case ACT_KIRI:
    case ACT_KANAN:
    case ACT_PUTAR: {
      int leftSpd, rightSpd;
      if (action == ACT_KIRI)       { leftSpd = -TURN_SPEED; rightSpd =  TURN_SPEED; }
      else if (action == ACT_KANAN) { leftSpd =  TURN_SPEED; rightSpd = -TURN_SPEED; }
      else                          { leftSpd =  TURN_SPEED; rightSpd = -TURN_SPEED; }

      setMotor(leftSpd, rightSpd);
      unsigned long t0 = millis();
      while (millis() - t0 < durasi) {
        updateDecisionLED(action);
        if (!digitalRead(BTN_STOP)) break; // bisa dibatalin manual kalau darurat
      }
      stopMotor();
      break;
    }
    case ACT_MUNDUR: {
      setMotor(-MUNDUR_SPEED, -MUNDUR_SPEED);
      unsigned long t0 = millis();
      while (millis() - t0 < durasi) {
        updateDecisionLED(action);
        if (!digitalRead(BTN_STOP)) break;
      }
      stopMotor();
      break;
    }
    case ACT_STOP_SESAAT: {
      stopMotor();
      unsigned long t0 = millis();
      while (millis() - t0 < durasi) {
        updateDecisionLED(action);
      }
      // abis ini FUNGSI SELESAI, balik ke runLineFollowing() normal lagi
      // di loop berikutnya - BUKAN ke FINISH atau IDLE.
      break;
    }
    case ACT_BERHENTI: {
      stopMotor();
      unsigned long t0 = millis();
      while (millis() - t0 < BERHENTI_PREVIEW_MS) {
        updateDecisionLED(action); // animasi nyempit ke tengah sesaat sebelum FINISH
      }
      Serial.println(F("[SIMPANG] Rute mapping selesai - FINISH"));
      finishStateEnteredAt = millis();
      currentState = ST_FINISH;
      break;
    }
    case ACT_SERVO_NAIK:
      stopMotor();
      indicateServoFlash(action);
      servoNaik();
      break;
    case ACT_SERVO_TURUN:
      stopMotor();
      indicateServoFlash(action);
      servoTurun();
      break;
    case ACT_SERVO_BUKA:
      stopMotor();
      indicateServoFlash(action);
      servoBukaJepit();
      break;
    case ACT_SERVO_TUTUP:
      stopMotor();
      indicateServoFlash(action);
      servoTutupJepit();
      break;
    default:
      break; // ACT_LURUS ga akan sampai sini (sudah di-filter sebelum manggil fungsi ini)
  }
}

// ================== SERVO (gripper) ==================
// 4 fungsi PRIMITIF terpisah, SENGAJA ga disequence jadi "ambil barang"/
// "simpan barang" - masing-masing dipanggil sendiri-sendiri (langsung
// atau lewat mapping.h) biar fleksibel dipasang di simpang manapun,
// termasuk rencana ke depan "gripper turun dipicu simpang duluan".

// gerak servo halus dari posisi SEKARANG ke target, 1 derajat per
// SERVO_STEP_DELAY_MS (pola sama kayak tesServo.ino, dibikin generic
// biar dipakai 4 fungsi di bawah tanpa duplikat kode)
void servoSweep(Servo &s, int &posVar, int target) {
  int step = (target >= posVar) ? 1 : -1;
  while (posVar != target) {
    posVar += step;
    s.write(posVar);
    delay(SERVO_STEP_DELAY_MS);
  }
}

void servoNaik() {
  Serial.println(F("[SERVO] Gripper naik"));
  servoSweep(angkatServo, angkatPos, ANGKAT_NAIK_ANGLE);
}

void servoTurun() {
  Serial.println(F("[SERVO] Gripper turun"));
  servoSweep(angkatServo, angkatPos, ANGKAT_TURUN_ANGLE);
}

void servoBukaJepit() {
  Serial.println(F("[SERVO] Jepit buka"));
  servoSweep(jepitServo, jepitPos, JEPIT_BUKA_ANGLE);
}

void servoTutupJepit() {
  Serial.println(F("[SERVO] Jepit tutup"));
  servoSweep(jepitServo, jepitPos, JEPIT_TUTUP_ANGLE);
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

  if (millis() - lastUpdate > 60) {
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
