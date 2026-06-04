#ifndef MAPPING_H
#define MAPPING_H

/*
  MAPPING JALUR ROBOT - versi SCRIPT per persimpangan
  ====================================================================
  File ini KHUSUS buat atur apa yang robot lakuin di tiap persimpangan,
  biar gampang diedit tanpa bongkar file utama.

  CARA PAKAI:
  Edit fungsi executeMapStep() di bawah. SWITCH LUAR (planNum) milih rute
  yang mana yang dipakai - planNum dipilih user di ST_IDLE lewat tombol
  (short-press CALIBRATE = naik, short-press STOP = turun, LED blink
  nunjukin nomor plan yang lagi kepilih). SWITCH DALAM (simpangKe) sama
  kayak sebelumnya: tiap "case N:" = persimpangan ke-N (N=1 simpang
  pertama, dst - sama kayak intersectionCount). Isi tiap case BEBAS,
  gabungin primitive di bawah sesuai urutan yang lu mau.

  WAJIB set actionDone = true kalau case itu mau AMBIL ALIH (skip
  logic default LURUS buat simpang ini). Kalau case kosong / nomor
  simpangnya belum di-map sama sekali, actionDone tetap false ->
  robot default LURUS (lanjut ngikutin PD normal, behavior lama,
  ga ada yang berubah).

  PRIMITIVE YANG TERSEDIA (implementasinya di file utama .ino):
    turnLeft()            - pivot belok kiri 90 derajat (LED cyan lari kiri)
    turnRight()            - pivot belok kanan 90 derajat (LED pink lari kanan)
    turnAround()            - putar balik 180 derajat (LED ungu nyebar)
    maju(ms)                - jalan lurus BASE_SPEED selama ms (default MAJU_MS)
    mundur(ms)               - jalan MUNDUR selama ms (default MUNDUR_MS), LED amber blink
    stopSesaat(ms)            - motor berhenti SEBENTAR (default STOP_SESAAT_MS),
                                 TAPI robot TETAP lanjut ke simpang berikutnya -
                                 BEDA sama selesaiRute() yang tamatin seluruh rute!
    resetLineFollowing()      - reset lastError/lastErrorSign PD, dipanggil abis
                                 manuver ganjil (kayak turnAround) biar PD ga "kaget"
    ambilKotak()              - servo: turun -> tutup jepit -> naik lagi (ambil barang)
    lepasKotak()              - servo: turun -> buka jepit -> naik lagi (taruh barang)
    servoNaik() / servoTurun()          - gerak servo angkat doang (individual)
    servoBukaJepit() / servoTutupJepit() - gerak servo jepit doang (individual)
    selesaiRute()              - tandain SELURUH rute kelar -> robot stop total,
                                 LED hijau blink (state FINISH), lalu OTOMATIS
                                 balik ke IDLE (tinggal pencet START lagi buat
                                 run berikutnya - TIDAK mengunci/hang selamanya)

  SOAL SERVO (pin & sudut di "servo_config.h"):
    Gerakannya SELALU halus (1 derajat per step, ga pernah loncat ke sudut
    jauh langsung) - ini penting karena servo & Arduino numpang di 1
    step-down yang sama; gerakan mendadak bisa bikin lonjakan arus yang
    ngedrop tegangan & ganggu servo/Arduino. Pas boot, servo otomatis
    di-sweep pelan ke posisi NAIK + BUKA (lihat setup() di file utama).

  CONTOH DI BAWAH (silakan diganti sesuai jalur arena beneran):
    simpang 1 -> belok kiri
    simpang 2 -> stop sesaat, ambil kotak, puter balik, reset PD
    simpang 3 -> (kosong, LURUS)
    simpang 4 -> stop sesaat, lepas kotak, puter balik
    simpang 5 -> belok kanan
    simpang 6 -> maju dikit, lalu SELESAI (auto balik ke IDLE)
  ====================================================================
*/

enum TurnAction { ACT_LURUS, ACT_KIRI, ACT_KANAN, ACT_PUTAR, ACT_BERHENTI };

// ---- Tuning maneuver (kecepatan & durasi) - WAJIB dikalibrasi manual di
//      lapangan, beda arena/beban robot = beda waktu yang pas ----
const int           TURN_SPEED     = 180; // kecepatan pivot belok/putar (0-255)
const unsigned long TURN_KIRI_MS   = 350; // durasi pivot kiri  (~90 derajat)
const unsigned long TURN_KANAN_MS  = 300; // durasi pivot kanan (~90 derajat)
const unsigned long TURN_PUTAR_MS  = 500; // durasi u-turn      (~180 derajat)
const unsigned long MAJU_MS        = 300; // durasi default maju()
const unsigned long MUNDUR_MS      = 100; // durasi default mundur()
const unsigned long STOP_SESAAT_MS = 1000; // durasi default stopSesaat()

// ================== MAPPING JALUR: PRIMITIVE (dipanggil dari mapping.h) ==================
// Semua primitive di bawah "blocking" (robot ga ngikutin PD selama primitive
// ini jalan) - sama gaya kayak doCalibration() yang udah ada. LED animasi
// tetap smooth karena update pakai millis(), bukan delay() polos.

// warna LED per jenis manuver, biar gampang dibedain dari mode lain
uint32_t decisionColor(TurnAction action) {
  switch (action) {
    case ACT_KIRI:     return strip.Color(0, 255, 255);   // cyan
    case ACT_KANAN:    return strip.Color(255, 0, 150);   // pink/magenta
    case ACT_PUTAR:    return strip.Color(150, 0, 255);   // ungu
    case ACT_BERHENTI: return strip.Color(255, 0, 0);     // merah (dipakai stopSesaat)
    default:           return strip.Color(255, 255, 255); // putih (ga kepake, jaga2)
  }
}

// Animasi dot LED nunjukin manuver yang lagi jalan. TIDAK PAKAI delay() -
// tiap dipanggil cuma update posisi kalau STEP_MS udah lewat (cek millis()),
// aman dipanggil berkali-kali di dalam while nunggu durasi tanpa nge-freeze:
//   ACT_KIRI     -> dot lari ke kiri (index 7 -> 0, ulang)
//   ACT_KANAN    -> dot lari ke kanan (index 0 -> 7, ulang)
//   ACT_PUTAR    -> 2 dot nyebar dari tengah ke pinggir (3,4 -> 0,7)
//   ACT_BERHENTI -> 2 dot nyempit dari pinggir ke tengah (0,7 -> 3,4) - dipakai stopSesaat()
void updateDecisionLED(TurnAction action) {
  static unsigned long lastUpdate = 0;
  static uint8_t phase = 0;
  const unsigned long STEP_MS = 40;

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
    default:
      break;
  }
  strip.show();
  phase++;
}

// blink amber, nandain "robot lagi mundur()"
void indicateMundur() {
  static bool on = false;
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 150) {
    on = !on;
    lastBlink = millis();
    uint32_t c = on ? strip.Color(255, 80, 0) : strip.Color(0, 0, 0);
    for (uint8_t i = 0; i < 8; i++) strip.setPixelColor(i, c);
    strip.show();
  }
}

// pivot belok/putar generik, dipakai turnLeft()/turnRight()/turnAround()
void runTurnManeuver(TurnAction kind, unsigned long durationMs) {
  int leftSpd, rightSpd;
  if (kind == ACT_KIRI)       { leftSpd = -TURN_SPEED; rightSpd =  TURN_SPEED; }
  else if (kind == ACT_KANAN) { leftSpd =  TURN_SPEED; rightSpd = -TURN_SPEED; }
  else                        { leftSpd =  TURN_SPEED; rightSpd = -TURN_SPEED; } // ACT_PUTAR

  setMotor(leftSpd, rightSpd);
  unsigned long t0 = millis();
  while (millis() - t0 < durationMs) {
    updateDecisionLED(kind);
    if (!digitalRead(BTN_STOP)) break; // bisa dibatalin manual kalau darurat
  }
  stopMotor();
}

void turnLeft() {
  Serial.println(F("[MAPPING] turnLeft()"));
  runTurnManeuver(ACT_KIRI, TURN_KIRI_MS);
}

void turnRight() {
  Serial.println(F("[MAPPING] turnRight()"));
  runTurnManeuver(ACT_KANAN, TURN_KANAN_MS);
}

void turnAround() {
  Serial.println(F("[MAPPING] turnAround()"));
  runTurnManeuver(ACT_PUTAR, TURN_PUTAR_MS);
}

void maju(unsigned long ms = MAJU_MS) {
  Serial.println(F("[MAPPING] maju()"));
  setMotor(BASE_SPEED, BASE_SPEED);
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (!digitalRead(BTN_STOP)) break;
  }
  stopMotor();
}

void mundur(unsigned long ms = MUNDUR_MS) {
  Serial.println(F("[MAPPING] mundur()"));
  setMotor(-PIVOT_SPEED, -PIVOT_SPEED);
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    indicateMundur();
    if (!digitalRead(BTN_STOP)) break;
  }
  stopMotor();
}

// berhenti SEBENTAR doang - robot TETAP lanjut ke simpang berikutnya
// setelah ini, BEDA sama selesaiRute() yang tamatin seluruh rute
void stopSesaat(unsigned long ms = STOP_SESAAT_MS) {
  Serial.println(F("[MAPPING] stopSesaat()"));
  stopMotor();
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    updateDecisionLED(ACT_BERHENTI); // dot merah nyempit ke tengah = "lagi pause"
  }
}

void resetLineFollowing() {
  lastError = 0;
  lastErrorSign = 1;
  Serial.println(F("[MAPPING] resetLineFollowing()"));
}

// ---------------- SERVO (gripper) ----------------
// Gerak servo SELALU halus (1 derajat per SERVO_STEP_DELAY_MS, dari
// servo_config.h), ga pernah write() langsung loncat ke sudut jauh -
// ini yang bikin ga ada lonjakan arus mendadak ke step-down yang
// dipakai bareng Arduino & motor.
void servoSweep(Adafruit_TiCoServo &s, int &posVar, int target) {
  int step = (target >= posVar) ? 1 : -1;
  while (posVar != target) {
    posVar += step;
    s.write(posVar);
    delay(SERVO_STEP_DELAY_MS);
  }
}

void servoNaik() {
  Serial.println(F("[SERVO] naik"));
  servoSweep(angkatServo, angkatPos, ANGKAT_NAIK_ANGLE);
}

void servoTurun() {
  Serial.println(F("[SERVO] turun"));
  servoSweep(angkatServo, angkatPos, ANGKAT_TURUN_ANGLE);
}

void servoBukaJepit() {
  Serial.println(F("[SERVO] buka jepit"));
  servoSweep(jepitServo, jepitPos, JEPIT_BUKA_ANGLE);
}

void servoTutupJepit() {
  Serial.println(F("[SERVO] tutup jepit"));
  servoSweep(jepitServo, jepitPos, JEPIT_TUTUP_ANGLE);
}

// kombinasi siap pakai - turun, jepit, naik lagi (biar barang ke-angkat)
void ambilKotak() {
  Serial.println(F("[MAPPING] ambilKotak()"));
  servoTurun();
  servoTutupJepit();
  servoNaik();
}

// kombinasi siap pakai - turun, lepas jepit, naik lagi
void lepasKotak() {
  Serial.println(F("[MAPPING] lepasKotak()"));
  servoTurun();
  servoBukaJepit();
  servoNaik();
}

// tandain SELURUH rute mapping udah kelar (bukan cuma 1 simpang) - robot
// stop total, LED hijau blink (state FINISH), lalu OTOMATIS balik ke IDLE
// begitu FINISH_DURATION_MS lewat (lihat switch(currentState) di loop()),
// siap buat run berikutnya tinggal pencet START lagi - TIDAK hang selamanya
void selesaiRute() {
  Serial.println(F("[MAPPING] selesaiRute() - rute selesai, FINISH -> auto balik IDLE"));
  stopMotor();
  finishStateEnteredAt = millis();
  currentState = ST_FINISH;
}

// ------------------------ EDIT DI SINI ------------------------
// planNum dipilih user di ST_IDLE lewat tombol (short-press CALIBRATE =
// naik, short-press STOP = turun, lihat loop() di file utama), LED nunjukin
// nomor plan yang lagi kepilih (blink cepat di atas rainbow). executeMapStep()
// dipanggil tiap simpang kedeteksi - switch LUAR pilih plan-nya (rute mana
// yang dipakai), switch DALAM sama persis kayak sebelumnya (per nomor
// simpang di rute itu).
//
// Nambah plan baru = tambah 1 "case N:" baru di switch luar, isi switch
// dalamnya bebas kayak biasa.
void executeMapStep(unsigned int simpangKe, bool &actionDone) {
  switch (planNum) {

    case 0: // warna kotak & gudang sejajar
      switch (simpangKe) {
        case 1: turnLeft(); actionDone = true; break;
        case 2: stopSesaat(); ambilKotak(); turnAround(); resetLineFollowing(); actionDone = true; break;
        case 3: actionDone = true; break;
        case 4: stopSesaat(); lepasKotak(); turnAround(); actionDone = true; break;
        case 5: turnRight(); actionDone = true; break;
        case 6: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 1: // warna kotak di atas 1
      switch (simpangKe) {
        case 1: turnLeft(); actionDone = true; break;
        case 2: stopSesaat(); ambilKotak(); turnAround(); resetLineFollowing(); actionDone = true; break;
        case 3: turnLeft(); actionDone = true; break;
        case 4: turnRight(); actionDone = true; break;
        case 5: stopSesaat(); lepasKotak(); turnAround(); actionDone = true; break;
        case 6: turnLeft(); actionDone = true; break;
        case 7: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 2: // warna kotak di atas 2
      switch (simpangKe) {
        case 1: turnLeft(); actionDone = true; break;
        case 2: stopSesaat(); ambilKotak(); turnAround(); resetLineFollowing(); actionDone = true; break;
        case 3: turnLeft(); actionDone = true; break;
        case 4: actionDone = true; break;
        case 5: turnRight(); actionDone = true; break;
        case 6: stopSesaat(); lepasKotak(); turnAround(); actionDone = true; break;
        case 7: turnLeft(); actionDone = true; break;
        case 8: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 3: // warna kotak di bawah 1
      switch (simpangKe) {
        case 1: turnLeft(); actionDone = true; break;
        case 2: stopSesaat(); ambilKotak(); turnAround(); resetLineFollowing(); actionDone = true; break;
        case 3: turnRight(); actionDone = true; break;
        case 4: turnLeft(); actionDone = true; break;
        case 5: stopSesaat(); lepasKotak(); turnAround(); actionDone = true; break;
        case 6: turnRight(); actionDone = true; break;
        case 7: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 4: // warna kotak di bawah 2
      switch (simpangKe) {
        case 1: turnLeft(); actionDone = true; break;
        case 2: stopSesaat(); ambilKotak(); turnAround(); resetLineFollowing(); actionDone = true; break;
        case 3: turnRight(); actionDone = true; break;
        case 4: actionDone = true; break;
        case 5: turnLeft(); actionDone = true; break;
        case 6: stopSesaat(); lepasKotak(); turnAround(); actionDone = true; break;
        case 7: turnRight(); actionDone = true; break;
        case 8: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 5: // kejar retry 1
      switch (simpangKe) {
        case 1: actionDone = true; break;
        case 2: turnLeft(); actionDone = true; break;
        case 3: turnLeft(); actionDone = true; break;
        case 4: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 6: // loncat 1 retry
      switch (simpangKe) {
        case 1: actionDone = true; break;
        case 2: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 7: // loncat 2 retry
      switch (simpangKe) {
        case 1: actionDone = true; break;
        case 2: actionDone = true; break;
        case 3: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;


    default:
      break; // planNum di luar 0-8 -> actionDone tetap false, LURUS normal
  }
}
// ----------------------------------------------------------------

//    case 0: // dari start ke finish langsung (ga ambil/lepas kotak)
//      switch (simpangKe) {
//        case 1: actionDone = true; break;
//        case 2: turnLeft(); actionDone = true; break;
//        case 3: actionDone = true; break;
//        case 4: actionDone = true; break;
//        case 5: actionDone = true; break;
//        case 6: actionDone = true; break;
//        case 7: maju(); selesaiRute(); actionDone = true; break;
//        default: break;
//      }
//      break;
// ====================================================================
// CATATAN: multi-plan (switch planNum di atas) UDAH diimplementasi - bukan
// rencana lagi. PLAN_COUNT (di file utama) HARUS disamain kalau nambah/
// kurang jumlah case di atas (misal nambah plan ke-9 = case 9, PLAN_COUNT
// jadi 10), soalnya itu yang dipakai buat wraparound tombol naik/turun plan.
// ====================================================================


#endif
