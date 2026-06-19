#ifndef CONFIG_H
#define CONFIG_H

// =====================================================
// PIN MOTOR (MX1508)
// NOTE: mapping ini diambil dari tesMotor.ino yang sudah
// teruji di hardware. Ini BEDA dari mapping awal di
// project summary markdown (IN1<->IN2 & IN3<->IN4 tertukar
// di dokumen lama). Dokumen akan disesuaikan mengikuti ini.
// =====================================================
#define IN1 6
#define IN2 11
#define IN3 3
#define IN4 5

// =====================================================
// PIN SERVO
// =====================================================
#define PIN_JEPIT  9
#define PIN_ANGKAT 10

// =====================================================
// PIN SENSOR LINE (8 channel)
// =====================================================
const byte SENSOR_PIN[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
// TODO: pastikan index 0 = paling kiri robot, index 7 = paling kanan.
// Kalau ternyata terbalik secara fisik, tinggal balik urutan array ini.

// =====================================================
// PIN TOMBOL & LED
// =====================================================
#define BTN_START 2   // tekan sekali -> mulai jalan
#define BTN_STOP  4   // tekan -> stop / pause darurat
#define BTN_CALIB 12  // tekan & tahan sebelum start -> kalibrasi ulang sensor
#define LED_INDICATOR 13

// =====================================================
// SUDUT SERVO (dari tesServo_nonblocking.ino)
// =====================================================
const int JEPIT_BUKA   = 0;
const int JEPIT_TUTUP  = 65;
const int ANGKAT_NAIK  = 65;
const int ANGKAT_TURUN = 180;
const int DELAY_SERVO  = 15;

// =====================================================
// KECEPATAN MOTOR (PWM 0-255)
// TODO: tuning di lapangan sesuai kondisi lintasan & beban
// =====================================================
const int BASE_SPEED   = 150; // kecepatan dasar line follow
const int MAX_SPEED    = 220; // batas atas PWM saat koreksi PID
const int ROTATE_SPEED = 150; // kecepatan saat belok/putar di persimpangan

// =====================================================
// PID LINE FOLLOWING
// TODO: tuning. Mulai naikkan KP pelan-pelan sampai mulai
// oscillasi ringan, baru tambah KD untuk meredam. KI biasanya
// tidak perlu / dibuat sangat kecil untuk line follower.
// =====================================================
float KP = 0.06;
float KI = 0.0;
float KD = 0.4;

// Bobot posisi tiap sensor: kiri negatif, kanan positif, tengah = 0
const int SENSOR_WEIGHT[8] = {-35, -25, -15, -5, 5, 15, 25, 35};

// Ambang deteksi persimpangan: minimal berapa sensor aktif bersamaan
// TODO: sesuaikan dengan bentuk fisik persimpangan (T-junction / cross)
const byte INTERSECTION_SENSOR_COUNT = 5;

// Timeout maksimal saat robot belok di persimpangan (safety, ms)
const unsigned long BELOK_TIMEOUT_MS = 2000;

// =====================================================
// JALUR / PATH ROBOT (SCALABLE)
// Tinggal ubah array PATH[] ini untuk menambah/mengubah
// rute robot -> TIDAK PERLU ubah logika state machine di
// RobotTransporter.ino sama sekali.
//
// Setiap elemen mewakili aksi yang dieksekusi di setiap
// persimpangan yang terdeteksi berurutan.
// =====================================================
enum Aksi {
  LURUS,          // lewati persimpangan, lanjut line follow
  BELOK_KIRI,
  BELOK_KANAN,
  PUTAR_BALIK,
  AMBIL_BARANG,   // berhenti, jalankan servo ambilBarang()
  ANTAR_BARANG,   // berhenti, jalankan servo simpanBarang()
  SELESAI         // akhir rute, robot berhenti total
};

// Contoh rute (silakan diedit sesuai jalur nyata):
// lurus -> belok kiri -> ambil barang -> putar balik ->
// belok kanan -> lurus -> antar barang -> selesai
const Aksi PATH[] = {
  LURUS,
  BELOK_KIRI,
  AMBIL_BARANG,
  PUTAR_BALIK,
  BELOK_KANAN,
  LURUS,
  ANTAR_BARANG,
  SELESAI
};
const byte PATH_LENGTH = sizeof(PATH) / sizeof(PATH[0]);

#endif
