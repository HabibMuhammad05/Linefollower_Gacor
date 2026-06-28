#ifndef CONFIG_H
#define CONFIG_H

// =====================================================
// PIN MOTOR (MX1508) - dari tesMotor.ino yang sudah teruji
// =====================================================
#define IN1 6
#define IN2 11
#define IN3 3
#define IN4 5

// =====================================================
// PIN SENSOR LINE (8 channel) - persis dari tesSensor.ino
// Array_resistor_8 di skematik: s0..s7 berurutan kiri ke kanan
// =====================================================
const byte SENSOR_PIN[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
// TODO: kalau urutan fisik kiri-kanan robot ternyata kebalik dari s0..s7
// di skematik, tinggal balik array ini, semua logika lain otomatis ikut.

// Arah sudah dikonfirmasi dari skematik + tesSensor.ino:
// GARIS (hitam) = ADC TINGGI, BACKGROUND (putih) = ADC RENDAH.
// Jadi TIDAK perlu invert. Tetap disediakan sebagai saklar andai
// nanti ganti jenis sensor / kondisi cahaya yang beda karakter.
#define SENSOR_TERBALIK false

// =====================================================
// PIN TOMBOL - persis dari tesSensor.ino (INPUT_PULLUP, aktif LOW)
// =====================================================
#define BTN_START 2   // tekan sekali -> mulai jalan
#define BTN_STOP  4   // tekan -> stop / balik ke standby
#define BTN_CALIB 12  // tekan -> kalibrasi ulang sensor (~5 detik)

// =====================================================
// LED STRIP WS2812 (8 pixel)
// =====================================================
#define LED_STRIP_PIN 13
#define NUM_LEDS 8
#define LED_BRIGHTNESS 60

// =====================================================
// KECEPATAN MOTOR (PWM 0-255)
// =====================================================
const int BASE_SPEED = 100; // kecepatan dasar maju lurus
const int MAX_SPEED  = 255; // batas atas PWM waktu koreksi

// =====================================================
// PID LINE FOLLOWING
// TODO: tuning di lapangan.
// Naikkan KP pelan2 sampai mulai oscillasi/geleng, baru naikkan
// KD buat meredam. KI biasanya 0 dulu buat line follower simpel.
// =====================================================
float KP = 16;
float KI = 0.0;
float KD = 0.0;

// Bobot posisi tiap sensor: kiri negatif, kanan positif, tengah = 0
const int SENSOR_WEIGHT[8] = {-35, -25, -15, -5, 5, 15, 25, 35};

// Hysteresis anti-flicker (lihat SensorLine.h)
const int HYSTERESIS = 15;

#endif
