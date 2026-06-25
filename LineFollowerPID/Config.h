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
// PIN SENSOR LINE (8 channel)
// =====================================================
const byte SENSOR_PIN[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
// TODO: pastikan index 0 = paling kiri robot, index 7 = paling kanan.

// =====================================================
// PIN TOMBOL
// =====================================================
#define BTN_START 2   // tekan sekali -> mulai jalan
#define BTN_STOP  4   // tekan -> stop / pause
#define BTN_CALIB 12  // tekan -> kalibrasi ulang sensor (~5 detik)

// =====================================================
// LED STRIP WS2812 (8 pixel)
// =====================================================
#define LED_STRIP_PIN 13
#define NUM_LEDS 8
#define LED_BRIGHTNESS 60 // 0-255, turunkan kalau terlalu silau / narik arus banyak

// =====================================================
// KECEPATAN MOTOR (PWM 0-255)
// TODO: ini titik utama yang perlu di-tuning bareng PID
// =====================================================
const int BASE_SPEED = 100; // kecepatan dasar maju lurus
const int MAX_SPEED  = 255; // batas atas PWM waktu koreksi belok tajam

// =====================================================
// PID LINE FOLLOWING
// TODO: tuning di lapangan.
// Cara umum: naikkan KP pelan2 sampai mulai oscillasi/geleng,
// baru naikkan KD untuk meredam oscillasi itu. KI biasanya
// dibiarkan 0 dulu untuk line follower simpel.
// =====================================================
float KP = 16;
float KI = 0.0;
float KD = 0.0;

// Bobot posisi tiap sensor: kiri negatif, kanan positif, tengah = 0
const int SENSOR_WEIGHT[8] = {-35, -25, -15, -5, 5, 15, 25, 35};

#endif
