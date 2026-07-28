#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

/*
  KONFIGURASI SERVO (gripper)
  ====================================================================
  Pin & sudut diambil dari tesServo.ino (sudah pernah dites & jalan).
  Kalau mekanik gripper berubah, tinggal ubah sudutnya di sini - ga
  perlu bongkar file lain.
*/

// pin servo (dari tesServo.ino)
const uint8_t SERVO_JEPIT_PIN  = 9;   // servo "jepit" -> buka/tutup
const uint8_t SERVO_ANGKAT_PIN = 10;  // servo "angkat" -> naik/turun

// sudut (derajat), dari tesServo.ino:
const int JEPIT_BUKA_ANGLE   = 0;    // jepit terbuka penuh
const int JEPIT_TUTUP_ANGLE  = 65;   // jepit tertutup
const int ANGKAT_NAIK_ANGLE  = 65;   // gripper naik / posisi atas
const int ANGKAT_TURUN_ANGLE = 180;  // gripper turun penuh

// delay per 1 derajat gerak servo - SENGAJA pelan (gerakan halus) biar
// ga ada lonjakan arus mendadak ke step-down yang dipakai bareng Arduino.
// Naikin angka ini kalau masih kerasa "nyentak", turunin kalau kelamaan.
const unsigned long SERVO_STEP_DELAY_MS = 15;

#endif
