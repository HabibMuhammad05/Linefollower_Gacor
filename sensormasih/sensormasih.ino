/*
  PURE LINE FOLLOWER - PID CONTROLLER
  Arduino Nano

  Basis pembacaan sensor dari tesSensor.ino + skematik Array_resistor_8:
  - A0..A7 -> 8 sensor reflektif (LED + photodiode)
  - GARIS (hitam) = ADC TINGGI, BACKGROUND (putih) = ADC RENDAH
  - Tombol di pin 2, 4, 12 (INPUT_PULLUP, aktif LOW)

  Tidak ada servo/gripper di sini, murni line follower buat tuning PID.

  Tombol:
  - BTN_START (D2)  -> mulai jalan
  - BTN_STOP  (D4)  -> stop / balik ke standby
  - BTN_CALIB (D12) -> kalibrasi ulang sensor (~5 detik, gerakkan
                       robot manual di atas garis & background)

  LED WS2812 (8 pixel, D8):
  - Biru kedip   -> standby
  - Kuning chase -> lagi kalibrasi
  - Hijau/merah per-LED -> saat jalan, tiap LED = 1 sensor
  - Merah solid  -> stop
*/

#include "Config.h"
#include "Motor.h"
#include "SensorLine.h"
#include "LedStatus.h"

enum RobotState {
  WAIT_START,
  KALIBRASI,
  LINE_FOLLOW
};

RobotState state = WAIT_START;
float lastError = 0;

bool tombolDitekan(int pin) {
  if (digitalRead(pin) == LOW) {
    delay(30); // debounce
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW) {
        // tunggu sampai dilepas
      }
      return true;
    }
  }
  return false;
}

void cetakHasilKalibrasi() {
  Serial.println(">> Hasil kalibrasi:");
  for (byte i = 0; i < 8; i++) {
    Serial.print("S"); Serial.print(i);
    Serial.print(" min="); Serial.print(sensorMin[i]);
    Serial.print(" max="); Serial.print(sensorMax[i]);
    Serial.print(" range="); Serial.print(sensorMax[i] - sensorMin[i]);
    Serial.print(" th="); Serial.println(sensorThreshold[i]);
  }
}

void setup() {
  Serial.begin(115200);

  motorInit();
  sensorInit();
  ledInit();

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(BTN_CALIB, INPUT_PULLUP);

  Serial.println("===== PURE LINE FOLLOWER (PID) READY =====");
  Serial.println("BTN_START=jalan | BTN_CALIB=kalibrasi | BTN_STOP=stop");
}

void loop() {

  // BTN_STOP selalu dicek duluan, di state manapun
  if (digitalRead(BTN_STOP) == LOW) {
    stopMotor();
    ledStop();
    state = WAIT_START;
    delay(300);
    return;
  }

  switch (state) {

    case WAIT_START: {
      ledStandby();

      if (tombolDitekan(BTN_CALIB)) {
        state = KALIBRASI;
        Serial.println(">> Kalibrasi mulai, gerakkan robot manual di atas garis & background.");
      }

      if (tombolDitekan(BTN_START)) {
        lastError = 0;
        state = LINE_FOLLOW;
        Serial.println(">> START.");
      }
      break;
    }

    case KALIBRASI: {
      unsigned long mulai = millis();
      while (millis() - mulai < 5000) {
        if (digitalRead(BTN_STOP) == LOW) break; // bisa dibatalkan
        sensorUpdate();
        ledKalibrasi();
        delay(10);
      }
      cetakHasilKalibrasi();
      // Peringatan otomatis kalau ada sensor dengan range terlalu sempit
      // (indikasi kurang kena kontras hitam-putih pas kalibrasi, atau hardware)
      for (byte i = 0; i < 8; i++) {
        int range = sensorMax[i] - sensorMin[i];
        if (range < 50) {
          Serial.print("!! WARNING: S"); Serial.print(i);
          Serial.print(" range cuma "); Serial.print(range);
          Serial.println(" -> cek posisi/kondisi sensor ini");
        }
      }
      state = WAIT_START;
      break;
    }

    case LINE_FOLLOW: {
      sensorUpdate();
      ledTampilkanSensor();

      bool lineDetected;
      float posisi = readLinePosition(lineDetected);

      if (!lineDetected) {
        // garis hilang total: cari ke arah error terakhir
        posisi = (lastError >= 0) ? 35 : -35;
      }

      float error = posisi; // target posisi = 0 (tengah)
      float derivative = error - lastError;
      float output = KP * error + KD * derivative;
      lastError = error;

      int pwmKiri  = constrain((int)(BASE_SPEED + output), -MAX_SPEED, MAX_SPEED);
      int pwmKanan = constrain((int)(BASE_SPEED - output), -MAX_SPEED, MAX_SPEED);

      setMotorPWM(pwmKiri, pwmKanan);

      // debug: buka salah satu buat bantu tuning
      // sensorDebugPrint();
      // Serial.print(error); Serial.print("\t"); Serial.println(output);
      break;
    }
  }
}
