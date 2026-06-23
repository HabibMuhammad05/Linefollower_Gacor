/*
  PURE LINE FOLLOWER - PID TUNING VERSION
  Arduino Nano

  Tidak ada servo/gripper/path di sini sama sekali, biar fokus
  tuning PID & respons robot doang.

  Tombol:
  - BTN_START (D2)  -> mulai jalan
  - BTN_STOP  (D4)  -> stop / balik ke standby
  - BTN_CALIB (D12) -> kalibrasi ulang sensor (~5 detik, gerakkan
                       robot manual di atas garis & background)

  LED WS2812 (8 pixel, D8):
  - Biru kedip   -> standby
  - Kuning chase -> lagi kalibrasi
  - Hijau/merah per-LED -> saat jalan, tiap LED mewakili 1 sensor
    (hijau = kena garis) -> alat bantu visual buat tuning
  - Merah solid  -> stop / garis hilang total
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

void setup() {
  Serial.begin(115200);

  motorInit();
  sensorInit();
  ledInit();

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(BTN_CALIB, INPUT_PULLUP);

  Serial.println("===== PURE LINE FOLLOWER READY =====");
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
      Serial.println(">> Kalibrasi selesai.");
      // cetak hasil kalibrasi buat sanity check di Serial Monitor
      for (byte i = 0; i < 8; i++) {
        Serial.print("S"); Serial.print(i);
        Serial.print(" min="); Serial.print(sensorMin[i]);
        Serial.print(" max="); Serial.print(sensorMax[i]);
        Serial.print(" th="); Serial.println(sensorThreshold[i]);
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

      // debug opsional, buka Serial Plotter buat lihat kurva error/output
      // Serial.print(error); Serial.print("\t"); Serial.println(output);
      break;
    }
  }
}
