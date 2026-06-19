 /*
  ROBOT LINE FOLLOWER TRANSPORTER
  Arduino Nano

  Integrasi dari:
  - tesMotor.ino          -> Motor.h
  - tesSensor_Threshold.ino -> SensorLine.h
  - tesServo_nonblocking.ino -> ServoGripper.h

  State machine dikendalikan oleh array PATH[] di Config.h,
  jadi untuk mengubah rute robot cukup edit PATH[] tanpa
  menyentuh logika di file ini.
*/

#include "Config.h"
#include "Motor.h"
#include "SensorLine.h"
#include "ServoGripper.h"

enum RobotState {
  INIT,
  WAIT_START,
  KALIBRASI,
  LINE_FOLLOW,
  EKSEKUSI_AKSI,
  SELESAI_STATE
};

RobotState state = INIT;
byte pathIndex = 0;
float lastError = 0;

unsigned long ledTimer = 0;
bool ledState = false;

// ---------- Debounce tombol sederhana ----------
bool tombolDitekan(int pin) {
  if (digitalRead(pin) == LOW) {
    delay(30); // debounce
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW) {
        // tunggu sampai dilepas, supaya tidak ke-trigger berkali-kali
      }
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);

  motorInit();
  servoInit();
  sensorInit();

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(BTN_CALIB, INPUT_PULLUP);
  pinMode(LED_INDICATOR, OUTPUT);

  Serial.println("===== ROBOT TRANSPORTER READY =====");
  Serial.println("BTN_START = mulai | BTN_CALIB = kalibrasi sensor | BTN_STOP = stop darurat");

  state = WAIT_START;
}

void loop() {

  // BTN_STOP selalu dicek duluan, di state manapun, sebagai emergency stop
  if (digitalRead(BTN_STOP) == LOW) {
    stopMotor();
    state = WAIT_START;
    Serial.println(">> STOP ditekan, robot dihentikan.");
    delay(300);
    return;
  }

  switch (state) {

    case WAIT_START: {
      // LED berkedip pelan menandakan robot standby
      if (millis() - ledTimer > 500) {
        ledTimer = millis();
        ledState = !ledState;
        digitalWrite(LED_INDICATOR, ledState);
      }

      if (tombolDitekan(BTN_CALIB)) {
        state = KALIBRASI;
        Serial.println(">> Mode kalibrasi sensor. Gerakkan robot manual di atas garis & background beberapa detik.");
      }

      if (tombolDitekan(BTN_START)) {
        pathIndex = 0;
        lastError = 0;
        state = LINE_FOLLOW;
        digitalWrite(LED_INDICATOR, HIGH);
        Serial.println(">> START. Robot mulai jalan.");
      }
      break;
    }

    case KALIBRASI: {
      // Kalibrasi berjalan ~5 detik, LED nyala cepat sebagai penanda
      unsigned long mulai = millis();
      while (millis() - mulai < 5000) {
        if (digitalRead(BTN_STOP) == LOW) break; // bisa dibatalkan
        sensorUpdate();
        digitalWrite(LED_INDICATOR, (millis() / 100) % 2);
        delay(20);
      }
      digitalWrite(LED_INDICATOR, LOW);
      Serial.println(">> Kalibrasi selesai.");
      state = WAIT_START;
      break;
    }

    case LINE_FOLLOW: {
      sensorUpdate();

      if (isIntersection()) {
        stopMotor();
        state = EKSEKUSI_AKSI;
        break;
      }

      bool lineDetected;
      float posisi = readLinePosition(lineDetected);

      if (!lineDetected) {
        // Garis hilang: cari lagi ke arah error terakhir
        posisi = (lastError >= 0) ? 35 : -35;
      }

      float error = posisi; // target posisi = 0 (tengah)
      float derivative = error - lastError;
      float output = KP * error + KD * derivative;
      lastError = error;

      int pwmKiri  = constrain((int)(BASE_SPEED + output), -MAX_SPEED, MAX_SPEED);
      int pwmKanan = constrain((int)(BASE_SPEED - output), -MAX_SPEED, MAX_SPEED);

      setMotorPWM(pwmKiri, pwmKanan);
      break;
    }

    case EKSEKUSI_AKSI: {
      if (pathIndex >= PATH_LENGTH) {
        state = SELESAI_STATE;
        break;
      }

      Aksi aksi = PATH[pathIndex];
      Serial.print(">> Eksekusi aksi index ");
      Serial.print(pathIndex);
      Serial.print(" : ");

      switch (aksi) {
        case LURUS:
          Serial.println("LURUS");
          // lewati persimpangan sebentar biar tidak langsung dianggap intersection lagi
          maju(BASE_SPEED);
          delay(300);
          stopMotor();
          break;

        case BELOK_KIRI:
        case BELOK_KANAN:
        case PUTAR_BALIK: {
          Serial.println(aksi == BELOK_KIRI ? "BELOK_KIRI" : (aksi == BELOK_KANAN ? "BELOK_KANAN" : "PUTAR_BALIK"));
          unsigned long mulai = millis();
          // keluar dulu dari zona persimpangan sesaat biar sensor tengah tidak langsung true
          while (millis() - mulai < 300) {
            if (aksi == BELOK_KANAN) putarKanan();
            else putarKiri(); // BELOK_KIRI dan PUTAR_BALIK sama arah dasarnya, cukup ganti durasi kalau perlu
          }

          mulai = millis();
          while (millis() - mulai < BELOK_TIMEOUT_MS) {
            sensorUpdate();
            if (garisDiTengah()) break;
            if (aksi == BELOK_KANAN) putarKanan();
            else putarKiri();
          }
          stopMotor();
          break;
        }

        case AMBIL_BARANG:
          Serial.println("AMBIL_BARANG");
          ambilBarang();
          break;

        case ANTAR_BARANG:
          Serial.println("ANTAR_BARANG");
          simpanBarang();
          break;

        case SELESAI:
          Serial.println("SELESAI");
          state = SELESAI_STATE;
          return;
      }

      pathIndex++;
      state = LINE_FOLLOW;
      break;
    }

    case SELESAI_STATE: {
      stopMotor();
      digitalWrite(LED_INDICATOR, HIGH); // LED nyala tetap = robot selesai tugas
      Serial.println(">> RUTE SELESAI. Tekan BTN_START untuk ulang.");
      while (state == SELESAI_STATE) {
        if (tombolDitekan(BTN_START)) {
          pathIndex = 0;
          lastError = 0;
          state = LINE_FOLLOW;
          Serial.println(">> Restart rute.");
        }
        if (digitalRead(BTN_STOP) == LOW) break;
      }
      break;
    }

    default:
      state = WAIT_START;
      break;
  }
}
