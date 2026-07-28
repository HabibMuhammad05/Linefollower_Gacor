#ifndef MAPPING_H
#define MAPPING_H

enum TurnAction { ACT_LURUS, ACT_KIRI, ACT_KANAN, ACT_PUTAR, ACT_BERHENTI };

const int           TURN_SPEED     = 180;
const unsigned long TURN_KIRI_MS   = 350;
const unsigned long TURN_KANAN_MS  = 300;
const unsigned long TURN_PUTAR_MS  = 500;
const unsigned long MAJU_MS        = 300;
const unsigned long MUNDUR_MS      = 100;
const unsigned long STOP_SESAAT_MS = 1000;

uint32_t decisionColor(TurnAction action) {
  switch (action) {
    case ACT_KIRI:     return strip.Color(0, 255, 255);
    case ACT_KANAN:    return strip.Color(255, 0, 150);
    case ACT_PUTAR:    return strip.Color(150, 0, 255);
    case ACT_BERHENTI: return strip.Color(255, 0, 0);
    default:           return strip.Color(255, 255, 255);
  }
}

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

void runTurnManeuver(TurnAction kind, unsigned long durationMs) {
  int leftSpd, rightSpd;
  if (kind == ACT_KIRI)       { leftSpd = -TURN_SPEED; rightSpd =  TURN_SPEED; }
  else if (kind == ACT_KANAN) { leftSpd =  TURN_SPEED; rightSpd = -TURN_SPEED; }
  else                        { leftSpd =  TURN_SPEED; rightSpd = -TURN_SPEED; }

  setMotor(leftSpd, rightSpd);
  unsigned long t0 = millis();
  while (millis() - t0 < durationMs) {
    updateDecisionLED(kind);
    if (!digitalRead(BTN_STOP)) break;
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

void stopMotors() {
  stopMotor();
}

void stopSesaat(unsigned long ms = STOP_SESAAT_MS) {
  Serial.println(F("[MAPPING] stopSesaat()"));
  stopMotor();
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    updateDecisionLED(ACT_BERHENTI);
  }
}

void resetLineFollowing() {
  lastError = 0;
  lastErrorSign = 1;
  Serial.println(F("[MAPPING] resetLineFollowing()"));
}

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

void ambilKotak() {
  Serial.println(F("[MAPPING] ambilKotak()"));
  servoTurun();
  servoTutupJepit();
  servoNaik();
}

void lepasKotak() {
  Serial.println(F("[MAPPING] lepasKotak()"));
  servoTurun();
  servoBukaJepit();
  servoNaik();
}

void selesaiRute() {
  Serial.println(F("[MAPPING] selesaiRute() - FINISH -> auto balik IDLE"));
  stopMotor();
  finishStateEnteredAt = millis();
  currentState = ST_FINISH;
}

// ------------------------ EDIT DI SINI ------------------------
void executeMapStep(uint8_t planNum, unsigned int simpangKe, bool &actionDone) {
  switch (planNum) {

    case 0: // jika warna kotak dan gudang sejajar
      switch (simpangKe) {
        case 1: turnLeft(); actionDone = true; break;
        case 2: stopMotors(); ambilKotak(); turnAround(); resetLineFollowing(); actionDone = true; break;
        case 3: actionDone = true; break;
        case 4: stopMotors(); lepasKotak(); turnAround(); actionDone = true; break;
        case 5: turnRight(); actionDone = true; break;
        case 6: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 1: // jika warna kotak diatas 1
      switch (simpangKe) {
        case 1: turnLeft(); actionDone = true; break;
        case 2: stopMotors(); ambilKotak(); turnAround(); resetLineFollowing(); actionDone = true; break;
        case 3: turnLeft(); actionDone = true; break;
        case 4: turnRight(); actionDone = true; break;
        case 5: stopMotors(); lepasKotak(); turnAround(); actionDone = true; break;
        case 6: turnLeft(); actionDone = true; break;
        case 7: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 2: // jika warna kotak diatas 2
      switch (simpangKe) {
        case 1: turnLeft(); actionDone = true; break;
        case 2: stopMotors(); ambilKotak(); turnAround(); resetLineFollowing(); actionDone = true; break;
        case 3: turnLeft(); actionDone = true; break;
        case 4: actionDone = true; break;
        case 5: turnRight(); actionDone = true; break;
        case 6: stopMotors(); lepasKotak(); turnAround(); actionDone = true; break;
        case 7: turnLeft(); actionDone = true; break;
        case 8: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 3: // jika warna kotak dibawah 1
      switch (simpangKe) {
        case 1: turnLeft(); actionDone = true; break;
        case 2: stopMotors(); ambilKotak(); turnAround(); resetLineFollowing(); actionDone = true; break;
        case 3: turnRight(); actionDone = true; break;
        case 4: turnLeft(); actionDone = true; break;
        case 5: stopMotors(); lepasKotak(); turnAround(); actionDone = true; break;
        case 6: turnRight(); actionDone = true; break;
        case 7: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 4: // jika warna kotak dibawah 2
      switch (simpangKe) {
        case 1: turnLeft(); actionDone = true; break;
        case 2: stopMotors(); ambilKotak(); turnAround(); resetLineFollowing(); actionDone = true; break;
        case 3: turnRight(); actionDone = true; break;
        case 4: actionDone = true; break;
        case 5: turnLeft(); actionDone = true; break;
        case 6: stopMotors(); lepasKotak(); turnAround(); actionDone = true; break;
        case 7: turnRight(); actionDone = true; break;
        case 8: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 5: // kejar retry 1
      switch (simpangKe) {
        case 1: actionDone = true; break;
        case 2: turnLeft(); actionDone = true; break;
        case 3: actionDone = true; break;
        case 4: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 6: // loncat 1 retry
      switch (simpangKe) {
        case 1: actionDone = true; break;
        case 2: turnLeft(); actionDone = true; break;
        case 3: actionDone = true; break;
        case 4: actionDone = true; break;
        case 5: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;
      
    case 8: // loncat 2 retry
      switch (simpangKe) {
        case 1: actionDone = true; break;
        case 2: actionDone = true; break;
        case 3: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }

    default: break;
  }
}
// ----------------------------------------------------------------

//    case 0: // dari start ke finish langsung
//      switch (simpangKe) {
//        case 1: actionDone = true; break;
//        case 2: turnLeft();  actionDone = true; break;
//        case 3: actionDone = true; break;
//        case 4: actionDone = true; break;
//        case 5: actionDone = true; break;
//        case 6: actionDone = true; break;
//        case 7: maju(); selesaiRute(); actionDone = true; break;
//        default: break;
//      }
//      break;
#endif
