#ifndef SERVO_GRIPPER_H
#define SERVO_GRIPPER_H
#include <Servo.h>
#include "Config.h"
#include "Motor.h"

Servo jepit;
Servo angkat;

void servoInit() {
  jepit.attach(PIN_JEPIT);
  angkat.attach(PIN_ANGKAT);

  jepit.write(JEPIT_TUTUP);
  angkat.write(ANGKAT_NAIK);
  delay(500);
}

void gerakServo(Servo &servo, int awal, int akhir) {
  if (awal < akhir) {
    for (int pos = awal; pos <= akhir; pos++) {
      servo.write(pos);
      delay(DELAY_SERVO);
    }
  } else {
    for (int pos = awal; pos >= akhir; pos--) {
      servo.write(pos);
      delay(DELAY_SERVO);
    }
  }
}

void bukaJepit()   { gerakServo(jepit, JEPIT_TUTUP, JEPIT_BUKA); }
void tutupJepit()  { gerakServo(jepit, JEPIT_BUKA, JEPIT_TUTUP); }
void angkatNaik()  { gerakServo(angkat, ANGKAT_TURUN, ANGKAT_NAIK); }
void angkatTurun() { gerakServo(angkat, ANGKAT_NAIK, ANGKAT_TURUN); }

// Robot berhenti total selama proses ambil/simpan (blocking, disengaja -
// lihat catatan "Delay Blocking" di project summary).
void ambilBarang() {
  stopMotor();
  bukaJepit();
  angkatTurun();
  tutupJepit();
  angkatNaik();
}

void simpanBarang() {
  stopMotor();
  angkatTurun();
  bukaJepit();
  angkatNaik();
  tutupJepit();
}

#endif
