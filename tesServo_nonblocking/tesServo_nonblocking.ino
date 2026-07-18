#include <Servo.h>

Servo jepit;
Servo angkat;

//=========================
// Sudut Servo
//=========================
const int JEPIT_BUKA  = 0;
const int JEPIT_TUTUP = 65;

const int ANGKAT_NAIK  = 65;
const int ANGKAT_TURUN = 180;

const int DELAY_SERVO = 15;

//=========================
// Prototype
//=========================
void gerakServo(Servo &servo, int awal, int akhir);
void bukaJepit();
void tutupJepit();
void angkatNaik();
void angkatTurun();

void ambilBarang();
void simpanBarang();

void setup() {

  jepit.attach(9);
  angkat.attach(10);

  // Posisi awal
  jepit.write(JEPIT_TUTUP);
  angkat.write(ANGKAT_NAIK);

  delay(500);
}

void loop() {

  ambilBarang();
  delay(2000);

  simpanBarang();
  delay(2000);
}

//====================================================

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

//====================================================

void bukaJepit() {
  gerakServo(jepit, JEPIT_TUTUP, JEPIT_BUKA);
}

void tutupJepit() {
  gerakServo(jepit, JEPIT_BUKA, JEPIT_TUTUP);
}

void angkatNaik() {
  gerakServo(angkat, ANGKAT_TURUN, ANGKAT_NAIK);
}

void angkatTurun() {
  gerakServo(angkat, ANGKAT_NAIK, ANGKAT_TURUN);
}

//====================================================

void ambilBarang() {

  bukaJepit();

  angkatTurun();

  tutupJepit();

  angkatNaik();

}

//====================================================

void simpanBarang() {

  angkatTurun();

  bukaJepit();

  angkatNaik();

  tutupJepit();

}
