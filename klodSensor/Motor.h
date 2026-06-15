#ifndef MOTOR_H
#define MOTOR_H
#include "Config.h"

void motorInit() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

// pwmKiri / pwmKanan: -255..255, negatif = mundur, 0 = stop
void setMotorPWM(int pwmKiri, int pwmKanan) {
  pwmKiri  = constrain(pwmKiri, -255, 255);
  pwmKanan = constrain(pwmKanan, -255, 255);

  if (pwmKiri >= 0) {
    analogWrite(IN1, pwmKiri);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, -pwmKiri);
  }

  if (pwmKanan >= 0) {
    analogWrite(IN3, pwmKanan);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    analogWrite(IN4, -pwmKanan);
  }
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void maju(int speed = BASE_SPEED)   { setMotorPWM(speed, speed); }
void mundur(int speed = BASE_SPEED) { setMotorPWM(-speed, -speed); }

// Belok halus (masih maju, salah satu roda melambat) - dipakai saat line follow biasa
void belokKiri(int speed = BASE_SPEED)  { setMotorPWM(speed * 0.2, speed); }
void belokKanan(int speed = BASE_SPEED) { setMotorPWM(speed, speed * 0.2); }

// Putar di tempat (satu roda maju, satu roda mundur) - dipakai di persimpangan / putar balik
void putarKiri(int speed = ROTATE_SPEED)  { setMotorPWM(-speed, speed); }
void putarKanan(int speed = ROTATE_SPEED) { setMotorPWM(speed, -speed); }

#endif
