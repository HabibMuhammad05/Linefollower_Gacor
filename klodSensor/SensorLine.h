#ifndef SENSOR_LINE_H
#define SENSOR_LINE_H
#include "Config.h"

int  sensorValue[8];
int  sensorMin[8];
int  sensorMax[8];
int  sensorThreshold[8];
bool sensorDigital[8];

void sensorInit() {
  for (byte i = 0; i < 8; i++) {
    sensorValue[i]     = analogRead(SENSOR_PIN[i]);
    sensorMin[i]       = sensorValue[i];
    sensorMax[i]       = sensorValue[i];
    sensorThreshold[i] = sensorValue[i];
  }
}

// Update pembacaan + kalibrasi min-max berjalan terus tiap dipanggil
// Hysteresis: sensor yang lagi ON butuh turun lebih jauh dari threshold buat
// jadi OFF (dan sebaliknya), biar gak flicker kalau nilainya pas nempel di
// threshold. Naikkan kalau masih flicker, turunkan kalau responsnya lambat.
const int HYSTERESIS = 15;

void sensorUpdate() {
  for (byte i = 0; i < 8; i++) {
    sensorValue[i] = analogRead(SENSOR_PIN[i]);

    if (sensorValue[i] < sensorMin[i]) sensorMin[i] = sensorValue[i];
    if (sensorValue[i] > sensorMax[i]) sensorMax[i] = sensorValue[i];

    sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;

    bool nyalaJikaKecil = SENSOR_TERBALIK; // true: garis = analog rendah

    if (nyalaJikaKecil) {
      if (sensorDigital[i]) sensorDigital[i] = sensorValue[i] < sensorThreshold[i] + HYSTERESIS;
      else                  sensorDigital[i] = sensorValue[i] < sensorThreshold[i] - HYSTERESIS;
    } else {
      if (sensorDigital[i]) sensorDigital[i] = sensorValue[i] > sensorThreshold[i] - HYSTERESIS;
      else                  sensorDigital[i] = sensorValue[i] > sensorThreshold[i] + HYSTERESIS;
    }
  }
}

// Normalisasi ke 0-1000: hasil 1000 = paling dekat garis, 0 = background.
// Arah dikontrol oleh SENSOR_TERBALIK di Config.h.
int sensorNormalize(byte i) {
  if (sensorMax[i] == sensorMin[i]) return 0;
  long norm;
  #if SENSOR_TERBALIK
    norm = (long)(sensorMax[i] - sensorValue[i]) * 1000L / (sensorMax[i] - sensorMin[i]);
  #else
    norm = (long)(sensorValue[i] - sensorMin[i]) * 1000L / (sensorMax[i] - sensorMin[i]);
  #endif
  return constrain(norm, 0, 1000);
}

// Posisi garis hasil weighted average, kira-kira di rentang -35..35.
// lineDetected = false kalau semua sensor tidak melihat apa-apa (garis hilang total).
float readLinePosition(bool &lineDetected) {
  long weightedSum = 0;
  long totalWeight  = 0;

  for (byte i = 0; i < 8; i++) {
    int norm = sensorNormalize(i);
    weightedSum += (long)norm * SENSOR_WEIGHT[i];
    totalWeight  += norm;
  }

  if (totalWeight == 0) {
    lineDetected = false;
    return 0;
  }

  lineDetected = true;
  return (float)weightedSum / (float)totalWeight;
}

// Panggil ini di Serial Monitor / Serial Plotter buat lihat kondisi tiap
// sensor secara live: raw analog, min/max hasil kalibrasi, threshold, digital.
// Sangat membantu buat nemuin sensor yang bermasalah (kayak kasus 0,3,7).
void sensorDebugPrint() {
  for (byte i = 0; i < 8; i++) {
    Serial.print("S"); Serial.print(i);
    Serial.print("[raw="); Serial.print(sensorValue[i]);
    Serial.print(" min="); Serial.print(sensorMin[i]);
    Serial.print(" max="); Serial.print(sensorMax[i]);
    Serial.print(" th="); Serial.print(sensorThreshold[i]);
    Serial.print(" d="); Serial.print(sensorDigital[i]);
    Serial.print("] ");
  }
  Serial.println();
}

#endif
