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

// Panggil berulang (misal selama beberapa detik, robot digerakkan
// manual di atas garis & background) untuk memperluas rentang min-max
// sebelum mulai jalan. Dipakai oleh mode BTN_CALIB.
void sensorUpdate() {
  for (byte i = 0; i < 8; i++) {
    sensorValue[i] = analogRead(SENSOR_PIN[i]);

    if (sensorValue[i] < sensorMin[i]) sensorMin[i] = sensorValue[i];
    if (sensorValue[i] > sensorMax[i]) sensorMax[i] = sensorValue[i];

    sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;

    // TODO: balik jadi '<' kalau digital hasilnya kebalik dari yang diharapkan
    sensorDigital[i] = sensorValue[i] > sensorThreshold[i];
  }
}

// Normalisasi ke 0-1000 berdasarkan rentang kalibrasi tiap sensor.
// Asumsi: makin besar nilai analog = makin dekat ke garis (hitam).
// TODO: sesuaikan arah kalau ternyata terbalik di sensor fisik.
int sensorNormalize(byte i) {
  if (sensorMax[i] == sensorMin[i]) return 0;
  long norm = (long)(sensorValue[i] - sensorMin[i]) * 1000L / (sensorMax[i] - sensorMin[i]);
  return constrain(norm, 0, 1000);
}

// Posisi garis hasil weighted average, kira-kira di rentang -35..35 (lihat SENSOR_WEIGHT).
// lineDetected diisi false kalau semua sensor tidak melihat apa-apa (garis hilang total).
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

// Deteksi persimpangan: banyak sensor aktif bersamaan (garis melebar).
// TODO: sesuaikan INTERSECTION_SENSOR_COUNT di Config.h dengan bentuk
// persimpangan fisik (T-junction vs cross vs sekadar garis lebar).
bool isIntersection() {
  byte count = 0;
  for (byte i = 0; i < 8; i++) {
    if (sensorDigital[i]) count++;
  }
  return count >= INTERSECTION_SENSOR_COUNT;
}

// Dipakai saat proses belok: apakah sensor tengah sudah menemukan garis lagi
bool garisDiTengah() {
  return sensorDigital[3] || sensorDigital[4];
}

#endif
