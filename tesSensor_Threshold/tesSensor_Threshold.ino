/*
  TEST SENSOR LINE 8 CHANNEL
  Arduino Nano
  A0 A1 A2 A3 A4 A5 A6 A7

  Serial Output :
  Analog     : xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx
  Threshold  : xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx
  Digital    : 0 1 1 0 0 1 1 0

  Sistem threshold adaptif:
  - Setiap sensor menyimpan nilai minimum dan maksimum.
  - Threshold = (min + max) / 2.
  - Semakin sering sensor melihat hitam dan putih,
    threshold akan semakin akurat.
*/

const byte sensorPin[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

int value[8];
int minValue[8];
int maxValue[8];
int threshold[8];
bool digitalValue[8];

void setup() {

  Serial.begin(115200);

  for (byte i = 0; i < 8; i++) {

    value[i] = analogRead(sensorPin[i]);

    minValue[i] = value[i];
    maxValue[i] = value[i];
    threshold[i] = value[i];
  }

  Serial.println("===== SENSOR TEST =====");
}

void loop() {

  // ==========================
  // BACA DAN UPDATE KALIBRASI
  // ==========================
  for (byte i = 0; i < 8; i++) {

    value[i] = analogRead(sensorPin[i]);

    if (value[i] < minValue[i])
      minValue[i] = value[i];

    if (value[i] > maxValue[i])
      maxValue[i] = value[i];

    threshold[i] = (minValue[i] + maxValue[i]) / 2;

    // Ubah tanda > menjadi < jika output terbalik
    digitalValue[i] = value[i] > threshold[i];
  }

  // ==========================
  // ANALOG
  // ==========================
  Serial.print("A : ");

  for (byte i = 0; i < 8; i++) {

    Serial.print(value[i]);
    Serial.print("\t");
  }
  
  Serial.println();
  // ==========================
  // THRESHOLD
  // ==========================
  Serial.print("T : ");

  for (byte i = 0; i < 8; i++) {

    Serial.print(threshold[i]);
    Serial.print("\t");
  }

  Serial.println();
  // ==========================
  // DIGITAL
  // ==========================
  Serial.print("D : ");

  for (byte i = 0; i < 8; i++) {

    Serial.print(digitalValue[i]);
    Serial.print("\t");
  }

    Serial.println("\n");

  delay(100);
}
