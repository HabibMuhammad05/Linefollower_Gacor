uint8_t IN2 = 11;
uint8_t IN1 = 6;
uint8_t IN4 = 5;
uint8_t IN3 = 3;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {

  // =====================
  // MAJU PWM 200
  // =====================
  analogWrite(IN1, 200);
  digitalWrite(IN2, LOW);
  analogWrite(IN3, 200);
  digitalWrite(IN4, LOW);
  delay(1000);

  // =====================
  // STOP
  // =====================
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  
  digitalWrite(IN4, LOW);
  delay(1000);

  // =====================
  // MUNDUR PWM 200
  // =====================
  digitalWrite(IN1, LOW);
  analogWrite(IN2, 200);
  digitalWrite(IN3, LOW);
  analogWrite(IN4, 200);
  delay(1000);

  // =====================
  // STOP
  // =====================
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(1000);

  // =====================
  // BELOK KANAN
  // Kiri cepat, kanan pelan
  // =====================
  analogWrite(IN1, 200);
  digitalWrite(IN2, LOW);
  analogWrite(IN3, 10);
  digitalWrite(IN4, LOW);
  delay(1000);

  // =====================
  // BELOK KIRI
  // Kiri pelan, kanan cepat
  // =====================
  analogWrite(IN1, 10);
  digitalWrite(IN2, LOW);
  analogWrite(IN3, 200);
  digitalWrite(IN4, LOW);
  delay(1000);

  // =====================
  // STOP
  // =====================
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(3000);
}
