void setup() {
  Serial.begin(115200);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
}

void loop() {
  Serial.print(analogRead(A0)); Serial.print("-");
  Serial.print(analogRead(A1)); Serial.print("-");
  Serial.print(analogRead(A2)); Serial.print("-");
  Serial.print(analogRead(A3)); Serial.print("-");
  Serial.print(analogRead(A4)); Serial.print("-");
  Serial.print(analogRead(A5)); Serial.print("-");
  Serial.print(analogRead(A6)); Serial.print("-");
  Serial.print(analogRead(A7)); Serial.print(" = PB: ");
  Serial.print(digitalRead(2)); Serial.print("-");
  Serial.print(digitalRead(4)); Serial.print("-");
  Serial.println(digitalRead(12));
  delay(100);

}
