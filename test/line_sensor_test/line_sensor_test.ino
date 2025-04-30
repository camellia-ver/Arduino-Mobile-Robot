#define LINE_SENSOR_LEFT A6
#define LINE_SENSOR_RIGHT A7

void setup() {
  Serial.begin(9600);
  pinMode(LINE_SENSOR_LEFT, INPUT);
  pinMode(LINE_SENSOR_RIGHT, INPUT);
}

void loop() {
  int leftValue = analogRead(LINE_SENSOR_LEFT);
  int rightValue = analogRead(LINE_SENSOR_RIGHT);

  Serial.print("Left Sensor (A6): ");
  Serial.print(leftValue);
  Serial.print(" | Right Sensor (A7): ");
  Serial.println(rightValue);

  delay(200);  // 0.2초 간격으로 출력
}
