/*******************************************************
 * 프로젝트명: lr 센서 읽기
 * 기능: lr 센서 읽기
 * 작성자: 조영란
 * 날짜: 2025-05-01
 *******************************************************/
// ----------------------------
// [1] 상수 및 핀 정의
// ----------------------------
#define pinLT1  A6  // Left IR sensor (sensor 1)
#define pinLT2  A7  // Right IR sensor (sensor 2)

// ----------------------------
// [2] setup()
// ----------------------------
void setup() {
  Serial.begin(9600);
}

// ----------------------------
// [3] loop()
// ----------------------------
void loop() {
  int value1 = analogRead(pinLT1);  // IR1 값 읽기
  int value2 = analogRead(pinLT2);  // IR2 값 읽기

  if (value1 < 450) {
    Serial.print("WHITE, ");
  } else {
    Serial.print("BLACK, ");
  }

  if (value2 < 450) {
    Serial.print("WHITE");
  } else {
    Serial.print("BLACK");
  }

  delay(100);
}