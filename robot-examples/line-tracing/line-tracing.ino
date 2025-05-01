/*******************************************************
 * 프로젝트명: 라인 트레이싱 (모니터링 모드)
 * 기능: 센서 값 모니터링 및 동작 출력
 * 작성자: 조영란
 * 날짜: 2025-05-01
 *******************************************************/

// ----------------------------
// [1] 상수 및 핀 정의
// ----------------------------
#define LEFT_SENSOR_PIN  A6  // 왼쪽 IR 센서 연결 핀
#define RIGHT_SENSOR_PIN A7  // 오른쪽 IR 센서 연결 핀
#define BUTTON_PIN       2   // 버튼 핀

// 색상 판단: White=[0..410] .. 560 .. [710..1023]=Black
#define SENSOR_THRESHOLD_ADJUSTMENT 60  // 센서 측정 조정값
#define WHITE_THRESHOLD  470 + SENSOR_THRESHOLD_ADJUSTMENT // 흰색 판단 최대값
#define BLACK_THRESHOLD  770 + SENSOR_THRESHOLD_ADJUSTMENT // 검은색 판단 최소값

// ----------------------------
// [2] 전역 변수 및 객체
// ----------------------------
int motorSpeed = 100;   // 기본 모터 속도
bool lastButtonState = HIGH;  // 버튼의 이전 상태 저장

float leftMotorSpeedAdjustment = 1.00;   // 왼쪽 모터 보정 비율
float rightMotorSpeedAdjustment = 1.00;  // 오른쪽 모터 보정 비율

// ----------------------------
// [3] 초기화 함수
// ----------------------------
void setup() {
  Serial.begin(9600);  // 시리얼 모니터 초기화
  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // 버튼 핀 설정
}

// ----------------------------
// [4] 센서 상태 출력 함수
// ----------------------------
void printSensorStatus(int sensorValue, const char* sensorName) {
  if (sensorValue > BLACK_THRESHOLD) {
    Serial.print(sensorName);
    Serial.println(": BLACK");
  } else if (sensorValue < WHITE_THRESHOLD) {
    Serial.print(sensorName);
    Serial.println(": WHITE");
  } else {
    Serial.print(sensorName);
    Serial.println(": MID");
  }
}

// ----------------------------
// [5] 로봇 동작 판단 함수
// ----------------------------
void determineAction(int leftSensorValue, int rightSensorValue) {
  if (leftSensorValue > BLACK_THRESHOLD && rightSensorValue > BLACK_THRESHOLD) {
    Serial.println("Action: STOP");  // 양쪽 센서가 검정색이면 정지
  } else if (leftSensorValue > BLACK_THRESHOLD && rightSensorValue < WHITE_THRESHOLD) {
    Serial.println("Action: LEFT TURN");  // 왼쪽만 검정색이면 좌회전
  } else if (rightSensorValue > BLACK_THRESHOLD && leftSensorValue < WHITE_THRESHOLD) {
    Serial.println("Action: RIGHT TURN");  // 오른쪽만 검정색이면 우회전
  } else if (leftSensorValue < WHITE_THRESHOLD && rightSensorValue < WHITE_THRESHOLD) {
    Serial.println("Action: FORWARD");  // 양쪽 센서가 흰색이면 전진
  }
}

// ----------------------------
// [6] 센서 값 모니터링 및 동작 출력
// ----------------------------
void monitorSensorValues() {
  int leftSensorValue = analogRead(LEFT_SENSOR_PIN);  // 왼쪽 센서 값 읽기
  int rightSensorValue = analogRead(RIGHT_SENSOR_PIN);  // 오른쪽 센서 값 읽기

  // 센서 값 출력
  Serial.print("Left Sensor: ");
  Serial.print(leftSensorValue);
  Serial.print("\tRight Sensor: ");
  Serial.println(rightSensorValue);

  // 센서 상태 출력 (검정/흰색 판단)
  printSensorStatus(leftSensorValue, "Left Sensor");
  printSensorStatus(rightSensorValue, "Right Sensor");

  // 동작 판단
  determineAction(leftSensorValue, rightSensorValue);

  Serial.println("----------------------------");
}

// ----------------------------
// [7] 메인 루프
// ----------------------------
void loop() {
  monitorSensorValues();  // 센서 값 모니터링

  delay(500);  // 0.5초 대기 후 다시 모니터링
}
