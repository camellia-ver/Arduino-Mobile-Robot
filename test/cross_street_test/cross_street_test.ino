// ===== 모터 핀 설정 =====
#define LEFT_MOTOR_DIR_PIN 7
#define LEFT_MOTOR_PWM_PIN 5
#define RIGHT_MOTOR_DIR_PIN 8
#define RIGHT_MOTOR_PWM_PIN 6

#define FORWARD HIGH
#define BACKWARD LOW

// ===== 버튼 핀 설정 =====
#define pinButton A3

// ===== 라인트레이싱 설정 =====
const int leftIRSensorPin = A0;
const int rightIRSensorPin = A1;

const int LINE_SENSOR_THRESHOLD = 512;  // IR센서 임계값
const int FOLLOW_LINE_DURATION = 10000; // 10초 동안 라인트레이싱

// ===== 모터 속도 및 회전 시간 =====
const int ROBOT_SPEED = 100;
const int TURN_DELAY_90 = 400;
const int TURN_DELAY_180 = 800;

bool IsDriving = false;        // 주행 상태
bool crossDetected = false;    // 교차로 감지 여부 플래그

// ====== 모터 제어 함수 ======
void controlMotors(int leftDirection, int leftPower, int rightDirection, int rightPower) {
  digitalWrite(LEFT_MOTOR_DIR_PIN, leftDirection);
  analogWrite(LEFT_MOTOR_PWM_PIN, leftPower);
  
  digitalWrite(RIGHT_MOTOR_DIR_PIN, rightDirection);
  analogWrite(RIGHT_MOTOR_PWM_PIN, rightPower);
}

void moveRobotForward(int power) {
  controlMotors(FORWARD, power, FORWARD, power);
}

void turnRobotLeftInPlace(int power) {
  controlMotors(BACKWARD, power, FORWARD, power);
}

void turnRobotRightInPlace(int power) {
  controlMotors(FORWARD, power, BACKWARD, power);
}

void stopMotors() {
  analogWrite(LEFT_MOTOR_PWM_PIN, 0);
  analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
}

void turnRight90() {
  turnRobotRightInPlace(ROBOT_SPEED);
  delay(TURN_DELAY_90);
  stopMotors();
}

void turnLeft90() {
  turnRobotLeftInPlace(ROBOT_SPEED);
  delay(TURN_DELAY_90);
  stopMotors();
}

void turnAround180() {
  turnRobotRightInPlace(ROBOT_SPEED);
  delay(TURN_DELAY_180);
  stopMotors();
}

// ====== 센서 읽기 함수 ======
int readSensorAverage(int pin) {
  int total = 0;
  for (int i = 0; i < 5; i++) {
    total += analogRead(pin);
  }
  return total / 5;
}

// ====== 라인트레이싱 함수 (교차로 좌회전 포함) ======
void followLine(int speed, int duration_ms) {
  unsigned long startTime = millis();

  while (millis() - startTime < duration_ms) {
    int leftValue = readSensorAverage(leftIRSensorPin);
    int rightValue = readSensorAverage(rightIRSensorPin);

    // ===== 교차로 감지 (두 센서 모두 라인 위 감지) =====
    if (leftValue > LINE_SENSOR_THRESHOLD && rightValue > LINE_SENSOR_THRESHOLD) {
      if (!crossDetected) {  // 교차로 처음 발견했을 때만
        Serial.println("교차로 감지 → 좌회전");

        moveRobotForward(speed); // 약간 전진
        delay(200);

        turnLeft90();            // 좌회전
        crossDetected = true;    // 이미 좌회전 했다고 표시
      }
    }
    // ===== 직진 =====
    else if (leftValue < LINE_SENSOR_THRESHOLD && rightValue < LINE_SENSOR_THRESHOLD) {
      Serial.println("직진");
      moveRobotForward(speed);
      crossDetected = false; // 다음 교차로 인식 가능하도록 초기화
    }
    // ===== 왼쪽으로 살짝 이동 =====
    else if (leftValue < LINE_SENSOR_THRESHOLD && rightValue > LINE_SENSOR_THRESHOLD) {
      Serial.println("좌측 조정");
      turnRobotLeftInPlace(speed);
      crossDetected = false;
    }
    // ===== 오른쪽으로 살짝 이동 =====
    else if (rightValue < LINE_SENSOR_THRESHOLD && leftValue > LINE_SENSOR_THRESHOLD) {
      Serial.println("우측 조정");
      turnRobotRightInPlace(speed);
      crossDetected = false;
    }

    delay(5); // 센서 읽기 주기
  }
}

// ====== 셋업 ======
void setup() {
  Serial.begin(115200);

  // 모터 핀 출력 설정
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);

  // 버튼 핀 입력 설정
  pinMode(pinButton, INPUT_PULLUP);

  stopMotors();  // 초기 정지
}

// ====== 루프 ======
void loop() {
  // 버튼 누르면 주행 시작/중지 토글
  if (digitalRead(pinButton) == LOW) {
    IsDriving = !IsDriving;
    delay(300); // 버튼 디바운싱
  }

  if (IsDriving) {
    followLine(ROBOT_SPEED, FOLLOW_LINE_DURATION);
    stopMotors();
    IsDriving = false;
  }
}
