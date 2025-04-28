// ===== 모터 핀 정의 =====
#define LEFT_MOTOR_DIR_PIN 7  // 1번(왼쪽) 모터 방향 제어 핀
#define LEFT_MOTOR_PWM_PIN 5  // 1번(왼쪽) 모터 속도 제어 핀
#define RIGHT_MOTOR_DIR_PIN 8 // 2번(오른쪽) 모터 방향 제어 핀
#define RIGHT_MOTOR_PWM_PIN 6 // 2번(오른쪽) 모터 속도 제어 핀

#define FORWARD HIGH
#define BACKWARD LOW

// ===== 버튼 핀 정의 =====
#define pinButton A3  // 주행 시작/정지 버튼 핀

// ===== 라인 센서 핀 및 설정 =====
#define LINE_SENSOR_LEFT A0
#define LINE_SENSOR_RIGHT A1
#define LINE_THRESHOLD 500 // 센서 임계값

// ===== 로봇 속도 및 회전 시간 =====
const int ROBOT_SPEED = 150;  // 로봇 속도
const int TURN_DELAY_90 = 400;  // 90도 회전 시간
const int TURN_DELAY_180 = 800; // 180도 회전 시간

bool IsDriving = false;  // 주행 상태 변수

// ===== 모터 제어 함수 =====
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

// ===== 라인 센서 읽기 함수 =====
int readSensorAverage(int pin) {
  int total = 0;
  for (int i = 0; i < 5; i++) {
    total += analogRead(pin);
  }
  return total / 5;
}

// ===== 라인트레이싱 함수 =====
void followLine(int speed) {
  int leftValue = readSensorAverage(LINE_SENSOR_LEFT);
  int rightValue = readSensorAverage(LINE_SENSOR_RIGHT);

  // 두 센서 모두 라인 위 (검정) → 정지
  if (leftValue > LINE_THRESHOLD && rightValue > LINE_THRESHOLD) {
    Serial.println("모터 정지");
    stopMotors();
  }
  // 두 센서 모두 라인 아래 (흰색) → 직진
  else if (leftValue < LINE_THRESHOLD && rightValue < LINE_THRESHOLD) {
    Serial.println("직진");
    moveRobotForward(speed);
  }
  // 왼쪽 센서만 라인 아래 → 좌회전
  else if (leftValue < LINE_THRESHOLD && rightValue > LINE_THRESHOLD) {
    Serial.println("좌회전");
    turnRobotLeftInPlace(speed);
  }
  // 오른쪽 센서만 라인 아래 → 우회전
  else if (rightValue < LINE_THRESHOLD && leftValue > LINE_THRESHOLD) {
    Serial.println("우회전");
    turnRobotRightInPlace(speed);
  }

  delay(5); // 센서 읽기 간격
}

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

// ===== 루프 =====
void loop() {
  // 버튼이 눌리면 주행 시작
  if (digitalRead(pinButton) == LOW) {
    IsDriving = !IsDriving;  // 주행 상태 토글
    delay(300); // 버튼 디바운싱용 딜레이
  }

  if (IsDriving) {
    followLine(ROBOT_SPEED);  // 라인 트레이싱 주행
  } else {
    stopMotors();  // 주행 중이지 않으면 모터 정지
  }
}
