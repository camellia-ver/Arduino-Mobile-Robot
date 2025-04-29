// ===== 모터 핀 정의 =====
#define LEFT_MOTOR_DIR_PIN 7  // 1번(왼쪽) 모터 방향 제어 핀
#define LEFT_MOTOR_PWM_PIN 5  // 1번(왼쪽) 모터 속도 제어 핀

#define RIGHT_MOTOR_DIR_PIN 8 // 2번(오른쪽) 모터 방향 제어 핀
#define RIGHT_MOTOR_PWM_PIN 6 // 2번(오른쪽) 모터 속도 제어 핀

#define LEFT_FORWARD  HIGH
#define LEFT_BACKWARD LOW
#define RIGHT_FORWARD LOW   // 실제 회로가 반대 방향이면
#define RIGHT_BACKWARD HIGH

// ===== 버튼 핀 설정 =====
#define pinButton A3

// ===== 라인 센서 핀 및 설정 =====
#define LINE_SENSOR_LEFT A6
#define LINE_SENSOR_RIGHT A7

#define LT_AJDUST       60             // 젤리비 센서 값 조정 (필요한 경우 이 값을 실험적으로 조정)
#define LT_MAX_WHITE   410 + LT_AJDUST // 흰색으로 판단하는 최대값
#define LT_MID_VALUE   560 + LT_AJDUST // 흑백 판단 경계값(중간값)
#define LT_MIN_BLACK   710 + LT_AJDUST // 검은색으로 판단하는 최소값

#define FOLLOW_LINE_DURATION 5000 // ms 단위

// ===== 로봇 속도 및 회전 시간 설정 =====
const int ROBOT_SPEED = 80; // 로봇 속도 (0~255)
const int TURN_DELAY_90 = 400;  // 90도 회전 시간
const int TURN_DELAY_180 = 800; // 180도 회전 시간

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
  controlMotors(LEFT_FORWARD, power, RIGHT_FORWARD, power);
}

void turnRobotLeftInPlace(int power) {
  controlMotors(LEFT_BACKWARD, power, RIGHT_FORWARD, power);
}

void turnRobotRightInPlace(int power) {
  controlMotors(LEFT_FORWARD, power, RIGHT_BACKWARD, power);
}

void stopMotors() {
  analogWrite(LEFT_MOTOR_PWM_PIN, 0);
  analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
  digitalWrite(LEFT_MOTOR_DIR_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_DIR_PIN, LOW);
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

// ====== 회전 후 보정 함수 ======
void adjustTurn(int targetAngle){
  unsigned long adjustStart = millis();
  const unsigned long adjustTimeout = 1000; // 1초 제한

  while (millis() - adjustStart < adjustTimeout) {
    int leftValue = readSensorAverage(LINE_SENSOR_LEFT);
    int rightValue = readSensorAverage(LINE_SENSOR_RIGHT);

    if (leftValue < LT_MIN_BLACK && rightValue < LT_MIN_BLACK) break; // 조기 탈출

    if (leftValue > LT_MIN_BLACK) {
      turnRobotLeftInPlace(ROBOT_SPEED / 2); // 속도 줄여 미세 조정
    } else if (rightValue > LT_MIN_BLACK) {
      turnRobotRightInPlace(ROBOT_SPEED / 2);
    } else {
      stopMotors();
    }
    delay(10);
  }
  stopMotors();
}

// ====== 회전 함수 수정 ======
void turnRight90WithAdjustment() {
  turnRobotRightInPlace(ROBOT_SPEED);
  delay(TURN_DELAY_90);
  stopMotors();
  adjustTurn(90);  // 회전 후 보정
}

void turnLeft90WithAdjustment() {
  turnRobotLeftInPlace(ROBOT_SPEED);
  delay(TURN_DELAY_90);
  stopMotors();
  adjustTurn(90);  // 회전 후 보정
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
    int leftValue = readSensorAverage(LINE_SENSOR_LEFT);
    int rightValue = readSensorAverage(LINE_SENSOR_RIGHT);

    Serial.print("L: "); Serial.print(leftValue);
    Serial.print(" R: "); Serial.println(rightValue);

    // ===== 교차로 감지 (두 센서 모두 라인 위 감지) =====
    if ((LT_MIN_BLACK < leftValue) && (LT_MIN_BLACK < rightValue)) {
      if (!crossDetected) {  // 교차로 처음 발견했을 때만
        Serial.println("교차로 감지 → 좌회전");

        moveRobotForward(speed); // 약간 전진
        delay(200);

        turnLeft90WithAdjustment();            // 좌회전
        crossDetected = true;    // 이미 좌회전 했다고 표시
      }
    }
    // ===== 왼쪽으로 살짝 이동 =====
    else if (leftValue > LT_MIN_BLACK) {
      Serial.println("좌측 조정");
      turnRobotLeftInPlace(speed);
      crossDetected = false;
    }
    // ===== 오른쪽으로 살짝 이동 =====
    else if (rightValue > LT_MIN_BLACK) {
      Serial.println("우측 조정");
      turnRobotRightInPlace(speed);
      crossDetected = false;
    }
    // ===== 직진 =====
    else{
      Serial.println("직진");
      moveRobotForward(speed);
      crossDetected = false; // 다음 교차로 인식 가능하도록 초기화
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
