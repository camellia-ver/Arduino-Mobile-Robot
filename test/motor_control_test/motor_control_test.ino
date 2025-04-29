// ===== 모터 핀 정의 =====
#define LEFT_MOTOR_DIR_PIN 7  // 1번(왼쪽) 모터 방향 제어 핀
#define LEFT_MOTOR_PWM_PIN 5  // 1번(왼쪽) 모터 속도 제어 핀

#define RIGHT_MOTOR_DIR_PIN 8 // 2번(오른쪽) 모터 방향 제어 핀
#define RIGHT_MOTOR_PWM_PIN 6 // 2번(오른쪽) 모터 속도 제어 핀

#define FORWARD HIGH
#define BACKWARD LOW

// ===== 로봇 속도 및 회전 시간 설정 =====
const int ROBOT_SPEED = 100; // 로봇 속도 (0~255)
const int TURN_DELAY_90 = 1500;  // 90도 회전 시간
const int TURN_DELAY_180 = 3000; // 180도 회전 시간

// 버튼 핀 정의 (주행 시작/정지)
#define pinButton A3  // 주행 시작/정지 버튼 연결 핀

bool IsDriving = false; // 주행 상태 변수

// 모터 제어 함수
void controlMotors(int leftDirection, int leftPower, int rightDirection, int rightPower) {
  // 왼쪽 모터 설정
  digitalWrite(LEFT_MOTOR_DIR_PIN, leftDirection);
  analogWrite(LEFT_MOTOR_PWM_PIN, leftPower);

  // 오른쪽 모터 설정
  digitalWrite(RIGHT_MOTOR_DIR_PIN, !rightDirection);
  analogWrite(RIGHT_MOTOR_PWM_PIN, rightPower);
}

void moveRobotForward(int power) {
  controlMotors(HIGH, power, HIGH, power);  // 두 모터 전진
}

void turnRobotLeftInPlace(int power) {
  controlMotors(LOW, power, HIGH, power);   // 왼쪽 모터 후진, 오른쪽 모터 전진
}

void turnRobotRightInPlace(int power) {
  controlMotors(HIGH, power, LOW, power);   // 왼쪽 모터 전진, 오른쪽 모터 후진
}

void stopMotors() {
  analogWrite(LEFT_MOTOR_PWM_PIN, 0);  // 왼쪽 모터 정지
  analogWrite(RIGHT_MOTOR_PWM_PIN, 0); // 오른쪽 모터 정지
}

void turnRight90() {
  turnRobotRightInPlace(ROBOT_SPEED);   // 우회전 시작
  delay(TURN_DELAY_90);                  // 90도 회전
  stopMotors();                          // 정지
}

void turnLeft90() {
  turnRobotLeftInPlace(ROBOT_SPEED);    // 좌회전 시작
  delay(TURN_DELAY_90);                 // 90도 회전
  stopMotors();                         // 정지
}

void turnAround180() {
  turnRobotRightInPlace(ROBOT_SPEED);   // 우회전 시작
  delay(TURN_DELAY_180);                // 180도 회전
  stopMotors();                         // 정지
}

void setup() {
  Serial.begin(115200);

  // 모터 핀 출력 설정
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(pinButton, INPUT);  // 버튼 핀 입력 설정

  IsDriving = false;

  Serial.println("모터 테스트 시작");
}

void loop() {
  // 버튼이 눌렸을 때 동작
  if (digitalRead(pinButton) == 0) { // 버튼 눌림 감지
    if (IsDriving) { // 주행 중이라면
      stopMotors(); // 모터 정지

      // 버튼이 떼어질 때까지 대기
      while (digitalRead(pinButton) == 0)
        delay(10);

      IsDriving = false; // 상태를 '정지'로 변경
    } else { // 정지 상태라면
      // 버튼이 떼어질 때까지 대기
      while (digitalRead(pinButton) == 0)
        delay(10);

      IsDriving = true; // 상태를 '주행 중'으로 변경
    }
    delay(100); // 버튼 채터링 방지
  }

  // 주행 중일 때 모터 동작 테스트
  if (IsDriving) {
    Serial.println("Testing motor control...");

    // moveRobotForward(ROBOT_SPEED);  // 전진
    // delay(2000);            // 2초 동안 전진
    // stopMotors();           // 정지
    // delay(1000);            // 1초 대기

    // turnLeft90();           // 좌회전
    // delay(2000);            // 2초 대기
    // stopMotors();           // 정지
    // delay(1000);            // 1초 대기

    // moveRobotForward(ROBOT_SPEED);  // 전진
    // delay(2000);            // 2초 동안 전진
    // stopMotors();           // 정지
    // delay(1000);            // 1초 대기

    // turnRight90();          // 우회전
    // delay(2000);            // 2초 대기
    // stopMotors();           // 정지
    // delay(1000);            // 1초 대기

    // moveRobotForward(ROBOT_SPEED);  // 전진
    // delay(2000);            // 2초 동안 전진
    // stopMotors();           // 정지
    // delay(1000);            // 1초 대기

    turnAround180();        // 180도 회전
    delay(2000);            // 2초 대기
    stopMotors();           // 정지

    IsDriving = false;      // 테스트 완료 후 주행 종료
  }
}
