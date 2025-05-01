/*******************************************************
 * 프로젝트명: 바퀴 움직이기
 * 기능: 모터를 움직여서 상하좌우 움직임 구현
 * 작성자: 조영란
 * 날짜: 2025-05-01
 *******************************************************/

// ----------------------------
// [1] 상수 및 핀 정의
// ----------------------------
#define PIN_BUTTON      A3  // 버튼 입력 핀

#define PIN_LEFT_DIR    7   // 왼쪽 모터 방향 제어 핀
#define PIN_LEFT_PWM    5   // 왼쪽 모터 속도 제어 핀
#define PIN_RIGHT_DIR   8   // 오른쪽 모터 방향 제어 핀
#define PIN_RIGHT_PWM   6   // 오른쪽 모터 속도 제어 핀

#define DIRECTION_FORWARD   0  // 전진
#define DIRECTION_BACKWARD  1  // 후진

// ----------------------------
// [2] 전역 변수 및 객체
// ----------------------------
int defaultMotorSpeed = 100;   // 기본 모터 속도
bool prevButtonState = HIGH;  // 버튼의 이전 상태 저장

float leftMotorRatio = 1.00;   // 왼쪽 모터 보정 비율
float rightMotorRatio = 1.00;  // 오른쪽 모터 보정 비율

unsigned long moveStartTime = 0;      // 주행 시작 시간
const unsigned long moveDuration = 1000; // 주행 지속 시간 (ms)
bool isMoving = false;                // 현재 주행 중 여부

enum MovementState { STATE_IDLE, STATE_FORWARD, STATE_BACKWARD, STATE_LEFT, STATE_RIGHT };
MovementState currentState = STATE_IDLE;

// ----------------------------
// [3] 모터 제어 함수
// ----------------------------
/**
 * 좌우 모터를 개별적으로 제어한다.
 * @param leftDirection 왼쪽 모터 방향 (0: 전진, 1: 후진)
 * @param leftSpeed     왼쪽 모터 속도 (0~255)
 * @param rightDirection 오른쪽 모터 방향
 * @param rightSpeed    오른쪽 모터 속도
 */
void setMotorControl(int leftDirection, int leftSpeed, int rightDirection, int rightSpeed) {
  bool leftDirSignal = (leftDirection == DIRECTION_FORWARD) ? HIGH : LOW;
  bool rightDirSignal = (rightDirection == DIRECTION_FORWARD) ? LOW : HIGH;

  digitalWrite(PIN_LEFT_DIR, leftDirSignal);
  analogWrite(PIN_LEFT_PWM, leftSpeed * leftMotorRatio);

  digitalWrite(PIN_RIGHT_DIR, rightDirSignal);
  analogWrite(PIN_RIGHT_PWM, rightSpeed * rightMotorRatio);
}

/**
 * 정지
 */
void haltMotors() {
  analogWrite(PIN_LEFT_PWM, 0);
  analogWrite(PIN_RIGHT_PWM, 0);
}

/**
 * 전진
 */
void driveForward(int speed) {
  setMotorControl(DIRECTION_FORWARD, speed, DIRECTION_FORWARD, speed);
}

/**
 * 후진
 */
void driveBackward(int speed) {
  setMotorControl(DIRECTION_BACKWARD, speed, DIRECTION_BACKWARD, speed);
}

/**
 * 좌회전
 */
void turnLeft(int speed) {
  setMotorControl(DIRECTION_BACKWARD, speed * 0.9, DIRECTION_FORWARD, speed);
}

/**
 * 우회전
 */
void turnRight(int speed) {
  setMotorControl(DIRECTION_FORWARD, speed, DIRECTION_BACKWARD, speed * 0.9);
}

// ----------------------------
// [4] 초기화 함수
// ----------------------------
void setup() {
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP); // 내부 풀업 저항 활성화

  Serial.begin(9600); // 디버깅용 시리얼 통신 시작
  Serial.println("시스템 초기화 완료");
}

// ----------------------------
// [5] 메인 루프
// ----------------------------
void loop() {
  bool buttonState = digitalRead(PIN_BUTTON);

  // 버튼이 눌린 경우 && 현재 모터가 정지 상태일 때
  if (buttonState == LOW && prevButtonState == HIGH && !isMoving) {
    // 다음 상태로 전환
    currentState = (MovementState)((currentState + 1) % 5);

    // 상태에 따라 동작 실행
    switch (currentState) {
      case STATE_FORWARD:
        Serial.println("상태: 전진");
        driveForward(defaultMotorSpeed);
        break;
      case STATE_BACKWARD:
        Serial.println("상태: 후진");
        driveBackward(defaultMotorSpeed);
        break;
      case STATE_LEFT:
        Serial.println("상태: 좌회전");
        turnLeft(defaultMotorSpeed);
        break;
      case STATE_RIGHT:
        Serial.println("상태: 우회전");
        turnRight(defaultMotorSpeed);
        break;
      default:
        Serial.println("상태: 정지");
        haltMotors();
        break;
    }

    isMoving = true;
    moveStartTime = millis(); // 시작 시간 기록
  }

  // 동작이 완료되면 모터 정지
  if (isMoving && (millis() - moveStartTime >= moveDuration)) {
    haltMotors();
    Serial.println("모터 정지");
    isMoving = false;
  }

  prevButtonState = buttonState; // 버튼 상태 갱신
}
