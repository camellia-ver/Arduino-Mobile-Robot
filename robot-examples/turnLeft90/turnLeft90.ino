// ----------------------------
// [1] 상수 및 핀 정의
// ----------------------------
#define BUTTON_PIN       A3   // 버튼 핀

#define PIN_LEFT_DIR    7   // 왼쪽 모터 방향 제어 핀
#define PIN_LEFT_PWM    5   // 왼쪽 모터 속도 제어 핀
#define PIN_RIGHT_DIR   8   // 오른쪽 모터 방향 제어 핀
#define PIN_RIGHT_PWM   6   // 오른쪽 모터 속도 제어 핀

#define DIRECTION_FORWARD   0  // 전진
#define DIRECTION_BACKWARD  1  // 후진

#define FORWARD_SPEED       70      // 감속 시 전진 속도
#define BACKWARD_SPEED      70      // 후진 속도
#define TURN_SPEED          100     // 회전 속도
int ROTATE_TIME = 300;             // 180도 회전 시간 (ms)
int BACKWARD_TIME = 230;           // 후진 시간 (ms)
int ROTATE_FINE_TUNE = 50;         // 회전 후 보정 시간 (ms)

#define POST_ROTATE_DELAY   500     // 회전 후 직진 시작 전 대기 시간 (ms)

// ----------------------------
// [2] 전역 변수 및 객체
// ----------------------------
float LEFT_MOTOR_RATIO = 1.00;
float RIGHT_MOTOR_RATIO = 1.00;

bool prevButtonState = HIGH;
bool alreadyRotated = false;

// ----------------------------
// [3] 모터 제어 함수
// ----------------------------
void setMotorControl(int leftDirection, int leftSpeed, int rightDirection, int rightSpeed) {
  bool leftDirSignal = (leftDirection == DIRECTION_FORWARD) ? HIGH : LOW;
  bool rightDirSignal = (rightDirection == DIRECTION_FORWARD) ? LOW : HIGH;

  digitalWrite(PIN_LEFT_DIR, leftDirSignal);
  analogWrite(PIN_LEFT_PWM, int(leftSpeed * LEFT_MOTOR_RATIO));

  digitalWrite(PIN_RIGHT_DIR, rightDirSignal);
  analogWrite(PIN_RIGHT_PWM, int(rightSpeed * RIGHT_MOTOR_RATIO));
}

void haltMotors() {
  analogWrite(PIN_LEFT_PWM, 0);
  analogWrite(PIN_RIGHT_PWM, 0);
}

void driveForward(int speed) {
  setMotorControl(DIRECTION_FORWARD, speed, DIRECTION_FORWARD, speed);
}

void driveBackward(int speed) {
  setMotorControl(DIRECTION_BACKWARD, speed, DIRECTION_BACKWARD, speed);
}

void turnLeft(int speed) {
  setMotorControl(DIRECTION_BACKWARD, int(speed * 0.9), DIRECTION_FORWARD, speed);
}

void turnRight(int speed) {
  setMotorControl(DIRECTION_FORWARD, speed, DIRECTION_BACKWARD, int(speed * 0.9));
}

// 왼쪽으로 90도 회전하는 함수
void turn90Left() {
  // 감속 전진
  setMotorControl(DIRECTION_FORWARD, FORWARD_SPEED, DIRECTION_FORWARD, FORWARD_SPEED); 
  delay(20);
  haltMotors();
  delay(100);

  // 후진
  setMotorControl(DIRECTION_BACKWARD, BACKWARD_SPEED, DIRECTION_BACKWARD, BACKWARD_SPEED); 
  delay(BACKWARD_TIME);

  // 왼쪽 회전 (비대칭 회전)
  setMotorControl(DIRECTION_BACKWARD, TURN_SPEED, DIRECTION_FORWARD, TURN_SPEED); 
  delay(ROTATE_TIME);

  // 미세 보정
  setMotorControl(DIRECTION_BACKWARD, 90, DIRECTION_FORWARD, 90); 
  delay(ROTATE_FINE_TUNE);

  haltMotors();  // 마지막 정지 추가
}

void printCurrentSettings() {
  Serial.println("=== 현재 설정값 ===");
  Serial.print("ROTATE_TIME: "); Serial.println(ROTATE_TIME);
  Serial.print("BACKWARD_TIME: "); Serial.println(BACKWARD_TIME);
  Serial.print("ROTATE_FINE_TUNE: "); Serial.println(ROTATE_FINE_TUNE);
  Serial.println("====================");
}

// ----------------------------
// [4] 초기화
// ----------------------------
void setup() {
  Serial.begin(9600);

  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // 버튼 풀업 설정

  Serial.println("▶ 90도 왼쪽 회전 테스트 준비 완료");
  printCurrentSettings();
}

// ----------------------------
// [5] 메인 루프
// ----------------------------
void loop() {
  bool buttonState = digitalRead(BUTTON_PIN);

  // 버튼이 눌리면 회전 수행
  if (prevButtonState == HIGH && buttonState == LOW) {
    Serial.println("▶ 버튼 눌림 → 90도 왼쪽 회전 시작");
    turn90Left();  // 왼쪽 회전 함수 호출
    Serial.println("▶ 회전 완료");
    printCurrentSettings();
  }
  prevButtonState = buttonState;

  // 설정값 조정은 계속 가능
  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'r') {
      ROTATE_TIME = Serial.parseInt();
      Serial.print("ROTATE_TIME 설정됨: ");
      Serial.println(ROTATE_TIME);
    } else if (command == 'b') {
      BACKWARD_TIME = Serial.parseInt();
      Serial.print("BACKWARD_TIME 설정됨: ");
      Serial.println(BACKWARD_TIME);
    } else if (command == 'f') {
      ROTATE_FINE_TUNE = Serial.parseInt();
      Serial.print("ROTATE_FINE_TUNE 설정됨: ");
      Serial.println(ROTATE_FINE_TUNE);
    } else if (command == 'p') {
      printCurrentSettings();
    }
  }

  delay(50);  // 짧은 대기
}
