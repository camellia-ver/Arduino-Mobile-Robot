// ----------------------------
// [1] 상수 및 핀 정의
// ----------------------------
#define BUTTON_PIN         A3   // 버튼 핀

#define PIN_LEFT_DIR       7    // 왼쪽 모터 방향 제어 핀
#define PIN_LEFT_PWM       5    // 왼쪽 모터 속도 제어 핀
#define PIN_RIGHT_DIR      8    // 오른쪽 모터 방향 제어 핀
#define PIN_RIGHT_PWM      6    // 오른쪽 모터 속도 제어 핀

#define DIRECTION_FORWARD  0    // 전진
#define DIRECTION_BACKWARD 1    // 후진

#define FORWARD_SPEED      70   // 감속 시 전진 속도
#define BACKWARD_SPEED     70   // 후진 속도
#define TURN_SPEED         100  // 회전 속도
#define FINE_TUNE_SPEED    90   // 회전 보정 속도

int ROTATE_TIME_90     = 400;  // 90도 회전 시간 (ms)
int ROTATE_TIME_180    = 800;  // 180도 회전 시간 (ms)
int BACKWARD_TIME      = 230;  // 후진 시간 (ms)
int ROTATE_FINE_TUNE   = 50;   // 회전 후 보정 시간 (ms)
#define POST_ROTATE_DELAY 500  // 회전 후 직진 전 대기 시간 (ms)


// ----------------------------
// [2] 모터 제어 함수
// ----------------------------
void setMotorControl(int leftDirection, int leftSpeed, int rightDirection, int rightSpeed) {
  bool leftDirSignal = (leftDirection == DIRECTION_FORWARD) ? HIGH : LOW;

  // 오른쪽 모터는 보드 특성상 정방향 시 LOW
  bool rightDirSignal = (rightDirection == DIRECTION_FORWARD) ? LOW : HIGH;

  digitalWrite(PIN_LEFT_DIR, leftDirSignal);
  analogWrite(PIN_LEFT_PWM, leftSpeed);

  digitalWrite(PIN_RIGHT_DIR, rightDirSignal);
  analogWrite(PIN_RIGHT_PWM, rightSpeed);
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


// ----------------------------
// [3] 회전 함수
// ----------------------------
void turn90(bool isLeft) {
  // 짧게 전진 후 멈춤
  driveForward(FORWARD_SPEED);
  delay(20);
  haltMotors();
  delay(100);

  // 후진
  driveBackward(BACKWARD_SPEED);
  delay(BACKWARD_TIME);

  // 방향 회전
  if (isLeft) {
    setMotorControl(DIRECTION_BACKWARD, TURN_SPEED, DIRECTION_FORWARD, TURN_SPEED);
    delay(ROTATE_TIME_90);
    setMotorControl(DIRECTION_BACKWARD, FINE_TUNE_SPEED, DIRECTION_FORWARD, FINE_TUNE_SPEED);
  } else {
    setMotorControl(DIRECTION_FORWARD, TURN_SPEED, DIRECTION_BACKWARD, TURN_SPEED);
    delay(ROTATE_TIME_90);
    setMotorControl(DIRECTION_FORWARD, FINE_TUNE_SPEED, DIRECTION_BACKWARD, FINE_TUNE_SPEED);
  }

  delay(ROTATE_FINE_TUNE);
  haltMotors();
}

void turn180() {
  turn90(true);  // 좌회전
  delay(POST_ROTATE_DELAY);
  turn90(true);  // 다시 좌회전
}


// ----------------------------
// [4] 버튼 테스트용 루프 변수
// ----------------------------
int testStep = 0;

unsigned long lastButtonPressTime = 0;
const unsigned long debounceDelay = 200;  // 200ms 디바운스

// ----------------------------
// [5] setup 및 loop
// ----------------------------
void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  bool buttonState = digitalRead(BUTTON_PIN) == LOW;
  unsigned long currentTime = millis();

  if (buttonState && currentTime - lastButtonPressTime > debounceDelay) {
    lastButtonPressTime = currentTime;
    testStep = (testStep + 1) % 3;

    Serial.print("테스트 단계: ");
    Serial.println(testStep);

    switch (testStep) {
      case 0:
        Serial.println("→ 90도 우회전 테스트");
        turn90(false);  // 우회전
        break;
      case 1:
        Serial.println("→ 90도 좌회전 테스트");
        turn90(true);   // 좌회전
        break;
      case 2:
        Serial.println("→ 180도 회전 테스트");
        turn180();
        break;
    }

    delay(1000); // 회전 후 대기
  }
}
