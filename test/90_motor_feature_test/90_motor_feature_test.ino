// ===== 모터 핀 정의 =====
#define LEFT_MOTOR_DIR_PIN 7
#define LEFT_MOTOR_PWM_PIN 5
#define RIGHT_MOTOR_DIR_PIN 8
#define RIGHT_MOTOR_PWM_PIN 6

#define LEFT_FORWARD  HIGH
#define LEFT_BACKWARD LOW
#define RIGHT_FORWARD LOW
#define RIGHT_BACKWARD HIGH

#define pinButton A3

const int ROBOT_SPEED = 80;
int TURN_DELAY_90 = 400; // 실험 후 조정

bool buttonPrevState = HIGH;
int turnCount = 0;

void controlMotors(int leftDirection, int leftPower, int rightDirection, int rightPower) {
  digitalWrite(LEFT_MOTOR_DIR_PIN, leftDirection);
  analogWrite(LEFT_MOTOR_PWM_PIN, leftPower);
  digitalWrite(RIGHT_MOTOR_DIR_PIN, rightDirection);
  analogWrite(RIGHT_MOTOR_PWM_PIN, rightPower);
}

void turnRight90() {
  Serial.print("오른쪽 90도 회전: ");
  Serial.print(++turnCount);
  Serial.println(" 회");
  controlMotors(LEFT_FORWARD, ROBOT_SPEED, RIGHT_BACKWARD, ROBOT_SPEED);
  delay(TURN_DELAY_90);
  stopMotors();
}

void stopMotors() {
  analogWrite(LEFT_MOTOR_PWM_PIN, 0);
  analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(pinButton, INPUT_PULLUP);
  stopMotors();
  Serial.println("버튼을 누르면 오른쪽으로 90도 회전합니다.");
}

void loop() {
  bool buttonState = digitalRead(pinButton);
  if (buttonState == LOW && buttonPrevState == HIGH) {
    delay(50); // 디바운싱
    turnRight90();
  }
  buttonPrevState = buttonState;
}
