#define LEFT_MOTOR_DIR_PIN 7
#define LEFT_MOTOR_PWM_PIN 5
#define RIGHT_MOTOR_DIR_PIN 8
#define RIGHT_MOTOR_PWM_PIN 6

#define LEFT_FORWARD  HIGH
#define RIGHT_FORWARD LOW

#define pinButton A3

const int BASE_SPEED = 100; // 기준 속도

// ===== 조정 가능한 보정 계수 =====
const float LEFT_CORRECTION = 1.0;   // 왼쪽 바퀴 보정 비율
const float RIGHT_CORRECTION = 0.9; // 오른쪽 바퀴 보정 비율

bool isMoving = false;
bool prevButton = HIGH;

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(pinButton, INPUT_PULLUP);

  Serial.println("=== 좌우 바퀴 출력 균형 테스트 시작 ===");
  Serial.print("기준 속도: "); Serial.println(BASE_SPEED);
  Serial.print("LEFT_CORRECTION: "); Serial.println(LEFT_CORRECTION, 2);
  Serial.print("RIGHT_CORRECTION: "); Serial.println(RIGHT_CORRECTION, 2);
  Serial.println("버튼을 눌러 전진 시작/정지");
}

void loop() {
  bool nowButton = digitalRead(pinButton);

  if (nowButton == LOW && prevButton == HIGH) {
    isMoving = !isMoving;
    delay(200); // 디바운싱

    if (isMoving) {
      Serial.println("▶ 전진 시작");
    } else {
      Serial.println("■ 정지");
    }
  }

  if (isMoving) {
    int leftPWM = BASE_SPEED * LEFT_CORRECTION;
    int rightPWM = BASE_SPEED * RIGHT_CORRECTION;

    digitalWrite(LEFT_MOTOR_DIR_PIN, LEFT_FORWARD);
    analogWrite(LEFT_MOTOR_PWM_PIN, leftPWM);

    digitalWrite(RIGHT_MOTOR_DIR_PIN, RIGHT_FORWARD);
    analogWrite(RIGHT_MOTOR_PWM_PIN, rightPWM);

    Serial.print("PWM → 좌: ");
    Serial.print(leftPWM);
    Serial.print(" | 우: ");
    Serial.println(rightPWM);

    delay(300); // 출력 주기 (속도 너무 빠르지 않게)
  } else {
    analogWrite(LEFT_MOTOR_PWM_PIN, 0);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
  }

  prevButton = nowButton;
}
