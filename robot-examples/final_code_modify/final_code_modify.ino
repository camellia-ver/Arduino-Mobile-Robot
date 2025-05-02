// 오래된 아두이노 버전에서 필요했으나 최근 버전에서는 불필요
#include <MFRC522.h>            // RFID 리더 라이브러리
#include <MFRC522Extended.h>    // 확장 기능 (편의 메서드)
#include <require_cpp11.h>      // C++11 요구 매크로
#include <SPI.h>                // SPI 통신 라이브러리

#define RFID_SS_PIN     2       // RFID 리더 SS 핀 (SPI Slave Select)
#define RFID_RST_PIN    4       // RFID 리더 RESET 핀

MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN);

// 부저와 버튼 핀 번호 정의
#define BUZZER_PIN     3        // 부저 출력 핀
#define BUTTON_PIN     A3       // 시작 버튼 입력 핀

// AGV 리프터(서보 모터) 제어
#include <Servo.h>
#define SERVO_PIN          9    // 서보 모터 제어 핀
#define SERVO_POS_DOWN   50     // 리프터 내려놓은 위치 (90-40)
#define SERVO_POS_UP    170     // 리프터 올린 위치 (180-10)
Servo servo;                 // 서보 객체 선언

// 리프터를 위로 올리는 함수
void liftUp() {
  servo.attach(SERVO_PIN);
  delay(10);
  servo.write(SERVO_POS_UP);
  delay(300);
  servo.detach();
  delay(10);
}
// 리프터를 아래로 내리는 함수
void liftDown() {
  servo.attach(SERVO_PIN);
  delay(10);
  servo.write(SERVO_POS_DOWN);
  delay(300);
  servo.detach();
  delay(10);
}

// 라인트레이싱 IR 센서 핀 및 임계값 설정
#define IR_PIN_LEFT      A6    // 왼쪽 바닥 센서
#define IR_PIN_RIGHT     A7    // 오른쪽 바닥 센서
#define IR_PIN_FRONT_LEFT   A0 // 전방 왼쪽 센서
#define IR_PIN_FRONT_CENTER A1 // 전방 중앙 센서
#define IR_PIN_FRONT_RIGHT  A2 // 전방 오른쪽 센서
#define IR_CALIB_OFFSET     60 // 센서 보정값
#define IR_THRESHOLD_WHITE (410 + IR_CALIB_OFFSET)  // 흰색 기준 최대값
#define IR_MID_THRESHOLD    (560 + IR_CALIB_OFFSET) // 흑백 중간값
#define IR_THRESHOLD_BLACK (710 + IR_CALIB_OFFSET)  // 검은색 기준 최소값
#define IR4_OBSTACLE_TEST    0 // 장애물 테스트 모드 플래그
int OBSTACLE_THRESHOLD = 1005; // 장애물 거리 임계값

// 모터 제어 핀 정의
#define MOTOR_LEFT_DIR_PIN   7 // 왼쪽 모터 방향 제어
#define MOTOR_LEFT_PWM_PIN   5 // 왼쪽 모터 속도 제어 (PWM)
#define MOTOR_RIGHT_DIR_PIN  8 // 오른쪽 모터 방향 제어
#define MOTOR_RIGHT_PWM_PIN  6 // 오른쪽 모터 속도 제어 (PWM)

// 모터 방향 정의
#define DIR_FORWARD   0        // 전진
#define DIR_BACKWARD  1        // 후진

// 두 모터 동시 제어 함수
void driveMotors(int dir1, int power1, int dir2, int power2) {
  // 방향에 따라 HIGH/LOW 설정
  bool dirHigh1 = (dir1 == DIR_FORWARD) ? HIGH : LOW;
  bool dirHigh2 = (dir2 == DIR_FORWARD) ? LOW  : HIGH;
  digitalWrite(MOTOR_LEFT_DIR_PIN, dirHigh1);
  analogWrite(MOTOR_LEFT_PWM_PIN, power1);
  digitalWrite(MOTOR_RIGHT_DIR_PIN, dirHigh2);
  analogWrite(MOTOR_RIGHT_PWM_PIN, power2);
}
// 기본 전진/후진/회전 함수
void moveForward(int p){ driveMotors(DIR_FORWARD,p, DIR_FORWARD,p);}   
void moveBackward(int p){ driveMotors(DIR_BACKWARD,p, DIR_BACKWARD,p);}  
void turnLeft(int p){ driveMotors(DIR_BACKWARD,p, DIR_FORWARD,p);}       
void turnRight(int p){ driveMotors(DIR_FORWARD,p, DIR_BACKWARD,p);}      
void stopMotors(){ analogWrite(MOTOR_LEFT_PWM_PIN,0); analogWrite(MOTOR_RIGHT_PWM_PIN,0);} 

// 설정 상수
#define TURN_SPEED      100     // 회전 시 속도
int defaultSpeed = 80;          // 라인트레이싱 기본 속도
int skipLineDuration = 200;     // 정지선/교차로 통과 시간(ms)

// 주행 상태 변수
int runState = 0;   // 0: 대기, 1~: 동작 상태
int selectedPath;   // 1=좌,2=중,3=우 경유로 저장

// 90도 좌회전 함수 (라인 인식에 의한 회전)
void turnLeft90Deg() {
  stopMotors(); delay(50);
  driveMotors(DIR_BACKWARD,80, DIR_BACKWARD,80); delay(20);
  stopMotors(); delay(50);
  // 1차 회전: 흰색 벗어날 때까지
  turnLeft(TURN_SPEED);
  while(analogRead(IR_PIN_LEFT) < IR_THRESHOLD_WHITE) delay(1);
  delay(40);
  // 2차 회전: 검은색 만날 때까지
  turnLeft(TURN_SPEED);
  while(analogRead(IR_PIN_LEFT) > IR_THRESHOLD_BLACK) delay(1);
  delay(40);
  moveForward(TURN_SPEED); delay(90);
  turnLeft(TURN_SPEED); delay(250);
  moveForward(defaultSpeed);
}
// 90도 우회전도 동일 로직
void turnRight90Deg() { /* ... 유사 구현 생략 ... */ }

// 180도 회전 함수 (bBack: 정지선 뒤로 빼고 회전)
void turnLeft180Deg(bool bBack) { /* ... */ }

// 라인트레이싱 수행 함수
unsigned long lineTraceStartTime;
void performLineTracing() {
  int v1 = analogRead(IR_PIN_LEFT);
  int v2 = analogRead(IR_PIN_RIGHT);
  // 교차로 감지: 양쪽 센서 모두 검정
  if(v1>IR_THRESHOLD_BLACK && v2>IR_THRESHOLD_BLACK) {
    switch(runState) {
      case 1:
        // 중앙 경유로 장애물 검사
        stopMotors(); delay(100);
        if(analogRead(IR_PIN_FRONT_CENTER)<OBSTACLE_THRESHOLD) {
          runState=11; turnLeft90Deg();
        } else {
          selectedPath=2; runState=102;
          moveForward(defaultSpeed); delay(skipLineDuration);
        }
        break;
      // 이후 상태별 처리 ...
    }
  }
  else if(v1>IR_THRESHOLD_BLACK) turnLeft(defaultSpeed);
  else if(v2>IR_THRESHOLD_BLACK) turnRight(defaultSpeed);
  else moveForward(defaultSpeed);
}

void setup() {
  // 핀 모드 설정
  pinMode(MOTOR_LEFT_DIR_PIN,OUTPUT);
  pinMode(MOTOR_LEFT_PWM_PIN,OUTPUT);
  pinMode(MOTOR_RIGHT_DIR_PIN,OUTPUT);
  pinMode(MOTOR_RIGHT_PWM_PIN,OUTPUT);

  pinMode(BUZZER_PIN,OUTPUT);
  noTone(BUZZER_PIN);
  
  Serial.begin(9600);
  
  liftDown();          // 리프터 초기 위치
  
  SPI.begin();         // RFID SPI 통신 시작
  mfrc522.PCD_Init();  // RFID 초기화
}

void loop() {
  switch(runState) {
    case 0:
      // 카드 태그 대기
      if(mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        runState=1;
        tone(BUZZER_PIN,262); delay(100);
        tone(BUZZER_PIN,330); delay(250);
        noTone(BUZZER_PIN);
      }
      delay(100);
      break;
    case 12:
      // 좌측 경유로 장애물 측정
      if(millis()-lineTraceStartTime>160) {
        stopMotors(); delay(100);
        if(analogRead(IR_PIN_FRONT_CENTER)<OBSTACLE_THRESHOLD) {
          selectedPath=3; runState=13;
          turnLeft180Deg(false);
        } else {
          selectedPath=1; runState=101;
          moveForward(defaultSpeed);
        }
      }
      break;
    default:
      performLineTracing();
      break;
  }
}
