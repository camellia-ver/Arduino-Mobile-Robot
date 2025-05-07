// 오래된 아두이노 버전에서 필요했으나 최근 버전에서는 불필요
#include <MFRC522.h>            // RFID 리더 라이브러리
#include <MFRC522Extended.h>    // 확장 기능 (편의 메서드)
#include <require_cpp11.h>      // C++11 요구 매크로
#include <SPI.h>                // SPI 통신 라이브러리
#include <Servo.h>

// === RFID 관련 상수 ===
constexpr uint8_t RFID_SS_PIN  = 2;
constexpr uint8_t RFID_RST_PIN = 4;

// === 핀 번호 ===
constexpr uint8_t BUZZER_PIN = 3;
constexpr uint8_t BUTTON_PIN = A3;
constexpr uint8_t SERVO_PIN  = 9;

constexpr uint8_t IR_PIN_LEFT         = A6;
constexpr uint8_t IR_PIN_RIGHT        = A7;
constexpr uint8_t IR_PIN_FRONT_LEFT   = A0;
constexpr uint8_t IR_PIN_FRONT_CENTER = A1;
constexpr uint8_t IR_PIN_FRONT_RIGHT  = A2;

constexpr uint8_t MOTOR_LEFT_DIR_PIN  = 7;
constexpr uint8_t MOTOR_LEFT_PWM_PIN  = 5;
constexpr uint8_t MOTOR_RIGHT_DIR_PIN = 8;
constexpr uint8_t MOTOR_RIGHT_PWM_PIN = 6;

// === IR 센서 임계값 ===
constexpr int IR_CALIB_OFFSET       = 60;
constexpr int IR_THRESHOLD_WHITE    = 410 + IR_CALIB_OFFSET;
constexpr int IR_MID_THRESHOLD      = 560 + IR_CALIB_OFFSET;
constexpr int IR_THRESHOLD_BLACK    = 710 + IR_CALIB_OFFSET;

// === 서보 위치 ===
constexpr int SERVO_POS_DOWN        = 50;
constexpr int SERVO_POS_UP          = 170;

// === 모터 속도 ===
constexpr int DEFAULT_SPEED         = 80;
constexpr int TURN_SPEED            = 100;

// === 동작 딜레이 ===
constexpr unsigned long SERVO_MOVE_DELAY = 300UL;
constexpr unsigned long SERVO_ATTACH_DELAY = 10UL;
constexpr unsigned long SERVO_DETACH_DELAY = 10UL;
constexpr unsigned long STOP_DELAY_BEFORE_TURN = 50UL;
constexpr unsigned long TURN_FINAL_FORWARD = 90UL;
constexpr unsigned long TURN_EXTRA_DELAY = 250UL;
constexpr unsigned long SKIP_LINE_DURATION = 200UL;
constexpr unsigned long INTERSECTION_WAIT = 160UL;
constexpr unsigned long LINE_SKIP_DELAY  = 90UL;

// === 장애물 임계값 ===
int OBSTACLE_THRESHOLD = 1005; // 변수 유지 (가변 가능성 있음)

// === 방향 정의 ===
constexpr int DIR_FORWARD  = 0;
constexpr int DIR_BACKWARD = 1;

MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN);

Servo servo;                 // 서보 객체 선언

// 리프터를 위로 올리는 함수
void liftUp() {
  servo.attach(SERVO_PIN);
  delay(SERVO_ATTACH_DELAY);
  servo.write(SERVO_POS_UP);
  delay(SERVO_MOVE_DELAY);
  servo.detach();
  delay(SERVO_DETACH_DELAY);
}

// 리프터를 아래로 내리는 함수
void liftDown() {
  servo.attach(SERVO_PIN);
  delay(SERVO_ATTACH_DELAY);
  servo.write(SERVO_POS_DOWN);
  delay(SERVO_MOVE_DELAY);
  servo.detach();
  delay(SERVO_DETACH_DELAY);
}

#define IR4_OBSTACLE_TEST    0 // 장애물 테스트 모드 플래그

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

// 주행 상태 변수
int runState = 0;   // 0: 대기, 1~: 동작 상태
int selectedPath;   // 1=좌,2=중,3=우 경유로 저장

// 90도 좌회전 함수 (라인 인식에 의한 회전)
void turnLeft90Deg() {
  stopMotors(); 
  delay(STOP_DELAY_BEFORE_TURN);
  driveMotors(DIR_BACKWARD,80, DIR_BACKWARD,80); 
  delay(TURN_EXTRA_DELAY);
  stopMotors(); 
  delay(STOP_DELAY_BEFORE_TURN);

  // 1차 회전: 흰색 벗어날 때까지
  turnLeft(TURN_SPEED);
  while(analogRead(IR_PIN_LEFT) < IR_THRESHOLD_WHITE) 
    delay(1);
  delay(STOP_DELAY_BEFORE_TURN);

  // 2차 회전: 검은색 만날 때까지
  turnLeft(TURN_SPEED);
  while(analogRead(IR_PIN_LEFT) > IR_THRESHOLD_BLACK) 
    delay(1);
  delay(INTERSECTION_WAIT);

  moveForward(TURN_SPEED); 
  delay(LINE_SKIP_DELAY);
  turnLeft(TURN_SPEED); 
  delay(TURN_FINAL_FORWARD);
  moveForward(DEFAULT_SPEED);
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
  if(v1 > IR_THRESHOLD_BLACK && v2 > IR_THRESHOLD_BLACK) {
    switch(runState) {
      case 1:
        // 중앙 경유로 장애물 검사
        stopMotors(); 
        delay(INTERSECTION_WAIT);

        if(analogRead(IR_PIN_FRONT_CENTER)<OBSTACLE_THRESHOLD) {
          runState=11; 
          turnLeft90Deg();
        } else {
          selectedPath=2; 
          runState=102;
          moveForward(DEFAULT_SPEED); 
          delay(SKIP_LINE_DURATION);
        }
        break;
      case 11:  // 좌측(1번 경유로) 장애물 검사하기 위해 우회전
        turnRight90Deg();  // 우회전 후에 정면을 바라보도록
        runState = 12;  // 약간 전진(시간차 정지 후 거리 측정)
        // TickStart = millis(); // POWER가 켜진 후 경과 시간(ms)
        break;
      case 13: // 우측 경유로로 가기 위해 뒤돌아 가다 정지선
        turnLeft90Deg();
        runState = 14;  // 우측(3번) 경유로로 이동시작
        break;
      case 14:  // 우측(3번) 경유로 장애물 검사하러 중앙선 통과
        runState = 15; // 우측 장애물 검사하러 전진
        moveForward(DEFAULT_SPEED);
        delay(SKIP_LINE_DURATION);
        break;
      case 15:  // 우측 장애물 검사는 생략 (마지막 남은 경유로)
        turnLeft90Deg();
        runState = 103; // 우측 경유로 진입하여 전진
        break;  
      case 101: // 좌측 경유로 통과
        turnRight90Deg();
        runState = 111; // 우회전하여 목표지점으로 전진
        break;  
      case 102: // 중앙 경유로 통과
        moveForward(DEFAULT_SPEED);
        delay(DEFAULT_SPEED);
        runState = 201; // 목표지점으로 전진
        break;  
      case 103: // 우측 경유로 통과
        turnLeft90Deg();
        runState = 113; // 좌회전하여 목표지점으로 전진
        break;  
      case 111: // 좌측 경유 목표지점 진입 교차로 도달
        turnLeft90Deg();
        runState = 201; // 좌회전하여 목표지점으로 전진
        break;  
      case 113: // 우측 경유 목표지점 진입 교차로 도달
        turnRight90Deg();
        runState = 201; // 우회전하여 목표지점으로 전진
        break;
      case 201: // 중간 목표지점 도착 전진
        stopMotors();
        liftUp();
  
        tone(BUZZER_PIN, 262);  // 도(4옥타브 C)
        delay(100);
        tone(BUZZER_PIN, 330);  // 미(4옥타브 E)
        delay(100);
        tone(BUZZER_PIN, 392);  // 솔(4옥타브 G)
        delay(250);
        noTone(BUZZER_PIN);     // 음소거(mute)
        
        delay(2000);  // 팔레트(화물) 싣는 시간
  
        runState = 203;  // 뒤돌아 홈으로 출발
        
        turnLeft180Deg(true); // (정지선에서) 살짝 후진하고 회전
        break;
  
      case 203: // 올때 선택한 경로에 따라서 되돌아갈 방향 선택
        if(selectedPath == 1) // 좌측 경유로
        {
          runState = 301;  // 좌측 경유로 따라서 귀환
          turnRight90Deg();
        }
        else if(selectedPath == 3) // 우측 경유로
        {
          runState = 401;  // 우측 경유로 따라서 귀환
          turnLeft90Deg();
        }
        else // SelectedPath == 2, 중앙 경유로로 직진
        {
          runState = 204;  // 교차로를 지나 중앙으로 귀환
          moveForward(DEFAULT_SPEED);
          delay(SKIP_LINE_DURATION);
        }
        break;        
      case 204: // 귀환 중 중앙 경유로 통과
        runState = 501;  // 귀환 진입
        moveForward(DEFAULT_SPEED);
        delay(SKIP_LINE_DURATION);
        break;
      case 301: // 귀환 중 좌측 경유로 진입
        runState = 302;  // 좌측 경유
        turnLeft90Deg();
        break;  
      case 302: // 귀환 중 좌측 경유로 통과
        runState = 303;  // 좌측 경유
        turnLeft90Deg();
        break;  
      case 303: // 귀환 중 (좌측 경유로 통과 후) 좌측 진입
        runState = 501;  // 귀환 진입
        turnRight90Deg();
        break;  
      case 401: // 귀환 중 우측 경유로 진입
        runState = 402;  // 우측 경유
        turnRight90Deg();
        break;  
      case 402: // 귀환 중 우측 경유로 통과
        runState = 403;  // 우측 경유
        turnRight90Deg();
        break;  
      case 403: // 귀환 중 (우측 경유로 통과 후) 우측 진입
        runState = 501;  // 귀환 진입
        turnLeft90Deg();
        break;  
      case 501: // A지점 정지선 도착 후 화물 내리고 180도 회전
        stopMotors();
  
        liftDown(); // 리프터를 아래로 내림 xxx
  
        tone(BUZZER_PIN, 392);  // 솔(4옥타브 G)
        delay( 150 );
        tone( BUZZER_PIN, 330 );  // 미(4옥타브 E)
        delay( 150 );
        tone( BUZZER_PIN, 262 );  // 도(4옥타브 C)
        delay( 250 );
        noTone( BUZZER_PIN );     // 음소거(mute)
        
        delay( 2000 ); // 팔레트(화물) 내리는 시간
  
        runState = 999;  // 다시 출발 대기 상태로 위치
        
        turnLeft180Deg(true); // (정지선에서) 살짝 후진하고 회전
        break;
      case 999: // (최종 180도 회전 후) 약간 후진 후 정지
        stopMotors(); // 정지
        driveMotors(DIR_BACKWARD, DEFAULT_SPEED - 10, DIR_BACKWARD, DEFAULT_SPEED - 10);
        delay( 160 );
        stopMotors(); // 완료 정지
  
        tone( BUZZER_PIN, 330 );  // 미(4옥타브 E)
        delay( 100 );
        tone( BUZZER_PIN, 262 );  // 도(4옥타브 C)
        delay( 250 );
        noTone( BUZZER_PIN );     // 음소거(mute)
    
        runState = 0;  // 출발 대기
        break;
      }
    }
  else if(v1>IR_THRESHOLD_BLACK) turnLeft(DEFAULT_SPEED);
  else if(v2>IR_THRESHOLD_BLACK) turnRight(DEFAULT_SPEED);
  else moveForward(DEFAULT_SPEED);
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
        tone(BUZZER_PIN,262); 
        delay(INTERSECTION_WAIT);
        tone(BUZZER_PIN,330); 
        delay(TURN_EXTRA_DELAY);
        noTone(BUZZER_PIN);
      }
      delay(INTERSECTION_WAIT);
      break;
    case 12:
      // 좌측 경유로 장애물 측정
      if(millis()-lineTraceStartTime>160) {
        stopMotors(); 
        delay(INTERSECTION_WAIT);
        if(analogRead(IR_PIN_FRONT_CENTER)<OBSTACLE_THRESHOLD) {
          selectedPath=3; runState=13;
          turnLeft180Deg(false); 
        } else {
          selectedPath=1; 
          runState=101;
          moveForward(DEFAULT_SPEED);
        }
      }
      break;
    default:
      performLineTracing();
      break;
  }
}
