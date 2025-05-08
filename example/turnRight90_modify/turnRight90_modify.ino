////////////////////  버튼 핀 번호

#define BUTTON_PIN     A3 // 버턴 핀 번호

////////////////////  AGV 리프터(Lifter) 서보 모터 제어

#include <Servo.h> 

#define SERVO_POSITION_DOWN  (90 - 40)   // Down 위치, 각 로봇에 맞도록 [-50 .. -10] 범위에서 조정하세요.
#define SERVO_POSITION_UP    (180 - 10)  // Up 위치 (떨림 방지)
#define SERVO_POSITION_DEFAULT   SERVO_POSITION_DOWN   // 기본 위치

#define SERVO_PIN   9   // 리프터(서보)모터 핀 번호

Servo lifterServo;   // 리프터용 서보 모터 인스턴스 선언

void moveLifterToPosition(int position) {
  lifterServo.attach(SERVO_PIN);
  delay(10);
  lifterServo.write(position);
  delay(300);
  lifterServo.detach();
  delay(10);
}

void lowerLifter() {
  moveLifterToPosition(SERVO_POSITION_DOWN);
}

//////////////  바닥의 라인트레이스 IR센서

#define IR_SENSOR_LEFT_PIN  A6 // 1번(왼쪽) IR센서 연결 핀
#define IR_SENSOR_RIGHT_PIN  A7 // 2번(오른쪽) IR센서 연결 핀

// 색상 판단: White=[0..410] .. 560 .. [710..1023]=Black
#define LINE_TRACE_ADJUST       60  // 현재 젤리비의 센서 측정 조정값
#define MAX_WHITE_THRESHOLD   410 + LINE_TRACE_ADJUST // 흰색으로 판단하는 최대값
#define MID_THRESHOLD   560 + LINE_TRACE_ADJUST // 흑백 판단 경계값(중간값)
#define MIN_BLACK_THRESHOLD   710 + LINE_TRACE_ADJUST // 검은색으로 판단하는 최소값

////////////////////  모터 1번(왼쪽)과 2번(오른쪽)

#define MOTOR_LEFT_DIR_PIN   7 // 1번(왼쪽)모터 방향 지정용 연결 핀
#define MOTOR_LEFT_PWM_PIN   5 // 1번(왼쪽)모터 속력 지정용 연결 핀

#define MOTOR_RIGHT_DIR_PIN   8 // 2번(오른쪽)모터 방향 지정용 연결 핀
#define MOTOR_RIGHT_PWM_PIN   6 // 2번(오른쪽)모터 속력 지정용 연결 핀

////////////////////  모터 회전 동작

#define DIRECTION_FORWARD   0 // 전진 방향
#define DIRECTION_BACKWARD  1 // 후진 방향

void driveMotors(int dir1, int power1, int dir2, int power2)
{
  boolean dirHighLow1, dirHighLow2;

  if(dir1 == DIRECTION_FORWARD)  // 1번(왼쪽)모터 방향
    dirHighLow1 = HIGH;
  else // BACKWARD
    dirHighLow1 = LOW;
  
  if(dir2 == DIRECTION_FORWARD)  // 2번(오른쪽)모터
    dirHighLow2 = LOW;
  else // BACKWARD
    dirHighLow2 = HIGH;

  digitalWrite(MOTOR_LEFT_DIR_PIN, dirHighLow1);
  analogWrite(MOTOR_LEFT_PWM_PIN, power1);

  digitalWrite(MOTOR_RIGHT_DIR_PIN, dirHighLow2);
  analogWrite(MOTOR_RIGHT_PWM_PIN, power2);
}

void  moveForward( int power )  // 전진
{
  driveMotors(DIRECTION_FORWARD, power, DIRECTION_FORWARD, power);
}

void  moveBackward( int power )  // 후진
{
  driveMotors(DIRECTION_BACKWARD, power, DIRECTION_BACKWARD, power);
}

void  turnLeft( int power )  // 좌회전
{
  driveMotors(DIRECTION_BACKWARD, power, DIRECTION_FORWARD, power);
}

void  turnRight( int power )  // 우회전
{
  driveMotors(DIRECTION_FORWARD, power, DIRECTION_BACKWARD, power);
}

void stopMotors()  // 정지
{
  analogWrite(MOTOR_LEFT_PWM_PIN, 0);
  analogWrite(MOTOR_RIGHT_PWM_PIN, 0);
}

////////////////////  메인 프로그램 (setup & loop)

void  setup()
{
  // 모터 제어 핀들을 모두 출력용으로 설정
  pinMode( MOTOR_LEFT_DIR_PIN, OUTPUT ); // 1번(왼쪽)모터 방향 핀
  pinMode( MOTOR_LEFT_PWM_PIN, OUTPUT ); // 1번(왼쪽)모터 속력 핀
  pinMode( MOTOR_RIGHT_DIR_PIN, OUTPUT ); // 2번(오른쪽)모터 방향 핀
  pinMode( MOTOR_RIGHT_PWM_PIN, OUTPUT ); // 2번(오른쪽)모터 속력 핀

  lowerLifter();  // 리프터를 아래로 내림
}


#define TURN_POWER  100 // 회전 속력 (주행 속력과는 별개로 고정)

int defaultPower = 100;  // 기본 주행(라인트레이싱) 속력 (80~120 정도)

// 주행 상태(이동중 또는 정지) 플래그
bool  isDriving = false; // 처음에는 정지 상태 (UP 버튼으로 변경)

void  simpleLineTracing()
{
  int v1 = analogRead( IR_SENSOR_LEFT_PIN );
  int v2 = analogRead( IR_SENSOR_RIGHT_PIN );

  if( (MIN_BLACK_THRESHOLD < v1) && (MIN_BLACK_THRESHOLD < v2) )
  { // 양쪽 IR센서 모두 검정색을 감지한 경우, (ex) 교차선
    stopMotors(); // 정지
    delay( 50 ); // (달려가던 관성 때문에 약간 앞으로 밀림)
    driveMotors(DIRECTION_BACKWARD, 80, DIRECTION_BACKWARD, 80); // 아주 살짝 후진
    delay( 20 );
    stopMotors(); // 다시 정지
    delay( 50 );

    // 이하, 오른쪽 LT 센서만 사용하여 오른쪽으로 90도 회전함
    
    turnRight( TURN_POWER ); // 1차 우회전 진행 (현재 라인 벗어나기)
    while( analogRead(IR_SENSOR_RIGHT_PIN) < MAX_WHITE_THRESHOLD ) // 흰색인 동안
      delay( 1 );
    delay( 40 );
    
    turnRight( TURN_POWER ); // 2차 우회전 진행 (반대쪽 흰바탕 도착)
    while( analogRead(IR_SENSOR_RIGHT_PIN) > MIN_BLACK_THRESHOLD ) // 검은색인 동안
      delay( 1 );
    delay( 40 );

    moveForward( TURN_POWER );
    delay( 90 );
    
    turnRight( TURN_POWER ); // 우회전 끝 마무리
    delay( 250 ); // 로봇 모양(관성)에 따라 조정이 필요함
    
    moveForward( defaultPower ); // 우회전 이후의 직진 시작
  }
  else if( v1 > MIN_BLACK_THRESHOLD )  // 왼쪽(IR1)만 검정
  {
    turnLeft( defaultPower ); // 좌회전
  }
  else if( v2 > MIN_BLACK_THRESHOLD )  // 오른쪽(IR2)만 검정
  {
    turnRight( defaultPower ); // 우회전
  }
  else  // 양쪽 모두 흰색
  {
    moveForward( defaultPower ); // 전진
  }
}

void  loop()
{
  ////////// 먼저, 버튼 눌림 체크 (버튼이 눌리면 주행 상태 변경)
  
  if( digitalRead(BUTTON_PIN) == 0 ) // 위(앞)쪽 버튼이 눌려진 상태이면
  {
    if( isDriving ) // 만약 현재 주행중 상태이면
    {
      stopMotors(); // 정지
      
      while( digitalRead(BUTTON_PIN) == 0 ) // 버튼이 올려질 때까지 대기
        delay( 10 );
        
      isDriving = false; // 정지 상태로 변경
    }
    else // 만약 현재 정지 상태이면
    {
      while( digitalRead(BUTTON_PIN) == 0 ) // 버튼이 올려질 때까지 대기
        delay( 10 );
        
      isDriving = true; // 주행중 상태로 변경
    }
    delay( 100 ); // 0.1초 지연
  }

  ////////// 주행 상태에 따라 주행 또는 대기
  
  if( isDriving ) // 현재 주행중 상태이면
  {
    simpleLineTracing();  // 라인트레이싱 진행
  }
  else // 정지중 상태이면
  {
    delay( 100 );  // 0.1초 대기
  }
}
