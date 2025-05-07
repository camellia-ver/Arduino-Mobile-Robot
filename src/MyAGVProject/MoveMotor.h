#ifndef ROBOT_CONTROL_H 
#define ROBOT_CONTROL_H 

#include <SPI.h> 

#define PIN_LEFT_DIR    7   ///< 왼쪽 모터 방향 제어 핀
#define PIN_LEFT_PWM    5   ///< 왼쪽 모터 속도 제어 핀
#define PIN_RIGHT_DIR   8   ///< 오른쪽 모터 방향 제어 핀
#define PIN_RIGHT_PWM   6   ///< 오른쪽 모터 속도 제어 핀

#define DIRECTION_FORWARD   0  ///< 전진 방향
#define DIRECTION_BACKWARD  1  ///< 후진 방향

int DEFAULT_MOTOR_SPEED = 80;   ///< 기본 모터 속도
int TURN_POWER  100 // 회전 속력 (주행 속력과는 별개로 고정)

int LEFT_MOTOR_RATIO = 100;   ///< 왼쪽 모터 보정 비율 (백분율)
int RIGHT_MOTOR_RATIO = 100;  ///< 오른쪽 모터 보정 비율 (백분율)

/**
 * @brief 좌우 모터를 개별적으로 제어한다.
 * 
 * @param leftDirection 왼쪽 모터 방향 (0: 전진, 1: 후진)
 * @param leftSpeed 왼쪽 모터 속도 (0~255)
 * @param rightDirection 오른쪽 모터 방향 (0: 전진, 1: 후진)
 * @param rightSpeed 오른쪽 모터 속도 (0~255)
 */
void setMotorControl(int leftDirection, int leftSpeed, int rightDirection, int rightSpeed) {
    bool leftDirSignal = (leftDirection == DIRECTION_FORWARD) ? HIGH : LOW;
    bool rightDirSignal = (rightDirection == DIRECTION_FORWARD) ? LOW : HIGH;
  
    digitalWrite(PIN_LEFT_DIR, leftDirSignal);
    analogWrite(PIN_LEFT_PWM, leftSpeed * LEFT_MOTOR_RATIO / 100);
  
    digitalWrite(PIN_RIGHT_DIR, rightDirSignal);
    analogWrite(PIN_RIGHT_PWM, rightSpeed * RIGHT_MOTOR_RATIO / 100);
}

/**
 * @brief 모터를 정지시킨다.
 */
void haltMotors() {
    analogWrite(PIN_LEFT_PWM, 0);
    analogWrite(PIN_RIGHT_PWM, 0);
}

/**
 * @brief 전진 동작을 한다.
 * 
 * @param speed 전진 속도 (0~255)
 */
void driveForward(int speed) {
    setMotorControl(DIRECTION_FORWARD, speed, DIRECTION_FORWARD, speed);
}

/**
 * @brief 후진 동작을 한다.
 * 
 * @param speed 후진 속도 (0~255)
 */
void driveBackward(int speed) {
    setMotorControl(DIRECTION_BACKWARD, speed, DIRECTION_BACKWARD, speed);
}

/**
 * @brief 좌회전 동작을 한다.
 * 
 * @param speed 좌회전 속도 (0~255)
 */
void turnLeft(int speed) {
    setMotorControl(DIRECTION_BACKWARD, speed * 0.9, DIRECTION_FORWARD, speed);
}

/**
 * @brief 우회전 동작을 한다.
 * 
 * @param speed 우회전 속도 (0~255)
 */
void turnRight(int speed) {
    setMotorControl(DIRECTION_FORWARD, speed, DIRECTION_BACKWARD, speed * 0.9);
}

/**
 * @brief 모터의 핀을 초기화한다.
 */
void initMotor() {
    pinMode(PIN_LEFT_DIR, OUTPUT);
    pinMode(PIN_LEFT_PWM, OUTPUT);
    pinMode(PIN_RIGHT_DIR, OUTPUT);
    pinMode(PIN_RIGHT_PWM, OUTPUT);
}

/**
 * @brief 로봇을 왼쪽으로 90도 회전시킵니다.  
 * 라인트레이서를 기준으로 회전하며, 회전 후 직진도 수행합니다.
 */
void  turnLeft90()
{
    haltMotors(); // 정지
    delay(50); // (달려가던 관성 때문에 약간 앞으로 밀림)
    setMotorControl(DIRECTION_BACKWARD, 80, DIRECTION_BACKWARD, 80); // 아주 살짝 후진
    delay( 20 );
    haltMotors(); // 다시 정지
    delay( 50 );

    // 이하, 왼쪽 LT 센서만 사용하여 왼쪽으로 90도 회전함
    
    turnLeft( TURN_POWER ); // 1차 좌회전 진행 (현재 라인 벗어나기)
    while( analogRead(pinLT1) < LT_MAX_WHITE ) // 흰색인 동안
        delay( 1 );
    delay( 40 );
  
    turnLeft( TURN_POWER ); // 2차 좌회전 진행 (반대쪽 흰바탕 도착)
    while( analogRead(pinLT1) > LT_MIN_BLACK ) // 검은색인 동안
        delay( 1 );
    delay( 40 );

    driveForward( TURN_POWER );
    delay( 90 );
    
    turnLeft( TURN_POWER ); // 좌회전 끝 마무리
    delay( 250 ); // 속도나 무게에 따라 조정

    driveForward( Power ); // 좌회전 이후의 직진 시작
}

/**
 * @brief 로봇을 오른쪽으로 90도 회전시킵니다.  
 * 라인트레이서를 기준으로 회전하며, 회전 후 직진도 수행합니다.
 */
void  turnRight90()
{
    haltMotors(); // 정지
    delay( 50 ); // (달려가던 관성 때문에 약간 앞으로 밀림)
    setMotorControl(DIRECTION_BACKWARD, 80, DIRECTION_BACKWARD, 80); // 아주 살짝 후진
    delay( 20 );
    haltMotors(); // 다시 정지
    delay( 50 );

    // 이하, 오른쪽 LT 센서만 사용하여 오른쪽으로 90도 회전함
    
    turnRight( TURN_POWER ); // 1차 우회전 진행 (현재 라인 벗어나기)
    while( analogRead(pinLT2) < LT_MAX_WHITE ) // 흰색인 동안
        delay( 1 );
    delay( 40 );
    
    turnRight( TURN_POWER ); // 2차 우회전 진행 (반대쪽 흰바탕 도착)
    while( analogRead(pinLT2) > LT_MIN_BLACK ) // 검은색인 동안
        delay( 1 );
    delay( 40 );

    driveForward( TURN_POWER );
    delay( 90 );
    
    turnRight( TURN_POWER ); // 우회전 끝 마무리
    delay( 250 ); // 속도나 무게에 따라 조정

    driveForward( Power ); // 우회전 이후의 직진 시작
}

/**
 * @brief 로봇을 왼쪽으로 180도 회전시킵니다.
 * 
 * @param bBack true일 경우 정지선에서 약간 후진한 후 회전을 시작합니다.  
 * false일 경우 직진 도중 바로 회전을 수행합니다.
 */
void  turnLeft180( bool bBack )
{
  if( bBack ) // (정지선에서) 살짝 후진하고 회전 시작
  {
    setMotorControl(DIRECTION_BACKWARD, 70, DIRECTION_BACKWARD, 70); // 뒤로 후진
    while( (analogRead(pinLT1) > LT_MIN_BLACK) ||
            (analogRead(pinLT2) > LT_MIN_BLACK) )
      delay( 1 ); // 정지선을 벗어날때까지 계속 후진
    delay( 130 ); // 속도나 무게에 따라 조정
  }
  else // 라인 위에서 직진하다가 바로 180도 회전 할 때
  {
    setMotorControl(DIRECTION_BACKWARD, 90, DIRECTION_BACKWARD, 90); // (위치 조정용) 약간 후진
    delay( 150 );
    setMotorControl(DIRECTION_FORWARD, 0, DIRECTION_FORWARD, 90); // LT1을 라인 좌측 밖으로
    delay( 300 );
  }

  setMotorControl(DIRECTION_BACKWARD, TURN_POWER, DIRECTION_FORWARD, TURN_POWER);
  while( analogRead(pinLT1) < LT_MIN_BLACK)
    delay( 1 ); // 왼쪽이 검은색이 아니면 계속 좌회전
  delay( 30 );

  setMotorControl(DIRECTION_BACKWARD, 90, DIRECTION_FORWARD, 90);
  while( analogRead(pinLT1) > LT_MAX_WHITE )
    delay( 1 ); // 왼쪽이 흰색이 아닌 동안 계속 좌회전
  
    driveForward( Power ); // (180도 회전 이후의) 직진 시작
}

#endif // ROBOT_CONTROL_H
