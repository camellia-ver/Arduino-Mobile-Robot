#ifndef ROBOT_CONTROL_H 
#define ROBOT_CONTROL_H 

#include <SPI.h> 
#include <Servo.h>

/// @brief 서보 모터의 이동 범위 설정 (기준값에서 ±값)
#define SERVO_RANGE 20

/// @brief 리프터 아래 위치 각도
#define SERVO_DOWN  (90 - SERVO_RANGE)

/// @brief 리프터 위 위치 각도
#define SERVO_UP    (180 - SERVO_RANGE)

/// @brief 리프터 기본 위치 (초기값)
#define SERVO_DEF   SERVO_DOWN

/// @brief 리프터 서보 모터가 연결된 핀 번호
#define pinServo 9

/// @brief 리프터용 서보 모터 객체
Servo servo;

/**
 * @brief 리프터를 지정한 각도로 이동시킵니다.
 * 
 * @param directMove 이동할 서보 모터 각도 (ex: SERVO_UP 또는 SERVO_DOWN)
 */
void moveLifter(int directMove) {
    servo.attach(pinServo);  ///< 서보 모터 연결
    delay(10);               ///< 안정화 대기
    
    servo.write(directMove); ///< 지정한 각도로 이동
    delay(300);              ///< 동작 완료까지 대기
    
    servo.detach();          ///< 서보 모터 연결 해제 (전력 절약)
}

/**
 * @brief 리프터를 위로 올립니다.
 */
void liftUp() {
    moveLifter(SERVO_UP);
}

/**
 * @brief 리프터를 아래로 내립니다.
 */
void liftDown() {
    moveLifter(SERVO_DOWN);
}

#endif // ROBOT_CONTROL_H