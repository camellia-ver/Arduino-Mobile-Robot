#pragma once
#include <Servo.h>
#include "constants.h"

// 서보 모터 객체 선언
Servo servo;

/**
 * 리프터를 위로 올리는 함수
 * 서보 모터를 제어하여 리프터를 올립니다.
 * 1. 서보 모터를 연결
 * 2. 지정된 위치로 이동
 * 3. 잠시 대기 후 서보 모터 연결 해제
 */
void liftUp() {
  servo.attach(SERVO_PIN);           // 서보 핀에 연결
  delay(SERVO_ATTACH_DELAY);         // 잠시 대기 (서보 연결 대기 시간)
  servo.write(SERVO_POS_UP);         // 서보를 위로 올리는 위치로 설정
  delay(SERVO_MOVE_DELAY);           // 서보 이동 대기 시간
  servo.detach();                    // 서보 모터 연결 해제
  delay(SERVO_DETACH_DELAY);         // 서보 연결 해제 후 대기
}

/**
 * 리프터를 아래로 내리는 함수
 * 서보 모터를 제어하여 리프터를 내립니다.
 * 1. 서보 모터를 연결
 * 2. 지정된 위치로 이동
 * 3. 잠시 대기 후 서보 모터 연결 해제
 */
void liftDown() {
  servo.attach(SERVO_PIN);           // 서보 핀에 연결
  delay(SERVO_ATTACH_DELAY);         // 잠시 대기 (서보 연결 대기 시간)
  servo.write(SERVO_POS_DOWN);       // 서보를 아래로 내리는 위치로 설정
  delay(SERVO_MOVE_DELAY);           // 서보 이동 대기 시간
  servo.detach();                    // 서보 모터 연결 해제
  delay(SERVO_DETACH_DELAY);         // 서보 연결 해제 후 대기
}
