#pragma once
#include "constants.h"

/**
 * 두 개의 모터를 동시에 제어하는 함수
 * 주어진 방향과 속도로 왼쪽, 오른쪽 모터를 제어합니다.
 * 
 * @param dir1 왼쪽 모터의 방향 (DIR_FORWARD 또는 DIR_BACKWARD)
 * @param power1 왼쪽 모터의 속도 (0-255 범위)
 * @param dir2 오른쪽 모터의 방향 (DIR_FORWARD 또는 DIR_BACKWARD)
 * @param power2 오른쪽 모터의 속도 (0-255 범위)
 */
void driveMotors(int dir1, int power1, int dir2, int power2) {
  bool dirHigh1 = (dir1 == DIR_FORWARD) ? HIGH : LOW; // 왼쪽 모터 방향 설정
  bool dirHigh2 = (dir2 == DIR_FORWARD) ? LOW  : HIGH; // 오른쪽 모터 방향 설정
  digitalWrite(MOTOR_LEFT_DIR_PIN, dirHigh1);          // 왼쪽 모터 방향 제어
  analogWrite(MOTOR_LEFT_PWM_PIN, power1);             // 왼쪽 모터 속도 제어
  digitalWrite(MOTOR_RIGHT_DIR_PIN, dirHigh2);         // 오른쪽 모터 방향 제어
  analogWrite(MOTOR_RIGHT_PWM_PIN, power2);            // 오른쪽 모터 속도 제어
}

/**
 * 왼쪽 모터와 오른쪽 모터를 모두 전진시키는 함수
 * 지정된 속도로 두 모터를 전진시킵니다.
 * 
 * @param p 전진 속도 (0-255 범위)
 */
void moveForward(int p) {
  driveMotors(DIR_FORWARD, p, DIR_FORWARD, p);  // 양쪽 모터 전진
}

/**
 * 왼쪽 모터와 오른쪽 모터를 모두 후진시키는 함수
 * 지정된 속도로 두 모터를 후진시킵니다.
 * 
 * @param p 후진 속도 (0-255 범위)
 */
void moveBackward(int p) {
  driveMotors(DIR_BACKWARD, p, DIR_BACKWARD, p); // 양쪽 모터 후진
}

/**
 * 왼쪽으로 회전하는 함수
 * 왼쪽 모터는 후진, 오른쪽 모터는 전진시켜 좌회전을 합니다.
 * 
 * @param p 회전 속도 (0-255 범위)
 */
void turnLeft(int p) {
  driveMotors(DIR_BACKWARD, p, DIR_FORWARD, p); // 왼쪽 후진, 오른쪽 전진
}

/**
 * 오른쪽으로 회전하는 함수
 * 왼쪽 모터는 전진, 오른쪽 모터는 후진시켜 우회전을 합니다.
 * 
 * @param p 회전 속도 (0-255 범위)
 */
void turnRight(int p) {
  driveMotors(DIR_FORWARD, p, DIR_BACKWARD, p); // 왼쪽 전진, 오른쪽 후진
}

/**
 * 두 모터를 멈추는 함수
 * 양쪽 모터의 속도를 0으로 설정하여 모터를 정지시킵니다.
 */
void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM_PIN, 0);  // 왼쪽 모터 정지
  analogWrite(MOTOR_RIGHT_PWM_PIN, 0); // 오른쪽 모터 정지
}
