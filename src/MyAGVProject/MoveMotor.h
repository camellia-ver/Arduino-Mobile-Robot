#ifndef ROBOT_CONTROL_H 
#define ROBOT_CONTROL_H 

#include <SPI.h> 

#define PIN_LEFT_DIR    7   ///< 왼쪽 모터 방향 제어 핀
#define PIN_LEFT_PWM    5   ///< 왼쪽 모터 속도 제어 핀
#define PIN_RIGHT_DIR   8   ///< 오른쪽 모터 방향 제어 핀
#define PIN_RIGHT_PWM   6   ///< 오른쪽 모터 속도 제어 핀

#define DIRECTION_FORWARD   0  ///< 전진 방향
#define DIRECTION_BACKWARD  1  ///< 후진 방향

int defaultMotorSpeed = 100;   ///< 기본 모터 속도

float leftMotorRatio = 1.00;   ///< 왼쪽 모터 보정 비율
float rightMotorRatio = 1.00;  ///< 오른쪽 모터 보정 비율

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
    analogWrite(PIN_LEFT_PWM, leftSpeed * leftMotorRatio);
  
    digitalWrite(PIN_RIGHT_DIR, rightDirSignal);
    analogWrite(PIN_RIGHT_PWM, rightSpeed * rightMotorRatio);
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

#endif // ROBOT_CONTROL_H
