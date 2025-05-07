#include <Arduino.h>

#define TEST_SPEED 150 // 테스트 시 사용할 속도 (0~255)
#define TEST_DURATION 5000 // 테스트 시간 (밀리초)

#define PIN_LEFT_DIR    7   ///< 왼쪽 모터 방향 제어 핀
#define PIN_LEFT_PWM    5   ///< 왼쪽 모터 속도 제어 핀
#define PIN_RIGHT_DIR   8   ///< 오른쪽 모터 방향 제어 핀
#define PIN_RIGHT_PWM   6   ///< 오른쪽 모터 속도 제어 핀

#define DIRECTION_FORWARD   0  ///< 전진 방향
#define DIRECTION_BACKWARD  1  ///< 후진 방향

int defaultMotorSpeed = 100;   ///< 기본 모터 속도

float LEFT_MOTOR_RATIO = 1.00;   ///< 왼쪽 모터 보정 비율
float RIGHT_MOTOR_RATIO = 1.00;  ///< 오른쪽 모터 보정 비율


void initMotor() {
    pinMode(PIN_LEFT_DIR, OUTPUT);
    pinMode(PIN_LEFT_PWM, OUTPUT);
    pinMode(PIN_RIGHT_DIR, OUTPUT);
    pinMode(PIN_RIGHT_PWM, OUTPUT);
}
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
    analogWrite(PIN_LEFT_PWM, leftSpeed * LEFT_MOTOR_RATIO);
  
    digitalWrite(PIN_RIGHT_DIR, rightDirSignal);
    analogWrite(PIN_RIGHT_PWM, rightSpeed * RIGHT_MOTOR_RATIO);
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



// 테스트용 함수
void testMotorCalibration() {
    // 좌우 모터를 동일한 속도로 구동
    Serial.begin(9600); // 시리얼 통신 초기화
    initMotor(); // 모터 핀 초기화
    
    // 전진 동작 시작
    Serial.println("Starting test...");
    driveForward(TEST_SPEED);
    delay(TEST_DURATION); // 일정 시간 동안 구동
    haltMotors(); // 모터 정지
    Serial.println("Test completed!");

    // 테스트 종료 후, 보정 비율을 조정하려면
    // 각 모터의 주행 상태를 확인하여 leftMotorRatio 또는 rightMotorRatio 값을 조정
    Serial.println("Adjust motor calibration ratios accordingly.");
}

void setup() {
    testMotorCalibration(); // 보정 비율 테스트 실행
}

void loop() {
    // 보정 비율을 계속해서 테스트하려면, 반복적으로 실행할 수 있음
}
