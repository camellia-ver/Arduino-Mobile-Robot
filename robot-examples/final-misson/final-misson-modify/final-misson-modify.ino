#include <deprecated.h>  // 오래된 아두이노 버전에서 필요했음
#include <MFRC522.h>
#include <MFRC522Extended.h>
#include <require_cpp11.h>
#include <SPI.h>

#define RFID_SS_PIN     2 // SPI 통신용 SS(Slave Select)
#define RFID_RST_PIN    4 // RESET 핀

MFRC522 rfidReader(RFID_SS_PIN, RFID_RST_PIN);

////////////////////  부저와 버튼 핀 번호

#define BUZZER_PIN      3  // 부저 핀 번호
#define BUTTON_PIN      A3 // 버튼 핀 번호

////////////////////  AGV 리프터(Lifter) 서보 모터 제어

#include <Servo.h>

#define SERVO_POSITION_DOWN    (90 - 40)   // Down 위치, 각 로봇에 맞도록 [-50 .. -10] 범위에서 조정하세요.
#define SERVO_POSITION_UP      (180 - 10)  // Up 위치 (떨림 방지)
#define SERVO_POSITION_DEFAULT SERVO_POSITION_DOWN   // 기본 위치

#define SERVO_PIN         9   // 리프터(서보)모터 핀 번호

Servo lifterServo;   // 리프터용 서보 모터 인스턴스 선언

void moveLifterToPosition(int position) {
    lifterServo.attach(SERVO_PIN);  // 서보 연결
    delay(10);
    lifterServo.write(position);    // 리프터 위치 설정
    delay(300);
    lifterServo.detach();           // 서보 연결 해제
    delay(10);
}

void raiseLifter() {
    moveLifterToPosition(SERVO_POSITION_UP);
}

void lowerLifter() {
    moveLifterToPosition(SERVO_POSITION_DOWN);
}

//////////////  라인트레이스 IR센서 및 장애물 감지 설정

#define IR_SENSOR_LEFT_PIN        A6 // 1번(왼쪽) IR센서 연결 핀
#define IR_SENSOR_RIGHT_PIN       A7 // 2번(오른쪽) IR센서 연결 핀

// 색상 판단: White=[0..410] .. 560 .. [710..1023]=Black
#define LINE_TRACE_ADJUST         60  // 현재 젤리비의 센서 측정 조정값
#define MAX_WHITE_THRESHOLD       (410 + LINE_TRACE_ADJUST) // 흰색으로 판단하는 최대값
#define MID_THRESHOLD             (560 + LINE_TRACE_ADJUST) // 흑백 판단 경계값(중간값)
#define MIN_BLACK_THRESHOLD       (710 + LINE_TRACE_ADJUST) // 검은색으로 판단하는 최소값

#define IR_SENSOR_FRONT_LEFT_PIN   A0  // 전면 왼쪽 IR센서
#define IR_SENSOR_FRONT_CENTER_PIN A1  // 전면 중앙 IR센서
#define IR_SENSOR_FRONT_RIGHT_PIN  A2  // 전면 오른쪽 IR센서

#define IR_OBSTACLE_TEST           0 // 장애물까지의 거리 측정 테스트용
int obstacleDistance = 1005;        // 장애물 존재 여부 판단 최대 거리값

////////////////////  모터 핀 및 방향 설정

#define MOTOR_LEFT_DIR_PIN    7 // 왼쪽 모터 방향 지정용 핀
#define MOTOR_LEFT_PWM_PIN    5 // 왼쪽 모터 속력 지정용 핀

#define MOTOR_RIGHT_DIR_PIN   8 // 오른쪽 모터 방향 지정용 핀
#define MOTOR_RIGHT_PWM_PIN   6 // 오른쪽 모터 속력 지정용 핀

#define DIRECTION_FORWARD     0 // 전진 방향
#define DIRECTION_BACKWARD    1 // 후진 방향

/**
 * @brief UID를 16진수 문자열로 변환할 때 사용할 버퍼 크기
 * @details MFRC522 라이브러리의 UID 최대 길이는 10바이트입니다.
 */
#define UID_BUFFER_SIZE 21  ///< 10바이트 * 2 + 1

void driveMotors(int dirLeft, int powerLeft, int dirRight, int powerRight) {
    bool leftDirHigh;
    bool rightDirHigh;

    if (dirLeft == DIRECTION_FORWARD)
        leftDirHigh = HIGH;
    else
        leftDirHigh = LOW;

    // 오른쪽 모터는 방향 로직 반대
    if (dirRight == DIRECTION_FORWARD)
        rightDirHigh = LOW;
    else
        rightDirHigh = HIGH;

    digitalWrite(MOTOR_LEFT_DIR_PIN, leftDirHigh);
    analogWrite(MOTOR_LEFT_PWM_PIN, powerLeft);

    digitalWrite(MOTOR_RIGHT_DIR_PIN, rightDirHigh);
    analogWrite(MOTOR_RIGHT_PWM_PIN, powerRight);
}

void moveForward(int power) {
    driveMotors(DIRECTION_FORWARD, power, DIRECTION_FORWARD, power);
}

void moveBackward(int power) {
    driveMotors(DIRECTION_BACKWARD, power, DIRECTION_BACKWARD, power);
}

void turnLeft(int power) {
    driveMotors(DIRECTION_BACKWARD, power, DIRECTION_FORWARD, power);
}

void turnRight(int power) {
    driveMotors(DIRECTION_FORWARD, power, DIRECTION_BACKWARD, power);
}

void stopMotors() {
    analogWrite(MOTOR_LEFT_PWM_PIN, 0);
    analogWrite(MOTOR_RIGHT_PWM_PIN, 0);
}

////////////////////  회전 동작 (각도 단위)

void turnLeft90Degrees() {
    stopMotors();
    delay(50);
    driveMotors(DIRECTION_BACKWARD, 80, DIRECTION_BACKWARD, 80);
    delay(20);
    stopMotors();
    delay(50);

    // 왼쪽 센서로 90도 회전
    turnLeft(100);
    while (analogRead(IR_SENSOR_LEFT_PIN) < MAX_WHITE_THRESHOLD) delay(1);
    delay(40);
    turnLeft(100);
    while (analogRead(IR_SENSOR_LEFT_PIN) > MIN_BLACK_THRESHOLD) delay(1);
    delay(40);
    moveForward(100);
    delay(90);
    turnLeft(100);
    delay(250);
    moveForward(defaultPower);
}

void turnRight90Degrees() {
    stopMotors();
    delay(50);
    driveMotors(DIRECTION_BACKWARD, 80, DIRECTION_BACKWARD, 80);
    delay(20);
    stopMotors();
    delay(50);

    // 오른쪽 센서로 90도 회전
    turnRight(100);
    while (analogRead(IR_SENSOR_RIGHT_PIN) < MAX_WHITE_THRESHOLD) delay(1);
    delay(40);
    turnRight(100);
    while (analogRead(IR_SENSOR_RIGHT_PIN) > MIN_BLACK_THRESHOLD) delay(1);
    delay(40);
    moveForward(100);
    delay(90);
    turnRight(100);
    delay(250);
    moveForward(defaultPower);
}

void turnAround180Degrees(bool backOnStopLine) {
    if (backOnStopLine) {
        driveMotors(DIRECTION_BACKWARD, 70, DIRECTION_BACKWARD, 70);
        while ((analogRead(IR_SENSOR_LEFT_PIN) > MIN_BLACK_THRESHOLD) || (analogRead(IR_SENSOR_RIGHT_PIN) > MIN_BLACK_THRESHOLD)) delay(1);
        delay(130);
    } else {
        driveMotors(DIRECTION_BACKWARD, 90, DIRECTION_BACKWARD, 90);
        delay(150);
        driveMotors(DIRECTION_FORWARD, 0, DIRECTION_FORWARD, 90);
        delay(300);
    }

    driveMotors(DIRECTION_BACKWARD, 100, DIRECTION_FORWARD, 100);
    while (analogRead(IR_SENSOR_LEFT_PIN) < MIN_BLACK_THRESHOLD) delay(1);
    delay(30);
    driveMotors(DIRECTION_BACKWARD, 90, DIRECTION_FORWARD, 90);
    while (analogRead(IR_SENSOR_LEFT_PIN) > MAX_WHITE_THRESHOLD) delay(1);
    moveForward(defaultPower);
}

////////////////////  라인트레이싱 동작

unsigned long tickStart;
int runState = 0;
int selectedPath = 0;
int defaultPower = 80;
int skipLineDuration = 200;

void performLineTracing() {
    int leftValue = analogRead(IR_SENSOR_LEFT_PIN);
    int rightValue = analogRead(IR_SENSOR_RIGHT_PIN);

    if ((leftValue > MIN_BLACK_THRESHOLD) && (rightValue > MIN_BLACK_THRESHOLD)) {
        // 교차로 감지
        switch (runState) {
            case 1: // 중앙 경유로 장애물 검사
                stopMotors();
                delay(100);
                if (analogRead(IR_SENSOR_FRONT_CENTER_PIN) < obstacleDistance) {
                    runState = 11;
                    turnLeft90Degrees();
                } else {
                    selectedPath = 2;
                    runState = 102;
                    moveForward(defaultPower);
                    delay(skipLineDuration);
                }
                break;

            case 11:
                turnRight90Degrees();
                runState = 12;
                tickStart = millis();
                break;

            case 13:
                turnLeft90Degrees();
                runState = 14;
                break;

            case 14:
                runState = 15;
                moveForward(defaultPower);
                delay(skipLineDuration);
                break;

            case 15:
                turnLeft90Degrees();
                runState = 103;
                break;

            case 101:
                turnRight90Degrees();
                runState = 111;
                break;

            case 102:
                moveForward(defaultPower);
                delay(skipLineDuration);
                runState = 201;
                break;

            case 103:
                turnLeft90Degrees();
                runState = 113;
                break;

            case 111:
                turnLeft90Degrees();
                runState = 201;
                break;

            case 113:
                turnRight90Degrees();
                runState = 201;
                break;

            case 201:
                stopMotors();
                raiseLifter();
                tone(BUZZER_PIN, 262); delay(100);
                tone(BUZZER_PIN, 330); delay(100);
                tone(BUZZER_PIN, 392); delay(250);
                noTone(BUZZER_PIN);
                delay(2000);
                runState = 203;
                turnAround180Degrees(true);
                break;

            case 203:
                if (selectedPath == 1) {
                    runState = 301;
                    turnRight90Degrees();
                } else if (selectedPath == 3) {
                    runState = 401;
                    turnLeft90Degrees();
                } else {
                    runState = 204;
                    moveForward(defaultPower);
                    delay(skipLineDuration);
                }
                break;

            case 204:
                runState = 501;
                moveForward(defaultPower);
                delay(skipLineDuration);
                break;

            case 301:
                runState = 302;
                turnLeft90Degrees();
                break;

            case 302:
                runState = 303;
                turnLeft90Degrees();
                break;

            case 303:
                runState = 501;
                turnRight90Degrees();
                break;

            case 401:
                runState = 402;
                turnRight90Degrees();
                break;

            case 402:
                runState = 403;
                turnRight90Degrees();
                break;

            case 403:
                runState = 501;
                turnLeft90Degrees();
                break;

            case 501:
                stopMotors();
                lowerLifter();
                tone(BUZZER_PIN, 392); delay(150);
                tone(BUZZER_PIN, 330); delay(150);
                tone(BUZZER_PIN, 262); delay(250);
                noTone(BUZZER_PIN);
                delay(2000);
                runState = 999;
                turnAround180Degrees(true);
                break;

            case 999:
                stopMotors();
                driveMotors(DIRECTION_BACKWARD, 70, DIRECTION_BACKWARD, 70);
                delay(160);
                stopMotors();
                tone(BUZZER_PIN, 330); delay(100);
                tone(BUZZER_PIN, 262); delay(250);
                noTone(BUZZER_PIN);
                runState = 0;
                break;
        }
    } else if (leftValue > MIN_BLACK_THRESHOLD) {
        turnLeft(defaultPower);
    } else if (rightValue > MIN_BLACK_THRESHOLD) {
        turnRight(defaultPower);
    } else {
        moveForward(defaultPower);
    }
}

////////////////////  메인 프로그램 (setup & loop)

void setup() {
    pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
    pinMode(MOTOR_LEFT_PWM_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM_PIN, OUTPUT);

    pinMode(BUZZER_PIN, OUTPUT);
    noTone(BUZZER_PIN);

    Serial.begin(9600);

    lowerLifter();

    SPI.begin();
    rfidReader.PCD_Init();

#if IR_OBSTACLE_TEST
    while (1) {
        obstacleDistance = analogRead(IR_SENSOR_FRONT_CENTER_PIN);
        Serial.print("obstacleDistance= ");
        Serial.println(obstacleDistance);
        delay(500);
    }
#endif
}

void loop() {
    switch (runState) {
        case 0:
            if (rfidReader.PICC_IsNewCardPresent()) {
                if (rfidReader.PICC_ReadCardSerial()) {
                    runState = 1;
                    tone(BUZZER_PIN, 262); delay(100);
                    tone(BUZZER_PIN, 330); delay(250);
                    noTone(BUZZER_PIN);
                }
            }
            delay(100);
            break;

        case 12:
            if (millis() - tickStart > 160) {
                stopMotors();
                delay(100);
                if (analogRead(IR_SENSOR_FRONT_CENTER_PIN) < obstacleDistance) {
                    selectedPath = 3;
                    runState = 13;
                    turnAround180Degrees(false);
                } else {
                    selectedPath = 1;
                    runState = 101;
                    moveForward(defaultPower);
                }
            }
            break;

        default:
            performLineTracing();
            break;
    }
}