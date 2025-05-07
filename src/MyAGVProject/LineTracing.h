#ifndef ROBOT_CONTROL_H 
#define ROBOT_CONTROL_H 

#include <SPI.h> 
#include "MoveMotor.h"

#define IR_PIN_LEFT      A6    // 왼쪽 바닥 센서
#define IR_PIN_RIGHT     A7    // 오른쪽 바닥 센서

#define IR_CALIB_OFFSET     60 // 센서 보정값
#define IR_THRESHOLD_WHITE (410 + IR_CALIB_OFFSET)  // 흰색 기준 최대값
#define IR_MID_THRESHOLD    (560 + IR_CALIB_OFFSET) // 흑백 중간값
#define IR_THRESHOLD_BLACK (710 + IR_CALIB_OFFSET)  // 검은색 기준 최소값

int DEFAULT_MOTOR_SPEED = 80;   ///< 기본 모터 속도

void performLineTracing(){
    int v1 = analogRead(IR_PIN_LEFT);
    int v2 = analogRead(IR_PIN_RIGHT);

    // 양쪽 IR 센서가 모두 검정(라인)일 때만 직진
    if ((v1 > IR_THRESHOLD_BLACK) && (v2 > IR_THRESHOLD_BLACK))
    {
        driveForward(DEFAULT_MOTOR_SPEED);
    }
    // 라인이 없으면 정지
    else
    {
        haltMotors();
    }
}

#endif // ROBOT_CONTROL_H