#pragma once
#include "constants.h"
#include "motor.h"
#include "utils.h"

// 전역 변수 정의
int runState = 0;                  ///< 현재 실행 상태
int selectedPath = 0;             ///< 선택된 경로 번호
unsigned long lineTraceStartTime = 0; ///< 라인트레이싱 시작 시간
int OBSTACLE_THRESHOLD = 1005;    ///< 장애물 감지를 위한 임계값

/**
 * 왼쪽으로 90도 회전하는 함수  
 * 센서와 모터를 활용하여 왼쪽으로 정확히 90도 회전합니다.
 * 
 * 1. 짧게 후진하여 회전을 준비합니다.
 * 2. 왼쪽으로 회전하고 IR 센서로 검정 선을 감지합니다.
 * 3. 감지된 후 정렬을 위해 조금 더 회전하고 전진합니다.
 */
void turnLeft90Deg() {
  stopMotors(); delay(STOP_DELAY_BEFORE_TURN);
  driveMotors(DIR_BACKWARD, 80, DIR_BACKWARD, 80); delay(20);
  stopMotors(); delay(STOP_DELAY_BEFORE_TURN);

  turnLeft(TURN_SPEED);
  while (isWhite(analogRead(IR_PIN_LEFT))) wait(1);
  delay(40);
  while (isBlack(IR_PIN_LEFT)) wait(1);
  delay(40);

  moveForward(TURN_SPEED); delay(TURN_FINAL_FORWARD);
  turnLeft(TURN_SPEED); delay(TURN_EXTRA_DELAY);
  moveForward(DEFAULT_SPEED);
}

/**
 * 오른쪽으로 90도 회전하는 함수  
 * 좌회전과 유사한 방식으로 우회전을 수행합니다.
 * 
 * TODO: 동일한 방식으로 회전 구현 필요
 */
void turnRight90Deg() {
  // TODO: 유사한 방식으로 구현
}

/**
 * 180도 회전하는 함수  
 * 로봇이 제자리에서 180도 회전하도록 동작합니다.
 * 
 * @param bBack 뒤로 이동한 후 회전 여부를 결정하는 플래그
 * 
 * TODO: 회전 로직 구현 필요
 */
void turnLeft180Deg(bool bBack) {
  // TODO: 180도 회전 동작 구현
}

/**
 * 라인트레이싱을 수행하는 함수  
 * IR 센서의 값을 바탕으로 경로를 따라 이동합니다.
 * 
 * - 좌우 모두 검정일 때 상태에 따라 분기 동작 수행
 * - 한쪽 센서만 감지 시 회전
 * - 아무것도 감지되지 않으면 직진
 */
void performLineTracing() {
  if (isBlack(IR_PIN_LEFT) && isBlack(IR_PIN_RIGHT)) {
    switch (runState) {
      case 1:
        stopMotors(); delay(100);
        if (analogRead(IR_PIN_FRONT_CENTER) < OBSTACLE_THRESHOLD) {
          runState = 11;
          turnLeft90Deg();
        } else {
          selectedPath = 2;
          runState = 102;
          moveForward(DEFAULT_SPEED); delay(SKIP_LINE_DURATION);
        }
        break;
      // 추가 상태 분기 가능
    }
  } else if (isBlack(IR_PIN_LEFT)) {
    turnLeft(DEFAULT_SPEED);
  } else if (isBlack(IR_PIN_RIGHT)) {
    turnRight(DEFAULT_SPEED);
  } else {
    moveForward(DEFAULT_SPEED);
  }
}
