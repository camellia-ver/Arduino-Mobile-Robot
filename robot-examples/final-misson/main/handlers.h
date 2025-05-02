#pragma once
#include "constants.h"
#include "motor.h"
#include "servo_lifter.h"
#include "rfid.h"
#include "line_trace.h"
#include "utils.h"

unsigned long lastCheck = 0;
const unsigned long checkInterval = 200;

// 외부에서 사용하는 상태 변수, 타이머 선언
extern bool isTurning;
extern unsigned long turnStartTime;

/**
 * RFID 태그를 인식할 때까지 대기하는 함수  
 * RFID 카드가 인식되면 라인트레이싱 상태로 전환하고, 성공음을 울림
 */
void handleWaitForRFID() {
  if (checkRFIDCard()) {  // RFID 카드가 인식되면
    runState = STATE_LINE_TRACE;  // 라인트레이싱 상태로 전환
    beepSuccess();                // RFID 인식 성공 신호음을 울림
  }
  delay(100);  // 안정적인 처리를 위한 짧은 대기
}

/**
 * 교차로에서의 경로 결정을 처리하는 함수  
 * 일정 시간이 경과한 후, 장애물 여부를 판단하여 회전 또는 직진을 결정
 */
void handleTurnDecision() {
  if (millis() - lineTraceStartTime > INTERSECTION_WAIT) {  // 일정 시간 대기 후
    stopMotors();  // 모터 정지
    delay(100);    // 안정적인 판독을 위한 짧은 대기

    // 전방에 장애물이 있을 경우 180도 회전
    if (analogRead(IR_PIN_FRONT_CENTER) < OBSTACLE_THRESHOLD) {
      selectedPath = 3;             // 경로 3(회전) 선택
      runState = STATE_TURN_LEFT_180;  // 180도 좌회전 상태로 전환
      turnLeft180Deg(false);        // 180도 좌회전 시작
    } else {
      selectedPath = 1;             // 경로 1(직진) 선택
      runState = STATE_FORWARD_AFTER_DECISION;  // 직진 상태로 전환
      moveForward(DEFAULT_SPEED);   // 전진 시작
    }
  }
}

/**
 * 180도 좌회전 처리 함수  
 * 회전 시작과 완료 여부를 확인하여 상태를 전환하고 전진을 재개
 */
void handleTurnLeft180() {
  if (!isTurning) {  // 회전이 시작되지 않았다면
    turnLeft180Deg(false);       // 180도 좌회전 시작
    isTurning = true;            // 회전 중임을 표시하는 플래그 설정
    turnStartTime = millis();    // 회전 시작 시간 기록
  } else {
    // 회전이 끝났는지 확인
    if (millis() - turnStartTime > TURN_DURATION_180) {  // 회전 시간이 1200ms 이상이라면
      isTurning = false;                     // 회전 중이 아님을 표시하는 플래그 해제
      runState = STATE_FORWARD_AFTER_TURN;   // 전진 상태로 전환
      moveForward(DEFAULT_SPEED);            // 전진 시작
    }
  }
}