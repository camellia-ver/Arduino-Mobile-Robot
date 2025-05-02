#pragma once
#include "constants.h"

/**
 * 일정 시간 동안 대기하는 함수  
 * delay() 함수를 감싸서 공통 대기 용도로 사용합니다.
 * 
 * @param ms 대기 시간 (밀리초)
 */
inline void wait(unsigned long ms) {
  delay(ms);
}

/**
 * 성공 알림음을 재생하는 함수  
 * 부저를 사용하여 두 음으로 구성된 성공 사운드를 재생합니다.
 */
inline void beepSuccess() {
  tone(BUZZER_PIN, 262); delay(100);
  tone(BUZZER_PIN, 330); delay(250);
  noTone(BUZZER_PIN);
}

/**
 * 핀 모드 초기화 함수  
 * 모터, 부저 등 사용되는 핀을 출력 모드로 설정하고 초기화합니다.
 */
inline void initPins() {
  pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_PWM_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);
}

/**
 * 흰색 선 감지 여부 확인 함수  
 * 주어진 센서 값이 흰색 기준 이하인지 판단합니다.
 * 
 * @param value IR 센서 값
 * @return true  흰색 감지 시  
 * @return false 흰색이 아닐 경우
 */
inline bool isWhite(int value) {
  return value < IR_THRESHOLD_WHITE;
}

/**
 * 검은색 선 감지 여부 확인 함수  
 * 주어진 센서 값이 검은색 기준 이상인지 판단합니다.
 * 
 * @param value IR 센서 값
 * @return true  검은색 감지 시  
 * @return false 검은색이 아닐 경우
 */
inline bool isBlack(int value) {
  return value > IR_THRESHOLD_BLACK;
}
