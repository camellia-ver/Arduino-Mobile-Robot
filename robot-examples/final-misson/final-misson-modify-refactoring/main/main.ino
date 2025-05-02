#include "constants.h"
#include "motor.h"
#include "servo_lifter.h"
#include "rfid.h"
#include "line_trace.h"
#include "utils.h"

/**
 * 초기 설정 함수  
 * 핀 초기화, 시리얼 통신 시작, 서보 리프터 초기 위치 설정,  
 * RFID 통신 초기화를 수행합니다.
 */
void setup() {
  initPins();            // 핀 모드 설정
  Serial.begin(9600);    // 시리얼 통신 시작
  liftDown();            // 리프터를 초기 위치(아래)로 이동
  SPI.begin();           // SPI 통신 시작
  mfrc522.PCD_Init();    // RFID 리더 초기화
}

/**
 * 메인 루프 함수  
 * runState 상태에 따라 라인트레이싱, RFID 확인, 회전 동작 등을 수행합니다.
 * 
 * - runState == 0: RFID 카드 인식 대기
 * - runState == 12: 교차로에서 상태 분기 (회전 또는 직진)
 * - 그 외: 라인트레이싱 동작 수행
 */
void loop() {
  switch(runState) {
    case 0:
      // RFID 카드가 인식되면 실행 상태 변경 및 부저 울림
      if (checkRFIDCard()) {
        runState = 1;
        beepSuccess();
      }
      delay(100);
      break;

    case 12:
      // 일정 시간 대기 후 센서를 읽고 회전 또는 직진 결정
      if (millis() - lineTraceStartTime > INTERSECTION_WAIT) {
        stopMotors(); delay(100);
        if (analogRead(IR_PIN_FRONT_CENTER) < OBSTACLE_THRESHOLD) {
          selectedPath = 3;
          runState = 13;
          turnLeft180Deg(false);
        } else {
          selectedPath = 1;
          runState = 101;
          moveForward(DEFAULT_SPEED);
        }
      }
      break;

    default:
      // 기본 동작: 라인트레이싱
      performLineTracing();
      break;
  }
}
