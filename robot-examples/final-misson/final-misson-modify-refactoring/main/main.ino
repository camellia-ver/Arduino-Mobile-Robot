#include "constants.h"
#include "motor.h"
#include "servo_lifter.h"
#include "rfid.h"
#include "line_trace.h"
#include "utils.h"
#include "handlers.h" 

bool isTurning = false;
unsigned long turnStartTime = 0;

void setup() {
  initPins();            // 핀 모드 설정
  Serial.begin(9600);    // 시리얼 통신 시작 (9600bps)
  liftDown();            // 서보 리프터 초기 위치(아래)로 이동
  SPI.begin();           // SPI 통신 시작 (RFID 모듈과의 통신을 위해 필요)
  mfrc522.PCD_Init();    // RFID 리더 초기화
}

void loop() {
  switch (runState) {
    case STATE_WAIT_FOR_RFID:
      handleWaitForRFID();
      break;
    case STATE_TURN_DECISION:
      handleTurnDecision();
      break;
    case STATE_TURN_LEFT_180:
      handleTurnLeft180();
      break;
    default:
      performLineTracing();
      break;
  }
}
