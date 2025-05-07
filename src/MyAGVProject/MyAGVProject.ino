#include <SPI.h>

#include "RFID.h"
#include "MoveMotor.h"
#include "Lifter.h"

void setup() {
    Serial.begin(115200);
    initRFID();  // RFID 초기화
}

void loop() {
    char rfidUID[16];  // 최대 7바이트 UID (14자 + null 문자)

    if (readRFID(rfidUID, sizeof(rfidUID))) {
      Serial.print("RFID UID 읽음: ");
      Serial.println(rfidUID);
  
      Point location = findValueByKey(rfidUID);
      if (location.x != -1) {
        Serial.print("위치: (");
        Serial.print(location.x);
        Serial.print(", ");
        Serial.print(location.y);
        Serial.println(")");
      } else {
        Serial.println("등록되지 않은 카드입니다.");
      }
    }
    delay(1000);
}