#pragma once
#include <SPI.h>
#include <MFRC522.h>
#include "constants.h"

// MFRC522 RFID 리더 인스턴스 생성
MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN);

/**
 * 새로운 RFID 카드를 감지하고 시리얼을 읽는 함수  
 * MFRC522 라이브러리를 사용하여 RFID 카드의 존재 여부와 UID를 확인합니다.
 * 
 * @return true  새로운 카드가 감지되고 UID가 성공적으로 읽힌 경우  
 * @return false 카드가 없거나 읽기에 실패한 경우
 */
bool checkRFIDCard() {
  return mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial();
}
