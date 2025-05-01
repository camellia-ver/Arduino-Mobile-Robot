/*******************************************************
 * 프로젝트명: RFID 카드 읽기 
 * 기능: RFID 카드에서 UID 읽어오기
 * 작성자: 조영란
 * 날짜: 2025-05-01
 *******************************************************/

#include <SPI.h>
#include <MFRC522.h>

// ----------------------------
// [1] 상수 및 핀 정의
// ----------------------------
#define SS_PIN 2  // SDA
#define RST_PIN 4

// ----------------------------
// [2] 전역 변수 및 객체
// ----------------------------
MFRC522 rfid(SS_PIN, RST_PIN);

// ----------------------------
// [3] RFID 카드를 읽어 UID를 반환하는 함수
// ----------------------------
bool readRFID(char* outUID, size_t maxLen) {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return false;
  }

  byte index = 0;
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10 && index < maxLen - 1) {
      outUID[index++] = '0';
    }
    if (index < maxLen - 2) {
      index += sprintf(&outUID[index], "%X", rfid.uid.uidByte[i]);
    }
  }
  outUID[index] = '\0';

  rfid.PICC_HaltA();  // 카드 통신 종료
  return true;
}

// ----------------------------
// [4] setup()
// ----------------------------
void setup() {
  Serial.begin(115200);
  SPI.begin();           // SPI 초기화
  rfid.PCD_Init();       // RFID 초기화
  if (!rfid.PCD_PerformSelfTest()) {
    Serial.println("RFID 리더 초기화 실패!");
  } else {
    Serial.println("RFID 리더 준비 완료!");
  }
}

// ----------------------------
// [5] loop()
// ----------------------------
void loop() {
  char rfidUID[30];
  if (readRFID(rfidUID, sizeof(rfidUID))) {
    Serial.print("RFID UID 읽음: ");
    Serial.println(rfidUID);
  }
  delay(1000);
}