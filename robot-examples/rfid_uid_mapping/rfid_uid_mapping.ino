/*******************************************************
 * 프로젝트명: RFID 카드 UID와 좌표 맵핑
 * 기능: RFID 카드에서 읽어온 UID에 해당하는 좌표를 반환
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

struct Point {
  int x;
  int y;
};

struct RfidData {
  const char* key;
  Point point;
};

RfidData rfidDataMap[] = { 
  // {UID, {x, y}}
  {"14081B74",{1,2}},
  {"640FCF73",{2,3}},
};

// ----------------------------
// [4] RFID 카드의 UID에 해당하는 좌표를 반환하는 함수
// ----------------------------
Point findValueByKey(const char* key){
  for (int i=0;i< sizeof(rfidDataMap)/sizeof(rfidDataMap[0]);i++){
    if(strcmp(rfidDataMap[i].key,key)==0){
      return rfidDataMap[i].point;
    }
  }
  return Point{-1, -1};
}

// ----------------------------
// [4] RFID 카드를 읽어 UID를 반환하는 함수
// ----------------------------
bool readRFID(char* outUID, size_t maxLen) {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return false;
  }

  byte index = 0;
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (index < maxLen - 2) {
      index += snprintf(&outUID[index], maxLen - index, "%02X", rfid.uid.uidByte[i]);
    }
  }
  outUID[index] = '\0';

  rfid.PICC_HaltA();  // 카드 통신 종료
  return true;
}

// ----------------------------
// [5] setup()
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
// [6] loop()
// ----------------------------
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