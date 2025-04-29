#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 2  // SDA
#define RST_PIN 4

MFRC522 rfid(SS_PIN, RST_PIN);

void setup() {
  Serial.begin(115200);
  SPI.begin();           // SPI 초기화
  rfid.PCD_Init();       // RFID 초기화
  Serial.println("RFID 리더 준비 완료!");
}

void loop() {
  String rfidUID = readRFID();
  if (rfidUID != "") {
    Serial.print("RFID UID 읽음: ");
    Serial.println(rfidUID);
  }
  delay(1000); // 1초 대기
}

String readRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return "";
  }

  String uidString = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uidString += "0";  // 앞자리가 한 자리일 때 0 추가
    uidString += String(rfid.uid.uidByte[i], HEX);
  }
  rfid.PICC_HaltA();  // 카드 통신 종료
  return uidString;
}
