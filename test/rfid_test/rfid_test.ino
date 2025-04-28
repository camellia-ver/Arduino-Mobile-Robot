#include <Wire.h>
#include <Adafruit_PN532.h>

#define SDA_PIN 2
#define SCL_PIN 3
Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);

void setup() {
  Serial.begin(115200);
  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("RFID 리더 오류!");
    while (1);
  }
  nfc.SAMConfig();
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
  uint8_t success;
  uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
  uint8_t uidLength;

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  if (success) {
    String uidString = "";
    for (int i = 0; i < uidLength; i++) {
      uidString += String(uid[i], HEX);
    }
    return uidString;
  }
  return ""; // RFID 읽기 실패
}
