#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 2  // SDA
#define RST_PIN 4

MFRC522 rfid(SS_PIN, RST_PIN);

struct KeyValue {
  const char* key;
  int value;
};

KeyValue keyValueMap[] = {
  {"14081b74",1},
  {"640fcf73",2},
};

int findValueByKey(const char* key){
  for (int i=0;i< sizeof(keyValueMap)/sizeof(keyValueMap[0]);i++){
    if(strcmp(keyValueMap[i].key,key)==0){
      return keyValueMap[i].value;
    }
  }
  return -1;
}

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
    int value = findValueByKey(rfidUID.c_str());
    Serial.println(value);
  }
  delay(100); // 1초 대기
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
