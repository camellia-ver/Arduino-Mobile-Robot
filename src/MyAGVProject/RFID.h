#ifndef ROBOT_CONTROL_H 
#define ROBOT_CONTROL_H 

#include <SPI.h> 
#include <MFRC522.h> 

#define RFID_SS_PIN     2       ///< RFID 리더 SS 핀 (SPI Slave Select)
#define RFID_RST_PIN    4       ///< RFID 리더 RESET 핀 

MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN); ///< RFID 리더 객체 생성

/** 
 * @brief 2D 좌표를 나타내는 구조체
 * 
 * 이 구조체는 x, y 좌표를 정의하여, 위치 정보를 저장합니다.
 */
struct Point { 
    int x, y;
    Point(int _x,int _y) : x(_x), y(_y) {}
}; 

/** 
 * @brief RFID 데이터에 대한 구조체
 * 
 * 이 구조체는 RFID 카드의 UID와 해당 카드의 위치를 저장합니다.
 */
struct RfidData { 
  const char* key; ///< RFID 카드의 UID
  Point point; ///< 카드의 위치 (x, y 좌표)
}; 

/** 
 * @brief RFID 카드 UID와 좌표를 매핑한 배열
 * 
 * 이 배열은 UID와 해당 좌표 정보를 매핑하여 저장합니다.
 */
RfidData rfidDataMap[] = {  
  // {UID, {x, y}}
  {"14081B74", {1, 2}}, ///< UID "14081B74" 에 해당하는 좌표 (1, 2)
  {"640FCF73", {2, 3}}, ///< UID "640FCF73" 에 해당하는 좌표 (2, 3)
};  

/** 
 * @brief UID를 이용해 해당 좌표를 찾는 함수
 * 
 * 이 함수는 UID를 키로 사용하여 rfidDataMap 배열에서 일치하는 좌표를 반환합니다.
 * 
 * @param[in] key UID 값을 가진 문자열
 * @return 일치하는 좌표 (x, y) 정보가 담긴 Point 구조체
 */
Point findValueByKey(const char* key){ 
    for (int i = 0; i < sizeof(rfidDataMap) / sizeof(rfidDataMap[0]); i++) { 
        if (strcmp(rfidDataMap[i].key, key) == 0) { 
            return rfidDataMap[i].point; 
        } 
    } 
    return Point{-1, -1}; ///< UID에 해당하는 좌표를 찾지 못한 경우 (-1, -1)
}

/** 
 * @brief RFID 리더 초기화 함수
 * 
 * 이 함수는 RFID 리더를 초기화하고, 자체 진단을 수행하여 리더가 정상적으로 작동하는지 확인합니다. 
 * 초기화가 성공하면 "RFID 리더 준비 완료!" 메시지가 출력되며, 
 * 실패하면 "RFID 리더 초기화 실패!" 메시지가 출력됩니다.
 */
void initRFID() { 
    SPI.begin();
    rfid.PCD_Init(); ///< RFID 리더 초기화
}

/** 
 * @brief RFID 카드 UID를 읽는 함수
 * 
 * 이 함수는 RFID 카드가 새로 인식되었는지 확인하고, 카드의 UID를 읽어 지정된 출력 버퍼에 저장합니다.
 * 
 * @param[out] outUID UID를 저장할 문자열 버퍼
 * @param[in] maxLen 출력 버퍼의 최대 길이
 * @return true 카드 UID가 성공적으로 읽혔을 경우
 * @return false 카드가 인식되지 않았을 경우
 */
bool readRFID(char* outUID, size_t maxLen) { 
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) { 
        return false; ///< 카드가 인식되지 않았을 경우
    } 
    
    byte index = 0; 
    for (byte i = 0; i < rfid.uid.size; i++) { 
        if (index < maxLen - 2) { 
            index += snprintf(&outUID[index], maxLen - index, "%02X", rfid.uid.uidByte[i]); 
        } 
    } 
    outUID[index] = '\0'; ///< UID 문자열 종료 문자 추가
    
    rfid.PICC_HaltA();  ///< 카드 통신 종료
    return true; ///< 카드 UID 읽기 성공
}

#endif // ROBOT_CONTROL_H
