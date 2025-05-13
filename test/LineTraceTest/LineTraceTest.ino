/**
 * @file    MyAGVProject.ino
 * @brief   AGV 로봇 메인 코드
 */

#include <SPI.h>               ///< SPI 통신을 위한 표준 라이브러리
#include <MFRC522.h>           ///< RFID 리더용 라이브러리
 
#define RFID_SS_PIN        2  // RFID 리더기 SS(Slave Select) 핀 번호
#define RFID_RST_PIN       4  // RFID 리더기 RESET 핀 번호
 
#define BUZZER_PIN         3  // 부저 출력 핀 번호
 
#define MOTOR_LEFT_DIR_PIN 7  // 왼쪽 모터 방향 제어 핀
#define MOTOR_LEFT_PWM_PIN 5  // 왼쪽 모터 속도(PWM) 제어 핀
#define MOTOR_RIGHT_DIR_PIN 8 // 오른쪽 모터 방향 제어 핀
#define MOTOR_RIGHT_PWM_PIN 6 // 오른쪽 모터 속도(PWM) 제어 핀

#define DIRECTION_FORWARD     0 // 전진 방향
#define DIRECTION_BACKWARD    1 // 후진 방향
 
#define IR_SENSOR_LEFT_PIN    A6  // 라인 트레이싱용 왼쪽 IR 센서 핀
#define IR_SENSOR_RIGHT_PIN   A7  // 라인 트레이싱용 오른쪽 IR 센서 핀

// 색상 판단: White=[0..410] .. 560 .. [710..1023]=Black
#define LINE_TRACE_ADJUST       60  // 현재 젤리비의 센서 측정 조정값
#define MAX_WHITE_THRESHOLD   (410  + LINE_TRACE_ADJUST)
#define MID_THRESHOLD         (560  + LINE_TRACE_ADJUST)
#define MIN_BLACK_THRESHOLD   (710  + LINE_TRACE_ADJUST)

#define SERVO_POSITION_DOWN    (90 - 40)   // Down 위치, 각 로봇에 맞도록 [-50 .. -10] 범위에서 조정하세요.
#define SERVO_POSITION_UP      (180 - 10)  // Up 위치 (떨림 방지)
#define SERVO_POSITION_DEFAULT SERVO_POSITION_DOWN   // 기본 위치 

/**
 * @def UID_BUFFER_SIZE
 * @brief   RFID UID를 16진수 문자열로 변환할 때 사용할 버퍼 크기
 * @details MFRC522 라이브러리의 UID 최대 길이는 10바이트이므로
 *          10바이트×2문자 + 널터미널(1) = 21
 */
#define UID_BUFFER_SIZE 21

/**
 * @def SOFT_TURN_FACTOR
 * @brief 회색 영역 보정 시 모터 출력 비율(백분율)
 * @details 0~100 사이 값, 클수록 보정이 부드러움
 */
#define SOFT_TURN_FACTOR 60

int defaultPower = 80;

MFRC522 rfidReader(RFID_SS_PIN, RFID_RST_PIN);

/**
* @brief 부저로 알림음 재생 (픽업/드롭 공통)
* @param seq     주파수 배열
* @param delays  각 음 길이 배열 (ms)
* @param length  배열 길이
*/
void playToneSequence(const int *seq, const int *delays, int length) {
  for (int i = 0; i < length; i++) {
      tone(BUZZER_PIN, seq[i]);
      delay(delays[i]);
  }
  noTone(BUZZER_PIN);
}

/**
* @brief 픽업 완료 알림음 재생
*/
void playPickupTone() {
  const int notes[]  = {262, 330, 392};
  const int times[]  = {100, 100, 250};
  playToneSequence(notes, times, 3);
}

/**
 * @brief 라인 센서 값 읽기
 * @param[out] leftValue  왼쪽 IR 센서 값 (0~1023)
 * @param[out] rightValue 오른쪽 IR 센서 값 (0~1023)
 */
void readLineSensors(int &leftValue, int &rightValue) {
  leftValue  = analogRead(IR_SENSOR_LEFT_PIN);
  rightValue = analogRead(IR_SENSOR_RIGHT_PIN);
}

int16_t centerValue;
// ---------------------------------------------
// 비례제어 라인트레이싱 함수
// ---------------------------------------------
void proportionalLineTrace(int baseSpeed) {
  int leftValue, rightValue;
  readLineSensors(leftValue, rightValue);

  int16_t turnSpeed = ((rightValue - leftValue) - centerValue) / 8;
  int16_t leftSpeed = baseSpeed + turnSpeed;
  int16_t rightSpeed = baseSpeed - turnSpeed;

  digitalWrite(MOTOR_LEFT_DIR_PIN,
               (leftSpeed  > 0) ? DIRECTION_BACKWARD : DIRECTION_FORWARD);
  digitalWrite(MOTOR_RIGHT_DIR_PIN,
               (rightSpeed > 0) ? DIRECTION_FORWARD : DIRECTION_BACKWARD);

  analogWrite(MOTOR_LEFT_PWM_PIN,  round(abs(leftSpeed)));
  analogWrite(MOTOR_RIGHT_PWM_PIN, round(abs(rightSpeed)));
}

/**
 * @brief RFID UID와 매핑된 좌표 저장용 구조체
 */
struct RFIDCoordinate {
  const char *uid;   ///< 16진수 UID 문자열 (예: "14081B74")
  uint8_t     x;     ///< A 지점 또는 B 지점의 X 좌표
  uint8_t     y;     ///< A 지점 또는 B 지점의 Y 좌표
};

/**
* @brief 등록된 UID-좌표 매핑 테이블
* @details 필요에 따라 여기에 UID와 좌표를 추가하세요.
*/
const RFIDCoordinate rfidMap[] = {
  { "14081B74", 1, 2 },
  { "A1B2C3D4", 3, 4 },
  { "DEADBEEF", 5, 6 },
  { "640FCF73", 2, 0 },
  // … 추가 매핑 …
};
const size_t rfidMapSize = sizeof(rfidMap) / sizeof(rfidMap[0]);

/**
* @brief UID 바이트 배열을 16진수 문자열로 변환
* @param uidBytes  UID 바이트 배열
* @param uidLen    UID 길이 (MFRC522::Uid.size)
* @param outStr    변환된 문자열을 저장할 버퍼. 최소 (uidLen*2 + 1) 크기여야 함.
*/
void uidToHexString(byte *uidBytes, byte uidLen, char *outStr) {
  for (byte i = 0; i < uidLen; i++) {
      byte hi = uidBytes[i] >> 4;
      byte lo = uidBytes[i] & 0x0F;
      outStr[i*2]     = hi  < 10 ? '0' + hi  : 'A' + (hi - 10);
      outStr[i*2 + 1] = lo  < 10 ? '0' + lo  : 'A' + (lo - 10);
  }
  outStr[uidLen*2] = '\0';
}

/**
* @brief RFID의 UID 문자열에 대응하는 좌표를 검색
* @param uidStr 16진수 UID 문자열
* @param x      찾은 X 좌표 (출력)
* @param y      찾은 Y 좌표 (출력)
* @return 매핑이 있으면 true, 없으면 false
*/
bool getCoordinatesFromUID(const char *uidStr, uint8_t &x, uint8_t &y) {
  for (size_t i = 0; i < rfidMapSize; i++) {
      if (strcmp(uidStr, rfidMap[i].uid) == 0) {
          x = rfidMap[i].x;
          y = rfidMap[i].y;
          return true;
      }
  }
  return false;
}

/**
 * @brief RFID 카드에서 UID를 읽고 좌표를 반환
 * @param[out] x A 지점의 X 좌표
 * @param[out] y A 지점의 Y 좌표
 * @return 성공적으로 매핑된 경우 true, 아니면 false
 */
bool readCoordinatesFromRFID(uint8_t &x, uint8_t &y) {
  // 새 카드가 닿았는지 확인
  if (!rfidReader.PICC_IsNewCardPresent() ||
      !rfidReader.PICC_ReadCardSerial()) {
      return false;
  }

  // UID를 16진수 문자열로 변환
  char uidStr[UID_BUFFER_SIZE];
  uidToHexString(rfidReader.uid.uidByte,
                 rfidReader.uid.size,
                 uidStr);

  // 매핑 테이블에서 좌표 검색
  if (getCoordinatesFromUID(uidStr, x, y)) {
      return true;
  } else {
      return false;
  }
}

/**
* @brief 교차로(정지선) 감지, 빠른 응답
* @return 교차로(양쪽 센서 모두 검정) 감지 시 true 반환
*/
bool isIntersection() {
  int leftValue, rightValue;
  readLineSensors(leftValue, rightValue);
  return (leftValue > MIN_BLACK_THRESHOLD && rightValue > MIN_BLACK_THRESHOLD);
}

/**
* @brief 모터 정지
*/
void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM_PIN,  0);
  analogWrite(MOTOR_RIGHT_PWM_PIN, 0);
}

/**
* @brief 전진
* @param power 모터 속도 (0~255)
*/
void moveForward(int power) {
  driveMotors(DIRECTION_FORWARD, power, DIRECTION_FORWARD, power);
}

/**
* @brief 좌회전
* @param power 모터 속도 (0~255)
*/
void turnLeft(int power) {
  driveMotors(DIRECTION_BACKWARD, power, DIRECTION_FORWARD, power);
}

/**
* @brief 우회전
* @param power 모터 속도 (0~255)
*/
void turnRight(int power) {
  driveMotors(DIRECTION_FORWARD, power, DIRECTION_BACKWARD, power);
}

/**
 * @brief 모터 구동 함수
 * @param dirLeft   왼쪽 모터 방향 (DIRECTION_FORWARD / DIRECTION_BACKWARD)
 * @param powerLeft 왼쪽 모터 속도 (0~255)
 * @param dirRight  오른쪽 모터 방향 (DIRECTION_FORWARD / DIRECTION_BACKWARD)
 * @param powerRight 오른쪽 모터 속도 (0~255)
 */
void driveMotors(int dirLeft, int powerLeft, int dirRight, int powerRight) {
  // 방향 논리 결정
  bool leftDirHigh  = (dirLeft  == DIRECTION_FORWARD) ? HIGH : LOW;
  // 오른쪽 모터 방향 반전
  bool rightDirHigh = (dirRight == DIRECTION_FORWARD) ? LOW  : HIGH;

  digitalWrite(MOTOR_LEFT_DIR_PIN,  leftDirHigh);
  analogWrite(MOTOR_LEFT_PWM_PIN,   powerLeft);
  digitalWrite(MOTOR_RIGHT_DIR_PIN, rightDirHigh);
  analogWrite(MOTOR_RIGHT_PWM_PIN,  powerRight);
}


/**
* @brief 하드웨어 초기화
* @details 각종 핀 모드 설정, 시리얼 통신, SPI, RFID 초기화 및 리프터 기본 위치 설정
*/
void setup() {
  // 모터 핀 초기화
  pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_PWM_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM_PIN, OUTPUT);

  // 부저 초기화
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);  // 초기 상태로 소리 없음

  // IR 센서(아날로그 입력) 초기화
  pinMode(IR_SENSOR_LEFT_PIN, INPUT);
  pinMode(IR_SENSOR_RIGHT_PIN, INPUT);

  // 시리얼 통신 시작
  Serial.begin(9600);

  // SPI 및 RFID 리더 초기화
  SPI.begin();
  rfidReader.PCD_Init();

  int rightValue,leftValue;
  readLineSensors(leftValue,rightValue);
  centerValue = rightValue - leftValue;
}

void loop() {
  uint8_t rx, ry;

  // 1) RFID 카드 태그 감지 대기
  if (readCoordinatesFromRFID(rx, ry)) {
    Serial.println(F(">> RFID 태그 감지: 테스트 시작"));
    
    // @brief RFID 인식 직후 부저 알림
    playPickupTone();
    delay(500);

    while(!isIntersection()){
      proportionalLineTrace(defaultPower);
    }
    stopMotors();
    delay(500);
  
    // 교차점 도착 후 90도 우회전
    rotate90(true, defaultPower);
    delay(500);

    while(!isIntersection()){
      proportionalLineTrace(defaultPower);
    }
    stopMotors();
    delay(500);

    // 교차점 도착 후 90도 좌회전
    rotate90(false, defaultPower);
    delay(500);
  }

  delay(10);
}

/**
 * @brief  교차로에서 90도 회전 (중앙 정렬 없이)
 * @param  clockwise  true: 시계방향(우회전), false: 반시계방향(좌회전)
 * @param  speed      회전 속도 (0~255)
 */
void rotate90(bool clockwise, int speed) {
  int lv, rv;

  // 0) 관성 보정: 미세 후진
  stopMotors();
  delay(50);
  driveMotors(DIRECTION_BACKWARD, speed/2,
              DIRECTION_BACKWARD, speed/2);
  delay(20);
  stopMotors();
  delay(50);

  // 1) 회전 시작
  if (clockwise) {
    driveMotors(DIRECTION_FORWARD,  speed,
                DIRECTION_BACKWARD, speed);
  } else {
    driveMotors(DIRECTION_BACKWARD, speed,
                DIRECTION_FORWARD,  speed);
  }

  // 2) 흰색(라인 바깥) 감지 → 교차로 탈출
  if (clockwise) {
    while (true) {
      readLineSensors(lv, rv);
      if (lv < MIN_BLACK_THRESHOLD) break;
    }
  } else {
    while (true) {
      readLineSensors(lv, rv);
      if (rv < MIN_BLACK_THRESHOLD) break;
    }
  }
  delay(40);

  // 3) 다음 검정 라인(정지선) 감지 → 90도 회전 완료
  if (clockwise) {
    while (true) {
      readLineSensors(lv, rv);
      if (lv > MIN_BLACK_THRESHOLD) break;
    }
  } else {
    while (true) {
      readLineSensors(lv, rv);
      if (rv > MIN_BLACK_THRESHOLD) break;
    }
  }
  stopMotors();
  delay(50);

  // 4) 짧게 전진해서 라인 위로 복귀
  driveMotors(DIRECTION_FORWARD, speed/2,
              DIRECTION_FORWARD, speed/2);
  while (true) {
    readLineSensors(lv, rv);
    // 둘 중 하나라도 라인을 감지하면 멈춤
    if (lv > MIN_BLACK_THRESHOLD || rv > MIN_BLACK_THRESHOLD) break;
  }
  stopMotors();
}
