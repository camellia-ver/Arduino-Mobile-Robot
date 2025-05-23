/**
 * @file    MyAGVProject.ino
 * @brief   AGV 로봇 메인 코드
 */

#include <SPI.h>               ///< SPI 통신을 위한 표준 라이브러리
#include <MFRC522.h>           ///< RFID 리더용 라이브러리
#include <Servo.h>             ///< 서보 모터 제어 라이브러리
 
#define RFID_SS_PIN        2  // RFID 리더기 SS(Slave Select) 핀 번호
#define RFID_RST_PIN       4  // RFID 리더기 RESET 핀 번호
 
#define BUZZER_PIN         3  // 부저 출력 핀 번호

#define SERVO_PIN          9  // 리프터(서보) 모터 제어 핀 번호
 
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
int turnPower = 100;

enum Direction { NORTH=0, EAST, SOUTH, WEST };
Direction currentDir = NORTH;
 
void turnTo(Direction targetDir) {
    uint8_t diff = (targetDir - currentDir + 4) % 4;
    if      (diff == 1)  turnRight90Degrees();
    else if (diff == 2)  turnAround180Degrees(false);
    else if (diff == 3)  turnLeft90Degrees();
    currentDir = targetDir;
}
 
void turnToNorth(){ turnTo(NORTH); }
void turnToEast() { turnTo(EAST); }
void turnToSouth(){ turnTo(SOUTH); }
void turnToWest() { turnTo(WEST); }
 
const bool gridMap[8][8] = {
    {true,true,true,true,true,true,true,true},
    {true,true,true,true,true,true,true,true},
    {true,true,true,true,true,true,true,true},
    {true,true,true,true,true,true,true,true},
    {true,true,true,true,true,true,true,true},
    {true,true,true,true,true,true,true,true},
    {true,true,true,true,true,true,true,true},
    {true,true,true,true,true,true,true,true},
};

MFRC522 rfidReader(RFID_SS_PIN, RFID_RST_PIN);
Servo lifterServo; 

/**
 * @brief AGV의 현재 X,Y 좌표 (0~7)
 */
uint8_t currentX = 0, currentY = 0;

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

  Serial.println(F("Debug: setup() 시작"));

  // SPI 및 RFID 리더 초기화
  SPI.begin();
  rfidReader.PCD_Init();

  // 리프터 서보 기본 위치로 내리기
  lowerLifter();

  Serial.println(F("Debug: setup() 완료"));
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
* @brief 전진
* @param power 모터 속도 (0~255)
*/
void moveForward(int power) {
  driveMotors(DIRECTION_FORWARD, power, DIRECTION_FORWARD, power);
}

/**
* @brief 후진
* @param power 모터 속도 (0~255)
*/
void moveBackward(int power) {
  driveMotors(DIRECTION_BACKWARD, power, DIRECTION_BACKWARD, power);
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
* @brief 모터 정지
*/
void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM_PIN,  0);
  analogWrite(MOTOR_RIGHT_PWM_PIN, 0);
}

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
* @brief 드롭 완료 알림음 재생
*/
void playDropTone() {
  const int notes[]  = {392, 330, 262};
  const int times[]  = {150, 150, 250};
  playToneSequence(notes, times, 3);
}

/**
* @brief 리프터 지정 위치로 이동
* @param position 서보 각도
*/
void moveLifterToPosition(int position) {
  lifterServo.attach(SERVO_PIN);
  delay(10);
  lifterServo.write(position);
  delay(300);
  lifterServo.detach();
  delay(10);
}

/**
* @brief 리프터 상승
*/
void raiseLifter() {
  moveLifterToPosition(SERVO_POSITION_UP);
}

/**
* @brief 리프터 하강
*/
void lowerLifter() {
  moveLifterToPosition(SERVO_POSITION_DOWN);
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

// ---------------------------------------------
// 비례제어 라인트레이싱 함수
// ---------------------------------------------
void proportionalLineTrace(int baseSpeed) {
  int leftValue, rightValue;
  readLineSensors(leftValue, rightValue);

  // 1) 에러(error) 계산: 오른쪽 검정도 - 왼쪽 검정도
  int16_t error = rightValue - leftValue;

  // 2) 중심 보정값(centerValue)이 필요하면 빼주고, 이 예제에선 0으로 가정
  //    (센서가 완벽히 중앙에 있을 때 error==0이 되면 됩니다)
  // int16_t centerValue = 0;
  // error -= centerValue;

  // 3) 비례 이득(Kp) 조절: 값을 나눠서 부드럽게
  const int16_t Kp_divisor = 8;    
  int16_t turnSpeed = error / Kp_divisor;

  // 4) 좌/우 모터 속도 계산
  int16_t leftSpeed  = baseSpeed + turnSpeed;
  int16_t rightSpeed = baseSpeed - turnSpeed;

  // 5) 속도 한계 걸기
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // 6) 모터 방향 세팅
  //    (DIRECTION_FORWARD, DIRECTION_BACKWARD 은 0/1 로 정의되어 있다고 가정)
  digitalWrite(MOTOR_LEFT_DIR_PIN,
               (leftSpeed  >= 0) ? DIRECTION_BACKWARD : DIRECTION_FORWARD);
  // 오른쪽 모터 방향 신호가 반전되어 있다면 아래처럼 반대로 설정
  digitalWrite(MOTOR_RIGHT_DIR_PIN,
               (rightSpeed >= 0) ? DIRECTION_FORWARD : DIRECTION_BACKWARD);

  // 7) PWM 출력
  analogWrite(MOTOR_LEFT_PWM_PIN,  abs(leftSpeed));
  analogWrite(MOTOR_RIGHT_PWM_PIN, abs(rightSpeed));
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
 * @brief 교차로에서 왼쪽으로 90도 회전 (inner pivot 방식)
 * @details
 * 1) 내측(왼쪽) 바퀴를 정지(pivot)  
 * 2) 외측(오른쪽) 바퀴만 전진  
 * 3) 왼쪽 IR 센서가 분기된 라인을 감지(> MIN_BLACK_THRESHOLD)하면 회전 정지  
 * 4) 이후 일반 라인트레이싱 재개
 * @param speed 회전용 바퀴 속도 (0~255)
 */
void intersectionTurnLeft(uint8_t speed) {
    // 1) 모터 정지
    stopMotors();
    delay(10);

    // 2) 회전 방향 설정: 전진(앞으로)
    digitalWrite(MOTOR_LEFT_DIR_PIN, DIRECTION_FORWARD);   // 전진
    digitalWrite(MOTOR_RIGHT_DIR_PIN, DIRECTION_FORWARD);  // 전진

    // 3) 내측(왼쪽) 바퀴는 고정, 외측(오른쪽)만 속도 부여
    analogWrite(MOTOR_LEFT_PWM_PIN, 0);
    analogWrite(MOTOR_RIGHT_PWM_PIN, speed);

    // 4) 왼쪽 센서가 새로운 라인(branch) 감지할 때까지 회전
    while (analogRead(IR_SENSOR_LEFT_PIN) < MIN_BLACK_THRESHOLD) {
        // 회전 계속
    }

    // 5) 회전 정지
    analogWrite(MOTOR_RIGHT_PWM_PIN, 0);
    delay(10);
}

/**
 * @brief 교차로에서 오른쪽으로 90도 회전 (inner pivot 방식)
 * @details
 * 1) 내측(오른쪽) 바퀴를 정지(pivot)  
 * 2) 외측(왼쪽) 바퀴만 전진  
 * 3) 오른쪽 IR 센서가 분기된 라인을 감지(> MIN_BLACK_THRESHOLD)하면 회전 정지  
 * 4) 이후 일반 라인트레이싱 재개
 * @param speed 회전용 바퀴 속도 (0~255)
 */
void intersectionTurnRight(uint8_t speed) {
    // 1) 모터 정지
    stopMotors();
    delay(10);

    // 2) 회전 방향 설정: 전진
    digitalWrite(MOTOR_LEFT_DIR_PIN, DIRECTION_FORWARD);   
    digitalWrite(MOTOR_RIGHT_DIR_PIN, DIRECTION_FORWARD);  

    // 3) 내측(오른쪽) 바퀴 고정, 외측(왼쪽)만 속도 부여
    analogWrite(MOTOR_RIGHT_PWM_PIN, 0);
    analogWrite(MOTOR_LEFT_PWM_PIN, speed);

    // 4) 오른쪽 센서가 새로운 라인(branch) 감지할 때까지 회전
    while (analogRead(IR_SENSOR_RIGHT_PIN) < MIN_BLACK_THRESHOLD) {
        // 회전 계속
    }

    // 5) 회전 정지
    analogWrite(MOTOR_LEFT_PWM_PIN, 0);
    delay(10);
}

/**
 * @brief 180° 전환을 수행하는 함수 (inner-pivot 방식)
 * @param backOnStopLine true일 경우 정지선 뒤에서 후진 후 회전, 
 *                       false일 경우 일반 후진 후 회전
 * @details
 * 1) backOnStopLine 모드: 정지선을 등지고 천천히 후진하여 완전히 벗어남  
 *    일반 모드: 잠시 후진 후 정지  
 * 2) inner-pivot 방식으로 좌측 피벗 90° 회전 두 번 수행  
 * 3) 회전 완료 후 전진 복귀  
 */
void turnAround180Degrees(bool backOnStopLine) {
    int softPower = defaultPower * SOFT_TURN_FACTOR / 100;

    // 1) 후진 모드 선택
    if (backOnStopLine) {
        // 정지선 등지고 후진 → 라인 벗어날 때까지
        driveMotors(DIRECTION_BACKWARD, softPower, DIRECTION_BACKWARD, softPower);
        while (analogRead(IR_SENSOR_LEFT_PIN)  > MIN_BLACK_THRESHOLD ||
               analogRead(IR_SENSOR_RIGHT_PIN) > MIN_BLACK_THRESHOLD) {
            delay(1);
        }
        delay(130);
    } else {
        // 일반 후진
        driveMotors(DIRECTION_BACKWARD, softPower, DIRECTION_BACKWARD, softPower);
        delay(150);
        stopMotors();
        delay(50);
    }

    // 2) 첫 번째 90° 좌측 회전
    intersectionTurnLeft(softPower);
    delay(50);

    // 3) 두 번째 90° 좌측 회전
    intersectionTurnLeft(softPower);
    delay(50);

    // 4) 직진 복귀
    moveForward(defaultPower);
}

/**
 * @brief 교차로에서 왼쪽으로 90° 회전하는 함수 (inner-pivot 방식)
 * @details
 * 1) 초기 정지 및 위치 보정  
 * 2) inner-pivot으로 90° 좌회전 (분기된 라인 감지 시 정지)  
 * 3) 회전 완료 후 전진 복귀  
 */
void turnLeft90Degrees() {
    // 1) 초기 정지 및 짧은 후진으로 위치 보정
    stopMotors();
    delay(50);
    driveMotors(DIRECTION_BACKWARD, 80, DIRECTION_BACKWARD, 80);
    delay(20);
    stopMotors();
    delay(50);

    // 2) inner-pivot 좌회전: 왼쪽 바퀴 고정, 오른쪽 바퀴만 구동
    intersectionTurnLeft(100);
    delay(30);

    // 3) 전진 복귀
    moveForward(defaultPower);
    delay(90);
}

/**
* @brief 오른쪽으로 90° 회전하는 함수
* @details
* 1) 초기 정지 및 위치 보정  
* 2) 첫 번째 화이트 라인 감지  
* 3) 두 번째 블랙 라인 감지  
* 4) 직진으로 빠져나온 후 최종 보정 회전 및 직진  
*/
void turnRight90Degrees() {
    // 1) 초기 정지 및 짧은 후진으로 위치 보정
    stopMotors();
    delay(50);
    driveMotors(DIRECTION_BACKWARD, 80, DIRECTION_BACKWARD, 80);
    delay(20);
    stopMotors();
    delay(50);

    // 2) inner-pivot 좌회전: 왼쪽 바퀴 고정, 오른쪽 바퀴만 구동
    intersectionTurnRight(100);
    delay(30);

    // 3) 전진 복귀
    moveForward(defaultPower);
    delay(90);
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
      Serial.print(F("알 수 없는 UID: "));
      Serial.println(uidStr);
      return false;
  }
}

/**
 * @enum RunState
 * @brief AGV 로봇의 동작 단계를 나타내는 상태 머신 열거형
 */
enum RunState {
  STATE_IDLE = 0,    ///< RFID 태그 대기
  STATE_MOVE_TO_A,   ///< A 지점(픽업)으로 이동
  STATE_PICKUP,      ///< 픽업 처리(리프터 상승 및 B 좌표 읽기)
  STATE_MOVE_TO_B,   ///< B 지점(드롭)으로 이동
  STATE_DROPOFF,     ///< 드롭 처리(리프터 하강)
  STATE_RETURN       ///< A 지점으로 복귀
};

/** @brief 현재 상태를 저장하는 전역 변수 */
volatile RunState runState = STATE_IDLE;

/** @brief A/B 지점의 좌표를 저장할 전역 변수 */
uint8_t coordAX, coordAY;  ///< A 지점 좌표
uint8_t coordBX, coordBY;  ///< B 지점 좌표

/**
 * @brief (currentX, currentY) → (x, y)까지 이동
 * @param x 목표 열
 * @param y 목표 행
 */
void navigateTo(uint8_t x, uint8_t y) {
  Direction path[64];
  uint8_t len = findPath(currentX, currentY, x, y, path);

  if (len == 0) {
      return;
  }
  for (uint8_t i = 0; i < len; i++) {
    moveOneCell(path[i]);
  }
}

/** @brief 네 방향 벡터 */
const int dx[4] = { 0, 1, 0, -1 };  // NORTH, EAST, SOUTH, WEST
const int dy[4] = {-1, 0, 1,  0 };

/**
* @brief (sx,sy) → (tx,ty) 최단 경로를 dirPath[]에 담아 반환
* @param sx, sy  시작 좌표
* @param tx, ty  목표 좌표
* @param dirPath 경로 방향을 저장할 배열 (최대 64)
* @return 경로 길이(스텝 수), 실패 시 0
*/
uint8_t findPath(uint8_t sx, uint8_t sy, uint8_t tx, uint8_t ty, Direction dirPath[]) {
  bool visited[8][8] = {false};
  uint8_t fromDir[8][8];
  uint8_t qx[64], qy[64];
  int head=0, tail=0;
  bool found = false;

  // 시작점
  visited[sy][sx] = true;
  qx[tail]=sx; qy[tail]=sy; tail++;

  // BFS
  while (head < tail) {
      uint8_t x = qx[head], y = qy[head++];
      if (x==tx && y==ty){
        found = true;
        break; 
      }
      for (uint8_t d=0; d<4; d++) {
          int nx = x + dx[d], ny = y + dy[d];
          if (nx<0||nx>=8||ny<0||ny>=8) continue;
          if (!gridMap[ny][nx] || visited[ny][nx]) continue;
          visited[ny][nx] = true;
          fromDir[ny][nx] = d;
          qx[tail]=nx; qy[tail++]=ny;
      }
  }
  // 목표 미도달
  if (!visited[ty][tx]) return 0;

  // 경로 역추적
  uint8_t pathLen=0;
  uint8_t cx=tx, cy=ty;
  while (!(cx==sx && cy==sy)) {
      uint8_t d = fromDir[cy][cx];
      dirPath[pathLen++] = (Direction)d;
      // 반대 방향으로 한 칸 뒤로
      cx -= dx[d]; cy -= dy[d];
  }
  
  // 뒤집기
  for (uint8_t i=0; i<pathLen/2; i++) {
      Direction tmp = dirPath[i];
      dirPath[i] = dirPath[pathLen-1-i];
      dirPath[pathLen-1-i] = tmp;
  }
  return pathLen;
}

/**
 * @brief AGV의 현재 좌표(currentX, currentY)를 방향(dir)에 따라 갱신
 * @param dir   이동 방향 (NORTH/EAST/SOUTH/WEST)
 * @details NORTH일 때 Y--, EAST일 때 X++, SOUTH일 때 Y++, WEST일 때 X--  
 *          좌표 범위는 0~7로 가정하며, 범위 체크가 필요하면 추가하세요.
 */
void updatePosition(Direction dir) {
  Serial.println("updatePosition");

  switch (dir) {
      case NORTH:
          if (currentY > 0) currentY--;
          break;
      case EAST:
          if (currentX < 7) currentX++;
          break;
      case SOUTH:
          if (currentY < 7) currentY++;
          break;
      case WEST:
          if (currentX > 0) currentX--;
          break;
  }
}

/**
 * @brief 교차로 도착 후 셀 중앙으로 위치 보정
 * @details
 * 1) 정지선(흑) 위에 멈춘 상태에서 후진하여 흰색 영역(라인) 밖으로 빠져나옴  
 * 2) 천천히 전진하며 첫 번째 흑(정지선) 감지 시 정지 → 이 지점이 셀 중앙
 * 3) 최종 정지
 * @param speed 모터 속도 (0~255)
 */
void correctPositionAfterIntersection(int speed) {
  // 1) 천천히 후진: 흑→백
  driveMotors(DIRECTION_BACKWARD, speed/2, DIRECTION_BACKWARD, speed/2);
  while (true) {
    // int lv = analogRead(IR_SENSOR_LEFT_PIN);
    // int rv = analogRead(IR_SENSOR_RIGHT_PIN);
    int leftValue, rightValue;
    readLineSensors(leftValue, rightValue);

    // 둘 중 하나라도 흑(> MIN_BLACK_THRESHOLD)이면 아직 정지선 위
    if (leftValue < MIN_BLACK_THRESHOLD && rightValue < MIN_BLACK_THRESHOLD) {
      // 둘 다 흰색 영역(라인 바로 바깥) 감지 → 탈출 완료
      break;
    }
    delay(1);
  }
  stopMotors();
  delay(20);

  alignToLine(speed / 2);

  // 2) 천천히 전진: 백→흑
  driveMotors(DIRECTION_FORWARD, speed/2, DIRECTION_FORWARD, speed/2);
  while (true) {
    // int lv = analogRead(IR_SENSOR_LEFT_PIN);
    // int rv = analogRead(IR_SENSOR_RIGHT_PIN);

    int leftValue, rightValue;
    readLineSensors(leftValue, rightValue);

    // 둘 다 흑(> MIN_BLACK_THRESHOLD) 감지 시 중앙 도달
    if (leftValue > MIN_BLACK_THRESHOLD && rightValue > MIN_BLACK_THRESHOLD) {
      break;
    }
    delay(1);
  }
  stopMotors();
  delay(20);
}

/**
 * @brief 한 칸(Cell) 만큼 이동 (회전 후 재정렬 추가)
 * @details
 * 1) 바라보는 방향으로 회전  
 * 2) 회전 직후 짧게 전진해서 센서를 라인 위로 올림  
 * 3) 라인 중앙 정렬 (alignToLine)  
 * 4) 현재 교차로 벗어나기  
 * 5) 다음 교차로까지 라인트레이싱  
 * 6) 교차로 도착 후 위치 보정  
 * 7) 좌표 갱신  
 * @param dir   이동 방향 (NORTH/EAST/SOUTH/WEST)
 */
void moveOneCell(Direction dir) {
    // 1) 바라보는 방향 맞추기
    switch(dir) {
        case NORTH: /* 북쪽으로 회전 */ turnToNorth(); break;
        case EAST:  /* 동쪽으로 회전 */ turnToEast();  break;
        case SOUTH: /* 남쪽으로 회전 */ turnToSouth(); break;
        case WEST:  /* 서쪽으로 회전 */ turnToWest();  break;
    }

    // 2) 회전 직후 짧게 전진해서 센서를 라인 위로 올리기
    moveForward(defaultPower);
    delay(50);
    stopMotors();

    // 3) 라인 중앙 정렬: 센서가 라인 위에 완전히 올라오도록 보정
    alignToLine(defaultPower / 2);

    // 4) 현재 교차로(정지선)에서 벗어나기
    while (isIntersection()) {
        proportionalLineTrace(defaultPower);
    }

    // 5) 다음 교차로까지 라인트레이싱
    while (!isIntersection()) {
        proportionalLineTrace(defaultPower);
    }
    stopMotors();

    // 6) 교차로 도착 → 위치 보정
    correctPositionAfterIntersection(defaultPower);

    // 7) currentX/Y 갱신
    updatePosition(dir);
}

/**
 * @brief 제자리에서 선 중앙 정렬: 한쪽에만 라인이 있을 때 제자리 회전(pivot)으로 보정
 * @param power 회전할 때 사용할 모터 속도 (0~255)
 */
void alignToLine(int power) {
  int leftValue, rightValue;

  // 초기 센서값 읽기
  readLineSensors(leftValue, rightValue);

  while (true) {
    // 센서값 갱신
    readLineSensors(leftValue, rightValue);

    // 1) 양쪽 다 흰색이면 라인 중앙에 위치했다고 판단 → 종료
    if (leftValue < MAX_WHITE_THRESHOLD && rightValue < MAX_WHITE_THRESHOLD) {
      break;
    }

    // 2) 한쪽만 검정이면 반대 방향으로 피벗 회전
    if (leftValue > MIN_BLACK_THRESHOLD) {
      turnRight(power);
      continue;
    }
    if (rightValue > MIN_BLACK_THRESHOLD) {
      turnLeft(power);
      continue;
    }

    // 3) 회색(Gray) 영역: 부드러운 보정 (soft turn)
    if (leftValue > MAX_WHITE_THRESHOLD) {
      driveMotors(DIRECTION_FORWARD, power,
                  DIRECTION_FORWARD, power * SOFT_TURN_FACTOR / 100);
    } 
    else if (rightValue > MAX_WHITE_THRESHOLD) {
      driveMotors(DIRECTION_FORWARD, power * SOFT_TURN_FACTOR / 100,
                  DIRECTION_FORWARD, power);
    }
    // (양쪽 다 검정인 경우는 특별히 처리 없이 루프 재진행)
  }

  // 보정 완료 후 모터 정지
  stopMotors();
}

/**
* @brief 메인 루프: 상태 머신으로 AGV 동작 제어
*/
void loop() {
  switch (runState) {
      case STATE_IDLE:
          // RFID 카드 태깅 대기
          if (readCoordinatesFromRFID(coordAX, coordAY)) {
              // 태깅 확인음
              tone(BUZZER_PIN, 262); delay(100);
              tone(BUZZER_PIN, 330); delay(250);
              noTone(BUZZER_PIN);

              alignToLine(defaultPower);

              runState = STATE_MOVE_TO_A;
          }
          break;
      case STATE_MOVE_TO_A:
          // A 지점으로 이동
          navigateTo(coordAX, coordAY);
          stopMotors();
          runState = STATE_PICKUP;
          break;
      case STATE_PICKUP:
          // B 지점 좌표 재읽기
          if (readCoordinatesFromRFID(coordBX, coordBY)) {
              raiseLifter();
              playPickupTone();
              runState = STATE_MOVE_TO_B;
          } else {
              // UID 미등록 시 대기 상태 복귀
              runState = STATE_IDLE;
          }
          break;
      case STATE_MOVE_TO_B:
          // B 지점으로 이동
          navigateTo(coordBX, coordBY);
          stopMotors();
          runState = STATE_DROPOFF;
          break;
      case STATE_DROPOFF:
          // 물건 하강 및 알림음
          lowerLifter();
          playDropTone();
          runState = STATE_RETURN;
          break;
      case STATE_RETURN:
          // A 지점으로 복귀
          navigateTo(coordAX, coordAY);
          stopMotors();
          runState = STATE_IDLE;
          break;
      default:
          // 예기치 않은 상태는 초기화
          runState = STATE_IDLE;
          break;
  }
  delay(100);
}