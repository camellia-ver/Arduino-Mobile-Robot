#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

// 핀 설정
#define SS_PIN 2  // SDA
#define RST_PIN 4

#define pinBuzzer 3
#define pinServo 9

#define LEFT_MOTOR_DIR_PIN 7  // 1번(왼쪽) 모터 방향 제어 핀
#define LEFT_MOTOR_PWM_PIN 5  // 1번(왼쪽) 모터 속도 제어 핀

#define RIGHT_MOTOR_DIR_PIN 8 // 2번(오른쪽) 모터 방향 제어 핀
#define RIGHT_MOTOR_PWM_PIN 6 // 2번(오른쪽) 모터 속도 제어 핀

// ===== 라인 센서 핀 및 설정 =====
#define LINE_SENSOR_LEFT A6
#define LINE_SENSOR_RIGHT A7
#define LT_AJDUST       60             // 젤리비 센서 값 조정 (필요한 경우 이 값을 실험적으로 조정)
#define LT_MAX_WHITE   410 + LT_AJDUST // 흰색으로 판단하는 최대값
#define LT_MID_VALUE   560 + LT_AJDUST // 흑백 판단 경계값(중간값)
#define LT_MIN_BLACK   710 + LT_AJDUST // 검은색으로 판단하는 최소값

// 맵의 크기 정의 (8x8)
#define GRID_SIZE 8

// 경로 최대 길이 (최적화를 위해 제한)
#define MAX_PATH_LENGTH 32

const int ROBOT_MOVEMENT_SPEED = 80; // 로봇 속도
const int TURN_DELAY_90 = 610;  // 90도 회전 시간
const int TURN_DELAY_180 = 1220; // 180도 회전 시간

// RFID 객체 생성
MFRC522 rfid(SS_PIN, RST_PIN);

struct Point {
  uint8_t x;
  uint8_t y;
};

// RFID UID와 위치를 매핑하는 구조체
struct RfidData {
  const char key[9];   // RFID UID (문자열, 8자리 + 널 문자)
  Point point;         // 해당 RFID가 가리키는 좌표
};

// UID → 좌표 매핑 데이터
RfidData rfidDataMap[] = {
  {"14081b74", {6, 5}},    // 예: 첫 번째 RFID는 (6, 5)로 이동
  {"640fcf73", {7, 3}},    // 두 번째 RFID는 (7, 3)로 이동
};

// 장애물 맵: 0은 통로, 1은 장애물
uint8_t grid[GRID_SIZE][GRID_SIZE] = {
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 1, 1, 1, 0, 0, 0, 0},
  {0, 0, 0, 1, 0, 0, 0, 0},
  {0, 0, 0, 1, 0, 0, 0, 0},
  {0, 0, 0, 1, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
};

// 상, 하, 좌, 우 방향 벡터
int directions[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

// BFS 탐색을 위한 큐 배열
Point queue[GRID_SIZE * GRID_SIZE];
int queueSize = 0;  // 큐 크기 (front/rear 없이 사용)

// 경로를 저장하는 배열
Point path[MAX_PATH_LENGTH];
int pathLength = 0;
int currentStep = 0;  // 현재 몇 번째 경로를 처리 중인지

// BFS 알고리즘으로 최단 경로 탐색
int bfs(Point start, Point goal, uint8_t previousPositionX[GRID_SIZE][GRID_SIZE], uint8_t previousPositionY[GRID_SIZE][GRID_SIZE]) {
  bool visited[GRID_SIZE][GRID_SIZE] = {false};   // 방문 여부 배열
  int dist[GRID_SIZE][GRID_SIZE];                 // 거리 배열

  // 초기화: 거리 -1, 이전 좌표 255로 설정
  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      dist[i][j] = -1;
      previousPositionX[i][j] = 255;
      previousPositionY[i][j] = 255;
    }
  }

  // 시작 지점을 큐에 추가
  queueSize = 0;
  queue[queueSize++] = start;
  visited[start.x][start.y] = true;
  dist[start.x][start.y] = 0;

  // BFS 탐색 루프
  for (int i = 0; i < queueSize; i++) {
    Point current = queue[i];

    // 목표 도달 시 거리 반환
    if (current.x == goal.x && current.y == goal.y) {
      return dist[current.x][current.y];
    }

    // 4방향 탐색
    for (int d = 0; d < 4; d++) {
      int nx = current.x + directions[d][0];
      int ny = current.y + directions[d][1];

      // 맵 안에 있고, 장애물 없고, 방문하지 않았다면
      if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE &&
          grid[nx][ny] == 0 && !visited[nx][ny]) {
        visited[nx][ny] = true;
        dist[nx][ny] = dist[current.x][current.y] + 1;
        previousPositionX[nx][ny] = current.x;
        previousPositionY[nx][ny] = current.y;
        queue[queueSize++] = { (uint8_t)nx, (uint8_t)ny };  // 다음 탐색 지점 추가
      }
    }
  }

  return -1;  // 경로 없음
}

// BFS 결과를 바탕으로 경로를 역추적하여 path[]에 저장
void traceAndSavePath(Point goal, uint8_t previousPositionX[GRID_SIZE][GRID_SIZE], uint8_t previousPositionY[GRID_SIZE][GRID_SIZE]) {
  pathLength = 0;
  uint8_t x = goal.x;
  uint8_t y = goal.y;

  // 목표 지점부터 시작해 이전 위치를 따라가며 저장
  while (x != 255 && y != 255 && pathLength < MAX_PATH_LENGTH) {
    path[pathLength++] = {x, y};
    uint8_t px = previousPositionX[x][y];
    uint8_t py = previousPositionY[x][y];
    x = px;
    y = py;
  }

  // 경로를 역순으로 뒤집음 (시작 → 목표 순으로)
  for (int i = 0; i < pathLength / 2; i++) {
    Point tmp = path[i];
    path[i] = path[pathLength - 1 - i];
    path[pathLength - 1 - i] = tmp;
  }

  currentStep = 0;  // 이동 시작점으로 초기화
}

// UID 문자열에 해당하는 좌표 찾기
Point findValueByKey(const char* key) {
  for (int i = 0; i < sizeof(rfidDataMap) / sizeof(rfidDataMap[0]); i++) {
    if (strcmp(rfidDataMap[i].key, key) == 0) {
      return rfidDataMap[i].point;
    }
  }
  return {255, 255};  // 못 찾으면 유효하지 않은 값 반환
}

// RFID로부터 UID 읽어오기
void readRFID(char* uidBuffer) {
  // 새 카드가 인식되지 않았거나 UID 읽기 실패 시 종료
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    uidBuffer[0] = '\0';
    return;
  }

  // UID를 문자열로 변환 (hex)
  for (byte i = 0; i < rfid.uid.size; i++) {
    sprintf(&uidBuffer[i * 2], "%02x", rfid.uid.uidByte[i]);
  }

  uidBuffer[rfid.uid.size * 2] = '\0';  // 문자열 종료 문자
  rfid.PICC_HaltA();  // 카드 통신 종료
}

enum RunStateEnum {
  // 카드 관련 상태
  STATE_WAIT_FOR_CARD = 0,        // RFID 카드를 기다리는 상태

  // 로봇 준비 및 검사 상태
  STATE_CHECK_CENTER = 1,         // 중심 위치 검사
  STATE_CHECK_LEFT_PREPARE = 10,  // 왼쪽 경로 검사 준비
  STATE_CHECK_LEFT_WAIT = 11,     // 왼쪽 경로 검사 대기
  STATE_CHECK_RIGHT_TURN = 12,    // 오른쪽으로 회전하는 상태
  STATE_CHECK_RIGHT_GO = 13,      // 오른쪽 경로로 이동 시작
  STATE_CHECK_RIGHT_DONE = 14,    // 오른쪽 경로 검사 완료

  // 경로 선택 상태
  STATE_LEFT_SELECTED = 100,      // 왼쪽 경로 선택됨
  STATE_CENTER_SELECTED = 101,    // 중앙 경로 선택됨
  STATE_RIGHT_SELECTED = 102,     // 오른쪽 경로 선택됨

  // 목표 도달 상태
  STATE_GOAL_REACHED_VIA_LEFT = 110,   // 왼쪽 경로로 목표 도달
  STATE_GOAL_REACHED_VIA_RIGHT = 111,  // 오른쪽 경로로 목표 도달
  STATE_GOAL_REACHED = 200,            // 목표에 도달한 상태

  // 반환 경로 상태
  STATE_RETURN_PREPARE = 201,     // 반환 경로 준비
  STATE_RETURN_CENTER = 202,      // 반환 경로에서 중앙으로 이동
  STATE_RETURN_LEFT_FIRST_STAGE = 300,  // 반환 경로에서 왼쪽 첫 번째 단계
  STATE_RETURN_LEFT_SECOND_STAGE = 301, // 반환 경로에서 왼쪽 두 번째 단계
  STATE_RETURN_LEFT_THIRD_STAGE = 302,  // 반환 경로에서 왼쪽 세 번째 단계
  STATE_RETURN_RIGHT_FIRST_STAGE = 400, // 반환 경로에서 오른쪽 첫 번째 단계
  STATE_RETURN_RIGHT_SECOND_STAGE = 401, // 반환 경로에서 오른쪽 두 번째 단계
  STATE_RETURN_RIGHT_THIRD_STAGE = 402, // 반환 경로에서 오른쪽 세 번째 단계

  // 완료 상태
  STATE_HOME_REACHED = 500,        // 집에 도달한 상태
  STATE_DONE = 999                 // 작업 완료 상태
};

RunStateEnum RunState = STATE_WAIT_FOR_CARD;

// 서보 모터 객체
Servo myservo;

// 설정된 변수
long startTime = 0;
int currentPath = 0;
int Power = 255;

// 시작 위치 (고정): (0, 0)
Point startPoint = {0, 0};

// 초기화 함수
void setup() {
  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();
  myservo.attach(pinServo);
  
  pinMode(pinBuzzer, OUTPUT);

  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);

  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  
  Serial.println("Ready to read RFID card");
}

// loop 함수
void loop() {
  executeLineTracing();
  // switch (RunState) {
  //   case STATE_WAIT_FOR_CARD:
  //     waitForRFIDCard();
  //     break;
      
  //   case STATE_CHECK_CENTER:
  //     handleCheckCenter();
  //     break;
      
  //   case STATE_CHECK_LEFT_WAIT:
  //     handleCheckLeftWait();
  //     break;
      
  //   // 여기에 필요한 상태들을 추가로 처리할 수 있습니다.
      
  //   case STATE_DONE:
  //     handleDone();
  //     break;

  //   default:
  //     executeLineTracing();  // 기본 라인 추적 동작
  //     break;
  // }
}

// 카드 감지 대기
void waitForRFIDCard() {
  static bool isPathAvailable = false;  // 경로 유무 상태 저장

  Serial.println("waitForRFIDCard() called");

  if (!isPathAvailable) {
      char rfidUID[16];  // UID 저장 버퍼
      readRFID(rfidUID); // UID 읽기

      // 디버깅: RFID UID 출력
      Serial.print("읽은 RFID UID: ");
      Serial.println(rfidUID);

      if (rfidUID[0] != '\0') {  // UID가 유효한 경우
          Serial.println("유효한 UID를 읽음, 다음 상태로 이동");

          RunState = STATE_CHECK_CENTER;

          // 소리 출력 (간단하게 변경)
          tone(pinBuzzer, 262, 100); // 100ms 동안 262Hz 소리
          delay(100);
          tone(pinBuzzer, 330, 250); // 250ms 동안 330Hz 소리
          noTone(pinBuzzer);

          // RFID UID로 목표 좌표 찾기
          Point goalPoint = findValueByKey(rfidUID);
          // 디버깅: 목표 좌표 출력
          Serial.print("목표 좌표: ");
          Serial.print(goalPoint.x);
          Serial.print(", ");
          Serial.println(goalPoint.y);

          if (goalPoint.x != 255 && goalPoint.y != 255) {  // 유효 좌표인지 확인
              uint8_t previousPositionX[GRID_SIZE][GRID_SIZE];
              uint8_t previousPositionY[GRID_SIZE][GRID_SIZE];

              // BFS 수행
              Serial.println("BFS를 수행하여 최단 경로를 찾습니다...");
              int result = bfs(startPoint, goalPoint, previousPositionX, previousPositionY);

              // 디버깅: BFS 결과 출력
              if (result != -1) {
                  Serial.print("경로가 존재합니다. 경로 길이: ");
                  Serial.println(result);
                  traceAndSavePath(goalPoint, previousPositionX, previousPositionY);  // 경로 저장
                  isPathAvailable = true;  // 다음 loop에서 이동 시작

                  // 경로 출력 (추가된 부분)
                  Serial.println("경로 출력:");
                  for (int i = 0; i < pathLength; i++) {
                      Serial.print("Step ");
                      Serial.print(i);
                      Serial.print(": ");
                      Serial.print(path[i].x);
                      Serial.print(", ");
                      Serial.println(path[i].y);
                  }
              } else {
                  Serial.println("경로를 찾을 수 없습니다.");
              }
          } else {
              Serial.println("유효하지 않은 좌표입니다.");
          }
      } else {
          Serial.println("유효한 RFID UID가 아닙니다.");
      }
  } else {
      // 경로를 따라 한 칸씩 이동
      if (currentStep < pathLength) {
          Serial.print("이동: ");
          Serial.print(path[currentStep].x);
          Serial.print(", ");
          Serial.println(path[currentStep].y);
          currentStep++;
          delay(500);  // 이동 간 시간 지연 (이 부분은 실제 환경에 맞게 최적화 가능)
      } else {
          Serial.println("경로 끝, 초기화...");
          isPathAvailable = false;  // 경로 끝나면 상태 초기화
          currentStep = 0;  // 경로 처음부터 다시 시작
      }
  }

  delay(100);  // 짧은 지연 시간, 필요에 따라 조정
}

// 센터 체크 처리
void handleCheckCenter() {
  // 센터 체크 로직 처리
  Serial.println("Checking center");
  RunState = STATE_CHECK_LEFT_PREPARE;
}

// 왼쪽 준비 상태 처리
void handleCheckLeftPrepare() {
  // 왼쪽 준비 상태에서 필요한 작업을 수행
  RunState = STATE_CHECK_LEFT_WAIT;
}

// 왼쪽 대기 상태 처리
void handleCheckLeftWait() {
  if (millis() - startTime > 160) {
    stopMotors();
    delay(100);

    if (isObstacleDetected()) {
      currentPath = 3;  // 우측 경로
      RunState = STATE_CHECK_RIGHT_TURN;
      turnAround180(false);  // 바로 뒤돌기
    } else {
      currentPath = 1;  // 좌측 경유로 선택
      RunState = STATE_LEFT_SELECTED;
      moveRobotForward(Power);
    }
  }
}

// 목표 도달 후 처리
void handleDone() {
  // 목표에 도달한 후 실행할 코드
  Serial.println("Goal reached. Completing task.");
  // 필요한 동작을 처리한 후 종료 상태로 변경
  RunState = STATE_DONE;
}

// ===== 라인 센서 읽기 함수 =====
int readSensorAverage(int pin) {
  int total = 0;
  for (int i = 0; i < 5; i++) {
    total += analogRead(pin);
  }
  return total / 5;
}

// 기준값 설정
const int SENSOR_WHITE = 400;  // 흰색일 때 측정값
const int SENSOR_BLACK = 700;  // 검정일 때 측정값

int normalizeSensor(int rawValue) {
  // 센서값을 0~1000 범위로 정규화
  return constrain(map(rawValue, SENSOR_WHITE, SENSOR_BLACK, 0, 1000), 0, 1000);
}

// 라인 추적 처리 (기본 동작)
void executeLineTracing() {
  Serial.println("=== Line Tracing Start ===");

  int leftValue = readSensorAverage(LINE_SENSOR_LEFT);
  int rightValue = readSensorAverage(LINE_SENSOR_RIGHT);

  // 정규화 적용
  int leftValue = normalizeSensor(leftRaw);
  int rightValue = normalizeSensor(rightRaw);

  int error = leftValue - rightValue;   // 차이 계산
  int correction = error / 4;           // 보정값: 더 민감하게
  int baseSpeed = ROBOT_MOVEMENT_SPEED; // 기본 속도

  // 속도 계산
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  // 속도 제한
  leftSpeed = constrain(leftSpeed, 50, 255);   // 최소값 50 이상
  rightSpeed = constrain(rightSpeed, 50, 255);

  // 모터 제어
  controlMotors(HIGH, leftSpeed, HIGH, rightSpeed);

  // 디버깅 출력
  Serial.print("Left Sensor: "); Serial.print(leftValue);
  Serial.print(" | Right Sensor: "); Serial.print(rightValue);
  Serial.print(" | Error: "); Serial.print(error);
  Serial.print(" | Correction: "); Serial.print(correction);
  Serial.print(" | Left Speed: "); Serial.print(leftSpeed);
  Serial.print(" | Right Speed: "); Serial.println(rightSpeed);

  delay(5); // 센서 주기
}

// 장애물 체크 함수
bool isObstacleDetected() {
  // 장애물이 있는지 체크하는 함수
  return false;  // 임시로 장애물 없음으로 처리
}

// 모터 제어 함수
void controlMotors(int leftDirection, int leftPower, int rightDirection, int rightPower) {
  // 왼쪽 모터 설정
  digitalWrite(LEFT_MOTOR_DIR_PIN, leftDirection);
  analogWrite(LEFT_MOTOR_PWM_PIN, leftPower);

  // 오른쪽 모터 설정
  digitalWrite(RIGHT_MOTOR_DIR_PIN, !rightDirection);
  analogWrite(RIGHT_MOTOR_PWM_PIN, rightPower);
}

void moveRobotForward(int power) {
  controlMotors(HIGH, power, HIGH, power);  // 두 모터 전진
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_DIR_PIN, LOW);
  digitalWrite(LEFT_MOTOR_PWM_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_DIR_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_PWM_PIN, LOW);
}

void turnRobotRight(int power) {
  controlMotors(HIGH, power, LOW, power);   // 왼쪽 모터 전진, 오른쪽 모터 후진
}

// 180도 회전 함수
void turnAround180(bool clockwise) {
  turnRobotRight(ROBOT_MOVEMENT_SPEED);   // 우회전 시작
  delay(TURN_DELAY_180);                // 180도 회전
  stopMotors();                         // 정지
}
