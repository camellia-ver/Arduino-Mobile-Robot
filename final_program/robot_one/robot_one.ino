#include <Servo.h>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <MFRC522.h>
#include <SPI.h>

// ===== RFID 핀 설정 =====
// RFID 모듈의 연결된 핀 설정
#define RFID_SS_PIN 2      // RFID 모듈의 SS 핀 (Slave Select)
#define RFID_RST_PIN 4     // RFID 모듈의 Reset 핀
MFRC522 rfidReader(RFID_SS_PIN, RFID_RST_PIN); // RFID 리더 객체 초기화

// ===== 모터 핀 설정 =====
// 로봇의 모터 제어를 위한 핀 설정 (모터의 방향 및 PWM)
#define LEFT_MOTOR_DIR_PIN 7
#define LEFT_MOTOR_PWM_PIN 5
#define RIGHT_MOTOR_DIR_PIN 8
#define RIGHT_MOTOR_PWM_PIN 6

// ===== 이동 방향 상수 =====
// 로봇이 이동하는 방향을 정의하는 상수 (FORWARD: 전진, BACKWARD: 후진)
#define FORWARD 0
#define BACKWARD 1

// ===== IR 센서 핀 =====
// 라인트레이싱을 위한 IR 센서 핀 설정
const int leftIRSensorPin = A0;  // 왼쪽 IR 센서 핀
const int rightIRSensorPin = A1; // 오른쪽 IR 센서 핀

// ===== 로봇 초기 설정 =====
// 로봇의 시작 위치 설정 (x, y 좌표)
const int ROBOT_START_X = 0, ROBOT_START_Y = 2;

// ===== A* 알고리즘용 자료구조 =====
// A* 알고리즘을 구현하기 위한 우선순위 큐 (f 값이 작은 순서대로 노드를 처리)
std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
// A* 알고리즘에서 방문한 노드를 추적하기 위한 집합 (이미 방문한 좌표는 다시 처리하지 않도록 함)
std::unordered_set<std::pair<int, int>, hash_pair> closedListSet;

// ===== 맵 설정 =====
// 로봇이 탐색할 맵을 2D 배열로 정의 (0은 비어있는 공간, 1은 장애물)
const int MAP_ROWS = 8;
const int MAP_COLS = 8;
uint8_t mapGrid[MAP_ROWS][MAP_COLS] = {
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
};  // 맵 초기화 (모두 0으로 설정)

// ===== A* 알고리즘 =====

void navigateToGoal(Node start, Node goal) {
  openList.push(start);  // 시작 노드를 우선순위 큐에 넣기

  while (!openList.empty()) {
    Node current = openList.top();
    openList.pop();
    closedListSet.insert({current.x, current.y});  // 현재 노드를 방문 처리

    if (current.x == goal.x && current.y == goal.y) {
      // 목표에 도달한 경우
      Serial.println("목표 지점에 도달했습니다.");
      return;
    }

    // 이웃 노드를 찾고 우선순위 큐에 넣기
    std::vector<Node> neighbors = findNeighborNodes(current, goal);
    for (const Node& neighbor : neighbors) {
      if (closedListSet.find({neighbor.x, neighbor.y}) == closedListSet.end()) {
        openList.push(neighbor);
      }
    }
  }
}

void returnToStart(Node start) {
  Node goal(start.x, start.y, 0, 0, 0, 0, 0);  // 출발 지점으로 설정
  navigateToGoal(start, goal);  // 출발 지점으로 이동
}

// 로봇을 전진시키는 함수
void moveRobotForward(int power) {
  controlMotors(FORWARD, power, FORWARD, power); // 두 모터를 전진으로 설정
}

// 모터를 정지시키는 함수
void stopMotors() {
  analogWrite(LEFT_MOTOR_PWM_PIN, 0);  // 왼쪽 모터 정지
  analogWrite(RIGHT_MOTOR_PWM_PIN, 0); // 오른쪽 모터 정지
}

// RFID 카드를 읽고 UID를 반환하는 함수
String readRFIDCard() {
  // RFID 카드가 인식되면 UID를 읽어 반환
  if (rfidReader.PICC_IsNewCardPresent() && rfidReader.PICC_ReadCardSerial()) {
    String uidString = "";
    for (byte i = 0; i < rfidReader.uid.size; i++) {
      if (rfidReader.uid.uidByte[i] < 0x10) uidString += "0";
      uidString += String(rfidReader.uid.uidByte[i], HEX);  // UID를 16진수 문자열로 변환
    }
    uidString.toUpperCase(); // 카드 UID 대소문자 일관성
    rfidReader.PICC_HaltA();  // 카드 인식 중지
    rfidReader.PCD_StopCrypto1();  // 암호화 종료
    delay(1000);  // 잠시 대기
    return uidString;  // 카드 UID 반환
  }
  return "";  // 카드가 인식되지 않으면 빈 문자열 반환
}

// RFID UID에 해당하는 목표 지점으로 로봇을 이동시키는 함수
void navigateToRFIDGoal(String rfidUID) {
  if (rfidToDestination.find(rfidUID) == rfidToDestination.end()) {
    Serial.println("등록되지 않은 RFID입니다.");  // 등록되지 않은 RFID 인식 시 경고 출력
    return;
  }

  // 목표 지점 좌표를 가져와서 A* 알고리즘을 이용해 이동
  Node start(ROBOT_START_X, ROBOT_START_Y, 0, 0, 0, 0, 0);
  Node goal(rfidToDestination[rfidUID].first, rfidToDestination[rfidUID].second, 0, 0, 0, 0, 0);
    
  // A* 알고리즘을 이용한 경로 탐색
  navigateToGoal(start, goal);
  Node goal(goalCoords.first, goalCoords.second, 0, 0, 0, 0, 0);  // 목표 위치 설정
  
  navigateToGoal(start, goal);  // 목표로 이동
}

void setup() {
  // 시리얼 모니터 초기화
  Serial.begin(9600);

  // 모터 핀 설정
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);

  // RFID 리더기 초기화
  SPI.begin();
  rfidReader.PCD_Init();

  Serial.println("로봇1 시작: RFID 카드 인식 대기 중...");
}

void loop() {
   // RFID 카드를 인식하면 해당 목표 지점으로 이동
  String rfidUID = readRFIDCard();
  if (rfidUID != "") {
    Serial.print("로봇1 RFID 카드 UID: ");
    Serial.println(rfidUID);
    navigateToRFIDGoal(rfidUID);  // 해당 RFID 목표 지점으로 이동

    // 목표 지점에 도달하면 반대 방향으로 돌아가는 동작
    Node currentGoal(rfidToDestination[rfidUID].first, rfidToDestination[rfidUID].second, 0, 0, 0, 0, 0);
    returnToStart(currentGoal);  // 목표 지점에서 다시 시작 지점으로 이동
  }
}
