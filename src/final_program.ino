#include <Wire.h>
#include <Adafruit_PN532.h>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <algorithm>

// RFID 리더 설정
#define SDA_PIN 2 // SDA 핀
#define SCL_PIN 3 // SCL 핀
Adafruit_PN532 nfc(SDA_PIN, SCL_PIN); // RFID 리더 객체

// 모터 핀 설정
#define pinDIR1 7 // 1번(왼쪽)모터 방향 지정용 연결 핀
#define pinPWM1 5 // 1번(왼쪽)모터 속력 지정용 연결 핀

#define pinDIR2 8 // 2번(오른쪽)모터 방향 지정용 연결 핀
#define pinPWM2 6 // 2번(오른쪽)모터 속력 지정용 연결 핀

// 속도 및 시간 설정
#define ROBOT_SPEED 150 // 로봇 속도
#define TURN_DELAY_90 400 // 90도 회전 시간
#define TURN_DELAY_180 800 // 180도 회전 시간
#define FOLLOW_LINE_DURATION 500 // 라인트레이싱 진행 시간

// 라인트레이싱 센서 설정
#define LINE_SENSOR_LEFT A0 // 왼쪽 센서 핀
#define LINE_SENSOR_RIGHT A1 // 오른쪽 센서 핀
#define LINE_THRESHOLD 500 // 라인 감지 기준값

// 시작 위치 설정
#define ROBOT_START_X 0
#define ROBOT_START_Y 0

// RFID 매핑 (A지점, B지점과 RFID 카드 연결)
std::unordered_map<String, std::pair<int, int>> rfidToDestination = {
  {"04AABBCCDD", {2, 3}},  // A지점
  {"0411223344", {5, 1}}   // B지점
};

// 현재 로봇의 방향 (0: 위, 1: 오른쪽, 2: 아래, 3: 왼쪽)
int robotDirection = 0;

// ===== Node 클래스 (A*) =====
// A* 알고리즘에 사용할 노드 클래스
struct Node {
  int x, y;  // 노드 위치
  int g, h, f; // g: 시작점에서 해당 노드까지의 비용, h: 휴리스틱 비용, f: g + h
  int px, py; // 부모 노드의 위치 (경로 추적용)

  Node(int x, int y, int g, int h, int f, int px, int py)
    : x(x), y(y), g(g), h(h), f(f), px(px), py(py) {}

  // f 값이 작은 순으로 정렬하기 위한 연산자
  bool operator>(const Node& other) const {
    return f > other.f;
  }
};

// A* 알고리즘에서 해시 맵을 사용할 때 필요한 구조체
struct hash_pair {
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2>& p) const {
    auto h1 = std::hash<T1>{}(p.first);
    auto h2 = std::hash<T2>{}(p.second);
    return h1 ^ h2;
  }
};

// A* 전역 변수
std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList; // 우선순위 큐 (A*에서 사용)
std::unordered_set<std::pair<int, int>, hash_pair> closedListSet; // 이미 처리된 노드
std::vector<std::pair<int, int>> path; // 최종 경로

// ===== A* 알고리즘 =====
// 두 점 사이의 휴리스틱 비용을 계산 (맨해튼 거리)
int heuristic(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

// 현재 노드의 인접 노드들을 구하는 함수
std::vector<Node> findNeighborNodes(Node& current, Node& goal) {
  std::vector<Node> neighbors;
  int dx[4] = {0, 1, 0, -1}; // 상, 우, 하, 좌 방향
  int dy[4] = {-1, 0, 1, 0};

  // 4방향으로 인접 노드 탐색
  for (int dir = 0; dir < 4; dir++) {
    int nx = current.x + dx[dir];
    int ny = current.y + dy[dir];
    if (nx >= 0 && ny >= 0) { // 유효한 범위 안에서만
      int g = current.g + 1; // 비용 증가
      int h = heuristic(nx, ny, goal.x, goal.y); // 휴리스틱 값 계산
      int f = g + h; // f = g + h
      neighbors.push_back(Node(nx, ny, g, h, f, current.x, current.y)); // 인접 노드 추가
    }
  }
  return neighbors;
}

// A* 알고리즘을 실행하여 경로를 찾는 함수
bool findPathAStar(Node start, Node goal) {
  while (!openList.empty()) openList.pop(); // 우선순위 큐 초기화
  closedListSet.clear(); // 이미 처리한 노드 초기화
  path.clear(); // 경로 초기화

  openList.push(start); // 시작 노드를 큐에 넣기
  std::unordered_map<std::pair<int, int>, std::pair<int, int>, hash_pair> parentMap; // 부모 노드 저장

  // A* 알고리즘 수행
  while (!openList.empty()) {
    Node current = openList.top(); // 큐에서 가장 적은 f 값을 가진 노드 선택
    openList.pop();

    if (current.x == goal.x && current.y == goal.y) { // 목표에 도달했을 경우
      std::pair<int, int> pos = {current.x, current.y};
      while (!(pos.first == start.x && pos.second == start.y)) {
        path.push_back(pos); // 경로 저장
        pos = parentMap[pos]; // 부모 노드로 이동
      }
      path.push_back({start.x, start.y}); // 시작점 추가
      std::reverse(path.begin(), path.end()); // 경로 역순으로 정렬
      return true;
    }

    closedListSet.insert({current.x, current.y}); // 처리한 노드에 추가

    // 인접 노드 탐색
    std::vector<Node> neighbors = findNeighborNodes(current, goal);
    for (const Node& neighbor : neighbors) {
      std::pair<int, int> npos = {neighbor.x, neighbor.y};
      if (closedListSet.find(npos) == closedListSet.end()) { // 이미 처리한 노드가 아니면
        openList.push(neighbor); // 큐에 추가
        parentMap[npos] = {current.x, current.y}; // 부모 노드 기록
      }
    }
  }
  return false; // 목표를 찾지 못했을 경우
}

// ===== 모터 제어 =====
// 모터 설정
void setupMotors() {
  pinMode(pinDIR1, OUTPUT);
  pinMode(pinPWM1, OUTPUT);
  pinMode(pinDIR2, OUTPUT);
  pinMode(pinPWM2, OUTPUT);
}

// 로봇을 앞으로 이동시키는 함수
void moveForward(int speed) {
  analogWrite(pinPWM1, speed);
  analogWrite(pinPWM2, speed);
  digitalWrite(pinDIR1, HIGH);
  digitalWrite(pinDIR2, HIGH);
}

// 모터 멈추는 함수
void stopMotors() {
  digitalWrite(pinDIR1, LOW);
  digitalWrite(pinDIR2, LOW);
}

// 로봇을 90도 우측으로 회전시키는 함수
void turnRobotRightInPlace(int speed) {
  analogWrite(pinPWM1, speed);
  analogWrite(pinPWM2, speed);
  digitalWrite(pinDIR1, HIGH);
  digitalWrite(pinDIR2, LOW);
}

// 로봇을 90도 좌측으로 회전시키는 함수
void turnRobotLeftInPlace(int speed) {
  analogWrite(pinPWM1, speed);
  analogWrite(pinPWM2, speed);
  digitalWrite(pinDIR1, LOW);
  digitalWrite(pinDIR2, HIGH);
}

// ===== 라인트레이싱 =====
// 라인을 따라 이동하는 함수
void followLine(int speed, int duration) {
  unsigned long startTime = millis(); // 시작 시간 기록
  while (millis() - startTime < duration) { // 주어진 시간 동안 계속 수행
    int leftSensor = analogRead(LINE_SENSOR_LEFT); // 왼쪽 센서 값 읽기
    int rightSensor = analogRead(LINE_SENSOR_RIGHT); // 오른쪽 센서 값 읽기

    // 라인을 따라가는 경우
    if (leftSensor < LINE_THRESHOLD && rightSensor < LINE_THRESHOLD) {
      moveForward(speed); // 두 센서 모두 라인을 감지하면 직진
    } else if (leftSensor > LINE_THRESHOLD) { // 왼쪽 센서만 라인을 감지하면 우회전
      turnRobotRightInPlace(speed);
    } else if (rightSensor > LINE_THRESHOLD) { // 오른쪽 센서만 라인을 감지하면 좌회전
      turnRobotLeftInPlace(speed);
    }
  }
  stopMotors(); // 이동 종료 후 모터 멈춤
}

// ===== 방향 전환 =====
// 로봇의 방향을 목표 방향으로 맞추는 함수
void rotateToDirection(int targetDirection) {
  int diff = (targetDirection - robotDirection + 4) % 4; // 현재 방향과 목표 방향의 차이 계산
  if (diff == 1) { // 90도 우회전
    turnRobotRightInPlace(ROBOT_SPEED);
    delay(TURN_DELAY_90);
  } else if (diff == 3) { // 90도 좌회전
    turnRobotLeftInPlace(ROBOT_SPEED);
    delay(TURN_DELAY_90);
  } else if (diff == 2) { // 180도 회전
    turnRobotRightInPlace(ROBOT_SPEED);
    delay(TURN_DELAY_180);
  }
  stopMotors(); // 회전 후 모터 멈춤
  robotDirection = targetDirection; // 로봇 방향 업데이트
}

// ===== 경로 따라 이동 =====
// A* 알고리즘으로 구한 경로를 라인트레이싱을 이용해 따라가는 함수
void followPathWithLineTracing() {
  for (int i = 1; i < path.size(); i++) { // 경로의 각 지점에 대해 이동
    int currentX = path[i-1].first;
    int currentY = path[i-1].second;
    int nextX = path[i].first;
    int nextY = path[i].second;

    int moveX = nextX - currentX; // x축 방향으로의 이동
    int moveY = nextY - currentY; // y축 방향으로의 이동

    int targetDirection = robotDirection; // 목표 방향 설정
    if (moveX == 1 && moveY == 0) targetDirection = 1; // 오른쪽으로 이동
    else if (moveX == -1 && moveY == 0) targetDirection = 3; // 왼쪽으로 이동
    else if (moveX == 0 && moveY == 1) targetDirection = 2; // 아래쪽으로 이동
    else if (moveX == 0 && moveY == -1) targetDirection = 0; // 위쪽으로 이동

    if (robotDirection != targetDirection) { // 방향이 다르면 회전
      rotateToDirection(targetDirection);
    }

    followLine(ROBOT_SPEED, FOLLOW_LINE_DURATION); // 라인트레이싱으로 경로 따라가기
  }
}

// ===== RFID 읽기 =====
// RFID 태그를 읽는 함수
String readRFID() {
  uint8_t success;
  uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
  uint8_t uidLength;

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength); // RFID 태그 읽기

  if (success) {
    String uidString = "";
    for (int i = 0; i < uidLength; i++) {
      uidString += String(uid[i], HEX); // 읽은 UID를 문자열로 변환
    }
    return uidString;
  }
  return ""; // RFID 읽기 실패
}

void navigateToRFIDGoal(String rfidUID) {
  // 1. RFID UID에 해당하는 목표 좌표 가져오기
  std::pair<int, int> goal = rfidToDestination[rfidUID];
  int goalX = goal.first;
  int goalY = goal.second;

  // 2. 현재 위치 (로봇 시작 위치) 정의
  Node start(ROBOT_START_X, ROBOT_START_Y, 0, heuristic(ROBOT_START_X, ROBOT_START_Y, goalX, goalY), 0, -1, -1);

  // 3. A* 알고리즘을 이용해 경로 찾기
  Node goalNode(goalX, goalY, 0, 0, 0, -1, -1);
  bool pathFound = findPathAStar(start, goalNode);

  if (pathFound) {
    // 4. 경로를 따라가며 이동하기
    followPathWithLineTracing();
    Serial.println("목표 지점에 도달했습니다!");

    // 5. 현재 RFID UID에 해당하는 목적지에서 반대방향으로 이동할 다음 RFID UID를 찾음
    String nextLocation = "";
    
    // 현재 RFID UID에 해당하는 다른 RFID UID를 찾아 반대 지점으로 설정
    for (const auto& entry : rfidToDestination) {
      if (entry.first != rfidUID) {
        nextLocation = entry.first; // 반대 RFID 카드 찾기
        break;
      }
    }

    // 6. 반대 목적지로 이동
    if (nextLocation != "") {
      navigateToRFIDGoal(nextLocation); // 반대 목적지로 이동
    } else {
      Serial.println("반대 지점이 없습니다.");
    }
  } else {
    Serial.println("목표 지점으로 가는 경로를 찾을 수 없습니다.");
  }
}

// ===== Arduino 기본 =====
void setup() {
  Serial.begin(115200); // 시리얼 모니터 시작
  setupMotors(); // 모터 초기화

  nfc.begin(); // RFID 리더 초기화
  uint32_t versiondata = nfc.getFirmwareVersion(); // RFID 리더의 버전 정보 가져오기
  if (!versiondata) {
    Serial.println("RFID 리더 오류!"); // 오류 발생 시 종료
    while (1);
  }
  nfc.SAMConfig(); // RFID 리더 설정
  Serial.println("시작 준비 완료!");
}

// ===== 메인 루프 =====
void loop() {
  String rfidUID = readRFID(); // RFID 태그 읽기
  if (rfidUID != "") { // RFID 태그가 읽혔다면
    Serial.print("RFID UID 읽음: ");
    Serial.println(rfidUID);

    // RFID 카드에 해당하는 목적지로 이동
    if (rfidToDestination.find(rfidUID) != rfidToDestination.end()) {
      String currentLocation = rfidUID;

      // 해당 RFID 카드에 매핑된 목적지로 이동
      Serial.println("등록된 RFID입니다. 이동을 시작합니다.");
      navigateToRFIDGoal(currentLocation); // 유동적으로 목적지 설정
    } else {
      Serial.println("등록되지 않은 RFID입니다.");
    }
    delay(1000); // 1초 대기
  }
}
