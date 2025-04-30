#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 2  // SDA
#define RST_PIN 4

#define GRID_SIZE 8

MFRC522 rfid(SS_PIN, RST_PIN);  // RFID 초기화

// 좌표를 표현하는 구조체
struct Point {
  int x;
  int y;
};

// RFID 키와 해당 위치 정보를 매핑하는 구조체
struct RfidData {
  const char* key;
  Point point;
};

// RFID 키와 목표 좌표 정보 (목표 지점 포함)
RfidData rfidDataMap[] = { 
  {"14081b74", {6, 5}},  
  {"640fcf73", {7, 3}},  // 목표 지점 (7, 3)
};

// 8x8 크기의 그리드 (0은 빈 공간, 1은 장애물)
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

// 상, 하, 좌, 우 방향
int directions[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

// 큐 (BFS에서 사용)
Point queue[GRID_SIZE * GRID_SIZE];
int front = 0, rear = 0;

// 방문 여부 및 거리, 이전 위치를 저장하는 배열
bool visited[GRID_SIZE][GRID_SIZE];
int dist[GRID_SIZE][GRID_SIZE];
Point prev[GRID_SIZE][GRID_SIZE];

// 큐에 좌표 추가
void enqueue(int x, int y) {
  queue[rear].x = x;
  queue[rear].y = y;
  rear = (rear + 1) % (GRID_SIZE * GRID_SIZE);
}

// 큐에서 좌표 추출
Point dequeue() {
  Point p = queue[front];
  front = (front + 1) % (GRID_SIZE * GRID_SIZE);
  return p;
}

// BFS를 통해 시작점에서 목표점까지의 최단 거리 계산
int bfs(Point start, Point goal) {
  // 초기화
  memset(visited, false, sizeof(visited));
  memset(dist, -1, sizeof(dist));  // -1은 미방문
  memset(prev, -1, sizeof(prev));  // -1로 초기화
  front = rear = 0;

  // 시작점 큐에 추가
  enqueue(start.x, start.y);
  visited[start.x][start.y] = true;
  dist[start.x][start.y] = 0;

  while (front != rear) {
    Point current = dequeue();

    // 목표 지점에 도달하면 거리 반환
    if (current.x == goal.x && current.y == goal.y) {
      return dist[current.x][current.y];
    }

    // 상, 하, 좌, 우로 이동
    for (int i = 0; i < 4; i++) {
      int nx = current.x + directions[i][0];
      int ny = current.y + directions[i][1];

      // 유효 좌표인지, 장애물이 없는지, 방문하지 않은 곳인지 확인
      if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE &&
          grid[nx][ny] == 0 && !visited[nx][ny]) {
        enqueue(nx, ny);
        visited[nx][ny] = true;
        dist[nx][ny] = dist[current.x][current.y] + 1;
        prev[nx][ny] = current;  // 이전 위치 기록
      }
    }
  }

  return -1;  // 목표 지점에 도달할 수 없으면 -1 반환
}

// 경로를 출력하는 함수
void printPath(Point goal) {
  Point path[GRID_SIZE * GRID_SIZE];
  int pathLength = 0;

  // 목표부터 시작하여 경로를 역추적
  for (Point p = goal; p.x != -1 && p.y != -1; p = prev[p.x][p.y]) {
    path[pathLength++] = p;
  }

  // 경로 출력 (역순으로 출력)
  Serial.println("경로: ");
  for (int i = pathLength - 1; i >= 0; i--) {
    Serial.print("(");
    Serial.print(path[i].x);
    Serial.print(", ");
    Serial.print(path[i].y);
    Serial.print(") ");
  }
  Serial.println();
}

// 주어진 RFID 키에 해당하는 좌표를 찾는 함수
Point findValueByKey(const char* key) {
  for (int i = 0; i < sizeof(rfidDataMap) / sizeof(rfidDataMap[0]); i++) {
    if (strcmp(rfidDataMap[i].key, key) == 0) {
      return rfidDataMap[i].point;
    }
  }
  return Point{-1, -1};  // 키가 없으면 (-1, -1) 반환
}

// RFID UID를 읽는 함수
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

Point startPoint = {0, 0};  // 시작점 (0, 0)

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

    // RFID UID에 해당하는 목표 지점 찾기
    Point goalPoint = findValueByKey(rfidUID.c_str());

    Serial.print("목표 위치: (");
    Serial.print(goalPoint.x); 
    Serial.print(", ");
    Serial.print(goalPoint.y);
    Serial.println(")");

    // 시작점에서 목표점까지의 최단 경로 계산
    int result = bfs(startPoint, goalPoint);

    if (result != -1) {
      Serial.print("최단 경로의 길이: ");
      Serial.println(result);
      printPath(goalPoint);  // 경로 출력
    } else {
      Serial.println("목표 지점에 도달할 수 없습니다.");
    }
  }

  delay(100);  // 0.1초 대기
}
