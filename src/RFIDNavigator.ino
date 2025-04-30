#include <SPI.h>         // SPI 통신 라이브러리 (RFID와 통신에 필요)
#include <MFRC522.h>     // MFRC522 RFID 리더 라이브러리

// RFID 핀 설정
#define SS_PIN 2         // RFID의 SDA 핀
#define RST_PIN 4        // RFID의 RST 핀

// 맵의 크기 정의 (8x8)
#define GRID_SIZE 8

// 경로 최대 길이 (최적화를 위해 제한)
#define MAX_PATH_LENGTH 32

MFRC522 rfid(SS_PIN, RST_PIN);  // RFID 리더 객체 생성

// 좌표를 표현하는 구조체 (x, y)
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
int qSize = 0;  // 큐 크기 (front/rear 없이 사용)

// 경로를 저장하는 배열
Point path[MAX_PATH_LENGTH];
int pathLength = 0;
int currentStep = 0;  // 현재 몇 번째 경로를 처리 중인지

// BFS 알고리즘으로 최단 경로 탐색
int bfs(Point start, Point goal, uint8_t prevX[GRID_SIZE][GRID_SIZE], uint8_t prevY[GRID_SIZE][GRID_SIZE]) {
  bool visited[GRID_SIZE][GRID_SIZE] = {false};   // 방문 여부 배열
  int dist[GRID_SIZE][GRID_SIZE];                 // 거리 배열

  // 초기화: 거리 -1, 이전 좌표 255로 설정
  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      dist[i][j] = -1;
      prevX[i][j] = 255;
      prevY[i][j] = 255;
    }
  }

  // 시작 지점을 큐에 추가
  qSize = 0;
  queue[qSize++] = start;
  visited[start.x][start.y] = true;
  dist[start.x][start.y] = 0;

  // BFS 탐색 루프
  for (int i = 0; i < qSize; i++) {
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
        prevX[nx][ny] = current.x;
        prevY[nx][ny] = current.y;
        queue[qSize++] = { (uint8_t)nx, (uint8_t)ny };  // 다음 탐색 지점 추가
      }
    }
  }

  return -1;  // 경로 없음
}

// BFS 결과를 바탕으로 경로를 역추적하여 path[]에 저장
void savePath(Point goal, uint8_t prevX[GRID_SIZE][GRID_SIZE], uint8_t prevY[GRID_SIZE][GRID_SIZE]) {
  pathLength = 0;
  uint8_t x = goal.x;
  uint8_t y = goal.y;

  // 목표 지점부터 시작해 이전 위치를 따라가며 저장
  while (x != 255 && y != 255 && pathLength < MAX_PATH_LENGTH) {
    path[pathLength++] = {x, y};
    uint8_t px = prevX[x][y];
    uint8_t py = prevY[x][y];
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

// 시작 위치 (고정): (0, 0)
Point startPoint = {0, 0};

void setup() {
  Serial.begin(115200);  // 시리얼 통신 시작
  SPI.begin();           // SPI 시작
  rfid.PCD_Init();       // RFID 초기화
  Serial.println("RFID 리더 준비 완료!");
}

void loop() {
  static bool hasPath = false;  // 경로 유무 상태 저장

  if (!hasPath) {
    char rfidUID[16];         // UID 저장 버퍼
    readRFID(rfidUID);        // UID 읽기

    if (rfidUID[0] != '\0') { // UID가 유효한 경우
      Point goal = findValueByKey(rfidUID); // 목표 지점 검색
      if (goal.x != 255 && goal.y != 255) { // 유효 좌표인지 확인
        uint8_t prevX[GRID_SIZE][GRID_SIZE];
        uint8_t prevY[GRID_SIZE][GRID_SIZE];
        int result = bfs(startPoint, goal, prevX, prevY); // BFS 수행
        if (result != -1) { // 경로 존재 시
          savePath(goal, prevX, prevY);  // 경로 저장
          hasPath = true;                // 다음 loop에서 이동 시작
        }
      }
    }
  } else {
    // 경로를 따라 한 칸씩 이동
    if (currentStep < pathLength) {
      Serial.print("이동: ");
      Serial.print(path[currentStep].x);
      Serial.print(", ");
      Serial.println(path[currentStep].y);
      currentStep++;
      delay(500); // 이동 간 시간 지연
    } else {
      hasPath = false;  // 경로 끝나면 상태 초기화
    }
  }
}
