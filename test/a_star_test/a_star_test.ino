#include <Arduino.h>

// 최대 경로 길이
#define MAX_PATH_LENGTH 100
#define MAX_QUEUE_SIZE 40
#define MAX_MAP_SIZE 8

// 구조체 최적화: f 제거 (계산으로 대체), 부모 좌표 제거
struct Node {
  uint8_t x, y;
  uint8_t g, h;
  Node() : x(0), y(0), g(0), h(0) {}
  Node(uint8_t x, uint8_t y, uint8_t g, uint8_t h)
    : x(x), y(y), g(g), h(h) {}
  uint8_t f() const { return g + h; }
};

// 우선순위 큐 (정렬 유지 삽입 방식)
Node openList[MAX_QUEUE_SIZE];
int openListSize = 0;

// 상태 관리: 0=미방문, 1=openList, 2=closedList
uint8_t status[MAX_MAP_SIZE][MAX_MAP_SIZE];

// 부모 좌표 저장 (역추적용)
uint8_t parentX[MAX_MAP_SIZE][MAX_MAP_SIZE];
uint8_t parentY[MAX_MAP_SIZE][MAX_MAP_SIZE];

// 최종 경로 저장
Node path[MAX_PATH_LENGTH];
int pathSize = 0;

// 맨해튼 휴리스틱
uint8_t heuristic(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

// 정렬 유지 삽입 (f 기준)
void pushOpenList(const Node& node) {
  if (openListSize >= MAX_QUEUE_SIZE) return;

  int i = openListSize - 1;
  while (i >= 0 && openList[i].f() > node.f()) {
    openList[i + 1] = openList[i];
    i--;
  }
  openList[i + 1] = node;
  openListSize++;
}

// 중복 노드가 openList에 있는지 확인
bool openListContains(uint8_t x, uint8_t y) {
  for (int i = 0; i < openListSize; i++) {
    if (openList[i].x == x && openList[i].y == y) {
      return true;
    }
  }
  return false;
}

// A* 탐색
bool findPathAStar(uint8_t sx, uint8_t sy, uint8_t gx, uint8_t gy) {
  openListSize = 0;
  pathSize = 0;
  memset(status, 0, sizeof(status));

  Node start(sx, sy, 0, heuristic(sx, sy, gx, gy));
  pushOpenList(start);
  status[sx][sy] = 1;
  parentX[sx][sy] = sx;
  parentY[sx][sy] = sy;

  int dx[4] = {0, 1, 0, -1};
  int dy[4] = {-1, 0, 1, 0};

  while (openListSize > 0) {
    Node current = openList[--openListSize];
    status[current.x][current.y] = 2;

    if (current.x == gx && current.y == gy) {
      uint8_t cx = gx, cy = gy;
      while (!(cx == sx && cy == sy)) {
        path[pathSize++] = Node(cx, cy, 0, 0);
        uint8_t px = parentX[cx][cy];
        uint8_t py = parentY[cx][cy];
        cx = px;
        cy = py;
      }
      path[pathSize++] = Node(sx, sy, 0, 0);
      break;
    }

    for (int dir = 0; dir < 4; dir++) {
      uint8_t nx = current.x + dx[dir];
      uint8_t ny = current.y + dy[dir];

      if (nx < MAX_MAP_SIZE && ny < MAX_MAP_SIZE && status[nx][ny] == 0) {
        uint8_t newG = current.g + 1;
        uint8_t newH = heuristic(nx, ny, gx, gy);

        if (!openListContains(nx, ny)) {
          Node neighbor(nx, ny, newG, newH);
          pushOpenList(neighbor);
          status[nx][ny] = 1;
          parentX[nx][ny] = current.x;
          parentY[nx][ny] = current.y;
        }
      }
    }
  }

  return pathSize > 0;
}

void setup() {
  Serial.begin(115200);

  uint8_t startX = 0, startY = 0;
  uint8_t goalX = 5, goalY = 5;

  findPathAStar(startX, startY, goalX, goalY);

  if (pathSize > 0) {
    Serial.println("경로 찾기 성공!");
    for (int i = pathSize - 1; i >= 0; i--) {
      Serial.print(path[i].x);
      Serial.print(", ");
      Serial.println(path[i].y);
    }
  } else {
    Serial.println("경로 찾기 실패");
  }
}

void loop() {
  // 사용 안 함
}
