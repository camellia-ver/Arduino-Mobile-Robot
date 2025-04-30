#define GRID_SIZE 8

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

struct Point {
  int x;
  int y;
};

int directions[4][2] = {{-1,0},{1,0},{0,-1},{0,1}}; // 상,하,좌,우

Point queue[GRID_SIZE * GRID_SIZE]; // 큐
int front = 0, rear = 0; // 큐의 앞과 뒤

bool visited[GRID_SIZE][GRID_SIZE]; // 방문 여부
int dist[GRID_SIZE][GRID_SIZE]; // 거리 배열
Point prev[GRID_SIZE][GRID_SIZE]; // 이전 위치 배열

void enqueue(int x,int y){
  queue[rear].x = x;
  queue[rear].y = y;
  rear = (rear + 1) % (GRID_SIZE * GRID_SIZE);
}

Point dequeue(){
  Point p = queue[front];
  front = (front + 1) % (GRID_SIZE * GRID_SIZE);
  return p;
}

// BFS
int bfs(Point start, Point goal){
  // 초기화
  memset(visited, false, sizeof(visited));
  memset(dist, -1, sizeof(dist)); // -1 : 미방문
  memset(prev, -1, sizeof(prev)); // -1로 초기화
  front = rear = 0;

  enqueue(start.x, start.y);
  visited[start.x][start.y] = true;
  dist[start.x][start.y] = 0; // 시작점 거리 설정

  while (front != rear){
    Point current = dequeue();

    // 목표 지점에 도달시 거리 반환
    if (current.x == goal.x && current.y == goal.y){
      return dist[current.x][current.y];
    }

    // 상, 하, 좌, 우로 이동
    for (int i = 0; i < 4; i++){
      int nx = current.x + directions[i][0];
      int ny = current.y + directions[i][1];

      // 유효 좌표 and 장애물 x and 방문 x
      if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE && grid[nx][ny] == 0 && !visited[nx][ny]){
        enqueue(nx,ny);
        visited[nx][ny] = true;
        dist[nx][ny] = dist[current.x][current.y] + 1;
        prev[nx][ny] = current; // 이전 위치 기록
      }
    }
  }

  return -1; // 경로가 없을 경우
}

void printPath(Point goal) {
  Point path[GRID_SIZE * GRID_SIZE];
  int pathLength = 0;
  
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

void setup() {
  Serial.begin(115200);
  Point startPoint = {0,0};
  Point goalPoint = {7,6};

  Serial.print("목표 위치: (");
  Serial.print(goalPoint.x); 
  Serial.print(", ");
  Serial.print(goalPoint.y);
  Serial.print(")");

  int result = bfs(startPoint,goalPoint);

  if (result != -1){
    Serial.print("최단 경로의 길이: ");
    Serial.println(result);
    printPath(goalPoint);
  }else{
    Serial.println("목표 지점에 도달할 수 없습니다.");
  }
}

void loop() {
}
