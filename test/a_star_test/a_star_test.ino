#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <algorithm>

struct Node {
  int x, y, g, h, f, px, py;
  Node(int x, int y, int g, int h, int f, int px, int py)
    : x(x), y(y), g(g), h(h), f(f), px(px), py(py) {}

  bool operator>(const Node& other) const {
    return f > other.f;
  }
};

std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
std::unordered_set<std::pair<int, int>, hash_pair> closedListSet;
std::vector<std::pair<int, int>> path;

int heuristic(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

std::vector<Node> findNeighborNodes(Node& current, Node& goal) {
  std::vector<Node> neighbors;
  int dx[4] = {0, 1, 0, -1};
  int dy[4] = {-1, 0, 1, 0};

  for (int dir = 0; dir < 4; dir++) {
    int nx = current.x + dx[dir];
    int ny = current.y + dy[dir];
    int g = current.g + 1;
    int h = heuristic(nx, ny, goal.x, goal.y);
    int f = g + h;
    neighbors.push_back(Node(nx, ny, g, h, f, current.x, current.y));
  }
  return neighbors;
}

bool findPathAStar(Node start, Node goal) {
  while (!openList.empty()) openList.pop();
  closedListSet.clear();
  path.clear();

  openList.push(start);
  std::unordered_map<std::pair<int, int>, std::pair<int, int>, hash_pair> parentMap;

  while (!openList.empty()) {
    Node current = openList.top();
    openList.pop();

    if (current.x == goal.x && current.y == goal.y) {
      std::pair<int, int> pos = {current.x, current.y};
      while (!(pos.first == start.x && pos.second == start.y)) {
        path.push_back(pos);
        pos = parentMap[pos];
      }
      path.push_back({start.x, start.y});
      std::reverse(path.begin(), path.end());
      return true;
    }

    closedListSet.insert({current.x, current.y});
    std::vector<Node> neighbors = findNeighborNodes(current, goal);
    for (const Node& neighbor : neighbors) {
      std::pair<int, int> npos = {neighbor.x, neighbor.y};
      if (closedListSet.find(npos) == closedListSet.end()) {
        openList.push(neighbor);
        parentMap[npos] = {current.x, current.y};
      }
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  Node start(0, 0, 0, heuristic(0, 0, 5, 5), 0, -1, -1);
  Node goal(5, 5, 0, 0, 0, -1, -1);

  if (findPathAStar(start, goal)) {
    Serial.println("경로 찾기 성공!");
    for (auto& p : path) {
      Serial.print(p.first);
      Serial.print(", ");
      Serial.println(p.second);
    }
  } else {
    Serial.println("경로 찾기 실패");
  }
}

void loop() {}
