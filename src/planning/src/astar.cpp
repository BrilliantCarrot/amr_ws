#include "planning/astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace planning{

// ============================================================
// inflateMap() — 장애물 팽창 (Obstacle Inflation)
//
// [왜 필요한가?]
//   A*는 로봇을 크기가 없는 점(point robot)으로 취급함.
//   하지만 실제 로봇은 반경이 있으므로, 장애물에 너무 가깝게
//   경로를 짜면 로봇 몸체가 충돌하게 됨.
//   → 해결책: 장애물을 미리 로봇 반경만큼 "부풀려"놓으면,
//     A*가 점 로봇 기준으로 경로를 짜도 실제 로봇은 안전하게 통과함.
//   이 기법을 Minkowski Sum 기반 장애물 팽창이라고 부름.
//
// [원형 팽창을 쓰는 이유]
//   정사각형(박스) 팽창보다 원형이 로봇의 실제 형태에 더 가까움.
//   dx² + dy² ≤ r² 조건으로 원 내부 셀만 선택함.
//
// [입력]
//   grid            : OccupancyGrid.data — 1D row-major 배열
//                     값 범위: 0(free) ~ 100(occupied), -1(unknown)
//   W, H            : 격자의 가로/세로 셀 수
//   inflation_cells : 팽창 반경 [셀 단위]
//                     = ceil(로봇 반경 / 맵 해상도)
//                     예) 로봇 반경 0.2m, 해상도 0.05m → 4셀
//
// [반환]
//   원본과 동일한 크기의 팽창된 격자
//   팽창된 셀은 모두 100(occupied)으로 마킹됨
// ============================================================
std::vector<int8_t> AStar::inflateMap(
  const std::vector<int8_t> & grid,
  int W, int H,
  int inflation_cells) const
{
  // 원본을 복사해서 inflated에 저장 — 원본은 건드리지 않음
  std::vector<int8_t> inflated = grid;

  // 원형 판별에 매번 sqrt를 쓰면 느리므로, r²을 미리 계산해둠
  // 나중에 dx²+dy² <= r² 조건으로 비교 (sqrt 불필요)
  int r2 = inflation_cells * inflation_cells;

  // 격자 전체를 순회하며 장애물/unknown 셀을 찾음
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      // 1D 배열에서 (x, y) 셀 값 꺼내기
      // OccupancyGrid는 row-major이므로 인덱스 = y*W + x
      int8_t val = grid[y * W + x];

      // free 셀(0~50)은 팽창 대상이 아니므로 건너뜀
      // occupied(>50) 또는 unknown(-1)인 셀만 팽창함
      if (val <= 50 && val >= 0) continue;

      // 이 장애물 셀(x, y)을 중심으로 inflation_cells 반경 내 모든 셀을 순회
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {

          // 원형 팽창 조건: (dx, dy)가 반경 r 원 내부에 있는지 확인
          // dx²+dy² > r² 이면 원 바깥 → 팽창 안 함 (모서리 잘라냄)
          if (dx * dx + dy * dy > r2) continue;

          // 팽창 대상 셀 좌표 계산
          int nx = x + dx;
          int ny = y + dy;

          // 격자 경계 밖으로 나가는 경우 무시
          if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;

          // 해당 셀을 100(완전 occupied)으로 마킹
          // → A*가 이 셀을 장애물로 인식해 경로에서 제외하게 됨
          inflated[ny * W + nx] = 100;
        }
      }
    }
  }

  return inflated;
}

// ============================================================
// findPath() — A* 8방향 경로 탐색
//
// [A* 알고리즘이란?]
//   다익스트라(Dijkstra) + 휴리스틱(Heuristic)의 조합.
//   다익스트라는 모든 방향을 균등하게 탐색하지만,
//   A*는 목표 방향으로 우선 탐색해 훨씬 빠름.
//
//   핵심 수식: f(n) = g(n) + h(n)
//     g(n) : 시작점 → 현재 셀 n까지의 실제 누적 이동 비용
//     h(n) : 현재 셀 n → 목표까지의 추정 비용 (휴리스틱)
//     f(n) : 이 셀을 거칠 때의 예상 총 비용
//   → f값이 가장 작은 셀부터 탐색 (min-heap 사용)
//
// [휴리스틱으로 유클리드 거리를 쓰는 이유]
//   8방향 이동이므로 대각선도 가능 → 유클리드 거리가 admissible
//   (실제 비용을 절대 과대추정하지 않음 → 최적 경로 보장)
//
// [자료구조]
//   open    : 탐색 대기 셀들의 min-heap (f값 기준으로 정렬)
//             priority_queue + greater<> → 가장 작은 f가 top()
//   g_cost  : 각 셀까지 도달하는 데 드는 현재까지 발견된 최솟값
//             초기값 = INF (아직 방문 안 한 셀)
//   parent  : 경로 역추적용. parent[n] = n으로 오기 직전 셀의 1D 인덱스
//             -1이면 미방문 또는 시작점
//
// [경로 역추적 원리]
//   목표 셀 도달 시 parent[]를 따라 goal → start 방향으로 거슬러 올라감
//   마지막에 reverse()로 뒤집어서 start → goal 순서로 반환
//
// [입력]
//   grid        : inflateMap() 결과 (팽창된 격자)
//   W, H        : 격자 크기
//   sx, sy      : 시작 셀 좌표 (격자 단위)
//   gx, gy      : 목표 셀 좌표 (격자 단위)
//
// [반환]
//   start → goal 순서의 격자 좌표 시퀀스 [(x,y), ...]
//   경로가 없으면 빈 벡터 {}
// ============================================================
std::vector<std::pair<int,int>> AStar::findPath(
  const std::vector<int8_t> & grid,
  int W, int H,
  int sx, int sy,
  int gx, int gy) const
{
  // --- 유효성 검사용 람다 ---

  // inBounds: 좌표가 격자 범위 [0,W) x [0,H) 안에 있는지 확인
  auto inBounds = [&](int x, int y) {
    return x >= 0 && x < W && y >= 0 && y < H;
  };

  // isBlocked: 셀이 통과 불가인지 확인
  //   v > 50  → occupied (장애물 또는 팽창 영역)
  //   v < 0   → unknown (slam_toolbox가 아직 탐색 못 한 영역)
  //   둘 다 로봇이 진입하면 안 됨
  auto isBlocked = [&](int x, int y) {
    int8_t v = grid[y * W + x];
    return v > 50 || v < 0;
  };

  // 시작/목표 좌표가 격자 밖이면 탐색 불가
  if (!inBounds(sx, sy) || !inBounds(gx, gy)) return {};
  // 목표 셀 자체가 장애물 안이면 도달 불가
  if (isBlocked(gx, gy)) return {};

  // --- 8방향 이동 벡터 정의 ---
  // 인덱스 0~3: 상하좌우 (직선, 비용 1.0)
  // 인덱스 4~7: 대각선 4방향 (비용 √2 ≈ 1.4142)
  // DX[i], DY[i]를 현재 좌표에 더하면 i번째 이웃 셀 좌표가 나옴
  const int DX[8] = { 1,  0, -1,  0,  1,  1, -1, -1};
  const int DY[8] = { 0,  1,  0, -1,  1, -1,  1, -1};
  const double COST[8] = {1.0, 1.0, 1.0, 1.0, 1.4142, 1.4142, 1.4142, 1.4142};

  // --- 배열 초기화 ---
  const double INF = std::numeric_limits<double>::infinity();

  // g_cost[y*W+x]: 시작점 → (x,y) 셀까지의 최소 이동 비용
  // 처음엔 모두 INF (아직 방문 전 = 비용을 모름)
  std::vector<double> g_cost(W * H, INF);

  // parent[y*W+x]: (x,y) 셀에 도달하기 직전 셀의 1D 인덱스
  // -1이면 아직 방문 안 했거나 시작점
  // 경로 역추적 시 parent[]를 거슬러 올라가면 전체 경로가 나옴
  std::vector<int> parent(W * H, -1);

  // --- min-heap(open list) 정의 ---
  // Cell = (f값, x좌표, y좌표) 튜플
  // std::greater<Cell> → f값이 작은 항목이 top()에 오는 min-heap
  // (기본 priority_queue는 max-heap이라 greater로 뒤집음)
  using Cell = std::tuple<double, int, int>;
  std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> open;

  // --- 휴리스틱 함수 h(x, y) ---
  // 현재 셀 (x,y) → 목표 (gx,gy) 까지의 유클리드 거리
  // std::hypot = sqrt((gx-x)² + (gy-y)²)
  // admissible: 실제 이동 비용(≥유클리드)을 절대 초과하지 않음
  // → A*가 항상 최적 경로를 반환하도록 보장
  auto h = [&](int x, int y) -> double {
    return std::hypot(static_cast<double>(gx - x),
                      static_cast<double>(gy - y));
  };

  // --- 시작점 등록 ---
  // 시작점의 g값은 0 (출발지에서 출발지까지 비용 = 0)
  // f = g + h = 0 + h(sx,sy) 로 open에 등록
  g_cost[sy * W + sx] = 0.0;
  open.emplace(h(sx, sy), sx, sy);

  // ============================================================
  // A* 메인 루프
  // open이 빌 때까지 반복. open이 비면 목표까지 경로 없음.
  // ============================================================
  while (!open.empty()) {
    // open에서 f값이 가장 작은 셀을 꺼냄 (가장 유망한 셀)
    // f_cur: 이 셀의 f값 / cx, cy: 이 셀의 좌표
    auto [f_cur, cx, cy] = open.top();
    open.pop();

    // --- 목표 도달 체크 ---
    if (cx == gx && cy == gy) {
      // 목표에 도달! parent[]를 역추적해서 경로를 복원함
      std::vector<std::pair<int,int>> path;

      // 목표 셀의 1D 인덱스에서 시작해서 parent를 따라 거슬러 올라감
      // parent[idx] == -1 이 되면 시작점을 지나쳤다는 뜻 → 종료
      int idx = gy * W + gx;
      while (idx != -1) {
        // 1D 인덱스 → 2D 좌표 변환: x = idx % W, y = idx / W
        path.emplace_back(idx % W, idx / W);
        idx = parent[idx];
      }

      // 역추적했으므로 현재 순서는 goal → start
      // reverse()로 뒤집어서 start → goal 순서로 만듦
      std::reverse(path.begin(), path.end());
      return path;
    }

    // --- stale(낡은) 항목 무시 ---
    // 같은 셀이 더 좋은 경로로 갱신되면 open에 중복 등록될 수 있음.
    // (A*는 closed list 대신 이 방식으로 중복 처리함 — Lazy Deletion)
    // f_cur이 현재 g_cost + h 보다 크다면, 이미 더 좋은 경로로
    // 이 셀이 처리됐다는 의미 → 이 항목은 무시하고 넘어감
    double cg = g_cost[cy * W + cx];
    if (f_cur > cg + h(cx, cy) + 1e-9) continue;

    // --- 8방향 이웃 셀 탐색 ---
    for (int i = 0; i < 8; ++i) {
      int nx = cx + DX[i];  // 이웃 셀 x좌표
      int ny = cy + DY[i];  // 이웃 셀 y좌표

      // 격자 경계 밖이면 무시
      if (!inBounds(nx, ny)) continue;
      // 장애물/unknown 셀이면 무시
      if (isBlocked(nx, ny)) continue;

      // 현재 셀(cx,cy)을 거쳐 이웃 셀(nx,ny)까지 오는 새 g값 계산
      // ng = 현재까지의 g값 + 이동 비용(직선 1.0 or 대각선 1.4142)
      double ng = cg + COST[i];

      // 이 경로가 기존에 기록된 g_cost보다 더 좋으면(비용이 작으면) 갱신
      if (ng < g_cost[ny * W + nx]) {
        // g_cost 갱신: 더 좋은 경로를 발견함
        g_cost[ny * W + nx] = ng;

        // parent 갱신: 이웃 셀의 직전 셀을 현재 셀(cx,cy)로 기록
        // 나중에 역추적할 때 이 정보를 사용함
        parent[ny * W + nx] = cy * W + cx;

        // 이웃 셀을 open에 등록 (f = ng + h)
        // 중복 등록되더라도 stale 체크로 나중에 걸러냄
        open.emplace(ng + h(nx, ny), nx, ny);
      }
    }
  }

  // open이 다 비었는데 목표에 도달 못 함 → 경로 없음
  return {};
}

} // namespace planning