#ifndef PLANNING__ASTAR_HPP_
#define PLANNING__ASTAR_HPP_

// ============================================================
// astar.hpp — 격자 기반 A* 경로 탐색기 헤더
//
// [역할]
//   nav_msgs/OccupancyGrid 데이터를 입력받아
//   시작 격자 셀에서 목표 격자 셀까지의 최단 경로를 탐색함.
//
// [ROS2 의존성 없음]
//   순수 C++17 — 단위 테스트 용이
//   MpcCore와 동일한 설계 원칙: 알고리즘 ↔ ROS2 인터페이스 분리
//
// [지원 기능]
//   1. inflateMap()  : 장애물 팽창 (로봇 반경만큼 occupied 확장)
//   2. findPath()    : A* 8-방향 탐색
// ============================================================

#include <vector>
#include <utility>
#include <cstdint>

namespace planning
{

class AStar
{
public:
  // ============================================================
  // inflateMap() — 장애물 팽창
  //
  // [원리]
  //   원본 격자에서 occupied(>50) 또는 unknown(-1)인 셀을
  //   반경 inflation_cells 만큼 원형으로 확장(Minkowski sum).
  //   로봇을 점(point robot)으로 취급하되, 장애물을 미리 팽창해두면
  //   경로 탐색 시 별도의 충돌 반경 계산이 필요 없어짐.
  //
  // [입력]
  //   grid           : OccupancyGrid.data (row-major, 1D)
  //   width, height  : 격자 크기
  //   inflation_cells: 팽창 반경 [셀 수] = ceil(robot_radius / resolution)
  //
  // [반환]
  //   팽창된 격자 (원본과 동일한 크기)
  // ============================================================
  std::vector<int8_t> inflateMap(
    const std::vector<int8_t> & grid,
    int width, int height,
    int inflation_cells) const;

  // ============================================================
  // findPath() — A* 경로 탐색 (8-방향 연결)
  //
  // [입력]
  //   grid          : inflateMap() 결과 (팽창된 격자)
  //   width, height : 격자 크기
  //   sx, sy        : 시작 셀 좌표
  //   gx, gy        : 목표 셀 좌표
  //
  // [반환]
  //   시작 → 목표 격자 좌표 시퀀스 [(x,y), ...]
  //   경로 없으면 빈 벡터
  //
  // [비용]
  //   직선 이동 = 1.0,  대각선 이동 = sqrt(2) ≈ 1.414
  //   휴리스틱  = 유클리드 거리 (admissible → 최적해 보장)
  // ============================================================
  std::vector<std::pair<int, int>> findPath(
    const std::vector<int8_t> & grid,
    int width, int height,
    int sx, int sy,
    int gx, int gy) const;
};

}  // namespace planning

#endif  // PLANNING__ASTAR_HPP_
