#pragma once

// ============================================================
// cbf_filter.hpp — CBF-QP Safety Filter 헤더
//
// [역할]
//   MPC nominal 입력 u_nom = [v_nom, ω_nom]을 받아서
//   CBF 조건을 만족하는 u_safe = [v_safe, ω_safe]를 반환하는 필터.
//
// [CBF-QP 공식화]
//   변수: u = [v, ω, δ₀, ..., δ_{N-1}]  (N+2개)
//
//   목적함수:
//     min  (v - v_nom)² + (ω - ω_nom)² + slack_p * Σδᵢ²
//
//   제약:
//     CBF Soft:  aᵢ·v + δᵢ ≥ -γ·hᵢ   (i = 0..N-1)
//     Slack:     δᵢ ≥ 0
//     속도:      v_min ≤ v ≤ v_max
//                w_min ≤ ω ≤ w_max
//
//   h(x)  = (x-xo)² + (y-yo)² - r²    (barrier 함수)
//   ḣ     = a(x)·v                     (Lie derivative)
//   a(x)  = 2[(x-xo)cosθ + (y-yo)sinθ] (v의 계수)
//
// [설계 방침]
//   매 filter() 호출마다 QP를 새로 셋업하고 풀음.
//   → update API의 lifetime/double-free 문제를 원천 차단.
//   → 소규모 QP(변수 ~7개, 제약 ~12개)라 속도 문제 없음.
//
//   CSC 배열은 c_malloc으로 할당 → osqp_cleanup 시 안전하게 해제됨.
// ============================================================

#include <vector>
#include <cmath>
#include <osqp.h>

namespace control_mpc
{

// 장애물 정보 구조체
// radius = 장애물 반경 + 로봇 반경 (obsCallback에서 이미 inflation 포함)
struct CbfObstacle {
  double x;
  double y;
  double radius;
};

class CbfFilter
{
public:
  struct Params {
    // γ: class-K 계수 (α(h) = γ*h)
    // 클수록 경계에서 강하게 밀어냄. 너무 크면 infeasible 위험.
    // 추천 초기값: 1.0 ~ 3.0
    double gamma   = 0.5;

    // slack_p: Soft CBF slack 변수 δ의 페널티 가중치
    // 클수록 Hard CBF에 가까워짐. 추천: 500.0
    double slack_p = 1000.0;

    // d_safe: 추가 안전 마진 [m]
    // 0이면 obs.radius만 사용. 여유를 더 주고 싶으면 양수로 설정.
    double d_safe  = 0.25;

    // lookahead distance [m]
    // 로봇 중심 대신 전방 L 지점에 대해 barrier를 정의
    // L이 커질수록 회전(ω)이 barrier 미분에 더 강하게 반영됨
    double lookahead = 0.65;

    // 속도 제한 (mpc_params_와 동일하게 설정할 것)
    double v_max   =  0.15;
    double v_min   = -0.10;
    double w_max   =  1.0;
    double w_min   = -1.0;
    double q_v_dev = 40.0;   // v_nom에서 벗어나는 비용
    double q_w_dev = 0.15;    // w_nom에서 벗어나는 비용
    double v_eps   = 0.0005;  // 너무 작은 속도는 0으로 정리
    bool   forbid_reverse = true;
    double react_dist = 1.0;      // 이 거리 안쪽 장애물만 CBF에 사용
    double rear_margin = -0.05;   // 너무 뒤에 있으면 제외
    int    max_active_obstacles = 2;
  };

  explicit CbfFilter(const Params & params);
  ~CbfFilter() = default;  // OSQP workspace는 filter() 내부에서 매번 cleanup

  // ──────────────────────────────────────────────────────────
  // filter() — CBF-QP Safety Filter 메인 함수
  //
  // [입력]
  //   robot_x, robot_y, robot_yaw : 현재 로봇 상태 (map frame)
  //   v_nom, w_nom                : MPC nominal 입력
  //   obstacles                   : 장애물 목록
  //
  // [출력]
  //   v_safe, w_safe : CBF 조건을 만족하는 안전 입력
  //
  // [반환값]
  //   true  : QP 성공 → v_safe, w_safe 사용
  //   false : QP 실패 → v_safe = v_nom, w_safe = w_nom (fallback)
  // ──────────────────────────────────────────────────────────
  bool filter(
    double robot_x, double robot_y, double robot_yaw,
    double v_nom,   double w_nom,
    const std::vector<CbfObstacle> & obstacles,
    double & v_safe, double & w_safe);

private:
  Params       params_;
  OSQPSettings settings_;

  // 회피 방향 흔들림 방지용 상태임
  bool recovery_active_ = false;
  int turn_dir_hold_ = 0;

  // h(x) = (x-xo)² + (y-yo)² - (radius + d_safe)²
  // h > 0: 안전, h = 0: 경계, h < 0: 위험
  double computeH(
    double rx, double ry, double ryaw,
    const CbfObstacle & obs) const;

  // lookahead point:
  //   px = rx + L cos(ryaw)
  //   py = ry + L sin(ryaw)
  //
  // h = (px-xo)^2 + (py-yo)^2 - (radius + d_safe)^2
  void computeAb(
    double rx, double ry, double ryaw,
    const CbfObstacle & obs,
    double & a_v, double & a_w) const;
};

}  // namespace control_mpc