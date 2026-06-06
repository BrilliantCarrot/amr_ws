#include "control_mpc/cbf_filter.hpp"

#include <cmath>
#include <algorithm>
#include <cstdlib>   // malloc, free
#include <iostream>

namespace control_mpc
{

// ============================================================
// 생성자
// ============================================================
CbfFilter::CbfFilter(const Params & params)
: params_(params)
{
  osqp_set_default_settings(&settings_);

  settings_.verbose            = false;
  settings_.warm_start         = true;
  settings_.max_iter           = 200;
  settings_.eps_abs            = 1e-4;
  settings_.eps_rel            = 1e-4;
  settings_.scaled_termination = true;
  settings_.polish             = false;
}


// ============================================================
// computeH() — lookahead point 기준 barrier 함수
//
// pf = [rx + L cos(yaw), ry + L sin(yaw)]
// h  = (pf_x - obs_x)^2 + (pf_y - obs_y)^2 - (r + d_safe)^2
// ============================================================
double CbfFilter::computeH(
  double rx, double ry, double ryaw,
  const CbfObstacle & obs) const
{
  const double L = params_.lookahead;

  // 전방 lookahead point 기준으로 barrier 정의함
  double px = rx + L * std::cos(ryaw);
  double py = ry + L * std::sin(ryaw);

  double dx = px - obs.x;
  double dy = py - obs.y;
  double r  = obs.radius + params_.d_safe;

  return dx * dx + dy * dy - r * r;
}


// ============================================================
// computeAb() — lookahead point 기준 h_dot 계수 계산
//
// pf_dot = [cos(yaw)*v - L sin(yaw)*omega,
//           sin(yaw)*v + L cos(yaw)*omega]
//
// h_dot = a_v * v + a_w * omega
// ============================================================
void CbfFilter::computeAb(
  double rx, double ry, double ryaw,
  const CbfObstacle & obs,
  double & a_v, double & a_w) const
{
  const double L = params_.lookahead;

  double px = rx + L * std::cos(ryaw);
  double py = ry + L * std::sin(ryaw);

  double dx = px - obs.x;
  double dy = py - obs.y;

  a_v = 2.0 * (dx * std::cos(ryaw) + dy * std::sin(ryaw));
  a_w = 2.0 * (-dx * L * std::sin(ryaw) + dy * L * std::cos(ryaw));
}


// ============================================================
// filter() — CBF-QP Safety Filter 메인 함수
//
// 변수:
//   x = [v, omega, delta_0, ..., delta_{N-1}]
//
// 목적함수:
//   min q_v_dev (v-v_nom)^2 + q_w_dev (omega-w_nom)^2 + slack_p * sum(delta_i^2)
//
// 제약:
//   a_v_i * v + a_w_i * omega + delta_i >= -gamma * h_i
//   delta_i >= 0
//   v_min <= v <= v_max
//   w_min <= omega <= w_max
//
// 구현 포인트:
//   - lookahead point 기반 barrier 사용
//   - 가까운 전방 장애물만 active set으로 사용
//   - v 편차 비용을 크게, w 편차 비용을 작게 둬서
//     정지보다 회전을 먼저 쓰도록 유도함
// ============================================================
bool CbfFilter::filter(
  double robot_x, double robot_y, double robot_yaw,
  double v_nom,   double w_nom,
  const std::vector<CbfObstacle> & obstacles,
  double & v_safe, double & w_safe)
{
  const bool dbg_print = false;

  // if (dbg_print) {
  //   std::cerr
  //     << "[CBF dbg input] obs=" << obstacles.size()
  //     << " robot=(" << robot_x << ", " << robot_y << ")"
  //     << " yaw=" << robot_yaw
  //     << " v_nom=" << v_nom
  //     << " w_nom=" << w_nom
  //     << std::endl;
  // }

  if (obstacles.empty()) {
    v_safe = v_nom;
    w_safe = w_nom;
    return true;
  }

  struct ActiveObs {
    CbfObstacle obs;
    double clearance;
    double forward_proj;
    double h;
  };

  std::vector<ActiveObs> active;
  active.reserve(obstacles.size());

  const double L  = params_.lookahead;
  const double px = robot_x + L * std::cos(robot_yaw);
  const double py = robot_y + L * std::sin(robot_yaw);

  for (const auto & obs : obstacles) {





    double dx = obs.x - px;
double dy = obs.y - py;

// 로봇 중심 기준 상대 위치도 따로 계산함
const double dx_robot = obs.x - robot_x;
const double dy_robot = obs.y - robot_y;

double center_dist = std::hypot(dx, dy);
if (center_dist < 1e-6) center_dist = 1e-6;

const double effective_r = obs.radius + params_.d_safe;
const double clearance = center_dist - effective_r;

// lookahead point 기준 전방 거리임
const double forward_proj = dx * std::cos(robot_yaw) + dy * std::sin(robot_yaw);

// 로봇 중심 기준 전방 거리임
const double forward_robot =
  dx_robot * std::cos(robot_yaw) + dy_robot * std::sin(robot_yaw);

if (clearance > params_.react_dist) continue;


if (forward_robot < 0.05) continue;



const double r = obs.radius + params_.d_safe;
const double h = center_dist * center_dist - r * r;


    active.push_back({obs, clearance, forward_proj, h});
  }

  if (active.empty()) {

    // if (dbg_print) {
    //   std::cerr << "[CBF dbg skip] no active obstacle" << std::endl;
    // }
    v_safe = v_nom;
    w_safe = w_nom;
    return true;
  }

  std::sort(active.begin(), active.end(),
    [](const ActiveObs & a, const ActiveObs & b) {
      return a.clearance < b.clearance;
    });

  if (params_.max_active_obstacles > 0 &&
      static_cast<int>(active.size()) > params_.max_active_obstacles) {
    active.resize(params_.max_active_obstacles);
  }

  double dbg_av = 0.0, dbg_aw = 0.0;
  computeAb(robot_x, robot_y, robot_yaw, active.front().obs, dbg_av, dbg_aw);

  double dbg_h   = active.front().h;
  double dbg_rhs = -params_.gamma * dbg_h;
  double dbg_lhs_nom = dbg_av * v_nom + dbg_aw * w_nom;

  double v_nom_eff = v_nom;
  double w_nom_eff = w_nom;









const double recovery_enter_clearance = 0.10;
const double recovery_exit_clearance  = 0.24;

const int preferred_turn_dir = (dbg_aw >= 0.0) ? 1 : -1;

// 가까운 전방 장애물이 있으면 recovery 모드 진입함
// 기존 코드에는 recovery_active_를 true로 만드는 부분이 없어 사실상 죽은 로직이었음
if (!recovery_active_ &&
    active.front().clearance < recovery_enter_clearance &&
    active.front().forward_proj > -0.05) {
  recovery_active_ = true;
  turn_dir_hold_ = preferred_turn_dir;
}

// 충분히 멀어졌거나, 장애물이 뒤쪽으로 넘어가면 recovery 해제함
if (recovery_active_ &&
    (active.front().clearance > recovery_exit_clearance ||
     active.front().forward_proj < -0.10)) {
  recovery_active_ = false;
  turn_dir_hold_ = 0;
}

const bool recovery_mode = recovery_active_;





double q_v_dev_eff = params_.q_v_dev;
double q_w_dev_eff = params_.q_w_dev;

if (recovery_mode) {
  // 매우 가까운 경우만 정지에 가깝게 두고, 그 밖에는 천천히 전진을 유지함.
  // 동적 장애물에서 v=0/0.03, w=±max 패턴이 반복되면 stop-and-go가 커진다.
  if (active.front().clearance < 0.03) {
    v_nom_eff = 0.0;
  } else if (active.front().clearance < 0.10) {
    v_nom_eff = std::clamp(v_nom, 0.04, 0.07);
  } else {
    v_nom_eff = std::clamp(v_nom, 0.06, 0.10);
  }

  if (turn_dir_hold_ == 0) {
    turn_dir_hold_ = preferred_turn_dir;
  }

  const double w_escape_mag = 0.22;
  double w_escape = static_cast<double>(turn_dir_hold_) * w_escape_mag;

  // 각속도 제한 안에 넣음
  w_escape = std::max(params_.w_min, std::min(params_.w_max, w_escape));
  w_nom_eff = w_escape;

  // omega 변경을 덜 싫어하게 하되, 최대 회전으로 바로 붙는 현상은 줄임.
  q_w_dev_eff = 0.20 * params_.q_w_dev;
}





  int N     = static_cast<int>(active.size());
  int n_var = N + 2;
  int n_con = 2 * N + 2;
  int P_nnz = n_var;
  int A_nnz = 4 * N + 2;

  if (dbg_print) {
    std::cerr
      << "[CBF dbg pre] N=" << N
      << " clr=" << active.front().clearance
      << " fwd=" << active.front().forward_proj
      << " h=" << dbg_h
      << " av=" << dbg_av
      << " aw=" << dbg_aw
      << " lhs_nom=" << dbg_lhs_nom
      << " rhs=" << dbg_rhs
      << " v_nom=" << v_nom
      << " w_nom=" << w_nom
      << std::endl;
  }

  constexpr c_float INF = static_cast<c_float>(1e30);

  // ── P 행렬 ────────────────────────────────────────────────
  c_int   * P_p = (c_int  *)malloc(sizeof(c_int)   * (n_var + 1));
  c_int   * P_i = (c_int  *)malloc(sizeof(c_int)   * P_nnz);
  c_float * P_x = (c_float*)malloc(sizeof(c_float) * P_nnz);

  for (int k = 0; k < n_var; ++k) {
    P_p[k] = k;
    P_i[k] = k;

    if (k == 0) {
      P_x[k] = static_cast<c_float>(2.0 * params_.q_v_dev);
    } else if (k == 1) {
      P_x[k] = static_cast<c_float>(2.0 * q_w_dev_eff);
    } else {

      
      const double slack_p_eff = params_.slack_p;
      P_x[k] = static_cast<c_float>(2.0 * slack_p_eff);

    }
  }

  P_p[n_var] = P_nnz;

  csc * P_mat = csc_matrix(n_var, n_var, P_nnz, P_x, P_i, P_p);

  // ── q 벡터 ────────────────────────────────────────────────
  c_float * q_vec = (c_float*)malloc(sizeof(c_float) * n_var);

  q_vec[0] = static_cast<c_float>(-2.0 * q_v_dev_eff * v_nom_eff);
  q_vec[1] = static_cast<c_float>(-2.0 * q_w_dev_eff * w_nom_eff);

  for (int k = 2; k < n_var; ++k) {
    q_vec[k] = 0.0f;
  }

  // ── A 행렬 ────────────────────────────────────────────────
  c_int   * A_p = (c_int  *)malloc(sizeof(c_int)   * (n_var + 1));
  c_int   * A_i = (c_int  *)malloc(sizeof(c_int)   * A_nnz);
  c_float * A_x = (c_float*)malloc(sizeof(c_float) * A_nnz);

  int nz = 0;

  // col 0: v
  A_p[0] = 0;
  for (int i = 0; i < N; ++i) {
    double a_v = 0.0, a_w = 0.0;
    computeAb(robot_x, robot_y, robot_yaw, active[i].obs, a_v, a_w);
    A_i[nz] = i;
    A_x[nz] = static_cast<c_float>(a_v);
    ++nz;
  }
  A_i[nz] = 2 * N;
  A_x[nz] = 1.0f;
  ++nz;

  // col 1: omega
  A_p[1] = nz;
  for (int i = 0; i < N; ++i) {
    double a_v = 0.0, a_w = 0.0;
    computeAb(robot_x, robot_y, robot_yaw, active[i].obs, a_v, a_w);
    A_i[nz] = i;
    A_x[nz] = static_cast<c_float>(a_w);
    ++nz;
  }
  A_i[nz] = 2 * N + 1;
  A_x[nz] = 1.0f;
  ++nz;

  // col 2+i: delta_i
  for (int k = 0; k < N; ++k) {
    A_p[2 + k] = nz;
    A_i[nz] = k;
    A_x[nz] = 1.0f;
    ++nz;

    A_i[nz] = N + k;
    A_x[nz] = 1.0f;
    ++nz;
  }
  A_p[n_var] = nz;

  csc * A_mat = csc_matrix(n_con, n_var, A_nnz, A_x, A_i, A_p);

  // ── l, u 벡터 ──────────────────────────────────────────────
  c_float * l_vec = (c_float*)malloc(sizeof(c_float) * n_con);
  c_float * u_vec = (c_float*)malloc(sizeof(c_float) * n_con);

  for (int i = 0; i < N; ++i) {
    double h_i = active[i].h;
    l_vec[i] = static_cast<c_float>(-params_.gamma * h_i);
    u_vec[i] = INF;
  }

  for (int i = 0; i < N; ++i) {
    l_vec[N + i] = 0.0f;
    u_vec[N + i] = INF;
  }

const double v_lower = params_.forbid_reverse ? 0.0 : params_.v_min;

l_vec[2 * N]     = static_cast<c_float>(v_lower);
u_vec[2 * N]     = static_cast<c_float>(params_.v_max);
l_vec[2 * N + 1] = static_cast<c_float>(params_.w_min);
u_vec[2 * N + 1] = static_cast<c_float>(params_.w_max);

  // ── OSQP data ──────────────────────────────────────────────
  OSQPData data;
  data.n = n_var;
  data.m = n_con;
  data.P = P_mat;
  data.q = q_vec;
  data.A = A_mat;
  data.l = l_vec;
  data.u = u_vec;

  OSQPWorkspace * workspace = nullptr;
  c_int status = osqp_setup(&workspace, &data, &settings_);

  if (status != 0 || workspace == nullptr) {
    free(P_p); free(P_i); free(P_x); free(P_mat);
    free(A_p); free(A_i); free(A_x); free(A_mat);
    free(q_vec); free(l_vec); free(u_vec);

    v_safe = v_nom;
    w_safe = w_nom;
    return false;
  }

  // q/l/u는 setup 후 해제 가능함
  free(q_vec);
  free(l_vec);
  free(u_vec);

  // ── solve ─────────────────────────────────────────────────
  osqp_solve(workspace);

  // std::cout
  // << "[CBF dbg solve] status=" << workspace->info->status_val
  // << std::endl;

  bool success = false;
  if (workspace->info->status_val == OSQP_SOLVED ||
      workspace->info->status_val == OSQP_SOLVED_INACCURATE) {
    v_safe = static_cast<double>(workspace->solution->x[0]);
    w_safe = static_cast<double>(workspace->solution->x[1]);

    v_safe = std::clamp(v_safe, v_lower, params_.v_max);
    w_safe = std::clamp(w_safe, params_.w_min, params_.w_max);

    // if (params_.forbid_reverse) {
    //   v_safe = std::clamp(v_safe, 0.0, params_.v_max);
    // } else {
    //   v_safe = std::clamp(v_safe, params_.v_min, params_.v_max);
    // }

    // w_safe = std::clamp(w_safe, params_.w_min, params_.w_max);

    if (std::abs(v_safe) < params_.v_eps) v_safe = 0.0;
    if (std::abs(w_safe) < 1e-3)          w_safe = 0.0;

    success = true;
  } else {
    v_safe = v_nom;
    w_safe = w_nom;
  }

  // ── cleanup ───────────────────────────────────────────────
  free(P_mat->p); free(P_mat->i); free(P_mat->x); free(P_mat);
  free(A_mat->p); free(A_mat->i); free(A_mat->x); free(A_mat);
  osqp_cleanup(workspace);

  // std::cout
  // << "[CBF dbg out] v_nom=" << v_nom
  // << " w_nom=" << w_nom
  // << " v_safe=" << v_safe
  // << " w_safe=" << w_safe
  // << std::endl;

  return success;
}

}  // namespace control_mpc
