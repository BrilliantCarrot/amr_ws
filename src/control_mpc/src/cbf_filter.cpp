#include "control_mpc/cbf_filter.hpp"

#include <cmath>
#include <algorithm>
#include <cstdlib>   // malloc, free
#include <iostream>

// ============================================================================
// cbf_filter.cpp
//
// 이 파일은 MPC가 계산한 nominal command(v_nom, w_nom)를 그대로 내보내기 전에
// CBF(Control Barrier Function) 기반 QP Safety Filter를 한 번 통과시키는 구현임.
//
// 핵심 아이디어:
//   1) MPC는 목표 경로를 잘 따라가기 위한 "성능 제어기" 역할을 함.
//   2) CBF 필터는 MPC 출력이 장애물 안전조건을 위반할 수 있을 때만
//      v, omega를 최소한으로 수정하는 "안전 필터" 역할을 함.
//   3) QP 목적함수는 nominal command에서 최대한 덜 벗어나게 만들고,
//      제약식은 barrier condition을 만족하도록 강제함.
//
// 여기서 쓰는 barrier function은 lookahead point 기준임.
// 로봇 중심점이 아니라 로봇 전방 L[m] 지점의 가상 점을 기준으로 장애물과의
// 거리를 계산함. 이렇게 하면 diff-drive 로봇에서 omega가 barrier 변화율에
// 직접 영향을 주게 되어, CBF가 단순 감속뿐 아니라 회전 보정도 만들 수 있음.
//
// 주의할 점:
//   - OSQP는 C 라이브러리 스타일로 메모리를 다루므로, csc_matrix에 넘긴 배열은
//     std::vector가 아니라 malloc으로 직접 할당하고 직접 해제해야 함.
//   - 본 구현은 매 filter() 호출마다 작은 QP를 새로 구성하고 해제함.
//     active obstacle 개수가 작으면 계산량은 충분히 작지만, 장애물이 많아지면
//     workspace 재사용 구조로 바꾸는 것이 더 좋음.
//   - 동적 장애물 속도(vx, vy)를 CBF 제약에 직접 반영하는 형태는 아직 아님.
//     현재 코드는 주로 현재 순간의 장애물 위치 기준 안전 필터로 이해하면 됨.
// ============================================================================

namespace control_mpc
{

// ============================================================
// 생성자
// ============================================================
CbfFilter::CbfFilter(const Params & params)
: params_(params)
{
  // OSQP 설정 구조체를 기본값으로 초기화함.
  // OSQPSettings는 C struct라서 반드시 기본 설정 함수를 먼저 호출하는 것이 안전함.
  // 이후 필요한 항목만 덮어써야 미초기화 필드로 인한 예기치 않은 동작을 막을 수 있음.
  osqp_set_default_settings(&settings_);

  // verbose=false: OSQP 내부 반복 로그를 끔. 제어 루프에서 로그가 과도하게 나오면
  // 실시간성 확인이 어려워지므로 보통 false로 둠.
  settings_.verbose            = false;

  // warm_start=true: 이전 해를 다음 QP 초기값으로 활용할 수 있게 함.
  // 현재 구현은 workspace를 매번 새로 만들기 때문에 warm-start 효과는 제한적임.
  // 추후 workspace 재사용 구조로 바꾸면 이 설정이 더 의미 있어짐.
  settings_.warm_start         = true;

  // max_iter: OSQP 반복 횟수 상한임. 너무 작으면 부정확하고, 너무 크면 제어주기 지연 가능성 있음.
  // 여기서는 작은 QP라 200회 정도면 충분하다는 가정임.
  settings_.max_iter           = 200;

  // eps_abs, eps_rel: 최적화 수렴 허용오차임.
  // safety filter에서는 너무 느슨하면 제약 위반 가능성이 커지고, 너무 빡세면 solve time이 늘 수 있음.
  settings_.eps_abs            = 1e-4;
  settings_.eps_rel            = 1e-4;

  // scaled_termination=true: 내부 scaling된 문제 기준으로 종료조건을 판단하게 함.
  // 수치 스케일이 섞인 QP에서 수렴 판정이 안정적일 수 있음.
  settings_.scaled_termination = true;

  // polish=false: OSQP 후처리 정밀화 단계 비활성화함.
  // 실시간 제어에서는 약간의 정밀도보다 계산시간 안정성이 더 중요할 때가 많음.
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
  // lookahead 거리 L임.
  // L=0이면 로봇 중심점 기준 CBF가 되고, L>0이면 로봇 앞쪽 가상 점 기준 CBF가 됨.
  // diff-drive에서 중심점 기준 h_dot은 omega 항이 잘 안 들어오지만,
  // lookahead point 기준으로 잡으면 회전에 의해 전방점이 좌우로 움직이므로 omega 항이 생김.
  const double L = params_.lookahead;

  // 전방 lookahead point 기준으로 barrier 정의함
  double px = rx + L * std::cos(ryaw);
  double py = ry + L * std::sin(ryaw);

  // dx, dy는 장애물 중심에서 lookahead point까지의 상대 위치임.
  // 부호 자체보다 거리 제곱에 들어가므로 h 계산에서는 상대 거리 크기가 핵심임.
  double dx = px - obs.x;
  double dy = py - obs.y;

  // effective safety radius임.
  // obs.radius는 tracker가 추정한 장애물 반경이고, d_safe는 추가 안전마진임.
  // h>=0이면 lookahead point가 안전 원 밖에 있다는 뜻이고, h<0이면 안전영역 침범임.
  double r  = obs.radius + params_.d_safe;

  // h = 거리제곱 - 안전반경제곱임.
  // CBF에서는 h(x)>=0 집합을 safe set으로 정의함.
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
  // computeAb는 h_dot = a_v*v + a_w*omega 형태의 계수를 만드는 함수임.
  // QP 제약식이 선형부등식이 되려면, 입력 v와 omega에 대해 h_dot이 선형으로 표현되어야 함.
  // lookahead point를 사용하면 diff-drive 입력 [v, omega]가 h_dot에 둘 다 들어옴.
  const double L = params_.lookahead;

  double px = rx + L * std::cos(ryaw);
  double py = ry + L * std::sin(ryaw);

  // dx, dy는 장애물 기준 lookahead point의 상대 위치임.
  // h = dx^2 + dy^2 - r^2 이므로 h_dot = 2*dx*dx_dot + 2*dy*dy_dot 이 됨.
  double dx = px - obs.x;
  double dy = py - obs.y;

  // lookahead point 속도:
  //   px_dot = cos(yaw)*v - L*sin(yaw)*omega
  //   py_dot = sin(yaw)*v + L*cos(yaw)*omega
  // 이를 h_dot에 대입하면 v 앞의 계수가 a_v, omega 앞의 계수가 a_w가 됨.
  // a_v가 음수이면 전진속도 v가 커질수록 h가 감소함, 즉 장애물 쪽으로 접근한다는 뜻임.
  // a_w 부호는 현재 기하에서 어느 방향 회전이 h를 증가시키는지 알려줌.
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
  // 디버그 출력 주기를 줄이기 위한 카운터임.
  // 제어 루프가 50Hz라면 매번 출력하면 터미널/CPU 부하가 커지므로 25회에 한 번만 찍는 구조임.
  // 현재 실제 dbg_print 사용부는 대부분 주석 처리되어 있음.
  static int dbg_count = 0;
  const bool dbg_print = (++dbg_count % 25 == 0);

  // if (dbg_print) {
  //   std::cerr
  //     << "[CBF dbg input] obs=" << obstacles.size()
  //     << " robot=(" << robot_x << ", " << robot_y << ")"
  //     << " yaw=" << robot_yaw
  //     << " v_nom=" << v_nom
  //     << " w_nom=" << w_nom
  //     << std::endl;
  // }

  // 장애물이 하나도 없으면 safety filter가 할 일이 없음.
  // 이 경우 nominal command를 그대로 통과시킴.
  // return true는 "필터 계산 실패가 아니라, 수정할 필요가 없었다"는 의미로 보면 됨.
  if (obstacles.empty()) {
    v_safe = v_nom;
    w_safe = w_nom;
    return true;
  }

  // CBF-QP에 실제로 넣을 장애물 후보 구조체임.
  // 모든 raw obstacle을 다 QP에 넣으면 제약 수가 늘고, 뒤쪽/먼 장애물이 불필요하게 제어를 방해할 수 있음.
  // 그래서 먼저 active set을 만들고 가까운 전방 장애물만 사용함.
  struct ActiveObs {
    CbfObstacle obs;      // 원본 장애물 정보임. 위치, 반경 등이 들어 있음.
    double clearance;     // lookahead point와 장애물 안전반경 사이의 여유거리임. 작을수록 위험함.
    double forward_proj;  // lookahead point 기준 전방 투영 거리임. 디버그 및 recovery 해제에 사용함.
    double h;             // CBF barrier 값임. h>=0이면 안전, h<0이면 safety set 내부 침범임.
  };

  std::vector<ActiveObs> active;
  active.reserve(obstacles.size());

  // 현재 로봇 pose에서 lookahead point 좌표를 한 번 계산해둠.
  // active obstacle 선별도 이 lookahead point 기준 거리로 판단함.
  const double L  = params_.lookahead;
  const double px = robot_x + L * std::cos(robot_yaw);
  const double py = robot_y + L * std::sin(robot_yaw);

  // raw obstacle들을 순회하면서 QP에 넣을 active obstacle만 선별함.
  // 현재 기준은:
  //   1) safety margin 기준으로 너무 멀면 제외함.
  //   2) 로봇 중심 기준으로 이미 뒤쪽이면 제외함.
  // 이 필터링이 중요함. 지나간 장애물을 계속 active에 남기면 로봇이 불필요하게 멈추거나 회전할 수 있음.
  for (const auto & obs : obstacles) {





    // lookahead point에서 장애물까지의 상대 위치임.
    // 여기서는 obs - lookahead 방향으로 계산하지만, 거리 계산에는 부호 영향이 없음.
    double dx = obs.x - px;
double dy = obs.y - py;

// 로봇 중심 기준 상대 위치도 따로 계산함.
// lookahead point 기준으로만 뒤/앞을 판단하면, 장애물을 거의 지난 뒤에도
// lookahead 기준으로는 애매하게 active로 남을 수 있음.
// 그래서 "지나간 장애물 제거"는 로봇 중심 기준 forward_robot을 따로 씀.
const double dx_robot = obs.x - robot_x;
const double dy_robot = obs.y - robot_y;

// lookahead point와 장애물 중심 사이 거리임.
// hypot은 sqrt(dx^2+dy^2)를 안정적으로 계산함.
double center_dist = std::hypot(dx, dy);

// 수치적으로 완전히 겹치는 경우 0 나눗셈/불안정 계산을 막기 위한 작은 하한임.
// 이 코드에서는 직접 나눗셈은 많지 않지만, 디버그/확장 시 안정성을 위해 둔 것으로 볼 수 있음.
if (center_dist < 1e-6) center_dist = 1e-6;

// 장애물 실제 반경 + 추가 안전거리임.
// clearance = 현재 거리 - 안전반경 이므로, clearance가 0보다 작으면 안전반경 안쪽임.
const double effective_r = obs.radius + params_.d_safe;
const double clearance = center_dist - effective_r;

// lookahead point 기준 전방 거리임
const double forward_proj = dx * std::cos(robot_yaw) + dy * std::sin(robot_yaw);

// 로봇 중심 기준 전방 거리임
const double forward_robot =
  dx_robot * std::cos(robot_yaw) + dy_robot * std::sin(robot_yaw);

// react_dist보다 충분히 멀면 CBF가 개입할 필요가 없으므로 제외함.
// react_dist는 "CBF가 관심을 갖기 시작하는 거리 여유"로 이해하면 됨.
if (clearance > params_.react_dist) continue;


// 로봇 중심 기준으로 이미 뒤쪽에 있는 장애물은 제외함.
// 0.05m는 약간의 전방 여유를 둔 값임. 너무 작으면 지나간 장애물이 오래 남고,
// 너무 크면 옆에 가까운 장애물을 너무 빨리 버릴 수 있음.
if (forward_robot < 0.05) continue;



// CBF barrier 값 h를 계산함.
// h = 거리제곱 - 안전반경제곱 이므로:
//   h > 0 : 안전반경 밖
//   h = 0 : 안전경계
//   h < 0 : 안전반경 안쪽
const double r = obs.radius + params_.d_safe;
const double h = center_dist * center_dist - r * r;


    active.push_back({obs, clearance, forward_proj, h});
  }

  // 필터링 후 active obstacle이 없으면 QP를 만들 필요가 없음.
  // 이 경우도 nominal command를 그대로 통과시킴.
  if (active.empty()) {

    // if (dbg_print) {
    //   std::cerr << "[CBF dbg skip] no active obstacle" << std::endl;
    // }
    v_safe = v_nom;
    w_safe = w_nom;
    return true;
  }

  // 위험도가 큰 장애물을 앞에 오도록 clearance 오름차순 정렬함.
  // clearance가 작을수록 안전반경에 더 가깝거나 이미 침범한 상태이므로 우선순위가 높음.
  std::sort(active.begin(), active.end(),
    [](const ActiveObs & a, const ActiveObs & b) {
      return a.clearance < b.clearance;
    });

  // QP에 넣을 active obstacle 수를 제한함.
  // 장애물이 많으면 제약이 많아져 계산량이 늘고, 여러 장애물이 서로 반대 방향 회피를 요구해
  // 제어가 흔들릴 수 있음. 디버깅/안정화 단계에서는 1개만 쓰는 것도 자주 사용함.
  if (params_.max_active_obstacles > 0 &&
      static_cast<int>(active.size()) > params_.max_active_obstacles) {
    active.resize(params_.max_active_obstacles);
  }

  // 가장 위험한 active obstacle 기준으로 디버그 및 recovery 판단용 값을 계산함.
  // active.front()는 위에서 clearance가 가장 작은 장애물임.
  double dbg_av = 0.0, dbg_aw = 0.0;
  computeAb(robot_x, robot_y, robot_yaw, active.front().obs, dbg_av, dbg_aw);

  // CBF 조건은 h_dot >= -gamma*h 임.
  // h_dot = a_v*v + a_w*omega 이므로 nominal command가 제약을 만족하는지
  // lhs_nom = a_v*v_nom + a_w*w_nom 와 rhs = -gamma*h 를 비교하면 됨.
  double dbg_h   = active.front().h;
  double dbg_rhs = -params_.gamma * dbg_h;
  double dbg_lhs_nom = dbg_av * v_nom + dbg_aw * w_nom;

  // QP 목적함수에서 추종할 "효과적인 nominal command"임.
  // 기본적으로는 MPC가 준 v_nom, w_nom을 그대로 사용함.
  // 단, recovery mode에서는 가까운 장애물을 빠져나오기 위해 nominal을 일부 바꿔
  // QP가 회전/감속을 더 쉽게 선택하도록 유도함.
  double v_nom_eff = v_nom;
  double w_nom_eff = w_nom;









// recovery mode 진입/해제 기준임.
// enter와 exit를 다르게 둔 것은 hysteresis 때문임.
// 하나의 임계값만 쓰면 clearance가 경계 근처에서 살짝 흔들릴 때 recovery가 켜졌다 꺼졌다 반복될 수 있음.
// enter는 더 작게, exit는 더 크게 잡아 상태 전환을 안정화함.
const double recovery_enter_clearance = 0.03;
const double recovery_exit_clearance  = 0.10;

// preferred_turn_dir는 현재 기하에서 h를 증가시키는 쪽의 회전 방향을 고르는 값임.
// dbg_aw는 omega가 h_dot에 미치는 계수임.
// dbg_aw가 양수면 양의 omega가 h를 증가시키는 방향이고, 음수면 음의 omega가 유리함.
const int preferred_turn_dir = (dbg_aw >= 0.0) ? 1 : -1;

// nominal 명령이 CBF 제약을 실제로 위반하는지 확인함
const double cbf_margin = 0.02;
const bool nominal_violates_cbf = (dbg_lhs_nom < dbg_rhs + cbf_margin);

// recovery 중이라도 더 이상 강제 회피가 필요 없으면 해제함.
// 해제 조건:
//   1) clearance가 충분히 커짐
//   2) nominal command만으로도 CBF 제약을 만족함
//   3) 장애물이 lookahead 기준 뒤쪽으로 충분히 넘어감
// turn_dir_hold_를 0으로 초기화해야 다음 장애물에서 새 회피 방향을 다시 선택할 수 있음.
if (recovery_active_ &&
    (active.front().clearance > recovery_exit_clearance ||
     !nominal_violates_cbf ||
     active.front().forward_proj < -0.10)) {
  recovery_active_ = false;
  turn_dir_hold_ = 0;
}

// 충분히 멀어졌거나, nominal 입력만으로 안전하거나,
// 장애물이 이미 뒤쪽으로 넘어가면 recovery 해제함
if (recovery_active_ &&
    (active.front().clearance > recovery_exit_clearance ||
     !nominal_violates_cbf ||
     active.front().forward_proj < -0.05)) {
  recovery_active_ = false;
  turn_dir_hold_ = 0;
}

// 최종적으로 이번 QP에서 recovery 목적함수 보정을 쓸지 결정함.
const bool recovery_mode = recovery_active_;





// 목적함수 가중치의 effective 값임.
// 기본값은 params_에서 온 가중치를 그대로 사용함.
// recovery mode에서는 q_w_dev_eff를 낮춰 omega가 nominal에서 더 쉽게 벗어나도록 함.
double q_v_dev_eff = params_.q_v_dev;
double q_w_dev_eff = params_.q_w_dev;

if (recovery_mode) {
  // 가까운 장애물에서는 전진 줄이고 회전 우선시함.
  // 여기서 실제 제약을 바꾸는 것이 아니라, QP 목적함수의 nominal target을 바꾸는 것임.
  // 즉 "이 상황에서는 원래 v_nom보다 낮은 속도를 더 선호하라"고 QP에 유도하는 방식임.
  v_nom_eff = std::min(std::max(v_nom, 0.02), 0.05);

  // 회피 방향 latch임.
  // 매 주기마다 preferred_turn_dir를 새로 쓰면 장애물 위치/노이즈 때문에 좌우 방향이 계속 바뀔 수 있음.
  // 한 번 방향을 정하면 recovery가 끝날 때까지 유지해서 와리가리 현상을 줄임.
  if (turn_dir_hold_ == 0) {
    turn_dir_hold_ = preferred_turn_dir;
  }

  // 최대 회전 대신 약간 낮춘 회전 명령 사용함.
  // w_max를 그대로 쓰면 너무 공격적으로 회전해서 경로 복귀가 어려울 수 있음.
  // 0.45rad/s는 "강제 탈출"과 "부드러운 회피" 사이의 절충값으로 사용한 것임.
  const double w_escape_mag = 0.45;
  double w_escape = static_cast<double>(turn_dir_hold_) * w_escape_mag;

  // 각속도 제한 안에 넣음
  w_escape = std::max(params_.w_min, std::min(params_.w_max, w_escape));
  w_nom_eff = w_escape;

  // omega 변경을 덜 싫어하게 해서 회전이 잘 나오게 함.
  // q_w_dev는 (omega - w_nom_eff)^2에 걸리는 비용임.
  // 값을 낮추면 QP가 nominal omega에서 벗어나는 것을 덜 부담스러워함.
  q_w_dev_eff = 0.08 * params_.q_w_dev;
}





  // QP 크기 정의임.
  // 변수 x = [v, omega, delta_0, ..., delta_{N-1}] 이므로 변수 개수는 N+2개임.
  // 제약은 다음과 같음:
  //   - CBF 제약 N개
  //   - slack delta_i >= 0 제약 N개
  //   - v bound 1개
  //   - omega bound 1개
  // 따라서 총 2N+2개임.
  int N     = static_cast<int>(active.size());
  int n_var = N + 2;
  int n_con = 2 * N + 2;

  // P는 대각 행렬이므로 non-zero 개수는 변수 개수와 같음.
  // A는 CSC column 기준으로 직접 구성함:
  //   v column: CBF N개 + v bound 1개 = N+1
  //   omega column: CBF N개 + omega bound 1개 = N+1
  //   delta columns: 각 delta마다 CBF 1개 + slack lower bound 1개 = 2N
  //   총 4N+2개임.
  int P_nnz = n_var;
  int A_nnz = 4 * N + 2;

    std::cout
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

  // OSQP에서 무한대 upper bound처럼 쓰기 위한 큰 수임.
  // C 라이브러리 내부 타입(c_float)에 맞춰 캐스팅함.
  constexpr c_float INF = static_cast<c_float>(1e30);

  // ── P 행렬 ────────────────────────────────────────────────
  // OSQP QP 표준형은 다음과 같음:
  //   minimize 0.5*x^T*P*x + q^T*x
  //   subject to l <= A*x <= u
  // 여기서 P는 목적함수의 2차항 행렬임.
  // 본 문제는 (v-v_nom)^2, (omega-w_nom)^2, delta_i^2만 있으므로 P는 대각 행렬임.
  // CSC(Compressed Sparse Column) 포맷으로 직접 구성함.
  c_int   * P_p = (c_int  *)malloc(sizeof(c_int)   * (n_var + 1));
  c_int   * P_i = (c_int  *)malloc(sizeof(c_int)   * P_nnz);
  c_float * P_x = (c_float*)malloc(sizeof(c_float) * P_nnz);

  for (int k = 0; k < n_var; ++k) {
    P_p[k] = k;
    P_i[k] = k;

    if (k == 0) {
      // v에 대한 quadratic weight임.
      // OSQP는 0.5*x^T*P*x 형식이므로, q*(v-v_ref)^2를 만들려면 P에는 2q가 들어감.
      P_x[k] = static_cast<c_float>(2.0 * params_.q_v_dev);
    } else if (k == 1) {
      // omega에 대한 quadratic weight임. recovery 모드에서는 q_w_dev_eff가 작아질 수 있음.
      P_x[k] = static_cast<c_float>(2.0 * q_w_dev_eff);
    } else {

      
      // delta_i는 CBF 제약을 부드럽게 완화하는 slack 변수임.
      // slack_p가 크면 제약 위반을 강하게 싫어하고, 작으면 nominal command 유지 쪽으로 타협하기 쉬움.
      // 안전 필터에서는 너무 작은 slack penalty를 쓰면 CBF가 사실상 무력해질 수 있음.
      const double slack_p_eff = params_.slack_p;
      P_x[k] = static_cast<c_float>(2.0 * slack_p_eff);

    }
  }

  P_p[n_var] = P_nnz;

  csc * P_mat = csc_matrix(n_var, n_var, P_nnz, P_x, P_i, P_p);

  // ── q 벡터 ────────────────────────────────────────────────
  // q는 목적함수의 1차항임.
  // q_v*(v-v_ref)^2 = q_v*v^2 - 2*q_v*v_ref*v + constant 이므로
  // OSQP의 q_vec에는 -2*q_v*v_ref가 들어감.
  // constant 항은 최적해에 영향을 주지 않으므로 넣지 않음.
  c_float * q_vec = (c_float*)malloc(sizeof(c_float) * n_var);

  q_vec[0] = static_cast<c_float>(-2.0 * q_v_dev_eff * v_nom_eff);
  q_vec[1] = static_cast<c_float>(-2.0 * q_w_dev_eff * w_nom_eff);

  for (int k = 2; k < n_var; ++k) {
    q_vec[k] = 0.0f;
  }

  // ── A 행렬 ────────────────────────────────────────────────
  // A는 모든 선형 제약의 좌변 계수 행렬임.
  // 제약 형태는 OSQP 표준형 l <= A*x <= u 로 들어감.
  // 본 구현에서 행(row) 배치는 다음 순서임:
  //   row 0 ~ N-1       : CBF 제약 a_v*v + a_w*omega + delta_i >= -gamma*h_i
  //   row N ~ 2N-1      : slack lower bound delta_i >= 0
  //   row 2N            : v lower/upper bound
  //   row 2N+1          : omega lower/upper bound
  // 열(column)은 변수 순서 [v, omega, delta_0, ...]에 맞춰 구성함.
  c_int   * A_p = (c_int  *)malloc(sizeof(c_int)   * (n_var + 1));
  c_int   * A_i = (c_int  *)malloc(sizeof(c_int)   * A_nnz);
  c_float * A_x = (c_float*)malloc(sizeof(c_float) * A_nnz);

  int nz = 0;

  // col 0: v
  // v는 모든 CBF 제약에 a_v 계수로 들어가고, v bound 행에도 1.0으로 들어감.
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
  // omega는 모든 CBF 제약에 a_w 계수로 들어가고, omega bound 행에도 1.0으로 들어감.
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
  // 각 slack 변수 delta_i는 자기 CBF 제약 row와 자기 nonnegative 제약 row에만 들어감.
  // delta_i를 CBF row에 +1로 넣으면 제약을 완화할 수 있음:
  //   a_v*v + a_w*omega + delta_i >= -gamma*h_i
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
  // OSQP는 모든 제약을 l <= A*x <= u 형태로 받음.
  // lower bound만 있는 제약은 u를 INF로 둠.
  // 양쪽 bound가 있는 v/omega 제약은 l과 u를 모두 설정함.
  c_float * l_vec = (c_float*)malloc(sizeof(c_float) * n_con);
  c_float * u_vec = (c_float*)malloc(sizeof(c_float) * n_con);

  // CBF 제약 row 설정임.
  // h_dot >= -gamma*h 조건을 사용함.
  // h가 작거나 음수이면 rhs가 커져서 더 강한 회피/감속을 요구함.
  // gamma가 클수록 boundary 근처 복구는 강해질 수 있지만, 안전영역 내부에서는 접근 허용이 느슨해질 수 있음.
  for (int i = 0; i < N; ++i) {
    double h_i = active[i].h;
    l_vec[i] = static_cast<c_float>(-params_.gamma * h_i);
    u_vec[i] = INF;
  }

  // slack 변수는 음수가 되면 안 됨.
  // delta_i >= 0이어야 "제약 완화량"이라는 의미가 유지됨.
  for (int i = 0; i < N; ++i) {
    l_vec[N + i] = 0.0f;
    u_vec[N + i] = INF;
  }

// 선속도 하한임.
// forbid_reverse가 true이면 후진을 금지하고 v>=0으로 둠.
// false이면 params_.v_min까지 허용함.
// 안전 필터 관점에서는 후진 허용 여부가 회피 행동에 큰 영향을 줄 수 있음.
const double v_lower = params_.forbid_reverse ? 0.0 : params_.v_min;

// v, omega bound 제약임.
// 이 bound는 QP 해가 물리적/제어기 한계 밖으로 나가지 않게 하는 역할임.
l_vec[2 * N]     = static_cast<c_float>(v_lower);
u_vec[2 * N]     = static_cast<c_float>(params_.v_max);
l_vec[2 * N + 1] = static_cast<c_float>(params_.w_min);
u_vec[2 * N + 1] = static_cast<c_float>(params_.w_max);

  // ── OSQP data ──────────────────────────────────────────────
  // OSQPData는 문제 크기와 P, q, A, l, u 포인터를 묶는 구조체임.
  // 여기서 넘기는 P/A의 내부 배열은 malloc 기반으로 할당되어 있어야 cleanup 시 안전함.
  OSQPData data;
  data.n = n_var;
  data.m = n_con;
  data.P = P_mat;
  data.q = q_vec;
  data.A = A_mat;
  data.l = l_vec;
  data.u = u_vec;

  // workspace 생성 및 문제 setup 단계임.
  // setup 과정에서 OSQP는 P/A/q/l/u 정보를 바탕으로 내부 factorization 준비를 함.
  OSQPWorkspace * workspace = nullptr;
  c_int status = osqp_setup(&workspace, &data, &settings_);

  // setup 실패 시 안전하게 nominal command로 fallback함.
  // safety filter가 실패했다고 해서 노드가 죽으면 안 되므로, false를 반환하고 상위에서 처리하게 함.
  if (status != 0 || workspace == nullptr) {
    free(P_p); free(P_i); free(P_x); free(P_mat);
    free(A_p); free(A_i); free(A_x); free(A_mat);
    free(q_vec); free(l_vec); free(u_vec);

    v_safe = v_nom;
    w_safe = w_nom;
    return false;
  }

  // q/l/u는 setup 후 해제 가능함.
  // OSQP setup 이후에는 필요한 값이 내부 workspace에 복사되므로 원본 q/l/u 배열을 여기서 해제함.
  // 반면 P/A matrix는 workspace cleanup과 별도로 아래에서 직접 해제하는 구조임.
  free(q_vec);
  free(l_vec);
  free(u_vec);

  // ── solve ─────────────────────────────────────────────────
  // 실제 QP 풀이 단계임.
  // 이 결과로 solution->x[0]에 v_safe, solution->x[1]에 w_safe가 들어감.
  osqp_solve(workspace);

  // std::cout
  // << "[CBF dbg solve] status=" << workspace->info->status_val
  // << std::endl;

  // OSQP 상태값을 보고 해를 사용할지 결정함.
  // SOLVED_INACCURATE는 약간 부정확하지만 실시간 제어에서 사용할 수 있다고 판단한 상태임.
  bool success = false;
  if (workspace->info->status_val == OSQP_SOLVED ||
      workspace->info->status_val == OSQP_SOLVED_INACCURATE) {
    // 최적해에서 앞 두 변수만 실제 제어입력으로 사용함.
    // 나머지 변수들은 slack delta_i라서 cmd_vel에는 직접 쓰지 않음.
    v_safe = static_cast<double>(workspace->solution->x[0]);
    w_safe = static_cast<double>(workspace->solution->x[1]);

    // 수치오차나 inaccurate solve로 인해 bound를 아주 조금 벗어날 수 있으므로 한 번 더 clamp함.
    // 이 clamp는 QP 제약과 동일한 한계를 코드 레벨에서 재확인하는 안전장치임.
    v_safe = std::clamp(v_safe, v_lower, params_.v_max);
    w_safe = std::clamp(w_safe, params_.w_min, params_.w_max);

    // if (params_.forbid_reverse) {
    //   v_safe = std::clamp(v_safe, 0.0, params_.v_max);
    // } else {
    //   v_safe = std::clamp(v_safe, params_.v_min, params_.v_max);
    // }

    // w_safe = std::clamp(w_safe, params_.w_min, params_.w_max);

    // 아주 작은 속도 명령은 0으로 잘라냄.
    // 시뮬레이터/실제 로봇에서 너무 작은 cmd_vel은 의미 있는 움직임을 만들지 못하면서
    // 노이즈처럼 보일 수 있음. 다만 v_eps가 너무 크면 장애물 근처에서 불필요하게 정지할 수 있음.
    if (std::abs(v_safe) < params_.v_eps) v_safe = 0.0;
    if (std::abs(w_safe) < 1e-3)          w_safe = 0.0;

    success = true;
  } else {
    // QP가 실패하거나 infeasible/unsolved 상태이면 nominal command를 그대로 사용함.
    // soft CBF slack을 넣었기 때문에 infeasible 가능성은 낮지만, 수치 문제는 항상 대비해야 함.
    v_safe = v_nom;
    w_safe = w_nom;
  }

  // ── cleanup ───────────────────────────────────────────────
  // 메모리 해제 순서가 중요함.
  // P_mat/A_mat는 csc_matrix가 포인터만 들고 있으므로 내부 p/i/x 배열을 먼저 해제하고,
  // 마지막에 matrix struct 자체를 해제함.
  // std::vector::data()를 넘겨놓고 여기서 free하면 double-free/invalid free가 날 수 있음.
  free(P_mat->p); free(P_mat->i); free(P_mat->x); free(P_mat);
  free(A_mat->p); free(A_mat->i); free(A_mat->x); free(A_mat);
  osqp_cleanup(workspace);

  // std::cout
  // << "[CBF dbg out] v_nom=" << v_nom
  // << " w_nom=" << w_nom
  // << " v_safe=" << v_safe
  // << " w_safe=" << w_safe
  // << std::endl;

  // success=true이면 QP 해를 사용했다는 뜻이고, false이면 nominal fallback이 수행됐다는 뜻임.
  return success;
}

}  // namespace control_mpc