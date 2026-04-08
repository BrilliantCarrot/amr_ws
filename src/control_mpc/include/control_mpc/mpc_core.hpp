#ifndef CONTROL_MPC__MPC_CORE_HPP_
#define CONTROL_MPC__MPC_CORE_HPP_

#include <Eigen/Dense>
#include <vector>

// OSQP가 하는 일 한 문장 요약
// "20스텝 동안 예측 상태(x_k)가 목표(x_ref[k])에서
// 최대한 가까워지는 입력 시퀀스 [u_0, ..., u_19]를 찾아라"

// OSQP C 헤더 — C++ 코드에서 C 함수 쓸 때 반드시 extern "C" 블록으로 감싸야 함
// OSQP가 순수 C언어로 작성된 라이브러리 이기 때문
// 안 감싸면 C++ name mangling 때문에 링커가 함수를 못 찾음
// v0.6.3 기준 타입:
//   희소 행렬 → csc  (cs.h에 정의됨, osqp.h가 include함)
//   실수형    → c_float
//   정수형    → c_int
extern "C" {
#include "osqp.h"
}

namespace control_mpc
{

// ============================================================
// MPC 상태/입력 차원 상수
//
// 상태 x = [x, y, θ, v, ω]  (5차원)
//   x, y : map frame 기준 위치 [m]
//   θ    : heading [rad]
//   v    : 선속도 [m/s]
//   ω    : 각속도 [rad/s]
//
// 입력 u = [Δv, Δω]          (2차원)
//   Δv   : 선속도 변화량 [m/s per step]
//   Δω   : 각속도 변화량 [rad/s per step]
//
// 왜 v,ω를 상태에, Δv,Δω를 입력에 넣나?
//   입력을 변화량(Δ)으로 설계하면 비용함수의 R항이
//   자동으로 "입력 급변 페널티" 역할을 함 → smooth 제어
//   만약 v,ω를 직접 입력으로 쓰면 별도 smoothness 항 필요
// ============================================================
constexpr int NX = 5;   // 상태 차원
constexpr int NU = 2;   // 입력 차원

// Eigen 타입 별칭 — 코드 가독성용
using StateVec      = Eigen::Matrix<double, NX, 1>;       // 5×1 상태 벡터
using InputVec      = Eigen::Matrix<double, NU, 1>;       // 2×1 입력 벡터
using StateMat      = Eigen::Matrix<double, NX, NX>;      // 5×5 상태 행렬
using InputMat      = Eigen::Matrix<double, NX, NU>;      // 5×2 입력 행렬
using CostStateMat  = Eigen::Matrix<double, NX, NX>;      // Q 가중치 행렬
using CostInputMat  = Eigen::Matrix<double, NU, NU>;      // R 가중치 행렬

// ============================================================
// 정적 장애물 정보 구조체
// ============================================================
struct Obstacle {
  double x;       // 장애물 중심 x [m] (map frame)
  double y;       // 장애물 중심 y [m] (map frame)
  double radius;  // 장애물 반경 [m] (크기의 절반 + 로봇 반경)
  double vx = 0.0;
  double vy = 0.0;
};

// ============================================================
// MPC 파라미터 구조체
// ============================================================
struct MpcParams
{
  int    N   = 20;       // prediction horizon 스텝 수
  double dt  = 0.02;     // 제어 주기 [s] (50Hz)

  // Q: 추적 오차 페널티 가중치 (단위 없음 — 오차²에 곱하는 가중치)
  double q_x   = 10.0;   // x 위치 오차 가중치  [1/m²]  → 오차 1m당 10 페널티
  double q_y   = 10.0;   // y 위치 오차 가중치  [1/m²]
  double q_th  = 5.0;    // heading 오차 가중치 [1/rad²] → 오차 1rad당 5 페널티
  double q_v   = 1.0;    // 선속도 오차 가중치  [1/(m/s)²]
  double q_w   = 1.0;    // 각속도 오차 가중치  [1/(rad/s)²]

  // R: 입력 페널티 (클수록 smooth, 단위 없음 — 입력²에 곱하는 가중치)
  double r_dv  = 30.0;   // Δv 페널티  [1/(m/s)²]   → 속도 변화 1m/s당 10 페널티
  double r_dw  = 2.0;    // Δω 페널티  [1/(rad/s)²] → 각속도 변화 1rad/s당 5 페널티

  // 상태 제약
  double v_max  =  0.5;  // 최대 선속도   [m/s]
  double v_min  = -0.5;  // 최소 선속도   [m/s]  (후진 허용)
  double w_max  =  1.0;  // 최대 각속도   [rad/s]  ≈ 57.3°/s
  double w_min  = -1.0;  // 최소 각속도   [rad/s]

  // 입력 제약
  double dv_max =  0.1;  // 최대 선속도 변화량  [m/s per step]  → 가속도 최대 5 m/s²
  double dv_min = -0.1;  // 최소 선속도 변화량  [m/s per step]
  double dw_max =  0.2;  // 최대 각속도 변화량  [rad/s per step] → 각가속도 최대 10 rad/s²
  double dw_min = -0.2;  // 최소 각속도 변화량  [rad/s per step]

  // W8: 장애물 soft penalty 파라미터
  double obs_weight    = 50.0;  // 장애물 penalty 가중치 (클수록 강하게 회피)
  double obs_safe_dist =  0.45;   // penalty 시작 거리 [m] (이 이내로 들어오면 penalty 발생)
};

// ============================================================
// MPC 솔루션 구조체
// solve() 한 번 호출의 결과물을 담는 컨테이너.
// mpc_node가 이걸 받아서 /cmd_vel 발행에 활용함.
// ============================================================
struct MpcSolution
{
  InputVec u0;
  // 이번 스텝에서 실제로 로봇에 적용할 최적 입력
  // u0 = [Δv, Δω]
  //   u0(0) = Δv : 선속도 변화량 [m/s]
  //   u0(1) = Δω : 각속도 변화량 [rad/s]
  // N개의 최적 입력 시퀀스 중 첫 번째만 사용 (Receding Horizon 원리)
  // 나머지 u_1~u_{N-1}은 버리고 다음 스텝에서 다시 풀어서 새로 구함
 
  bool success;
  // OSQP 솔버가 정상적으로 수렴했는지 여부
  // true  : 최적해 찾음 → u0 신뢰 가능, /cmd_vel 발행해도 됨
  // false : 수렴 실패 (infeasible, 제약 충돌 등) → u0 사용 금지, 정지 명령 발행
  // mpc_node의 controlCallback()에서 이 값을 먼저 확인 후 처리함
 
  double solve_time_ms;
  // QP 풀기에 걸린 시간 [밀리초]
  // KPI 목표: 평균 < 20ms, 99th percentile < 50ms
  // W6 실측: 평균 ~2.0ms, 최대 3.3ms → 기준 대비 10배 여유
  // /metrics/control_latency_ms 토픽으로 발행해서 실시간 모니터링
 
  double cost;
  // OSQP가 찾은 최적해의 목적함수 값
  // cost = Σ(x_k-x_ref_k)ᵀQ(x_k-x_ref_k) + Σu_kᵀRu_k
  // 값이 작을수록 reference를 잘 따라가면서 입력도 작다는 의미
  // 음수가 나올 수 있음 (q_vec = -Q*x_ref 선형항 때문)
  // 직접 제어에 쓰진 않고 튜닝/디버깅 모니터링 용도
};

// ============================================================
// MpcCore — 순수 수학/최적화 클래스 (ROS2 의존성 없음)
//
// [QP 변수 벡터 z 구조]
//   z = [x_0, x_1, ..., x_N, u_0, u_1, ..., u_{N-1}]
//
// [QP 형태]
//   min  ½ zᵀ P z + qᵀ z
//   s.t. l ≤ A z ≤ u

// z = QP가 찾아야 할 미지수 벡터
// z = [x_0, x_1, ..., x_20,  u_0, u_1, ..., u_19]
//      ↑ 예측 상태 시퀀스        ↑ 최적 입력 시퀀스
//      (5×21 = 105개)            (2×20 = 40개)
//      총 145개
// P = 비용 Hessian (2차 항 가중치)
// "2차": z를 제곱하는 항, z^2처럼 생김
// ½ · p · z²   ← 이게 "2차 항"
//                z가 0에서 멀어질수록 비용이 제곱으로 커짐
//                p가 클수록 비용이 더 빠르게 커짐 (더 엄격하게 제한)
// P = blkdiag(0, Q, Q, ..., Q,  R, R, ..., R)
//           ↑x_0  ↑x_1~x_20    ↑u_0~u_19
// "x가 reference에서 얼마나 벗어났나(Q)" + "입력이 얼마나 큰가(R)"
// P가 블록 대각 구조로 생김:

// P = 블록 대각(0,  Q,  Q, ...,  Q,   R,  R, ...,  R)
//          ↑x_0  ↑x_1  ↑x_2    ↑x_20  ↑u_0  ↑u_1    ↑u_19
// x_0 자리가 0인 이유: x_0는 등식 제약으로 이미 현재 상태로 고정. 비용을 매길 필요가 없음.

// Q 블록이 하는 일: x_k가 0에서 멀어질수록 비용이 커짐. 근데 여기서 원하는 건 
// "x_k가 0에서 멀어지는 것"이 아니라 "x_k가 x_ref에서 멀어지는 것"에 페널티를 주는 것, 그래서 q 벡터가 필요.
// R 블록이 하는 일: u_k = [Δv, Δω]가 클수록 비용이 커짐. 
// Δv, Δω가 크다는 건 속도를 급격히 바꾼다는 뜻 → smooth 제어 강제.

// x_ref: k 스텝 후에 로봇이 있어야 할 목표 상태
// generateReference() 함수가 waypoints에서 뽑아줌.

// x_k - x_ref_k 가 뭔가
// "k스텝 후 예측 상태가 목표에서 얼마나 벗어났나".
// 구체적인 숫자로 보자. k=1일 때:
// x_ref[1] = [1.1, 0.0, 0.0, 0.3, 0.0]   ← 목표 (waypoint)
// x_1      = [1.08, 0.02, 0.01, 0.28, 0.0] ← OSQP가 예측한 상태
// x_1 - x_ref[1] = [1.08-1.1,  0.02-0.0,  0.01-0.0,  0.28-0.3,  0.0-0.0]
//                = [-0.02,      +0.02,      +0.01,     -0.02,      0.0]
//                   ↑x오차      ↑y오차      ↑θ오차     ↑v오차
// 이 오차 벡터에 Q를 곱해서 비용을 계산:
// (x_1 - x_ref[1])ᵀ · Q · (x_1 - x_ref[1])
// Q = diag(10, 10, 5, 1, 1)  ← 위치 오차를 더 엄격하게 페널티
// = 10·(-0.02)² + 10·(0.02)² + 5·(0.01)² + 1·(-0.02)² + 1·(0.0)²
// = 10·0.0004  + 10·0.0004  + 5·0.0001  + 1·0.0004  + 0
// = 0.004 + 0.004 + 0.0005 + 0.0004
// = 0.009

// q = 선형 비용 벡터
// q의 상태 블록: -Q · x_ref[k]   (reference를 따라가게 유도)
// q의 입력 블록: 0
// "선형" 이라는 말 — z를 1제곱하는 항. z처럼 생긴 항.

// qᵀ z = q₀·z₀ + q₁·z₁ + q₂·z₂ + ...   ← 각 변수에 상수를 곱해서 더함
// P만 있으면 뭐가 문제냐:

// ½ zᵀ P z 만 있으면 → z=0 일 때 비용이 최소
// 즉 솔버가 x_k = 0, u_k = 0 을 향해 당김
// 우리가 원하는 건 x_k → x_ref[k] 인데 방향이 틀림
// q가 하는 일 — "당기는 방향을 바꿔줌":

// tracking error (x_k - x_ref_k)ᵀ Q (x_k - x_ref_k) 를 전개하면:

// (x_k - x_ref_k)ᵀ Q (x_k - x_ref_k)
// = x_kᵀ Q x_k  -  2·x_ref_kᵀ Q x_k  +  x_ref_kᵀ Q x_ref_k
//    ↑ 2차항        ↑ 선형항              ↑ 상수 (최적화에 영향 없음)
//    → P 블록       → q 블록
// 선형항 -2·x_ref_kᵀ Q x_k 에서 OSQP는 ½ zᵀPz + qᵀz 형태를 쓰니까:

// qᵀz = -x_ref_kᵀ Q x_k
// q_k  = -Q · x_ref_k

// MATLAB 3가지 vs 프로젝트 상 OSQP
// MATLAB	우리 코드 l ≤ A·z ≤ u	사용처
// Ax ≤ b (inequality)	l = -∞, u = b	—
// Aeq·x = beq (equality)	l = u = beq	① 초기 상태, ② 동역학
// lb ≤ x ≤ ub (bound)	l = lb, u = ub	③ 상태 범위, ④ 입력 범위
// 순수 단방향 부등식 Ax ≤ b (l=-∞)는 없음.
// 제약은 전부 등식(l=u) 아니면 양방향 범위(lb ≤ · ≤ ub)
// ① l = u = x_current     → equality   (Aeq 타입)
// ② l = u = -c_k          → equality   (Aeq 타입)
// ③ -0.5 ≤ v ≤ +0.5       → bound      (lb/ub 타입)
// ④ -0.1 ≤ Δv ≤ +0.1      → bound      (lb/ub 타입)

// ============================================================
class MpcCore
{
public:
  MpcCore();
  ~MpcCore();

  void init(const MpcParams & params);

  MpcSolution solve(
  const StateVec & x0,
  // 현재 로봇 상태 [x, y, θ, v, ω]
  // /map_ekf/odom에서 추출한 map frame 기준 실제 위치
  // QP의 초기 상태 등식 제약 (x_0 = x_current)에 사용됨
  // ⚠️ /ekf/odom(odom frame) 아닌 /map_ekf/odom(map frame) 사용할 것
 
  const std::vector<StateVec> & x_ref,
  // N+1개의 목표 상태 시퀀스 [x_ref[0], ..., x_ref[N]]
  // x_ref[k]: k스텝 후의 목표 상태 [x, y, θ, v, ω]
  // waypoints에서 현재 위치 기준으로 generateReference()가 뽑아줌
  // QP 비용함수의 tracking error 항에서 기준값으로 사용됨
  // 선형화 기준점(xbar)으로도 사용됨
 
  const InputVec & u_prev);
  // 직전 스텝에서 실제로 적용한 입력 [v_prev, ω_prev]
  // ⚠️ [Δv, Δω]가 아니라 실제 속도값 [v, ω]임
  // 현재는 입력 연속성 제약 확장 시 사용 예정 (buildQP에서 u_prev 활용)
  // W7 튜닝 단계에서 이전 입력 기반 warm start 개선에 활용 가능
 
  const MpcParams & getParams() const { return params_; }
  // 현재 MPC 파라미터 읽기 (외부에서 N, dt, Q, R 등 확인용)
  // const 반환 → 외부에서 파라미터 수정 불가, 읽기 전용

  // W8: 장애물 목록 설정 (mpc_node에서 호출)
  void setObstacles(const std::vector<Obstacle> &obstacles);

private:
  // 비선형 운동 모델 f(x, u)
  StateVec motionModel(const StateVec & x, const InputVec & u) const;
  // 비선형 운동 방정식 f(x, u) — 다음 상태 예측
  // x_{k+1} = f(x_k, u_k) 계산
  //   x_next(0) = x + v·cos(θ)·dt   (world frame x 이동)
  //   x_next(1) = y + v·sin(θ)·dt   (world frame y 이동)
  //   x_next(2) = θ + ω·dt           (heading 회전)
  //   x_next(3) = v + Δv             (속도 갱신)
  //   x_next(4) = ω + Δω             (각속도 갱신)
  // 선형화의 nominal point c_k 계산에서 f(xbar, ubar) 로 호출됨
  // c_k = f(xbar, ubar) - A_k*xbar - B_k*ubar

  // 야코비안 선형화 → A_k (5×5), B_k (5×2)
  void linearize(
    const StateVec & xbar,
    // 선형화 기준점 — 이 상태 근처에서 1차 테일러 전개
    // x_ref[k]를 기준점으로 사용 (현재 waypoint)
    const InputVec & ubar,
    // 선형화 기준 입력 — 현재는 u_nom = [0, 0] (속도 유지 가정)
    // B_k는 xbar, ubar에 무관한 상수 행렬이라 실질적으로 미사용
 
    StateMat & Ak,
    // 출력: 상태 전이 야코비안 A_k (5×5)
    // A_k = ∂f/∂x|_{xbar}
    // 비선형 항만 채워짐:
    //   Ak(0,2) = -v·sin(θ)·dt   (∂x/∂θ)
    //   Ak(0,3) =  cos(θ)·dt     (∂x/∂v)
    //   Ak(1,2) =  v·cos(θ)·dt   (∂y/∂θ)
    //   Ak(1,3) =  sin(θ)·dt     (∂y/∂v)
    //   Ak(2,4) =  dt             (∂θ/∂ω)
    //   나머지: setIdentity()로 단위행렬 초기화
  
    InputMat & Bk) const;
    // 출력: 입력 야코비안 B_k (5×2)
    // B_k = ∂f/∂u|_{xbar}
    // u = [Δv, Δω] → v, ω에 선형으로 더해지므로 상수 행렬
    //   Bk(3,0) = 1   (Δv → v)
    //   Bk(4,1) = 1   (Δω → ω)
    //   나머지: 0

  // QP 행렬 P, q, A, l, u 조립
  void buildQP(
    const StateVec & x0,
    // 초기 상태 등식 제약에 사용 (x_0 = x_current)
  
    const std::vector<StateVec> & x_ref,
    // 비용함수의 reference — q_vec = -Q*x_ref[k]
  
    const InputVec & u_prev,
    // 미래 확장용 (현재 buildQP 내부에서 미사용)
  
    const std::vector<StateMat> & Ak_seq,
    // 크기 N — 각 스텝의 선형화된 상태 전이 행렬 A_k 시퀀스
    // 동역학 등식 제약 블록에 삽입됨
  
    const std::vector<InputMat> & Bk_seq,
    // 크기 N — 각 스텝의 입력 야코비안 B_k 시퀀스
    // 동역학 등식 제약 블록에 삽입됨
  
    const std::vector<StateVec> & ck_seq);
    // 크기 N — 각 스텝의 선형화 bias 보정 항 c_k 시퀀스
    // c_k = f(xbar,ubar) - A_k*xbar - B_k*ubar
    // 동역학 등식의 우변: l = u = -c_k

  // Eigen 밀집 행렬 → OSQP CSC 희소 행렬 변환
  // v0.6.3 API: csc 타입 (c_malloc/c_free 사용)
  csc * eigenToCsc(const Eigen::MatrixXd & M);
  // Eigen 밀집 행렬(dense) → OSQP CSC 희소 행렬(sparse) 변환
  // OSQP v0.6.3은 CSC(Compressed Sparse Column) 형식만 받음
  // 0인 원소는 저장 안 함 → 메모리/연산 효율
  // P 행렬은 상삼각(upper triangle)만 전달 (OSQP 규약)
  // 반환된 csc*는 freeCsc()로 반드시 해제해야 메모리 누수 없음
  void  freeCsc(csc * M);
  // eigenToCsc()로 할당한 CSC 행렬 메모리 해제
  // c_malloc으로 할당했으므로 반드시 c_free로 해제 (delete 금지)
  // solve() 호출마다 P_csc, A_csc 생성 후 즉시 해제

  // --- OSQP 멤버 ---
  OSQPWorkspace * work_ = nullptr;
  // OSQP 솔버 내부 상태 전체를 담는 구조체
  // osqp_setup()으로 생성, osqp_cleanup()으로 해제
  // 현재 구현: 매 solve()마다 재생성 (workspace 재활용 미적용)
  // warm_start=true 설정으로 이전 해를 초기값으로 활용
  // nullptr 초기화: 소멸자에서 이중 해제 방지용
  
  OSQPSettings * settings_ = nullptr;
  // OSQP 솔버 동작 옵션 모음
  // 주요 설정값:
  //   verbose    = false  : 매 solve 출력 끄기 (터미널 도배 방지)
  //   warm_start = true   : 이전 해 초기값 재활용 (수렴 빠름)
  //   max_iter   = 200    : 최대 반복 수 (20ms 제한 내에 맞추기 위함)
  //   eps_abs    = 1e-4   : 절대 수렴 허용 오차
  //   eps_rel    = 1e-4   : 상대 수렴 허용 오차
  //   polish     = false  : polishing 끄기 (정확도보다 속도 우선)
  // c_malloc으로 할당했으므로 소멸자에서 c_free로 해제

  // QP 행렬 (dense)
  Eigen::MatrixXd P_mat_;
  // QP 비용 Hessian 행렬 (nz × nz)
  // min ½ zᵀPz + qᵀz 에서의 P
  // 블록 대각 구조:
  //   상태 블록 (k=1~N): Q (5×5)  ← tracking error 가중치
  //   입력 블록 (k=0~N-1): R (2×2) ← input effort 가중치
  //   x_0 블록: 0 (초기 상태는 등식으로 고정이라 비용 불필요)
  // OSQP에 넘길 때 상삼각만 사용
  
  Eigen::VectorXd q_vec_;
  // QP 선형 비용 벡터 (nz × 1)
  // min ½ zᵀPz + qᵀz 에서의 q
  // tracking error 전개 시 생기는 선형 항:
  //   상태 블록 (k=1~N): -Q * x_ref[k]
  //   입력 블록: 0 (R항은 2차만 있고 선형 항 없음)
  // q가 음수인 이유: reference를 빼면서 부호가 뒤집힘
  // → 이 때문에 cost 값이 음수로 나올 수 있음
  
  Eigen::MatrixXd A_mat_;
  // QP 제약 행렬 (nc × nz)
  // l ≤ A·z ≤ u 에서의 A
  // 4개 블록으로 구성 (행 순서):
  //   ① 초기 상태 등식 (NX행)
  //   ② 동역학 등식 (NX*N행): A_k, -I, B_k 블록
  //   ③ 상태 부등식 (NX*N행): v, ω 범위
  //   ④ 입력 부등식 (NU*N행): Δv, Δω 범위
  // 대부분 0이고 일부만 채워지는 희소 행렬 → eigenToCsc()로 변환
  
  Eigen::VectorXd l_vec_;
  // QP 제약 하한 벡터 (nc × 1)
  // l ≤ A·z ≤ u 에서의 l
  // 등식 제약: l = -c_k (동역학), l = x_current (초기 상태)
  // 부등식 하한: v_min, ω_min, Δv_min, Δω_min
  // x, y, θ 위치 제약 없음: -1e9 (무한대 대용)
  
  Eigen::VectorXd u_vec_;
  // QP 제약 상한 벡터 (nc × 1)
  // l ≤ A·z ≤ u 에서의 u
  // 등식 제약: u = -c_k (= l과 동일 → 등식)
  // 부등식 상한: v_max, ω_max, Δv_max, Δω_max
  // x, y, θ 위치 제약 없음: +1e9
  
  int nz_;
  // QP 최적화 변수 수 (z 벡터 길이)
  // nz = NX*(N+1) + NU*N
  //    = 5*21 + 2*20 = 105 + 40 = 145  (N=20 기준)
  // z = [x_0,...,x_N, u_0,...,u_{N-1}]
  //      ↑ 상태 시퀀스   ↑ 입력 시퀀스
  
  int nc_;
  // QP 제약 수 (A 행렬 행 수)
  // nc = NX + NX*N + NX*N + NU*N
  //    = 5 + 100 + 100 + 40 = 245  (N=20 기준)
  //      ↑①  ↑②    ↑③    ↑④

  MpcParams params_;
  // MPC 파라미터 전체 (init()에서 저장, 이후 변경 없음)
  // N, dt, Q 가중치, R 가중치, 상태/입력 제약값 포함
  
  CostStateMat Q_;
  // 추적 오차 가중치 행렬 (5×5 대각)
  // params_에서 구성:
  //   Q_(0,0) = q_x  = 10.0  (x 위치 오차)
  //   Q_(1,1) = q_y  = 10.0  (y 위치 오차)
  //   Q_(2,2) = q_th = 5.0   (heading 오차)
  //   Q_(3,3) = q_v  = 1.0   (속도 오차)
  //   Q_(4,4) = q_w  = 1.0   (각속도 오차)
  // 값이 클수록 해당 상태를 reference에 정확히 맞추려 함
  // P_mat_와 q_vec_ 구성에 사용됨
  
  CostInputMat R_;
  // 입력 페널티 행렬 (2×2 대각)
  // params_에서 구성:
  //   R_(0,0) = r_dv = 10.0  (Δv 페널티 → 선속도 변화 억제)
  //   R_(1,1) = r_dw = 5.0   (Δω 페널티 → 각속도 변화 억제)
  // 값이 클수록 입력 변화를 억제 → smooth 제어
  // R이 클수록 Q보다 smooth를 우선시, 작을수록 추종을 우선시
  // P_mat_ 구성에 사용됨
  
  bool is_initialized_ = false;
  // init() 호출 완료 여부 플래그
  // solve() 진입 시 첫 번째로 확인
  // false이면 solve() 즉시 실패 반환 (미초기화 상태에서 접근 방지)
  // init() 마지막 줄에서 true로 설정됨

  // W8: 장애물 목록 (map frame 기준)
  std::vector<Obstacle> obstacles_;
};

}  // namespace control_mpc

#endif  // CONTROL_MPC__MPC_CORE_HPP_