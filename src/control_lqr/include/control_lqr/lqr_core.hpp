#ifndef CONTROL_LQR__LQR_CORE_HPP_
#define CONTROL_LQR__LQR_CORE_HPP_

// ============================================================
// lqr_core.hpp — TVLQR(Time-Varying LQR) 순수 수학 코어
//
// [설계 철학]
//   MpcCore와 동일한 패턴:
//     LqrCore  : 순수 수학/알고리즘 (ROS 의존성 전혀 없음)
//     LqrNode  : ROS2 인터페이스 (토픽, 타이머, 파라미터)
//
// [TVLQR이란?]
//   Diff-Drive AMR은 본질적으로 비선형 시스템임.
//   상태 방정식: ẋ = f(x, u) (비선형)
//
//   LQR은 선형 시스템에만 직접 적용 가능:
//     ẋ = A·x + B·u  (선형)
//
//   TVLQR은 매 제어 주기마다 현재 동작점(x_bar, u_bar)에서
//   Taylor 1차 근사로 선형화하여 A, B를 업데이트함:
//     A_k = ∂f/∂x|_{x_bar, u_bar}
//     B_k = ∂f/∂u|_{x_bar, u_bar}
//
//   → 매 주기 갱신된 A_k, B_k로 Riccati 방정식을 풀어
//     게인 K를 업데이트함 (Time-Varying이라는 이름의 이유)
//
// [상태/입력 정의]
//   상태 x = [x_pos, y_pos, θ, v, ω]  (5×1)
//   입력 u = [Δv, Δω]                  (2×1)
//   오차 e = x - x_ref                 (5×1)
//   제어: u = -K·e (최적 피드백 게인)
//
// [이산시간 Riccati 방정식]
//   P_{k} = Q + A^T P_{k+1} A - A^T P_{k+1} B (R + B^T P_{k+1} B)^{-1} B^T P_{k+1} A
//   K = (R + B^T P B)^{-1} B^T P A
//   → 후방 재귀(Backward Recursion)로 수렴할 때까지 반복
//
// [MPC vs TVLQR 비교]
//   MPC   : horizon N=20, QP 풀기 (제약 처리 가능, 무거움)
//   TVLQR : horizon=1, Riccati 재귀 (제약 처리 불가, 가벼움)
//   → AMR 비교 실험: 제어 지연, RMSE, 진동, 수렴 속도 비교
// ============================================================

#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace control_lqr
{

// ============================================================
// 상태/입력 차원 상수
// MpcCore와 동일한 5D 상태, 2D 입력 사용
// ============================================================
constexpr int LQR_NX = 5;  // 상태 차원: [x, y, θ, v, ω]
constexpr int LQR_NU = 2;  // 입력 차원: [Δv, Δω]

// Eigen 타입 별칭
using LqrStateVec  = Eigen::Matrix<double, LQR_NX, 1>;         // 5×1
using LqrInputVec  = Eigen::Matrix<double, LQR_NU, 1>;         // 2×1
using LqrStateMat  = Eigen::Matrix<double, LQR_NX, LQR_NX>;   // 5×5
using LqrInputMat  = Eigen::Matrix<double, LQR_NX, LQR_NU>;   // 5×2
using LqrGainMat   = Eigen::Matrix<double, LQR_NU, LQR_NX>;   // 2×5 (K 행렬)
using LqrQMat      = Eigen::Matrix<double, LQR_NX, LQR_NX>;   // Q: 상태 비용 가중치
using LqrRMat      = Eigen::Matrix<double, LQR_NU, LQR_NU>;   // R: 입력 비용 가중치

// ============================================================
// LQR 파라미터 구조체
// ============================================================
struct LqrParams
{
  double dt  = 0.02;   // 제어 주기 [s] (50Hz, MPC와 동일)

  // Q: 상태 추적 오차 페널티 가중치 (대각 행렬)
  // MPC의 Q와 동일한 의미: 클수록 해당 상태 오차를 엄격하게 줄이려 함
  double q_x   = 10.0;  // x 위치 오차 가중치
  double q_y   = 10.0;  // y 위치 오차 가중치
  double q_th  =  5.0;  // heading 오차 가중치
  double q_v   =  1.0;  // 선속도 오차 가중치
  double q_w   =  1.0;  // 각속도 오차 가중치

  // R: 입력 페널티 가중치 (대각 행렬)
  // MPC의 R과 동일한 의미: 클수록 제어 입력 변화를 줄이려 함 (smooth)
  double r_dv  = 10.0;  // Δv 페널티
  double r_dw  =  5.0;  // Δω 페널티

  // 속도 제한 (MPC와 동일)
  double v_max  =  0.5;  // 최대 선속도 [m/s]
  double v_min  = -0.1;  // 최소 선속도 [m/s]
  double w_max  =  1.0;  // 최대 각속도 [rad/s]
  double w_min  = -1.0;  // 최소 각속도 [rad/s]

  // LQR에서 리카티 방정식을 푸는 목적은 최적의 비용 함수를 나타내는 행렬 P를 찾는 것
  // 현재 상태에서 미래를 무한히 내다보았을 때 오차Q와 입력 R의 합을 최소화하는 P를 구하기 위해 반복 연산을 수행

  // Riccati 반복 수렴 설정
  // 5×5 시스템에서 보통 50~200회로 수렴함
  int    riccati_iter = 500;   // 최대 반복 횟수
  // 리카티 방정식을 풀기 위한 반복 루프를 최대 몇 번까지 수행할 것인가를 결정하는 안정장치
  // 루프가 무한히 도는것을 방지하고, CPU 자원의 최대 점유 시간을 제한
  // 너무 낮게 설정 (예: 10): 행렬 P가 수렴하기 전에 계산이 끝납니다. 이 경우 얻어진 게인 K는 
  // 무한 구간(Infinite-horizon)이 아닌 짧은 미래(Finite-horizon)에 대한 최적해에 가까워져, 제어 성능이 떨어질 수 있습니다.
  // 너무 높게 설정 (예: 5000): 시스템 행렬(A, B)이 특이하거나 수렴이 매우 느린 상황에서 루프가 너무 길게 돌아 
  // 제어 주기(20ms)를 초과하는 Latency Spike가 발생할 수 있습니다.
  double riccati_tol  = 1e-3;  // 수렴 판정 허용 오차 (P 변화량 기준), 허용 오차 완화
  // 이전 단계의 P 행렬과 현재 단계의 P행렬 사이의 차이가 얼마 이하일 때 충분히 수렴했다고 판단할것인지를 정하는 정밀도 기준
  // 기능: P 행렬의 변화량이 이 오차보다 작아지면 반복 계산을 즉시 중단하고 결과를 반환하여 계산 시간을 절약합니다.
  // 다르게 설정했을 시 결과:너무 엄격하게 설정 (예: 1e-9): 부동 소수점 오차나 시스템 특성상 해당 정밀도에 도달하기 어려워 
  // 항상 riccati_iter 끝까지 루프를 돌게 됩니다. 이는 불필요한 연산 낭비를 초래합니다.
  // 너무 느슨하게 설정 (예: 0.5): P 행렬이 제대로 완성되지 않은 상태에서 계산이 조기 종료됩니다. 
  // 결과적으로 제어 게인 K가 매 주기마다 불연속적으로 튀게 되어, 로봇 바퀴가 미세하게 떨리는 진동(Oscillation) 현상이 발생할 수 있습니다.
};

// ============================================================
// LQR 솔루션 구조체
// compute() 한 번 호출의 결과물
// ============================================================
struct LqrSolution
{
  LqrInputVec u0;      // 이번 스텝 최적 입력 [Δv, Δω]
  LqrGainMat  K;       // 게인 행렬 (2×5)
  bool        success; // Riccati 수렴 여부
};

// ============================================================
// LqrCore 클래스
// ============================================================
class LqrCore
{
public:
  LqrCore() = default;

  // --------------------------------------------------------
  // 초기화: 파라미터를 받아 Q, R 행렬 구성
  // lqr_node 생성자에서 한 번만 호출
  // --------------------------------------------------------
  void init(const LqrParams & params);

  // --------------------------------------------------------
  // TVLQR 메인 계산 함수
  //
  //   매 제어 주기(50Hz)마다 lqr_node가 호출함.
  //
  //   [처리 순서]
  //   1. 현재 동작점(x0, x_ref)에서 연속시간 선형화 → A_c, B_c
  //   2. 이산화 (Euler 1차): A_d, B_d
  //   3. Backward Riccati 반복 → P 수렴
  //   4. 게인 계산: K = (R + B^T P B)^{-1} B^T P A
  //   5. 오차 계산: e = x0 - x_ref (yaw 정규화 포함)
  //   6. 최적 입력: Δu = -K·e
  //   7. 클리핑 후 반환
  //
  //   x0   : 현재 로봇 상태 [x, y, θ, v, ω]
  //   x_ref: 현재 스텝 목표 상태 [x, y, θ, v, ω]
  //   u_prev: 직전 스텝 입력 [v_prev, ω_prev] (Δu → u 변환에 사용)
  // --------------------------------------------------------
  LqrSolution compute(
    const LqrStateVec & x0,
    const LqrStateVec & x_ref,
    const LqrInputVec & u_prev);
    
  // --------------------------------------------------------
  // Q, R 행렬 반환 (디버깅/로깅용)
  // --------------------------------------------------------
  const LqrQMat & getQ() const { return Q_; }
  const LqrRMat & getR() const { return R_; }

private:
  // --------------------------------------------------------
  // 연속시간 Jacobian 선형화
  //
  //   Diff-Drive 운동 방정식:
  //     ẋ_pos = v·cos(θ)
  //     ẏ_pos = v·sin(θ)
  //     θ̇     = ω
  //     v̇     = Δv / dt   (입력을 속도 변화량으로 정의)
  //     ω̇     = Δω / dt
  //
  //   상태 x = [x_pos, y_pos, θ, v, ω] 기준 Jacobian:
  //
  //   A_c = ∂f/∂x = [0, 0, -v·sin(θ), cos(θ), 0  ]  ← x_pos 행
  //                  [0, 0,  v·cos(θ), sin(θ), 0  ]  ← y_pos 행
  //                  [0, 0,  0,        0,       1  ]  ← θ    행
  //                  [0, 0,  0,        0,       0  ]  ← v    행
  //                  [0, 0,  0,        0,       0  ]  ← ω    행
  //
  //   B_c = ∂f/∂u = [0,    0   ]  ← x_pos 행
  //                  [0,    0   ]  ← y_pos 행
  //                  [0,    0   ]  ← θ    행
  //                  [1/dt, 0   ]  ← v    행 (Δv → v̇)
  //                  [0,    1/dt]  ← ω    행 (Δω → ω̇)
  //
  //   x_bar : 선형화 동작점 상태 (보통 x_ref 또는 x0)
  // --------------------------------------------------------
  void linearize(
    const LqrStateVec & x_bar,
    LqrStateMat & A_c,
    LqrInputMat & B_c) const;

  // --------------------------------------------------------
  // 이산화 (Euler 1차 근사)
  //
  //   A_d = I + A_c * dt
  //   B_d =     B_c * dt
  //
  //   [왜 Euler?]
  //   정밀도: ZOH(Matrix Exponential) > Euler
  //   속도  : Euler 훨씬 빠름
  //   50Hz(dt=0.02s), 저속(v<0.5m/s)에서 Euler 오차는 무시 가능 수준
  //   TVLQR은 매 주기 선형화를 다시 하므로 이산화 오차가 누적되지 않음
  // --------------------------------------------------------
  void discretize(
    const LqrStateMat & A_c,
    const LqrInputMat & B_c,
    LqrStateMat & A_d,
    LqrInputMat & B_d) const;

  // --------------------------------------------------------
  // DARE 직접 풀이 (Hamiltonian 행렬 + 고유벡터 분해)
  //
  //   Backward iteration은 A_d 고유값이 unit circle 위에 있을 때
  //   수렴 속도가 O(1/n)으로 극히 느림.
  //   Diff-Drive의 A_d는 고유값이 모두 1.0 → 수천 회도 부족.
  //
  //   대신 10×10 Hamiltonian 행렬을 구성하고
  //   안정 불변 부분공간의 고유벡터로 P를 직접 계산함.
  //   연산량: O(n³) 1회, 반복 없음 → 항상 수렴, 빠름.
  //
  //   H = [A,          -B R⁻¹ B^T ]
  //       [-Q A^{-T},   A^T        ]
  //   의 고유벡터 중 unit circle 내부 고유값에 해당하는 것을 추출.
  //   P = X₂ * X₁⁻¹  (X₁, X₂: 안정 고유벡터 상하 블록)
  //
  //   내부 상태(P_prev_)를 갱신해야 하므로 const 키워드 제거
  // --------------------------------------------------------
  bool solveRiccati(
    const LqrStateMat & A_d,
    const LqrInputMat & B_d,
    LqrStateMat & P_out) ;

  // --- 파라미터 저장 ---
  LqrParams params_;

  // Q: 상태 추적 비용 행렬 (5×5 대각)
  LqrQMat Q_;

  // R: 입력 비용 행렬 (2×2 대각)
  LqrRMat R_;

  // --------------------------------------------------------
  // [신규 추가] Warm Start를 위한 이전 스텝 P 행렬 캐싱 변수
  // --------------------------------------------------------
  LqrStateMat P_prev_;
  bool is_p_initialized_ = false;
};

}  // namespace control_lqr

#endif  // CONTROL_LQR__LQR_CORE_HPP_
