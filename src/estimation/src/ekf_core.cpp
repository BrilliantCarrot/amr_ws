#include "estimation/ekf_core.hpp"
#include <cmath>
#include <stdexcept>

namespace estimation
{

// ============================================================
// 생성자
//   멤버 변수를 안전한 초기값으로 설정함.
//   실제 초기화는 init()에서 이루어짐.
// ============================================================
EkfCore::EkfCore()
: innovation_norm_(0.0),
  is_initialized_(false)
{
  // 상태/공분산을 영으로 초기화
  x_.setZero();
  P_.setZero();

  // 기본 프로세스 노이즈 Q — 대각 행렬
  // 각 값의 의미: 해당 상태가 1스텝당 얼마나 불확실한지
  // 튜닝 포인트: 실험 결과에 따라 조정 필요
  Q_.setZero();
  Q_(0, 0) = 0.01;   // x 위치 노이즈 [m²]
  Q_(1, 1) = 0.01;   // y 위치 노이즈 [m²]
  Q_(2, 2) = 0.001;  // θ (yaw) 노이즈 [rad²]
  Q_(3, 3) = 0.1;    // vx 노이즈 [m²/s²]
  Q_(4, 4) = 0.1;    // vy 노이즈 [m²/s²]
  Q_(5, 5) = 0.01;   // ω 노이즈 [rad²/s²]

  // 기본 관측 노이즈 R — Odom 센서의 신뢰도
  // Odom은 wheel slip이 있으므로 Q보다 크게 설정
  R_.setZero();
  R_(0, 0) = 0.1;    // vx 관측 노이즈 [m²/s²]
  R_(1, 1) = 0.1;    // vy 관측 노이즈 [m²/s²]
  R_(2, 2) = 0.05;   // ω 관측 노이즈 [rad²/s²]

  // 관측 행렬 H — Odom이 측정하는 상태 성분을 지정함
  // 상태: [x, y, θ, vx, vy, ω]
  // Odom 관측: [vx, vy, ω] → 인덱스 3, 4, 5에 해당
  //
  //       x   y   θ   vx  vy  ω
  // H = [ 0   0   0   1   0   0 ]  ← vx 관측
  //     [ 0   0   0   0   1   0 ]  ← vy 관측
  //     [ 0   0   0   0   0   1 ]  ← ω  관측
  //
  // [왜 H가 고정인가?]
  //   관측 함수 h(x) = H·x 가 선형이기 때문.
  //   위치(x, y, θ)는 Odom으로 직접 측정 불가능하고
  //   속도(vx, vy, ω)만 wheel encoder로 측정 가능함.
  H_.setZero();
  H_(0, 3) = 1.0;
  H_(1, 4) = 1.0;
  H_(2, 5) = 1.0;
}

// ============================================================
// 초기화
// ============================================================
void EkfCore::init(const StateVec & x0, const StateMat & P0)
{
  x_ = x0;
  P_ = P0;
  is_initialized_ = true;
}

// ============================================================
// 파라미터 설정
// ============================================================
void EkfCore::setProcessNoise(const StateMat & Q)
{
  Q_ = Q;
}

void EkfCore::setMeasurementNoise(const Eigen::Matrix<double, OBS_DIM, OBS_DIM> & R)
{
  R_ = R;
}

// ============================================================
// 비선형 상태 전이 함수 f(x, u)
//
//   [Differential Drive 운동 방정식]
//   body frame의 속도(vx, vy)를 world frame으로 변환해 위치를 갱신함.
//   회전 행렬:
//     [cos θ  -sin θ] [vx]   ← world frame x 속도
//     [sin θ   cos θ] [vy]   ← world frame y 속도
//
//   전체 상태 전이:
//     x_new  = x  + (vx·cosθ - vy·sinθ)·dt
//     y_new  = y  + (vx·sinθ + vy·cosθ)·dt
//     θ_new  = θ  + ω·dt
//     vx_new = vx + ax·dt   ← 가속도 적분으로 속도 갱신
//     vy_new = vy + ay·dt
//     ω_new  = ω_imu        ← IMU 각속도를 직접 사용 (더 신뢰도 높음)
// ============================================================
StateVec EkfCore::motionModel(
  double ax, double ay, double omega_imu, double dt) const
{
  StateVec x_new;

  const double theta = x_(2);   // 현재 yaw
  const double vx    = x_(3);   // 현재 body frame x 속도
  const double vy    = x_(4);   // 현재 body frame y 속도
  const double omega = x_(5);   // 현재 각속도

  // world frame 위치 갱신 (body → world 좌표 변환 후 적분)
  x_new(0) = x_(0) + (vx * std::cos(theta) - vy * std::sin(theta)) * dt;
  x_new(1) = x_(1) + (vx * std::sin(theta) + vy * std::cos(theta)) * dt;

  // yaw 갱신 — 각속도 적분
  // atan2(sin, cos)으로 -π ~ π 범위 정규화
  const double theta_new = theta + omega * dt;
  x_new(2) = std::atan2(std::sin(theta_new), std::cos(theta_new));

  // 속도 갱신 — IMU 가속도 적분
  x_new(3) = vx + ax * dt;
  x_new(4) = vy + ay * dt;

  // 각속도 — IMU 직접 사용 (적분 대신)
  // [왜 적분 안 하나?]
  //   IMU는 각속도를 직접 측정하므로 그 값 자체가 ω의 최선 추정값.
  //   ω를 또 적분하면 오차가 중첩됨.
  x_new(5) = omega_imu;

  return x_new;
}

// ============================================================
// 야코비안 행렬 F = ∂f/∂x 계산 (6×6)
//
//   비선형 함수 f를 현재 상태 x_ 주변에서 편미분.
//   선형 항목은 단위행렬 형태, 비선형 항목(sin/cos)만 별도 계산.
//
//   핵심 비선형 편미분 (θ에 대한 미분):
//     ∂x_new/∂θ = (-vx·sinθ - vy·cosθ)·dt
//     ∂y_new/∂θ = ( vx·cosθ - vy·sinθ)·dt
//
//   vx, vy에 대한 편미분:
//     ∂x_new/∂vx =  cosθ·dt
//     ∂x_new/∂vy = -sinθ·dt
//     ∂y_new/∂vx =  sinθ·dt
//     ∂y_new/∂vy =  cosθ·dt
//
//   최종 F 행렬 구조 (비선형 항목만 표시):
//          x   y   θ                      vx          vy         ω
//   x  [   1   0  (-vx·sθ - vy·cθ)·dt    cθ·dt      -sθ·dt      0  ]
//   y  [   0   1  ( vx·cθ - vy·sθ)·dt    sθ·dt       cθ·dt      0  ]
//   θ  [   0   0   1                      0           0          dt ]
//   vx [   0   0   0                      1           0           0  ]
//   vy [   0   0   0                      0           1           0  ]
//   ω  [   0   0   0                      0           0           0  ]
// ============================================================
StateMat EkfCore::computeJacobian(
  double /*ax*/, double /*ay*/, double /*omega_imu*/, double dt) const
{
  StateMat F;
  F.setIdentity();  // 기본값: 단위행렬 (선형 항목)

  const double theta = x_(2);
  const double vx    = x_(3);
  const double vy    = x_(4);

  const double c_theta = std::cos(theta);
  const double s_theta = std::sin(theta);

  // x 위치 행 (0행)
  F(0, 2) = (-vx * s_theta - vy * c_theta) * dt;  // ∂x_new/∂θ
  F(0, 3) =   c_theta * dt;                         // ∂x_new/∂vx
  F(0, 4) =  -s_theta * dt;                         // ∂x_new/∂vy

  // y 위치 행 (1행)
  F(1, 2) = ( vx * c_theta - vy * s_theta) * dt;  // ∂y_new/∂θ
  F(1, 3) =   s_theta * dt;                         // ∂y_new/∂vx
  F(1, 4) =   c_theta * dt;                         // ∂y_new/∂vy

  // θ 행 (2행) — ω에 대한 편미분
  F(2, 5) = dt;  // ∂θ_new/∂ω

  // vx, vy, ω 행은 단위행렬 그대로 (선형 항목)
  // ω 행은 IMU 직접 대입이므로 상태 의존성 없음 → 0 (setIdentity로 이미 설정됨)
  F(5, 5) = 0.0;  // ∂ω_new/∂ω = 0 (IMU 직접 대입이므로 이전 상태 ω 무관)

  return F;
}

// ============================================================
// Predict 단계
//   1. 운동 방정식으로 상태 예측
//   2. 야코비안으로 공분산 전파
// ============================================================
void EkfCore::predict(double ax, double ay, double omega_imu, double dt)
{
  if (!is_initialized_) {
    throw std::runtime_error("[EkfCore] predict() 호출 전 init()이 필요함");
  }

  // dt 유효성 검사 — 너무 크면 선형화 오차 폭증
  if (dt <= 0.0 || dt > 1.0) {
    return;  // 비정상적인 dt는 무시
  }

  // 1) 상태 예측: x_pred = f(x, u)
  x_ = motionModel(ax, ay, omega_imu, dt);

  // 2) 야코비안 계산: F = ∂f/∂x (현재 상태 기준 선형화)
  const StateMat F = computeJacobian(ax, ay, omega_imu, dt);

  // 3) 공분산 전파: P_pred = F·P·Fᵀ + Q
  //   [의미]
  //     F·P·Fᵀ : 상태 전이로 인한 불확실성 전파
  //     Q      : 모델링 오차, IMU 노이즈로 인해 새로 추가되는 불확실성
  //   시간이 지날수록 P가 커지는 이유: 예측만 하면 오차가 쌓이기 때문.
  //   → Update 단계에서 관측으로 P를 줄여줌.
  P_ = F * P_ * F.transpose() + Q_;
}

// ============================================================
// Update 단계
//   Odom 관측값으로 Predict의 예측값을 보정함.
// ============================================================
void EkfCore::update(double vx_odom, double vy_odom, double omega_odom)
{
  if (!is_initialized_) {
    throw std::runtime_error("[EkfCore] update() 호출 전 init()이 필요함");
  }

  // 1) 관측 벡터 구성
  ObsVec z;
  z << vx_odom, vy_odom, omega_odom;

  // 2) Innovation(잔차) 계산: y = z - H·x_pred
  //   [의미]
  //     실제 관측값(z)과 예측된 관측값(H·x)의 차이.
  //     y가 크면 → 예측이 현실과 많이 다름 → 더 많이 보정.
  //     y가 작으면 → 예측이 정확함 → 조금만 보정.
  const ObsVec y = z - H_ * x_;

  // innovation norm 저장 (W10 Watchdog용 — EKF divergence 감지)
  innovation_norm_ = y.norm();

  // 3) Innovation 공분산: S = H·P·Hᵀ + R
  //   [의미]
  //     예측 불확실성(H·P·Hᵀ)과 센서 노이즈(R)의 합.
  //     S가 클수록 전체 불확실성이 큰 것.
  const Eigen::Matrix<double, OBS_DIM, OBS_DIM> S = H_ * P_ * H_.transpose() + R_;

  // 4) 칼만 게인 계산: K = P·Hᵀ·S⁻¹
  //   [의미]
  //     K는 예측(모델)과 관측(센서) 중 얼마나 센서를 믿을지 결정하는 가중치.
  //     P가 크고(예측 불확실) R이 작으면(센서 신뢰) → K 커짐 → 센서를 더 믿음.
  //     P가 작고(예측 정확) R이 크면(센서 불신) → K 작아짐 → 모델을 더 믿음.
  //
  //   ldlt(): S가 대칭 양정치 행렬이므로 LDLT 분해로 역행렬 대신 선형시스템 풀기
  //   → 수치적으로 더 안정적이고 빠름
  const KalmanMat K = P_ * H_.transpose() * S.ldlt().solve(
    Eigen::Matrix<double, OBS_DIM, OBS_DIM>::Identity());

  // 5) 상태 보정: x = x_pred + K·y
  //   [의미]
  //     예측 상태에서 innovation 방향으로 K만큼 이동.
  //     K·y = "센서가 말하는 방향으로, 칼만 게인만큼 수정"
  x_ = x_ + K * y;

  // yaw 정규화: -π ~ π 유지
  x_(2) = std::atan2(std::sin(x_(2)), std::cos(x_(2)));

  // 6) 공분산 갱신: P = (I - K·H)·P_pred
  //   [의미]
  //     관측으로 불확실성이 줄어듦.
  //     K·H 만큼 P가 감소 → "관측 덕분에 이만큼 더 확신하게 됐다"
  //
  //   [수치 안정성을 위한 조셉 형태 (Joseph form) 선택지]
  //     P = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ
  //     → 이론적으로 P의 양정치성 보장이 더 강함 (이번엔 기본형 사용)
  const StateMat I = StateMat::Identity();
  P_ = (I - K * H_) * P_;
}

}  // namespace estimation