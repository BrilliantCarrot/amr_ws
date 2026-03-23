#ifndef ESTIMATION__EKF_CORE_HPP_
#define ESTIMATION__EKF_CORE_HPP_

#include <Eigen/Dense>

namespace estimation
{

// ============================================================
// EKF (Extended Kalman Filter) 핵심 연산 클래스
//
// [EKF가 뭔가?]
//   칼만 필터(KF)는 선형 시스템에서 최적의 상태 추정을 하는 알고리즘임.
//   근데 실제 로봇의 운동 방정식은 비선형 (sin, cos 포함)이라서
//   KF를 그대로 쓸 수 없음.
//   EKF는 비선형 함수를 현재 상태 주변에서 1차 테일러 전개로 선형화해서
//   KF를 적용하는 방식임.
//
// [이 클래스의 역할]
//   ROS2 의존성 없이 순수하게 수식만 담당함.
//   덕분에 ROS 없이도 단독으로 단위 테스트 가능.
// ============================================================

// --- 상태/입력/관측 차원 상수 ---
// constexpr: 컴파일 타임 상수 → 매직 넘버 대신 이름으로 관리
constexpr int STATE_DIM      = 6;  // 상태 벡터 차원: [x, y, θ, vx, vy, ω]
constexpr int INPUT_DIM      = 3;  // IMU 입력 차원: [ax, ay, ω_imu]
constexpr int OBS_DIM        = 3;  // Odom 관측 차원: [vx, vy, ω_odom]
constexpr int LIDAR_OBS_DIM  = 3;  // LiDAR 관측 차원: [x, y, θ]
// Odom은 속도(vx, vy, ω)를 간접 관측 → 위치 추정에 누적 오차 발생
// LiDAR는 위치(x, y, θ)를 map frame 기준으로 직접 관측 → 드리프트 보정 가능

// --- Eigen 타입 별칭 ---
// 코드 가독성을 위해 긴 Eigen 타입을 짧게 alias 처리함
// 이 둘은 완전히 동일함
// Eigen::Matrix<double, 6, 1> x_;   // 원래 형태
// StateVec x_;                        // 별칭 사용
// Eigen::Matrix<데이터타입, 행 수, 열 수>
using StateVec       = Eigen::Matrix<double, STATE_DIM, 1>;              // 6×1 상태 벡터
// 값들의 묶음, 열벡터, 더하고 빼는 연산 위주
// Eigen::Matrix<double, 6, 1> x_; 와 동일
using StateMat       = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;      // 6×6 행렬
// 공분산 P, 자코비안 F, 노이즈 Q
// 변환행렬, 선형 연산자, 행렬 곱 연산 위주
using ObsVec         = Eigen::Matrix<double, OBS_DIM, 1>;                // 3×1 Odom 관측 벡터
// 센서에서 측정한 값들의 묶음
using ObsMat         = Eigen::Matrix<double, OBS_DIM, STATE_DIM>;        // 3×6 Odom 관측 행렬
// 6차원 상태를 3차원 관측으로 투영하는 변환
// 3행 6열이야 차원이 맞음
using KalmanMat      = Eigen::Matrix<double, STATE_DIM, OBS_DIM>;        // 6×3 Odom 칼만 게인 행렬

// LiDAR 관측 전용 타입
// Odom 타입과 이름이 달라서 함께 사용해도 충돌 없음
using LidarObsVec    = Eigen::Matrix<double, LIDAR_OBS_DIM, 1>;          // 3×1 LiDAR 관측 벡터 [x, y, θ]
using LidarObsMat    = Eigen::Matrix<double, LIDAR_OBS_DIM, STATE_DIM>;  // 3×6 LiDAR 관측 행렬
using LidarKalmanMat = Eigen::Matrix<double, STATE_DIM, LIDAR_OBS_DIM>;  // 6×3 LiDAR 칼만 게인 행렬
using LidarNoiseMat  = Eigen::Matrix<double, LIDAR_OBS_DIM, LIDAR_OBS_DIM>; // 3×3 LiDAR 관측 노이즈
// 크기는 같지만 이름을 분리한 이유가 있음.
// `LidarNoiseMat`이 핵심인데, 이게 **가변 R 행렬** 때문이야. Odom의 R은 고정값으로 멤버 변수에 저장했지만, LiDAR의 R은 localization 상태(NORMAL/DEGRADED)에 따라 매 호출마다 달라져야 해서 `updateLidar()` 인자로 외부에서 주입받는 구조. 그래서 타입을 명시적으로 분리. 즉:
// NORMAL   → R_lidar 작게 → 칼만 게인 K 커짐 → LiDAR 강하게 반영
// DEGRADED → R_lidar 크게 → 칼만 게인 K 작아짐 → LiDAR 약하게 반영
// LOST     → updateLidar() 호출 자체 안 함

// 벡터: 상태/관측값 자체를 담는 데이터 컨테이너(열벡터)
// 행렬: 상태를 변환하는 연산자

class EkfCore
{
public:
  EkfCore();

  // --------------------------------------------------------
  // 초기화
  //   x0 : 초기 상태 벡터 (보통 영벡터로 시작)
  //   P0 : 초기 공분산 행렬
  //        → 대각 성분이 클수록 "나는 초기값을 잘 모른다"는 의미
  //        → 이후 데이터가 쌓이면 P가 수렴하면서 추정 신뢰도 올라감
  // --------------------------------------------------------
  void init(const StateVec & x0, const StateMat & P0);

  // --------------------------------------------------------
  // Predict 단계 (IMU → 200Hz마다 호출)
  //
  //   IMU의 가속도(ax, ay)와 각속도(omega_imu)를 입력으로 받아
  //   운동 방정식 f(x, u)로 다음 상태를 예측함.
  //
  //   수식:
  //     x_pred = f(x, u)           ← 비선형 상태 전이
  //     P_pred = F·P·Fᵀ + Q        ← 공분산 전파
  //       F: 상태 전이 함수의 야코비안 (선형화 행렬)
  //       Q: 프로세스 노이즈 (IMU 적분 오차, 모델링 오차 등)
  //
  //   [왜 IMU로 예측하나?]
  //     IMU는 200Hz로 빠르게 들어와서 짧은 dt 동안의 운동을
  //     적분해 위치/속도를 갱신하기에 적합함.
  //     단, 적분 오차가 누적되므로 반드시 Odom으로 보정해야 함.
  //
  //   ax        : body frame x축 선가속도 [m/s²]
  //   ay        : body frame y축 선가속도 [m/s²]
  //   omega_imu : IMU 측정 각속도 [rad/s]
  //   dt        : 이전 predict 이후 경과 시간 [s]
  // --------------------------------------------------------
  void predict(double ax, double ay, double omega_imu, double dt);

  // --------------------------------------------------------
  // Update 단계 (Odom → 50Hz마다 호출)
  //
  //   Odom의 속도(vx, vy, ω)를 관측값으로 받아 예측값을 보정함.
  //   Odometry는 바퀴가 얼마나 굴렀는지로 내 위치를 추정하는것
  //
  //   바퀴에 엔코더(회전 센서) 부착
  //   ↓
  //   왼쪽 바퀴 N번 회전 + 오른쪽 바퀴 M번 회전
  //   ↓
  //   바퀴 반지름 × 회전수 = 이동 거리 계산
  //   ↓
  //   이전 위치 + 이동 거리 = 현재 위치 추정
  //
  //   수식:
  //     y = z - H·x_pred           ← innovation (잔차): 실제 관측과 예측의 차이
  //     (3×1) = (3×1) - (3×6)·(6×1)
  //     S = H·P·Hᵀ + R             ← innovation 공분산
  //     (3×3) = (3×6)·(6×6)·(6×3) + (3×3)
  //     K = P·Hᵀ·S⁻¹               ← 칼만 게인: 얼마나 관측을 믿을지 결정
  //     (6×3) = (6×6)·(6×3)·(3×3)
  //     x = x_pred + K·y           ← 상태 보정
  //     (6×1) = (6×1) + (6×3)·(3×1)
  //     P = (I - K·H)·P_pred       ← 공분산 갱신
  //
  //   [칼만 게인 K의 의미]
  //     K가 크면 → 관측(Odom)을 더 믿음 (R이 작을 때)
  //     K가 작으면 → 예측(IMU 모델)을 더 믿음 (Q가 작을 때)
  //     즉, K는 센서 노이즈와 모델 불확실성의 균형을 자동으로 맞춰줌.
  //
  //   vx_odom    : Odom 선속도 x [m/s]
  //   vy_odom    : Odom 선속도 y [m/s] (Diff-Drive에서는 보통 0)
  //   omega_odom : Odom 각속도 [rad/s]
  // --------------------------------------------------------
  void update(double vx_odom, double vy_odom, double omega_odom);

  // --------------------------------------------------------
  // 파라미터 설정
  //
  //   Q (프로세스 노이즈 공분산):
  //     모델이 현실을 얼마나 못 따라가는지를 나타냄.
  //     Q가 크면 → 모델을 불신 → 관측을 더 따라감 (반응성 ↑, 노이즈 민감)
  //     Q가 작으면 → 모델을 신뢰 → 예측 위주로 감 (안정적, 반응성 ↓)
  //
  //   R (관측 노이즈 공분산):
  //     센서(Odom)의 측정 노이즈 수준을 나타냄.
  //     R이 크면 → 센서를 불신 → 예측 위주로 감
  //     R이 작으면 → 센서를 신뢰 → 관측을 더 따라감
  // --------------------------------------------------------
  void setProcessNoise(const StateMat & Q);
  void setMeasurementNoise(const Eigen::Matrix<double, OBS_DIM, OBS_DIM> & R);

  //   H_lidar_는 생성자에서 이렇게 초기화됨:
  // H_lidar_.setZero();
  // H_lidar_(0, 0) = 1.0;  // x 관측
  // H_lidar_(1, 1) = 1.0;  // y 관측
  // H_lidar_(2, 2) = 1.0;  // θ 관측
  //            x   y   θ   vx  vy  ω
  // H_lidar = [ 1   0   0   0   0   0 ]  ← x를 직접 관측
  //            [ 0   1   0   0   0   0 ]  ← y를 직접 관측
  //            [ 0   0   1   0   0   0 ]  ← θ를 직접 관측
  // Odom H와 비교하면:
  //          x   y   θ   vx  vy  ω
  // H      = [ 0   0   0   1   0   0 ]  ← 속도만 관측 (위치 모름)
  // H_lidar= [ 1   0   0   0   0   0 ]  ← 위치를 직접 관측
  //            [ 0   1   0   0   0   0 ]
  //            [ 0   0   1   0   0   0 ]
  // **LiDAR가 위치를 직접 관측하기 때문에 드리프트를 끊어낼 수 있는 것.
  // Odom은 아무리 빠르게 보정해도 속도만 알 뿐 위치 오차를 직접 잡아주지 못함.
  // 전체 흐름은
  // W2:  predict(IMU) → update(Odom 속도)
  //                          ↓
  //                위치는 간접 추정만 → 드리프트 누적

  // W5:  predict(IMU) → update(Odom 속도) → updateLidar(LiDAR 위치)
  //                                                ↓
  //                                map frame 절대 위치로 직접 보정 → 드리프트 제거
  // --------------------------------------------------------
  // LiDAR Update 단계 (map→base_link TF → 10Hz마다 호출)
  //
  //   slam_toolbox가 추정한 map 프레임 기준 위치(x, y, θ)를
  //   관측값으로 받아 EKF 상태를 보정함.
  //
  //   [Odom update()와의 차이]
  //     update()     : 속도(vx, vy, ω) 관측 → H 고정, R 고정
  //     updateLidar(): 위치(x, y, θ)  관측 → H 고정, R 가변
  //
  //   [왜 R을 매 호출마다 외부에서 주입하나?]
  //     LiDAR localization 품질이 상황마다 달라지기 때문.
  //     localization_monitor가 발행하는 status에 따라:
  //       NORMAL   → R 작게 (LiDAR를 신뢰)
  //       DEGRADED → R 크게 (LiDAR를 반신반의)
  //       LOST     → updateLidar() 호출 자체를 건너뜀 (map_ekf_node에서 처리)
  //     이렇게 하면 EKF 내부 수식은 그대로이고,
  //     노드 레벨에서 R 값만 바꿔서 동작을 제어할 수 있음.
  //
  //   [H_lidar 구조]
  //         x   y   θ   vx  vy  ω
  //     x [ 1   0   0   0   0   0 ]
  //     y [ 0   1   0   0   0   0 ]
  //     θ [ 0   0   1   0   0   0 ]
  //     위치(x, y, θ)를 직접 관측 → 선형 관측 함수 → H는 상수 행렬
  //
  //   [yaw innovation 처리]
  //     θ의 차이(innovation)는 -π ~ π 범위로 정규화 필요.
  //     예: 실제 θ = 3.1rad, 예측 θ = -3.1rad → 차이가 6.2rad가 아닌 0.08rad이어야 함.
  //
  //   x_lidar : map 프레임 기준 x 위치 [m]
  //   y_lidar : map 프레임 기준 y 위치 [m]
  //   yaw_lidar: map 프레임 기준 yaw [rad]
  //   R_lidar : 이번 관측의 노이즈 공분산 (3×3, 외부에서 주입)
  // --------------------------------------------------------
  void updateLidar(
    double x_lidar, double y_lidar, double yaw_lidar,
    const LidarNoiseMat & R_lidar);

  // --- getter ---
  const StateVec & getState()       const { return x_; }
  const StateMat & getCovariance()  const { return P_; }

  // innovation norm 반환
  // innovation이 갑자기 폭증하면 EKF가 발산(diverge)하고 있다는 신호임.
  // → W10 Watchdog에서 이 값으로 Fail-safe 트리거 예정
  double getInnovationNorm() const { return innovation_norm_; }

private:
  // --------------------------------------------------------
  // 비선형 상태 전이 함수 f(x, u)
  //
  //   현재 상태 x_와 IMU 입력(ax, ay, ω)을 받아
  //   dt 후의 예측 상태를 반환함.
  //
  //   world frame (odom frame):
  //   고정된 기준 좌표계.
  //   로봇이 어디 있든 상관없이 땅에 고정된 축.
  //   x = 동쪽, y = 북쪽 같은 개념.
  //
  //   body frame:
  //   로봇에 붙어있는 좌표계.
  //   로봇이 회전하면 같이 회전함.
  //   vx = 로봇 앞방향 속도, vy = 로봇 옆방향 속도.
  //
  //   Diff-Drive 로봇의 운동 방정식 (body frame 기준):
  //     x_new  = x  + (vx·cos θ - vy·sin θ)·dt body에서 world
  //     y_new  = y  + (vx·sin θ + vy·cos θ)·dt body에서 world
  //     로봇의 heading θ만큼 회전시켜서 world frame 속도를 구하고, 거기에 dt 곱해서 위치를 갱신하는 것.
  //     θ_new  = θ  + ω·dt yaw 갱신, 각속도를 dt동안 적분하면 얼마나 회전했는지 나오는 단순 적분
  //     단 결과를 -pi ~ pi로 정규화 해야 됨
  //     vx_new = vx + ax·dt
  //     vy_new = vy + ay·dt
  //     ω_new  = ω_imu  ← IMU 직접 사용 (적분 안 함)
  //     가속도(ax) → 적분 → 속도(vx)  (IMU가 직접 못 줌)
  //     각속도(ω)  → 직접 측정         (IMU가 직접 줌)
  //
  //   [왜 sin/cos가 포함되나?]
  //     body frame 속도를 world frame 위치로 변환할 때
  //     로봇의 현재 heading(θ)을 기준으로 회전 변환이 필요하기 때문.
  // --------------------------------------------------------
  StateVec motionModel(double ax, double ay, double omega_imu, double dt) const;

  // --------------------------------------------------------
  // 야코비안 행렬 F 계산
  //
  //   EKF는 비선형 함수 f를 현재 상태 주변에서 편미분해서 선형화함,
  //   비선형 운동 방정식을 칼만 필터가 다룰수 있는 선형 형태로 근사화.
  //   이 선형화 행렬이 야코비안 F이며, 공분산 전파에 사용,
  //   P: 내 추정값이 얼마나 불확실한가
  //   P가 P_pred = F·P·Fᵀ + Q에서 로봇이 이동하면 불확실성도 같이 이동/변형되는데,
  //   그 변형을 계산하려면 운동 방정식의 기울기(자코비안 F)가 필요
  //   로봇이 북쪽으로 직진 중:
  //   x 방향 불확실성 → 북쪽으로 늘어남
  //   y 방향 불확실성 → 거의 안 바뀜
  //
  //   로봇이 45도 방향으로 직진 중:
  //   x, y 불확실성 → 둘 다 대각선으로 늘어남
  //
  //   이 "불확실성이 어느 방향으로 얼마나 늘어나는지"를
  //   계산하는 게 F·P·Fᵀ이고,
  //   F(야코비안)가 없으면 방향 정보를 모름.
  //
  //   F = ∂f/∂x  (6×6 행렬)
  //
  //   비선형 항목(sin θ, cos θ)만 편미분이 생기고
  //   나머지는 단위행렬 형태가 됨.
  //   핵심 비선형 편미분:
  //     ∂x_new/∂θ = (-vx·sin θ - vy·cos θ)·dt
  //     ∂y_new/∂θ = ( vx·cos θ - vy·sin θ)·dt
  // --------------------------------------------------------
  StateMat computeJacobian(double ax, double ay, double omega_imu, double dt) const;

  // --- 멤버 변수 ---
  StateVec x_;   // 현재 상태 추정값
  StateMat P_;   // 현재 공분산 (추정 불확실성)
  StateMat Q_;   // 프로세스 노이즈 공분산
  Eigen::Matrix<double, OBS_DIM, OBS_DIM> R_;  // Odom 관측 노이즈 공분산
  ObsMat   H_;      // Odom 관측 행렬: [vx, vy, ω] 고정
  LidarObsMat H_lidar_;  // LiDAR 관측 행렬: [x, y, θ] 고정
  // H_lidar_는 생성자에서 한 번만 설정하고 이후 변경 없음
  // LiDAR 관측 함수 h_lidar(x) = H_lidar · x 가 선형이기 때문

  double innovation_norm_;  // 최근 update의 innovation 크기
  bool is_initialized_;     // init() 호출 여부 플래그
};

}  // namespace estimation

#endif  // ESTIMATION__EKF_CORE_HPP_