#include "control_mpc/mpc_core.hpp"

#include <chrono>
#include <cstring>
#include <cmath>

namespace control_mpc
{

// ============================================================
// 생성자 / 소멸자
// ============================================================

MpcCore::MpcCore()
: work_(nullptr), settings_(nullptr), is_initialized_(false)
{
}

MpcCore::~MpcCore()
{
  if (work_) {
    osqp_cleanup(work_);
    work_ = nullptr;
  }
  if (settings_) {
    c_free(settings_);
    settings_ = nullptr;
  }
}

// ============================================================
// init() — OSQP 세팅 초기화 및 QP 차원 설정
// ============================================================

void MpcCore::init(const MpcParams & params)
{
  params_ = params;
  int N = params_.N;

  // QP 변수/제약 차원 계산
  // z = [x_0,...,x_N, u_0,...,u_{N-1}]
  nz_ = NX * (N + 1) + NU * N;

  // 제약 행 수:
  //   초기 상태 등식:  NX      (x_0 = x_current)
  //   동역학 등식:    NX * N   (x_{k+1} = A_k*x_k + B_k*u_k + c_k)
  //   상태 부등식:    NX * N   (v, ω 범위)
  //   입력 부등식:    NU * N   (Δv, Δω 범위)
  nc_ = NX + NX * N + NX * N + NU * N;

  // --- 비용 가중치 행렬 구성 ---
  Q_.setZero();
  Q_(0, 0) = params_.q_x;
  Q_(1, 1) = params_.q_y;
  Q_(2, 2) = params_.q_th;
  Q_(3, 3) = params_.q_v;
  Q_(4, 4) = params_.q_w;

  R_.setZero();
  R_(0, 0) = params_.r_dv;
  R_(1, 1) = params_.r_dw;

  // QP 행렬 사전 할당
  P_mat_.setZero(nz_, nz_);
  q_vec_.setZero(nz_);
  A_mat_.setZero(nc_, nz_);
  l_vec_.setZero(nc_);
  u_vec_.setZero(nc_);

  // --- OSQP settings 초기화 ---
  // c_malloc: OSQP 내부 할당자 (c_free로 해제해야 함)
  // OSQP는 C 라이브러리라서 메모리 관리를 직접 해야 함. C++ new가 아니라 C 스타일 c_malloc/c_free를 씀.
  // 줄별 상세 설명
  // ① settings_ = static_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));
  // *: OSQPSettings 구조체의 주소를 저장하는 포인터
  // c_malloc(sizeof(OSQPSettings))
  // sizeof(OSQPSettings): OSQPSettings 구조체 크기(바이트)를 컴파일 타임에 계산
  // c_malloc(...): OSQP 전용 malloc. 내부적으로 malloc()과 동일하지만 
  // OSQP 빌드 설정에 따라 커스텀 할당자로 교체 가능하도록 래핑된 것
  // 반환 타입이 void*이므로 static_cast<OSQPSettings *>로 명시적 캐스팅
  // ⚠️ 이렇게 할당한 메모리는 반드시 c_free(settings_)로 해제해야 함. delete 쓰면 안 됨.
  settings_ = static_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings_);

  settings_->verbose    = false;   // 매 solve마다 출력 끄기
  settings_->warm_start = true;    // v0.6.3 필드명: warm_start (v1.x는 warm_starting)
  settings_->max_iter   = 400;     // 최대 반복 수
  settings_->eps_abs    = 1e-4;    // 절대 수렴 허용 오차
  settings_->eps_rel    = 1e-4;    // 상대 수렴 허용 오차
  settings_->polish     = false;   // polishing 끄기 (시간 절약)

// 구조체 내부
// typedef struct {
//     c_int   max_iter;       // 최대 반복 횟수
//     c_float eps_abs;        // 절대 수렴 허용 오차
//     c_float eps_rel;        // 상대 수렴 허용 오차
//     c_int   verbose;        // 로그 출력 여부
//     c_int   warm_start;     // 웜 스타트 여부
//     c_int   polish;         // polishing 여부
//     // ... 그 외 많은 필드들
// } OSQPSettings;

  is_initialized_ = true;
}

// ============================================================
// motionModel() — 비선형 운동 방정식 f(x, u)
//
//   클래스 멤버가 아닌 namespace 레벨 타입이므로
//   반환 타입을 MpcCore::StateVec 이 아닌 StateVec으로 선언
// ============================================================

StateVec MpcCore::motionModel(const StateVec & x, const InputVec & u) const
{
  StateVec x_next;
  double theta = x(2);
  double v     = x(3);
  double dt    = params_.dt;

  // body frame 속도 → world frame 위치 변환
  x_next(0) = x(0) + v * std::cos(theta) * dt;   // x
  x_next(1) = x(1) + v * std::sin(theta) * dt;   // y
  x_next(2) = x(2) + x(4) * dt;                  // θ (ω 적분)
  x_next(3) = x(3) + u(0);                        // v (Δv 적용)
  x_next(4) = x(4) + u(1);                        // ω (Δω 적용)

  // θ 정규화 제거
  // mpc_node에서 연속값(Continuous Yaw)으로 넘겨주므로, 
  // 선형화 오차 누적을 막기 위해 모델 내부에서도 절대값 연속성을 그대로 유지함
  return x_next;
}

// ============================================================
// linearize() — 야코비안 A_k, B_k 계산
//
//   f(x,u) ≈ f(xbar,ubar) + A_k*(x-xbar) + B_k*(u-ubar)
// ============================================================

void MpcCore::linearize(
  const StateVec & xbar,
  const InputVec & /*ubar*/,
  StateMat & Ak,
  InputMat & Bk) const
{
  double theta = xbar(2);
  double v     = xbar(3);
  double dt    = params_.dt;

  // A_k: 상태 전이 야코비안 (기본 = 단위행렬)
  Ak.setIdentity();
  //   x_next(0) = x + v*cos(θ)*dt  → ∂/∂θ = -v*sin(θ)*dt, ∂/∂v = cos(θ)*dt
  //   x_next(1) = y + v*sin(θ)*dt  → ∂/∂θ =  v*cos(θ)*dt, ∂/∂v = sin(θ)*dt
  //   x_next(2) = θ + ω*dt         → ∂/∂ω = dt
  Ak(0, 2) = -v * std::sin(theta) * dt;
  Ak(0, 3) =      std::cos(theta) * dt;
  Ak(1, 2) =  v * std::cos(theta) * dt;
  Ak(1, 3) =      std::sin(theta) * dt;
  Ak(2, 4) = dt;
  // Ak.setIdentity();              // 대각선 전부 1로 초기화
  // Ak(0, 2) = -v * sin(θ) * dt;  // A(0,2) = ∂f₀/∂θ
  // Ak(0, 3) =  cos(θ) * dt;      // A(0,3) = ∂f₀/∂v
  // Ak(1, 2) =  v * cos(θ) * dt;  // A(1,2) = ∂f₁/∂θ
  // Ak(1, 3) =  sin(θ) * dt;      // A(1,3) = ∂f₁/∂v
  // Ak(2, 4) =  dt;                // A(2,4) = ∂f₂/∂ω
  // setIdentity()로 단위행렬을 깔고, 비선형 항 4 + dt항 1개만 덮어쓰는 구조.

  // B_k: 입력 야코비안
  //   v_next = v + Δv → B(3,0) = 1
  //   ω_next = ω + Δω → B(4,1) = 1
  // B_k는 xbar에 무관한 상수 행렬 (선형이므로 선형화 오차 없음)
  // 편미분 결과가 상수 1, 항상 똑같은 행렬
  // 반면 A_k는 매 스탭 바뀜
  Bk.setZero();
  Bk(3, 0) = 1.0;
  Bk(4, 1) = 1.0;
}

// ============================================================
// setObstacles() — 장애물 목록 설정
// ============================================================
void MpcCore::setObstacles(const std::vector<Obstacle> & obstacles)
{
  obstacles_ = obstacles;
}

// ============================================================
// buildQP() — P, q, A, l, u 행렬 조립
//
//   z = [x_0, ..., x_N, u_0, ..., u_{N-1}]
// ============================================================

void MpcCore::buildQP(
  const StateVec & x0,
  const std::vector<StateVec> & x_ref,
  const InputVec & u_prev,
  const std::vector<StateMat> & Ak_seq,
  const std::vector<InputMat> & Bk_seq,
  const std::vector<StateVec> & ck_seq)
{
  int N = params_.N;

  P_mat_.setZero(nz_, nz_);
  q_vec_.setZero(nz_);
  A_mat_.setZero(nc_, nz_);
  l_vec_.setZero(nc_);
  u_vec_.setZero(nc_);

  int u_offset = NX * (N + 1);   // z 벡터에서 입력 시퀀스 시작 인덱스

  // === 1. P 행렬 (비용 Hessian) ===
  // 상태 블록 (k=1~N): Q
  // x_0는 등식으로 고정되므로 비용 블록 불필요
  for (int k = 1; k <= N; ++k) {
    int idx = k * NX;
    P_mat_.block<NX, NX>(idx, idx) = Q_;
  }
  // 입력 블록 (k=0~N-1): R
  for (int k = 0; k < N; ++k) {
    int idx = u_offset + k * NU;
    P_mat_.block<NU, NU>(idx, idx) = R_;
  }

  // === 2. q 벡터 (선형 비용) ===
  // tracking 항 전개: (x_k - x_ref_k)ᵀ Q (x_k - x_ref_k)
  // 선형 항 = -Q * x_ref_k (OSQP는 ½ zᵀPz + qᵀz 형태)
  for (int k = 1; k <= N; ++k) {
    int idx = k * NX;
    q_vec_.segment<NX>(idx) = -Q_ * x_ref[k];
  }

  // === 3. A 행렬 및 l, u 벡터 ===
  // ================================================================
  // A 행렬 전체 구조 (먼저 큰 그림부터 이해하고 코드 읽기)
  //
  // OSQP가 푸는 문제: l ≤ A·z ≤ u
  //
  // z 벡터 = [x_0, x_1, ..., x_N, u_0, u_1, ..., u_{N-1}]
  //           ↑ 상태 시퀀스 (각 5차원)  ↑ 입력 시퀀스 (각 2차원)
  //
  // A 행렬은 "z 안의 어떤 원소들이 어떤 조건을 만족해야 하나"를
  // 행 하나당 제약 하나씩 표현한 큰 행렬임.
  //
  // 행이 쌓이는 순서 (row 변수가 아래로 내려가면서 채워짐):
  //
  //   행  0 ~  4 : ① 초기 상태 등식   x_0 = x_current         (5행)
  //   행  5 ~ 104 : ② 동역학 등식     x_{k+1} = A_k*x_k+...  (5*N행)
  //   행 105 ~ 204 : ③ 상태 부등식    v, ω 범위 제한          (5*N행)
  //   행 205 ~ 244 : ④ 입력 부등식    Δv, Δω 범위 제한        (2*N행)
  //
  //   (N=20 기준 예시)
  // ================================================================

  int row = 0;  // 지금 A 행렬의 몇 번째 행을 채우고 있는지 추적하는 커서

  // ----------------------------------------------------------------
  // ① 초기 상태 등식: x_0 = x_current
  //
  // "MPC가 계획을 세울 때 출발점은 반드시 지금 EKF가 말하는
  //  실제 로봇 위치에서 시작해야 한다"는 제약임.
  //
  // 수식으로 표현하면:
  //   I · x_0 = x_current
  //
  // A 행렬에서 어떻게 표현하나?
  //   z 벡터에서 x_0는 맨 앞 NX(=5)개 원소임.
  //   그 위치에 단위행렬 I를 꽂으면:
  //   A·z 를 계산할 때 "x_0 그대로 뽑아라" 가 됨.
  //
  //   l = x_current  (하한)
  //   u = x_current  (상한)
  //   → l = u 이면 등식 제약. "반드시 이 값이어야 함"
  //
  // 시각적으로:
  //   A 행렬 (이 5행만 보면):
  //
  //   열→  x_0열     x_1열  x_2열  ...  u_0열  ...
  //   ┌─────────────────────────────────────────┐
  //   │  1 0 0 0 0 │ 0 ... 0 │ ... │ 0 ... 0  │  ← 행0
  //   │  0 1 0 0 0 │ 0 ... 0 │ ... │ 0 ... 0  │  ← 행1
  //   │  0 0 1 0 0 │ 0 ... 0 │ ... │ 0 ... 0  │  ← 행2
  //   │  0 0 0 1 0 │ 0 ... 0 │ ... │ 0 ... 0  │  ← 행3
  //   │  0 0 0 0 1 │ 0 ... 0 │ ... │ 0 ... 0  │  ← 행4
  //   └─────────────────────────────────────────┘
  //   l = [x, y, θ, v, ω]_current
  //   u = [x, y, θ, v, ω]_current

  // z = [x_0 | x_1 | x_2 | ... | x_20 | u_0 | u_1 | ... | u_19]
  // x_0  : 열 0~4      (5개)
  // x_1  : 열 5~9      (5개)
  // x_2  : 열 10~14    (5개)
  // ...
  // x_20 : 열 100~104  (5개)
  // ─────────────────────────────
  // u_0  : 열 105~106  (2개) ← u_offset = NX*(N+1) = 5*21 = 105
  // u_1  : 열 107~108  (2개)
  // ...
  // u_19 : 열 143~144  (2개)
  // ─────────────────────────────
  // 총 145열
  // ----------------------------------------------------------------
  A_mat_.block<NX, NX>(row, 0) = StateMat::Identity();
  // block<행 크기, 열 크기>(시작행, 시작열)
  // → A 행렬의 (row, 0) 위치에 5×5 단위행렬 삽입
  // → z의 x_0 구간(열 0~4)을 그대로 읽는 효과

  l_vec_.segment<NX>(row) = x0;  // 하한 = 현재 상태
  u_vec_.segment<NX>(row) = x0;  // 상한 = 현재 상태 (l=u → 등식)
  row += NX;  // 커서를 5칸 아래로 내림 (다음 제약 블록 시작 위치)


  // ----------------------------------------------------------------
  // ② 동역학 등식: x_{k+1} = A_k * x_k + B_k * u_k + c_k
  //
  // "MPC가 예측하는 미래 상태 시퀀스는 반드시 운동 방정식을
  //  따라야 한다"는 제약임. 물리 법칙을 강제하는 것.
  //
  // 수식 정리 과정:
  //   x_{k+1} = A_k * x_k + B_k * u_k + c_k
  //   0 = A_k * x_k - x_{k+1} + B_k * u_k + c_k
  //   A_k * x_k - x_{k+1} + B_k * u_k = -c_k  ← 이 형태로 넣음
  //
  // z 벡터에서 각 변수의 열 위치:
  //   x_k   : k * NX        번째 열부터 NX개
  //   x_{k+1}: (k+1) * NX   번째 열부터 NX개
  //   u_k   : u_offset + k * NU 번째 열부터 NU개
  //
  // A 행렬에서 어떻게 표현하나? (k=0인 경우 예시)
  //
  //   열→  x_0열     x_1열       u_0열
  //   ┌──────────────────────────────────────┐
  //   │  A_0(5×5) │ -I(5×5) │  B_0(5×2)  │  ← 5행 (k=0 동역학)
  //   └──────────────────────────────────────┘
  //
  //   A_0  : 현재 상태 x_0가 다음 상태에 미치는 영향 (자코비안)
  //   -I   : 다음 상태 x_1을 빼는 것 (왼쪽으로 이항했으므로)
  //   B_0  : 입력 u_0가 다음 상태에 미치는 영향
  //
  //   l = u = -c_0 (등식)
  //   → "이 행의 A·z 계산 결과가 반드시 -c_k 이어야 함"
  //   → 즉 A_k*x_k - x_{k+1} + B_k*u_k = -c_k 강제

  // A 행렬에 A_k, -I, B_k 세 블록을 꽂고 l=u=-c_k 로 설정하면, 
  // OSQP가 z를 풀 때 x_{k+1} = A_k·x_k + B_k·u_k + c_k 를 자동으로 만족하는 해만 찾게 됨.
  // ----------------------------------------------------------------
  for (int k = 0; k < N; ++k) {

    // z 벡터에서 각 변수의 시작 열 인덱스 계산
    int x_k_col  = k * NX;              // x_k가 z에서 시작하는 열
    int x_k1_col = (k + 1) * NX;        // x_{k+1}이 z에서 시작하는 열
    int u_k_col  = u_offset + k * NU;   // u_k가 z에서 시작하는 열
    // u_offset = NX*(N+1) : 상태 시퀀스가 끝나고 입력 시퀀스가 시작하는 위치

    A_mat_.block<NX, NX>(row, x_k_col)  =  Ak_seq[k];
    // x_k 열 위치에 야코비안 A_k 삽입
    // → "x_k가 바뀌면 동역학 제약이 A_k만큼 영향받음"

    A_mat_.block<NX, NX>(row, x_k1_col) = -StateMat::Identity();
    // x_{k+1} 열 위치에 -I 삽입
    // → "x_{k+1}을 왼쪽으로 이항했으므로 부호가 음수"
    // → A_k*x_k - 1*x_{k+1} + B_k*u_k = -c_k 에서 -1 부분

    A_mat_.block<NX, NU>(row, u_k_col)  =  Bk_seq[k];
    // u_k 열 위치에 입력 야코비안 B_k 삽입
    // → "u_k(=[Δv,Δω])가 바뀌면 동역학 제약이 B_k만큼 영향받음"
    // → B_k는 항상 [0;0;0;1;0 / 0;0;0;0;1] 형태 (Δv→v, Δω→ω)

    l_vec_.segment<NX>(row) = -ck_seq[k];  // 하한 = -c_k
    u_vec_.segment<NX>(row) = -ck_seq[k];  // 상한 = -c_k (등식)
    // c_k = f(x̄,ū) - A_k*x̄ - B_k*ū : 선형화 bias 보정 항
    // 부호 주의: 수식을 우변으로 정리하면 -c_k가 됨

    row += NX;  // 커서를 5칸 아래로 (다음 k의 동역학 제약 시작 위치)
  }


  // ----------------------------------------------------------------
  // ③ 상태 부등식: v_min ≤ v_k ≤ v_max, ω_min ≤ ω_k ≤ ω_max
  //
  // "예측된 미래 상태의 v, ω가 물리적으로 가능한 범위를
  //  벗어나면 안 된다"는 제약임.
  //
  // k=1부터 N까지 적용하는 이유:
  //   k=0은 현재 상태로 ①에서 이미 고정됨 → 여기서 또 제약 불필요
  //   k=1~N은 OSQP가 최적화할 예측 상태들 → 범위 제한 필요
  //
  // A 행렬에서 어떻게 표현하나? (k=1인 경우 예시)
  //
  //   열→  x_0열  x_1열     x_2열  ...
  //   ┌─────────────────────────────┐
  //   │  0...0 │ I(5×5) │ 0...0  │  ← 5행 (k=1 상태 제약)
  //   └─────────────────────────────┘
  //
  //   → A·z = I * x_1 = x_1  즉 x_1 자체를 그대로 뽑아냄
  //
  //   l = [-∞, -∞, -∞, v_min, ω_min]  (x,y,θ는 제한 없음)
  //   u = [+∞, +∞, +∞, v_max, ω_max]
  //
  //   l ≤ A·z ≤ u
  //   → -∞ ≤ x_k ≤ +∞  (x 위치: 어디든 OK)
  //   → -∞ ≤ y_k ≤ +∞  (y 위치: 어디든 OK)
  //   → -∞ ≤ θ_k ≤ +∞  (heading: 어디든 OK)
  //   → v_min ≤ v_k ≤ v_max   ← 이게 핵심 제약
  //   → ω_min ≤ ω_k ≤ ω_max  ← 이게 핵심 제약
  // ----------------------------------------------------------------
  for (int k = 1; k <= N; ++k) {

    int x_k_col = k * NX;  // x_k가 시작하는 열 인덱스

    A_mat_.block<NX, NX>(row, x_k_col) = StateMat::Identity();
    // x_k 열 위치에 단위행렬 삽입
    // → A·z를 계산하면 x_k 자체가 그대로 나옴
    // → 그 결과에 l ≤ ... ≤ u 적용 = x_k 원소별 범위 제한

    // 상태벡터 x_k = [x, y, θ, v, ω] 순서이므로
    // row+0=x, row+1=y, row+2=θ, row+3=v, row+4=ω 에 대응
    l_vec_(row + 0) = -1e9;             // x 위치: 하한 없음 (−∞ 대신 큰 음수)
    l_vec_(row + 1) = -1e9;             // y 위치: 하한 없음
    l_vec_(row + 2) = -1e9;             // θ heading: 하한 없음 (yaw wrap은 별도 처리)
    l_vec_(row + 3) = params_.v_min;    // v 하한: −0.5 m/s
    l_vec_(row + 4) = params_.w_min;    // ω 하한: −1.0 rad/s

    u_vec_(row + 0) =  1e9;             // x 위치: 상한 없음
    u_vec_(row + 1) =  1e9;             // y 위치: 상한 없음
    u_vec_(row + 2) =  1e9;             // θ heading: 상한 없음
    u_vec_(row + 3) = params_.v_max;    // v 상한: +0.5 m/s
    u_vec_(row + 4) = params_.w_max;    // ω 상한: +1.0 rad/s

    row += NX;  // 커서를 5칸 아래로 (다음 k의 상태 제약 시작 위치)
  }


  // ----------------------------------------------------------------
  // ④ 입력 부등식: Δv_min ≤ Δv_k ≤ Δv_max, Δω_min ≤ Δω_k ≤ Δω_max
  //
  // "각 스텝에서 속도를 너무 급격하게 바꾸면 안 된다"는 제약임.
  // 이게 smoothness를 물리적으로 보장하는 핵심 제약.
  //
  // 예: Δv_max = 0.1 이면
  //   한 스텝(20ms)에 선속도를 최대 0.1 m/s까지만 변경 가능
  //   → 0 → 0.5 m/s 도달하려면 최소 5스텝(100ms) 필요
  //   → 실제 결과: v=0.1 → 0.2 → 0.3 → 0.4 → 0.5 (부드럽게 가속)
  //
  // A 행렬에서 어떻게 표현하나? (k=0인 경우 예시)
  //
  //   열→  x_0열...x_N열  │  u_0열    u_1열  ...
  //   ┌────────────────────────────────────────┐
  //   │        0...0       │ I(2×2) │ 0...0  │  ← 2행 (k=0 입력 제약)
  //   └────────────────────────────────────────┘
  //
  //   → A·z = I * u_0 = u_0 = [Δv_0, Δω_0]  (u_0 자체를 뽑아냄)
  //
  //   l = [Δv_min, Δω_min] = [-0.1, -0.2]
  //   u = [Δv_max, Δω_max] = [+0.1, +0.2]
  //
  //   l ≤ A·z ≤ u
  //   → -0.1 ≤ Δv_k ≤ +0.1
  //   → -0.2 ≤ Δω_k ≤ +0.2
  // ----------------------------------------------------------------
  for (int k = 0; k < N; ++k) {

    int u_k_col = u_offset + k * NU;
    // u_offset = NX*(N+1) : z 벡터에서 입력 시퀀스가 시작하는 열 위치
    // u_k_col : k번째 입력 u_k = [Δv_k, Δω_k]가 시작하는 열

    A_mat_.block<NU, NU>(row, u_k_col) =
      Eigen::Matrix<double, NU, NU>::Identity();
    // u_k 열 위치에 2×2 단위행렬 삽입
    // → A·z를 계산하면 u_k = [Δv_k, Δω_k] 자체가 그대로 나옴
    // → 그 결과에 l ≤ ... ≤ u 적용 = Δv, Δω 각각 범위 제한
    // NU×NU 단위행렬을 Eigen::Matrix<double, NU, NU>로 명시한 이유:
    // StateMat::Identity()는 NX×NX(5×5)라서 크기가 다름 → 별도 선언 필요

    l_vec_(row + 0) = params_.dv_min;  // Δv 하한: −0.1 m/s
    l_vec_(row + 1) = params_.dw_min;  // Δω 하한: −0.2 rad/s
    u_vec_(row + 0) = params_.dv_max;  // Δv 상한: +0.1 m/s
    u_vec_(row + 1) = params_.dw_max;  // Δω 상한: +0.2 rad/s
    // row+0 = Δv (입력벡터 u의 0번째 원소)
    // row+1 = Δω (입력벡터 u의 1번째 원소)

    row += NU;  // 커서를 2칸 아래로 (다음 k의 입력 제약 시작 위치)
  }
  // 여기까지 오면 row = nc_ (전체 제약 수)
  // 즉 A 행렬이 빈틈 없이 꽉 채워진 상태

  // === 5. 장애물 Soft Penalty (W9 동적 장애물 CV 예측 적용) ===
  //
  // [설계 원리]
  //   Hard constraint (등식/부등식 제약)로 장애물을 막으면 QP가 infeasible해질 수 있음.
  //   대신 장애물에 가까워질수록 비용이 커지는 penalty를 q_vec에 추가 (Soft Constraint).
  //   제약으로 막는 것이 아닌 가까워질수록 비용이 비싸지게 만드는 것.
  //
  //   penalty = w * max(0, d_safe - d_eff)²
  //   → d_eff가 d_safe보다 작을 때만 penalty 발생
  //   → d_eff가 0에 가까울수록 penalty가 제곱으로 폭증 → 강하게 밀어냄
  //   d_eff   = 로봇과 장애물 표면 사이의 실제 거리
  //   d_safe  = penalty가 시작되는 안전 거리
  //   violation = d_safe - d_eff → "얼마나 안전 구역을 침범했나"
  //
  //   penalty의 x_k에 대한 gradient를 q_vec에 추가:
  //   ∂penalty/∂x_k = -2w * violation * (px - ox) / d
  //   ∂penalty/∂y_k = -2w * violation * (py - oy) / d
  //
  // [W9 time-varying 확장]
  //   장애물의 현재 위치(x,y)뿐만 아니라 현재 속도(vx,vy)를 이용하여
  //   미래 k스텝 시점의 장애물 위치를 예측하고 penalty에 반영:
  //   pred_ox = obs.x + obs.vx * (k * dt)
  //   → 동적 장애물의 미래 위치가 MPC 비용함수에 반영됨
  //
  // [전체 구조 요약]
  //   x_sim = x0  (현재 위치)
  //   for k = 1 to N:
  //     x_sim으로 k스텝 후 로봇 예측 위치 계산
  //     for each obstacle:
  //       pred_ox = obs.x + obs.vx * k * dt  ← CV 예측 (W9 핵심)
  //       d_eff = dist(x_sim, pred_obs) - obs.radius
  //       violation = d_safe - d_eff
  //       if violation > 0:
  //         grad_x, grad_y 계산 → q_vec에 추가
  //     x_sim = forward_kinematics(x_sim)  ← 다음 스텝 위치 갱신
  //
  // [정적/동적 장애물 처리 차이]
  //   is_dynamic (obs_speed > 0.15 m/s):
  //     - 뒤끝 제거 없음 (페널티 항상 적용 → 예측 궤적 토막남 방지)
  //     - params_.obs_safe_dist, params_.obs_weight 사용
  //     - grad_x *= 0.5 (연속성을 위해 x방향 반발력 완화)
  //   !is_dynamic (정적 장애물):
  //     - 뒤끝 제거: 로봇 뒤로 넘어간 장애물은 무시 (dot_product < 0)
  //     - applied_safe_dist=0.45, applied_weight=50.0 (하드코딩)
  //     - grad_x = 0.0 (브레이크 없이 측면 우회만)
  //     - grad_y *= 2.0 (측면 회피력 강화)

  // ── active obstacle selection ─────────────────────────────
  // 너무 멀거나 뒤에 있는 장애물까지 모두 penalty에 넣으면
  // MPC가 보수적으로 굳어서 정지 해를 선택하기 쉬움
  // 1. 구조체 정의
  struct ObsCandidate {
    Obstacle obs;
    double clearance;
    double forward_proj;
  };

  // 2. 후보 탐색 (obstacles_를 순회)
  std::vector<ObsCandidate> active_candidates;
  active_candidates.reserve(obstacles_.size());

  const double heading_x0 = std::cos(x0(2));
  const double heading_y0 = std::sin(x0(2));

  for (const auto & obs : obstacles_) {  // ← active_obstacles 아닌 obstacles_
    double dx = obs.x - x0(0);
    double dy = obs.y - x0(1);
    double center_dist  = std::hypot(dx, dy);
    double clearance    = center_dist - obs.radius;
    double forward_proj = dx * heading_x0 + dy * heading_y0;

    if (forward_proj < -0.05) continue;
    if (clearance > 1.2) continue;

    active_candidates.push_back({obs, clearance, forward_proj});
  }

  // 3. 가까운 순 정렬 후 최대 3개만 유지
  std::sort(active_candidates.begin(), active_candidates.end(),
    [](const ObsCandidate & a, const ObsCandidate & b) {
      return a.clearance < b.clearance;
    });

  if (active_candidates.size() > 3) {
    active_candidates.resize(3);
  }

  // 4. active_obstacles 추출
  std::vector<Obstacle> active_obstacles;
  active_obstacles.reserve(active_candidates.size());
  for (const auto & c : active_candidates) {
    active_obstacles.push_back(c.obs);
  }

  StateVec x_sim = x0;  // 현재 상태에서 rollout 시작

  for (int k = 1; k <= N; ++k) {
    // safety filter가 직전 스텝에서 v를 줄였더라도
    // MPC penalty rollout은 "계획 의도"를 볼 수 있게
    // reference 속도를 기준으로 약한 전진 가정을 둠
    double v_roll = std::clamp(x_ref[k - 1](3), 0.04, params_.v_max);
    double w_roll = std::clamp(x_ref[k - 1](4), params_.w_min, params_.w_max);

    x_sim(0) += v_roll * std::cos(x_sim(2)) * params_.dt;
    x_sim(1) += v_roll * std::sin(x_sim(2)) * params_.dt;
    x_sim(2) += w_roll * params_.dt;
    x_sim(3) = v_roll;
    x_sim(4) = w_roll;

    const double px  = x_sim(0);
    const double py  = x_sim(1);
    const double pth = x_sim(2);

    int idx = k * NX;

    for (const auto & obs : active_obstacles) {
      // 속도 폭주 방지: 추정 속도를 ±0.6 m/s로 클램핑
      // obstacle_tracker의 EMA 필터가 튀는 값을 내보낼 경우 대비
      double safe_vx = std::clamp(obs.vx, -0.6, 0.6);
      double safe_vy = std::clamp(obs.vy, -0.6, 0.6);

      // CV(Constant Velocity) 모델로 k스텝 후 장애물 위치 예측
      // t_pred = k * dt : k스텝 후의 경과 시간 [s]
      // 정적 장애물(vx=vy=0): pred = obs.x/y (변화 없음, W8 동작과 동일)
      // int pred_k = std::min(k, 5);
      // double pred_ox = obs.x + safe_vx * (pred_k * params_.dt);
      // double pred_oy = obs.y + safe_vy * (pred_k * params_.dt);

      // // 로봇(예측 위치) → 장애물(예측 위치) 벡터
      // double dx    = px - pred_ox;  // (로봇 - 장애물) x, 양수=로봇이 오른쪽
      // double dy    = py - pred_oy;  // (로봇 - 장애물) y, 양수=로봇이 위
      // double d     = std::sqrt(dx * dx + dy * dy);         // 중심 간 거리
      // double d_eff = std::max(d - obs.radius, 1e-6);       // 표면 간 거리 (반경 보정)

      // 보수적 버블 전략: CV 예측 대신 현재 위치 기준 팽창 반경 사용
      // 방향 전환 시 CV 예측이 실패하는 문제를 근본 해결
      // 버블 반경 = obs.radius + obs_speed * t_pred (최대 0.5m 팽창)
      double obs_speed_bubble = std::hypot(safe_vx, safe_vy);
      double t_pred  = std::min(k, 5) * params_.dt;
      double bubble_extra = std::min(obs_speed_bubble * t_pred, 0.5);

      double pred_ox = obs.x;  // 현재 위치 사용 (CV 예측 제거)
      double pred_oy = obs.y;

      double dx    = px - pred_ox;
      double dy    = py - pred_oy;
      double d     = std::sqrt(dx * dx + dy * dy);
      double d_eff = std::max(d - obs.radius - bubble_extra, 1e-6);

      // 동적 장애물 판별: 추정 속도가 0.15 m/s 초과이면 동적으로 처리
      double obs_speed = obs_speed_bubble;
      bool is_dynamic  = (obs_speed > 0.15);

      // ── 정적 장애물 전용: 뒤끝 페널티 제거 ──────────────────────
      // 내적(Dot Product)을 이용해 장애물이 로봇 진행 방향 뒤에 있는지 확인.
      // 로봇이 이미 장애물을 지나쳤으면 페널티를 줄 필요 없음.
      // (동적 장애물에 적용하면 예측 궤적이 토막나며 로봇이 진동함 → 적용 안 함)
      double heading_x    = std::cos(pth);           // 로봇 진행 방향 벡터 x
      double heading_y    = std::sin(pth);           // 로봇 진행 방향 벡터 y
      double vec_to_obs_x = pred_ox - px;            // 로봇→장애물 벡터 x
      double vec_to_obs_y = pred_oy - py;            // 로봇→장애물 벡터 y

      // 내적 = |heading| * |vec_to_obs| * cos(각도)
      // 음수: 장애물이 로봇 뒤에 있음 → 페널티 무시
      double dot_product = vec_to_obs_x * heading_x + vec_to_obs_y * heading_y;

      if (!is_dynamic && dot_product < 0.0) {
        continue;  // 정적 장애물이 이미 로봇 뒤 → 페널티 스킵
      }

      // 정적/동적에 따라 다른 파라미터 적용
      // 정적: 하드코딩 값 (파라미터 독립적, 보수적 튜닝)
      // 동적: params_에서 로드한 값 (런타임 조정 가능)
      double applied_safe_dist = is_dynamic ? params_.obs_safe_dist : 0.30;
      double applied_weight    = is_dynamic ? params_.obs_weight    : 20.0;

      double violation = applied_safe_dist - d_eff;
      if (violation <= 0.0) continue;  // 안전 거리 밖 → 페널티 없음

      double d_raw = std::max(d, 1e-6);  // 0 나눗셈 방지

      // Soft Penalty gradient 계산
      // ∂penalty/∂px = -2w * violation * (px - pred_ox) / d
      //              = -2w * violation * dx / d_raw
      // 부호: dx > 0 이면 grad_x < 0 → OSQP가 x_k 증가 → 로봇 오른쪽 이동
      //       (장애물이 로봇 왼쪽에 있을 때 → 로봇을 오른쪽으로 밀어냄) ✓
      double grad_x = -2.0 * applied_weight * violation * (dx / d_raw);
      double grad_y = -2.0 * applied_weight * violation * (dy / d_raw);

      if (!is_dynamic) {
        // ── 정적 장애물: 로컬 프레임(진행방향/측면) 기준으로 힘 분리 ──
        //
        // [수정 이유]
        //   기존에는 map frame x축을 "진행방향", y축을 "측면"으로 가정했음.
        //   그러나 로봇 heading(θ)이 x축이 아닌 경우(y방향 주행, 대각선 등)
        //   잘못된 방향으로 힘이 작용 → 직선 경로 이외에서는 버그 발생.
        //
        // [해결: 로컬 프레임 변환]
        //   글로벌 gradient → 로봇 헤딩 기준 로컬 프레임으로 회전 변환
        //   로컬 프레임에서 진행방향 힘 제거 / 측면 힘 강화
        //   다시 글로벌 프레임으로 역변환 → q_vec에 적용
        //
        // [변환 수식]
        //   로컬 = R(θ) * 글로벌    (R = 2D 회전 행렬)
        //   글로벌 = R(θ)ᵀ * 로컬  (R의 전치 = 역변환)
        //
        //   글로벌 → 로컬:
        //     forward =  grad_x * cos(θ) + grad_y * sin(θ)  ← 진행방향 성분
        //     lateral = -grad_x * sin(θ) + grad_y * cos(θ)  ← 측면 성분
        //
        //   로컬 → 글로벌:
        //     grad_x = forward * cos(θ) - lateral * sin(θ)
        //     grad_y = forward * sin(θ) + lateral * cos(θ)

        double cos_th = std::cos(pth);
        double sin_th = std::sin(pth);

        // 글로벌 gradient → 로컬 프레임 분해
        double grad_forward =  grad_x * cos_th + grad_y * sin_th;  // 진행방향 성분
        double grad_lateral = -grad_x * sin_th + grad_y * cos_th;  // 측면 성분

        // 로컬 프레임에서 힘 조절
        // grad_forward = std::min(grad_forward, 0.0); // 진행방향 힘 완전 제거 → 브레이크/가속 없이 전진 유지
        grad_forward = 0.0;
        grad_lateral *= 1.5; // 측면 힘 n배 강화 → 충분한 측면 우회 유도

        // 로컬 프레임 → 글로벌 프레임 역변환
        grad_x = grad_forward * cos_th - grad_lateral * sin_th;
        grad_y = grad_forward * sin_th + grad_lateral * cos_th;

      } else {
        // 동적 장애물: x방향 힘 절반으로 줄여 움직임의 연속성 확보
        // 완전히 끄면 전후 궤적 토막남 → 0.5로 완화
        // (동적 장애물은 횡단 접근이 많아 x, y 양방향 힘이 모두 필요함)
        grad_x *= 0.3;
        // grad_y는 원본값 유지 — 동적 장애물 y방향 회피력 그대로 사용
      }

      // q_vec에 gradient 누적
      // q_vec_(idx+0) += grad_x → x_k(0)의 비용함수 선형항 추가
      // q_vec_(idx+1) += grad_y → x_k(1)의 비용함수 선형항 추가
      // OSQP는 이 값을 최소화하는 방향으로 x_k를 결정함
      // → 로봇 heading 기준 측면 방향으로 자연스럽게 밀려남
      q_vec_(idx + 0) += grad_x;
      q_vec_(idx + 1) += grad_y;
    }
  }

  

  (void)u_prev;   // 향후 입력 연속성 제약 확장 시 사용 예정
}

// ============================================================
// eigenToCsc() — Eigen 밀집 행렬 → OSQP v0.6.3 CSC 변환
//
//   CSC(Compressed Sparse Column):
//     - x: 0이 아닌 원소 배열 (c_float)
//     - i: 각 원소의 행 인덱스 (c_int)
//     - p: 각 열의 시작 인덱스 (c_int, 크기 = 열 수 + 1)
//     - nz = -1: CSC 형식 표시
// ============================================================

csc * MpcCore::eigenToCsc(const Eigen::MatrixXd & M)
{
  int rows = static_cast<int>(M.rows());
  int cols = static_cast<int>(M.cols());

  // 0이 아닌 원소 수집 (열 우선 순서)
  std::vector<c_float> values;
  std::vector<c_int>   row_ind;
  std::vector<c_int>   col_ptr;

  col_ptr.push_back(0);
  for (int j = 0; j < cols; ++j) {
    for (int i = 0; i < rows; ++i) {
      if (std::abs(M(i, j)) > 1e-10) {
        values.push_back(static_cast<c_float>(M(i, j)));
        row_ind.push_back(static_cast<c_int>(i));
      }
    }
    col_ptr.push_back(static_cast<c_int>(values.size()));
  }

  int nnz = static_cast<int>(values.size());

  // OSQP csc 구조체 할당 (c_malloc 사용 — OSQP 내부 할당자와 일치)
  csc * mat = static_cast<csc *>(c_malloc(sizeof(csc)));
  mat->m     = rows;
  mat->n     = cols;
  mat->nzmax = nnz;
  mat->nz    = -1;   // -1: CSC 형식임을 나타냄

  mat->x = static_cast<c_float *>(c_malloc(nnz * sizeof(c_float)));
  mat->i = static_cast<c_int   *>(c_malloc(nnz * sizeof(c_int)));
  mat->p = static_cast<c_int   *>(c_malloc((cols + 1) * sizeof(c_int)));

  for (int k = 0; k < nnz; ++k) {
    mat->x[k] = values[k];
    mat->i[k] = row_ind[k];
  }
  for (int k = 0; k <= cols; ++k) {
    mat->p[k] = col_ptr[k];
  }

  return mat;
}

void MpcCore::freeCsc(csc * M)
{
  if (!M) { return; }
  c_free(M->x);
  c_free(M->i);
  c_free(M->p);
  c_free(M);
}

// ============================================================
// solve() — 매 제어 주기마다 호출
//
//   1. 선형화 시퀀스 계산 (A_k, B_k, c_k)
//   2. QP 행렬 조립
//   3. OSQP setup & solve
//   4. u_0 추출 및 반환
// ============================================================

MpcSolution MpcCore::solve(
  const StateVec & x0,
  const std::vector<StateVec> & x_ref,
  const InputVec & u_prev)
{
  MpcSolution sol;
  sol.success       = false;
  sol.solve_time_ms = 0.0;
  sol.cost          = 0.0;
  sol.u0.setZero();

  if (!is_initialized_) { return sol; }

  int N = params_.N;

  auto t_start = std::chrono::high_resolution_clock::now();

  // --- 1. 선형화 시퀀스 계산 ---
  // nominal input: Δu = 0 (현재 속도 유지 가정)
  std::vector<StateMat> Ak_seq(N);
  std::vector<InputMat> Bk_seq(N);
  std::vector<StateVec> ck_seq(N);
  InputVec u_nom = InputVec::Zero();

  for (int k = 0; k < N; ++k) {
    const StateVec & xbar = x_ref[k];
    linearize(xbar, u_nom, Ak_seq[k], Bk_seq[k]);

    // c_k = f(xbar, ubar) - A_k*xbar - B_k*ubar
    // 선형화 오차를 보정하는 상수 항
    StateVec f_xbar = motionModel(xbar, u_nom);
    ck_seq[k] = f_xbar - Ak_seq[k] * xbar - Bk_seq[k] * u_nom;
  }

  // --- 2. QP 행렬 조립 ---
  buildQP(x0, x_ref, u_prev, Ak_seq, Bk_seq, ck_seq);

  // --- 3. OSQP setup & solve ---
  // P는 상삼각(upper triangle)만 전달 (OSQP v0.6.3 규약)
  Eigen::MatrixXd P_upper = P_mat_.triangularView<Eigen::Upper>();

  csc * P_csc = eigenToCsc(P_upper);
  csc * A_csc = eigenToCsc(A_mat_);

  // Eigen VectorXd → c_float 배열 변환
  // OSQP v0.6.3은 c_float*(= double*) 포인터를 직접 받음
  std::vector<c_float> q_arr(nz_), l_arr(nc_), u_arr(nc_);
  for (int i = 0; i < nz_; ++i) { q_arr[i] = static_cast<c_float>(q_vec_(i)); }
  for (int i = 0; i < nc_; ++i) { l_arr[i] = static_cast<c_float>(l_vec_(i)); }
  for (int i = 0; i < nc_; ++i) { u_arr[i] = static_cast<c_float>(u_vec_(i)); }

  // 매 solve마다 workspace 재생성
  // warm_start=true로 설정했으므로 이전 해가 있으면 자동으로 초기값으로 활용
  // (workspace 재생성이지만 OSQP 내부에서 warm start 처리)
  if (work_) {
    osqp_cleanup(work_);
    work_ = nullptr;
  }

  // v0.6.3 API: osqp_setup은 OSQPData 구조체로 묶어서 전달
  // P, q, A, l, u를 개별로 받는 건 v1.x API임
  OSQPData data;
  data.n = static_cast<c_int>(nz_);   // 변수 수
  data.m = static_cast<c_int>(nc_);   // 제약 수
  data.P = P_csc;
  data.q = q_arr.data();
  data.A = A_csc;
  data.l = l_arr.data();
  data.u = u_arr.data();

  c_int exitflag = osqp_setup(&work_, &data, settings_);

  freeCsc(P_csc);
  freeCsc(A_csc);

  if (exitflag != 0) {
    return sol;
  }

  // QP 풀기
  osqp_solve(work_);

  auto t_end = std::chrono::high_resolution_clock::now();
  sol.solve_time_ms =
    std::chrono::duration<double, std::milli>(t_end - t_start).count();

  // --- 4. 결과 추출 ---
  // OSQP v0.6.3 상태 코드:
  //   1 = OSQP_SOLVED (정상 수렴)
  //   2 = OSQP_SOLVED_INACCURATE (부정확하지만 해 존재)
  if (work_->info->status_val != 1 && work_->info->status_val != 2) {
    return sol;
  }

  // z = [x_0,...,x_N, u_0,...,u_{N-1}]
  // u_0 시작 위치: NX*(N+1)
  int u0_idx = NX * (N + 1);
  sol.u0(0) = work_->solution->x[u0_idx + 0];   // Δv
  sol.u0(1) = work_->solution->x[u0_idx + 1];   // Δω
  sol.cost    = work_->info->obj_val;
  sol.success = true;

  return sol;
}

}  // namespace control_mpc