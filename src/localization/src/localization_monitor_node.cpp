#include "localization/localization_monitor_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

namespace localization
{

// 멤버변수 vs 매개변수 역할 구분
// tf_timeout_sec_   → "기준값" (파라미터, 잘 안 바뀜)
// tf_jump_thresh_m_ → "기준값" (파라미터, 잘 안 바뀜)

// tf_age_sec   → "측정값" (매 루프마다 새로 계산됨)
// tf_jump_m    → "측정값" (매 루프마다 새로 계산됨)

// ============================================================
// 생성자
//   TF 버퍼/리스너, 타이머, 구독/발행, 파라미터 초기화
// ============================================================
LocalizationMonitorNode::LocalizationMonitorNode(const rclcpp::NodeOptions & options)
: Node("localization_monitor_node", options)
{
  // --------------------------------------------------------
  // 파라미터 선언
  //
  //   tf_timeout_sec:   map→odom TF가 이 시간 이상 안 오면 LOST
  //   tf_jump_thresh_m: 한 스텝에서 이 거리 이상 점프하면 DEGRADED
  //   window_size:      RMSE 슬라이딩 윈도우 크기 (샘플 수)
  //   robot_name:       Ground Truth 추출 시 child_frame_id
  // --------------------------------------------------------
  this->declare_parameter<double>("tf_timeout_sec",    1.0);
  this->declare_parameter<double>("tf_jump_thresh_m",  0.3);
  this->declare_parameter<int>   ("window_size",       100);
  this->declare_parameter<std::string>("robot_name",  "amr_robot");

  tf_timeout_sec_   = this->get_parameter("tf_timeout_sec").as_double();
  tf_jump_thresh_m_ = this->get_parameter("tf_jump_thresh_m").as_double();
  window_size_      = this->get_parameter("window_size").as_int();
  robot_name_       = this->get_parameter("robot_name").as_string();
  // declare_parameter로 먼저 "이런 파라미터가 있다"고 ROS2에 등록하고, get_parameter로 값을 읽어서 멤버변수에 저장하는 두 단계

  // --------------------------------------------------------
  // TF 버퍼 + 리스너
  //
  //   tf2_ros::Buffer:            과거 TF를 시간 순으로 저장
  //   tf2_ros::TransformListener: /tf 토픽을 백그라운드에서 계속 구독하면서 들어오는 데이터를 Buffer에 자동으로 채워주는 역할. 
  //   이게 없으면 Buffer는 빈 상태로 유지됨
  //   10초 버퍼: 과거 10초치 TF를 보관 → 지연된 조회도 가능
  // --------------------------------------------------------
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // --------------------------------------------------------
  // QoS 설정
  // --------------------------------------------------------
  auto sensor_qos   = rclcpp::SensorDataQoS();
  auto reliable_qos = rclcpp::QoS(10).reliable();

  // --------------------------------------------------------
  // Ground Truth 구독
  //   pose_rmse_node와 동일한 토픽/방식으로 amr_robot 포즈 추출
  // --------------------------------------------------------
  gt_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    "/world/simple_room/dynamic_pose/info",
    sensor_qos,
    std::bind(&LocalizationMonitorNode::gtCallback, this, std::placeholders::_1)
  );

  // --------------------------------------------------------
  // /localization/status 발행
  //   MPC(W6~), Fail-safe SM(W10~)이 이 토픽을 구독할 예정
  // --------------------------------------------------------
  status_pub_ = this->create_publisher<amr_msgs::msg::LocalizationStatus>(
    "/localization/status",
    reliable_qos
  );

  // --------------------------------------------------------
  // 10Hz 모니터링 타이머
  //   map→odom TF 조회 + 상태 판정 + 발행을 주기적으로 수행
  //   10Hz: 빠른 변화도 잡을 수 있고 CPU 부하도 적당함
  // --------------------------------------------------------
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // 100ms = 10Hz
    std::bind(&LocalizationMonitorNode::monitorCallback, this)
  );

  RCLCPP_INFO(this->get_logger(),
    "[LocalizationMonitor] 초기화 완료. "
    "timeout=%.1fs, jump_thresh=%.2fm, window=%d",
    tf_timeout_sec_, tf_jump_thresh_m_, window_size_);
}

// ============================================================
// Ground Truth 콜백
//   TFMessage 배열에서 robot_name_ 항목만 추출해서 
// Gazebo는 월드 안의 모든 모델 위치를 하나의 배열(transforms[])에 담아서 보내. 벽, 장애물, 로봇이 전부 섞여 있음
// 그래서 배열을 순회하면서 child_frame_id == "amr_robot"인 것만 골라내는 것
// ============================================================
void LocalizationMonitorNode::gtCallback(
  const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  for (const auto & tf : msg->transforms) {
    if (tf.child_frame_id == robot_name_) {
      gt_x_ = tf.transform.translation.x;
      gt_y_ = tf.transform.translation.y;

      tf2::Quaternion q(
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w
      );
      gt_yaw_ = tf2::getYaw(q); // monitorCallback 안에서 yaw RMSE 계산에 쓰임
      has_gt_ = true;
      return;
    }
  }
}

// ============================================================
// map 좌표계 기준으로 로봇이 지금 어디 있는지 TF에서 읽어오는 함수
// map frame pose 읽기
//
//   TF 버퍼에서 map→base_link 변환을 조회함.
//
//   [왜 map→odom이 아니라 map→base_link를 읽나?]
//     map→base_link = map→odom(slam_toolbox) + odom→base_link(EKF)
//     로봇의 map 프레임 기준 절대 위치가 필요하므로
//     전체 체인의 끝인 base_link까지 조회함.
//
//   lookupTransform(target, source, time):
//     "source 프레임이 target 프레임 기준으로 어디 있나?"를 반환
//     rclcpp::Time(0) → 버퍼에서 가장 최신 변환을 조회
//
//   실패 시 false 반환 (TF 체인 미완성, 타임아웃 등)
// ============================================================
bool LocalizationMonitorNode::getMapPose(
  double & x, double & y, double & yaw)
{
  try {
    // map 프레임 기준 base_link 위치 조회
    // timeout 100ms: 이 시간 안에 TF가 없으면 예외 발생
    geometry_msgs::msg::TransformStamped tf_stamped =
      tf_buffer_->lookupTransform(
        "map",        // target frame 기준 좌표계
        "base_link",  // source frame 찾곳 싶은 것
        rclcpp::Time(0),             // 가장 최신 변환, 시간
        rclcpp::Duration::from_seconds(0.1)  // 최대 대기, 타임아웃
      );

  // target, source 개념:
  // 
  // 질문: "base_link가 map 기준으로 어디 있어?"
  // target = map      (기준, 원점)
  // source = base_link (찾고 싶은 대상)
  // 반환값 = map 원점에서 base_link까지의 변환 (위치 + 회전)

  // 왜 map→base_link를 한번에 조회하나?
  // 
  // TF 트리 구조:
  // map ──(slam_toolbox)──▶ odom ──(EKF)──▶ base_link

  // lookupTransform("map", "base_link")은
  // 이 두 변환을 자동으로 합쳐서 반환해줌.
  // 직접 map→odom, odom→base_link를 따로 읽고 더할 필요가 없음.

  // `rclcpp::Time(0)` 의미:
  // 
  // Time(0) = "가장 최신 변환을 줘"
  // 특정 시간을 지정하면 그 시점의 TF를 조회하는데,
  // 0으로 하면 버퍼에서 제일 최근 것을 꺼내옴.

  // 타임아웃 0.1초 의미:
  // 
  // TF가 아직 버퍼에 없을 때 최대 0.1초 기다림.
  // 0.1초 안에 안 오면 TransformException 발생 → catch로 넘어감.

    x = tf_stamped.transform.translation.x;
    y = tf_stamped.transform.translation.y;

    tf2::Quaternion q(
      tf_stamped.transform.rotation.x,
      tf_stamped.transform.rotation.y,
      tf_stamped.transform.rotation.z,
      tf_stamped.transform.rotation.w
    );
    yaw = tf2::getYaw(q);
    // gt_yaw_는 클래스 멤버변수라서 같은 클래스의 모든 메서드가 공유. 그림으로 보면:

    // LocalizationMonitorNode 객체 (메모리에 하나 존재)
    // ├── gt_x_   = 1.23   ← gtCallback이 씀
    // ├── gt_y_   = 0.45   ← gtCallback이 씀
    // ├── gt_yaw_ = 0.78   ← gtCallback이 씀, monitorCallback이 읽음
    // ├── has_gt_ = true
    // ├── prev_map_x_ ...
    // └── ...

    // gtCallback()      → gt_yaw_ 에 값 저장
    // monitorCallback() → gt_yaw_ 에서 값 읽기
    // 둘 다 같은 객체의 멤버변수를 보고 있으니까 반환 없이도 공유가 되는 것. W2에서 ekf_core.cpp의 update()가 x_를 반환 없이 수정하고 publishEkfOdom()에서 읽던 것과 완전히 같은 원리.

    // 마지막 정상 TF 수신 시간 갱신
    last_tf_time_      = this->now();
    has_last_tf_time_  = true;

    return true;

  } catch (const tf2::TransformException & ex) {
    // TF 조회 실패 — 체인 미완성 or 타임아웃
    // RCLCPP_WARN은 너무 자주 찍히면 노이즈가 되므로 throttle 적용
    // 3000ms마다 한 번만 출력
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "[LocalizationMonitor] TF 조회 실패: %s", ex.what());
    return false;
  }
}

// ============================================================
// 상태 판정
//
//   우선순위: LOST > DEGRADED > NORMAL
//
//   LOST:     TF 타임아웃 (tf_age_sec > tf_timeout_sec_)
//             → slam_toolbox가 localization을 완전히 잃음
//
//   DEGRADED: TF 점프 감지 (tf_jump_m > tf_jump_thresh_m_)
//             → 갑작스러운 위치 보정 = 스캔 매칭 불안정
//
//   NORMAL:   위 조건 모두 해당 없음
// ============================================================
uint8_t LocalizationMonitorNode::evaluateStatus(
  double tf_age_sec, double tf_jump_m)
{
  // 상태 상수 — LocalizationStatus.msg의 status 필드값과 일치
  constexpr uint8_t NORMAL   = 0;
  constexpr uint8_t DEGRADED = 1;
  constexpr uint8_t LOST     = 2;

  if (tf_age_sec > tf_timeout_sec_) {
    return LOST;
  }
  if (tf_jump_m > tf_jump_thresh_m_) {
    return DEGRADED;
  }
  return NORMAL;
}

// ============================================================
// 메인 모니터링 루프 (10Hz)
//
//   [처리 흐름]
//     1. map→base_link TF 조회
//     2. TF 경과 시간(tf_age_sec) 계산
//     3. 이전 위치 대비 점프 크기(tf_jump_m) 계산
//     4. RMSE 슬라이딩 윈도우 갱신 (Ground Truth 있을 때만)
//     5. 상태 판정 (NORMAL / DEGRADED / LOST)
//     6. /localization/status 발행
// ============================================================
void LocalizationMonitorNode::monitorCallback()
{
  amr_msgs::msg::LocalizationStatus status_msg;
  status_msg.header.stamp    = this->now();
  status_msg.header.frame_id = "map";

  // ---- 1. map frame pose 읽기 ----
  // TF 버퍼에서 map→base_link 변환을 조회해서 로봇의 map 프레임 기준 절대 위치를 가져옴. 
  // slam_toolbox가 살아있으면 true, TF 체인이 끊겼으면 false 반환.
  double map_x = 0.0, map_y = 0.0, map_yaw = 0.0;
  const bool tf_ok = getMapPose(map_x, map_y, map_yaw);

  // ---- 2. TF 경과 시간 계산 ----
  double tf_age_sec = 0.0;
  if (!tf_ok) {
    // TF 조회 실패 — 마지막 정상 TF 이후 경과 시간 계산
    if (has_last_tf_time_) {
      tf_age_sec = (this->now() - last_tf_time_).seconds();
    } else {
      // 한 번도 TF를 받지 못한 초기 상태
      // timeout보다 크게 설정해서 즉시 LOST로 판정하지 않도록
      // (노드 시작 직후 slam_toolbox가 아직 준비 중일 수 있음)
      tf_age_sec = 0.0;
    }
  }
  // tf_ok이면 tf_age_sec = 0 (방금 정상 수신)

  // ---- 3. 점프 크기 계산 ----
  // 이전 스텝의 위치와 현재 위치의 유클리드 거리를 계산함. 100ms 안에 0.3m 이상 순간이동하면 slam_toolbox가 스캔 매칭을 갑자기 크게 보정했다는 신호 → DEGRADED 판정.
  // 정상 주행 시 로봇 최대 속도 0.5m/s 기준 100ms동안 최대 이동거리는 0.05m라서, 0.3m 임계값이면 정상 주행과 구분 가능.

  double tf_jump_m = 0.0;
  if (tf_ok) {
    if (has_prev_tf_) {
      const double dx = map_x - prev_map_x_;
      const double dy = map_y - prev_map_y_;
      tf_jump_m = std::sqrt(dx * dx + dy * dy);
    }
    // 현재 위치를 다음 스텝의 이전 위치로 저장
    prev_map_x_  = map_x;
    prev_map_y_  = map_y;
    has_prev_tf_ = true;
  }
  // 왜 if (has_prev_tf_) 또 감싸는지
  // 점프는 "이전 위치 대비 현재 위치" 비교인데, 노드가 시작되고 첫 번째 스텝에서는 비교할 이전 위치가 없음
  // 그래서 has_prev_tf_ = false인 첫 스텝은 점프 계산을 건너뛰고, tf_jump_m = 0.0으로 유지함.
  //   스텝 1 (t=0.0s):
  //   has_prev_tf_ = false
  //   → 점프 계산 건너뜀, tf_jump_m = 0.0
  //   → prev_map_x_ = 1.5 저장, has_prev_tf_ = true

  // 스텝 2 (t=0.1s):
  //   has_prev_tf_ = true
  //   map_x = 1.53  (정상 이동, 0.5m/s * 0.1s = 0.05m)
  //   dx = 1.53 - 1.50 = 0.03m  → tf_jump_m = 0.03m  → NORMAL
  //   → prev_map_x_ = 1.53 저장

  // 스텝 3 (t=0.2s):
  //   has_prev_tf_ = true
  //   map_x = 1.85  (갑자기 0.32m 점프!)
  //   dx = 1.85 - 1.53 = 0.32m  → tf_jump_m = 0.32m  → DEGRADED (0.3m 초과)
  //   → prev_map_x_ = 1.85 저장

  // ---- 4. RMSE 슬라이딩 윈도우 갱신 ----
  // has_gt_: Ground Truth(Gazebr가 알고있는 실제 위치)를 최소 한 번 이상 받았을 때
  if (tf_ok && has_gt_) {
    // Ground Truth vs map frame pose 오차
    // Ground Truth(Gazebo가 알고 있는 실제 위치)에서 map frame pose(slam_toolbox가 추정한 위치)를 뺀 값.
    const double err_x = gt_x_ - map_x;
    const double err_y = gt_y_ - map_y;

    const double raw_diff = gt_yaw_ - map_yaw;
    const double err_yaw  = std::atan2(
      std::sin(raw_diff), std::cos(raw_diff));

    // 슬라이딩 윈도우에 오차² 추가
    auto pushWindow = [&](std::deque<double> & buf, double err_sq) {
      buf.push_back(err_sq);
      if (static_cast<int>(buf.size()) > window_size_) {
        buf.pop_front();
      }
    };

    pushWindow(buf_x_,   err_x   * err_x);
    pushWindow(buf_y_,   err_y   * err_y);
    pushWindow(buf_yaw_, err_yaw * err_yaw);

    // RMSE 계산
    // 람다 표현, 일반함수로 쓴다면 double calcRmse(const td::deque<double> & buf) {함수본문}
    // 문법 4조각 분해
    //
    // auto calcRmse = [](const std::deque<double> & buf) -> double { ... };
    // ①              ② ③                                 ④
    // ① auto calcRmse 람다를 담는 변수. 타입을 직접 쓰기엔 너무 복잡해서 auto로 컴파일러한테 타입 추론을 맡김.
    //
    // ② [] — 캡처 외부 변수를 가져올지 말지 선언하는 곳.
    //
    // []   // 아무것도 캡처 안 함 → 외부 변수 접근 불가
    // [&]  // 외부 변수 전부 참조로 캡처
    // [=]  // 외부 변수 전부 복사로 캡처
    // calcRmse는 buf만 매개변수로 받고 외부 변수를 쓰지 않아서 []으로 충분함. 반면 위의 pushWindow는 window_size_(클래스 멤버변수)를 써야 해서 [&]를 씀.
    //
    // ③ (const std::deque<double> & buf) — 매개변수 일반 함수 매개변수와 완전히 동일.
    //
    // const → 함수 안에서 buf 내용을 수정 못 함 (읽기 전용)
    // & → 참조로 받음, 복사 없이 원본을 직접 읽음 (deque가 클 수 있으니 복사 비용 절약)
    // ④ -> double — 반환 타입 명시 "이 람다는 double을 반환한다"고 명시. 생략해도 컴파일러가 추론하지만 명시하면 가독성이 좋아짐.
    
    auto calcRmse = [](const std::deque<double> & buf) -> double {
      if (buf.empty()) { return 0.0; }
      double sum = 0.0;
      for (const auto & v : buf) { sum += v; }
      return std::sqrt(sum / static_cast<double>(buf.size()));
    };

    status_msg.rmse_x   = calcRmse(buf_x_);
    status_msg.rmse_y   = calcRmse(buf_y_);
    status_msg.rmse_yaw = calcRmse(buf_yaw_);
    status_msg.rmse_total = std::sqrt(
      status_msg.rmse_x * status_msg.rmse_x +
      status_msg.rmse_y * status_msg.rmse_y
    );
  }

  // ---- 5. 상태 판정 ----
  const uint8_t status = evaluateStatus(tf_age_sec, tf_jump_m);

  status_msg.status       = status;
  status_msg.is_localized = (status != 2);  // LOST가 아니면 true
  status_msg.tf_jump_m    = tf_jump_m;
  status_msg.tf_age_sec   = tf_age_sec;

  // ---- 6. 발행 ----
  status_pub_->publish(status_msg);

  // 상태 변화 또는 5초마다 로그 출력
  static uint8_t prev_status = 0;
  static int     log_counter = 0;
  log_counter++;

  const bool status_changed = (status != prev_status);
  const bool periodic_log   = (log_counter % 50 == 0);  // 50 * 100ms = 5초

  if (status_changed || periodic_log) {
    const char * status_str[] = {"NORMAL", "DEGRADED", "LOST"};
    RCLCPP_INFO(this->get_logger(),
      "[LocalizationMonitor] status=%s | "
      "RMSE(x=%.3f y=%.3f yaw=%.3f total=%.3f) | "
      "jump=%.3fm age=%.2fs",
      status_str[status],
      status_msg.rmse_x, status_msg.rmse_y,
      status_msg.rmse_yaw, status_msg.rmse_total,
      tf_jump_m, tf_age_sec);

    prev_status = status;
  }
}

}  // namespace localization

// ============================================================
// main
// ============================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<localization::LocalizationMonitorNode>());
  rclcpp::shutdown();
  return 0;
}