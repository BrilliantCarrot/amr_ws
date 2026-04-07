// ============================================================
// obstacle_tracker_node.cpp — W9 동적 장애물 추적노드
// LiDAR의 /sacn 데이터를 받아 인접한 점들을 묶고(clusterging), 이전 프레임의 군집과 비교하여
// 속도(Tracking)을 추정하는 노드. C++의 std::vector와 유클리디안 거리 공식만을 사용.
//
// [전체 처리 흐름 요약]
//
//   /scan (LiDAR 원시 데이터, laser_link 좌표계)
//     ↓ ① TF2 변환
//   map frame 기준 2D 포인트 배열
//     ↓ ② Euclidean Clustering
//   클러스터 목록 (연속된 점들의 묶음)
//     ↓ ③ Tracking + LPF(저역통과필터)
//   이전 프레임과 매칭 → 속도 추정 → 노이즈 완화
//     ↓ ④ 발행
//   /obstacles/detected (ObstacleArray, map frame)
//
// [왜 map frame으로 변환하나?]
//   LiDAR는 laser_link 좌표계(로봇 시점)에서 데이터를 냄.
//   MPC의 waypoint와 제어 로직은 map frame(전역 좌표계) 기준.
//   → 비교/연산을 위해 반드시 같은 좌표계로 맞춰야 함.
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <amr_msgs/msg/obstacle_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>

namespace control_mpc {

// ============================================================
// TrackedObs 구조체 — 하나의 추적 중인 장애물 정보를 담는 컨테이너
//
// 장애물 하나당 이 구조체 1개가 할당됨.
// tracked_obstacles_ 벡터에 담겨 프레임 간 상태를 유지함.
// ============================================================
struct TrackedObs {
  int id;               // 장애물 고유 ID (새로 발견될 때마다 next_id_++ 로 부여)
  double x, y;          // map frame 기준 장애물 중심 위치 [m]
                        // EMA 필터가 적용된 "부드러운" 위치값
  double vx, vy;        // 추정 속도 [m/s]
                        // 이전 위치와 현재 위치의 차분(Δx/Δt)으로 계산
                        // EMA 필터 적용으로 급격한 튀는 값 억제
  double radius;        // 클러스터의 추정 반경 [m]
                        // centroid에서 가장 먼 포인트까지의 거리
  rclcpp::Time last_seen; // 이 장애물이 마지막으로 감지된 시각
                          // 속도 추정 시 Δt 계산에 사용
};

// ============================================================
// ObstacleTrackerNode 클래스
//
// [상속 구조]
//   rclcpp::Node를 상속받아 ROS2 노드로 동작.
//   생성자에서 구독/발행자를 설정하고,
//   scanCallback()이 LiDAR 데이터 수신마다 자동 호출됨.
// ============================================================
class ObstacleTrackerNode : public rclcpp::Node {
public:
  // ──────────────────────────────────────────────────────────
  // 생성자 — 노드 초기화
  //
  // [초기화 항목]
  //   1. tf_buffer_ / tf_listener_ : TF2 변환 시스템 초기화
  //      "laser_link → map" 변환을 실시간으로 받아오기 위해 필요.
  //   2. scan_sub_ : /scan 구독자
  //   3. obs_pub_ : /obstacles/detected 발행자
  // ──────────────────────────────────────────────────────────
  ObstacleTrackerNode()
  : Node("obstacle_tracker_node"),
    tf_buffer_(this->get_clock()),   // TF2 버퍼: 노드 시계 기준으로 초기화
    tf_listener_(tf_buffer_)         // TF2 리스너: 버퍼에 변환 정보를 자동으로 채워줌
  {
    // [QoS 설정] SensorDataQoS 사용 이유:
    //   LiDAR 같은 센서 데이터는 BEST_EFFORT + 소규모 히스토리가 적합.
    //   RELIABLE로 설정하면 네트워크 지연 시 구버전 데이터가 쌓여 실시간성이 깨짐.
    //   Gazebo의 /scan 브릿지도 BEST_EFFORT로 발행하므로 QoS를 맞춰야 연결됨.
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&ObstacleTrackerNode::scanCallback, this, std::placeholders::_1)
    );

    obs_pub_ = this->create_publisher<amr_msgs::msg::ObstacleArray>("/obstacles/detected", 10);

    RCLCPP_INFO(this->get_logger(), "W9 Obstacle Tracker Node가 시작되었습니다. /scan 구독 중...");
  }

private:
  // ──────────────────────────────────────────────────────────
  // scanCallback() — LiDAR 데이터 수신마다 호출되는 메인 콜백 (10Hz)
  //
  // [처리 순서]
  //   1. TF2로 laser_link → map 변환 행렬 획득
  //   2. 유효한 scan 포인트를 map frame으로 변환 + 필터링
  //   3. Euclidean Clustering으로 장애물 후보 클러스터 추출
  //   4. 이전 프레임 장애물과 매칭 → EMA 필터 → 속도 추정
  //   5. ObstacleArray 메시지 발행
  // ──────────────────────────────────────────────────────────
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // 1초마다 노드 생존 로그 출력 (터미널에서 정상 동작 여부 확인용)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "LiDAR 데이터 수신 및 처리 중...");

    rclcpp::Time current_time = msg->header.stamp;

    // ── STEP 1: TF2 변환 행렬 획득 ────────────────────────────
    // [목적] laser_link 좌표계의 포인트를 map 좌표계로 변환하기 위한 행렬을 가져옴.
    //
    // lookupTransform("map", "laser_link", tf2::TimePointZero)의 의미:
    //   "map 좌표계에서 laser_link 좌표계로의 변환을 알려줘"
    //   즉, laser_link 기준 포인트를 map 기준으로 바꾸는 행렬.
    //   tf2::TimePointZero = "가장 최신 TF를 써라" (시간 동기화 없이)
    //
    // [예외 처리] slam_toolbox가 아직 map frame을 발행하지 않았거나
    //   초기화 중이면 TF를 못 찾아 예외가 발생 → warn 출력 후 이번 프레임 스킵.
    geometry_msgs::msg::TransformStamped tf_laser_to_map;
    try {
      tf_laser_to_map = tf_buffer_.lookupTransform(
        "map",                   // 목적 좌표계 (변환 후)
        msg->header.frame_id,    // 원본 좌표계 (= "laser_link")
        tf2::TimePointZero       // 가장 최신 TF 사용
      );
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "TF 대기 중 (map frame이 필요합니다): %s", ex.what());
      return;  // 이번 scan 프레임은 처리하지 않고 다음 콜백을 기다림
    }

    // ── STEP 2: scan 포인트 → map frame 변환 + 필터링 ─────────
    //
    // LiDAR scan 데이터 구조:
    //   ranges[i] = i번째 각도 방향의 거리 [m]
    //   angle = angle_min + i * angle_increment [rad]
    //   → 극좌표(r, angle)를 직교좌표(x, y)로 변환 후 TF2 적용
    //
    // 필터링 조건:
    //   - inf/nan: 반사 없음(유리면 등) → 제거
    //   - range_min 미만: 로봇 자체 반사 → 제거
    //   - range_max 초과: 너무 먼 거리 → 제거
    //   - 방 경계(2.1m) 초과: 벽 포인트 → 제거
    //     (simple_room은 5×5m이므로 중심에서 최대 2.5m, 벽 제거를 위해 2.1m로 제한)
    std::vector<std::pair<double, double>> valid_points;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double r = msg->ranges[i];

      // 유효하지 않은 range 값 제거
      if (std::isinf(r) || std::isnan(r) || r < msg->range_min || r > msg->range_max) continue;

      // 극좌표 → laser_link frame 직교좌표
      double angle = msg->angle_min + i * msg->angle_increment;
      geometry_msgs::msg::PointStamped pt_laser, pt_map;
      pt_laser.point.x = r * std::cos(angle);  // lx = r * cos(θ)
      pt_laser.point.y = r * std::sin(angle);  // ly = r * sin(θ)

      // tf2::doTransform: laser_link frame 포인트 → map frame 포인트로 변환
      // 내부적으로 회전행렬 + 평행이동 계산 (쿼터니언 → 회전 행렬 변환 포함)
      tf2::doTransform(pt_laser, pt_map, tf_laser_to_map);

      // 방 경계 밖 포인트 제거 (벽 클러스터링 방지)
      // simple_room 크기 기준 ±2.1m 범위 내의 포인트만 사용
      if (std::abs(pt_map.point.x) > 2.1 || std::abs(pt_map.point.y) > 2.1) continue;

      valid_points.push_back({pt_map.point.x, pt_map.point.y});
    }

    // ── STEP 3: Euclidean Clustering ──────────────────────────
    //
    // [알고리즘 개념]
    //   연속된 scan 포인트들 중 인접한 것끼리 같은 클러스터로 묶음.
    //   scan 데이터는 각도 순서로 정렬되어 있으므로,
    //   인접 포인트끼리 거리가 가까우면 같은 물체일 가능성이 높음.
    //
    // [한계]
    //   이 방식은 "마지막 추가된 포인트"와만 비교하므로
    //   정확한 Euclidean clustering(BFS 방식)보다 단순하지만 속도가 빠름.
    //   단, 클러스터가 구부러진 형태면 잘못 쪼개질 수 있음.
    //
    // [cluster_tol = 0.25m]
    //   이 거리보다 가까운 연속 포인트는 같은 클러스터로 묶음.
    //   너무 크면 서로 다른 장애물이 합쳐짐.
    //   너무 작으면 장애물이 여러 조각으로 분리됨.
    std::vector<std::vector<std::pair<double, double>>> clusters;
    double cluster_tol = 0.25;  // 클러스터 묶음 거리 임계값 [m]

    for (const auto& pt : valid_points) {
      if (clusters.empty()) {
        // 첫 번째 포인트: 새 클러스터 시작
        clusters.push_back({pt});
      } else {
        auto& last_c = clusters.back();  // 현재까지 마지막으로 만들어진 클러스터
        double dist = std::hypot(
          pt.first  - last_c.back().first,   // Δx
          pt.second - last_c.back().second    // Δy
        );

        if (dist < cluster_tol) {
          // 가까우면 현재 클러스터에 추가
          last_c.push_back(pt);
        } else {
          // 멀면 새 클러스터 시작
          clusters.push_back({pt});
        }
      }
    }

    // ── STEP 4: Tracking + LPF(저역통과필터) + 속도 추정 ──────
    //
    // [Tracking이 필요한 이유]
    //   클러스터링만 하면 프레임마다 새 장애물 목록이 생성됨.
    //   → 속도를 추정할 수 없고, 노이즈에 취약함.
    //   이전 프레임의 같은 장애물과 매칭하면:
    //   ① 속도 추정: Δposition / Δtime
    //   ② 노이즈 억제: EMA(지수이동평균) 필터 적용
    //
    // [EMA 필터 (지수이동평균, Exponential Moving Average)]
    //   smoothed = (1-α) * 이전값 + α * 새 관측값
    //   α = 0.2 → 새 관측값을 20%만 반영, 이전 값을 80% 유지
    //   → 1프레임에서 급격히 튀는 값을 억제 (저역통과필터 효과)
    //   α가 클수록: 최신 값에 빠르게 반응, 노이즈 민감
    //   α가 작을수록: 부드럽지만 실제 변화 반응이 느림
    //
    // [매칭 방식: 최근접 이웃 (Nearest Neighbor)]
    //   새 클러스터의 centroid와 이전 프레임 장애물 목록을 비교.
    //   거리가 0.8m 이내인 가장 가까운 이전 장애물 = 같은 장애물로 판단.
    std::vector<TrackedObs> current_detected;

    for (const auto& cluster : clusters) {
      // [노이즈 필터링] 포인트 수 7개 미만 클러스터는 노이즈로 판단하고 제거.
      // 실제 장애물(원기둥 r≈0.15m)은 10Hz scan에서 최소 7~10개 포인트가 잡힘.
      if (cluster.size() < 7) continue;

      // centroid 계산: 클러스터 내 모든 포인트의 좌표 평균
      double cx = 0, cy = 0;
      for (const auto& p : cluster) { cx += p.first; cy += p.second; }
      cx /= cluster.size();
      cy /= cluster.size();

      // radius 계산: centroid에서 가장 먼 포인트까지의 거리
      // 이 값이 장애물의 "크기(반경)"에 대한 추정값이 됨
      double max_d = 0;
      for (const auto& p : cluster)
        max_d = std::max(max_d, std::hypot(p.first - cx, p.second - cy));
      double radius = max_d;

      // ── 이전 프레임 장애물과 매칭 ──────────────────────────────
      // 현재 클러스터(cx, cy)와 가장 가까운 이전 장애물 탐색
      int matched_idx = -1;
      double min_match_dist = 0.8;  // 매칭 허용 최대 거리 [m]
                                    // 이보다 멀면 새 장애물로 판단

      for (size_t i = 0; i < tracked_obstacles_.size(); ++i) {
        double d = std::hypot(
          cx - tracked_obstacles_[i].x,
          cy - tracked_obstacles_[i].y
        );
        // 가장 가까운 이전 장애물 탐색 (greedy 최근접)
        if (d < min_match_dist) {
          min_match_dist = d;
          matched_idx = i;
        }
      }

      TrackedObs obs;
      obs.last_seen = current_time;

      if (matched_idx != -1) {
        // ── 매칭 성공: 이전 장애물과 동일 → EMA 필터 + 속도 추정 ──
        double alpha = 0.2;  // EMA 필터 계수 (0~1, 클수록 최신값 강조)

        obs.id = tracked_obstacles_[matched_idx].id;  // 기존 ID 유지

        // EMA 위치 필터: 급격한 위치 튐 억제
        // 이전 위치를 80% 유지하고 새 관측을 20% 반영
        obs.x = (1.0 - alpha) * tracked_obstacles_[matched_idx].x + alpha * cx;
        obs.y = (1.0 - alpha) * tracked_obstacles_[matched_idx].y + alpha * cy;

        // EMA 반경 필터: 반경 추정도 부드럽게 유지
        obs.radius = (1.0 - alpha) * tracked_obstacles_[matched_idx].radius + alpha * radius;

        // 속도 추정: 1차 차분 (Δposition / Δtime)
        // dt: 이전 프레임과 현재 프레임 사이의 실제 시간 간격
        // 10Hz scan이면 dt ≈ 0.1s
        double dt = (current_time - tracked_obstacles_[matched_idx].last_seen).seconds();
        if (dt > 0) {
          // raw 속도: 필터 적용 전 "순수 차분" 속도
          double rx = (obs.x - tracked_obstacles_[matched_idx].x) / dt;  // raw vx [m/s]
          double ry = (obs.y - tracked_obstacles_[matched_idx].y) / dt;  // raw vy [m/s]

          // EMA 속도 필터: 이전 속도를 80% 유지 + 새 속도 20% 반영
          // 속도는 위치보다 노이즈에 훨씬 민감하므로 강하게 평활화
          obs.vx = (1.0 - alpha) * tracked_obstacles_[matched_idx].vx + alpha * rx;
          obs.vy = (1.0 - alpha) * tracked_obstacles_[matched_idx].vy + alpha * ry;
        } else {
          // dt = 0이면 속도 계산 불가 → 이전 속도 유지
          obs.vx = tracked_obstacles_[matched_idx].vx;
          obs.vy = tracked_obstacles_[matched_idx].vy;
        }

      } else {
        // ── 매칭 실패: 새로 등장한 장애물 ────────────────────────
        // 첫 감지 시 속도를 알 수 없으므로 0으로 초기화
        // 다음 프레임부터 매칭되면 속도 추정 시작
        obs.id     = next_id_++;  // 새 ID 부여
        obs.x      = cx;
        obs.y      = cy;
        obs.radius = radius;
        obs.vx     = 0.0;  // 첫 감지: 속도 불명 → 0으로 시작
        obs.vy     = 0.0;
      }

      current_detected.push_back(obs);
    }

    // 현재 프레임 결과를 멤버 변수에 저장 (다음 프레임의 "이전 프레임" 역할)
    tracked_obstacles_ = current_detected;

    // ── STEP 5: ObstacleArray 메시지 조립 및 발행 ──────────────
    //
    // amr_msgs::ObstacleArray 구조:
    //   header: 타임스탬프 + 좌표계 ("map")
    //   count:  감지된 장애물 수
    //   x[], y[]: centroid 위치 배열 (map frame)
    //   vx[], vy[]: 속도 배열 [m/s]
    //   radius[]: 추정 반경 배열 [m]
    //
    // MpcNode가 이 토픽을 구독해서 obstacle_tracker → MPC 연결
    amr_msgs::msg::ObstacleArray out_msg;
    out_msg.header.stamp    = current_time;
    out_msg.header.frame_id = "map";           // MPC가 map frame 기준으로 동작함
    out_msg.count           = tracked_obstacles_.size();

    for (const auto& o : tracked_obstacles_) {
      out_msg.x.push_back(o.x);
      out_msg.y.push_back(o.y);
      out_msg.vx.push_back(o.vx);
      out_msg.vy.push_back(o.vy);
      out_msg.radius.push_back(o.radius);
    }

    obs_pub_->publish(out_msg);
  }

  // ── 멤버 변수 ─────────────────────────────────────────────
  tf2_ros::Buffer              tf_buffer_;          // TF2 변환 정보를 저장하는 버퍼
                                                    // lookupTransform()이 이 버퍼에서 읽음
  tf2_ros::TransformListener   tf_listener_;        // TF 토픽을 구독해 tf_buffer_를 자동으로 갱신
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;  // /scan 구독자
  rclcpp::Publisher<amr_msgs::msg::ObstacleArray>::SharedPtr   obs_pub_;   // /obstacles/detected 발행자

  std::vector<TrackedObs> tracked_obstacles_;  // 현재 추적 중인 장애물 목록
                                               // 프레임 간 상태 유지용 (멤버 변수라 콜백 간 살아남음)

  int next_id_ = 0;  // 장애물 ID 카운터 (새 장애물이 등장할 때마다 1씩 증가)
                     // 디버깅 시 특정 장애물을 식별하는 데 사용
};

}  // namespace control_mpc

// ============================================================
// main() — 노드 진입점
//
// rclcpp::spin(): 무한 루프로 콜백 대기
//   → /scan 데이터가 올 때마다 scanCallback() 자동 호출
//   → Ctrl+C 시 spin() 탈출 → shutdown()
// ============================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control_mpc::ObstacleTrackerNode>());
  rclcpp::shutdown();
  return 0;
}