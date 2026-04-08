// ============================================================
// obstacle_tracker_node.cpp — W9 동적 장애물 추적 노드
//
// [전체 처리 흐름]
//   /scan (LiDAR 원시 데이터, laser_link 좌표계)
//     ↓ TF2 변환 (laser_link → map frame)
//   map frame 기준 2D 포인트 배열
//     ↓ Euclidean Clustering
//   클러스터 목록 (연속된 포인트의 묶음)
//     ↓ Tracking + EMA 필터
//   이전 프레임 매칭 → 속도 추정 → 노이즈 완화
//     ↓ 발행
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
// TrackedObs 구조체 — 추적 중인 장애물 하나의 상태를 담는 컨테이너
//
// tracked_obstacles_ 벡터에 프레임 간 보존되며,
// 이전 프레임과 매칭하여 속도 추정 및 EMA 필터 적용에 사용됨.
// ============================================================
struct TrackedObs {
  int id;                  // 장애물 고유 ID (새로 감지될 때마다 next_id_++ 로 부여)
  double x, y;             // map frame 기준 centroid 위치 [m] (EMA 필터 적용됨)
  double vx, vy;           // 추정 속도 [m/s] (1차 차분 + EMA 필터 적용됨)
  double radius;           // 클러스터 추정 반경 [m] (centroid에서 가장 먼 포인트까지 거리)
  rclcpp::Time last_seen;  // 마지막 감지 시각 (속도 추정 시 Δt 계산에 사용)
};

// ============================================================
// ObstacleTrackerNode — LiDAR 기반 동적 장애물 추적 노드
//
// rclcpp::Node를 상속받아 ROS2 노드로 동작.
// 생성자에서 구독/발행자를 초기화하고,
// /scan 수신마다 scanCallback()이 자동으로 호출됨.
// ============================================================
class ObstacleTrackerNode : public rclcpp::Node {
public:
  // ──────────────────────────────────────────────────────────
  // 생성자 — 노드 초기화
  //
  // tf_buffer_ : TF2 변환 정보를 저장하는 버퍼
  //              lookupTransform()이 이 버퍼에서 변환 행렬을 읽음
  // tf_listener_: /tf, /tf_static 토픽을 구독해 tf_buffer_를 자동으로 채움
  // ──────────────────────────────────────────────────────────
  ObstacleTrackerNode() : Node("obstacle_tracker_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    // [설정] QoS는 LiDAR 센서 데이터에 맞춰 SensorDataQoS를 사용합니다.
    // SensorDataQoS = BEST_EFFORT + 소규모 히스토리
    // Gazebo의 /scan 브릿지도 BEST_EFFORT로 발행하므로 QoS를 맞춰야 연결됨.
    // RELIABLE로 설정하면 송신측과 QoS 불일치로 데이터를 수신하지 못함.
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&ObstacleTrackerNode::scanCallback, this, std::placeholders::_1));
    
    obs_pub_ = this->create_publisher<amr_msgs::msg::ObstacleArray>("/obstacles/detected", 10);
    
    RCLCPP_INFO(this->get_logger(), "W9 Obstacle Tracker Node가 시작되었습니다. /scan 구독 중...");
  }

private:
  // ──────────────────────────────────────────────────────────
  // scanCallback() — /scan 수신마다 호출되는 메인 콜백 (10Hz)
  //
  // [처리 순서]
  //   1. TF2로 laser_link → map 변환 행렬 획득
  //   2. 유효한 scan 포인트를 map frame으로 변환 + 필터링
  //   3. Euclidean Clustering으로 장애물 클러스터 추출
  //   4. 이전 프레임 장애물과 매칭 → EMA 필터 → 속도 추정
  //   5. ObstacleArray 메시지 발행
  // ──────────────────────────────────────────────────────────
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // 1초마다 노드 생존 신고 로그를 출력합니다.
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "LiDAR 데이터 수신 및 처리 중...");

    rclcpp::Time current_time = msg->header.stamp;
    geometry_msgs::msg::TransformStamped tf_laser_to_map;
    try {
      // laser_link에서 map 좌표계로의 변환 정보를 가져옵니다.
      // lookupTransform("map", frame_id, ...) 의미:
      //   frame_id(=laser_link) 기준 포인트를 map 기준으로 변환하는 행렬을 반환.
      // tf2::TimePointZero = "가장 최신 TF를 써라" (시간 동기화 없이)
      tf_laser_to_map = tf_buffer_.lookupTransform("map", msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      // slam_toolbox 초기화 중이거나 map frame이 아직 발행 안 된 경우.
      // 이번 프레임은 건너뛰고 다음 콜백을 기다림.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF 대기 중 (map frame이 필요합니다): %s", ex.what());
      return; 
    }

    // ── STEP 1: scan 포인트 → map frame 변환 + 필터링 ─────────
    // LiDAR scan 데이터 구조:
    //   ranges[i] = i번째 각도 방향의 거리 [m]
    //   angle = angle_min + i * angle_increment [rad]
    //   → 극좌표(r, angle) → laser_link 직교좌표(x,y) → map frame 변환
    //
    // 필터링 조건:
    //   inf/nan          : 반사 없음(유리면 등) → 제거
    //   range_min 미만   : 로봇 자체 반사 → 제거
    //   range_max 초과   : 센서 유효 범위 초과 → 제거
    //   방 경계(2.1m) 초과: 벽 포인트 → 제거 (클러스터링 오류 방지)
    std::vector<std::pair<double, double>> valid_points;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double r = msg->ranges[i];
      // 유효하지 않은 range 값 제거
      if (std::isinf(r) || std::isnan(r) || r < msg->range_min || r > msg->range_max) continue;
      
      // 극좌표 → laser_link frame 직교좌표 변환
      // lx = r * cos(θ),  ly = r * sin(θ)
      double angle = msg->angle_min + i * msg->angle_increment;
      geometry_msgs::msg::PointStamped pt_laser, pt_map;
      pt_laser.point.x = r * std::cos(angle);
      pt_laser.point.y = r * std::sin(angle);
      // tf2::doTransform: laser_link frame → map frame 변환 (회전 + 평행이동)
      tf2::doTransform(pt_laser, pt_map, tf_laser_to_map);
      
      // 방 경계(2.3m) 외곽은 무시합니다.
      // simple_room(5×5m) 기준 중심에서 ±2.5m가 최대이므로
      // 2.1m로 제한해서 벽 포인트가 클러스터링에 섞이지 않도록 차단
      if (std::abs(pt_map.point.x) > 2.1 || std::abs(pt_map.point.y) > 2.1) continue;
      valid_points.push_back({pt_map.point.x, pt_map.point.y});
    }

    // ── STEP 2: Euclidean Clustering ──────────────────────────
    // [알고리즘]
    //   scan 포인트는 각도 순서로 정렬되어 있으므로,
    //   인접한 포인트끼리 거리가 cluster_tol 이하면 같은 클러스터로 묶음.
    //   마지막으로 추가된 포인트와만 비교하는 순차 방식.
    //   (BFS보다 단순하지만, scan이 각도 순 정렬이라 실용상 충분함)
    //
    // cluster_tol = 0.25m 기준:
    //   너무 크면 서로 다른 장애물이 하나로 합쳐짐.
    //   너무 작으면 장애물이 여러 조각으로 쪼개짐.
    std::vector<std::vector<std::pair<double, double>>> clusters;
    double cluster_tol = 0.25; 
    for (const auto& pt : valid_points) {
      if (clusters.empty()) { clusters.push_back({pt}); } 
      else {
        auto& last_c = clusters.back();
        // 현재 포인트와 클러스터 마지막 포인트 사이 거리 비교
        if (std::hypot(pt.first - last_c.back().first, pt.second - last_c.back().second) < cluster_tol) {
          last_c.push_back(pt);    // 가까우면 현재 클러스터에 추가
        } else { clusters.push_back({pt}); }   // 멀면 새 클러스터 시작
      }
    }

    // ── STEP 3: Tracking + EMA 필터 + 속도 추정 ───────────────
    // [Tracking이 필요한 이유]
    //   클러스터링만 하면 프레임마다 새 장애물 목록이 생성됨 → 속도 추정 불가.
    //   이전 프레임의 같은 장애물과 매칭하면:
    //     ① 속도 추정: Δposition / Δtime (1차 차분)
    //     ② 노이즈 억제: EMA 필터 적용
    //
    // [EMA 필터 (지수이동평균, Exponential Moving Average)]
    //   smoothed = (1-α) * 이전값 + α * 새 관측값
    //   α = 0.2 → 새 관측을 20%만 반영, 이전 값 80% 유지
    //   → 한 프레임에서 급격히 튀는 값을 억제 (저역통과필터 효과)
    //   α가 클수록: 최신 값에 빠르게 반응하지만 노이즈에 민감
    //   α가 작을수록: 부드럽지만 실제 변화에 느리게 반응
    //
    // [매칭 방식: 최근접 이웃 (Nearest Neighbor)]
    //   새 클러스터 centroid와 이전 프레임 장애물 목록 비교.
    //   거리 0.8m 이내에서 가장 가까운 것 = 같은 장애물로 판단.
    //   0.8m 기준: 0.30 m/s × 0.1s(10Hz) = 0.03m 이동 → 충분한 여유
    std::vector<TrackedObs> current_detected;
    for (const auto& cluster : clusters) {
      // 포인트 수 7개 미만 클러스터는 노이즈로 판단하고 제거.
      // 실제 장애물(원기둥 r≈0.15m)은 10Hz scan에서 최소 7~10개 포인트가 잡힘.
      if (cluster.size() < 7) continue; 
      
      // centroid 계산: 클러스터 내 모든 포인트의 좌표 평균
      double cx = 0, cy = 0;
      for (const auto& p : cluster) { cx += p.first; cy += p.second; }
      cx /= cluster.size(); cy /= cluster.size();
      
      // radius 계산: centroid에서 가장 먼 포인트까지의 거리
      // 이 값이 장애물 "크기(반경)"의 추정값이 됨
      double max_d = 0;
      for (const auto& p : cluster) max_d = std::max(max_d, std::hypot(p.first-cx, p.second-cy));
      double radius = max_d; // 여유분 축소

      // 이전 프레임 장애물과 매칭 (최근접 이웃 탐색)
      // matched_idx: 매칭된 이전 장애물의 인덱스 (-1이면 새 장애물)
      // min_match_dist: 탐색 중 발견된 최소 거리 (초기값 = 허용 최대 거리)
      int matched_idx = -1;
      double min_match_dist = 0.8;
      for (size_t i = 0; i < tracked_obstacles_.size(); ++i) {
        double d = std::hypot(cx - tracked_obstacles_[i].x, cy - tracked_obstacles_[i].y);
        if (d < min_match_dist) { min_match_dist = d; matched_idx = i; }
      }

      TrackedObs obs;
      obs.last_seen = current_time;
      if (matched_idx != -1) {
        // ── 매칭 성공: 이전 장애물과 동일 → EMA 필터 + 속도 추정 ──
        double alpha = 0.08; // EMA 필터 가중치 (값이 작을수록 부드러워짐)
        obs.id = tracked_obstacles_[matched_idx].id;  // 기존 ID 유지

        // EMA 위치 필터: 급격한 위치 튐 억제 (이전 80% + 새 관측 20%)
        obs.x = (1.0 - alpha) * tracked_obstacles_[matched_idx].x + alpha * cx;
        obs.y = (1.0 - alpha) * tracked_obstacles_[matched_idx].y + alpha * cy;
        // EMA 반경 필터: 반경 추정도 부드럽게 유지
        obs.radius = (1.0 - alpha) * tracked_obstacles_[matched_idx].radius + alpha * radius;
        
        // 속도 추정: 1차 차분 (Δposition / Δtime) + EMA 필터
        // dt: 이전 프레임과 현재 프레임 사이의 실제 시간 간격 (10Hz면 ≈0.1s)
        double dt = (current_time - tracked_obstacles_[matched_idx].last_seen).seconds();
        if (dt > 0) {
          // raw 속도: EMA 필터 적용 전 순수 차분 속도 [m/s]
          double rx = (obs.x - tracked_obstacles_[matched_idx].x) / dt;
          double ry = (obs.y - tracked_obstacles_[matched_idx].y) / dt;
          // EMA 속도 필터: 속도는 위치보다 노이즈에 훨씬 민감하므로 강하게 평활화
          obs.vx = (1.0 - alpha) * tracked_obstacles_[matched_idx].vx + alpha * rx;
          obs.vy = (1.0 - alpha) * tracked_obstacles_[matched_idx].vy + alpha * ry;
        }
        // dt <= 0이면 속도 계산 불가 → vx, vy는 기본값 0 유지
      } else {
        // ── 매칭 실패: 새로 등장한 장애물 ────────────────────────
        // 첫 감지 시 속도를 알 수 없으므로 0으로 초기화.
        // 다음 프레임부터 매칭되면 속도 추정 시작됨.
        obs.id = next_id_++; obs.x = cx; obs.y = cy; obs.radius = radius; obs.vx = 0.0; obs.vy = 0.0;
      }
      current_detected.push_back(obs);
    }
    // 현재 프레임 결과를 저장 → 다음 프레임의 "이전 프레임"으로 사용됨
    tracked_obstacles_ = current_detected;

    // ── STEP 4: ObstacleArray 메시지 조립 및 발행 ─────────────
    // amr_msgs::ObstacleArray 구조:
    //   header: 타임스탬프 + 좌표계 ("map")
    //   count:  감지된 장애물 수
    //   x[], y[]: centroid 위치 배열 (map frame)
    //   vx[], vy[]: 속도 배열 [m/s] — MpcCore::buildQP()의 CV 예측에 사용됨
    //   radius[]: 추정 반경 배열 [m]
    // MpcNode::obsCallback()이 이 토픽을 구독해서 setObstacles()로 전달함. [cite: 350]
    amr_msgs::msg::ObstacleArray out_msg;
    out_msg.header.stamp = current_time;
    out_msg.header.frame_id = "map";   // MPC가 map frame 기준으로 동작하므로 반드시 "map"
    out_msg.count = tracked_obstacles_.size();
    for (const auto& o : tracked_obstacles_) {
      out_msg.x.push_back(o.x); out_msg.y.push_back(o.y);
      out_msg.vx.push_back(o.vx); out_msg.vy.push_back(o.vy);
      out_msg.radius.push_back(o.radius);
    }
    obs_pub_->publish(out_msg);
  }

  // ── 멤버 변수 ─────────────────────────────────────────────
  tf2_ros::Buffer tf_buffer_;               // TF2 변환 정보 버퍼 (lookupTransform이 여기서 읽음)
  tf2_ros::TransformListener tf_listener_;  // /tf 토픽 구독 → tf_buffer_ 자동 갱신
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;  // /scan 구독자
  rclcpp::Publisher<amr_msgs::msg::ObstacleArray>::SharedPtr obs_pub_;     // /obstacles/detected 발행자
  std::vector<TrackedObs> tracked_obstacles_;  // 현재 추적 중인 장애물 목록 (프레임 간 유지됨)
  int next_id_ = 0;  // 장애물 ID 카운터 (새 장애물 등장마다 1씩 증가, 디버깅용)
};
}

// ============================================================
// main() — 노드 진입점
//
// rclcpp::spin(): 무한 루프로 콜백 대기
//   → /scan 수신마다 scanCallback() 자동 호출
//   → Ctrl+C 시 spin() 탈출 → shutdown()
// ============================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control_mpc::ObstacleTrackerNode>());
  rclcpp::shutdown();
  return 0;
}