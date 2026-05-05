// ============================================================
// obstacle_tracker_node.cpp — W9 동적 장애물 추적 노드
//
// [전체 처리 흐름]
//   /scan (LiDAR 원시 데이터, laser_link 좌표계)
//     ↓ range 기반 필터링 (LiDAR frame, 맵 크기 무관)
//   유효 포인트 → TF2 변환 (laser_link → map frame)
//     ↓ Euclidean Clustering
//   클러스터 목록 (연속된 포인트의 묶음)
//     ↓ Tracking + EMA 필터
//   이전 프레임 매칭 → 속도 추정 → 노이즈 완화
//     ↓ 발행
//   /obstacles/detected (ObstacleArray, map frame)
//
// [하드코딩 제거 설계 방침]
//   기존 코드는 map_boundary_=2.9, y범위(-0.5~5.8) 등을
//   맵 크기에 맞춰 하드코딩했음.
//   → 맵이 바뀔 때마다 코드 수정이 필요했음.
//
//   신규 설계: 벽 필터링을 map frame 좌표 기반이 아닌
//   LiDAR range 기반으로 수행.
//   → 벽은 항상 range_max 근처에서 잡히고,
//     장애물은 항상 range_max보다 가까이 있다는 물리적 사실 활용.
//   → 맵 크기, 로봇 위치, 좌표 범위와 완전히 독립.
//
// [파라미터 일람]
//   max_range_factor    (0.92)  LiDAR range_max의 몇 배 이하만 유효한 장애물로 처리
//                               → 벽(range_max 근처)을 자동으로 제외
//   cluster_tolerance   (0.25)  클러스터 병합 거리 임계값 [m]
//   min_cluster_points  (7)     유효 클러스터 최소 포인트 수
//   max_cluster_radius  (0.6)   유효 클러스터 최대 반경 [m] (이 이상 = 벽/대형 구조물)
//   ema_alpha           (0.08)  EMA 필터 가중치 (0~1, 클수록 최신값 반영)
//   match_dist_thresh   (0.5)   프레임 간 장애물 매칭 최대 거리 [m]
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <amr_msgs/msg/obstacle_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

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
// ============================================================
class ObstacleTrackerNode : public rclcpp::Node {
public:
  ObstacleTrackerNode()
  : Node("obstacle_tracker_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ── 파라미터 선언 ──────────────────────────────────────────
    //
    // [max_range_factor]
    //   LiDAR range_max 대비 유효 장애물 감지 거리 비율.
    //   기본값 0.92 = range_max의 92% 이내 포인트만 장애물로 처리.
    //   나머지 8% (벽 등)는 자동 제외.
    //   → 맵 크기나 LiDAR 스펙이 달라져도 코드 수정 불필요.
    //   → Gazebo LiDAR range_max가 10m이면 9.2m 이내만 처리.
    this->declare_parameter("max_range_factor",   0.7);

    // [cluster_tolerance]
    //   인접 포인트 간 최대 거리 [m]. 이 이하면 같은 클러스터로 묶음.
    //   0.25m: 0.3×0.3m 박스 장애물이 여러 조각으로 쪼개지지 않을 최소값.
    this->declare_parameter("cluster_tolerance",  0.25);

    // [min_cluster_points]
    //   유효 클러스터로 인정하는 최소 포인트 수.
    //   7개 미만: 단일 반사점 노이즈 또는 맵 경계 아티팩트로 판단하여 제거.
    this->declare_parameter("min_cluster_points", 7);

    // [max_cluster_radius]
    //   클러스터 반경 상한 [m]. 이 이상이면 벽/기둥/대형 구조물로 판단하여 제거.
    //   0.6m: 단일 장애물(r≈0.15m)과 벽(r>>1m)을 구분하는 임계값.
    this->declare_parameter("max_cluster_radius", 0.60);

    // [ema_alpha]
    //   EMA(지수이동평균) 필터 가중치 (0~1).
    //   smoothed = (1-α)*이전값 + α*새 관측값
    //   0.08: 새 관측을 8%만 반영 → 강한 평활화, 노이즈 억제 우선.
    this->declare_parameter("ema_alpha",          0.08);

    // [match_dist_thresh]
    //   프레임 간 장애물 매칭 최대 거리 [m].
    //   이 거리 이내에서 가장 가까운 이전 장애물을 같은 장애물로 판단.
    //   0.5m: 동적 장애물 속도 0.5m/s × 주기 0.1s = 0.05m 이동 → 충분한 여유.
    this->declare_parameter("match_dist_thresh",  0.50);

    // ── 파라미터 로드 ──────────────────────────────────────────
    max_range_factor_   = this->get_parameter("max_range_factor").as_double();
    cluster_tolerance_  = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_points_ = this->get_parameter("min_cluster_points").as_int();
    max_cluster_radius_ = this->get_parameter("max_cluster_radius").as_double();
    ema_alpha_          = this->get_parameter("ema_alpha").as_double();
    match_dist_thresh_  = this->get_parameter("match_dist_thresh").as_double();

    // ── subscriber / publisher ─────────────────────────────────
    // SensorDataQoS = BEST_EFFORT + 소규모 히스토리
    // Gazebo /scan 브릿지도 BEST_EFFORT로 발행하므로 QoS를 맞춰야 연결됨.
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&ObstacleTrackerNode::scanCallback, this, std::placeholders::_1));

    obs_pub_ = this->create_publisher<amr_msgs::msg::ObstacleArray>(
      "/obstacles/detected", 10);

    RCLCPP_INFO(this->get_logger(),
      "Obstacle Tracker 시작 | "
      "range_factor=%.2f cluster_tol=%.2fm min_pts=%d "
      "max_r=%.2fm ema_α=%.2f match=%.2fm",
      max_range_factor_, cluster_tolerance_, min_cluster_points_,
      max_cluster_radius_, ema_alpha_, match_dist_thresh_);
  }

private:
  // ============================================================
  // scanCallback() — /scan 수신마다 호출 (10Hz)
  // ============================================================
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "LiDAR 수신 중 | 감지 장애물: %zu개", tracked_obstacles_.size());

    rclcpp::Time current_time = msg->header.stamp;

    // ── TF 획득 ───────────────────────────────────────────────
    geometry_msgs::msg::TransformStamped tf_laser_to_map;
    try {
      tf_laser_to_map = tf_buffer_.lookupTransform(
        "map", msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "TF 대기 중: %s", ex.what());
      return;
    }

    // ── STEP 1: 유효 포인트 추출 ──────────────────────────────
    //
    // [벽 필터링 전략 — range 기반]
    //   기존: map frame 절대 좌표(x < 2.9, y 범위) → 맵 크기에 종속
    //   신규: LiDAR range < range_max * max_range_factor_ → 맵 크기 무관
    //
    //   물리적 근거:
    //     벽 → LiDAR에서 range_max 근처로 잡힘
    //     장애물 → 벽보다 가까이 있어 range가 작음
    //   따라서 range_max의 일정 비율 이상인 포인트 = 벽으로 판단하여 제거.
    //   → 맵이 어떤 크기여도, LiDAR 스펙이 달라져도 동작.
    //
    // [필터 조건 요약]
    //   ① inf/nan, range_min 미만, range_max 초과: 센서 무효값 → 제거
    //   ② range >= range_max * max_range_factor_: 벽으로 판단 → 제거
    //   ③ 나머지: 장애물 후보 → map frame으로 변환 후 클러스터링
    double effective_max_range = msg->range_max * max_range_factor_;

    std::vector<std::pair<double, double>> valid_points;
    valid_points.reserve(msg->ranges.size());

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double r = msg->ranges[i];

      // ① 센서 무효값 제거
      if (std::isinf(r) || std::isnan(r) ||
          r < msg->range_min || r > msg->range_max) {
        continue;
      }

      // ② 벽 필터링: range_max 근처 포인트 제거
      // 코드 수정 없이 맵 크기에 독립적으로 동작하는 핵심 로직
      if (r >= effective_max_range) {
        continue;
      }

      // ③ laser frame 극좌표 → 직교좌표 변환
      double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      geometry_msgs::msg::PointStamped pt_laser, pt_map;
      pt_laser.point.x = r * std::cos(angle);
      pt_laser.point.y = r * std::sin(angle);
      pt_laser.point.z = 0.0;

      // laser_link → map frame 변환
      tf2::doTransform(pt_laser, pt_map, tf_laser_to_map);

      valid_points.push_back({pt_map.point.x, pt_map.point.y});
    }

    // ── STEP 2: Euclidean Clustering ──────────────────────────
    //
    // scan 포인트는 각도 순서로 정렬되어 있으므로
    // 인접 포인트 간 거리가 cluster_tolerance_ 이하면 같은 클러스터로 묶음.
    // cluster_tolerance_ 파라미터로 런타임 조정 가능.
    std::vector<std::vector<std::pair<double, double>>> clusters;

    for (const auto & pt : valid_points) {
      if (clusters.empty()) {
        clusters.push_back({pt});
      } else {
        auto & last_c = clusters.back();
        double dist = std::hypot(
          pt.first  - last_c.back().first,
          pt.second - last_c.back().second);
        if (dist < cluster_tolerance_) {
          last_c.push_back(pt);   // 기존 클러스터에 추가
        } else {
          clusters.push_back({pt}); // 새 클러스터 시작
        }
      }
    }

    // ── STEP 3: 클러스터 검증 + Tracking + EMA ────────────────
    std::vector<TrackedObs> current_detected;
    current_detected.reserve(clusters.size());

    for (const auto & cluster : clusters) {

      // 최소 포인트 수 미달 → 노이즈로 제거
      // min_cluster_points_ 파라미터로 런타임 조정 가능
      if (static_cast<int>(cluster.size()) < min_cluster_points_) continue;

      // centroid 계산: 클러스터 내 모든 포인트의 좌표 평균
      double cx = 0.0, cy = 0.0;
      for (const auto & p : cluster) { cx += p.first; cy += p.second; }
      cx /= static_cast<double>(cluster.size());
      cy /= static_cast<double>(cluster.size());

      // radius 계산: centroid에서 가장 먼 포인트까지의 거리
      double max_d = 0.0;
      for (const auto & p : cluster) {
        max_d = std::max(max_d, std::hypot(p.first - cx, p.second - cy));
      }
      double radius = max_d;

      // 최대 반경 초과 → 벽/대형 구조물로 제거
      // max_cluster_radius_ 파라미터로 런타임 조정 가능
      if (radius > max_cluster_radius_) continue;

      // ── 이전 프레임과 매칭 (최근접 이웃) ──────────────────────
      // 거리 match_dist_thresh_ 이내에서 가장 가까운 이전 장애물 탐색
      int    matched_idx    = -1;
      double min_match_dist = match_dist_thresh_;

      for (size_t i = 0; i < tracked_obstacles_.size(); ++i) {
        double d = std::hypot(
          cx - tracked_obstacles_[i].x,
          cy - tracked_obstacles_[i].y);
        if (d < min_match_dist) {
          min_match_dist = d;
          matched_idx    = static_cast<int>(i);
        }
      }

      TrackedObs obs;
      obs.last_seen = current_time;

      if (matched_idx != -1) {
        // ── 매칭 성공: EMA 필터 + 속도 추정 ──────────────────
        // ema_alpha_ 파라미터로 런타임 조정 가능
        const TrackedObs & prev = tracked_obstacles_[matched_idx];
        double alpha = ema_alpha_;

        obs.id     = prev.id;
        obs.x      = (1.0 - alpha) * prev.x      + alpha * cx;
        obs.y      = (1.0 - alpha) * prev.y      + alpha * cy;
        obs.radius = (1.0 - alpha) * prev.radius + alpha * radius;

        // 속도 추정: 1차 차분 (Δpos / Δt) + EMA 필터
        double dt = (current_time - prev.last_seen).seconds();
        if (dt > 0.0) {
          double raw_vx = (obs.x - prev.x) / dt;
          double raw_vy = (obs.y - prev.y) / dt;
          obs.vx = (1.0 - alpha) * prev.vx + alpha * raw_vx;
          obs.vy = (1.0 - alpha) * prev.vy + alpha * raw_vy;
        } else {
          obs.vx = prev.vx;
          obs.vy = prev.vy;
        }

      } else {
        // ── 매칭 실패: 새 장애물 등록 ─────────────────────────
        obs.id     = next_id_++;
        obs.x      = cx;
        obs.y      = cy;
        obs.radius = radius;
        obs.vx     = 0.0;
        obs.vy     = 0.0;
      }

      current_detected.push_back(obs);
    }

    // 현재 프레임 결과 저장 → 다음 프레임의 이전 프레임으로 사용
    tracked_obstacles_ = std::move(current_detected);

    // ── STEP 4: ObstacleArray 발행 ────────────────────────────
    amr_msgs::msg::ObstacleArray out_msg;
    out_msg.header.stamp    = current_time;
    out_msg.header.frame_id = "map";
    out_msg.count           = static_cast<int>(tracked_obstacles_.size());

    for (const auto & o : tracked_obstacles_) {
      out_msg.x.push_back(o.x);
      out_msg.y.push_back(o.y);
      out_msg.vx.push_back(o.vx);
      out_msg.vy.push_back(o.vy);
      out_msg.radius.push_back(o.radius);
    }

    obs_pub_->publish(out_msg);
  }

  // ── 멤버 변수 ─────────────────────────────────────────────
  tf2_ros::Buffer           tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<amr_msgs::msg::ObstacleArray>::SharedPtr   obs_pub_;

  std::vector<TrackedObs> tracked_obstacles_;
  int next_id_ = 0;

  // 파라미터 (모두 런타임 로드, 하드코딩 없음)
  double max_range_factor_;    // LiDAR range_max 대비 유효 감지 비율
  double cluster_tolerance_;   // 클러스터 병합 거리 [m]
  int    min_cluster_points_;  // 최소 유효 클러스터 포인트 수
  double max_cluster_radius_;  // 최대 클러스터 반경 [m] (벽 제거 기준)
  double ema_alpha_;           // EMA 필터 가중치
  double match_dist_thresh_;   // 프레임 간 매칭 최대 거리 [m]
};

}  // namespace control_mpc

// ============================================================
// main()
// ============================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control_mpc::ObstacleTrackerNode>());
  rclcpp::shutdown();
  return 0;
}