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
//   min_obstacle_radius (0.35)  제어/시각화용 최소 장애물 반경 [m]
//                               → 사람 다리처럼 작게 잡힌 클러스터도 안전 반경으로 부풀림
//   persistence_timeout (0.45)  장애물 미검출 시 유지 시간 [s]
//                               → 1~2프레임 감지 누락으로 마커 CBF 입력이 사라지는 것 방지
//   merge_close_obstacles (true) 가까운 장애물들을 하나의 큰 장애물로 병합
//                               → 사람 다리 두 개를 다리 사이 통로로 오해하는 문제 방지
//   obstacle_merge_distance (0.65) 병합할 중심 간 거리 임계값 [m]
//   merge_extra_radius (0.10) 병합 후 추가 안전 반경 [m]
//   max_merged_obstacle_radius (1.20) 병합 장애물 최대 반경 [m]
//   velocity_deadband (0.10) 정지 장애물의 TF/centroid 노이즈 속도 제거 [m/s]
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
#include <limits>

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

    // [min_obstacle_radius]
    //   제어와 RViz 시각화에 발행할 최소 장애물 반경 [m]임.
    //   Isaac Sim 사람 모델은 2D LiDAR 평면에서 다리 두 개처럼 매우 작게 찍힐 수 있음.
    //   raw cluster radius를 그대로 쓰면 CBF가 사람을 작은 점으로 봐서 회피가 늦어짐.
    //   따라서 raw radius는 필터링에 사용하고, 발행 radius는 이 값 이상으로 부풀림.
    this->declare_parameter("min_obstacle_radius", 0.35);

    // [persistence_timeout]
    //   장애물이 한두 프레임 순간적으로 사라져도 유지할 시간 [s]임.
    //   LiDAR 포인트 수가 순간적으로 min_cluster_points_보다 작아지면 obstacle이 깜빡일 수 있음.
    //   이 값을 두면 RViz 마커와 CBF 입력이 바로 사라지지 않음.
    this->declare_parameter("persistence_timeout", 0.45);

    // [merge_close_obstacles]
    //   서로 가까운 장애물들을 하나의 큰 장애물로 병합할지 결정함.
    //   사람 다리가 2D LiDAR에서 두 개의 작은 장애물로 분리되어 보일 때,
    //   planner가 다리 사이를 통로로 오해하는 문제를 줄이기 위함임.
    this->declare_parameter("merge_close_obstacles", true);

    // [obstacle_merge_distance]
    //   두 장애물 중심 간 거리가 이 값 이하이면 같은 객체 후보로 보고 병합함.
    //   사람 보행/정지 시 양쪽 다리 간격이 보통 작다는 가정을 이용함.
    this->declare_parameter("obstacle_merge_distance", 0.65);

    // [merge_extra_radius]
    //   병합된 원이 두 장애물을 덮은 뒤 추가로 더하는 안전 여유 반경임.
    this->declare_parameter("merge_extra_radius", 0.10);

    // [max_merged_obstacle_radius]
    //   병합 결과가 너무 커져서 전체 맵을 과도하게 막는 것을 방지하는 상한임.
    this->declare_parameter("max_merged_obstacle_radius", 0.80);

    // [velocity_deadband]
    //   정지 물체도 로봇 이동 중 scan/TF 시간차, 클러스터 centroid 변화 때문에
    //   작은 속도가 생길 수 있음. static planner 분류가 흔들리지 않도록
    //   이 값 미만의 추정 속도는 0으로 발행함.
    this->declare_parameter("velocity_deadband", 0.10);

    // ── 파라미터 로드 ──────────────────────────────────────────
    max_range_factor_   = this->get_parameter("max_range_factor").as_double();
    cluster_tolerance_  = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_points_ = this->get_parameter("min_cluster_points").as_int();
    max_cluster_radius_ = this->get_parameter("max_cluster_radius").as_double();
    ema_alpha_          = this->get_parameter("ema_alpha").as_double();
    match_dist_thresh_  = this->get_parameter("match_dist_thresh").as_double();
    min_obstacle_radius_ = this->get_parameter("min_obstacle_radius").as_double();
    persistence_timeout_ = this->get_parameter("persistence_timeout").as_double();
    merge_close_obstacles_ = this->get_parameter("merge_close_obstacles").as_bool();
    obstacle_merge_distance_ = this->get_parameter("obstacle_merge_distance").as_double();
    merge_extra_radius_ = this->get_parameter("merge_extra_radius").as_double();
    max_merged_obstacle_radius_ = this->get_parameter("max_merged_obstacle_radius").as_double();
    velocity_deadband_ = this->get_parameter("velocity_deadband").as_double();

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
      "max_r=%.2fm min_pub_r=%.2fm ema_α=%.2f match=%.2fm persist=%.2fs "
      "merge=%d merge_dist=%.2fm merge_extra=%.2fm vel_deadband=%.2fm/s",
      max_range_factor_, cluster_tolerance_, min_cluster_points_,
      max_cluster_radius_, min_obstacle_radius_, ema_alpha_,
      match_dist_thresh_, persistence_timeout_,
      merge_close_obstacles_ ? 1 : 0, obstacle_merge_distance_, merge_extra_radius_,
      velocity_deadband_);
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
        "map", msg->header.frame_id, msg->header.stamp);
    } catch (tf2::TransformException & ex) {
      try {
        tf_laser_to_map = tf_buffer_.lookupTransform(
          "map", msg->header.frame_id, tf2::TimePointZero);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "scan stamp TF 없음 — 최신 TF로 fallback: %s", ex.what());
      } catch (tf2::TransformException & ex_latest) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "TF 대기 중: %s", ex_latest.what());
        return;
      }
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
    current_detected.reserve(clusters.size() + tracked_obstacles_.size());

    // 이전 장애물 하나가 현재 프레임의 여러 클러스터와 중복 매칭되지 않도록 표시함.
    // 사람 다리처럼 두 클러스터가 가까워도 ID 중복/속도 튐을 줄이기 위함임.
    std::vector<bool> prev_matched(tracked_obstacles_.size(), false);

    for (const auto & cluster : clusters) {

      // 최소 포인트 수 미달 → 노이즈로 제거
      // min_cluster_points_ 파라미터로 런타임 조정 가능
      if (static_cast<int>(cluster.size()) < min_cluster_points_) continue;

      // centroid 계산: 클러스터 내 모든 포인트의 좌표 평균
      double cx = 0.0, cy = 0.0;
      for (const auto & p : cluster) { cx += p.first; cy += p.second; }
      cx /= static_cast<double>(cluster.size());
      cy /= static_cast<double>(cluster.size());

      // radius 계산: centroid에서 가장 먼 포인트까지의 거리임.
      // raw_radius는 벽/대형 구조물 필터링에 사용함.
      double max_d = 0.0;
      for (const auto & p : cluster) {
        max_d = std::max(max_d, std::hypot(p.first - cx, p.second - cy));
      }
      double raw_radius = max_d;

      // 최대 반경 초과 → 벽/대형 구조물로 제거
      // max_cluster_radius_ 파라미터로 런타임 조정 가능
      if (raw_radius > max_cluster_radius_) continue;

      // 제어/시각화에 발행할 radius는 최소 안전 반경 이상으로 부풀림.
      // 사람 다리가 r=0.03~0.05m로 잡혀도 CBF에는 r≈0.35m 장애물로 전달하기 위함임.
      double radius = std::max(raw_radius, min_obstacle_radius_);

      // ── 이전 프레임과 매칭 (최근접 이웃) ──────────────────────
      // 거리 match_dist_thresh_ 이내에서 가장 가까운 이전 장애물 탐색
      int    matched_idx    = -1;
      double min_match_dist = match_dist_thresh_;

      for (size_t i = 0; i < tracked_obstacles_.size(); ++i) {
        if (prev_matched[i]) continue;

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
        prev_matched[static_cast<size_t>(matched_idx)] = true;

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
          if (std::hypot(obs.vx, obs.vy) < velocity_deadband_) {
            obs.vx = 0.0;
            obs.vy = 0.0;
          }
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

    // ── STEP 3-1: Persistence 처리 ───────────────────────────
    // 현재 프레임에서 매칭되지 않은 이전 장애물도 일정 시간 동안 유지함.
    // LiDAR가 사람 다리/얇은 물체를 한두 프레임 놓칠 때 obstacle이 바로 사라지는 문제를 줄임.
    for (size_t i = 0; i < tracked_obstacles_.size(); ++i) {
      if (prev_matched[i]) continue;

      const TrackedObs & prev = tracked_obstacles_[i];
      double age = (current_time - prev.last_seen).seconds();

      if (age <= persistence_timeout_) {
        TrackedObs kept = prev;

        // 미검출 상태에서는 속도 추정값을 조금씩 줄임.
        // 오래된 속도가 남아서 정지 장애물을 동적 장애물처럼 보이게 하는 것 방지함.
        kept.vx *= 0.5;
        kept.vy *= 0.5;
        kept.radius = std::max(kept.radius, min_obstacle_radius_);

        current_detected.push_back(kept);
      }
    }

    // ── STEP 3-2: 가까운 장애물 병합 ─────────────────────────
    // 사람 다리처럼 서로 가까운 두 클러스터를 하나의 큰 장애물로 합침.
    // 이렇게 해야 A* planner가 두 다리 사이를 통과 가능한 빈 공간으로 오해하지 않음.
    if (merge_close_obstacles_) {
      current_detected = mergeCloseObstacles(current_detected, current_time);
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


  // ============================================================
  // mergeCloseObstacles() — 가까운 장애물 병합
  //
  // [목적]
  //   2D LiDAR에서는 사람 다리가 두 개의 작은 원으로 분리되어 보일 수 있음.
  //   이 상태를 그대로 planner에 넘기면 다리 사이 공간을 지나갈 수 있다고 판단할 수 있음.
  //   중심 거리가 obstacle_merge_distance_ 이하인 장애물들을 하나의 큰 원으로 병합함.
  // ============================================================
  std::vector<TrackedObs> mergeCloseObstacles(
    const std::vector<TrackedObs> & input,
    const rclcpp::Time & current_time) const
  {
    if (input.empty()) return input;

    std::vector<TrackedObs> merged;
    std::vector<bool> used(input.size(), false);

    for (size_t i = 0; i < input.size(); ++i) {
      if (used[i]) continue;

      std::vector<size_t> group;
      std::vector<size_t> queue;
      used[i] = true;
      queue.push_back(i);

      // BFS 방식으로 연결된 가까운 장애물들을 모두 같은 그룹에 넣음.
      while (!queue.empty()) {
        size_t idx = queue.back();
        queue.pop_back();
        group.push_back(idx);

        for (size_t j = 0; j < input.size(); ++j) {
          if (used[j]) continue;
          double d = std::hypot(input[idx].x - input[j].x, input[idx].y - input[j].y);
          if (d <= obstacle_merge_distance_) {
            used[j] = true;
            queue.push_back(j);
          }
        }
      }

      if (group.size() == 1) {
        TrackedObs single = input[group.front()];
        single.radius = std::max(single.radius, min_obstacle_radius_);
        merged.push_back(single);
        continue;
      }

      // 반경 가중 평균으로 병합 중심을 계산함.
      // 큰 클러스터가 더 안정적인 중심을 제공한다고 보고 더 높은 가중치를 부여함.
      double sum_w = 0.0;
      double mx = 0.0, my = 0.0;
      double mvx = 0.0, mvy = 0.0;
      int min_id = input[group.front()].id;
      rclcpp::Time latest_seen = input[group.front()].last_seen;

      for (size_t idx : group) {
        const auto & o = input[idx];
        double w = std::max(o.radius, 0.05);
        sum_w += w;
        mx += w * o.x;
        my += w * o.y;
        mvx += o.vx;
        mvy += o.vy;
        min_id = std::min(min_id, o.id);
        if (o.last_seen > latest_seen) latest_seen = o.last_seen;
      }

      mx /= std::max(sum_w, 1e-6);
      my /= std::max(sum_w, 1e-6);
      mvx /= static_cast<double>(group.size());
      mvy /= static_cast<double>(group.size());
      if (std::hypot(mvx, mvy) < velocity_deadband_) {
        mvx = 0.0;
        mvy = 0.0;
      }

      // 병합 원이 그룹 내 모든 원을 포함하도록 반경 계산함.
      double mr = min_obstacle_radius_;
      for (size_t idx : group) {
        const auto & o = input[idx];
        double cover_r = std::hypot(o.x - mx, o.y - my) + std::max(o.radius, min_obstacle_radius_);
        mr = std::max(mr, cover_r);
      }
      mr = std::min(mr + merge_extra_radius_, max_merged_obstacle_radius_);

      TrackedObs out;
      out.id = min_id;
      out.x = mx;
      out.y = my;
      out.vx = mvx;
      out.vy = mvy;
      out.radius = mr;
      out.last_seen = latest_seen > current_time ? current_time : latest_seen;
      merged.push_back(out);
    }

    return merged;
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
  double ema_alpha_;            // EMA 필터 가중치
  double match_dist_thresh_;    // 프레임 간 매칭 최대 거리 [m]
  double min_obstacle_radius_;  // 제어/시각화용 최소 발행 반경 [m]
  double persistence_timeout_;  // 미검출 장애물 유지 시간 [s]
  bool merge_close_obstacles_;  // 가까운 장애물 병합 사용 여부
  double obstacle_merge_distance_;  // 병합 중심 거리 임계값 [m]
  double merge_extra_radius_;  // 병합 후 추가 안전 반경 [m]
  double max_merged_obstacle_radius_;  // 병합 장애물 반경 상한 [m]
  double velocity_deadband_;  // 정지 장애물 속도 노이즈 제거 임계값 [m/s]
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
