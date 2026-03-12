#include <rclcpp/rclcpp.hpp>
#include "estimation/ekf_node.hpp"

// ============================================================
// main.cpp — EKF 노드 진입점
//
// [역할]
//   ROS2 런타임을 초기화하고 EkfNode를 생성해서 실행함.
//   모든 실제 로직은 EkfNode와 EkfCore에 있고,
//   이 파일은 단순히 노드를 띄우는 역할만 함.
//
// [rclcpp::spin이란?]
//   노드를 이벤트 루프에 올려서 토픽 콜백이 계속 실행되게 함.
//   Ctrl+C 또는 rclcpp::shutdown() 호출 전까지 블로킹됨.
//   내부적으로 subscriber 큐를 폴링하면서
//   메시지가 오면 등록된 콜백(imuCallback, odomCallback)을 실행함.
// ============================================================
int main(int argc, char ** argv)
{
  // ROS2 초기화 — argc, argv는 런타임 인자 전달용
  // (ros2 run ... --ros-args -p publish_tf:=false 같은 파라미터 전달에 필요)
  rclcpp::init(argc, argv);

  // EkfNode 생성 및 실행
  // make_shared: 스마트 포인터로 생성 → 메모리 자동 관리
  rclcpp::spin(std::make_shared<estimation::EkfNode>());

  // 종료 처리 — spin에서 빠져나오면 ROS2 정리
  rclcpp::shutdown();

  return 0;
}