#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include "control_mpc/mpc_node.hpp"

// =================================================================
// 멀티스레드용 main 프로그램
// 멀티스레드로 돌리려면 이 파일 이름을 main으로 바꿈
// =================================================================
// MultiThreadedExecutor 전역 포인터 (시그널 핸들러에서 접근하여 취소하기 위함)
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> g_executor;

// 인수인계 문서 문제 2 해결: Ctrl+C 강제 종료를 위한 커스텀 시그널 핸들러
void signalHandler(int signum)
{
  (void)signum;
  if (g_executor) {
    // 실행 중인 스레드와 펜딩된 콜백들을 안전하게 취소하여 데드락 없이 스핀을 종료합니다.
    g_executor->cancel(); 
  }
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // OS의 SIGINT(Ctrl+C) 시그널을 가로채서 커스텀 핸들러로 연결합니다.
  std::signal(SIGINT, signalHandler);

  auto node = std::make_shared<control_mpc::MpcNode>();
  
  // Step 6: SingleThreadedExecutor(rclcpp::spin)에서 MultiThreadedExecutor로 공식 전환
  g_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  g_executor->add_node(node);
  
  // 다중 스레드 풀을 활용하여 콜백 그룹들을 병렬로 실행합니다.
  g_executor->spin();
  
  g_executor->remove_node(node);
  return 0;
}