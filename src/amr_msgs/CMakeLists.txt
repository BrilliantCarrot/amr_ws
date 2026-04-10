cmake_minimum_required(VERSION 3.8)
project(amr_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlLatency.msg"
  "msg/PoseRmse.msg"
  "msg/MinObstacleDistance.msg"
  "msg/LocalizationStatus.msg"
  "msg/ObstacleArray.msg"
  "msg/SafetyStatus.msg"       # W10 Step1: /safety/state 토픽용
  "msg/WatchdogEvent.msg"      # W10 Step2: /safety/watchdog_event 토픽용
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()