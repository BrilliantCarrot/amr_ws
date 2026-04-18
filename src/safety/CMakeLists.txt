cmake_minimum_required(VERSION 3.8)
project(safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(amr_msgs REQUIRED)

# ---- state_machine_node ----
add_executable(state_machine_node
  src/state_machine_node.cpp
)
ament_target_dependencies(state_machine_node
  rclcpp std_msgs geometry_msgs diagnostic_msgs amr_msgs
)
target_include_directories(state_machine_node PUBLIC include)

# ---- watchdog_node ----
add_executable(watchdog_node
  src/watchdog_node.cpp
)
ament_target_dependencies(watchdog_node
  rclcpp std_msgs geometry_msgs amr_msgs
)
target_include_directories(watchdog_node PUBLIC include)

# ---- mock_link_node (W11 Step1) ----
add_executable(mock_link_node
  src/mock_link_node.cpp
)
ament_target_dependencies(mock_link_node
  rclcpp std_msgs geometry_msgs amr_msgs
)
target_include_directories(mock_link_node PUBLIC include)

# ---- qos_experiment_node (W11 Step2) ----
add_executable(qos_experiment_node
  src/qos_experiment_node.cpp
)
ament_target_dependencies(qos_experiment_node
  rclcpp std_msgs geometry_msgs nav_msgs amr_msgs
)
target_include_directories(qos_experiment_node PUBLIC include)

# ---- deadline_monitor_node (W11 Step4) ----
# MPC 제어 루프 Deadline Miss 감지 노드
# Eigen3 불필요 (순수 ROS2 + STL 연산)
add_executable(deadline_monitor_node
  src/deadline_monitor_node.cpp
)
ament_target_dependencies(deadline_monitor_node
  rclcpp std_msgs diagnostic_msgs amr_msgs
)
target_include_directories(deadline_monitor_node PUBLIC include)

install(TARGETS
  state_machine_node
  watchdog_node
  mock_link_node
  qos_experiment_node
  deadline_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()