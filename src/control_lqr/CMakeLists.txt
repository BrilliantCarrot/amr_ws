cmake_minimum_required(VERSION 3.8)
project(control_lqr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# OSQP 의존성 없음 — Eigen3만으로 Riccati 계산

include_directories(include)

# ----------------------------------------------------------------
# lqr_node — TVLQR 제어기 노드
#   lqr_core.cpp : 순수 수학 (선형화, 이산화, Riccati, K 계산)
#   lqr_node.cpp : ROS2 인터페이스 + main()
# ----------------------------------------------------------------
add_executable(lqr_node
  src/lqr_core.cpp
  src/lqr_node.cpp
)

ament_target_dependencies(lqr_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  amr_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(lqr_node
  Eigen3::Eigen
)

install(TARGETS
  lqr_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
