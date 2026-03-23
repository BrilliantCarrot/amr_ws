cmake_minimum_required(VERSION 3.8)
project(estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

include_directories(include)

add_executable(ekf_node
  src/ekf_core.cpp
  src/ekf_node.cpp
  src/main.cpp
)

ament_target_dependencies(ekf_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  amr_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(ekf_node
  Eigen3::Eigen
)

# pose_rmse_node — Ground Truth vs EKF RMSE 계산 노드
add_executable(pose_rmse_node
  src/pose_rmse_node.cpp
)

ament_target_dependencies(pose_rmse_node
  rclcpp
  geometry_msgs
  nav_msgs
  amr_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs
)

target_link_libraries(pose_rmse_node
  Eigen3::Eigen
)

# map_ekf_node — W5 신규: map 프레임 기반 LiDAR fusion EKF
# ekf_core.cpp를 공유해서 updateLidar()를 그대로 사용함
add_executable(map_ekf_node
  src/ekf_core.cpp
  src/map_ekf_node.cpp
)

ament_target_dependencies(map_ekf_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  amr_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs
)

target_link_libraries(map_ekf_node
  Eigen3::Eigen
)

install(TARGETS
  ekf_node
  pose_rmse_node
  map_ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()