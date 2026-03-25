cmake_minimum_required(VERSION 3.8)
project(control_mpc)

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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# OSQP: 소스 빌드로 /usr/local에 설치된 버전 사용
find_library(OSQP_LIB osqp HINTS /usr/local/lib REQUIRED)
find_path(OSQP_INCLUDE_DIR osqp.h HINTS /usr/local/include/osqp REQUIRED)

include_directories(include ${OSQP_INCLUDE_DIR})

add_executable(mpc_node
  src/mpc_core.cpp
  src/mpc_node.cpp
  src/main.cpp
)

ament_target_dependencies(mpc_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  amr_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(mpc_node
  Eigen3::Eigen
  ${OSQP_LIB}
)

install(TARGETS
  mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()