cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(amr_msgs REQUIRED)

add_executable(localization_monitor_node
  src/localization_monitor_node.cpp
)

ament_target_dependencies(localization_monitor_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs
  geometry_msgs
  amr_msgs
)

target_include_directories(localization_monitor_node
  PUBLIC
    include
)

install(TARGETS localization_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# config, launch 디렉토리를 install에 포함
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
