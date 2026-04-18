cmake_minimum_required(VERSION 3.8)
project(planning)

find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(nav_msgs    REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(amr_msgs    REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# ---------------------------------------------------------------
# path_planner_node
#   - astar.cpp     : A* 탐색 + 맵 팽창
#   - path_planner_node.cpp : ROS2 노드 (구독/발행, 재계획 로직)
#   - main.cpp      : 진입점
# ---------------------------------------------------------------
add_executable(path_planner_node
  src/astar.cpp
  src/path_planner_node.cpp
  src/main.cpp
)

ament_target_dependencies(path_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  amr_msgs
)

install(TARGETS
  path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
