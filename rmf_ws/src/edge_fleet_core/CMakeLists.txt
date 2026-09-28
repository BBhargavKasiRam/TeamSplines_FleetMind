cmake_minimum_required(VERSION 3.8)
project(edge_fleet_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(edge_fleet_msgs REQUIRED)

include_directories(
  include
)

# 1. P2P Peer Node
add_executable(p2p_peer_node src/p2p_peer_node.cpp)
ament_target_dependencies(p2p_peer_node
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  edge_fleet_msgs
)

# 2. Conflict Resolver Node
add_executable(conflict_resolver_node src/conflict_resolver_node.cpp)
ament_target_dependencies(conflict_resolver_node
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  edge_fleet_msgs
)

# 3. Task Allocator Node (Contract Net Protocol)
add_executable(task_allocator_node src/task_allocator_node.cpp)
ament_target_dependencies(task_allocator_node
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  std_msgs
  edge_fleet_msgs
)

# 4. Fleet Benchmark Node
add_executable(fleet_benchmark_node src/fleet_benchmark_node.cpp)
ament_target_dependencies(fleet_benchmark_node
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  edge_fleet_msgs
)

# 5. Edge Perception Node
add_executable(edge_perception_node src/edge_perception_node.cpp)
ament_target_dependencies(edge_perception_node
  rclcpp
  rclcpp_components
  sensor_msgs
  visualization_msgs
  std_msgs
)

# 6. Race Manager Node (Head-to-Head Comparative Evaluator)
add_executable(race_manager_node src/race_manager_node.cpp)
ament_target_dependencies(race_manager_node
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  edge_fleet_msgs
)

install(TARGETS
  p2p_peer_node
  conflict_resolver_node
  task_allocator_node
  fleet_benchmark_node
  edge_perception_node
  race_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
