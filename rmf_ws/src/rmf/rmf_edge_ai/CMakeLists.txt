cmake_minimum_required(VERSION 3.8)
project(rmf_edge_ai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(edge_fleet_msgs REQUIRED)

# 1. Edge Perception Node (Lane Closure)
add_executable(rmf_edge_perception_node src/rmf_edge_perception_node.cpp)
ament_target_dependencies(rmf_edge_perception_node
  rclcpp
  sensor_msgs
  std_msgs
  rmf_fleet_msgs
)

# 2. Decentralized P2P Bridge Node
add_executable(rmf_edge_p2p_bridge_node src/rmf_edge_p2p_bridge_node.cpp)
ament_target_dependencies(rmf_edge_p2p_bridge_node
  rclcpp
  geometry_msgs
  rmf_fleet_msgs
  edge_fleet_msgs
)

# 3. Dynamic Conflict Resolver Node (Pause Override)
add_executable(rmf_edge_conflict_resolver_node src/rmf_edge_conflict_resolver_node.cpp)
ament_target_dependencies(rmf_edge_conflict_resolver_node
  rclcpp
  rmf_fleet_msgs
  edge_fleet_msgs
)

# 4. Decentralized Task Allocator Node (Contract Net Protocol)
add_executable(rmf_edge_task_allocator_node src/rmf_edge_task_allocator_node.cpp)
ament_target_dependencies(rmf_edge_task_allocator_node
  rclcpp
  std_msgs
  rmf_task_msgs
  edge_fleet_msgs
)

install(TARGETS
  rmf_edge_perception_node
  rmf_edge_p2p_bridge_node
  rmf_edge_conflict_resolver_node
  rmf_edge_task_allocator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
