cmake_minimum_required(VERSION 3.8)
project(edge_fleet_bringup)

find_package(ament_cmake REQUIRED)

# Install python scripts as executables
install(PROGRAMS
  scripts/simulated_amr.py
  scripts/inject_blockage.py
  scripts/dashboard_server.py
  scripts/run_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, dashboard frontend UI, and RViz config
install(DIRECTORY launch dashboard_ui rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
