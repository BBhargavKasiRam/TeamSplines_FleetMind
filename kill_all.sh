#!/usr/bin/env bash
# ==============================================================================
# KILL ALL: Cleanly terminates both Open-RMF Simulations, Dashboards & Visualizers
# ==============================================================================

echo "================================================================================"
echo " [STOPPING ALL] Stopping Centralized Dashboard, rmf_ws and rmf_ws_t ..."
echo "================================================================================"

# Terminate dashboard & telemetry servers
pkill -9 -f "dashboard_server.py" 2>/dev/null || true
pkill -9 -f "telemetry_server.py" 2>/dev/null || true

# Terminate ROS 2 launch parents & nodes
pkill -9 -f "ros2 launch rmf_demos" 2>/dev/null || true
pkill -9 -f "warehouse.launch.xml" 2>/dev/null || true
pkill -9 -f "fleet_adapter" 2>/dev/null || true
pkill -9 -f "fleet_manager" 2>/dev/null || true
pkill -9 -f "building_map_server" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "tile_windows.py" 2>/dev/null || true
pkill -9 -f "tile_dual_windows.py" 2>/dev/null || true

# Terminate Gazebo and RViz2 visualizers
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gz-sim-gui" 2>/dev/null || true
pkill -9 -f "gz-sim-server" 2>/dev/null || true
killall -9 rviz2 gz ruby 2>/dev/null || true

# Free HTTP and WebSocket ports if still held
fuser -k 8080/tcp 2>/dev/null || true
fuser -k 8081/tcp 2>/dev/null || true
fuser -k 8765/tcp 2>/dev/null || true
fuser -k 8006/tcp 2>/dev/null || true

sleep 1

echo "================================================================================"
echo " [✓] All Smart Warehouse simulations, dashboards, and nodes stopped."
echo " Ports 8080, 8081, and 8765 are now free."
echo "================================================================================"
