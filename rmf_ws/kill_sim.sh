#!/usr/bin/env bash
# Cleanly terminates all simulation, visualizer, and dashboard processes
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "dashboard_server.py" 2>/dev/null || true
pkill -9 -f "tile_windows.py" 2>/dev/null || true
pkill -9 -f "ros2 launch rmf_" 2>/dev/null || true
pkill -9 -f "lib/rmf_" 2>/dev/null || true
pkill -9 -f "fleet_adapter" 2>/dev/null || true
pkill -9 -f "fleet_manager" 2>/dev/null || true
killall -9 gz ruby rviz2 2>/dev/null || true
echo "[✓] All simulation, visualizer, and fleet nodes stopped."
