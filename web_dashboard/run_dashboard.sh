#!/usr/bin/env bash
# ==============================================================================
# Centralized Fleet Comparison Web Dashboard Launcher
# ==============================================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$(cd "$DIR/.." && pwd)"

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-10}
export TRADITIONAL_API_URL=${TRADITIONAL_API_URL:-"http://localhost:8081/api/status"}
export FLEETMIND_DIR="$BASE_DIR"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi
if [ -f "$BASE_DIR/rmf_ws/install/setup.bash" ]; then
  source "$BASE_DIR/rmf_ws/install/setup.bash"
fi

echo "================================================================================"
echo " 🌐 STARTING CENTRALIZED COMPARATIVE WEB DASHBOARD"
echo " Port: 8080 (HTTP) | Port: 8765 (WebSocket)"
echo " UI  : $DIR/dashboard_ui"
echo " DB  : $DIR/benchmark_history.db"
echo "================================================================================"

exec python3 -u "$DIR/dashboard_server.py" "$@"
