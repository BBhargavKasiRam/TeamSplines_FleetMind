#!/usr/bin/env bash
set -e

pkill -9 -f dashboard_server.py 2>/dev/null || true
pkill -9 -f telemetry_server.py 2>/dev/null || true
sleep 1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="${FLEETMIND_DIR:-$SCRIPT_DIR}"
EDGE_WS="$BASE_DIR/rmf_ws"
TRAD_WS="$BASE_DIR/rmf_ws_t"
WEB_DASH="$BASE_DIR/web_dashboard"

nohup setsid bash -c '
  export ROS_DOMAIN_ID=20
  source /opt/ros/jazzy/setup.bash
  source "'"$TRAD_WS"'/install/setup.bash"
  exec python3 -u "'"$TRAD_WS"'/telemetry_server.py"
' </dev/null > "$TRAD_WS/telemetry.log" 2>&1 &
TRAD_PID=$!
disown $TRAD_PID 2>/dev/null || true

nohup setsid bash -c '
  export ROS_DOMAIN_ID=10
  export TRADITIONAL_API_URL="http://localhost:8081/api/status"
  export FLEETMIND_DIR="'"$BASE_DIR"'"
  source /opt/ros/jazzy/setup.bash
  source "'"$EDGE_WS"'/install/setup.bash"
  exec python3 -u "'"$WEB_DASH"'/dashboard_server.py"
' </dev/null > "$WEB_DASH/dashboard.log" 2>&1 &
DASH_PID=$!
disown $DASH_PID 2>/dev/null || true

echo "Started Traditional Telemetry (PID $TRAD_PID) and Central Dashboard (PID $DASH_PID)"
