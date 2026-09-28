#!/usr/bin/env bash
set -e

pkill -9 -f dashboard_server.py 2>/dev/null || true
pkill -9 -f telemetry_server.py 2>/dev/null || true
sleep 1

SIH_DIR="/home/manoj/SIH"
EDGE_WS="$SIH_DIR/rmf_ws"
TRAD_WS="$SIH_DIR/rmf_ws_t"
WEB_DASH="$SIH_DIR/web_dashboard"

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
  export SIH_DIR="'"$SIH_DIR"'"
  source /opt/ros/jazzy/setup.bash
  source "'"$EDGE_WS"'/install/setup.bash"
  exec python3 -u "'"$WEB_DASH"'/dashboard_server.py"
' </dev/null > "$WEB_DASH/dashboard.log" 2>&1 &
DASH_PID=$!
disown $DASH_PID 2>/dev/null || true

echo "Started Traditional Telemetry (PID $TRAD_PID) and Central Dashboard (PID $DASH_PID)"
