#!/usr/bin/env bash
# ==============================================================================
# UNIFIED MULTI-WORKSPACE LAUNCHER (run_all.sh)
#
# Starts all three components simultaneously in isolated domains & partitions:
#   1. Centralized Fleet Dashboard & Comparison Gateway (Port 8080 & WS 8765)
#   2. Traditional Open-RMF Simulation: rmf_ws_t (ROS Domain 20, Gazebo: traditional)
#   3. Decentralized Edge-AI Simulation: rmf_ws (ROS Domain 10, Gazebo: edge_ai)
#
# Usage:
#   ./run_all.sh           # Starts all 3 systems in foreground with live health monitoring (Ctrl+C stops all)
#   ./run_all.sh --daemon  # Starts all 3 systems detached in background
# ==============================================================================

set -e

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EDGE_WS="$BASE_DIR/rmf_ws"
TRAD_WS="$BASE_DIR/rmf_ws_t"
WEB_DASH="$BASE_DIR/web_dashboard"

DAEMON_MODE=0
if [ "$1" = "--daemon" ] || [ "$1" = "-d" ]; then
  DAEMON_MODE=1
fi

echo "================================================================================"
echo " 🚀 LAUNCHING SMART WAREHOUSE DUAL-FLEET BENCHMARK SUITE"
echo " Root Folder : $BASE_DIR"
echo " Components  : Central Dashboard (8080) | Edge-AI (rmf_ws) | Traditional (rmf_ws_t)"
echo "================================================================================"

# 1. Clean up any previous runs
echo "[1/4] Stopping any previous simulations and servers..."
"$BASE_DIR/kill_all.sh" >/dev/null 2>&1 || true
sleep 1

# Common NVIDIA GPU & display settings
export DISPLAY=${DISPLAY:-:1}
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export QT_QPA_PLATFORM=xcb
export GZ_RENDERING_ENGINE=ogre2
export PATH="$HOME/.local/bin:$PATH"

# 2. Start Centralized Dashboard & Gateway (web_dashboard on port 8080)
echo "[2/4] Starting Centralized Comparison Dashboard on port 8080 & 8765..."
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
echo "      -> Dashboard process started (PID: $DASH_PID, logs: $WEB_DASH/dashboard.log)"

# 3. Start Traditional Open-RMF Simulation & Telemetry Server (rmf_ws_t)
echo "[3/4] Launching Traditional Open-RMF Warehouse (rmf_ws_t) on ROS Domain 20..."
nohup setsid bash -c '
  export ROS_DOMAIN_ID=20
  source /opt/ros/jazzy/setup.bash
  source "'"$TRAD_WS"'/install/setup.bash"
  exec python3 -u "'"$TRAD_WS"'/telemetry_server.py"
' </dev/null > "$TRAD_WS/telemetry.log" 2>&1 &
TRAD_TELEMETRY_PID=$!
disown $TRAD_TELEMETRY_PID 2>/dev/null || true

nohup setsid bash -c '
  export ROS_DOMAIN_ID=20
  export GZ_PARTITION=traditional
  export GZ_IP=127.0.0.1
  export SDF_PATH="'"$TRAD_WS"'/install/rmf_demos_assets/share/rmf_demos_assets/models:'"$TRAD_WS"'/install/rmf_demos_maps/share/rmf_demos_maps/maps/warehouse/models:$HOME/.gazebo/models:${SDF_PATH:-}"
  export GZ_SIM_RESOURCE_PATH="'"$TRAD_WS"'/install/rmf_demos_assets/share/rmf_demos_assets/models:'"$TRAD_WS"'/install/rmf_demos_maps/share/rmf_demos_maps/maps/warehouse/models:$HOME/.gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
  source /opt/ros/jazzy/setup.bash
  source "'"$TRAD_WS"'/install/setup.bash"
  exec ros2 launch rmf_demos_gz warehouse.launch.xml
' </dev/null > "$TRAD_WS/simulation.log" 2>&1 &
TRAD_SIM_PID=$!
disown $TRAD_SIM_PID 2>/dev/null || true
echo "      -> Traditional Telemetry API started on port 8081 (PID: $TRAD_TELEMETRY_PID)"
echo "      -> Traditional Gazebo & RViz2 launched (PID: $TRAD_SIM_PID, logs: $TRAD_WS/simulation.log)"

sleep 3

# 4. Start Decentralized Edge-AI Simulation (rmf_ws)
echo "[4/4] Launching Decentralized Edge-AI Warehouse (rmf_ws) on ROS Domain 10..."
nohup setsid bash -c '
  export ROS_DOMAIN_ID=10
  export GZ_PARTITION=edge_ai
  export GZ_IP=127.0.0.1
  export SDF_PATH="'"$EDGE_WS"'/install/rmf_demos_assets/share/rmf_demos_assets/models:'"$EDGE_WS"'/install/rmf_demos_maps/share/rmf_demos_maps/maps/warehouse/models:$HOME/.gazebo/models:${SDF_PATH:-}"
  export GZ_SIM_RESOURCE_PATH="'"$EDGE_WS"'/install/rmf_demos_assets/share/rmf_demos_assets/models:'"$EDGE_WS"'/install/rmf_demos_maps/share/rmf_demos_maps/maps/warehouse/models:$HOME/.gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
  source /opt/ros/jazzy/setup.bash
  source "'"$EDGE_WS"'/install/setup.bash"
  exec ros2 launch rmf_demos_gz warehouse.launch.xml mode:=edge_ai use_edge_ai:=true
' </dev/null > "$EDGE_WS/simulation.log" 2>&1 &
EDGE_SIM_PID=$!
disown $EDGE_SIM_PID 2>/dev/null || true
echo "      -> Edge-AI Gazebo & RViz2 launched (PID: $EDGE_SIM_PID, logs: $EDGE_WS/simulation.log)"

# Auto-tile visualizer windows in background
nohup setsid python3 "$BASE_DIR/tile_dual_windows.py" </dev/null >/dev/null 2>&1 &

echo ""
echo "================================================================================"
echo " ⏳ WAITING FOR SERVERS AND SIMULATION BRIDGES TO INITIALIZE..."
echo "================================================================================"

for i in $(seq 1 35); do
  DASH_OK=0
  TRAD_OK=0

  if curl -s -m 1 http://localhost:8080/ >/dev/null 2>&1; then
    DASH_OK=1
  fi
  if curl -s -m 1 http://localhost:8081/api/health >/dev/null 2>&1; then
    TRAD_OK=1
  fi

  if [ $DASH_OK -eq 1 ] && [ $TRAD_OK -eq 1 ]; then
    echo " [✓] Central Dashboard (Port 8080) is ONLINE"
    echo " [✓] Traditional Telemetry Gateway (Port 8081) is ONLINE"
    break
  fi

  # Auto-start Edge-AI fleet adapter once traffic schedule has initialized
  if [ $i -ge 4 ] && ! pgrep -f "rmf_ws/install/rmf_demos_fleet_adapter/lib/rmf_demos_fleet_adapter/fleet_adapter" >/dev/null 2>&1; then
    nohup setsid bash -c '
      export ROS_DOMAIN_ID=10
      export GZ_PARTITION=edge_ai
      export GZ_IP=127.0.0.1
      source /opt/ros/jazzy/setup.bash
      source "'"$EDGE_WS"'/install/setup.bash"
      exec python3 "'"$EDGE_WS"'/install/rmf_demos_fleet_adapter/lib/rmf_demos_fleet_adapter/fleet_adapter" -c "'"$EDGE_WS"'/install/rmf_demos/share/rmf_demos/config/warehouse/tinyRobot_config.yaml" -n "'"$EDGE_WS"'/install/rmf_demos_maps/share/rmf_demos_maps/maps/warehouse/nav_graphs/0.yaml" -sim --use_sim_time
    ' </dev/null > "$EDGE_WS/fleet_adapter_edge.log" 2>&1 &
  fi

  printf "   Waiting for services... (%ds/35s)\r" "$i"
  sleep 1
done
echo ""

# Ensure Edge-AI fleet adapter is started if schedule took longer
if ! pgrep -f "rmf_ws/install/rmf_demos_fleet_adapter/lib/rmf_demos_fleet_adapter/fleet_adapter" >/dev/null 2>&1; then
  nohup setsid bash -c '
    export ROS_DOMAIN_ID=10
    export GZ_PARTITION=edge_ai
    export GZ_IP=127.0.0.1
    source /opt/ros/jazzy/setup.bash
    source "'"$EDGE_WS"'/install/setup.bash"
    exec python3 "'"$EDGE_WS"'/install/rmf_demos_fleet_adapter/lib/rmf_demos_fleet_adapter/fleet_adapter" -c "'"$EDGE_WS"'/install/rmf_demos/share/rmf_demos/config/warehouse/tinyRobot_config.yaml" -n "'"$EDGE_WS"'/install/rmf_demos_maps/share/rmf_demos_maps/maps/warehouse/nav_graphs/0.yaml" -sim --use_sim_time
  ' </dev/null > "$EDGE_WS/fleet_adapter_edge.log" 2>&1 &
fi

# Automatically open Web Dashboard in browser
export DISPLAY=${DISPLAY:-:1}
nohup setsid google-chrome http://localhost:8080/ </dev/null >/dev/null 2>&1 &

# Auto-tile visualizer and dashboard windows
nohup setsid python3 "$BASE_DIR/tile_dual_windows.py" </dev/null >/dev/null 2>&1 &

echo "================================================================================"
echo " 🎉 ALL 3 SYSTEMS ARE RUNNING AND FULLY CONNECTED!"
echo "================================================================================"
echo " 1. CENTRALIZED WEB DASHBOARD : http://localhost:8080"
echo " 2. TRADITIONAL TELEMETRY API : http://localhost:8081/api/status"
echo " 3. COMPARISON SCALE API      : http://localhost:8080/api/status"
echo " 4. SQLITE DATABASE           : $WEB_DASH/benchmark_history.db"
echo ""
echo " 🚀 HOW TO ASSIGN TASKS TO BOTH WAREHOUSES SIMULTANEOUSLY:"
echo "    Option A (CLI): ./dispatch_dual_tasks.sh benchmark all"
echo "    Option B (CLI): ./dispatch_dual_tasks.sh putaway all"
echo "    Option C (Web): Click [🚀 DISPATCH COMMAND] or [▶ Run Edge-AI Mission Benchmark] in Dashboard"
echo ""
echo " 🛑 TO STOP EVERYTHING:"
echo "    ./kill_all.sh"
echo "================================================================================"

if [ $DAEMON_MODE -eq 1 ]; then
  echo "[*] Daemon mode active. All processes running in background."
  exit 0
fi

echo "[*] Keeping launcher active. Press Ctrl+C at any time to cleanly stop all systems."
cleanup() {
  echo ""
  echo "[*] Signal received. Terminating all services..."
  "$BASE_DIR/kill_all.sh"
  exit 0
}
trap cleanup INT TERM

# Keep monitoring status
while true; do
  sleep 2
done
