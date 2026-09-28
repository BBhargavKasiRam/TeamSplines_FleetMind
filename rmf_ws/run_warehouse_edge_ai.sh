#!/usr/bin/env bash
# ==============================================================================
# BHARAT ELECTRONICS (BEL) - OPEN-RMF SMART WAREHOUSE BENCHMARK SUITE
# MODE 1: DECENTRALIZED EDGE-AI MODE (ALL ROBOTS)
#
# Launches the 4x Expanded Warehouse with ALL available AMRs running under:
#  - Decentralized Peer-to-Peer (P2P) Mesh Intent Sharing
#  - Local ORCA / Dynamic Velocity Modulation (0.4 m/s crawl at intersections)
#  - Immediate Deadlock Override (Zero Stop-and-Wait Standstills)
# ==============================================================================

set -e

echo "================================================================================"
echo " [LAUNCHING MODE 1] WAREHOUSE SIMULATION: DECENTRALIZED EDGE-AI MODE"
echo " Configuration: ALL ROBOTS IN FLEET WILL OPERATE UNDER DECENTRALIZED EDGE-AI"
echo "================================================================================"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

export DISPLAY=${DISPLAY:-:1}
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export PATH="$HOME/.local/bin:$PATH"
export SDF_PATH="$WS_DIR/install/rmf_demos_assets/share/rmf_demos_assets/models:$WS_DIR/install/rmf_demos_maps/share/rmf_demos_maps/maps/warehouse/models:$HOME/.gazebo/models:${SDF_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$WS_DIR/install/rmf_demos_assets/share/rmf_demos_assets/models:$WS_DIR/install/rmf_demos_maps/share/rmf_demos_maps/maps/warehouse/models:$HOME/.gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

# 1. Clean up any previous simulator and node instances
"$WS_DIR/kill_sim.sh" >/dev/null 2>&1 || true
sleep 1

# 2. Start Fleet Dashboard in EDGE_AI mode
python3 -u "$WS_DIR/src/edge_fleet_bringup/scripts/dashboard_server.py" --ros-args -p mode:=EDGE_AI > /tmp/dashboard_server.log 2>&1 &
echo "[*] Live Fleet Dashboard Server started on http://localhost:8080 (MODE: EDGE_AI)"

# 3. Launch 4x Warehouse in EDGE_AI mode
# Automatically tiles Gazebo Sim (Left) and RViz2 (Right) side-by-side on display
python3 "$WS_DIR/src/edge_fleet_bringup/scripts/tile_windows.py" >/dev/null 2>&1 &

echo "[*] Launching Gazebo Sim & RViz2 in Decentralized Edge-AI Mode..."
ros2 launch rmf_demos_gz warehouse.launch.xml mode:=edge_ai

