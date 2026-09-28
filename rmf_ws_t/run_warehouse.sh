#!/usr/bin/env bash
# ==============================================================================
# Open-RMF Warehouse Demonstration (Standard Open-RMF Algorithm)
# ==============================================================================

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

export DISPLAY=${DISPLAY:-:1}
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

export SDF_PATH="$WS_DIR/install/rmf_demos_assets/share/rmf_demos_assets/models:$WS_DIR/install/rmf_demos_maps/share/rmf_demos_maps/maps/warehouse/models:$HOME/.gazebo/models:${SDF_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$WS_DIR/install/rmf_demos_assets/share/rmf_demos_assets/models:$WS_DIR/install/rmf_demos_maps/share/rmf_demos_maps/maps/warehouse/models:$HOME/.gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

# Clean up any existing instances
"$WS_DIR/kill_sim.sh" >/dev/null 2>&1 || true
sleep 1

# Start Traditional Telemetry API Server on port 8081
python3 "$WS_DIR/telemetry_server.py" >/tmp/rmf_ws_t_telemetry.log 2>&1 &
echo "[*] Traditional Open-RMF Telemetry API Server started on http://localhost:8081"

echo "[*] Launching Open-RMF Warehouse Demo with standard Open-RMF algorithm & NVIDIA GPU..."
ros2 launch rmf_demos_gz warehouse.launch.xml "$@"
