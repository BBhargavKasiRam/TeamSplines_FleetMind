#!/usr/bin/env bash
# ==============================================================================
# Open-RMF Office Simulation with NVIDIA Hardware Acceleration
# ==============================================================================

set -e

source /opt/ros/jazzy/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/install/setup.bash"

export DISPLAY=${DISPLAY:-:1}
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

echo "[*] Launching Open-RMF Office demo with NVIDIA GPU acceleration..."
ros2 launch rmf_demos_gz office.launch.xml "$@"
