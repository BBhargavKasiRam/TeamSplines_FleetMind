#!/usr/bin/env bash
# ==============================================================================
# STANDARDIZED BENCHMARK WORKLOAD DISPATCHER
#
# Dispatches identical symmetric patrol missions to ALL available robots:
#  - tinyRobot1: loading_dock <-> Rack_F_02 <-> Rack_E_02
#  - tinyRobot2: packing_01 <-> Rack_East_04 <-> Rack_East_01
#
# Usage:
#   ./dispatch_benchmark_tasks.sh [NUM_ROUNDS]   (Default: 1 round)
# ==============================================================================

set -e

ROUNDS=${1:-1}

echo "================================================================================"
echo " [BENCHMARK SUITE] DISPATCHING STANDARDIZED TASKS ($ROUNDS ROUNDS)"
echo " All AMRs will execute identical, repeatable intersecting trajectories."
echo "================================================================================"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

echo "[*] Dispatching Task 1 to tinyRobot1 (loading_dock -> Rack_F_02 -> Rack_E_02 -> tinyRobot1_charger)..."
ros2 run rmf_demos_tasks dispatch_patrol -p loading_dock Rack_F_02 Rack_E_02 tinyRobot1_charger -n $ROUNDS --use_sim_time -F tinyRobot -R tinyRobot1 &
PID1=$!

sleep 0.4

echo "[*] Dispatching Task 2 to tinyRobot2 (packing_01 -> Rack_East_04 -> Rack_East_01 -> tinyRobot2_charger)..."
ros2 run rmf_demos_tasks dispatch_patrol -p packing_01 Rack_East_04 Rack_East_01 tinyRobot2_charger -n $ROUNDS --use_sim_time -F tinyRobot -R tinyRobot2 &
PID2=$!

sleep 0.4

echo "[*] Dispatching Task 3 to tinyRobot3 (loading_area -> Rack_D_02 -> packing_02 -> charger_gamma)..."
ros2 run rmf_demos_tasks dispatch_patrol -p loading_area Rack_D_02 packing_02 charger_gamma -n $ROUNDS --use_sim_time -F tinyRobot -R tinyRobot3 &
PID3=$!

sleep 0.4

echo "[*] Dispatching Task 4 to tinyRobot4 (shipping_area -> Rack_C_02 -> Rack_B_02 -> charger_delta)..."
ros2 run rmf_demos_tasks dispatch_patrol -p shipping_area Rack_C_02 Rack_B_02 charger_delta -n $ROUNDS --use_sim_time -F tinyRobot -R tinyRobot4 &
PID4=$!

sleep 0.4

echo "[*] Dispatching Task 5 to tinyRobot5 (packing_03 -> Rack_A_02 -> storage_area -> charger_epsilon)..."
ros2 run rmf_demos_tasks dispatch_patrol -p packing_03 Rack_A_02 storage_area charger_epsilon -n $ROUNDS --use_sim_time -F tinyRobot -R tinyRobot5 &
PID5=$!

wait $PID1 $PID2 $PID3 $PID4 $PID5

echo "================================================================================"
echo " [✓] Standardized tasks successfully queued for ALL 5 AMRs!"
echo " Monitor live performance on Web Dashboard: http://localhost:8080"
echo "================================================================================"
