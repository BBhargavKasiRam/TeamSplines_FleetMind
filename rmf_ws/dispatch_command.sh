#!/usr/bin/env bash
# ==============================================================================
# ON-DEMAND MULTI-ROBOT COMMAND DISPATCHER
#
# Commands an individual robot or the entire fleet to execute a single task.
# The robot(s) execute the commanded mission and then remain idle awaiting the
# next user command. No continuous auto-looping.
#
# Usage:
#   ./dispatch_command.sh all                                    # Dispatches 1-round benchmark workload across all 5 AMRs
#   ./dispatch_command.sh all dock                               # Commands all robots to return to home chargers
#   ./dispatch_command.sh tinyRobot1 loading_dock Rack_F_02      # Commands tinyRobot1 through specified waypoints
#   ./dispatch_command.sh tinyRobot1 dock                        # Commands tinyRobot1 to return to its charger
# ==============================================================================

set -e

ROBOT="${1:-all}"
shift || true
PLACES=("$@")

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

echo "================================================================================"
echo " [COMMAND DISPATCHER] TARGET: $ROBOT"
echo "================================================================================"

if [ "$ROBOT" = "all" ] && [ "${PLACES[0]}" = "dock" ]; then
    echo "[*] Commanding all robots to return and dock at charging bays..."
    curl -s -X POST http://localhost:8080/api/return_to_charger \
         -H "Content-Type: application/json" \
         -d '{"robot": "all"}' | grep -o '"message":[^}]*' || true
    echo ""
    exit 0
fi

if [ "${PLACES[0]}" = "dock" ]; then
    echo "[*] Commanding $ROBOT to return and dock at its charging bay..."
    curl -s -X POST http://localhost:8080/api/return_to_charger \
         -H "Content-Type: application/json" \
         -d "{\"robot\": \"$ROBOT\"}" | grep -o '"message":[^}]*' || true
    echo ""
    exit 0
fi

if [ "$ROBOT" = "all" ] && [ ${#PLACES[@]} -eq 0 ]; then
    echo "[*] Dispatching standardized 1-round benchmark patrol to ALL 5 AMRs..."
    ./dispatch_benchmark_tasks.sh 1
    exit 0
fi

if [ ${#PLACES[@]} -eq 0 ]; then
    case "$ROBOT" in
        tinyRobot1) PLACES=("loading_dock" "Rack_F_02" "Rack_E_02" "tinyRobot1_charger") ;;
        tinyRobot2) PLACES=("packing_01" "Rack_East_04" "Rack_East_01" "tinyRobot2_charger") ;;
        tinyRobot3) PLACES=("loading_area" "Rack_D_02" "packing_02" "charger_gamma") ;;
        tinyRobot4) PLACES=("shipping_area" "Rack_C_02" "Rack_B_02" "charger_delta") ;;
        tinyRobot5) PLACES=("packing_03" "Rack_A_02" "storage_area" "charger_epsilon") ;;
        *) PLACES=("loading_dock" "Rack_F_02" "tinyRobot1_charger") ;;
    esac
fi

echo "[*] Dispatching single-run patrol command to $ROBOT: ${PLACES[*]}..."
ros2 run rmf_demos_tasks dispatch_patrol -p "${PLACES[@]}" -n 1 --use_sim_time -F tinyRobot -R "$ROBOT"

echo "================================================================================"
echo " [✓] Command dispatched to $ROBOT. Robot will complete mission and remain idle."
echo "================================================================================"
