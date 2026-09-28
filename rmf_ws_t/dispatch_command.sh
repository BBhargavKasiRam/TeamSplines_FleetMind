#!/usr/bin/env bash
# ==============================================================================
# STANDARD OPEN-RMF WAREHOUSE TASK DISPATCHER
#
# Commands an individual robot or all robots in the warehouse fleet to execute
# patrol/dispatch tasks using standard Open-RMF task dispatching.
#
# Usage:
#   ./dispatch_command.sh all                                    # Dispatches 1-round benchmark patrol across all 5 robots
#   ./dispatch_command.sh all dock                               # Dispatches patrol to return each robot to its charger
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
echo " [OPEN-RMF TASK DISPATCHER] TARGET: $ROBOT"
echo "================================================================================"

get_charger() {
    case "$1" in
        tinyRobot1) echo "tinyRobot1_charger" ;;
        tinyRobot2) echo "tinyRobot2_charger" ;;
        tinyRobot3) echo "charger_gamma" ;;
        tinyRobot4) echo "charger_delta" ;;
        tinyRobot5) echo "charger_epsilon" ;;
        *) echo "tinyRobot1_charger" ;;
    esac
}

if [ "$ROBOT" = "all" ] && [ "${PLACES[0]}" = "dock" ]; then
    echo "[*] Commanding all robots to return to their chargers..."
    for r in tinyRobot1 tinyRobot2 tinyRobot3 tinyRobot4 tinyRobot5; do
        ch=$(get_charger "$r")
        ros2 run rmf_demos_tasks dispatch_patrol -p "$ch" -n 1 --use_sim_time -F tinyRobot -R "$r" &
    done
    wait
    echo "[✓] Dock commands dispatched to all robots."
    exit 0
fi

if [ "${PLACES[0]}" = "dock" ]; then
    ch=$(get_charger "$ROBOT")
    echo "[*] Commanding $ROBOT to return to charger: $ch..."
    ros2 run rmf_demos_tasks dispatch_patrol -p "$ch" -n 1 --use_sim_time -F tinyRobot -R "$ROBOT"
    echo "[✓] Dock command dispatched to $ROBOT."
    exit 0
fi

if [ "$ROBOT" = "all" ] && [ ${#PLACES[@]} -eq 0 ]; then
    echo "[*] Dispatching standardized 1-round patrol to all 5 robots..."
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

echo "[*] Dispatching standard patrol task to $ROBOT: ${PLACES[*]}..."
ros2 run rmf_demos_tasks dispatch_patrol -p "${PLACES[@]}" -n 1 --use_sim_time -F tinyRobot -R "$ROBOT"

echo "================================================================================"
echo " [✓] Standard task successfully dispatched to $ROBOT."
echo "================================================================================"
