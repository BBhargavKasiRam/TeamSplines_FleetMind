# Autonomous 3D Warehouse Robot Simulation (ROS 2 Jazzy + Gazebo Sim)

A realistic, high-fidelity 3D autonomous warehouse simulation environment built from scratch with **ROS 2 Jazzy** and **Gazebo Sim**. This project features multi-AMR operations, procedurally configured storage infrastructure, dynamic task lifecycles, and a clean **Logic Adapter** interface designed for seamless plug-in of external multi-robot coordination and deadlock resolution algorithms.

> [!NOTE]
> **Collaborator & Logic Testing Notice**:
> This demo warehouse was built specifically to test and benchmark multi-AMR coordination logic, task allocation, and deadlock resolution algorithms. If you are developing or testing the dispatch and fleet coordination logic, see [Section 10: Connecting External / Collaborator Logic](#10-connecting-external--collaborator-logic). The built-in mock controller can be disabled (`use_mock_controller:=false`) to give your external algorithm direct closed-loop control of all AMRs.

---

## Architecture Overview

```text
                 ┌────────────────────────────────────────────────────────┐
                 │                  EXTERNAL LOGIC                        │
                 │  - Priority Scheduling   - Task Allocation             │
                 │  - Path Planning         - Deadlock Detection/Resolv   │
                 │  - Fleet Coordination    - Yielding Rules              │
                 └───────────────────────────┬────────────────────────────┘
                                             │
                       ROS 2 Topics / Python API Client
                                             │
                                             ▼
                 ┌────────────────────────────────────────────────────────┐
                 │                WAREHOUSE LOGIC ADAPTER                 │
                 │  - /logic/<robot>/cmd_vel, waypoint, path              │
                 │  - /logic/<robot>/control (WAIT, RESUME, YIELD, STOP)  │
                 │  - /warehouse/assign_task Service / Topic              │
                 │  - Deadlock & Proximity Stream                         │
                 └───────────────────────────┬────────────────────────────┘
                                             │
                               ROS 2 Node / Topic Layer
                                             │
                                             ▼
                 ┌────────────────────────────────────────────────────────┐
                 │                  ROS-GZ PARAMETER BRIDGE               │
                 │  Translates ROS 2 standard messages <-> Gazebo Sim gz  │
                 └───────────────────────────┬────────────────────────────┘
                                             │
          ┌──────────────────────────────────┼──────────────────────────────────┐
          ▼                                  ▼                                  ▼
     Robot Alpha                        Robot Beta                       Robot Gamma ...
  (/robot_alpha/*)                   (/robot_beta/*)                  (/robot_gamma/*)
          │                                  │                                  │
          └──────────────────────────────────┼──────────────────────────────────┘
                                             │
                                             ▼
                 ┌────────────────────────────────────────────────────────┐
                 │                    GAZEBO SIM WORLD                    │
                 │  30m x 20m Warehouse | 6 Rack Rows | 30+ Packages      │
                 │  AMR Fleet (Alpha..Epsilon) | Docks | Packing Stations │
                 └────────────────────────────────────────────────────────┘
```

---

## 1. Installation & Environment

The simulation runs natively in ROS 2 Jazzy or within the pre-configured Docker container `warehouse_3d_sim` (image: `warehouse_jazzy_base`).

### Prerequisites
- **ROS 2 Jazzy Jalisco** (or `ros:jazzy-desktop`)
- **Gazebo Sim** (Harmonic / gz-sim)
- `ros-jazzy-ros-gz` (or `ros-jazzy-ros-gz-sim`, `ros-jazzy-ros-gz-bridge`)
- `xacro`, `robot-state-publisher`, `rviz2`

### Container Setup (if running in Docker)
```bash
docker start warehouse_3d_sim
docker exec -it warehouse_3d_sim bash
```

---

## 2. Build Instructions

Navigate to the ROS 2 workspace:
```bash
cd /workspace/warehouse_3d/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

All 8 packages will compile:
1. `warehouse_3d_interfaces`: Custom message, service, and action definitions.
2. `warehouse_3d_description`: AMR URDF/Xacro models with parameterizable accents.
3. `warehouse_3d_world`: SDF world generator and 3D warehouse world file.
4. `warehouse_3d_robot`: Robot spawning and TF/state broadcast system.
5. `warehouse_task_generator`: Procedural task lifecycle generator.
6. `warehouse_logic_adapter`: Coordination bridge and Python client API.
7. `warehouse_3d_visualization`: 3D visual markers for RViz2.
8. `warehouse_3d_bringup`: Launch scripts and bridge configuration.

---

## 3. Launching the Simulation

### Main Launch Command (Full GUI: Gazebo + RViz2 + Mock Demo)
```bash
cd /workspace/warehouse_3d/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch warehouse_3d_bringup simulation.launch.py
```

### Configurable Launch Arguments
You can override parameters on the command line:

```bash
ros2 launch warehouse_3d_bringup simulation.launch.py \
    robot_count:=5 \
    package_count:=30 \
    task_count:=10 \
    headless:=false \
    use_rviz:=true \
    use_mock_controller:=false
```

| Argument | Default | Description |
| :--- | :--- | :--- |
| `robot_count` | `5` | Active AMRs spawned (1 to 5: Alpha, Beta, Gamma, Delta, Epsilon). |
| `package_count` | `30` | Number of cardboard package boxes on racks. |
| `task_count` | `10` | Initial active dynamic tasks generated. |
| `headless` | `false` | Set `true` to run Gazebo headlessly (server only, no GUI). |
| `use_rviz` | `true` | Set `true` to launch RViz2 with pre-configured visualization. |
| `use_mock_controller` | `true` | Set `false` when connecting your friend's external logic. |
| `use_sim_time` | `true` | Synchronize all nodes with Gazebo `/clock`. |

---

## 4. Warehouse 3D Structure

The warehouse environment (`/workspace/warehouse_3d/ros2_ws/src/warehouse_3d_world/worlds/warehouse.sdf`) is 30m × 20m × 6m and contains:
- **Outer Industrial Walls**: Durable textured perimeter boundaries.
- **6 Storage Rack Rows** (`Rack_A` through `Rack_F`): Each rack is 10m long, multi-tiered (lower shelf z=0.5m, middle shelf z=1.3m, upper shelf z=2.1m), with safety yellow upright end frames and orange cross-beams.
- **Aisles & Bottlenecks**: Longitudinal aisles (2.2m clearance), narrow connecting cross-aisles (1.4m clearance), and dead-end corners to test traffic routing and deadlock resolution.
- **30+ Biscuit-Colored Package Boxes** (`P001` - `P030+`): 0.35m × 0.28m × 0.22m standard cardboard textured parcels resting on shelves and wooden pallets.
- **Workstations & Zones**:
  - **Dock Zones**: Dock_01 and Dock_02 (Inbound goods receiving).
  - **Packing Stations**: Packing_01, Packing_02, Packing_03 (Outbound fulfillment).
  - **AMR Charging Bays**: 5 dedicated docking bays for Alpha through Epsilon.
- **Ceiling Trusses & Lighting**: Overhead roof trusses and high-bay industrial point lights for realistic illumination.

---

## 5. Robot Structure (AMR Fleet)

Each AMR model (`warehouse_amr.urdf.xacro`) is a heavy-duty industrial differential drive platform (0.72m × 0.52m × 0.26m, 25kg) equipped with:
- **Differential Drive Actuation**: High-torque drive wheels and passive omnidirectional caster wheels.
- **Visual Identity System**:
  - **Alpha**: Sapphire Blue (`robot_alpha`)
  - **Beta**: Crimson Red (`robot_beta`)
  - **Gamma**: Emerald Green (`robot_gamma`)
  - **Delta**: Industrial Amber Yellow (`robot_delta`)
  - **Epsilon**: Royal Purple (`robot_epsilon`)
  - Visual identity plates and LED status beacon on top.
- **Sensor Suite**:
  - **360° 2D LiDAR**: 12m range, 0.05m minimum distance, published on `/<robot>/scan`.
  - **Forward Depth / Color Camera**: 80° FOV, published on `/<robot>/camera/image_raw`.
  - **IMU**: 6-DOF angular velocity & acceleration on `/<robot>/imu`.
  - **Wheel Encoders**: Odometry stream on `/<robot>/odom`.

---

## 6. Dynamic Task Generation

Tasks are managed dynamically by the `warehouse_task_generator` node:
- **Task Attributes**: Unique ID (`T001`, `T002`...), Task Priority (`CRITICAL`, `HIGH`, `MEDIUM`, `LOW`), Package ID (`P001` - `P030`), Pickup Location (with shelf level), Destination Station, Timestamp, Deadline.
- **Procedural Distribution**: Tasks are generated across different rack slots and fulfillment stations to ensure realistic multi-robot contention.
- **Task Summary Stream**: A human-readable and machine-parsable queue published on `/warehouse/task_summary` and `/warehouse/task_stream`.

---

## 7. ROS 2 Interfaces (`warehouse_3d_interfaces`)

Custom messages and services defined in `warehouse_3d_interfaces`:
- `RobotState.msg`: Comprehensive AMR state (pose, velocity, battery %, assigned task, carrying package ID, status).
- `WarehouseTask.msg`: Full task specification (priority enum, locations, status).
- `RobotTaskAssignment.msg`: Assignment dispatch message (`ACTION_ASSIGN`, `ACTION_CANCEL`, `ACTION_PAUSE`).
- `DeadlockStatus.msg`: Real-time deadlock warning containing conflicting robot IDs and coordinates.
- `WarehouseLocation.msg`: Standard coordinates for warehouse waypoints.
- `AssignTask.srv`: Service to allocate a task to an AMR.
- `SetRobotState.srv`: Service to override robot execution state.

---

## 8. ROS Topics Directory

### Fleet-Wide Topics
- `/warehouse/fleet_state`: Complete JSON snapshot of all robots' coordinates, velocities, and statuses.
- `/warehouse/task_stream`: Active and pending `WarehouseTask` stream.
- `/warehouse/task_summary`: ASCII dashboard of active warehouse operations.
- `/warehouse/deadlock_status`: Real-time alerts when two or more robots encounter spatial conflicts.

### Per-Robot Simulation Topics (e.g. `robot_alpha`)
- `/<ns>/cmd_vel`: Velocity input directly sent to Gazebo differential drive plugin.
- `/<ns>/odom`: Odometry state from Gazebo physics.
- `/<ns>/scan`: LaserScan data from the 360° LiDAR.
- `/<ns>/camera/image_raw`: RGB visual feed from the front sensor.
- `/<ns>/imu`: Accelerometer and gyroscope data.
- `/<ns>/robot_state`: High-level aggregated AMR state (`warehouse_3d_interfaces/msg/RobotState`).

---

## 9. Logic Adapter System

The `warehouse_logic_adapter` node acts as an intelligent intermediary between your friend's external scheduling/planning algorithms and the low-level simulation.

### Available Control Modalities (Options A - D)
1. **Option A: Direct Velocity (`Twist`)**
   - Publish to `/logic/<ns>/cmd_vel`. The adapter forwards directly to the robot's physical actuators.
2. **Option B: Waypoint Navigation (`PoseStamped`)**
   - Publish to `/logic/<ns>/waypoint`. The adapter's closed-loop proportional controller drives the robot to the specified `(x, y)` coordinate with yaw alignment.
3. **Option C: Trajectory / Nav Path (`Path`)**
   - Publish to `/logic/<ns>/path`. The adapter follows the sequential list of waypoints.
4. **Option D: Task-Level Dispatch (`RobotTaskAssignment`)**
   - Publish to `/logic/assign_task` or call `/warehouse/assign_task` service.

### Coordination & Deadlock Resolution Signals
Publish a `std_msgs/String` to `/logic/<ns>/control`:
- `"WAIT"` / `"PAUSE"`: Halts robot motion immediately while preserving current target.
- `"RESUME"`: Resumes path tracking.
- `"YIELD"`: Decelerates and stops to give way to higher-priority robots.
- `"STOP"`: Cancels current motion target.
- `"REROUTE"`: Signals that an alternative path is incoming.

---

## 10. Connecting External / Collaborator Logic

This demo warehouse was designed specifically so that an external testing and coordination logic (e.g. multi-agent path planning, conflict-based search, priority scheduling, or deadlock resolution) can be plugged in directly with zero friction.

### Quick Start for Collaborators / Logic Developers
1. **Clone & Build the Workspace**:
   ```bash
   git clone <repo-url>
   cd WHS/ros2_ws   # or cd ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```
2. **Launch the Demo Simulation (Headless or GUI)**:
   Disable the built-in mock controller so your algorithm takes full control:
   ```bash
   ros2 launch warehouse_3d_bringup simulation.launch.py use_mock_controller:=false
   ```
   *(For headless/server testing, add `headless:=true use_rviz:=false`)*

### Integration Option 1: Python Client API (`WarehouseLogicClient`)
Your logic script can simply import the high-level Python API without dealing directly with lower-level ROS 2 boilerplates:

```python
import rclpy
from warehouse_logic_adapter.api_client import WarehouseLogicClient

rclpy.init()
client = WarehouseLogicClient()

# 1. Inspect fleet poses and active dynamic tasks
states = client.get_all_robot_states()
tasks = client.get_tasks()

for robot_id, state in states.items():
    print(f"Robot {robot_id} at ({state.current_pose.position.x:.2f}, {state.current_pose.position.y:.2f})")

# 2. Check for spatial conflicts and deadlocks
deadlock = client.get_deadlock_status()
if deadlock and deadlock.deadlock_detected:
    print(f"Conflict detected between: {deadlock.involved_robots}")
    # Instruct conflicting AMR to yield or wait:
    client.send_control("robot_beta", "YIELD")

# 3. Dispatch an AMR to a goal coordinate (X, Y, Yaw):
client.send_waypoint("robot_alpha", x=-2.0, y=4.5, yaw=0.0)

# Or send direct velocity commands:
# client.send_velocity("robot_alpha", linear_x=0.5, angular_z=0.0)
```

### Integration Option 2: Native ROS 2 Topics & Services
If your logic is implemented as a ROS 2 node (C++ or Python):
- **Inputs to Your Logic**:
  - `/warehouse/fleet_state` (`std_msgs/String` / JSON snapshot of all AMRs)
  - `/warehouse/task_stream` (`warehouse_3d_interfaces/msg/WarehouseTask`)
  - `/warehouse/deadlock_status` (`warehouse_3d_interfaces/msg/DeadlockStatus`)
  - `/<robot_namespace>/scan`, `odom`, `camera/image_raw`
- **Outputs from Your Logic**:
  - `/<robot_namespace>/cmd_vel` or `/logic/<robot_namespace>/cmd_vel` (`geometry_msgs/msg/Twist`)
  - `/logic/<robot_namespace>/waypoint` (`geometry_msgs/msg/PoseStamped`)
  - `/logic/<robot_namespace>/control` (`std_msgs/msg/String`: `"WAIT"`, `"RESUME"`, `"YIELD"`, `"STOP"`, `"REROUTE"`)
  - `/warehouse/assign_task` (`warehouse_3d_interfaces/srv/AssignTask`)

---

## 11. How Data Enters and Leaves Gazebo

```text
[Friend's Logic] 
       │ 
       ▼
   /logic/<ns>/waypoint or cmd_vel (ROS 2)
       │
       ▼
[warehouse_logic_adapter]
       │
       ▼
   /<ns>/cmd_vel (geometry_msgs/Twist)
       │
       ▼
[ros_gz_bridge (parameter_bridge)]
       │
       ▼
   /<ns>/cmd_vel (gz.msgs.Twist)
       │
       ▼
[Gazebo Differential Drive Plugin] ──> Moves Physical Wheels in Simulation World
       │
       ▼
[Gazebo Sensors: Odom, LiDAR, Camera, IMU]
       │
       ▼
   /<ns>/odom, scan, camera/image_raw, imu (gz.msgs.*)
       │
       ▼
[ros_gz_bridge (parameter_bridge)]
       │
       ▼
   /<ns>/odom, scan, camera/image_raw, imu (ROS 2 sensor_msgs / nav_msgs)
       │
       ▼
[robot_state_broadcaster] ──> Computes TF (/tf) & Publishes /<ns>/robot_state
       │
       ▼
[Friend's Logic Receives Updated State]
```

---

## 12. Customization & Extension

### Adding New Robots
1. Open `/workspace/warehouse_3d/ros2_ws/src/warehouse_3d_bringup/launch/simulation.launch.py`.
2. Add a new robot entry to `ROBOT_FLEET` (specifying name, accent color, namespace, initial pose).
3. Add corresponding topic mappings in `/workspace/warehouse_3d/ros2_ws/src/warehouse_3d_bringup/config/ros_gz_bridge.yaml`.

### Adding / Repositioning Packages
1. Open `/workspace/warehouse_3d/ros2_ws/src/warehouse_3d_world/generate_world.py`.
2. Adjust package dimensions, counts, or shelf heights in the package generation loop.
3. Re-generate the world:
   ```bash
   python3 /workspace/warehouse_3d/ros2_ws/src/warehouse_3d_world/generate_world.py
   ```

### Modifying Warehouse Dimensions
1. In `generate_world.py`, update `WAREHOUSE_LENGTH`, `WAREHOUSE_WIDTH`, and `WALL_HEIGHT`.
2. Run `python3 generate_world.py` to regenerate `warehouse.sdf`.
3. Update location coordinates in `warehouse_task_generator/warehouse_locations.py` to match the new dimensions.

---

## 13. Summary Report Checklist

- **Project Location**: `/workspace/warehouse_3d`
- **Build Status**: 8/8 packages successfully built with `colcon build --symlink-install`
- **Launch Command**: `ros2 launch warehouse_3d_bringup simulation.launch.py`
- **Robots**: 5 active AMRs (Alpha, Beta, Gamma, Delta, Epsilon) with distinct color badges and full sensor suites
- **Packages**: 30+ biscuit-colored cardboard parcel boxes placed on multi-tier racks and pallets
- **Dynamic Tasks**: 10 simultaneous tasks generated across 4 priority levels (`CRITICAL`, `HIGH`, `MEDIUM`, `LOW`)
- **Logic Adapter**: Active and tested with Options A, B, C, D and coordination signals (`WAIT`, `YIELD`, `RESUME`, `REROUTE`)
- **Gazebo Sim Status**: Verified with ODE/DART physics, LiDAR, camera, IMU, and differential drive
- **RViz2 Status**: Verified with 3D model meshes, TF coordinate frames, laser scans, and warehouse markers
