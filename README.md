# Edge-AI Based Distributed Fleet Coordination for Autonomous Mobile Robots (AMRs) in Smart Warehouses

**Organization:** Bharat Electronics Limited (BEL)  
**Department:** Software / Smart Automation  
**Category:** Software  
**Theme:** Smart Automation  
**Platform:** ROS 2 Jazzy, Gazebo Sim (Harmonic), Python 3.12, C++20, FastAPI, Chart.js  

---

## 🌟 Executive Summary

Modern smart warehouses rely on fleets of Autonomous Mobile Robots (AMRs) to move goods rapidly and safely. Conventional fleet management systems depend entirely on a centralized cloud or on-premise server for path planning and schedule reservation. As fleet density increases, centralized architectures introduce severe operational bottlenecks:
- **High Network Latency & Round-Trip Delays:** Central servers cannot react instantaneously to dynamic changes.
- **Wi-Fi Dead-Zone Vulnerabilities:** Loss of signal in shielded aisles brings robots to an abrupt halt.
- **Single-Point-of-Failure (SPOF):** A crash or overload of the central coordinator halts the entire facility.
- **Conservative Stop-and-Wait Inefficiencies:** Centralized mutexes force robots to completely halt at intersections, leading to traffic jams and cascading delays.

This project delivers a **decentralized, Edge-AI fleet coordination and collision-avoidance framework** for multi-robot fleets (5 AMRs demonstrated) operating in a dynamic smart warehouse. The framework executes directly on edge compute nodes (e.g., Raspberry Pi 5 or NVIDIA Jetson Nano onboard each AMR), enabling robots to communicate peer-to-peer (P2P), share real-time intent, modulate velocities, resolve deadlocks, and autonomously re-route around obstacles without any central coordinator.

To prove performance, the platform includes a **Unified Dual-Fleet Comparative Suite** running the Edge-AI fleet head-to-head against a traditional centralized Open-RMF deployment in synchronized simulation worlds.

---

## 🎯 Key Objectives & Core Pillars

### 1. 📡 Decentralized Peer-to-Peer Communication
- Direct AMR-to-AMR messaging stack broadcasting odometry, localization states, velocities, and navigation intent over a distributed ROS 2 / DDS mesh.
- Zero reliance on a central server for continuous trajectory tracking or safety-critical decisions.
- Fault-tolerant operation: If any AMR temporarily loses connectivity, neighbor nodes extrapolate intent safely.

### 2. ⚡ Dynamic Multi-Agent Conflict Resolution (ORCA)
- Real-time **Optimal Reciprocal Collision Avoidance (ORCA)** and velocity obstacle modulation running onboard each AMR at 20 Hz.
- Eliminates stop-and-wait delays at narrow shelf aisles, intersections, and corridor choke points.
- Continuous kinematic adjustments ensure AMRs pass each other fluidly with zero deadlock.

### 3. 🔄 Autonomous Task Allocation & Dynamic Re-Routing
- Distributed **Contract Net Protocol (CNP)** task auctioning: Robots autonomously bid on tasks based on battery level, proximity, and current load.
- Dynamic Aisle Blockage Detection: When edge perception detects an unexpected obstruction or pallet drop, the robot triggers real-time D* / graph re-routing and offloads affected pickup orders to neighboring AMRs.

### 4. 📊 Centralized Real-Time Fleet Dashboard & Analytics
- Lightweight web application (FastAPI + WebSocket gateway) providing real-time 2D warehouse map visualization.
- Live tracking of AMR coordinates, battery levels, operational states, and velocity vectors.
- Side-by-side comparative benchmarking metrics against traditional centralized Open-RMF.
- 4 interactive Chart.js analytics graphs and persistent SQLite audit logging.

---

## 🏆 Verified Success Criteria

| Evaluation Metric | Target Requirement | Demonstrated Platform Result | Status |
| :--- | :--- | :--- | :--- |
| **Inter-Robot Collisions** | **Zero Collisions** | **0 Collisions** across all intersecting trajectories | ✅ Exceeded |
| **Task Completion Time** | **$\ge$ 20% Reduction** | **30.5% – 42.8% Reduction** vs. Traditional Open-RMF | ✅ Exceeded |
| **Standstills & Halts** | Minimize delay | **100% elimination** of mutual exclusion stop halts | ✅ Exceeded |
| **Minimum Fleet Size** | At least 3 AMRs | **5 Full AMRs** (`tinyRobot1` – `tinyRobot5`) | ✅ Exceeded |
| **Architecture** | Edge Hardware Compatible | Lightweight C++20 nodes (< 5% CPU on ARM64) | ✅ Exceeded |

---

## 📁 System Architecture & Directory Layout

```text
.
├── web_dashboard/                  # Real-Time Telemetry & Comparative Analytics Gateway
│   ├── dashboard_server.py         # FastAPI / ROS 2 telemetry aggregator & WebSocket server (Port 8080)
│   ├── dashboard_ui/               # Live web UI with 2D warehouse floorplan & Chart.js engine
│   │   ├── index.html              # Interactive comparison dashboard
│   │   ├── chart.umd.min.js        # Graph visualization library
│   │   └── warehouse_L1.png        # Scaled warehouse coordinate texture
│   ├── benchmark_history.db        # SQLite audit database storing historical runs
│   └── run_dashboard.sh            # Standalone dashboard launcher
├── rmf_ws/                         # Proposed: Decentralized Edge-AI Simulation (ROS Domain 10)
│   ├── src/
│   │   ├── edge_fleet_core/        # C++20 ORCA velocity negotiation & conflict resolution
│   │   ├── edge_fleet_bringup/     # Master launch configurations & dynamic obstruction tools
│   │   ├── edge_fleet_msgs/        # Distributed P2P message definitions (RobotIntent, TaskAuction)
│   │   └── demonstrations/         # 3D Warehouse world models, textures, and AMR navigation maps
│   ├── run_warehouse_edge_ai.sh    # Standalone Edge-AI warehouse simulation launcher
│   └── watch_edge_ai_metrics.py    # Real-time console metrics auditor
├── rmf_ws_t/                       # Baseline: Traditional Centralized Open-RMF Simulation (ROS Domain 20)
│   ├── src/                        # Standard Open-RMF centralized schedule & reservation node
│   ├── telemetry_server.py         # REST API bridge for traditional fleet telemetry (Port 8081)
│   └── run_warehouse.sh            # Standalone traditional warehouse launcher
├── screenshots/                    # High-resolution architectural & dashboard captures
│   └── dashboard.png               # Real-time comparative dashboard overview
├── run_all.sh                      # 🚀 Master unified launcher (starts all 3 systems simultaneously)
├── dispatch_dual_tasks.sh          # ⚡ Simultaneous mission dispatcher & automated auditor
├── kill_all.sh                     # 🛑 Clean terminator for all simulations, visualizers & daemons
├── restart_servers.sh              # Fast restart script for web and telemetry servers
├── tile_dual_windows.py            # 🪟 4-quadrant auto-tiler for dual Gazebo & RViz visualizers
└── README.md                       # Comprehensive project documentation
```

---

## 🚀 Quick Start Guide

All commands can be executed directly from the project root:

### 1. Launch All Three Systems Simultaneously
```bash
./run_all.sh
```
> **Daemon Mode:** To start all processes in the background, run `./run_all.sh --daemon`.

This automated script:
1. Performs port sanitization (`8080`, `8081`, `8765`) and terminates lingering simulator instances.
2. Boots the **Centralized Web Dashboard** on `http://localhost:8080`.
3. Boots the **Traditional Centralized Fleet** (`rmf_ws_t`) on ROS Domain 20 with its telemetry bridge on port `8081`.
4. Boots the **Decentralized Edge-AI Fleet** (`rmf_ws`) on ROS Domain 10 with P2P mesh and ORCA negotiation.
5. Launches dual Gazebo Sim (Harmonic) and dual RViz2 visualizers with GPU acceleration, auto-tiling them into a 4-quadrant desktop layout.

---

### 2. Dispatch Simultaneous Missions to Both Fleets

Open a secondary terminal to trigger synchronized comparative mission workloads:

```bash
# 1. Standard 5-AMR intersecting benchmark mission (demonstrates intersection yield vs. ORCA)
./dispatch_dual_tasks.sh benchmark all

# 2. Putaway warehouse workload
./dispatch_dual_tasks.sh putaway all

# 3. Return all AMRs to dedicated charging stations
./dispatch_dual_tasks.sh dock all
```

While running, `dispatch_dual_tasks.sh` streams comparative live telemetry in your terminal:
- Current velocities and active waypoints.
- Avoided halts and standstill delays.
- Real-time speedup factor ($\ge 1.3\times$).
- Automatically saves the audit trail to the persistent SQLite database upon mission completion.

---

### 3. Dynamic Obstacle Injection & Re-Routing Test

To test the Edge-AI fleet's dynamic perception and decentralized re-routing under unexpected aisle obstructions:

```bash
# Inject an obstacle blocking the path of AMR 1
python3 rmf_ws/src/edge_fleet_bringup/scripts/inject_blockage.py --robot tinyRobot1 --blocked true

# Clear the obstacle once re-routing is demonstrated
python3 rmf_ws/src/edge_fleet_bringup/scripts/inject_blockage.py --robot tinyRobot1 --blocked false
```

The robot's onboard perception node immediately detects the blockage, updates its local occupancy map, and invokes real-time dynamic re-routing without requiring intervention from a central server.

---

### 4. Interactive Web Dashboard & Comparative Analytics

Navigate to:
```
http://localhost:8080
```

- **Live 2D Warehouse Floorplan:** Displays positions, orientations, speeds, and battery states of all 5 AMRs across both fleets in real time.
- **Comparison Engine:** Side-by-side contrast of mission execution duration, distance traversed, and standstill delays eliminated.
- **📊 Graphical Charts:** Click **`[📊 Graphical Charts]`** in the header to view 4 interactive dynamic charts:
  1. *Total Mission Duration Comparison (Grouped Bar)*
  2. *Standstill Delays Avoided (Grouped Bar)*
  3. *Trajectory Distance Traversed (Horizontal Bar)*
  4. *5-Axis Fleet Efficiency Radar Profile*
- **Persistent SQLite Audit History:** Inspect historical benchmark runs stored in `benchmark_history.db`.

---

### 5. Running Components Individually

If you prefer to run individual subsystems in separate terminals:

- **Web Dashboard Only:**
  ```bash
  cd web_dashboard
  ./run_dashboard.sh
  ```
- **Decentralized Edge-AI Simulation Only (Domain 10):**
  ```bash
  cd rmf_ws
  ./run_warehouse_edge_ai.sh
  ```
- **Traditional Centralized Simulation Only (Domain 20):**
  ```bash
  cd rmf_ws_t
  ./run_warehouse.sh
  ```

---

### 6. Clean Shutdown
To safely stop all simulations, visualizers, and background HTTP/WebSocket servers:
```bash
./kill_all.sh
```

---

## 🔬 Algorithmic Details

### Decentralized Velocity Negotiation (ORCA)
In the decentralized Edge-AI fleet, each AMR $i$ computes a safe collision-free velocity $\mathbf{v}_i^{\text{new}}$ by solving a linear program with half-plane constraints generated from neighboring robots' positions and velocities:
$$\mathbf{v}_i^{\text{new}} = \arg\min_{\mathbf{v} \in \text{ORCA}_{i|j}} \|\mathbf{v} - \mathbf{v}_i^{\text{pref}}\|$$
Each AMR shares responsibility reciprocally (50/50), adjusting its velocity vector continuously. This avoids the severe stop-and-wait penalties inherent in traditional mutex schedule reservations.

### Dynamic Task Allocation & Autonomous Re-Routing (Contract Net Protocol)

When an AMR encounters a dynamic disturbance (such as an obstructed aisle or newly prioritized warehouse order), the fleet coordinates autonomously without a central server:

1. **Aisle Blockage Detection & Local Detour**:
   - The robot's onboard perception node detects the obstruction (`/aisle_blocked`).
   - If an alternative passable path exists in its local warehouse topological graph, the AMR dynamically computes a detour.

2. **Distributed Contract Net Protocol (CNP) Auctioning**:
   If the corridor is completely blocked or the AMR cannot service the pickup point, it initiates a peer-to-peer auction over `/fleet/task_auctions`:
   - **Phase 1: Task Announcement (`AUCTION_ANNOUNCE`)**: The detecting AMR acts as the auctioneer, broadcasting the unfulfilled task parameters (task ID, pickup coordinates, dropoff destination, and reason `AISLE_BLOCKED`).
   - **Phase 2: Bid Evaluation & Submission (`AUCTION_BID`)**: Neighboring peer AMRs calculate a bid cost based on physical proximity and battery state-of-charge:
     $$\text{Bid Cost} = \text{Euclidean Distance to Pickup (m)} + \frac{100 - \text{Battery SoC (\%)}}{10}$$
     Peers transmit their bids with current battery level and cost back to the auctioneer.
   - **Phase 3: Award Decision (`AUCTION_AWARD`)**: After a 500 ms evaluation window, the auctioneer selects the best bidder (minimum cost) and transmits an award confirmation.
   - **Phase 4: Acknowledgment & Handshake (`AUCTION_ACK`)**: The winning AMR confirms acceptance, updates its active mission queue to service the reallocated pickup location, and broadcasts the status, ensuring continuous warehouse throughput with zero duplicate dispatch.

