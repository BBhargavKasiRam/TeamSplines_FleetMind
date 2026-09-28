# SIH Smart Warehouse Dual-Fleet Benchmark Suite
### Decentralized Edge-AI vs. Traditional Open-RMF Comparative Platform

This directory contains the unified codebase for the Smart Warehouse multi-fleet simulation and comparative benchmarking platform.

---

## 📁 Directory Structure

```text
/home/manoj/SIH/
├── web_dashboard/           # Centralized Comparative Web Application (Port 8080)
│   ├── dashboard_server.py  # FastAPI / ROS 2 telemetry aggregator & WebSocket gateway
│   ├── dashboard_ui/        # Real-time web UI with live canvas map & 4 Chart.js analytics
│   ├── benchmark_history.db # Persistent SQLite benchmark audit database
│   └── run_dashboard.sh     # Standalone dashboard launcher
├── rmf_ws/                  # Decentralized Edge-AI Open-RMF Simulation (ROS Domain 10)
│   ├── src/                 # Edge-AI algorithms (ORCA velocity modulation, P2P mesh)
│   ├── install/             # Compiled ROS 2 packages
│   └── run_warehouse_edge_ai.sh # Standalone Edge-AI warehouse launcher
├── rmf_ws_t/                # Traditional Centralized Open-RMF Simulation (ROS Domain 20)
│   ├── src/                 # Standard Open-RMF traffic schedule & reservation node
│   ├── install/             # Compiled ROS 2 packages
│   ├── telemetry_server.py  # REST API bridge for traditional fleet (Port 8081)
│   └── run_warehouse.sh     # Standalone Traditional warehouse launcher
├── run_all.sh               # 🚀 Master unified launcher (starts all 3 systems simultaneously)
├── dispatch_dual_tasks.sh   # ⚡ Simultaneous mission dispatcher & live metric recorder
├── kill_all.sh              # 🛑 Clean terminator for all simulations, visualizers & servers
├── tile_dual_windows.py     # 🪟 4-quadrant auto-tiler for Gazebo Sim and RViz2 windows
└── README.md                # Project documentation
```

---

## 🚀 Quick Start Guide

All commands can be run directly from `/home/manoj/SIH`:

### 1. Launch All Three Systems Simultaneously
```bash
cd /home/manoj/SIH
./run_all.sh
```
> **Daemon Mode:** To start everything in the background, run `./run_all.sh --daemon`.

This command:
1. Cleans up any prior simulator processes and frees ports `8080`, `8081`, and `8765`.
2. Starts the **Centralized Web Dashboard** on `http://localhost:8080`.
3. Starts the **Traditional Open-RMF Simulation** (`rmf_ws_t`) on ROS Domain 20 with its Telemetry Gateway on port 8081.
4. Starts the **Decentralized Edge-AI Simulation** (`rmf_ws`) on ROS Domain 10.
5. Launches dual Gazebo Sim and RViz2 visualizers with NVIDIA hardware acceleration and auto-tiles them into a 4-quadrant grid.

---

### 2. Dispatch Missions to Both Fleets Simultaneously
In a separate terminal or after starting the launcher:
```bash
cd /home/manoj/SIH

# Standard 5-AMR intersecting benchmark mission
./dispatch_dual_tasks.sh benchmark all

# Putaway workload mission
./dispatch_dual_tasks.sh putaway all

# Return all AMRs to charger docks
./dispatch_dual_tasks.sh dock all
```
While running, `dispatch_dual_tasks.sh` streams live comparative odometry, standstill counts, and speedups in your terminal, and automatically commits the final audit record to the SQLite database.

---

### 3. Central Web Application & Real-Time Graphical Analytics
Open your browser to:
```
http://localhost:8080
```
- **Live 2D Warehouse Map:** Displays real-time positions, orientations, and speeds of all AMRs across both fleets.
- **Comparison Scale:** Contrasts Edge-AI vs. Traditional metrics (speedup factor, standstills avoided, distance traveled).
- **📊 Graphical Charts:** Click **`[📊 Graphical Charts]`** in the header to view 4 interactive dynamic charts:
  1. *Total Mission Duration Comparison (Grouped Bar)*
  2. *Standstill Delays Avoided (Grouped Bar)*
  3. *Trajectory Distance Traversed (Horizontal Bar)*
  4. *5-Axis Fleet Efficiency Radar Profile*
- **SQLite History Log:** View all past benchmark runs persisted in `web_dashboard/benchmark_history.db`.

---

### 4. Running Components Individually
If you ever want to run a single component independently:

- **Web Dashboard Only:**
  ```bash
  cd /home/manoj/SIH/web_dashboard
  ./run_dashboard.sh
  ```
- **Traditional Warehouse Simulation Only:**
  ```bash
  cd /home/manoj/SIH/rmf_ws_t
  ./run_warehouse.sh
  ```
- **Edge-AI Warehouse Simulation Only:**
  ```bash
  cd /home/manoj/SIH/rmf_ws
  ./run_warehouse_edge_ai.sh
  ```

---

### 5. Stopping Everything
To cleanly stop all simulations, GUI visualizers, and background HTTP/WebSocket servers:
```bash
cd /home/manoj/SIH
./kill_all.sh
```
