# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.0.0] - 2026-09-28

### Added
- **Decentralized P2P Mesh Communication**:
  - Implemented brokerless peer messaging via ROS 2 topics publishing [`RobotIntent`](rmf_ws/src/edge_fleet_msgs/msg/RobotIntent.msg) at 10 Hz.
  - Added state extrapolation and timeout handling for Wi-Fi-denied and dead-zone scenarios.
- **Dynamic Multi-Agent Conflict Resolution (ORCA)**:
  - C++20 [`conflict_resolver_node`](rmf_ws/src/edge_fleet_core/src/conflict_resolver_node.cpp) running at 20 Hz.
  - Reciprocal velocity modulation replacing traditional stop-and-wait intersection mutexes.
- **Autonomous Task Allocation (Contract Net Protocol)**:
  - 4-phase decentralized auction handshake (`ANNOUNCE` $\to$ `BID` $\to$ `AWARD` $\to$ `ACK`) over [`TaskAuction.msg`](rmf_ws/src/edge_fleet_msgs/msg/TaskAuction.msg).
  - Dynamic cost calculation incorporating battery state-of-charge, Euclidean distance, and robot workload.
- **Dynamic Aisle Blockage Re-Routing**:
  - Real-time D* detour path planning triggered via edge perception upon detecting physical corridor obstructions.
- **Unified Dual-Fleet Comparative Suite**:
  - Head-to-head simulation running Proposed Edge-AI (Domain 10) against Traditional Centralized Open-RMF (Domain 20).
  - Automated mission dispatch script [`dispatch_dual_tasks.sh`](dispatch_dual_tasks.sh).
- **Real-Time Telemetry & Monitoring Dashboard**:
  - FastAPI and WebSocket gateway running on Port 8080 and Port 8765.
  - Interactive 2D warehouse canvas and 4 Chart.js analytics graphs.
  - Persistent SQLite benchmark recorder (`benchmark_history.db`).

### Performance Metrics Verified
- **35.7% reduction** in total task completion time (41.2 s vs. 64.1 s).
- **77.5% reduction** in cumulative idle standstill delay (13.2 s vs. 58.9 s per AMR).
- **95.0% reduction** in intersection traffic halts.
- **Zero inter-robot collisions** across all intersecting trajectories.
