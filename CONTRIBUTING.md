# Contributing Guidelines

Thank you for your interest in contributing to the **Edge-AI Distributed Fleet Coordination for Autonomous Mobile Robots (AMRs)** framework.

This document outlines the standard engineering workflows, coding conventions, testing protocols, and submission processes expected for all contributions.

---

## 1. Development Environment Prerequisites

Ensure your host environment meets the baseline platform requirements:
* **Operating System:** Ubuntu 24.04 LTS (Noble Numbat)
* **ROS 2 Distribution:** ROS 2 Jazzy Jalisco
* **Simulation Engine:** Gazebo Sim 8.15.0 (Harmonic)
* **Compiler & Toolchain:** GCC 13+ / Clang 17+ (C++20 standard), Python 3.12+
* **Build System:** `colcon` with `ament_cmake` and `ament_python`

---

## 2. Git Workflow & Branching Strategy

We follow a structured trunk-based Git workflow:
* `main`: Production-ready, fully verified codebase. Direct pushes are restricted; all changes arrive via pull requests.
* `feature/<feature-name>`: New capabilities, algorithms, or subsystems.
* `fix/<bug-name>`: Bug fixes and defect resolutions.
* `docs/<topic>`: Documentation enhancements and architectural diagrams.
* `perf/<optimization>`: Algorithmic optimizations and latency reductions.

### Workflow Steps:
1. Fork or clone the repository:
   ```bash
   git clone https://github.com/BBhargavKasiRam/TeamSplines_FleetMind.git
   cd TeamSplines_FleetMind
   ```
2. Create your working branch from `main`:
   ```bash
   git checkout -b feature/dynamic-speed-profiler
   ```
3. Implement changes with clean, atomic commits following the **Conventional Commits** specification.
4. Verify local builds and simulation safety before pushing.
5. Submit a Pull Request targeting `main`.

---

## 3. Commit Message Convention

Commit messages must adhere to [Conventional Commits](https://www.conventionalcommits.org/):

```text
<type>(<scope>): <short imperative summary>

[optional body explaining rationale and context]

[optional footer, e.g., Closes #123]
```

### Allowed Types:
* `feat`: A new feature or algorithmic capability.
* `fix`: A bug fix.
* `docs`: Documentation-only updates.
* `perf`: Code changes improving computational performance or latency.
* `refactor`: Code refactoring without changing observable behavior.
* `test`: Adding or updating test suites or benchmark scenarios.
* `ci`: Changes to CI/CD workflows or build scripts.

---

## 4. Coding Standards

### C++ Guidelines (C++20):
* Adhere to the [ROS 2 C++ Style Guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
* Modern C++ idioms: Use `auto`, `std::unique_ptr`, `std::shared_ptr`, and RAII; avoid raw owning pointers.
* Explicit memory and threading semantics: Keep multi-threaded callback queues thread-safe.
* Real-time predictability: In high-frequency loops (e.g., ORCA 20 Hz loop), avoid unbounded dynamic allocations.

### Python Guidelines (Python 3.12):
* Adhere to [PEP 8](https://peps.python.org/pep-0008/).
* Static typing: Use Python type annotations (`typing`) where practical.
* Formatting and linting: Code must pass `flake8` verification with zero syntax errors.

---

## 5. Local Build & Verification

Before submitting any code, verify compilation and tests locally:

### Build Custom Packages:
```bash
source /opt/ros/jazzy/setup.bash
cd rmf_ws
colcon build --symlink-install --packages-select edge_fleet_msgs edge_fleet_core rmf_edge_ai edge_fleet_bringup
```

### Full Multi-Robot Simulation Validation:
```bash
# Launch both fleets simultaneously
./run_all.sh

# Run the standardized 5-AMR intersecting benchmark
./dispatch_dual_tasks.sh benchmark all
```

### Validation Criteria:
* ✅ Zero inter-robot collisions throughout the run.
* ✅ No deadlock or perpetual standstill at warehouse intersections.
* ✅ All edge nodes maintain < 10% CPU utilization on ARM64 platforms.

---

## 6. Submitting a Pull Request

* Complete all fields in the [Pull Request Template](.github/PULL_REQUEST_TEMPLATE.md).
* Keep PRs concise and focused on a single responsibility.
* Include screenshots or benchmark log outputs for visual or performance-altering changes.
