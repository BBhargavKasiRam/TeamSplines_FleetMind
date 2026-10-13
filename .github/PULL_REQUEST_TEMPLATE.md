## Description

<!-- Provide a brief description of the changes introduced in this PR and the rationale. -->

## Subsystem(s) Affected

- [ ] P2P Communication Stack (`edge_fleet_core/p2p_peer_node`)
- [ ] ORCA Collision Avoidance (`edge_fleet_core/conflict_resolver_node`)
- [ ] Task Allocation & Auctioning (`edge_fleet_core/task_allocator_node`)
- [ ] Perception & Obstacle Handling (`edge_fleet_core/edge_perception_node`)
- [ ] Simulation Assets / World Files (`rmf_demos_maps`, `rmf_demos_gz`)
- [ ] Centralized Telemetry Dashboard (`web_dashboard`)
- [ ] Automation & Scripts (`run_all.sh`, `dispatch_dual_tasks.sh`, etc.)

## Type of Change

- [ ] Bug fix (non-breaking change resolving an open issue)
- [ ] New feature (non-breaking enhancement adding functionality)
- [ ] Performance optimization (algorithmic or runtime speedup)
- [ ] Breaking change (fix or feature requiring downstream interface updates)
- [ ] Documentation update

## Verification & Validation Checklist

- [ ] **Build Validation**: Package builds cleanly using `colcon build` with zero compiler warnings.
- [ ] **Simulation Test**: Verified multi-robot behavior in Gazebo Harmonic simulation.
- [ ] **Collision Safety**: Confirmed **zero inter-robot collisions** during intersecting navigation.
- [ ] **Edge Compatibility**: Verified low computational footprint (< 10% CPU per node on ARM64).
- [ ] **Code Quality**: Follows ROS 2 C++20 and Python (PEP 8) style conventions.
- [ ] **Documentation**: Updated `README.md` or relevant docstrings if configuration parameters changed.

## Related Issues

<!-- Link any associated issues: e.g., Closes #12, Fixes #34 -->
Closes #
