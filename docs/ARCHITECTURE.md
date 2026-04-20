# Architecture Notes

## Runtime Loop

The repository is organized around four cooperating layers:

1. **Status monitoring**
   - Tracks robot position, route progress, task ownership, and replanning triggers.
   - Feeds task and robot tables used by the rest of the stack.
2. **Task grouping / assignment**
   - Builds task groups for available robots instead of assigning only one target at a time.
   - Uses a TSP-based allocation policy to iteratively build minimum-travel-distance task groups for online task streams.
   - Estimates remaining work and updates robot-task ownership.
3. **Route planning / traffic control**
   - Plans collision-aware routes on a topological lane graph.
   - Maintains a route table with dependency information for execution.
4. **Execution / visualization**
   - Pushes route updates to robot-side controllers.
   - Publishes state to RViz panels and the route-table visualizer.

## Package Roles

### Core orchestration

- `fms_core`
  - Fleet manager node
  - Task/robot state tables
  - TSP-based multi-agent task group assignment
  - Multi-agent route planning
  - Route-table bookkeeping
- `fms_msgs`
  - Shared messages and services
- `fms_server`
  - Maps, graphs, launch files, RViz configs, task lists

### Execution layer

- `robot_manager`
  - Robot-side execution and controller nodes
- `follow_poses`
  - Converts FMS routes into Nav2 `NavigateThroughPoses` goals
- `clear_costmap`
  - Clears navigation costmaps around active robots
- `fms_ctrl_board`
  - Groups the robot-side helper launches

### Visualization

- `fms_panel`
  - RViz panel for robot/task state inspection
- `fms_route_viz`
  - Draws route-table dependencies as a layered graph

### Simulation

- `fms_simulation`
  - Scenario-specific launch flows
- `fms_gazebo`
  - Gazebo-facing assets and Nav2 launch support
- `fms_custom_tb`
  - Custom SSH-style testbed assets

## Included vs External

This repository intentionally keeps authored code and authored configuration close together, while leaving large third-party dependencies outside the tree.

### Included

- planning and execution packages
- launch files
- maps, graph configs, RViz configs
- demo media

### External

- TurtleBot3 packages
- Dynamixel SDK
- AWS RoboMaker warehouse world
- optional legacy Stage runtime support

That split keeps the repository reviewable without losing the system story.
