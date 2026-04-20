# Setup Guide

## Recommended Environment

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo / Nav2

## System Dependencies

```bash
sudo apt update
sudo apt install -y \
  python3-vcstool \
  python3-rosdep \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-rviz2 \
  python3-matplotlib \
  python3-networkx \
  libboost-all-dev \
  libyaml-cpp-dev \
  libeigen3-dev \
  libfltk1.3-dev
```

## External ROS Packages

The `deps/` directory contains pinned `vcs` manifests for the validated dependency sets.

```bash
cd ~/fleet_ws
vcs import src < src/multi-robot-fleet-management-system/deps/ros2-humble-core.repos
```

For the larger AWS-style warehouse worlds:

```bash
cd ~/fleet_ws
vcs import src < src/multi-robot-fleet-management-system/deps/ros2-humble-aws-worlds.repos
```

## Optional Legacy Stage Support

The original workspace bundled upstream copies of `Stage` and `stage_ros2` so the Stage-based launch path could be replayed locally. Those upstream repositories are not mirrored in this portfolio repository, but you can restore the same dependency chain with:

```bash
cd ~/fleet_ws
vcs import src < src/multi-robot-fleet-management-system/deps/ros2-humble-stage.repos
```

Then install ROS dependencies and build:

```bash
cd ~/fleet_ws
# Run `sudo rosdep init` once on a fresh machine if needed.
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Notes

- The Gazebo flow is the cleanest entry point for inspection and replay.
- For headless servers, launch the Gazebo demo with `enable_gzclient:=false launch_rviz:=false`.
- The Stage-related launch files remain available for legacy scenario replay when the optional Stage dependencies are installed.
- Environment-specific maps, graph files, and task lists live under `src/server/fms_server/cfg`.
