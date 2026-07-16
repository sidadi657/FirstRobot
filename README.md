# SARNav

**Autonomous Search-and-Rescue Navigation for Rocker-Bogie Rovers**

SARNav is a ROS 2 / Gazebo simulation platform, a six-wheeled rocker-bogie rover, aimed at autonomous navigation across earthquake rubble.

---

## Table of Contents

- [System Overview](#system-overview)
- [Hardware](#hardware)
- [Software Stack](#software-stack)
- [Repository Structure](#repository-structure)
- [Build Instructions](#build-instructions)
- [Running the Simulation](#running-the-simulation)
- [Sending the Rover Somewhere](#sending-the-rover-somewhere)
- [Manual Override (twist_mux)](#manual-override-twist_mux)
- [Tuning Traversability](#tuning-traversability)
- [Known Issues & Limitations](#known-issues--limitations)
- [Troubleshooting](#troubleshooting)
- [Credits](#credits)

---

## System Overview

- **Robot:** SARNav — 6-wheel rocker-bogie chassis, 4 independently steered corner wheels (front/back), 2 fixed drive-only mid wheels, fully passive rocker/bogie suspension (no actuation, driven by physics/terrain contact — matching real rocker-bogie mechanisms like those on Mars rovers).
- **Simulation:** Gazebo (`gz-sim`)
- **Perception:** Horizontal 2D LiDAR (360°, SLAM + general obstacle avoidance) + RGB-D depth camera (close-range terrain relief analysis).
- **Navigation:** Nav2 (MPPI controller, dual costmap architecture) + slam_toolbox (online async SLAM).

## Hardware

| Component | Target |
|---|---|
| Onboard compute | Raspberry Pi 5, 8GB (CPU-only — no CUDA dependency anywhere in this stack) |
| LiDAR | Any 2D LiDAR publishing `sensor_msgs/LaserScan` |
| Depth sensor | Any RGB-D camera publishing `sensor_msgs/PointCloud2` (developed against a simulated RGB-D camera; real-hardware equivalents: Intel RealSense D400-series or similar) |
| Drive | 6x drive motors, 4x steering servos (front/back corners) |
| Dev/sim machine | A machine with an NVIDIA GPU is strongly recommended for running the Gazebo simulation itself (rendering-heavy) — the deployed rover does **not** need one |

> **Design note:** we deliberately avoided GPU-dependent terrain-analysis approaches (e.g. `elevation_mapping_cupy`) specifically because the target deployment hardware (Raspberry Pi 5) has no CUDA-capable GPU. The relief filter is pure NumPy, fully vectorized, and tuned to run on CPU-only embedded hardware.

## Software Stack

| Layer | What / Why |
|---|---|
| Host OS | Ubuntu 26.04 |
| Host ROS 2 distro | ROS 2 "Lyrical Luth" — runs the Gazebo sim, robot description, and all custom nodes |
| Nav2 / slam_toolbox runtime | Docker container running ROS 2 "Jazzy" — used because Lyrical is new enough that Nav2/slam_toolbox binaries weren't yet published for it at development time. The container talks to the host over standard ROS 2 DDS (shared `ROS_DOMAIN_ID`), so this split is invisible at the topic level. |
| Manual override | `twist_mux`, multiplexing autonomous (`/cmd_vel_nav`) and joystick (`/cmd_vel_joy`) commands into the final `/cmd_vel` |


## Build Instructions

### 1. Host side (ROS 2 Lyrical)

```bash
sudo apt update
sudo rosdep init      # first time only on a fresh machine
rosdep update

cd ~/Documents/rover_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
sudo apt install ros-<distro>-foxglove-bridge  #if you are using foxglove can use rviz if wanted
```

### 2. Nav2 / SLAM container (ROS 2 Jazzy)

```bash
docker run -it --network host -e ROS_DOMAIN_ID=<your domain id here> osrf/ros:jazzy-desktop bash
apt update
apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox 
```

We recommend committing this container as a reusable image once set up, so config edits and installed packages persist across sessions:

```bash
docker commit <container_name> sarnav-nav2:latest
# subsequent runs:
docker run -it --network host -e ROS_DOMAIN_ID=<your domain id> sarnav-nav2:latest bash
```

### 3. Consistent domain ID (both sides)

```bash
echo 'export ROS_DOMAIN_ID=<your domain id>' >> ~/.bashrc
```
Set the same variable inside the container's environment so host and container discover each other over DDS.

## Running the Simulation

Open the following in separate terminals:

**Terminal 1 — Host: simulation**
```bash
cd ~/Documents/rover_ws
source install/setup.bash
ros2 launch forros_description sim.launch.py
```

**Terminal 2 —Docker Container: SLAM**
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true slam_params_file:=(slam params file location, download the file from github repo)
```

**Terminal 3 — Container: Nav2**
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=(slam params file location, download the file from github repo)
```

**Terminal 4 — Host: visualization**
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
Connect from [Foxglove Studio](https://foxglove.dev) to `ws://localhost:8765`.

**Terminal 5 — Host: commands**
```bash
ros2 run forros_description cmd_vel_to_rover
```

**Terminal 6 — Host: joystick**
```bash
ros2 launch teleop_twist_joy teleop-launch.py
```

## Sending the Rover Somewhere(NOT CURRENTLY WORKING PROPERLY)

Once `/map` and both costmaps are populated:

```bash
ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

Or click a goal directly in Foxglove's 3D panel using its goal-pose publishing tool.

Watch it plan and drive:
```bash
ros2 topic echo /cmd_vel
```

## Manual Override (twist_mux)

A gamepad can take priority over autonomous commands at any time — useful if the rover gets physically stuck on rubble:

```bash
sudo apt install ros-jazzy-twist-mux   # match your distro
```

`config/twist_mux.yaml`:
```yaml
twist_mux:
  ros__parameters:
    topics:
      navigation:
        topic: cmd_vel_nav
        timeout: 0.5
        priority: 10
      joystick:
        topic: cmd_vel_joy
        timeout: 0.5
        priority: 100
```

Higher `priority` wins. Joystick input, when present, always overrides autonomous driving.

## Tuning Traversability

The obstacle height variables in the nav2_params.yaml file can be changed to define what all objects can be scales and what are true obstacles.

## Known Issues & Limitations

- **Bogie joint instability:** Even though passive they are rotating on their own causing serious instability and severely limiting the climbing capacities of the rocker bogie mechanism. The cause is currently unknown, may be software or hardware issue
- **Stair-pattern detection:** the relief filter reasons per-cell/per-neighbor and handles continuous ramps well, but a discrete staircase's alternating riser/tread pattern is not yet given dedicated multi-cell pattern recognition.
- **Chassis-pitch during climbing:** the horizontal LiDAR's scan plane tilts with chassis pitch when climbing, which can transiently affect obstacle readings. An IMU-based dynamic correction is scoped but not yet implemented.

## Credits

Built by Sourish Choudhary. Robot name: **Rocker Bogie**. Project name: **SARNav**.
