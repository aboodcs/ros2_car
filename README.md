# ROS2 Car: Autonomous Robotic Vehicle Platform

[![ROS2 Version](https://img.shields.io/badge/ROS2-Humble%20%7C%20Iron%20%7C%20Jazzy-blueviolet?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow?style=flat-square)](https://opensource.org/licenses/MIT)
[![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/aboodcs/ros2_car/ci.yml?style=flat-square&logo=github)](https://github.com/aboodcs/ros2_car/actions)
[![GitHub Stars](https://img.shields.io/github/stars/aboodcs/ros2_car?style=flat-square&logo=github)](https://github.com/aboodcs/ros2_car/stargazers)
[![GitHub Forks](https://img.shields.io/github/forks/aboodcs/ros2_car?style=flat-square&logo=github)](https://github.com/aboodcs/ros2_car/network/members)
[![GitHub Issues](https://img.shields.io/github/issues/aboodcs/ros2_car?style=flat-square&logo=github)](https://github.com/aboodcs/ros2_car/issues)
[![GitHub Contributors](https://img.shields.io/github/contributors/aboodcs/ros2_car?style=flat-square&logo=github)](https://github.com/aboodcs/ros2_car/graphs/contributors)
[![Code Size](https://img.shields.io/github/languages/code-size/aboodcs/ros2_car?style=flat-square)](https://github.com/aboodcs/ros2_car)
[![Last Commit](https://img.shields.io/github/last-commit/aboodcs/ros2_car?style=flat-square)](https://github.com/aboodcs/ros2_car/commits/main)

🚀 **Welcome to ROS2 Car** – an advanced, modular ROS2 package for building, simulating, and deploying autonomous robotic vehicles. This project transforms a simple RC car into a sophisticated platform for robotics research, education, and experimentation. Whether you're integrating AI for perception, implementing SLAM for mapping, or testing control algorithms, this repo provides a robust foundation with hardware and simulation support.

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Demo](#demo)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Building the Workspace](#building-the-workspace)
- [Quick Start: Launching the System](#quick-start-launching-the-system)
- [Advanced Commands and Usage](#advanced-commands-and-usage)
- [Repository Structure Explained](#repository-structure-explained)
- [System Architecture](#system-architecture)
- [Customization and Extensions](#customization-and-extensions)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [Acknowledgments](#acknowledgments)
- [License](#license)

## Overview

🌟 **Project Purpose**: The `ros2_car` package is designed for enthusiasts and professionals alike to explore ROS2 in the context of mobile robotics. It supports real hardware (e.g., Raspberry Pi-based cars with sensors) and Gazebo simulations, enabling seamless transitions from virtual testing to real-world deployment. Key focuses include teleoperation, sensor fusion, autonomous navigation, and computer vision integration.

Built on ROS2's distributed architecture, it leverages topics, services, actions, and parameters for modular control. This repo is ideal for:
- Learning ROS2 fundamentals.
- Prototyping self-driving algorithms.
- Integrating with ML frameworks like TensorFlow or PyTorch for perception tasks.

📈 **Status**: Actively maintained with CI/CD pipelines for testing on Ubuntu 22.04+.

## Key Features

- **Modular Nodes**: Separate components for control, sensing, and planning.
- **Teleoperation**: Keyboard, joystick, or web-based control.
- **Sensor Suite**: LIDAR (e.g., RPLIDAR), cameras (USB/OpenCV), IMU, encoders.
- **Autonomy Stack**: SLAM (Cartographer/SLAM Toolbox), Navigation2 for path planning, obstacle avoidance.
- **Simulation Environment**: Gazebo with realistic physics, URDF models, and worlds.
- **Visualization Tools**: RViz for 3D views, rqt for debugging.
- **Extensibility**: Hooks for custom AI models, ROS2 bridges (e.g., to Unity or Webots).
- **Data Logging**: ROS2 bags for recording and playback.
- **Hardware Agnostic**: Adaptable to various chassis (e.g., differential drive, Ackermann steering).

## Demo

![ROS2 Car in Action](https://via.placeholder.com/800x400?text=ROS2+Car+Gazebo+Simulation)  
*Gazebo simulation of the car navigating a custom track.*

For a live demo, check out our [YouTube video](https://www.youtube.com/watch?v=example) showcasing autonomous lane following.

Or run it yourself:  
```bash
ros2 launch ros2_car full_sim_demo.launch.py
```

## Prerequisites

### Software
- **OS**: Ubuntu 22.04 (Jammy) or 24.04 (Noble) for optimal compatibility.
- **ROS2**: Humble, Iron, or Jazzy. Install from [official docs](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
- **Build Tools**: `colcon`, `rosdep`.
- **Dependencies**:
  - Core: `ros-<distro>-rclcpp`, `ros-<distro>-rosidl-default-generators`.
  - Simulation: `ros-<distro>-gazebo-ros-pkgs`, `ros-<distro>-gazebo-plugins`.
  - Navigation: `ros-<distro>-navigation2`, `ros-<distro>-slam-toolbox`.
  - Teleop: `ros-<distro>-teleop-twist-keyboard`, `ros-<distro>-joy`.
  - Vision: Python libs via `pip`: `opencv-python`, `numpy`, `scikit-image`.
- **Version Control**: Git.

### Hardware (for Real Deployment)
- Robotic chassis (e.g., 4WD car with DC motors).
- Microcontroller: ESP32 or Arduino for motor control.
- Sensors: RPLIDAR A1/A2, USB camera, MPU6050 IMU, wheel encoders.
- Compute: Raspberry Pi 4/5 or NVIDIA Jetson with ROS2 installed.
- Power: LiPo battery, voltage regulators.

## Installation

1. **Create a ROS2 Workspace**:
   ```bash
   mkdir -p ~/ros2_car_ws/src
   cd ~/ros2_car_ws/src
   ```

2. **Clone the Repository**:
   ```bash
   git clone https://github.com/aboodcs/ros2_car.git
   ```

3. **Install Dependencies**:
   ```bash
   cd ~/ros2_car_ws
   rosdep install --from-paths src --ignore-src -r -y
   sudo apt update && sudo apt install -y python3-colcon-common-extensions ros-<distro>-rviz2
   pip install -r src/ros2_car/requirements.txt  # If a requirements.txt exists
   ```

## Building the Workspace

```bash
cd ~/ros2_car_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

- Use `--cmake-args -DCMAKE_BUILD_TYPE=Release` for optimized performance.
- For cleaning: `rm -rf build install log && colcon build`.

## Quick Start: Launching the System

Always source the workspace first: `source ~/ros2_car_ws/install/setup.bash`.

- **Teleoperation on Hardware**:
  ```bash
  ros2 launch ros2_car teleop_hardware.launch.py
  ```
  Use WASD keys for movement.

- **Gazebo Simulation**:
  ```bash
  ros2 launch ros2_car gazebo_sim.launch.py world:=obstacle_course.world
  ```

- **Autonomous Navigation**:
  ```bash
  ros2 launch ros2_car autonomy_stack.launch.py map:=my_map.yaml
  ```

- **Full End-to-End (Sim + Nav + Viz)**:
  ```bash
  ros2 launch ros2_car complete_system.launch.py
  ```

## Advanced Commands and Usage

| Command | Description | Example Usage |
|---------|-------------|---------------|
| `ros2 run ros2_car motor_controller` | Launches the node for motor PWM/Servo control. | Use with hardware connected via `/dev/ttyUSB0`. |
| `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...` | Publishes velocity commands. | `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"` – Move forward and turn. |
| `ros2 topic echo /scan` | Monitors LIDAR data. | Real-time obstacle detection output. |
| `ros2 bag record -a` | Records all topics to a bag file. | `ros2 bag record -a -o session_data` for playback analysis. |
| `ros2 param set /nav2_controller max_vel_x 0.5` | Dynamically sets parameters. | Adjust speed limits on the fly. |
| `ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph` | Saves a map from SLAM. | For persistent mapping. |
| `ros2 action send_goal /navigate_to_pose ...` | Sends a nav goal via action. | `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 2.0, y: 1.0}}}}"` |
| `rqt --perspective-file config/rqt_perspective.perspective` | Opens custom rqt dashboard. | For multi-plugin monitoring. |
| `ros2 launch ros2_car vision_pipeline.launch.py` | Starts computer vision nodes (e.g., lane detection). | Integrates OpenCV for processing `/image_raw`. |
| `gz topic -l` | Lists Gazebo topics (simulation only). | Debug simulation interfaces. |

For scripting: Combine with `ros2 run` in bash scripts for automated tests.

## Repository Structure Explained

Here's a detailed breakdown of the repo's organization:

- **`src/ros2_car/`**: Main package directory.
  - **`nodes/`**: Python/C++ source for individual nodes.
    - `motor_controller.py`: Interfaces with hardware for velocity commands via serial/ROS bridge.
    - `sensor_fusion_node.cpp`: Fuses IMU, odometry, and LIDAR for accurate state estimation.
    - `autonomy_brain.py`: High-level decision-making for modes (teleop vs. auto).
  - **`launch/`**: XML/Python launch files.
    - `teleop_hardware.launch.py`: Includes teleop_twist_keyboard and motor nodes.
    - `gazebo_sim.launch.py`: Spawns URDF in Gazebo with plugins.
    - `autonomy_stack.launch.py`: Brings up Nav2, AMCL, and SLAM.
  - **`config/`**: YAML and other configs.
    - `nav2_params.yaml`: Tuned parameters for Navigation2 (controllers, planners).
    - `rviz_config.rviz`: Pre-set views for topics like /map, /path.
    - `slam_config.yaml`: Settings for SLAM Toolbox or Cartographer.
  - **`urdf/`**: Robot models.
    - `car_model.urdf.xacro`: Macro-based URDF for the car, including meshes and sensors.
    - `gazebo_control_plugins.xml`: Diff-drive plugin for simulation dynamics.
  - **`worlds/`**: Gazebo environments.
    - `obstacle_course.world`: Custom SDF world with ramps, walls, and textures.
  - **`meshes/`**: 3D models (STL/DAE) for visuals/collisions.
  - **`scripts/`**: Utility scripts, e.g., `calibrate_sensors.sh`.
- **`package.xml`**: Declares dependencies (e.g., geometry_msgs, sensor_msgs).
- **`CMakeLists.txt`**: Builds C++ nodes, installs files.
- **`setup.py`**: Entry points for Python executables.
- **`.github/workflows/ci.yml`**: GitHub Actions for automated builds/tests.
- **`requirements.txt`**: Python dependencies.
- **`LICENSE`**: MIT terms.
- **`README.md`**: This file!

Each file includes headers/comments for self-documentation.

## System Architecture

The system follows ROS2's pub-sub model:

```mermaid
graph TD
    A[User Input] -->|cmd_vel| B[Motor Controller Node]
    C[Sensors] -->|scan, imu, image| D[Sensor Fusion Node]
    D -->|odom, map| E[Navigation Stack]
    E -->|cmd_vel| B
    B -->|Feedback| D
    F[Gazebo/Sim] <-->|Plugins| B
    G[RViz/rqt] <--|Visualize| D
```

- **Topics**: /cmd_vel (control), /scan (LIDAR), /odom (position), /map (SLAM output).
- **Services/Actions**: Nav2 actions for goals, services for mode switching.
- **Parameters**: Tunable via YAML for gains, thresholds.

## Customization and Extensions

- **Add a New Node**: Create in `nodes/`, add to `setup.py`, include in launch files.
- **Integrate ML**: Use `vision_pipeline` to publish processed images; add TensorFlow in a new node.
- **Hardware Mods**: Edit URDF for new sensors; update serial ports in configs.
- **Web Interface**: Bridge with rosbridge_suite for browser control.

## Troubleshooting

- **No Devices Found**: Check `ls /dev/tty*` and udev rules.
- **Simulation Lag**: Set `use_sim_time:=true`; optimize URDF.
- **Nav Failures**: Verify map with `ros2 run nav2_map_server map_server`.
- Run `ros2 doctor --report` for diagnostics.
- Logs: `ros2 log /node_name`.

## Contributing

🤝 We welcome PRs! Follow these steps:
1. Fork and clone.
2. Create a feature branch: `git checkout -b feature/xyz`.
3. Commit with descriptive messages.
4. Push and open a PR.
5. Adhere to [Contributor Covenant](https://www.contributor-covenant.org).

For issues: Use templates, provide logs/screenshots.

## Acknowledgments

- Inspired by ROS2 tutorials and open-source robotics projects.
- Thanks to contributors and the ROS community.

## License

Licensed under the MIT License. See [LICENSE](LICENSE) for details.
