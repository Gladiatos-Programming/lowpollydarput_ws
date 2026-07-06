# Darput ROS 2 & Gazebo Workspace

A bipedal robot simulation and control workspace using ROS 2 (Humble/Jazzy) and Gazebo Fortress.

---

## Prerequisites

- **Ubuntu 22.04+**
- **ROS 2 Humble** (or Jazzy)
- **Gazebo Fortress** (via `ros_gz`)
- **Python 3.10+**
- **Pinocchio** (`pip install pinocchio`)
- **PyQt5** (`sudo apt install python3-pyqt5`)

## Installation

```bash
# 1. Install ROS 2 + Gazebo dependencies
sudo apt install ros-${ROS_DISTRO}-xacro \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-joint-state-publisher \
                 ros-${ROS_DISTRO}-joint-state-publisher-gui \
                 ros-${ROS_DISTRO}-rviz2 \
                 ros-${ROS_DISTRO}-ros-gz-sim \
                 ros-${ROS_DISTRO}-ros-gz-bridge \
                 ros-${ROS_DISTRO}-ros-gz-interfaces \
                 ros-${ROS_DISTRO}-controller-manager \
                 ros-${ROS_DISTRO}-joint-trajectory-controller \
                 ros-${ROS_DISTRO}-imu-sensor-broadcaster

# 2. Install Python packages
pip install pinocchio numpy
sudo apt install python3-pyqt5

# 3. Clone & build
cd lowpollydarput_ws
colcon build

# 4. Source the workspace
source install/setup.bash
```

## Usage

### Display robot in RViz (no simulation)

```bash
ros2 launch darput_description display.launch.py
```

### Launch full simulation in Gazebo

```bash
ros2 launch darput_description gazebo.launch.py
```

This spawns: Gazebo Fortress, ROS-Gazebo bridges, robot state publisher, joint trajectory controller, IMU broadcaster, foot contact sensors, Pinocchio IK node, and the UI interface.

### Launch UI interface standalone

```bash
ros2 run darput_description UI_Interface
```

---

## UI_Interface.py — Command Reference

The UI is a PyQt5 launcher window with buttons, sliders, and a log panel. It launches ROS 2 nodes via shell commands and publishes slider values to ROS topics.

### Buttons

| Button | Command | Description |
|---|---|---|
| **Pre Walking** | `ros2 run darput_description Pre_Walking_state` | Moves robot to pre-walking pose — sends IK targets to both legs (via Pinocchio) and manual joint commands to arms/head. |
| **Walking** | `ros2 run darput_description Walking_State` | Starts the dual-leg walking gait controller. Runs a cycloid-based step trajectory at 50 Hz with PD balance compensation using IMU + foot contact feedback. |
| **Initial Position** | `ros2 run darput_description Init_Position` | Stands the robot up from a collapsed/neutral pose by publishing a smooth joint trajectory to all 22 joints. |
| **Visual Marker** | `ros2 run darput_description VisualMarker` | Launches an RViz interactive marker (red sphere) to manually drag & set the right leg IK target pose in real time. |
| **Reset IK Memory** | `ros2 service call /reset_ik_memory std_srvs/srv/Trigger` | Resets the Pinocchio IK node's internal joint memory to match the current robot pose (call this after manual resets). |
| **Reset Simulation Pos** | `ign service -s /world/empty/set_pose ...` | Respawns the robot at the origin (x:0, y:0, z:0) in the Gazebo world. |
| **Go walk** | `ros2 topic pub --once /start_walking std_msgs/msg/Int64 "data: 1"` | Publishes `1` to `/start_walking` to trigger the Walking_State node to begin stepping. |
| **Colcon Build** | `colcon build` | Rebuilds the workspace from the UI. |

### Slider

| Label | Topic | Type | Range | Description |
|---|---|---|---|---|
| **Target Pitch (in degrees)** | `/target_pitch` | `Int64` | -60 to 60 | Sets the desired pitch angle (forward/backward lean) for the walking controller's PD balance loop. |

### Bottom Controls

| Control | Description |
|---|---|
| **Save Config** | Saves current slider values + window position to `config/ui_config.json` (loaded automatically on next launch). |
| **Stop All** | Kills all running subprocesses launched by button presses. |
| **Clear Log** | Clears the output log panel. |

### Architecture

- **`Pre_Walking_state`** — Publishes IK pose targets to `/target_pose/right_leg` and `/target_pose/left_leg`, and joint trajectory to `/joint_trajectory_controller/joint_trajectory`.
- **`Walking_State`** — Runs a 50 Hz gait loop; subscribes to `/imu` (Vector3), `/foot_contact/right`, `/foot_contact/left` (Int32), `/start_walking` (Int64), `/target_pitch` (Int64).
- **`Init_Position`** — Publishes a multi-waypoint JointTrajectory to stand up from zero.
- **`VisualMarker`** — Interactive marker server + publisher to `/target_pose/right_leg`.
- **`PinnochioIK`** — Core IK solver using Pinocchio; subscribes to `/target_pose/*` topics, publishes joint commands to `/joint_trajectory_controller/joint_trajectory`. Exposes `/reset_ik_memory` and `/servo_speed_ns` services.
- **`imu_reader`** — Converts IMU quaternion to Euler angles (degrees), publishes to `/imu`.
- **`foot_sensor`** — Bridges Gazebo contact data to `/foot_contact/right` and `/foot_contact/left` with timeout watchdog.
- **`test_foot_state`** — Debug node that prints foot contact status changes.

### Config Files (`config/`)

| File | Purpose |
|---|---|
| `joint_controller.yaml` | Controller manager config: joint names, PID, IMU sensor. |
| `default.rviz` | RViz layout for Gazebo launch. |
| `display.rviz` | RViz layout for display-only launch. |
