# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 workspace for **AVATAR** — an 8 DOF humanoid robot controlled via Dynamixel servo motors. The system layers low-level serial motor control (Dynamixel SDK) beneath a MoveIt2 motion planning stack.

## Build & Run

```bash
# Build all packages
colcon build

# Build a specific package
colcon build --packages-select dynamixel_control
colcon build --packages-select robot_client
colcon build --packages-select moveit_gripper

# Source the workspace after building
source install/setup.bash
```

**Lint/test** (ament_lint_auto based — no custom test suite):
```bash
colcon test --packages-select dynamixel_control
colcon test-result --verbose
```

**Running nodes:**
```bash
ros2 run dynamixel_control read_write_node
ros2 run dynamixel_control csv_joint_state_publisher
ros2 launch moveit_gripper demo.launch.py
ros2 launch dynamixel_control robot_visualization.launch.py
```

**Sending commands:**
```bash
# Publish target joint positions (radians)
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "{name: [], position: [0,0,0,0,0,0,0,0]}"
# Enable/disable torque
ros2 topic pub -1 /command std_msgs/msg/String "data: 'enable'"
```

## Package Structure

| Package | Role |
|---|---|
| `Dynamixel/dynamixel_sdk` | Low-level serial communication with Dynamixel servos (Protocol 1 & 2, shared library) |
| `Dynamixel/dynamixel_control` | ROS 2 nodes for motor read/write, torque control, CSV playback, visualization |
| `moveit_gripper` | MoveIt2 configuration (URDF/SRDF, kinematics, controllers, launch) |
| `robot_client` | High-level planning nodes that interface with MoveIt2 |

## Architecture & Data Flow

```
USB (/dev/ttyUSB0 @ 1 Mbps)
  └─ dynamixel_sdk (serial packet layer)
       └─ dynamixel_control nodes
            ├─ publishes  → /joint_states (sensor_msgs/JointState)
            └─ subscribes → /command (std_msgs/String)

MoveIt2 (moveit_gripper config)
  └─ robot_client/moveit_control
       ├─ subscribes → /target_position (geometry_msgs/PoseStamped)
       └─ executes planned trajectory via ros2_controllers
```

**Key control nodes in `dynamixel_control/src/`:**
- `read_write_node.cpp` — basic position read/write for 2 servos (ID 1 & 2)
- `torque_compensation_node.cpp` — current-feedback torque control
- `csv_joint_state_publisher.cpp` — plays back recorded motion from CSV
- `poppy_csv_joint_state_publisher.cpp` — same, targeting Poppy robot joints
- `h1_csv_joint_state_publisher.cpp` — same, targeting Unitree H1_2 joints

**Robot descriptions supported:** AVATAR (primary, URDF at `Dynamixel/dynamixel_control/urdf/AVATAR_URDF_Test3.urdf`), Unitree H1_2 (`h1_2_description/`), Poppy (`poppy_description/`).

## Key Configuration

- **Serial device:** `/dev/ttyUSB0`, baud rate 1 Mbps (hardcoded in control nodes)
- **Motor IDs:** big motor = 1, small motor = 2 (in `read_write_node`)
- **C++ standard:** C++17 for `robot_client` and `dynamixel_sdk`; C++14 for `dynamixel_control`
- **MoveIt config files:** `moveit_gripper/config/` — kinematics.yaml, ros2_controllers.yaml, joint_limits.yaml, gripper_arm.srdf
- **CSV data logs:** `Dynamixel/dynamixel_control/data/csv/` — recorded motor positions and currents for offline analysis

## Motion Planning Interface

`robot_client/src/moveit_control.cpp` subscribes to `/target_position` (PoseStamped) and calls MoveIt2's `MoveGroupInterface` to plan and execute. The planning group name and end-effector link are defined in `moveit_gripper/config/gripper_arm.srdf`.
