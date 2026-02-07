# Dynamixel Gripper Integration

This document describes how the Dynamixel servo-based gripper was integrated into the xArm ROS 2 joystick control system.

## 1. Objective

Add gripper control (open/close) to the existing joystick functionality, which already controlled the robot arm's movement. The original proof-of-concept was in Python for ROS 1 and needed to be adapted for ROS 2 C++.

## 2. Integration Strategy

Instead of running a separate Python node for the gripper, a more robust approach was chosen:

1.  **Port to C++:** The core gripper control logic was translated from Python to C++.
2.  **Direct Integration:** This new C++ logic was integrated directly into the existing `JoyToServoPub` node (`xarm_moveit_servo` package).

This centralizes all joystick-related control into a single node, making the system more efficient and architecturally consistent.

## 3. Implementation Details

### 3.1. GripperController C++ Class

A new C++ class, `GripperController`, was created within the `xarm_moveit_servo` package:
- **Files**: `gripper_controller.h` and `gripper_controller.cpp`
- **Communication**: Uses the C++ `dynamixel_sdk` for serial communication (e.g., `/dev/ttyUSB0`)
- **Initialization**:
  1.  Sets **Operating Mode to 1 (Velocity Control)**
  2.  Enables the motor's torque
- **Primary Method**: `moveWithVelocity(int velocity, int& current_pos)` sends a raw velocity command and returns the current position

### 3.2. Joystick Button Mapping

The gripper is controlled using the **A** and **B** buttons on the XBOX 360 controller:

| Button | Action | Velocity |
|--------|--------|----------|
| **A** (held) | Opens gripper | +30 |
| **B** (held) | Closes gripper | -30 |
| Released | Stops gripper | 0 |

### 3.3. ROS Topic Publishing

The gripper state is published on ROS topics for data collection and monitoring:

| Topic | Type | Description |
|-------|------|-------------|
| `/gripper/command` | `std_msgs/Int32` | Commanded velocity (-30, 0, or +30) |
| `/gripper/state` | `std_msgs/Float64` | Normalized position (0.0 = open, 1.0 = closed) |

**Normalization Formula:**
```cpp
// Hardware limits
const int32_t POS_MIN = 2600;  // Fully open
const int32_t POS_MAX = 3700;  // Fully closed

// Normalization
double normalized_pos = (current_pos - POS_MIN) / (double)(POS_MAX - POS_MIN);
normalized_pos = std::clamp(normalized_pos, 0.0, 1.0);
```

### 3.4. Build System Updates

The `CMakeLists.txt` for `xarm_moveit_servo` was modified to:
- Include the DynamixelSDK headers
- Link the `libdynamixel_sdk.so` library

## 4. How to Run

```bash
source install/setup.bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py \
  robot_ip:=192.168.1.175 \
  joystick_type:=1 \
  gripper_port:=/dev/ttyUSB0 \
  gripper_baudrate:=57600
```

## 5. Key Files

| File | Purpose |
|------|---------|
| `xarm_moveit_servo/include/xarm_moveit_servo/gripper_controller.h` | GripperController class header |
| `xarm_moveit_servo/src/gripper_controller.cpp` | GripperController implementation |
| `xarm_moveit_servo/src/xarm_joystick_input.cpp` | JoyToServoPub node with gripper integration |
