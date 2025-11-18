# Integrating Gripper Control with the Joystick

This document outlines the process of integrating a Dynamixel servo-based gripper into the existing ROS 2 joystick control system for the xArm6 Lite.

## 1. Objective

The goal was to add gripper control (open/close) to the existing joystick functionality, which already controlled the robot arm's movement. The original proof-of-concept for gripper control was written in Python for ROS 1, which needed to be adapted for our ROS 2 C++ environment.

## 2. Integration Strategy

Instead of running a separate Python node for the gripper, which would create unnecessary complexity and potential conflicts, a more robust strategy was chosen:

1.  **Port to C++:** The core gripper control logic was translated from Python to C++.
2.  **Direct Integration:** This new C++ logic was integrated directly into the existing `xarm_joystick_input` node (`xarm_moveit_servo` package), which already handles all joystick inputs for the arm.

This approach centralizes all joystick-related control into a single node, making the system more efficient, easier to maintain, and architecturally consistent.

## 3. Implementation Details

### 3.1. New `GripperController` C++ Class

-   A new C++ class, `GripperController`, was created (`gripper_controller.h` and `gripper_controller.cpp`) within the `xarm_moveit_servo` package.
-   This class is a direct port of the original Python logic and is responsible for all low-level communication with the Dynamixel servo motor.
-   It uses the C++ `dynamixel_sdk` to open the serial port (e.g., `/dev/ttyUSB0`) and send commands.
-   Upon initialization, it configures the servo by:
    1.  Setting the **Operating Mode to 1 (Velocity Control)**.
    2.  Enabling the motor's torque.
-   Its primary method, `moveWithVelocity(int velocity)`, sends a raw velocity command to the motor, with safety checks to prevent movement beyond predefined limits.

### 3.2. Modifications to `xarm_joystick_input` Node

The existing `JoyToServoPub` node was modified to incorporate the gripper:

-   An instance of the new `GripperController` class is created and managed by the node.
-   The main joystick callback function (`_joy_callback`) was updated to handle gripper commands **in addition** to arm commands.
-   The conflicting logic that previously mapped the A, B, X, and Y buttons to arm joint jogging was removed to prioritize gripper control.

### 3.3. Joystick Button Mapping

The gripper is now controlled using the **A** and **B** buttons on the XBOX 360 controller:

-   **Press and Hold Button A:** Opens the gripper (sends a positive velocity command).
-   **Press and Hold Button B:** Closes the gripper (sends a negative velocity command).
-   **Release Buttons:** Stops the gripper (sends a zero velocity command).

### 3.4. Build System Updates

-   The `CMakeLists.txt` file for the `xarm_moveit_servo` package was modified to treat the `DynamixelSDK` as an external library.
-   It now includes the necessary paths to the SDK's headers and links the compiled `libdynamixel_sdk.so` library directly, resolving all build dependencies.

## 4. How to Run

1.  **Source the workspace:**
    ```bash
    source install/setup.bash
    ```
2.  **Launch the system:**
    Ensure the gripper is connected and run the following command, replacing `<your_ip>` with the robot's IP address.
    ```bash
    ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=<your_ip> joystick_type:=1 gripper_port:=/dev/ttyUSB0
    ```
