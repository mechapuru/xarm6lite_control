# Master Control Guide: xArm6, Gripper, and Camera

This document provides a comprehensive guide to configuring, running, and collecting data from the xArm6 Lite, the Dynamixel-based gripper, and a RealSense camera using a joystick for real-time control.

## 1. System Architecture

The control system is a ROS 2 pipeline that transforms joystick inputs into robot motion.

**Data Flow:**
`Joystick Hardware` -> **`joy_node`** -> **`JoyToServoPub`** -> **`moveit_servo`** -> **`ros2_control`** -> `Robot & Gripper Hardware`

1.  **`joy_node`**: Reads from the physical joystick and publishes raw button and axis data to the `/joy` topic.
2.  **`JoyToServoPub`**: This is a custom node that subscribes to `/joy` and is the central point for interpreting user intent.
    *   **Arm Control**: It converts joystick axis movements into `TwistStamped` (velocity) commands for the arm and publishes them to `/servo_server/delta_twist_cmds`.
    *   **Gripper Control**: It converts joystick button presses (A/B) into velocity commands and sends them directly to the gripper hardware. It also publishes the command and resulting state to ROS topics for data collection.
3.  **`moveit_servo`**: Subscribes to the arm's velocity commands and calculates a short, safe joint trajectory to achieve that motion, publishing it to `/lite6_traj_controller/joint_trajectory`.
4.  **`ros2_control`**: The `joint_trajectory_controller` receives the trajectory and interpolates it at a high frequency, sending fine-grained position commands to the robot's hardware interface.

## 2. Summary of Modifications for Data Collection

To enable this workflow, several key changes were implemented:

-   **Gripper Topic Publishing:** The `xarm_joystick_input` node was modified to publish the gripper's commanded velocity on `/gripper/command` (`std_msgs/Int32`) and its current position on `/gripper/state` (`std_msgs/Float64`).
-   **Data Normalization:** The gripper's raw hardware position (e.g., 2600-3700) is now normalized to an intuitive `0.0` (fully open) to `1.0` (fully closed) range on the `/gripper/state` topic.
-   **Frequency Tuning:**
    -   The `joy_node` was configured with `autorepeat_rate` to provide a steady **30 Hz** command rate, improving system responsiveness.
    -   The `ros2_control` `update_rate` was adjusted to **200 Hz** to prevent DDS buffer overflow errors while maintaining high-frequency state reporting.

## 3. Key Configuration & Tuning

This section provides a centralized reference for the most important parameters for tuning system performance.

| Parameter | Purpose | File |
| :--- | :--- | :--- |
| `autorepeat_rate` | Sets the publishing frequency of the joystick node. **Crucial for responsiveness.** | `xarm_moveit_servo/launch/_robot_moveit_servo_realmove.launch.py` |
| `update_rate` | Sets the frequency of the `ros2_control` loop, which dictates the rate of `/joint_states`. **Important for stability.** | `xarm_controller/config/lite6_controllers.yaml` |
| `publish_period` | Controls the output rate of `moveit_servo`. A smaller value means more frequent plans. | `xarm_moveit_servo/config/xarm_moveit_servo_config.yaml` |
| `scale.linear` | Adjusts the maximum linear speed of the robot arm when using the joystick. | `xarm_moveit_servo/config/xarm_moveit_servo_config.yaml` |
| `scale.rotational`| Adjusts the maximum rotational speed of the robot arm when using the joystick. | `xarm_moveit_servo/config/xarm_moveit_servo_config.yaml` |
| `low_pass_filter_coeff` | Smooths noisy joint state feedback for `moveit_servo`. Higher values mean more smoothing. | `xarm_moveit_servo/config/xarm_moveit_servo_config.yaml` |
| `rgb_camera.color_profile` | Sets the resolution and FPS for the camera's color stream (e.g., `"1280,720,30"`). | Set as a launch argument. |

## 4. How to Run the System

Follow these steps to launch the full system.

### Step 4.1: Launch the Robot and Camera

Open two separate terminals. In each, navigate to your workspace root (`~/rrc/xarm_ws`) and source the environment.

**Terminal 1: Launch the Robot Control**
This command starts the robot drivers, `ros2_control`, `moveit_servo`, and the joystick nodes.

```bash
source install/setup.bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.175 joystick_type:=1 gripper_port:=/dev/ttyUSB0
```

**Terminal 2: Launch the Camera**
This command starts the RealSense camera node. You can add parameters to configure the stream quality.

```bash
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:="1280,720,30"
```

### Step 4.2: Verify Operation

In a new terminal, you can check that the system is running correctly.

-   **Check for new gripper topics:**
    ```bash
    ros2 topic list | grep /gripper
    # Expected output:
    # /gripper/command
    # /gripper/state
    ```
-   **Check the gripper frequency (should be ~30 Hz):**
    ```bash
    ros2 topic hz /gripper/state
    ```
-   **Check the normalized gripper value (should be 0.0-1.0):**
    ```bash
    ros2 topic echo /gripper/state
    ```

## 5. Data Collection Guide

Once the system is running, you can record all the important, synchronized data into a single rosbag file.

**In a new terminal, run the following command:**
This will create a bag file named `sync_data_bag` in your current directory.

```bash
ros2 bag record -o sync_data_bag \
/lite6_traj_controller/controller_state \
/ufactory/joint_states \
/gripper/command \
/gripper/state \
/camera/camera/color/image_raw \
/camera/camera/depth/image_rect_raw
```

Now, operate the robot with the joystick. When you are finished, press `Ctrl+C` in the terminal where the `ros2 bag record` command is running to stop recording and save the file.
