# Synchronized Data Collection for Robot, Gripper, and Camera

This document outlines the plan to collect high-quality, synchronized data for the xArm6 robot arm, the Dynamixel gripper, and the RealSense camera.

## 1. Objective

The goal is to record the following data streams into a single ROS bag file for offline analysis and processing:
- **Robot Arm**: Commanded and current state (joint positions, velocities).
- **Gripper**: Commanded velocity and current **normalized** position (0.0 for open, 1.0 for closed).
- **Camera**: Color and depth image streams.

## 2. The Challenge: Missing Gripper Topics & Raw Data

While the robot and camera data are already available as ROS topics, the gripper commands and its current state are handled internally and are not published. Additionally, the raw position data from the gripper is in abstract hardware units. To solve this, we will:
1.  Expose the gripper command and state on new ROS topics.
2.  Normalize the raw position data to an intuitive `0.0` to `1.0` range.

## 3. Strategy

The process is broken down into three steps:

1.  **Code Modification**: Modify the `xarm_moveit_servo` package to create and update the gripper topics.
2.  **Build & Run**: Rebuild the workspace and run the system as usual.
3.  **Record Data**: Use the `ros2 bag record` command to capture all the necessary topics simultaneously.

---

## Step 1: Code Modifications

We will make small changes to the relevant files to publish the normalized gripper data.

### File 1: `xarm_moveit_servo/include/xarm_moveit_servo/gripper_controller.h`

First, we expose the hardware limits by making `POS_MIN` and `POS_MAX` public constants. This allows other nodes to use them for normalization.

**Change this:**
```cpp
private: 
    // ...
    // Position Limits
    const int32_t POS_MIN = 2600;
    const int32_t POS_MAX = 3700;
```

**To this:**
```cpp
public: 
    // ...
    // Position Limits
    static const int32_t POS_MIN = 2600;
    static const int32_t POS_MAX = 3700;
```

### File 2: `xarm_moveit_servo/include/xarm_moveit_servo/xarm_joystick_input.h`

Next, we update the `gripper_state_pub_` to use the `Float64` message type for our normalized value and include the required header.

**Change this:**
```cpp
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gripper_state_pub_;
```

**To this (and ensure the header is included):**
```cpp
#include "std_msgs/msg/float64.hpp"
//...
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_state_pub_;
```

### File 3: `xarm_moveit_servo/src/xarm_joystick_input.cpp`

Finally, we implement the normalization logic.

**1. In the `JoyToServoPub` constructor, update the publisher creation:**
```cpp
// Change this line
    gripper_state_pub_ = this->create_publisher<std_msgs::msg::Int32>("/gripper/state", ros_queue_size_);
// To this
    gripper_state_pub_ = this->create_publisher<std_msgs::msg::Float64>("/gripper/state", ros_queue_size_);
```

**2. In the `_joy_callback` function, update the gripper logic to calculate and publish the normalized position:**
```cpp
// ... inside _joy_callback ...
    // This is the new logic for gripper control and publishing
    int32_t gripper_cmd_velocity = 0;
    int32_t gripper_current_pos = 0;
    auto gripper_cmd_msg = std::make_unique<std_msgs::msg::Int32>();
    auto gripper_state_msg = std::make_unique<std_msgs::msg::Float64>(); // Use Float64

    if (joystick_type_ == JOYSTICK_XBOX360_WIRED || joystick_type_ == JOYSTICK_XBOX360_WIRELESS)
    {
        if (msg->buttons[XBOX360_BTN_A]) {
            gripper_cmd_velocity = GRIPPER_OPEN_VELOCITY;
        } else if (msg->buttons[XBOX360_BTN_B]) {
            gripper_cmd_velocity = GRIPPER_CLOSE_VELOCITY;
        }
    }
    
    // Send command and get state
    gripper_controller_->moveWithVelocity(gripper_cmd_velocity, gripper_current_pos);

    // --- NORMALIZATION LOGIC ---
    // Normalize the raw position to a 0.0-1.0 range
    double normalized_pos = (double)(gripper_current_pos - GripperController::POS_MIN) / (double)(GripperController::POS_MAX - GripperController::POS_MIN);
    // Clamp the value between 0.0 and 1.0 to ensure it's always in range
    normalized_pos = std::max(0.0, std::min(1.0, normalized_pos));

    // Publish both command and state
    gripper_cmd_msg->data = gripper_cmd_velocity;
    gripper_state_msg->data = normalized_pos; // Publish normalized value
    gripper_command_pub_->publish(std::move(gripper_cmd_msg));
    gripper_state_pub_->publish(std::move(gripper_state_msg));

    // ... rest of the function
```

---

## Step 2: Build and Source the Workspace

After saving the code changes, rebuild your workspace from the root (`~/rrc/xarm_ws`):

```bash
colcon build --packages-select xarm_moveit_servo
source install/setup.bash
```

---

## Step 3: Record the Data

### 3.1 Configure Joystick/Gripper Frequency

To ensure a high and consistent data rate, set the `autorepeat_rate` for the `joy_node` in `_robot_moveit_servo_realmove.launch.py` to your desired frequency (e.g., 30.0 for 30 Hz).

```python
# In _robot_moveit_servo_realmove.launch.py
            ComposableNode(
                package='joy',
                plugin='joy::Joy',
                name='joy_node',
                parameters=[
                    {'autorepeat_rate': 30.0},
                ],
            ),
```

### 3.2 Configure Camera Frequency (Optional)

You can check available camera profiles and set them at launch time.

```bash
# Check available profiles (while camera node is running)
ros2 param describe /camera/camera rgb_camera.color_profile

# Launch camera with a specific profile
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:="1280,720,30"
```

### 3.3 Address DDS Buffer Overflows (if needed)

If you encounter `sequence size exceeds remaining buffer` errors, it indicates that the ROS 2 DDS middleware is struggling with the volume or frequency of messages. A common solution is to reduce the highest frequency data streams.

You found that reducing the `update_rate` of the `joint_trajectory_controller` in `lite6_controllers.yaml` from 250 Hz to **200 Hz** resolved this issue.

**Locate this section in `xarm_controller/config/lite6_controllers.yaml`:**
```yaml
controller_manager:
  ros__parameters:
    update_rate: 250  # Hz
```

**Change it to:**
```yaml
controller_manager:
  ros__parameters:
    update_rate: 200  # Hz
```
This reduces the frequency at which the robot's state is read and published, alleviating pressure on the DDS buffers.

### 3.4 Start Recording

1.  Run the launch files for the robot and camera.
2.  In a new terminal, run the `ros2 bag record` command. Note that `/gripper/state` will now contain the normalized `0.0-1.0` value.

    ```bash
    ros2 bag record -o sync_data_bag \
    /lite6_traj_controller/controller_state \
    /ufactory/joint_states \
    /gripper/command \
    /gripper/state \
    /camera/camera/color/image_raw \
    /camera/camera/depth/image_rect_raw
    ```
3.  Collect your data, then stop the recording with `Ctrl+C`.
