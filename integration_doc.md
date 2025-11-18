# Synchronized Data Collection for Robot, Gripper, and Camera

This document outlines the plan to collect high-quality, synchronized data for the xArm6 robot arm, the Dynamixel gripper, and the RealSense camera.

## 1. Objective

The goal is to record the following data streams into a single ROS bag file for offline analysis and processing:
- **Robot Arm**: Commanded and current state (joint positions, velocities).
- **Gripper**: Commanded velocity and current position.
- **Camera**: Color and depth image streams.

## 2. The Challenge: Missing Gripper Topics

While the robot and camera data are already available as ROS topics, the gripper commands and its current state are handled internally within the `xarm_joystick_input` node and are not published. To record this data, we must first modify the code to expose it on new ROS topics.

## 3. Strategy

The process is broken down into three steps:

1.  **Code Modification**: Modify the `xarm_moveit_servo` package to create two new topics: `/gripper/command` and `/gripper/state`.
2.  **Build & Run**: Rebuild the workspace and run the system as usual.
3.  **Record Data**: Use the `ros2 bag record` command to capture all the necessary topics simultaneously.

---

## Step 1: Code Modifications

We will make small changes to three files to publish the gripper data.

### File 1: `xarm_moveit_servo/include/xarm_moveit_servo/gripper_controller.h`

Modify the `moveWithVelocity` function declaration to allow it to return the current position of the servo.

**Change this:**
```cpp
bool moveWithVelocity(int velocity_raw);
```

**To this:**
```cpp
bool moveWithVelocity(int velocity_raw, int32_t& current_position);
```

### File 2: `xarm_moveit_servo/src/gripper_controller.cpp`

Update the implementation of `moveWithVelocity` to write the position value it reads into the new reference parameter.

**Change this section:**
```cpp
// ... inside moveWithVelocity ...
    // Read present position
    int32_t present_position = 0;
    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, dxl_id_, ADDR_PRESENT_POSITION, (uint32_t*)&present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
// ...
```

**To this:**
```cpp
// ... inside moveWithVelocity ...
    // Read present position
    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, dxl_id_, ADDR_PRESENT_POSITION, (uint32_t*)&current_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
// ...
```
*(Note: We are changing `present_position` to `current_position` and removing the local variable declaration to use the one passed by reference).*

### File 3: `xarm_moveit_servo/include/xarm_moveit_servo/xarm_joystick_input.h`

Add two new publisher member variables to the `JoyToServoPub` class definition.

**Add these lines inside the `private:` section:**
```cpp
    // ... other private members
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gripper_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gripper_state_pub_;
```

### File 4: `xarm_moveit_servo/src/xarm_joystick_input.cpp`

Here, we will initialize the new publishers and use them in the joystick callback.

**1. In the `JoyToServoPub` constructor, add the publisher initializations:**
```cpp
// ... after other initializations in the constructor ...
    gripper_command_pub_ = this->create_publisher<std_msgs::msg::Int32>("/gripper/command", ros_queue_size_);
    gripper_state_pub_ = this->create_publisher<std_msgs::msg::Int32>("/gripper/state", ros_queue_size_);
```

**2. In the `_joy_callback` function, modify the gripper control logic:**
```cpp
// ... inside _joy_callback ...
    // This is the new logic
    int32_t gripper_cmd_velocity = 0;
    int32_t gripper_current_pos = 0;
    auto gripper_cmd_msg = std_msgs::msg::Int32();
    auto gripper_state_msg = std_msgs::msg::Int32();

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

    // Publish both command and state
    gripper_cmd_msg.data = gripper_cmd_velocity;
    gripper_state_msg.data = gripper_current_pos;
    gripper_command_pub_->publish(gripper_cmd_msg);
    gripper_state_pub_->publish(gripper_state_msg);


    // Create the messages we might publish for the arm
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
// ... rest of the function
```
*(Note: This replaces the original `if/else if/else` block for the gripper with a more explicit version that also publishes the data.)*

---

## Step 2: Build and Source the Workspace

After saving the code changes, rebuild your workspace from the root (`~/rrc/xarm_ws`):

```bash
colcon build --packages-select xarm_moveit_servo
source install/setup.bash
```

---

## Step 3: Record the Data

### 3.1 Configure Camera Frequency (Optional)

The RealSense camera can be launched with different frame rates. The quality and size of your dataset will depend on these settings. Before recording, you can decide on and configure your desired camera profile.

First, ensure the camera node is running (`ros2 launch realsense2_camera rs_launch.py`). Then, in a new terminal, find the exact node name:

```bash
ros2 node list
# It will likely be /camera/camera
```

Use the node name to check the available profiles for the color and depth streams:

```bash
# Check available color profiles
ros2 param describe /camera/camera rgb_camera.color_profile

# Check available depth profiles
ros2 param describe /camera/camera depth_module.depth_profile
```

To launch the camera with a specific profile (e.g., 30 FPS), you would run:

```bash
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:="1280,720,30" depth_module.depth_profile:="1280,720,30"
```
**Note:** The camera is launched as a separate process from the xArm control launch file.

### 3.2 Start Recording

1.  Run the main launch file for the robot arm (and the camera launch file, if not already running).
    ```bash
    # Terminal 1: Launch the robot
    ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=<your_ip> joystick_type:=1 gripper_port:=/dev/ttyUSB0
    
    # Terminal 2: Launch the camera (if not already running)
    ros2 launch realsense2_camera rs_launch.py
    ```

2.  In a **new terminal** (after sourcing the workspace), run the following command to start recording. This will save all the specified topics into a bag file named `sync_data_bag`.

    ```bash
    ros2 bag record -o sync_data_bag \
    /lite6_traj_controller/controller_state \
    /ufactory/joint_states \
    /gripper/command \
    /gripper/state \
    /camera/camera/color/image_raw \
    /camera/camera/depth/image_rect_raw
    ```

3.  Move the robot and gripper with the joystick to collect your data. Press `Ctrl+C` in the bag record terminal to stop recording.

You will now have a rosbag file in the `sync_data_bag` directory containing all the necessary, timestamped data streams for your analysis.
