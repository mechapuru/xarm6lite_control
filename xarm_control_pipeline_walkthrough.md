# xArm6 Lite Control Pipeline: Senior Debugger Analysis

**Command Under Analysis:**
```bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.175 joystick_type:=1 gripper_port:=/dev/ttyUSB0
```

---

## Phase 1: Launch File Resolution

### Step 1.1: Entry Launch File
**File**: [`lite6_moveit_servo_realmove.launch.py`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_moveit_servo/launch/lite6_moveit_servo_realmove.launch.py)

```python
# Line 33-52: Includes the main launch file with parameters
robot_moveit_servo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('xarm_moveit_servo'), 
        'launch', 
        '_robot_moveit_servo_realmove.launch.py'  # ← THE REAL WORK HAPPENS HERE
    ])),
    launch_arguments={
        'robot_ip': robot_ip,           # 192.168.1.175
        'dof': '6',                      # 6 DOF for Lite6
        'robot_type': 'lite',            # lite type robot
        'ros2_control_plugin': 'uf_robot_hardware/UFRobotSystemHardware',
        'gripper_port': gripper_port,    # /dev/ttyUSB0
        'gripper_baudrate': gripper_baudrate,  # 57600 (default)
    }.items(),
)
```

### Step 1.2: Main Launch File - Node Instantiation Order
**File**: [`_robot_moveit_servo_realmove.launch.py`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_moveit_servo/launch/_robot_moveit_servo_realmove.launch.py)

The nodes are launched in this **exact order**:

| Order | Node | Package | What It Does |
|-------|------|---------|--------------|
| 1 | `joint_state_publisher` | `joint_state_publisher` | Merges joint states from multiple sources |
| 2 | `ros2_control_node` | `controller_manager` | Loads hardware interface, manages controllers |
| 3 | `lite6_traj_controller` | `controller_manager/spawner` | Spawns trajectory controller |
| 4 | ComposableNodeContainer containing: | | |
| 4a | `robot_state_publisher` | `robot_state_publisher` | Publishes TF from URDF |
| 4b | `servo_server` | `moveit_servo` | Converts twist→trajectory |
| 4c | `joy_to_servo_node` | `xarm_moveit_servo` | Converts joystick→twist |
| 4d | `joy_node` | `joy` | Reads Linux joystick device |

---

## Phase 2: ros2_control Initialization

### Step 2.1: Controller Manager + Hardware Interface Load
**File**: [`_ros2_control.launch.py`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_controller/launch/_ros2_control.launch.py)

```python
# Line 131-140: Creates ros2_control_node
ros2_control_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[
        robot_description,       # URDF with <ros2_control> tags
        ros2_control_params,     # lite6_controllers.yaml
        robot_params,            # xarm_params.yaml
    ],
)
```

### Step 2.2: Hardware Interface Initialization Sequence
**File**: [`uf_robot_system_hardware.cpp`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_controller/src/hardware/uf_robot_system_hardware.cpp)

```cpp
// STEP 2.2.1: on_init() called first (line 161)
CallbackReturn UFRobotSystemHardware::on_init(const hardware_interface::HardwareInfo& info) {
    // Line 181: Initialize xarm driver
    _init_ufactory_driver();
    
    // Line 183-186: Resize state/command vectors to 6 joints
    position_states_.resize(info_.joints.size(), NaN);  // [6]
    velocity_states_.resize(info_.joints.size(), NaN);  // [6]
    position_cmds_.resize(info_.joints.size(), NaN);    // [6]
    velocity_cmds_.resize(info_.joints.size(), NaN);    // [6]
}

// STEP 2.2.2: _init_ufactory_driver() (line 65)
void UFRobotSystemHardware::_init_ufactory_driver() {
    // Line 156: Initialize xarm_driver with TCP connection
    xarm_driver_.init(node_, robot_ip_, true);  // true = in_ros_control mode
    
    // Line 158: Get reference to joint state message
    joint_state_msg_ = xarm_driver_.get_joint_states();
}
```

### Step 2.3: XArmDriver TCP Connection to Robot
**File**: [`xarm_driver.cpp`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_api/src/xarm_driver.cpp)

```cpp
// Line 143: init() function
void XArmDriver::init(rclcpp::Node::SharedPtr& node, std::string &server_ip, bool in_ros_control) {
    // Line 189-204: Create XArmAPI instance with TCP connection
    arm = new XArmAPI(
        server_ip,           // "192.168.1.175"
        true,                // is_radian = true
        true,                // do_not_open = true (delayed connect)
        true,                // check_tcp_limit
        true,                // check_joint_limit
        true,                // check_cmdnum_limit
        false,               // check_robot_sn
        true,                // check_is_ready
        true,                // check_is_pause
        0,                   // max_callback_thread_count
        512,                 // max_cmdnum
        dof_,                // 6 for Lite6
        DEBUG_MODE,          // debug level
        report_type_         // "normal" or "dev"
    );
    
    // Line 211: Establish TCP connection to robot
    arm->connect();  // Opens TCP socket to 192.168.1.175:502
}
```

> [!IMPORTANT]
> **TCP Connection Details**: The xArm SDK connects to `robot_ip:502` (Modbus TCP port). Two connections are established: **control** (commands) and **report** (state feedback).

### Step 2.4: Hardware Interface Activation
**File**: [`uf_robot_system_hardware.cpp`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_controller/src/hardware/uf_robot_system_hardware.cpp)

```cpp
// Line 248: on_activate() - Called when controller activates
CallbackReturn UFRobotSystemHardware::on_activate(...) {
    // Line 250-254: Enable robot motors and set servo mode
    xarm_driver_.arm->clean_error();
    xarm_driver_.arm->clean_warn();
    xarm_driver_.arm->motion_enable(true);
    xarm_driver_.arm->set_mode(XARM_MODE::SERVO);  // Mode 1 = servo mode
    xarm_driver_.arm->set_state(XARM_STATE::START); // State 0 = start
}
```

---

## Phase 3: Joystick Input Processing

### Step 3.1: Linux Joystick → joy_node
**Node**: `joy::Joy` (from ros2 `joy` package)
**Config** (from launch file line 253-256):
```python
'autorepeat_rate': 30.0,   # 30 Hz publishing rate
'device_id': 0,            # /dev/input/js0
```

**Output Topic**: `/joy`
**Message Type**: `sensor_msgs/msg/Joy`

```
sensor_msgs/msg/Joy:
  header:
    stamp: <time>
    frame_id: "joy"
  axes: [float32; 8]    # For Xbox controller
    [0]: left_stick_lr    # -1.0 (left) to 1.0 (right)
    [1]: left_stick_fb    # -1.0 (forward) to 1.0 (back)
    [2]: LT trigger       #  1.0 (released) to -1.0 (pressed)
    [3]: right_stick_lr
    [4]: right_stick_fb
    [5]: RT trigger
    [6]: dpad_lr
    [7]: dpad_fb
  buttons: [int32; 11]
    [0]: A, [1]: B, [2]: X, [3]: Y
    [4]: LB, [5]: RB
    [6]: BACK, [7]: START
    [8]: GUIDE, [9]: LS, [10]: RS
```

### Step 3.2: JoyToServoPub - Joystick to Twist Conversion
**Node**: `JoyToServoPub`
**File**: [`xarm_joystick_input.cpp`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_moveit_servo/src/xarm_joystick_input.cpp)

```cpp
// Line 271-277: _joy_callback receives Joy message
void _joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // === GRIPPER CONTROL (lines 283-309) ===
    // Processed FIRST, in parallel with arm control
    int gripper_cmd_vel = 0;
    if (msg->buttons[A_BUTTON]) {       // A pressed → close
        gripper_cmd_vel = -GRIPPER_CLOSE_VEL;  // -400
    } else if (msg->buttons[B_BUTTON]) { // B pressed → open
        gripper_cmd_vel = GRIPPER_OPEN_VEL;    // +400
    }
    // Direct Dynamixel SDK call (bypasses ROS)
    gripper_controller_->move(gripper_cmd_vel, &gripper_pos);
    
    // === ARM CONTROL (lines 313-359) ===
    // Calls _convert_xbox360_joy_to_cmd()
}

// Line 186-245: Xbox conversion function
bool _convert_xbox360_joy_to_cmd(Joy msg, TwistStamped& twist, JointJog& joint) {
    // Line 189-196: Axis indices for WIRED controller
    int left_stick_fb   = 1;   // Forward/back
    int left_stick_lr   = 0;   // Left/right  
    int LT_trigger      = 2;   // Left trigger
    int right_stick_fb  = 4;   
    int right_stick_lr  = 3;
    int RT_trigger      = 5;   // Right trigger
    
    // Line 237-243: DIRECT AXIS MAPPING TO TWIST
    twist.twist.linear.x  = msg.axes[left_stick_fb];   // Forward/back → X
    twist.twist.linear.y  = msg.axes[left_stick_lr];   // Left/right → Y
    twist.twist.linear.z  = -0.5 * (msg.axes[LT_trigger] - msg.axes[RT_trigger]);  // Triggers → Z
    twist.twist.angular.y = msg.axes[right_stick_fb];  // Right stick → pitch
    twist.twist.angular.x = msg.axes[right_stick_lr];  // Right stick → roll
    twist.twist.angular.z = msg.buttons[LB] - msg.buttons[RB];  // Bumpers → yaw
    
    return true;  // Indicates twist command (not joint jog)
}
```

**Data Transformation Example**:
```
INPUT (Joy):
  axes[1] = -0.75  (left stick pushed forward 75%)
  axes[0] = 0.0    (left stick centered)
  axes[2] = 1.0    (LT released)
  axes[5] = -0.5   (RT half pressed)

OUTPUT (TwistStamped):
  twist.linear.x  = -0.75     # Forward at 75%
  twist.linear.y  = 0.0       # No lateral
  twist.linear.z  = -0.5 * (1.0 - (-0.5)) = -0.75  # Up at 75%
  twist.angular.* = 0.0       # No rotation
```

**Output Topic**: `/servo_server/delta_twist_cmds`
**Message Type**: `geometry_msgs/msg/TwistStamped`

---

## Phase 4: MoveIt Servo - Differential IK

### Step 4.1: ServoCalcs Main Loop
**File**: [`servo_calcs.h`](file:///opt/ros/humble/include/moveit_servo/servo_calcs.h)
**Config**: [`xarm_moveit_servo_config.yaml`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_moveit_servo/config/xarm_moveit_servo_config.yaml)

```yaml
publish_period: 0.034        # ~30 Hz output rate
command_in_type: "unitless"  # Input is [-1, 1] range
scale:
  linear: 0.3                # meters per second max
  rotational: 0.5            # radians per second max
  joint: 0.5                 # joint rad/s max
low_pass_filter_coeff: 2.0   # Smoothing on joint states
```

### Step 4.2: Twist to Joint Delta Conversion
```cpp
// servo_calcs.h line 125-126: cartesianServoCalcs()
bool cartesianServoCalcs(TwistStamped& cmd, JointTrajectory& joint_trajectory) {
    // STEP 1: Scale the unitless twist to physical velocities
    // Line 152: scaleCartesianCommand()
    Eigen::VectorXd delta_x = scaleCartesianCommand(cmd);
    // Result: delta_x = [linear.x * 0.3, linear.y * 0.3, linear.z * 0.3,
    //                    angular.x * 0.5, angular.y * 0.5, angular.z * 0.5]
    //         * publish_period (0.034s)
    
    // STEP 2: Get Jacobian matrix from current robot state
    // Uses: joint_model_group_->getJacobian(current_state_)
    Eigen::MatrixXd J = /* 6x6 Jacobian for Lite6 */;
    
    // STEP 3: Compute joint deltas using pseudo-inverse
    // delta_theta = J^(-1) * delta_x
    // For 6-DOF robot with 6-DOF twist, uses direct inverse
    Eigen::ArrayXd delta_theta_ = J.inverse() * delta_x;
    
    // STEP 4: Apply to current joint positions
    // Line 188: internalServoUpdate()
    for (int i = 0; i < 6; i++) {
        new_joint_pos[i] = current_joint_pos[i] + delta_theta_[i];
    }
}
```

**Data Transformation Example**:
```
INPUT (scaled twist):
  delta_x = [0.0102, 0.0, 0.0102, 0.0, 0.0, 0.0]  # 0.3m/s * 0.034s for x,z

JACOBIAN (example at some pose):
  J = [j11 ... j16]
      [... 6x6 ...]
      
OUTPUT (joint deltas in radians):
  delta_theta = J^(-1) * delta_x
  ≈ [0.002, -0.003, 0.001, 0.0, 0.002, 0.0]  # small joint increments
```

**Output Topic**: `/lite6_traj_controller/joint_trajectory`
**Message Type**: `trajectory_msgs/msg/JointTrajectory`

```
trajectory_msgs/msg/JointTrajectory:
  header:
    stamp: <current_time>
    frame_id: "link_base"
  joint_names: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
  points:
    - positions: [0.123, -0.456, 0.789, 0.0, 1.234, 0.0]  # 6 joint positions
      velocities: []      # Empty for position-only command
      accelerations: []
      effort: []
      time_from_start:
        sec: 0
        nanosec: 50000000  # 50ms
```

---

## Phase 5: Trajectory Controller - Interpolation

### Step 5.1: JointTrajectoryController Configuration
**Config**: [`lite6_controllers.yaml`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_controller/config/lite6_controllers.yaml)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 250  # 250 Hz control loop

lite6_traj_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    state_publish_rate: 100.0
    action_monitor_rate: 20.0
```

### Step 5.2: Trajectory Interpolation
**File**: [`trajectory.hpp`](file:///opt/ros/humble/include/joint_trajectory_controller/joint_trajectory_controller/trajectory.hpp)

```cpp
// Line 127-130: interpolate_between_points()
void interpolate_between_points(
    const rclcpp::Time& time_a, const JointTrajectoryPoint& state_a,  // Start
    const rclcpp::Time& time_b, const JointTrajectoryPoint& state_b,  // End
    const rclcpp::Time& sample_time,                                  // Now
    JointTrajectoryPoint& output                                      // Interpolated
) {
    // Lines 105-117 (from docs): Automatic spline selection
    // - Position only → LINEAR interpolation
    // - Position + velocity → CUBIC spline
    // - Position + velocity + accel → QUINTIC spline
    
    // For this system (position-only from servo): LINEAR
    double ratio = (sample_time - time_a) / (time_b - time_a);
    for (int i = 0; i < 6; i++) {
        output.positions[i] = state_a.positions[i] + 
                              ratio * (state_b.positions[i] - state_a.positions[i]);
    }
}
```

**Timing Analysis**:
```
Servo publishes at:    30 Hz (every 33.3ms)
Controller runs at:   250 Hz (every 4ms)

Between two servo messages, the controller interpolates ~8 intermediate positions:
  t=0ms:   Use trajectory point 0
  t=4ms:   Interpolate 12.5% toward point 1
  t=8ms:   Interpolate 25% toward point 1
  ...
  t=33ms:  Receive new trajectory point, become point 0
```

---

## Phase 6: Hardware Interface - Write to Robot

### Step 6.1: Read from Robot SDK
**File**: [`uf_robot_system_hardware.cpp`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_controller/src/hardware/uf_robot_system_hardware.cpp)

```cpp
// Line 295: read() - Called at 250 Hz by controller manager
hardware_interface::return_type UFRobotSystemHardware::read(...) {
    // Line 301: Get current joint states from robot
    read_code_ = xarm_driver_.update_joint_states(initialized_);
    
    if (read_code_ == 0 && read_ready_) {
        // Line 311-314: Copy to state vectors
        for (int j = 0; j < 6; j++) {
            position_states_[j] = joint_state_msg_->position[j];  // radians
            velocity_states_[j] = joint_state_msg_->velocity[j];  // rad/s
        }
    }
}
```

### Step 6.2: Write to Robot SDK
```cpp
// Line 339: write() - Called at 250 Hz
hardware_interface::return_type UFRobotSystemHardware::write(...) {
    // Line 376-377: Convert double[] to float[] for SDK
    for (int i = 0; i < 6; i++) { 
        cmds_float_[i] = (float)position_cmds_[i];  // double → float
    }
    
    // Line 380: Only send if changed or 1 second elapsed
    if (time_since_last > 1.0 || _check_cmds_is_change(...)) {
        // Line 383: THE CRITICAL SDK CALL
        cmd_ret = xarm_driver_.arm->set_servo_angle_j(
            cmds_float_,  // float[7] joint positions in radians
            0,            // speed (ignored in servo mode)
            0,            // acceleration (ignored)
            0             // time (ignored)
        );
    }
}
```

**Data at write():**
```cpp
cmds_float_[6] = {
    0.123456f,   // joint1 position (radians)
   -0.789012f,   // joint2
    1.234567f,   // joint3
    0.000000f,   // joint4
    0.567890f,   // joint5
    0.000000f    // joint6
};
```

### Step 6.3: xArm SDK → TCP Packet
```cpp
// Inside xarm_driver_.arm->set_servo_angle_j():
// Converts float[7] to binary packet and sends via TCP

// Packet structure (simplified):
// [HEADER][LEN][CMD_ID=0x11][ANGLES_FLOAT_LE][CHECKSUM]
// Each float is 4 bytes, little-endian IEEE 754
```

---

## Phase 7: Complete Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│ LINUX KERNEL: /dev/input/js0                                                    │
│ Raw HID events from USB joystick                                               │
└────────────────────────────────────────┬────────────────────────────────────────┘
                                         │ 30 Hz
                                         ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ joy::Joy (joy_node)                                                             │
│ Converts HID → sensor_msgs/Joy                                                  │
│ axes[8]: float32 normalized to [-1, 1]                                         │
│ buttons[11]: int32 (0 or 1)                                                    │
└────────────────────────────────────────┬────────────────────────────────────────┘
                                         │ /joy @ 30 Hz
                                         ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ JoyToServoPub (_joy_callback @ xarm_joystick_input.cpp:271)                    │
│                                                                                 │
│ axes[1] → twist.linear.x                                                       │
│ axes[0] → twist.linear.y                                                       │
│ -0.5*(axes[2]-axes[5]) → twist.linear.z                                        │
│ axes[4] → twist.angular.y                                                      │
│ axes[3] → twist.angular.x                                                      │
│ buttons[LB]-buttons[RB] → twist.angular.z                                      │
│                                                                                 │
│ Values remain unitless [-1, 1]                                                 │
└────────────────────────────────────────┬────────────────────────────────────────┘
                                         │ /servo_server/delta_twist_cmds @ 30 Hz
                                         ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ moveit_servo::ServoNode → ServoCalcs                                           │
│                                                                                 │
│ 1. SCALING (scaleCartesianCommand):                                            │
│    linear *= 0.3 m/s × 0.034s = 0.0102 m per cycle                            │
│    angular *= 0.5 rad/s × 0.034s = 0.017 rad per cycle                        │
│                                                                                 │
│ 2. INVERSE KINEMATICS (cartesianServoCalcs):                                   │
│    delta_theta[6] = Jacobian⁻¹ × delta_x[6]                                   │
│                                                                                 │
│ 3. POSITION UPDATE:                                                            │
│    new_pos[i] = current_pos[i] + delta_theta[i]                               │
│                                                                                 │
│ Output: JointTrajectory with 1 point, 50ms duration                           │
└────────────────────────────────────────┬────────────────────────────────────────┘
                                         │ /lite6_traj_controller/joint_trajectory
                                         ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ joint_trajectory_controller (250 Hz)                                           │
│                                                                                 │
│ INTERPOLATION (trajectory.hpp:127):                                            │
│ - Receives trajectory at 30 Hz                                                 │
│ - Interpolates linearly every 4ms (250 Hz)                                    │
│ - ~8 interpolated positions per trajectory segment                            │
│                                                                                 │
│ Writes to hardware interface state variables:                                  │
│ position_cmds_[6] (double, radians)                                           │
└────────────────────────────────────────┬────────────────────────────────────────┘
                                         │ ros2_control internal (no topic)
                                         ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ UFRobotSystemHardware::write() (250 Hz)                                        │
│                                                                                 │
│ uf_robot_system_hardware.cpp:376-383:                                          │
│ double position_cmds_[6] → float cmds_float_[6]                               │
│                                                                                 │
│ xarm_driver_.arm->set_servo_angle_j(cmds_float_, 0, 0, 0)                     │
│                                                                                 │
│ SDK converts float[6] to binary TCP packet                                    │
└────────────────────────────────────────┬────────────────────────────────────────┘
                                         │ TCP to 192.168.1.175:502
                                         ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ xArm Controller Hardware                                                        │
│                                                                                 │
│ - Receives position command via TCP                                            │
│ - Internal PID control loop (>250 Hz)                                         │
│ - Sends current to motor drivers                                              │
│ - Motors move to commanded positions                                          │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## Timing Summary

| Stage | Frequency | Latency |
|-------|-----------|---------|
| Joystick HID polling | 100-1000 Hz | ~1ms |
| joy_node publish | 30 Hz | ~1ms |
| JoyToServoPub callback | 30 Hz | <1ms |
| MoveIt Servo calculation | 30 Hz | ~5ms |
| Trajectory controller | 250 Hz | <1ms |
| Hardware write | 250 Hz | <1ms |
| TCP to robot | N/A | ~1-2ms |
| Robot internal loop | >250 Hz | <1ms |
| **Total joystick-to-motion** | | **~15-25ms** |

---

## Key Source Files Reference

| File | Purpose | Critical Lines |
|------|---------|----------------|
| [`lite6_moveit_servo_realmove.launch.py`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_moveit_servo/launch/lite6_moveit_servo_realmove.launch.py) | Entry launch | 33-52 |
| [`_robot_moveit_servo_realmove.launch.py`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_moveit_servo/launch/_robot_moveit_servo_realmove.launch.py) | Main launch | 206-261 (container) |
| [`xarm_joystick_input.cpp`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_moveit_servo/src/xarm_joystick_input.cpp) | Joy→Twist | 186-245, 271-309 |
| [`servo_calcs.h`](file:///opt/ros/humble/include/moveit_servo/servo_calcs.h) | IK header | 125-126, 152-158 |
| [`trajectory.hpp`](file:///opt/ros/humble/include/joint_trajectory_controller/joint_trajectory_controller/trajectory.hpp) | Interpolation | 127-130 |
| [`uf_robot_system_hardware.cpp`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_controller/src/hardware/uf_robot_system_hardware.cpp) | HW interface | 295-336, 339-396 |
| [`xarm_driver.cpp`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_api/src/xarm_driver.cpp) | SDK wrapper | 143-211, 317-361 |
| [`lite6_controllers.yaml`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_controller/config/lite6_controllers.yaml) | Controller config | All |
| [`xarm_moveit_servo_config.yaml`](file:///home/paddy/rrc/xarm_ws/src/xarm_ros2/xarm_moveit_servo/config/xarm_moveit_servo_config.yaml) | Servo config | All |
