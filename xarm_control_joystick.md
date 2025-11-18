# Controlling the xArm6 Lite with a Joystick

This document provides a detailed, granular breakdown of the process for controlling the xArm6 Lite robot using an XBOX 360 joystick. The entire workflow, from launch command to hardware execution, is verified against the source code and configuration files.

## 1. Launch Command

To begin, source your ROS 2 workspace and execute the following command, replacing `<your_ip>` with the robot's IP address. This example assumes a wired XBOX 360 controller (`joystick_type:=1`).

```bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.175 joystick_type:=1 gripper_port:=/dev/ttyUSB0 gripper_baudrate:=57600
```

## 2. System Architecture Overview

The control system is a pipeline that transforms raw joystick input into smooth, controlled robot motion. At a high level, the architecture is as follows:

**Joystick Hardware** -> **`joy` Node** -> **`JoyToServoPub` Node** -> **`moveit_servo` Node** -> **`ros2_control` Framework** -> **Robot Hardware**

- **Input**: Raw joystick axis and button states.
- **Transformation 1**: Joystick states are converted into a desired end-effector velocity (`TwistStamped` message).
- **Transformation 2**: The desired velocity is converted into a short-term joint trajectory (`JointTrajectory` message) via inverse kinematics.
- **Transformation 3**: The joint trajectory is interpolated at a high frequency into single position commands.
- **Execution**: The position commands are sent to the robot's hardware controller.

## 3. Component Deep Dive

Here is a detailed look at each file, node, and configuration involved in the process.

---

### 3.1. Launch Files

#### `lite6_moveit_servo_realmove.launch.py`
- **Purpose**: The top-level launch file specific to the `lite6`. Its main job is to call the generic real-time servo launch file while passing the correct robot-specific parameters (`robot_type:='lite'`, `dof:='6'`).

#### `_robot_moveit_servo_realmove.launch.py`
- **Purpose**: The core launch file that assembles and launches all the necessary components for the joystick control system. It is responsible for loading the MoveIt configuration, the `ros2_control` pipeline, RViz, and the container of nodes that perform the actual work.

---

### 3.2. Nodes and Configuration

#### `joy::Joy` Node
- **Purpose**: A standard ROS 2 node that reads data directly from the connected joystick hardware via the operating system.
- **Frequency**: High, typically **>100 Hz**, depending on the OS and driver.
- **Output**: Publishes raw joystick data (`sensor_msgs/msg/Joy`) to the `/joy` topic.

#### `xarm_moveit_servo::JoyToServoPub` Node
- **Source File**: `xarm_moveit_servo/src/xarm_joystick_input.cpp`
- **Purpose**: Translates raw joystick messages into meaningful velocity commands for `moveit_servo`. This node can publish two different types of commands depending on the user's input.
- **Frequency**: Event-driven; runs every time a message is received on the `/joy` topic.
- **Logic & Transformation**:
    1. The `_joy_callback` function is triggered by new `/joy` messages.
    2. It determines which command mode to use (operational space or joint space) based on the buttons pressed.
    3. It calls `_filter_twist_msg` (for operational space commands), which applies a **deadband filter**. Any velocity command smaller than `0.2` is set to `0` to prevent robot drift from minor, unintentional joystick movements.
- **Output Modes**:
    - **Operational Space Control (Primary Mode)**: When using the analog sticks and triggers, the node publishes a `geometry_msgs/msg/TwistStamped` message to the `/servo_server/delta_twist_cmds` topic. This commands the end-effector's velocity in 3D space.
        - **Left Stick (F/B)**: Linear X Velocity (forward/backward)
        - **Left Stick (L/R)**: Linear Y Velocity (left/right)
        - **Left & Right Triggers**: Linear Z Velocity (down/up)
        - **Right Stick (F/B)**: Angular Y Velocity (pitch)
        - **Right Stick (L/R)**: Angular X Velocity (roll)
        - **Left & Right Bumpers**: Angular Z Velocity (yaw)
    - **Joint Space Control (Secondary Mode)**: When using the D-Pad or face buttons (A, B, X, Y), the node publishes a `control_msgs/msg/JointJog` message to the `/servo_server/delta_joint_cmds` topic. This allows for direct control ("jogging") of individual joints.
        - **D-Pad (L/R)**: Joint 1 Velocity
        - **D-Pad (U/D)**: Joint 2 Velocity
        - **Y & A Buttons**: Joint 5 Velocity
        - **B & X Buttons**: Joint 6 Velocity

#### `moveit_servo::ServoNode`
- **Source File**: Pre-compiled from the `moveit_servo` package.
- **Configuration**: `xarm_moveit_servo/config/xarm_moveit_servo_config.yaml`
- **Purpose**: The intelligent core of the system. It receives desired end-effector velocities and calculates the required joint commands to achieve them safely.
- **Frequency**: Publishes its output at **~30 Hz** by default, as defined by `publish_period: 0.034`.
- **Logic & Transformation**:
    1. Runs a continuous internal control loop.
    2. In the loop, it reads the incoming `TwistStamped` command and the robot's current joint state (from `/joint_states`).
    3. **Input Smoothing**: Before using the joint state data, it is passed through a low-pass filter controlled by the `low_pass_filter_coeff` parameter. This smooths out high-frequency encoder noise, providing a more stable input to the kinematics solver.
    4. It solves the inverse differential kinematics using the robot's Jacobian matrix to determine the joint velocities needed to produce the commanded end-effector velocity.
    5. This process inherently handles interpolation, singularity avoidance, and joint limit checking.
- **Output**: Publishes a `trajectory_msgs/msg/JointTrajectory` message to the `/lite6_traj_controller/joint_trajectory` topic. By default (`publish_joint_positions: true`), this trajectory primarily contains populated `position` goals for a short time horizon.

#### `joint_trajectory_controller`
- **Source File**: Pre-compiled from the `ros2_control` packages.
- **Configuration**: `xarm_controller/config/lite6_controllers.yaml`
- **Purpose**: To receive a trajectory from a high-level planner (like `moveit_servo`) and ensure the robot's hardware follows it precisely.
- **Frequency**: The internal control loop runs at **150 Hz** (defined by `update_rate: 150`), but it publishes its status to the `/lite6_traj_controller/controller_state` topic at a lower rate of **~25 Hz** (defined by `state_publish_rate: 25.0`). This is a common optimization to reduce network traffic.
- **Logic & Transformation**:
    1. Receives the `JointTrajectory` message from `moveit_servo` (~30 Hz).
    2. At a much higher rate (150 Hz), it **interpolates along this trajectory** to calculate the exact joint command required for that specific moment in time (every ~6.7ms).
    3. It compares this desired command to the actual measured state and computes the final command to send to the hardware.
    4. The `interface_name: position` parameter explicitly tells the controller to send **position commands** to the hardware interface.
- **Output**: A stream of position commands sent to the hardware interface at 150 Hz.

#### `UFRobotSystemHardware` (Hardware Interface)
- **Source File**: `xarm_controller/src/hardware/uf_robot_system_hardware.cpp`
- **Purpose**: This is a critical `ros2_control` plugin that acts as the direct bridge between the generic ROS 2 control framework and the proprietary UFactory robot SDK. It has two primary responsibilities in each control cycle.
- **`read()` method**: This function is called to get the latest state (position and velocity) from the actual robot hardware. This information is used to populate the `/joint_states` topic, which provides the feedback needed for all other nodes (like `moveit_servo`) to function correctly.
- **`write()` method**: This function receives the final command from the `joint_trajectory_controller` and sends it to the robot. It contains logic to handle the different control modes:
    - **In Position Control Mode (Default)**: The `write` method calls the SDK function **`xarm_driver_.arm->set_servo_angle_j(...)`**. This is the robot's real-time streaming interface for joint positions. The call is wrapped in an optimization check to prevent sending redundant commands if the target position has not changed.
    - **In Velocity Control Mode**: If `velocity_control:=true` is set at launch (and the controllers are configured correctly), the `write` method instead calls the SDK function **`xarm_driver_.arm->vc_set_joint_velocity(...)`**. This is the robot's real-time streaming interface for joint velocities.

## 4. Advanced Concepts & Tuning



### Control Modes (`position` vs. `velocity` vs. `effort`)

- The `velocity_control` and `effort_control` launch arguments modify the robot's URDF to change the command interface exposed by the hardware to `ros2_control`.

- To use a different mode (e.g., `velocity`), you must perform a complete chain of configuration: 

    1. Set the launch argument (`velocity_control:=true`).

    2. Modify `lite6_controllers.yaml` to change `interface_name` from `position` to `velocity`.

    3. Crucially, you should also modify `xarm_moveit_servo_config.yaml` to set `publish_joint_velocities: true`. This ensures `moveit_servo` provides the ideal target velocities to the controller as a feed-forward command, leading to better and more accurate performance.

- **Impedance Control**: `effort_control` is a prerequisite for impedance control, a strategy that makes the robot compliant. However, this package does not provide a ready-to-use impedance controller.



#### A Note on Velocity-Only Control

For this specific stack, sending only velocity commands to the `joint_trajectory_controller` is not recommended as it will lead to poor performance and drift. The controller's internal PID loops rely on **position error** (`target_position - current_position`) to function correctly.

- Without a position goal, the controller has no reference to correct against. Small errors in velocity tracking will accumulate over time, causing the robot's actual position to drift away from the intended path.

- For accurate arm control, the trajectory message should always contain `positions`. The `velocities` are best used as a supplemental "feed-forward" term to help the controller achieve the position goal more efficiently, not as a replacement for it.



### Input Trajectory Smoothing

The calculated velocity and jerk plots may appear noisy, even when the position trajectory looks smooth. This is often because `moveit_servo` is reacting to small, high-frequency noise from the joint state encoders.

To address this at the source, you can tune the low-pass filter that `moveit_servo` applies to the incoming `/joint_states` messages. This is controlled by the `low_pass_filter_coeff` parameter in `xarm_moveit_servo/config/xarm_moveit_servo_config.yaml`.

```yaml
## Incoming Joint State properties
low_pass_filter_coeff: 2.0  # Larger --> trust the filtered data more, trust the measurements less.
```

- **How it works**: This filter smooths the joint states that `moveit_servo` uses as feedback. By providing a less noisy input to its internal calculations, `moveit_servo` will generate a smoother output trajectory (`JointTrajectory` message).
- **Tuning**: The default value is `2.0`. Increasing this to a higher value (e.g., `5.0` or `10.0`) will apply more aggressive filtering, leading to a smoother plan with less jitter in velocity and acceleration. This is a key parameter for improving the quality of the generated motion.



### Understanding Control Frequencies

- **`moveit_servo` (~30-60 Hz)**: This node sends a new "mini-plan" or trajectory to the controller. Its frequency is governed by `publish_period`.

- **`joint_trajectory_controller` (150 Hz)**: This controller runs at a much higher frequency, defined by `update_rate`. In each cycle, it interpolates the plan from `moveit_servo` and sends a single, fine-grained command to the hardware interface.

- **Hardware Capability**: The ROS 2 driver sends commands at 150 Hz because the `update_rate` is 150. This indicates that the developers found this rate to be effective. The robot's own internal controller will process this stream of commands using its own high-frequency loops (e.g., >>200 Hz), using the most recent command it has received as its target.



### Tuning the Frequency

- You can increase the responsiveness of the system by increasing the `moveit_servo` output frequency. To set it to **60 Hz**, change the following line in `xarm_moveit_servo/config/xarm_moveit_servo_config.yaml`:

  ```yaml

  # From (approx. 30 Hz)

  publish_period: 0.034

  # To (approx. 60 Hz)

  publish_period: 0.01667

  ```

- **Effect**: This will make `moveit_servo` send new "mini-plans" to the `joint_trajectory_controller` more often, resulting in motion that is more reactive to your joystick input. The `joint_trajectory_controller` will still perform its own 150 Hz interpolation on these more frequently updated plans.



## 5. Tuning Speed and Responsiveness



For fine-grained control over the robot's speed when using a joystick, you can configure `moveit_servo` to use "unitless" commands. This is done by editing `xarm_moveit_servo/config/xarm_moveit_servo_config.yaml`.



1.  **Set Command Type to `unitless`**:

    Change the `command_in_type` parameter to `"unitless"`. This tells `moveit_servo` to interpret incoming commands as values in the range `[-1, 1]`.



    ```yaml

    command_in_type: "unitless"

    ```



2.  **Adjust Scaling Factors**:

    The `scale` parameters determine the maximum velocity when using `unitless` commands. You can adjust these values to make the robot move faster or slower.



    ```yaml

    scale:

      # Max linear velocity in [m/s]. Applies to X, Y, and Z axes.

      linear:  0.1

      # Max angular velocity in [rad/s].

      rotational:  0.2

      # Max joint velocity for joint-space commands.

      joint: 0.2

    ```



    -   `linear`: Affects the speed of linear movements (X, Y, Z).

    -   `rotational`: Affects the speed of rotational movements.



#### Z-Axis Speed Anomaly with Xbox Controllers



When using an Xbox controller, you may notice the Z-axis moves significantly faster than the X and Y axes, even with the same `linear` scaling.



-   **Cause**: The joystick sticks (controlling X and Y) produce output in a `[-1.0, 1.0]` range. However, the triggers (controlling Z) are treated as separate axes, and their combined output can be in a `[-2.0, 2.0]` range. This larger input value results in a higher final velocity for the Z-axis.

-   **Solution**: The file `xarm_moveit_servo/src/xarm_joystick_input.cpp` has been modified to halve the Z-axis command value, normalizing it to the same `[-1.0, 1.0]` range as the other axes. This ensures that the `linear` scaling factor applies consistently across all three linear axes.



With this fix, you can now tune the `linear` scale parameter to get your desired speed for all directions of Cartesian movement.



## 6. Introspection and Analysis



To dive deeper into the performance and smoothness of the control pipeline, you can record the trajectory data at key points and visualize it. The following steps allow you to inspect the "plan" from `moveit_servo` and the final "output" from the `joint_trajectory_controller`.



### 6.1. Real-time Introspection



You can inspect the key topics in the pipeline in real-time using `ros2 topic echo`.



-   **`joint_trajectory_controller` State (~25 Hz):** To see the final interpolated command sent to the hardware interface, along with detailed controller status (feedback, error, etc.), use:

    ```bash

    ros2 topic echo /lite6_traj_controller/controller_state

    ```

    *   The `output.positions` field in this message shows the command sent to the hardware. While this topic publishes at ~25 Hz, the commands are being calculated and sent internally at 150 Hz.



-   **`moveit_servo` Output (~30 Hz):** To see the direct output from `moveit_servo` (the "mini-plan" sent to the controller), use:

    ```bash

    ros2 topic echo /lite6_traj_controller/joint_trajectory

    ```



### 6.2. Data Collection for Offline Analysis



For more detailed analysis, you can record the data from these topics to a file using `ros2 bag`. This is the best method for comparing the planned trajectory to the executed one.



1.  **Start Recording:** While the robot is running, open a new terminal and run the following command. It will save the data to a directory named `trajectory_data_bag`.

    ```bash

    ros2 bag record -o trajectory_data_bag /lite6_traj_controller/joint_trajectory /lite6_traj_controller/controller_state

    ```



2.  **Perform Actions:** Move the robot with the joystick. The data will be captured.



3.  **Stop Recording:** Press `Ctrl+C` in the terminal where the bag command is running.



### 6.3. Parsing and Visualization



To visualize the collected data, you can use the provided Python scripts. The goal is to visualize the smoothness of each trajectory by plotting both the joint positions and their calculated velocities over time. A spiky velocity plot indicates fluctuations or jerkiness in the motion.



1.  **Parse the Bag File:** The `parse_bag.py` script reads the recorded bag file and extracts the position data into a single, easy-to-use CSV file named `trajectory_analysis.csv`.

    ```bash

    python3 parse_bag.py

    ```



2.  **Visualize the Trajectories:** The `plot_trajectories.py` script reads the CSV file and generates four plots to help analyze smoothness:

    *   MoveIt Servo Plan (Position)

    *   MoveIt Servo Plan (Calculated Velocity)

    *   Controller Output (Position)

    *   Controller Output (Calculated Velocity)



    To run the script (installing libraries if needed), use:

    ```bash

    pip install pandas matplotlib

    python3 plot_trajectories.py

    ```

By comparing the "Calculated Velocity" plots, you can determine if any fluctuations are originating from the `moveit_servo` plan or from the `joint_trajectory_controller`'s execution.



1. Checked by keeping the xbox controller stationary. For stationary commands, all the topics remain stationary.
2. Commented out move_servo_j in the hardware interface file to see what happens.
3. The controller is working. No input shows zero across the /joy topic.
4. While /lite6_traj_controller/joint_trajectory remains stationary, in the rostopic /lite6_traj_controller/controller_state there is error in the position space, even with zero external commands and stationary robot.
5. Going to try to measure the error published by /lite6_traj_controller/controller_state while Changing filter to 1 or 0.2 in xarm_joystick_input.cpp 
6. Made the low_latency_mode to true in xarm_moveit_servo_config
7. set the low_latency_mode to false again, and low_pass_filter to 0.2 (original) but lite6_controller publish_rate is increased to 150.
8. Changing update rate to 250, and publish rate to 150. 
9. changing joint topic to ufactory/joint_state. new_topic
10. changed publish_rate of the arm back to 25. new_topic_new_state.
11. Changing low_pass_filter to 10. low_10_new.
12. Making low_latency_mode true in moveit_servo. low_lat XXX Not Working
13. Decreasing publish_rate of moveit.
14. Back to normal publish_rate
