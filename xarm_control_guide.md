# xArm Control Guide

Quick reference for controlling the xArm6 Lite with a joystick.

> **For technical details and debugging, see [`xarm_control_pipeline_walkthrough.md`](xarm_control_pipeline_walkthrough.md)**

---

## 1. Launch Command

```bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py \
  robot_ip:=192.168.1.175 \
  joystick_type:=1 \
  gripper_port:=/dev/ttyUSB0 \
  gripper_baudrate:=57600
```

---

## 2. Joystick Controls (Xbox 360)

| Input | Action |
|-------|--------|
| **Left Stick** (F/B) | Linear X (forward/back) |
| **Left Stick** (L/R) | Linear Y (left/right) |
| **LT / RT Triggers** | Linear Z (down/up) |
| **Right Stick** (F/B) | Angular Y (pitch) |
| **Right Stick** (L/R) | Angular X (roll) |
| **LB / RB Bumpers** | Angular Z (yaw) |
| **D-Pad** (L/R) | Joint 1 jog |
| **D-Pad** (U/D) | Joint 2 jog |
| **A Button** | Open gripper |
| **B Button** | Close gripper |

---

## 3. Configuration Files

### MoveIt Servo Config
**File**: `xarm_moveit_servo/config/xarm_moveit_servo_config.yaml`

```yaml
publish_period: 0.034        # Output rate (~30 Hz)
command_in_type: "unitless"  # Input range [-1, 1]

scale:
  linear: 0.1                # Max linear velocity (m/s)
  rotational: 0.2            # Max angular velocity (rad/s)
  joint: 0.2                 # Max joint velocity (rad/s)

low_pass_filter_coeff: 50.0  # Joint state smoothing
```

### Controller Config
**File**: `xarm_controller/config/lite6_controllers.yaml`

```yaml
lite6_traj_controller:
  ros__parameters:
    update_rate: 250             # Control loop frequency (Hz)
    state_publish_rate: 30.0     # Status topic rate (Hz)
    command_interfaces: [position, velocity]
```

---

## 4. Tuning Tips

| Goal | Parameter | Change |
|------|-----------|--------|
| **Faster movement** | `scale.linear` | Increase (e.g., 0.2) |
| **Smoother motion** | `low_pass_filter_coeff` | Increase (e.g., 60.0) |
| **More responsive** | `publish_period` | Decrease (e.g., 0.01667 for 60Hz) |
| **Slower rotation** | `scale.rotational` | Decrease (e.g., 0.1) |

---

## 5. Data Collection

### Record Bag
```bash
ros2 bag record -o data_bag \
  --compression-mode file --compression-format zstd \
  /lite6_traj_controller/controller_state \
  /ufactory/joint_states \
  /gripper/command \
  /gripper/state \
  /camera/camera/color/image_raw \
  /camera/camera/depth/image_rect_raw
```

### Key Topics
| Topic | Description |
|-------|-------------|
| `/ufactory/joint_states` | Current joint positions/velocities |
| `/gripper/state` | Normalized gripper position (0.0-1.0) |
| `/lite6_traj_controller/controller_state` | Controller feedback |

---

## 6. Troubleshooting

| Issue | Solution |
|-------|----------|
| Robot not moving | Check `ros2 topic echo /joy` for input |
| Jerky motion | Increase `low_pass_filter_coeff` |
| Gripper not responding | Verify `gripper_port` is correct |
| DDS buffer overflow | Reduce `update_rate` to 200 |
