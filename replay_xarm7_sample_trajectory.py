#!/usr/bin/env python3
"""Replay one recorded xArm7 joint trajectory without invoking a planner.

Commands:
  ros2 launch xarm_controller _ros2_control.launch.py robot_ip:=<ROBOT_IP> dof:=7 robot_type:=xarm add_gripper:=true
  ros2 run controller_manager spawner joint_state_broadcaster
  ros2 run controller_manager spawner xarm7_traj_controller
  ./replay_xarm7_sample_trajectory.py

Optional:
  ./replay_xarm7_sample_trajectory.py --sample xarm7_sample_data/episode_123.npz
  ./replay_xarm7_sample_trajectory.py --first-position-duration 5.0
  ./replay_xarm7_sample_trajectory.py --dry-run
"""

import argparse
from pathlib import Path
import sys
import time

import numpy as np


JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]


def _duration(seconds: float):
    from builtin_interfaces.msg import Duration

    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


def _default_sample() -> Path:
    samples = sorted(Path("xarm7_sample_data").glob("*.npz"))
    if not samples:
        raise FileNotFoundError("no .npz files found in xarm7_sample_data")
    return samples[0]


def load_positions(path: Path, key: str) -> np.ndarray:
    data = np.load(path)
    if key not in data.files:
        raise KeyError(f"{path} does not contain key {key!r}; available keys: {data.files}")
    positions = np.asarray(data[key], dtype=np.float64)
    if positions.ndim != 2 or positions.shape[1] != 7:
        raise ValueError(f"expected {key!r} to have shape (N, 7), got {positions.shape}")
    if len(positions) < 2:
        raise ValueError("trajectory must contain at least two joint samples")
    if not np.all(np.isfinite(positions)):
        raise ValueError("trajectory contains NaN or infinite joint positions")
    return positions


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay one xarm7_sample_data .npz trajectory on xArm7 via FollowJointTrajectory."
    )
    parser.add_argument("--sample", type=Path, default=None, help="Path to a sample .npz file.")
    parser.add_argument("--key", default="action", help="NPZ key containing the (N, 7) joint trajectory.")
    parser.add_argument("--rate", type=float, default=20.0, help="Recording rate in Hz.")
    parser.add_argument(
        "--action-name",
        default="/xarm7_traj_controller/follow_joint_trajectory",
        help="FollowJointTrajectory action name.",
    )
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument(
        "--first-position-duration",
        type=float,
        default=5.0,
        help="Requested move time for reaching the first recorded joint pose with /xarm/set_servo_angle.",
    )
    parser.add_argument(
        "--first-position-speed",
        type=float,
        default=0.35,
        help="Max joint speed for the initial /xarm/set_servo_angle reach.",
    )
    parser.add_argument(
        "--first-position-acc",
        type=float,
        default=10.0,
        help="Joint acceleration for the initial /xarm/set_servo_angle reach.",
    )
    parser.add_argument(
        "--first-position-timeout",
        type=float,
        default=20.0,
        help="Timeout for the initial /xarm/set_servo_angle reach.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Load and validate only; do not contact ROS.")
    return parser.parse_args()


def run_replay(args: argparse.Namespace, positions: np.ndarray) -> int:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node

    from control_msgs.action import FollowJointTrajectory
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectoryPoint
    from xarm_msgs.srv import Call, MoveJoint, SetInt16, SetInt16ById

    class XArm7SampleReplayer(Node):
        def __init__(self, action_name: str, joint_state_topic: str):
            super().__init__("xarm7_sample_replayer")
            self._current_positions = None
            self._action_client = ActionClient(self, FollowJointTrajectory, action_name)
            self.create_subscription(JointState, joint_state_topic, self._joint_state_cb, 10)
            self._clean_error = self.create_client(Call, "/xarm/clean_error")
            self._clean_warn = self.create_client(Call, "/xarm/clean_warn")
            self._motion_enable = self.create_client(SetInt16ById, "/xarm/motion_enable")
            self._set_mode = self.create_client(SetInt16, "/xarm/set_mode")
            self._set_state = self.create_client(SetInt16, "/xarm/set_state")
            self._set_servo_angle = self.create_client(MoveJoint, "/xarm/set_servo_angle")

        def _joint_state_cb(self, msg: JointState) -> None:
            name_to_index = {name: i for i, name in enumerate(msg.name)}
            if all(name in name_to_index for name in JOINT_NAMES):
                self._current_positions = np.array(
                    [msg.position[name_to_index[name]] for name in JOINT_NAMES],
                    dtype=np.float64,
                )

        def wait_for_current_positions(self, timeout: float) -> np.ndarray:
            deadline = time.monotonic() + timeout
            while rclpy.ok() and self._current_positions is None and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self._current_positions is None:
                raise TimeoutError(f"did not receive {JOINT_NAMES} on joint_states before timeout")
            return self._current_positions.copy()

        def call_service(self, client, request, name: str, timeout: float = 10.0) -> None:
            if not client.wait_for_service(timeout_sec=timeout):
                raise TimeoutError(f"{name} service is not available")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done():
                raise TimeoutError(f"{name} service call timed out")
            response = future.result()
            if response is None:
                raise RuntimeError(f"{name} service call failed")
            ret = getattr(response, "ret", 0)
            if ret != 0:
                message = getattr(response, "message", "")
                raise RuntimeError(f"{name} returned ret={ret}: {message}")

        def set_mode_state(self, mode: int) -> None:
            mode_req = SetInt16.Request()
            mode_req.data = mode
            self.call_service(self._set_mode, mode_req, "/xarm/set_mode")

            state_req = SetInt16.Request()
            state_req.data = 0
            self.call_service(self._set_state, state_req, "/xarm/set_state")

        def reach_first_position(self, q: np.ndarray, speed: float, acc: float, mvtime: float, timeout: float) -> None:
            self.call_service(self._clean_error, Call.Request(), "/xarm/clean_error")
            self.call_service(self._clean_warn, Call.Request(), "/xarm/clean_warn")

            enable_req = SetInt16ById.Request()
            enable_req.id = 8
            enable_req.data = 1
            self.call_service(self._motion_enable, enable_req, "/xarm/motion_enable")

            self.set_mode_state(0)

            move_req = MoveJoint.Request()
            move_req.angles = q.astype(np.float32).tolist()
            move_req.speed = float(speed)
            move_req.acc = float(acc)
            move_req.mvtime = float(mvtime)
            move_req.wait = True
            move_req.timeout = float(timeout)
            move_req.radius = -1.0
            move_req.relative = False
            self.get_logger().info("moving to first recorded joint pose with /xarm/set_servo_angle")
            self.call_service(self._set_servo_angle, move_req, "/xarm/set_servo_angle", timeout + 5.0)

            self.set_mode_state(1)
            time.sleep(0.5)

        def replay(self, positions: np.ndarray, rate: float) -> int:
            if not self._action_client.wait_for_server(timeout_sec=5.0):
                raise TimeoutError("xarm7 trajectory action server is not available")

            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = JOINT_NAMES

            for i, q in enumerate(positions):
                point = JointTrajectoryPoint()
                point.positions = q.tolist()
                point.time_from_start = _duration((i + 1) / rate)
                goal.trajectory.points.append(point)

            goal.goal_time_tolerance = _duration(1.0)

            self.get_logger().info(
                f"sending {len(goal.trajectory.points)} points to xarm7_traj_controller"
            )
            goal_future = self._action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, goal_future)
            goal_handle = goal_future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error("trajectory goal was rejected")
                return 1

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result().result
            if result.error_code != result.SUCCESSFUL:
                self.get_logger().error(
                    f"trajectory failed with error_code={result.error_code}: {result.error_string}"
                )
                return 1
            self.get_logger().info("trajectory replay finished successfully")
            return 0

    rclpy.init()
    node = XArm7SampleReplayer(args.action_name, args.joint_state_topic)
    try:
        current = node.wait_for_current_positions(timeout=5.0)
        start_error = float(np.max(np.abs(current - positions[0])))
        print(f"current-start max joint error: {start_error:.4f} rad")

        node.reach_first_position(
            positions[0],
            speed=args.first_position_speed,
            acc=args.first_position_acc,
            mvtime=args.first_position_duration,
            timeout=args.first_position_timeout,
        )

        return node.replay(positions, rate=args.rate)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    args = parse_args()
    sample = args.sample or _default_sample()
    positions = load_positions(sample, args.key)
    if args.rate <= 0:
        raise ValueError("--rate must be positive")
    if args.first_position_duration < 0:
        raise ValueError("--first-position-duration must be non-negative")
    if args.first_position_speed <= 0:
        raise ValueError("--first-position-speed must be positive")
    if args.first_position_acc <= 0:
        raise ValueError("--first-position-acc must be positive")
    if args.first_position_timeout <= 0:
        raise ValueError("--first-position-timeout must be positive")

    print(
        f"sample={sample} key={args.key} points={len(positions)} "
        f"duration={len(positions) / args.rate:.2f}s rate={args.rate:.1f}Hz"
    )
    print(f"first={np.array2string(positions[0], precision=4)}")
    print(f"last ={np.array2string(positions[-1], precision=4)}")

    if args.dry_run:
        return 0

    return run_replay(args, positions)


if __name__ == "__main__":
    sys.exit(main())
