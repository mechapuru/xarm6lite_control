#!/usr/bin/env python3
"""
Synchronous Policy Inference Node

REQUIRED SETUP BEFORE RUNNING:
==============================

Terminal 1 - Robot + Gripper (inference mode):
    cd ~/rrc/xarm_ws
    source install/setup.bash
    ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py \
        robot_ip:=192.168.1.175 \
        gripper_port:=/dev/ttyUSB0 \
        inference_mode:=true

Terminal 2 - Camera:
    source install/setup.bash
    ros2 launch realsense2_camera rs_launch.py \
        camera_name:=camera \
        camera_namespace:=camera \
        rgb_camera.color_profile:="640,360,30" \
        depth_module.depth_profile:="640,360,30"

Terminal 3 - Run this script:
    source install/setup.bash
    python3 src/xarm_ros2/synchronous_policy_inference.py
"""

'''
Parameters to Experiment With:
    1. action_execution_horizon
    2. slop in message_filter
'''

import rclpy
from rclpy.node import Node
import cv2
import json
import base64
import numpy as np
import websocket
import threading
import time
import message_filters
from collections import deque

from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int32, Float64
from cv_bridge import CvBridge

class SynchronousPolicyInferenceNode(Node):
    def __init__(self):
        super().__init__('synchronous_policy_inference_node')
        
        # Parameters
        self.declare_parameter('server_url', 'ws://10.4.25.44:8000/ws')
        self.server_url = self.get_parameter('server_url').value

        self.declare_parameter('history_size', 1)
        self.history_size = self.get_parameter('history_size').value
        
        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        self.rgb_topic = self.get_parameter('rgb_topic').value

        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.depth_topic = self.get_parameter('depth_topic').value

        self.declare_parameter('joint_state_topic', '/ufactory/joint_states')
        self.joint_state_topic = self.get_parameter('joint_state_topic').value

        self.declare_parameter('action_execution_horizon', 5)
        self.action_execution_horizon = self.get_parameter('action_execution_horizon').value
        
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # Data storage
        self.observation_history = deque(maxlen=self.history_size)
        self.latest_realtime_joints = None
        self.current_gripper_state = 0.0
        self.gripper_action_history = deque(maxlen=5)
        
        # Publisher for arm commands
        self.arm_action_pub = self.create_publisher(JointTrajectory, '/lite6_traj_controller/joint_trajectory', 1)
        
        # Gripper Publisher & Subscriber
        self.gripper_command_pub = self.create_publisher(Int32, '/gripper/command', 1)
        self.gripper_state_sub = self.create_subscription(Float64, '/gripper/state', self.gripper_state_callback, 10)

        # 1. Direct Subscriber for Real-time Joint States (for execution)
        self.joint_state_sub = self.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, 10)

        # 2. Synchronized Subscriber for Inference Inputs (RGB + Depth + Joints)
        # Note: message_filters.Subscriber in ROS 2 takes 'mode' as first arg, or 'node' ? 
        # Standard usage: message_filters.Subscriber(node, type, topic)
        rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        joint_sub = message_filters.Subscriber(self, JointState, self.joint_state_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, joint_sub], 
            queue_size=10, 
            slop=0.01
        )
        self.ts.registerCallback(self.synchronized_callback)
        
        self.ws = None
        self.connect_to_server()
        
        self.get_logger().info(f"Synchronous Policy Inference Node started. URL: {self.server_url}")

    def connect_to_server(self):
        """Establishes a blocking websocket connection."""
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to {self.server_url}...")
                self.ws = websocket.create_connection(self.server_url)
                self.get_logger().info("Connected to server.")
                break
            except Exception as e:
                self.get_logger().error(f"Connection failed: {e}. Retrying in 2s...")
                time.sleep(2)

    def joint_state_callback(self, msg):
        """Callback for real-time joint states."""
        if len(msg.position) >= 6:
            with self.lock:
                self.latest_realtime_joints = list(msg.position[:6])

    def gripper_state_callback(self, msg):
        """Callback for gripper state."""
        with self.lock:
            self.current_gripper_state = msg.data

    def synchronized_callback(self, rgb_msg, depth_msg, joint_msg):
        """Callback for synchronized sensor data."""
        self.get_logger().info("Received synchronized frames!", throttle_duration_sec=2)
        try:
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            # Handle depth encoding
            if depth_msg.encoding == "16UC1":
                depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            else:
                depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            
            # Resize images to expected input size
            rgb_resized = cv2.resize(rgb_img, (640, 360), interpolation=cv2.INTER_AREA)
            depth_resized = cv2.resize(depth_img, (640, 360), interpolation=cv2.INTER_NEAREST)
             
            joints = list(joint_msg.position[:6])
            
            with self.lock:
                obs = {
                    'rgb': rgb_resized,
                    'depth': depth_resized,
                    'joints': joints,
                    'gripper': self.current_gripper_state
                }
                self.observation_history.append(obs)
                
        except Exception as e:
            # Throttle not directly available in rclpy logger easily effectively, so just log warn
            self.get_logger().warn(f"Error in sync callback: {e}")

    def get_inference_and_send(self):
        """
        1. Waits for history to be populated.
        2. Constructs the payload.
        3. Sends to server and waits for response (blocking).
        4. Returns the list of actions.
        """
        # Wait until we have enough history
        while rclpy.ok():
            with self.lock:
                if len(self.observation_history) >= self.history_size:
                    history_list = list(self.observation_history)
                    break
            self.get_logger().info(f"Waiting for observation history ({len(self.observation_history)}/{self.history_size})...", throttle_duration_sec=2)
            # Use a slightly longer sleep to avoid spamming if uncommented
            time.sleep(0.01)
        
        if not rclpy.ok(): return []

        # Prepare Payload
        rgb_list = []
        depth_list = []
        agent_pos_list = []

        for obs in history_list:
            _, rgb_buffer = cv2.imencode('.jpg', obs['rgb'])
            rgb_base64 = base64.b64encode(rgb_buffer).decode('utf-8')
            rgb_list.append(rgb_base64)
            
            _, depth_buffer = cv2.imencode('.png', obs['depth'])
            depth_base64 = base64.b64encode(depth_buffer).decode('utf-8')
            depth_list.append(depth_base64)

            agent_pos = obs['joints'] + [obs['gripper']]
            agent_pos_list.append(agent_pos)

        payload = {
            'observation': {
                'rgb': rgb_list,
                'depth': depth_list,
                'agent_pos': agent_pos_list
            }
        }
        # print(payload)

        # Send and Receive
        try:
            self.get_logger().info("Sending payload to server...")
            self.ws.send(json.dumps(payload))
            result = self.ws.recv() # Blocking receive
            self.get_logger().info(f"Received response of length {len(result)}")

            data = json.loads(result)
            if 'action' in data:
                actions = data['action']
                self.get_logger().info(f"Received {len(actions)} actions.")
                if len(actions) > 0:
                     self.get_logger().info(f"First action sample: {actions[0]}")
                return actions
            else:
                self.get_logger().warn("Received data without 'action' field.")
                return []
        except (websocket.WebSocketException, BrokenPipeError, ConnectionResetError) as e:
            self.get_logger().error(f"Websocket communication error: {e}")
            try:
                self.ws.close()
            except: pass
            self.connect_to_server() # Reconnect
            return []
        except Exception as e:
            self.get_logger().error(f"Error getting inference: {e}")
            return []

    def execute_actions(self, actions):
        """
        Executes the list of actions sequentially.
        """
        if not actions:
            return

        self.get_logger().info(f"Executing {len(actions)} actions...")
        for i, action_delta in enumerate(actions):
            if not rclpy.ok(): break
            
            # Use absolute positions directly
            target_positions = action_delta[:6]

            # Gripper Control Logic
            if len(action_delta) > 6:
                
                pred_gripper_val = action_delta[6]
                cmd_msg = Int32()
                cmd_msg.data = 0

                print("the predicted gripper values are",pred_gripper_val)
                
                
                if pred_gripper_val >= 0.1 and self.current_gripper_state < 3970:
                    cmd_msg.data = 30 # Close velocity
                elif pred_gripper_val <= -0.1 and self.current_gripper_state > 3770:
                    cmd_msg.data = -30 # Open velocity
                else:
                    cmd_msg.data = 0
                
                self.gripper_command_pub.publish(cmd_msg)
                
                #Publishing The values we are getting
                

                
                print("got gripper message", cmd_msg.data)
                
                # Default stop
                # self.gripper_action_history.append(pred_gripper_val)

                # c
                # cmd_msg.data = 0 # Default stop

                # if len(self.gripper_action_history) == 5:
                #     recent_values = list(self.gripper_action_history)
                #     if all(v > 0.6 for v in recent_values):
                #         cmd_msg.data = 30 # Open velocity
                #     elif all(v < -0.6 for v in recent_values):
                #         cmd_msg.data = -30 # Close velocity
                    
                #     # If command is non-zero, publish it
                #     if cmd_msg.data != 0:
                #         self.gripper_command_pub.publish(cmd_msg)

            # Create and publish trajectory

            traj_msg = JointTrajectory()
            traj_msg.header.stamp = self.get_clock().now().to_msg()
            traj_msg.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
            
            point = JointTrajectoryPoint()
            point.positions = target_positions
            point.time_from_start = rclpy.duration.Duration(seconds=0.1).to_msg() # Duration for this step
            
            traj_msg.points.append(point)
            self.arm_action_pub.publish(traj_msg)

            # Wait for the action to complete (blocking with timeout)
            start_time = time.time()
            timeout = 3  # Timeout in seconds
            
            while rclpy.ok():
                # Check for timeout
                if time.time() - start_time > timeout:
                    # self.get_logger().warn("Action execution timed out.")
                    break
                
                # Check error if we have fresh joint states
                with self.lock:
                    current_joints = self.latest_realtime_joints
                
                if current_joints:
                    error = max([abs(c - t) for c, t in zip(current_joints, target_positions)])
                    if error < 0.02: # 0.02 radians ~ 1.15 degrees tolerance
                        break
                
                time.sleep(0.01) 

    def inference_loop(self):
        # Wait for controller connection
        # Check if anyone is subscribed to our publisher (i.e. the controller)
        while self.arm_action_pub.get_subscription_count() == 0 and rclpy.ok():
             self.get_logger().info("Waiting for controller subscriber...", throttle_duration_sec=2)
             time.sleep(0.1)
        self.get_logger().info("Controller connected.")

        while rclpy.ok():
            # 1. Sync & Send
            actions = self.get_inference_and_send()
            actions = actions[:self.action_execution_horizon]
            # 2. Execute
            self.execute_actions(actions)

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    inference_thread = None
    
    try:
        node = SynchronousPolicyInferenceNode()
        
        # Run inference loop in a separate thread causing blocking IO
        inference_thread = threading.Thread(target=node.inference_loop)
        inference_thread.start()
        
        # Spin the node in the main thread to handle callbacks
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        if inference_thread:
            inference_thread.join()

if __name__ == '__main__':
    main()
