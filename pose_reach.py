import rclpy
from rclpy.node import Node
from xarm_msgs.srv import SetInt16, MoveJoint

class XArmJointController(Node):
    def __init__(self):
        super().__init__('xarm_joint_controller')

        # Create clients
        self.mode_cli = self.create_client(SetInt16, '/xarm/set_mode')
        self.state_cli = self.create_client(SetInt16, '/xarm/set_state')
        self.move_joint_cli = self.create_client(MoveJoint, '/xarm/set_servo_angle')

        self.get_logger().info('Waiting for services...')
        self.mode_cli.wait_for_service()
        self.state_cli.wait_for_service()
        self.move_joint_cli.wait_for_service()

    def call_set_int16(self, client, value):
        req = SetInt16.Request(data=value)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_joint_goal(self):
        # STEP 1: Set Mode 0 (Location/Position Control)
        self.get_logger().info('Setting Mode 0...')
        self.call_set_int16(self.mode_cli, 0)

        # STEP 2: Set State 0 (Ready/Enable)
        self.get_logger().info('Setting State 0...')
        self.call_set_int16(self.state_cli, 0)

        # STEP 3: Send the Movement
        req = MoveJoint.Request()
        req.angles = [
            0.06879374384880066,
           -0.6134774088859558,
            0.7866519689559937,
            0.0004016763996332884,
            1.399990200996399,
            0.06126338988542557
        ]
        req.speed = 0.35
        req.acc = 10.0
        req.mvtime = 0.0
        req.wait = True

        self.get_logger().info('Sending joint command...')
        future = self.move_joint_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = XArmJointController()
    
    result = node.send_joint_goal()
    
    if result:
        # If result.ret is still 9, check if the emergency stop is pressed 
        # or if the robot has a hardware error.
        node.get_logger().info(f'Final Response Code: {result.ret}')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()