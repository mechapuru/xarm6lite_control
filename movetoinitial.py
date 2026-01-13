import rclpy
from rclpy.node import Node

from xarm_msgs.srv import SetInt16, MoveCartesian

class XArmInitAndMove(Node):
    def __init__(self):
        super().__init__('xarm_init_and_move')

        self.mode_cli = self.create_client(SetInt16, '/xarm/set_mode')
        self.state_cli = self.create_client(SetInt16, '/xarm/set_state')
        self.move_cli = self.create_client(MoveCartesian, '/xarm/set_position')

        self.get_logger().info('Waiting for services...')
        self.mode_cli.wait_for_service()
        self.state_cli.wait_for_service()
        self.move_cli.wait_for_service()

    def set_mode(self, value):
        req = SetInt16.Request()
        req.data = value
        future = self.mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def set_state(self, value):
        req = SetInt16.Request()
        req.data = value
        future = self.state_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def move(self):
        req = MoveCartesian.Request()
        req.pose = [125.0, 10.0, 177.0, 3.14, 0.0, 0.0]
        req.speed = 50.0
        req.acc = 2.0
        req.mvtime = 0.0
        req.wait = True
        req.motion_type = 0

        future = self.move_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    node = XArmInitAndMove()

    node.get_logger().info('Setting mode...')
    res = node.set_mode(0)
    if res.ret != 0:
        node.get_logger().error(res.message)
        return

    node.get_logger().info('Setting state...')
    res = node.set_state(0)
    if res.ret != 0:
        node.get_logger().error(res.message)
        return

    node.get_logger().info('Moving robot...')
    res = node.move()
    if res.ret != 0:
        node.get_logger().error(res.message)
    else:
        node.get_logger().info('Motion complete')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
