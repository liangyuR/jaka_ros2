#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time

class AlsonClientTestNode(Node):
    def __init__(self):
        super().__init__('alson_client_test_node')
        self.change_project_client = self.create_client(Trigger, 'change_project')
        self.run_project_client = self.create_client(Trigger, 'run_project')
        self.timer = self.create_timer(1.0, self.run_test)
        self.test_done = False

    def run_test(self):
        if self.test_done:
            return
        # 切换 project
        if not self.change_project_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('change_project service not available')
            self.test_done = True
            return
        req = Trigger.Request()
        future = self.change_project_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Change project success')
        else:
            self.get_logger().error('Change project failed')
            self.test_done = True
            return
        # 运行 project
        if not self.run_project_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('run_project service not available')
            self.test_done = True
            return
        req2 = Trigger.Request()
        future2 = self.run_project_client.call_async(req2)
        rclpy.spin_until_future_complete(self, future2)
        if future2.result() and future2.result().success:
            self.get_logger().info('Run project success')
        else:
            self.get_logger().error('Run project failed')
        self.test_done = True
        self.get_logger().info('Test finished.')


def main(args=None):
    rclpy.init(args=args)
    node = AlsonClientTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 