#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from jaka_msgs.srv import RunProject
from std_msgs.msg import String

class AlsonClientTestNode(Node):
    def __init__(self):
        super().__init__('alson_client_test_node')
        self.run_project_client = self.create_client(RunProject, '/alson_client_node/run_project')
        self.alson_event_sub = self.create_subscription(String, 'alson_events', self.alson_event_callback, 10)
        self.timer = self.create_timer(0.2, self.run_test)
        self.test_done = False
        self.test_results = []
        self.test_step = 0
        self.block_future = None
        self.block_start_time = None
        self.emptyid_future = None
        self.emptyid_start_time = None
        self.keep_listening = False

    def alson_event_callback(self, msg):
        self.get_logger().info(f'[Topic] {msg.data}')

    def run_test(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.test_done:
            if not self.keep_listening:
                self.get_logger().info('=== 所有测试完成，继续监听 /alson_events，按 Ctrl+C 退出 ===')
                self.keep_listening = True
            return

        # 步骤0: 等待服务可用
        if self.test_step == 0:
            if self.run_project_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().info('服务已就绪，开始测试')
                self.test_step = 1
            return

        # 步骤1: 发起阻塞模式请求
        if self.test_step == 1:
            self.get_logger().info('=== 测试1: 阻塞模式 ===')
            req2 = RunProject.Request()
            req2.project_id = "11"
            req2.fl_tcp_position = [0.6, 0.1, 0.4, 0.1, 0.1, 0.1]
            self.block_future = self.run_project_client.call_async(req2)
            self.block_start_time = now
            self.test_step = 2
            return

        # 步骤2: 等待阻塞future完成
        if self.test_step == 2:
            if self.block_future.done():
                result2 = self.block_future.result()
                if result2.status_code == 0:
                    self.get_logger().info(f'阻塞模式成功: {result2.message}')
                    self.get_logger().info(f'状态码: {result2.status_code}')
                    if result2.pose:
                        self.get_logger().info(f'目标位姿: {result2.pose}')
                    if hasattr(result2, 'joint_angles') and result2.joint_angles:
                        self.get_logger().info(f'关节角度: {result2.joint_angles}')
                    self.test_results.append('阻塞模式通过')
                else:
                    self.get_logger().error(f'阻塞模式失败: {result2.message}')
                    self.test_results.append('阻塞模式失败')
                self.block_start_time = now
                self.test_step = 3
            elif now - self.block_start_time > 60:
                self.get_logger().error('阻塞模式调用超时')
                self.test_results.append('阻塞模式失败')
                self.block_start_time = now
                self.test_step = 3
            return

        # 步骤3: 等待1秒再进行空项目ID测试
        if self.test_step == 3:
            if now - self.block_start_time > 1:
                self.test_step = 4
            return

        # 步骤4: 发起空项目ID测试
        if self.test_step == 4:
            self.get_logger().info('=== 测试2: 空项目ID测试 ===')
            req3 = RunProject.Request()
            req3.project_id = ""
            req3.fl_tcp_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.emptyid_future = self.run_project_client.call_async(req3)
            self.emptyid_start_time = now
            self.test_step = 5
            return

        # 步骤5: 等待空项目IDfuture完成
        if self.test_step == 5:
            if self.emptyid_future.done():
                result3 = self.emptyid_future.result()
                if result3.status_code == -1:
                    self.get_logger().info(f'空项目ID测试正确返回失败: {result3.message}')
                    self.test_results.append('空项目ID通过')
                else:
                    self.get_logger().warn('空项目ID测试意外成功')
                    self.test_results.append('空项目ID失败')
                self.test_step = 6
            elif now - self.emptyid_start_time > 10:
                self.get_logger().error('空项目ID测试调用超时')
                self.test_results.append('空项目ID失败')
                self.test_step = 6
            return

        # 步骤6: 测试结束
        if self.test_step == 6:
            self.get_logger().info('=== 所有测试完成 ===')
            self.get_logger().info(f'测试结果: {self.test_results}')
            self.test_done = True
            # 不再销毁节点，继续监听 topic


def main(args=None):
    rclpy.init(args=args)
    node = AlsonClientTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 