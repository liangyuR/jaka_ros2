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
        self.waiting_future = None
        self.waiting_topic = False
        self.waiting_start_time = None
        self.async_result_received = False
        self.async_result_msg = None
        self.block_result = None
        self.block_future = None
        self.block_start_time = None
        self.emptyid_result = None
        self.emptyid_future = None
        self.emptyid_start_time = None

    def alson_event_callback(self, msg):
        self.get_logger().info(f'[Topic] {msg.data}')
        # 只要收到 ResponseCode:310 就认为是异步最终结果
        if msg.data.startswith('ResponseCode:310'):
            self.async_result_received = True
            self.async_result_msg = msg.data

    def run_test(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.test_done:
            return

        # 步骤0: 等待服务可用
        if self.test_step == 0:
            if self.run_project_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().info('服务已就绪，开始测试')
                self.test_step = 1
            return

        # 步骤1: 发起异步模式请求
        if self.test_step == 1:
            self.get_logger().info('=== 测试1: 异步模式 ===')
            req = RunProject.Request()
            req.project_id = "11"
            req.fl_tcp_position = [0.5, 0.0, 0.3, 0.0, 0.0, 0.0]
            req.wait_for_completion = False
            self.async_result_received = False
            self.async_result_msg = None
            self.waiting_future = self.run_project_client.call_async(req)
            self.waiting_start_time = now
            self.test_step = 2
            return

        # 步骤2: 等待异步future完成
        if self.test_step == 2:
            if self.waiting_future.done():
                result = self.waiting_future.result()
                if result.success:
                    self.get_logger().info(f'异步模式服务端已接收: {result.message}')
                    self.waiting_start_time = now
                    self.test_step = 3
                else:
                    self.get_logger().error(f'异步模式服务端返回失败: {result.message}')
                    self.test_results.append('异步模式失败')
                    self.test_step = 10
            elif now - self.waiting_start_time > 10:
                self.get_logger().error('异步模式调用超时')
                self.test_results.append('异步模式失败')
                self.test_step = 10
            return

        # 步骤3: 等待topic异步结果
        if self.test_step == 3:
            if self.async_result_received:
                self.get_logger().info(f'异步模式最终结果: {self.async_result_msg}')
                self.test_results.append('异步模式通过')
                self.waiting_start_time = now
                self.test_step = 4
            elif now - self.waiting_start_time > 20:
                self.get_logger().error('异步模式未收到最终结果')
                self.test_results.append('异步模式失败')
                self.test_step = 4
            return

        # 步骤4: 等待2秒再进行阻塞模式
        if self.test_step == 4:
            if now - self.waiting_start_time > 2:
                self.test_step = 5
            return

        # 步骤5: 发起阻塞模式请求
        if self.test_step == 5:
            self.get_logger().info('=== 测试2: 阻塞模式 ===')
            req2 = RunProject.Request()
            req2.project_id = "11"
            req2.fl_tcp_position = [0.6, 0.1, 0.4, 0.1, 0.1, 0.1]
            req2.wait_for_completion = True
            self.block_future = self.run_project_client.call_async(req2)
            self.block_start_time = now
            self.test_step = 6
            return

        # 步骤6: 等待阻塞future完成
        if self.test_step == 6:
            if self.block_future.done():
                result2 = self.block_future.result()
                if result2.success:
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
                self.waiting_start_time = now
                self.test_step = 7
            elif now - self.block_start_time > 60:
                self.get_logger().error('阻塞模式调用超时')
                self.test_results.append('阻塞模式失败')
                self.waiting_start_time = now
                self.test_step = 7
            return

        # 步骤7: 等待1秒再进行空项目ID测试
        if self.test_step == 7:
            if now - self.waiting_start_time > 1:
                self.test_step = 8
            return

        # 步骤8: 发起空项目ID测试
        if self.test_step == 8:
            self.get_logger().info('=== 测试3: 空项目ID测试 ===')
            req3 = RunProject.Request()
            req3.project_id = ""
            req3.fl_tcp_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            req3.wait_for_completion = False
            self.emptyid_future = self.run_project_client.call_async(req3)
            self.emptyid_start_time = now
            self.test_step = 9
            return

        # 步骤9: 等待空项目IDfuture完成
        if self.test_step == 9:
            if self.emptyid_future.done():
                result3 = self.emptyid_future.result()
                if not result3.success:
                    self.get_logger().info(f'空项目ID测试正确返回失败: {result3.message}')
                    self.test_results.append('空项目ID通过')
                else:
                    self.get_logger().warn('空项目ID测试意外成功')
                    self.test_results.append('空项目ID失败')
                self.test_step = 10
            elif now - self.emptyid_start_time > 10:
                self.get_logger().error('空项目ID测试调用超时')
                self.test_results.append('空项目ID失败')
                self.test_step = 10
            return

        # 步骤10: 测试结束
        if self.test_step == 10:
            self.get_logger().info('=== 所有测试完成 ===')
            self.get_logger().info(f'测试结果: {self.test_results}')
            self.test_done = True
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = AlsonClientTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 