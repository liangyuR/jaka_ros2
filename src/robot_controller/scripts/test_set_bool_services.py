#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time

class SetBoolServiceTester(Node):
    def __init__(self):
        super().__init__('set_bool_service_tester')
        
        # 创建服务客户端
        self.power_client = self.create_client(SetBool, '/robot_controller/power')
        self.enable_client = self.create_client(SetBool, '/robot_controller/enable')
        
        self.get_logger().info('SetBool Service Tester initialized')
    
    def test_power_service(self, power_on=True):
        """测试上下电服务"""
        request = SetBool.Request()
        request.data = power_on
        
        self.get_logger().info(f'Calling power service with data={power_on}')
        
        future = self.power_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            response = future.result()
            self.get_logger().info(f'Power service response: success={response.success}, message={response.message}')
            return response.success
        else:
            self.get_logger().error('Power service call failed')
            return False
    
    def test_enable_service(self, enable=True):
        """测试上下使能服务"""
        request = SetBool.Request()
        request.data = enable
        
        self.get_logger().info(f'Calling enable service with data={enable}')
        
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            response = future.result()
            self.get_logger().info(f'Enable service response: success={response.success}, message={response.message}')
            return response.success
        else:
            self.get_logger().error('Enable service call failed')
            return False
    
    def run_tests(self):
        """运行所有测试"""
        self.get_logger().info('Starting SetBool service tests...')
        
        # 等待服务可用
        while not self.power_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for power service...')
        
        while not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for enable service...')
        
        # 测试上下电
        self.get_logger().info('=== Testing Power Service ===')
        self.test_power_service(True)   # 上电
        time.sleep(2)
        self.test_power_service(False)  # 下电
        
        # 测试上下使能
        self.get_logger().info('=== Testing Enable Service ===')
        self.test_enable_service(True)  # 使能
        time.sleep(2)
        self.test_enable_service(False) # 下使能
        
        self.get_logger().info('All tests completed')

def main(args=None):
    rclpy.init(args=args)
    
    tester = SetBoolServiceTester()
    tester.run_tests()
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 