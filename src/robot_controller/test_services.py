#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from jaka_msgs.msg import RobotMsg
from std_msgs.msg import Bool
import time

class RobotControllerTester(Node):
    def __init__(self):
        super().__init__('robot_controller_tester')
        
        # 创建服务客户端
        self.enable_client = self.create_client(Empty, '/robot_controller/enable')
        self.disable_client = self.create_client(Empty, '/robot_controller/disable')
        self.restart_client = self.create_client(Empty, '/robot_controller/restart')
        self.clear_error_client = self.create_client(Empty, '/robot_controller/clear_error')
        self.reset_sensor_client = self.create_client(Empty, '/robot_controller/reset_sensor')
        
        # 创建订阅者
        self.robot_status_sub = self.create_subscription(
            RobotMsg, '/robot_controller/status', self.robot_status_callback, 10)
        self.connection_status_sub = self.create_subscription(
            Bool, '/robot_controller/connection_status', self.connection_status_callback, 10)
        
        self.get_logger().info('Robot Controller Tester initialized')
    
    def robot_status_callback(self, msg):
        self.get_logger().info(f'Robot Status: motion={msg.motion_state}, power={msg.power_state}, servo={msg.servo_state}, collision={msg.collision_state}')
    
    def connection_status_callback(self, msg):
        status = "Connected" if msg.data else "Disconnected"
        self.get_logger().info(f'Connection Status: {status}')
    
    def call_service(self, client, service_name):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {service_name} not available')
            return False
        
        request = Empty.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'{service_name} service called successfully')
            return True
        else:
            self.get_logger().error(f'{service_name} service call failed')
            return False
    
    def test_services(self):
        self.get_logger().info('Testing robot controller services...')
        
        # 测试启用服务
        self.get_logger().info('Testing enable service...')
        self.call_service(self.enable_client, 'Enable')
        time.sleep(2)
        
        # 测试清错服务
        self.get_logger().info('Testing clear error service...')
        self.call_service(self.clear_error_client, 'Clear Error')
        time.sleep(2)
        
        # 测试重置传感器服务
        self.get_logger().info('Testing reset sensor service...')
        self.call_service(self.reset_sensor_client, 'Reset Sensor')
        time.sleep(2)
        
        # 测试禁用服务
        self.get_logger().info('Testing disable service...')
        self.call_service(self.disable_client, 'Disable')
        time.sleep(2)
        
        # 测试重启服务
        self.get_logger().info('Testing restart service...')
        self.call_service(self.restart_client, 'Restart')
        time.sleep(5)
        
        self.get_logger().info('Service testing completed')

def main(args=None):
    rclpy.init(args=args)
    
    tester = RobotControllerTester()
    
    try:
        # 等待一段时间让状态发布器启动
        time.sleep(3)
        tester.test_services()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 