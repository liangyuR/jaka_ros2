#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time


class ManipulationTestNode(Node):
    def __init__(self):
        super().__init__('manipulation_test_node')
        
        # 创建服务客户端
        self.add_table_client = self.create_client(SetBool, 'add_table')
        self.add_box_client = self.create_client(SetBool, 'add_box')
        self.clear_scene_client = self.create_client(SetBool, 'clear_scene')
        self.attach_tool_client = self.create_client(SetBool, 'attach_tool')
        self.detach_tool_client = self.create_client(SetBool, 'detach_tool')
        
        # 等待服务可用
        self.get_logger().info('Waiting for services...')
        self.add_table_client.wait_for_service(timeout_sec=10.0)
        self.add_box_client.wait_for_service(timeout_sec=10.0)
        self.clear_scene_client.wait_for_service(timeout_sec=10.0)
        self.attach_tool_client.wait_for_service(timeout_sec=10.0)
        self.detach_tool_client.wait_for_service(timeout_sec=10.0)
        
        self.get_logger().info('All services are available!')
        
    def add_table(self, add=True):
        """添加或移除桌子"""
        request = SetBool.Request()
        request.data = add
        
        future = self.add_table_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result()
        if result is not None:
            self.get_logger().info(f'Table operation: {result.message}')
            return result.success
        else:
            self.get_logger().error('Failed to call add_table service')
            return False
    
    def add_box(self, add=True):
        """添加或移除盒子"""
        request = SetBool.Request()
        request.data = add
        
        future = self.add_box_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result()
        if result is not None:
            self.get_logger().info(f'Box operation: {result.message}')
            return result.success
        else:
            self.get_logger().error('Failed to call add_box service')
            return False
    
    def clear_scene(self):
        """清空场景"""
        request = SetBool.Request()
        request.data = True
        
        future = self.clear_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result()
        if result is not None:
            self.get_logger().info(f'Clear scene: {result.message}')
            return result.success
        else:
            self.get_logger().error('Failed to call clear_scene service')
            return False
    
    def attach_tool(self):
        """附加工具"""
        request = SetBool.Request()
        request.data = True
        
        future = self.attach_tool_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result()
        if result is not None:
            self.get_logger().info(f'Attach tool: {result.message}')
            return result.success
        else:
            self.get_logger().error('Failed to call attach_tool service')
            return False
    
    def detach_tool(self):
        """分离工具"""
        request = SetBool.Request()
        request.data = True
        
        future = self.detach_tool_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result()
        if result is not None:
            self.get_logger().info(f'Detach tool: {result.message}')
            return result.success
        else:
            self.get_logger().error('Failed to call detach_tool service')
            return False
    
    def run_demo(self):
        """运行演示"""
        self.get_logger().info('Starting manipulation demo...')
        
        # 1. 清空场景
        self.get_logger().info('1. Clearing scene...')
        self.clear_scene()
        time.sleep(2)
        
        # 2. 添加桌子
        self.get_logger().info('2. Adding table...')
        self.add_table(True)
        time.sleep(2)
        
        # 3. 添加盒子
        self.get_logger().info('3. Adding box...')
        self.add_box(True)
        time.sleep(2)
        
        # 4. 移除盒子
        self.get_logger().info('4. Removing box...')
        self.add_box(False)
        time.sleep(2)
        
        # 5. 重新添加盒子
        self.get_logger().info('5. Adding box again...')
        self.add_box(True)
        time.sleep(2)
        
        # 6. 附加工具
        self.get_logger().info('6. Attaching tool...')
        self.attach_tool()
        time.sleep(2)
        
        # 7. 分离工具
        self.get_logger().info('7. Detaching tool...')
        self.detach_tool()
        time.sleep(2)
        
        # 8. 重新附加工具
        self.get_logger().info('8. Attaching tool again...')
        self.attach_tool()
        time.sleep(2)
        
        self.get_logger().info('Demo completed!')


def main(args=None):
    rclpy.init(args=args)
    
    node = ManipulationTestNode()
    
    try:
        node.run_demo()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 