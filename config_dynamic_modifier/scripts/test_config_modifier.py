#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import time

class ConfigModifierTester(Node):
    def __init__(self):
        super().__init__('config_modifier_tester')
        
        # 创建发布者
        self.point_pub = self.create_publisher(PointStamped, '/clicked_point', 10)
        self.command_pub = self.create_publisher(String, '/config_modifier/command', 10)
        
        # 创建订阅者
        self.status_sub = self.create_subscription(String, '/config_modifier/status', self.status_callback, 10)
        
        self.get_logger().info('配置修改器测试节点已启动')
        
    def status_callback(self, msg):
        self.get_logger().info(f'状态: {msg.data}')
        
    def test_point_publishing(self):
        """测试点发布功能"""
        self.get_logger().info('测试点发布功能...')
        
        # 创建测试点
        point_msg = PointStamped()
        point_msg.header.frame_id = 'map'
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point.x = 2.0
        point_msg.point.y = 3.0
        point_msg.point.z = 0.0
        
        # 发布点消息
        self.point_pub.publish(point_msg)
        self.get_logger().info(f'已发布点: ({point_msg.point.x}, {point_msg.point.y})')
        
    def test_command_publishing(self):
        """测试命令发布功能"""
        self.get_logger().info('测试命令发布功能...')
        
        # 测试选择区域
        area_cmd = String()
        area_cmd.data = 'area 1'
        self.command_pub.publish(area_cmd)
        self.get_logger().info('已发送区域选择命令')
        
        time.sleep(1)
        
        # 测试开始多边形收集
        start_polygon_cmd = String()
        start_polygon_cmd.data = 'start_polygon'
        self.command_pub.publish(start_polygon_cmd)
        self.get_logger().info('已发送开始多边形收集命令')
        
        time.sleep(1)
        
        # 测试完成多边形
        finish_polygon_cmd = String()
        finish_polygon_cmd.data = 'finish_polygon'
        self.command_pub.publish(finish_polygon_cmd)
        self.get_logger().info('已发送完成多边形命令')
        
        time.sleep(1)
        
        # 测试重置
        reset_cmd = String()
        reset_cmd.data = 'reset'
        self.command_pub.publish(reset_cmd)
        self.get_logger().info('已发送重置命令')

def main():
    rclpy.init()
    
    tester = ConfigModifierTester()
    
    try:
        # 等待一段时间让节点启动
        time.sleep(2)
        
        # 运行测试
        tester.test_command_publishing()
        time.sleep(2)
        tester.test_point_publishing()
        
        # 保持节点运行一段时间
        time.sleep(5)
        
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 