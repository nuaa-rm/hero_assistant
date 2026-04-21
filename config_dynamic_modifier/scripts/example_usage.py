#!/usr/bin/env python3

"""
配置动态修改器使用示例

这个脚本演示了如何使用配置动态修改器来修改costmap参数。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import time

class ConfigModifierExample(Node):
    def __init__(self):
        super().__init__('config_modifier_example')
        
        # 创建发布者
        self.point_pub = self.create_publisher(PointStamped, '/clicked_point', 10)
        self.command_pub = self.create_publisher(String, '/config_modifier/command', 10)
        
        # 创建订阅者
        self.status_sub = self.create_subscription(String, '/config_modifier/status', self.status_callback, 10)
        
        self.get_logger().info('配置修改器示例节点已启动')
        
    def status_callback(self, msg):
        self.get_logger().info(f'状态: {msg.data}')
        
    def select_area(self, area_index):
        """选择要修改的区域"""
        cmd = String()
        cmd.data = f'area {area_index}'
        self.command_pub.publish(cmd)
        self.get_logger().info(f'已选择区域 {area_index}')
        
    def start_polygon_collection(self):
        """开始收集多边形点"""
        cmd = String()
        cmd.data = 'start_polygon'
        self.command_pub.publish(cmd)
        self.get_logger().info('已开始收集多边形点')
        
    def finish_polygon(self):
        """完成多边形并应用"""
        cmd = String()
        cmd.data = 'finish_polygon'
        self.command_pub.publish(cmd)
        self.get_logger().info('已完成多边形定义')
        
    def clear_polygon(self):
        """清除当前多边形点"""
        cmd = String()
        cmd.data = 'clear_polygon'
        self.command_pub.publish(cmd)
        self.get_logger().info('已清除当前多边形点')
        
    def click_point(self, x, y, z=0.0):
        """模拟点击地图上的点"""
        point_msg = PointStamped()
        point_msg.header.frame_id = 'map'
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = z
        
        self.point_pub.publish(point_msg)
        self.get_logger().info(f'已点击点: ({x}, {y}, {z})')
        
    def reset_all_areas(self):
        """重置所有区域"""
        cmd = String()
        cmd.data = 'reset'
        self.command_pub.publish(cmd)
        self.get_logger().info('已重置所有区域')
        
    def reload_costmap(self):
        """重新加载costmap"""
        cmd = String()
        cmd.data = 'reload'
        self.command_pub.publish(cmd)
        self.get_logger().info('已请求重新加载costmap')
        
    def enable_auto_reload(self, enable=True):
        """启用或禁用自动重新加载（仅高级版本）"""
        cmd = String()
        cmd.data = f'auto_reload {"on" if enable else "off"}'
        self.command_pub.publish(cmd)
        self.get_logger().info(f'已{"启用" if enable else "禁用"}自动重新加载')
        
    def run_example(self):
        """运行示例"""
        self.get_logger().info('开始运行配置修改器示例...')
        
        # 等待节点启动
        time.sleep(2)
        
        # 示例1: 选择区域并开始多边形收集
        self.get_logger().info('=== 示例1: 选择区域并开始多边形收集 ===')
        self.select_area(0)
        time.sleep(1)
        self.start_polygon_collection()
        time.sleep(1)
        
        # 示例2: 点击多个点定义多边形
        self.get_logger().info('=== 示例2: 点击多个点定义多边形 ===')
        self.click_point(2.0, 3.0)
        time.sleep(0.5)
        self.click_point(3.0, 3.0)
        time.sleep(0.5)
        self.click_point(3.0, 2.0)
        time.sleep(0.5)
        self.click_point(2.0, 2.0)
        time.sleep(1)
        
        # 示例3: 完成多边形
        self.get_logger().info('=== 示例3: 完成多边形 ===')
        self.finish_polygon()
        time.sleep(1)
        
        # 示例4: 切换到不同区域并定义另一个多边形
        self.get_logger().info('=== 示例4: 切换到不同区域并定义另一个多边形 ===')
        self.select_area(1)
        time.sleep(1)
        self.start_polygon_collection()
        time.sleep(1)
        self.click_point(1.0, 4.0)
        time.sleep(0.5)
        self.click_point(2.0, 4.0)
        time.sleep(0.5)
        self.click_point(2.0, 3.0)
        time.sleep(0.5)
        self.click_point(1.0, 3.0)
        time.sleep(1)
        self.finish_polygon()
        time.sleep(1)
        
        # 示例5: 启用自动重新加载（仅高级版本）
        self.get_logger().info('=== 示例5: 启用自动重新加载 ===')
        self.enable_auto_reload(True)
        time.sleep(1)
        
        # 示例6: 手动重新加载
        self.get_logger().info('=== 示例6: 手动重新加载 ===')
        self.reload_costmap()
        time.sleep(1)
        
        # 示例7: 重置所有区域
        self.get_logger().info('=== 示例7: 重置所有区域 ===')
        self.reset_all_areas()
        time.sleep(1)
        
        self.get_logger().info('示例运行完成！')

def main():
    rclpy.init()
    
    example = ConfigModifierExample()
    
    try:
        # 运行示例
        example.run_example()
        
        # 保持节点运行一段时间以查看结果
        time.sleep(5)
        
    except KeyboardInterrupt:
        pass
    finally:
        example.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 