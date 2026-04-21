#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('config_dynamic_modifier')
    
    # 声明launch参数
    config_file_path_arg = DeclareLaunchArgument(
        'config_file_path',
        default_value='/home/tc/Desktop/auto_sentry2025/src/auto_sentry2025/robot_bring_up/config/sentry.yaml',
        description='配置文件路径'
    )
    
    point_topic_arg = DeclareLaunchArgument(
        'point_topic',
        default_value='/clicked_point',
        description='RViz2点话题'
    )
    
    command_topic_arg = DeclareLaunchArgument(
        'command_topic',
        default_value='/config_modifier/command',
        description='命令话题'
    )
    
    status_topic_arg = DeclareLaunchArgument(
        'status_topic',
        default_value='/config_modifier/status',
        description='状态话题'
    )
    
    min_polygon_points_arg = DeclareLaunchArgument(
        'min_polygon_points',
        default_value='3',
        description='最小多边形点数'
    )
    
    # 创建节点
    config_modifier_node = Node(
        package='config_dynamic_modifier',
        executable='config_dynamic_modifier_node',
        name='config_dynamic_modifier_node',
        output='screen',
        parameters=[{
            'config_file_path': LaunchConfiguration('config_file_path'),
            'point_topic': LaunchConfiguration('point_topic'),
            'command_topic': LaunchConfiguration('command_topic'),
            'status_topic': LaunchConfiguration('status_topic'),
            'min_polygon_points': LaunchConfiguration('min_polygon_points'),
        }]
    )
    
    return LaunchDescription([
        config_file_path_arg,
        point_topic_arg,
        command_topic_arg,
        status_topic_arg,
        min_polygon_points_arg,
        config_modifier_node,
    ]) 